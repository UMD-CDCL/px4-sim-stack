"""Stabilize the simulated gimbal the way the MAVLink flags describe.

The tracker emulates the two stabilized behaviors of the gimbal
protocol for a device that ignores the frame flags:

    track    a region of interest: every axis holds on a world point,
             the DO_SET_ROI_LOCATION behavior.
    follow   the protocol's default lock flags: pitch holds against the
             horizon, roll seeks level in the world, and yaw follows
             the vehicle heading at the offset set when the follow
             started.

PX4 cannot do either for this device. Its v2 gimbal output computes an
ROI attitude once per command (output_mavlink.cpp, OutputMavlinkV2),
and the simulated gimbal applies whatever arrives as vehicle-relative
joint angles with the frame flags ignored. So this file recomputes the
vehicle-relative attitude as the vehicle moves, roll level.
docs/px4-simulated-gimbal.md records the device behaviors.

ClickToGimbal forwards each vehicle attitude message to on_vehicle,
and the tracker recomputes on that stream, gated to 20 Hz
(TICK_MIN_INTERVAL_S). The simulated device polls its setpoint at
50 Hz (GZGimbal.cpp with patches/px4-gzgimbal-rate.patch), so every
command lands on the joints within one tick and the 20 Hz cadence is
the pacing item. A click bypasses the gate: track and follow
recompute at once.

A recompute becomes a command only when it moves the desired attitude
by more than COMMAND_DEADBAND_DEG. The vehicle attitude stream carries
hover noise, and a device that chases it twitches on a point it
already holds. A click bypasses the band too: a deliberate small
correction always lands.

The whole emulation lives in this one file so it is easy to drop,
revert, or improve. ClickToGimbal owns one RoiTracker and gives it a
narrow surface: tf_buffer, reference and optical frame names,
cmd_q_body_link, command_body_attitude, its logger, and the vehicle
pose stream forwarded to on_vehicle. Delete this
file and the few lines that build and call it, and the stack is back
to one-shot commands.

The pointing math avoids the EKF heading trap. The tracker converts
world targets into the vehicle frame with a corrected vehicle attitude,
not with the raw EKF attitude, whose heading error measured 5 to 16
degrees in flight. The correction comes from the one pair of sources
that is world true: the full TF chain to the camera, and the joint
state in cmd_q_body_link. Those two are entangled: the chain fixes
only their product, so only one can be updated at a time, and each
update goes to the one that actually moved. A click first re-derives
the joint state from the chain under the standing correction, because
a disagreement there means another controller, usually the QGC
joystick, moved the joints, and the correction drifts only at EKF
speed. The correction itself refreshes on every tick, blended a tenth
at a time (CORRECTION_BLEND): the TF chain trails the joints by up to
one device cycle, and the blend keeps that transient under the command
deadband while real EKF drift still tracks with a time constant well
under a second, so the stabilization stays current through flight.
"""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped

from sim_bridge.projection import (LINK_TO_OPTICAL, body_frd_to_flu,
                                   pointing_rpy_body, quat_conj,
                                   quat_from_rpy, quat_mul, quat_rotate,
                                   ros_to_aerospace, rpy_from_quat, wrap_pi)

# ------------------------------------------------------------------- tunables
# The shortest interval between pose-driven recomputes, 20 Hz. The
# simulated device polls its setpoint at 50 Hz (GZGimbal.cpp with
# patches/px4-gzgimbal-rate.patch), so this cadence is the pacing item.
# A click bypasses the gate: track and follow recompute at once.
TICK_MIN_INTERVAL_S = 0.05
# How much of each measured correction folds in per tick. The measurement
# mis-attributes up to one command step to the vehicle while the TF chain
# trails the joints by a device cycle, so folding it in a tenth at a time
# keeps that transient under the deadband, while real EKF drift, degrees
# per minute, still tracks with a time constant well under a second. A
# click snaps the correction exactly instead: at click time the joints
# are settled, so the chain is current.
CORRECTION_BLEND = 0.1
# The smallest attitude change worth commanding. The vehicle attitude and
# the device report both carry noise; below this band a new setpoint only
# makes the joints chase that noise, and the camera twitches on a point it
# already holds. 0.5 degrees sits above the hover noise and shifts the
# view by under a meter at typical working distances. A click bypasses
# the band, so a deliberate small correction always lands.
COMMAND_DEADBAND_DEG = 0.5
# A click adopts the TF-implied joint state when it disagrees with the
# last command by more than this. Above TF lag and attitude noise, far
# below any deliberate joystick move.
EXTERNAL_MOVE_DEG = 3.0

IDENTITY = (0.0, 0.0, 0.0, 1.0)

# "No camera TF was passed" versus "the fetch was tried and failed" (None).
# Without the distinction, a failed fetch in _tick would make each callee
# retry the lookup on its own.
_UNSET = object()


def quat_angle_deg(a, b) -> float:
    dot = abs(sum(x * y for x, y in zip(a, b)))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def quat_nlerp(a, b, t: float):
    """Normalized lerp from a toward b by t. Enough for the small angles
    the correction moves by between ticks."""
    if sum(x * y for x, y in zip(a, b)) < 0.0:
        b = tuple(-v for v in b)
    mix = tuple(av + (bv - av) * t for av, bv in zip(a, b))
    norm = math.sqrt(sum(v * v for v in mix)) or 1.0
    return tuple(v / norm for v in mix)


class RoiTracker:
    def __init__(self, node) -> None:
        self.node = node
        self.point: tuple[float, float, float] | None = None
        # (world pitch, yaw offset from the vehicle heading), radians.
        self.follow_angles: tuple[float, float] | None = None
        self.vehicle_q: tuple[float, float, float, float] | None = None
        # The body yaw the EKF attitude is missing, refreshed against the
        # TF chain. Identity only until the first target arrives.
        self.correction = IDENTITY
        self.last_pose_tick = 0.0
        # True for the tick a click starts, so that tick commands even
        # inside the deadband.
        self.force_command = False

    def track(self, point_map: tuple[float, float, float]) -> None:
        """Hold every axis on a point in the reference frame.

        Called at click time, when the TF chain is current, so the joint
        state re-syncs and the correction refreshes unconditionally."""
        self.point, self.follow_angles = point_map, None
        camera = self._camera_tf()
        self._sync_joint_state(camera)
        self._refresh_correction(camera)
        self.force_command = True
        self._tick()

    def follow(self, pitch_world: float, yaw_world: float) -> None:
        """Hold pitch on the horizon and follow the vehicle heading,
        starting from the given world yaw. Roll seeks level."""
        if self.vehicle_q is None:
            self.node.get_logger().warn(
                "follow dropped: no vehicle attitude yet")
            return
        self.point = None
        camera = self._camera_tf()
        self._sync_joint_state(camera)
        self._refresh_correction(camera)
        heading = self._heading(self._vehicle_attitude())
        self.follow_angles = (pitch_world, wrap_pi(yaw_world - heading))
        self.force_command = True
        self._tick()

    def clear(self) -> None:
        self.point = self.follow_angles = None

    def on_vehicle(self, msg: PoseStamped) -> None:
        """Take one vehicle attitude message, forwarded by ClickToGimbal.
        The attitude is always stored; the recompute is gated to 20 Hz."""
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)
        now = time.monotonic()
        if now - self.last_pose_tick < TICK_MIN_INTERVAL_S:
            return
        self.last_pose_tick = now
        self._tick()

    # ---------------------------------------------------------------- internal
    def _camera_tf(self):
        try:
            # Latest available, no wait: a blocking lookup on the shared
            # executor starves the click and service callbacks when TF lags.
            tf = self.node.tf_buffer.lookup_transform(
                self.node.reference, self.node.optical, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - lookup raises several types
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _sync_joint_state(self, camera) -> None:
        """Adopt the joint state the TF chain implies when it disagrees
        with the last command.

        The chain fixes the product of the vehicle attitude and the
        joint state. A disagreement with cmd_q_body_link therefore means
        the joints moved under a command this node never saw, usually
        the QGC joystick, because the other factor, the correction,
        drifts only at EKF speed. Adopting the TF answer keeps the
        correction valid and puts the first click from any gimbal pose
        on target, instead of offset by the external move until the
        settle refresh caught up."""
        if camera is None or self.vehicle_q is None:
            return
        _, q_map_optical = camera
        q_link = quat_mul(
            quat_conj(self._vehicle_attitude()),
            quat_mul(q_map_optical, quat_conj(LINK_TO_OPTICAL)))
        if quat_angle_deg(q_link, self.node.cmd_q_body_link) \
                > EXTERNAL_MOVE_DEG:
            self.node.cmd_q_body_link = q_link
            self.node.get_logger().info(
                "gimbal was moved by another controller; joint state "
                "re-synced from TF")

    def _refresh_correction(self, camera=_UNSET, blend: float = 1.0) -> None:
        if camera is _UNSET:
            camera = self._camera_tf()
        if camera is None or self.vehicle_q is None:
            return
        _, q_map_optical = camera
        q_vehicle_true = quat_mul(
            q_map_optical,
            quat_conj(quat_mul(self.node.cmd_q_body_link, LINK_TO_OPTICAL)))
        measured = quat_mul(quat_conj(self.vehicle_q), q_vehicle_true)
        self.correction = (measured if blend >= 1.0
                           else quat_nlerp(self.correction, measured, blend))

    def _vehicle_attitude(self):
        """The corrected vehicle attitude, map ENU reference, FLU body."""
        return quat_mul(self.vehicle_q, self.correction)

    @staticmethod
    def _heading(q_vehicle_enu) -> float:
        """The vehicle compass heading in radians, aerospace convention."""
        return rpy_from_quat(ros_to_aerospace(q_vehicle_enu))[2]

    def _point_attitude(self, q_vehicle_enu, camera=_UNSET):
        """The vehicle-relative attitude that puts the axis on the point."""
        if camera is _UNSET:
            camera = self._camera_tf()
        if camera is None:
            return None
        position, _ = camera
        to_target = tuple(t - p for t, p in zip(self.point, position))
        norm = math.sqrt(sum(v * v for v in to_target))
        if norm < 1.0:
            return None    # directly on top of the point: no useful direction
        return pointing_rpy_body(quat_rotate(
            quat_conj(q_vehicle_enu), tuple(v / norm for v in to_target)))

    def _follow_attitude(self, q_vehicle_enu):
        """The vehicle-relative attitude that holds the world pitch, levels
        the roll, and keeps the yaw offset from the heading."""
        q_vehicle = ros_to_aerospace(q_vehicle_enu)
        pitch_world, yaw_offset = self.follow_angles
        heading = rpy_from_quat(q_vehicle)[2]
        desired = quat_from_rpy(
            0.0, pitch_world, wrap_pi(heading + yaw_offset))
        return rpy_from_quat(quat_mul(quat_conj(q_vehicle), desired))

    def _tick(self) -> None:
        if (self.point is None and self.follow_angles is None) \
                or self.vehicle_q is None:
            return
        # One TF fetch serves the refresh and the point attitude. The
        # 50 Hz device report keeps the chain within one cycle of the
        # joints, so the correction refreshes on every tick, blended,
        # and the stabilization stays current through flight instead of
        # waiting for a still moment that flight never offers.
        camera = self._camera_tf()
        self._refresh_correction(camera, blend=CORRECTION_BLEND)
        q_vehicle_enu = self._vehicle_attitude()
        attitude = (self._point_attitude(q_vehicle_enu, camera)
                    if self.point is not None
                    else self._follow_attitude(q_vehicle_enu))
        if attitude is None:
            return

        desired_link = body_frd_to_flu(quat_from_rpy(*attitude))
        # The joints hold the last setpoint exactly, so inside the
        # deadband a new command only chases noise.
        force, self.force_command = self.force_command, False
        if not force and quat_angle_deg(
                desired_link, self.node.cmd_q_body_link) < COMMAND_DEADBAND_DEG:
            return
        self.node.command_body_attitude(*attitude)
