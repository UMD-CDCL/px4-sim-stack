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
vehicle-relative attitude as the vehicle moves, roll level, at a rate
close to PX4's own gimbal loop. docs/px4-simulated-gimbal.md records
the device behaviors.

The whole emulation lives in this one file so it is easy to drop,
revert, or improve. ClickToGimbal owns one RoiTracker and gives it a
narrow surface: tf_buffer, reference and optical frame names,
vehicle_q, cmd_q_body_link, last_command_time, and
command_body_attitude. Delete this file and the few lines that build
and call it, and the stack is back to one-shot commands.

The pointing math avoids the EKF heading trap. The tracker converts
world targets into the vehicle frame with a corrected vehicle attitude,
not with the raw EKF attitude, whose heading error measured 5 to 16
degrees in flight. The correction comes from the one pair of sources
that is world true: the full TF chain to the camera, and the joint
state the node last commanded. It refreshes whenever the gimbal has
been still long enough for the TF chain to catch up, and between
refreshes the EKF supplies only short-term attitude changes, which
stay accurate while its absolute heading drifts.

Ticks command the gimbal only when the pointing error passes a
deadband, so a steady hover produces almost no traffic.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration

from sim_bridge.projection import (LINK_TO_OPTICAL, body_frd_to_flu,
                                   pointing_rpy_body, quat_conj,
                                   quat_from_rpy, quat_mul, quat_rotate,
                                   ros_to_aerospace, rpy_from_quat, wrap_pi)

# ------------------------------------------------------------------- tunables
# Close to PX4's own gimbal update pace, and light on the setpoint topic.
TICK_HZ = 5.0
# How long after a gimbal command the TF chain is trusted again for the
# attitude correction. Covers the joint slew plus one device report.
CORRECTION_SETTLE_S = 1.5
# Pointing errors below this are left alone.
DEADBAND_DEG = 0.3

IDENTITY = (0.0, 0.0, 0.0, 1.0)


class RoiTracker:
    def __init__(self, node) -> None:
        self.node = node
        self.point: tuple[float, float, float] | None = None
        # (world pitch, yaw offset from the vehicle heading), radians.
        self.follow_angles: tuple[float, float] | None = None
        # The body yaw the EKF attitude is missing, refreshed against the
        # TF chain. Identity only until the first target arrives.
        self.correction = IDENTITY
        node.create_timer(1.0 / TICK_HZ, self._tick)

    def track(self, point_map: tuple[float, float, float]) -> None:
        """Hold every axis on a point in the reference frame.

        Called at click time, when the gimbal is settled and the TF chain
        is current, so the correction refreshes unconditionally."""
        self.point, self.follow_angles = point_map, None
        self._refresh_correction()
        self._tick()

    def follow(self, pitch_world: float, yaw_world: float) -> None:
        """Hold pitch on the horizon and follow the vehicle heading,
        starting from the given world yaw. Roll seeks level."""
        if self.node.vehicle_q is None:
            self.node.get_logger().warn(
                "follow dropped: no vehicle attitude yet")
            return
        self.point = None
        self._refresh_correction()
        heading = self._heading(self._vehicle_attitude())
        self.follow_angles = (pitch_world, wrap_pi(yaw_world - heading))
        self._tick()

    def clear(self) -> None:
        self.point = self.follow_angles = None

    # ---------------------------------------------------------------- internal
    def _camera_tf(self):
        try:
            tf = self.node.tf_buffer.lookup_transform(
                self.node.reference, self.node.optical, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:  # noqa: BLE001 - lookup raises several types
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _refresh_correction(self) -> None:
        camera = self._camera_tf()
        if camera is None or self.node.vehicle_q is None:
            return
        _, q_map_optical = camera
        q_vehicle_true = quat_mul(
            q_map_optical,
            quat_conj(quat_mul(self.node.cmd_q_body_link, LINK_TO_OPTICAL)))
        self.correction = quat_mul(quat_conj(self.node.vehicle_q),
                                   q_vehicle_true)

    def _vehicle_attitude(self):
        """The corrected vehicle attitude, map ENU reference, FLU body."""
        return quat_mul(self.node.vehicle_q, self.correction)

    @staticmethod
    def _heading(q_vehicle_enu) -> float:
        """The vehicle compass heading in radians, aerospace convention."""
        return rpy_from_quat(ros_to_aerospace(q_vehicle_enu))[2]

    def _seconds_since_command(self) -> float:
        now = self.node.get_clock().now().nanoseconds / 1e9
        return now - self.node.last_command_time

    def _point_attitude(self, q_vehicle_enu):
        """The vehicle-relative attitude that puts the axis on the point."""
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
        desired = quat_from_rpy(
            0.0, pitch_world, wrap_pi(self._heading(q_vehicle_enu) + yaw_offset))
        return rpy_from_quat(quat_mul(quat_conj(q_vehicle), desired))

    def _tick(self) -> None:
        if (self.point is None and self.follow_angles is None) \
                or self.node.vehicle_q is None:
            return
        if self._seconds_since_command() > CORRECTION_SETTLE_S:
            self._refresh_correction()
        q_vehicle_enu = self._vehicle_attitude()
        attitude = (self._point_attitude(q_vehicle_enu)
                    if self.point is not None
                    else self._follow_attitude(q_vehicle_enu))
        if attitude is None:
            return

        desired_link = body_frd_to_flu(quat_from_rpy(*attitude))
        dot = abs(sum(a * b for a, b in
                      zip(desired_link, self.node.cmd_q_body_link)))
        error = math.degrees(2.0 * math.acos(min(1.0, dot)))
        if error < DEADBAND_DEG:
            return
        self.node.command_body_attitude(*attitude)
