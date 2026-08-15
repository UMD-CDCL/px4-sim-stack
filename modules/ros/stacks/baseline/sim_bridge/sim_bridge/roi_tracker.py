"""Hold the simulated gimbal on a world-fixed point.

This file emulates what PX4 would do with a region of interest if the
simulated gimbal obeyed the world frame flags: recompute the pointing to
the point as the vehicle moves, roll level, and update the gimbal at a
rate close to PX4's own gimbal loop. PX4 cannot do it itself for this
device. Its v2 gimbal output computes the ROI attitude once per command
(output_mavlink.cpp, OutputMavlinkV2), and the simulated gimbal applies
whatever arrives as vehicle-relative joint angles with the frame flags
ignored. docs/px4-simulated-gimbal.md records both behaviors.

The whole emulation lives in this one file so it is easy to drop,
revert, or improve. ClickToGimbal owns one RoiTracker and gives it a
narrow surface: tf_buffer, reference and optical frame names,
cmd_q_body_link, last_command_time, and command_body_direction. Delete
this file and the two lines that build and call the tracker, and the
stack is back to click-to-point only.

The pointing math avoids the EKF heading trap. The tracker converts the
world direction to the ROI into the vehicle frame with a corrected
vehicle attitude, not with the raw EKF attitude, whose heading error
measured 5 to 16 degrees in flight. The correction comes from the one
pair of sources that is world true: the full TF chain to the camera,
and the joint state the node last commanded. It refreshes whenever the
gimbal has been still long enough for the TF chain to catch up, and
between refreshes the EKF supplies only short-term attitude changes,
which stay accurate while its absolute heading drifts.

Ticks command the gimbal only when the pointing error passes a
deadband, so a steady hover produces almost no traffic.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

from sim_bridge.projection import (LINK_TO_OPTICAL, quat_conj, quat_mul,
                                   quat_rotate)

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
        self.target: tuple[float, float, float] | None = None
        self.vehicle_q: tuple[float, float, float, float] | None = None
        # The body yaw the EKF attitude is missing, refreshed against the
        # TF chain. Identity only until the first target arrives.
        self.correction = IDENTITY
        node.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_vehicle, qos_profile_sensor_data)
        node.create_timer(1.0 / TICK_HZ, self._tick)

    def track(self, point_map: tuple[float, float, float]) -> None:
        """Start holding the camera on a point in the reference frame.

        Called at click time, when the gimbal is settled and the TF chain
        is current, so the correction refreshes unconditionally."""
        self.target = point_map
        self._refresh_correction()
        self._tick()

    def clear(self) -> None:
        self.target = None

    # ---------------------------------------------------------------- internal
    def _on_vehicle(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)

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
        if camera is None or self.vehicle_q is None:
            return
        _, q_map_optical = camera
        q_vehicle_true = quat_mul(
            q_map_optical,
            quat_conj(quat_mul(self.node.cmd_q_body_link, LINK_TO_OPTICAL)))
        self.correction = quat_mul(quat_conj(self.vehicle_q), q_vehicle_true)

    def _seconds_since_command(self) -> float:
        now = self.node.get_clock().now().nanoseconds / 1e9
        return now - self.node.last_command_time

    def _tick(self) -> None:
        if self.target is None or self.vehicle_q is None:
            return
        if self._seconds_since_command() > CORRECTION_SETTLE_S:
            self._refresh_correction()
        camera = self._camera_tf()
        if camera is None:
            return
        position, _ = camera

        vehicle = quat_mul(self.vehicle_q, self.correction)
        to_target = tuple(t - p for t, p in zip(self.target, position))
        norm = math.sqrt(sum(v * v for v in to_target))
        if norm < 1.0:
            return    # directly on top of the point: no useful direction
        d_body = quat_rotate(quat_conj(vehicle),
                             tuple(v / norm for v in to_target))

        axis = quat_rotate(
            quat_mul(self.node.cmd_q_body_link, LINK_TO_OPTICAL),
            (0.0, 0.0, 1.0))
        error = math.degrees(math.acos(max(-1.0, min(1.0, sum(
            a * b for a, b in zip(axis, d_body))))))
        if error < DEADBAND_DEG:
            return
        self.node.command_body_direction(d_body)
