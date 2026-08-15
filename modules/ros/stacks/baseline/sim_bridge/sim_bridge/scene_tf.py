#!/usr/bin/env python3
"""Publish the frame tree for the airframe and its cameras.

MAVROS supplies map -> base_link. Everything below base_link is this node's
job, because MAVROS knows nothing about where the payloads sit.

    map                         local ENU, origin at the launch point
     └── base_link              FLU body frame, from MAVROS
          ├── gimbal_mount      static, where the gimbal bolts on
          │    └── gimbal_camera_link       turns with the gimbal
          │         └── gimbal_camera_optical_frame
          └── nadir_camera_link static, pitched to look straight down
               └── nadir_camera_optical_frame

A camera *link* frame uses the body convention, x forward along the view
axis. A camera *optical* frame uses REP 103, x right, y down, z forward,
which CameraInfo and every projection formula assume. The fixed rotation
between them is rpy (-90, 0, -90) degrees.

The gimbal math here undoes three conventions in turn. docs/interfaces.md
section 6 records each one and the wrong answers they produced.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

try:
    from mavros_msgs.msg import GimbalDeviceAttitudeStatus
    HAVE_GIMBAL_MSG = True
except ImportError:  # pragma: no cover - only when mavros_extras is absent
    HAVE_GIMBAL_MSG = False

from sim_bridge.projection import (LINK_TO_OPTICAL, body_frd_to_flu,
                                   quat_conj, quat_from_rpy, quat_mul)

# ------------------------------------------------------------------- tunables
GIMBAL_PUBLISH_RATE_HZ = 30.0
# How long a commanded setpoint stays authoritative before the device report
# takes over, in gimbal_source "auto".
SETPOINT_TIMEOUT_S = 3.0

# GIMBAL_DEVICE_FLAGS_YAW_LOCK
YAW_LOCK = 16


def quat_yaw_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


def yaw_only(q):
    """The yaw part of a quaternion, as a rotation about z."""
    return quat_from_rpy(0.0, 0.0, math.radians(quat_yaw_deg(q)))


# Aerospace to ROS. NED_TO_ENU swaps the reference frame; FRD_TO_FLU swaps
# the body axes. Applied as NED_TO_ENU * q * FRD_TO_FLU, the same conversion
# MAVROS applies to every other attitude.
NED_TO_ENU = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)


def aerospace_to_ros(q):
    """An absolute attitude, NED reference and FRD body, into ENU and FLU."""
    return quat_mul(quat_mul(NED_TO_ENU, q), FRD_TO_FLU)


class SceneTf(Node):
    def __init__(self) -> None:
        super().__init__("scene_tf")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gimbal_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("nadir_xyz", [0.10, 0.0, -0.06])
        # The nadir mounting, in degrees, roll pitch yaw against base_link.
        # 0 90 0 matches the sensor pose in x500_recon/model.sdf: a quarter
        # turn about y points the camera's view axis at the ground. A small
        # error here shifts where every detection from this camera lands, so
        # it stays trimmable without a rebuild.
        self.declare_parameter("nadir_rpy_deg", [0.0, 90.0, 0.0])
        # Which frame the reported gimbal attitude is in. PX4's simulated
        # gimbal reports an absolute attitude while its flag claims vehicle
        # frame (GZGimbal.cpp), so "earth" matches what PX4 sends and this
        # node divides the vehicle attitude back out. Set "vehicle" for a
        # gimbal that reports honestly, or with px4-gzgimbal-frame.patch
        # applied.
        self.declare_parameter("gimbal_reference", "earth")
        # Where the gimbal orientation comes from.
        #   setpoint  GIMBAL_DEVICE_SET_ATTITUDE, the commanded angles,
        #             already vehicle relative.
        #   status    GIMBAL_DEVICE_ATTITUDE_STATUS, what the device reports.
        #   auto      prefer a fresh setpoint, fall back to the status. An
        #             untouched gimbal produces no setpoint at all, so "auto"
        #             avoids sitting at identity until the first command.
        self.declare_parameter("gimbal_source", "auto")
        # A fixed mounting rotation applied after the attitude is made body
        # relative, in degrees, as roll, pitch, yaw. The diagnostics log
        # prints the number to put here.
        self.declare_parameter("gimbal_offset_rpy_deg", [0.0, 0.0, 0.0])
        # Log vehicle heading against gimbal heading once a second. With the
        # gimbal centered, "gimbal rel body" should read near zero at every
        # aircraft heading. A constant is the gimbal_offset_rpy_deg value; a
        # value that moves with the aircraft means a frame handling bug.
        self.declare_parameter("log_gimbal_diagnostics", False)

        self.base = self.get_parameter("base_frame").value
        self.gimbal_reference = self.get_parameter("gimbal_reference").value
        self.gimbal_source = self.get_parameter("gimbal_source").value
        offset = [float(v) for v in self.get_parameter("gimbal_offset_rpy_deg").value]
        self.gimbal_offset = quat_from_rpy(*(math.radians(v) for v in offset))

        # Identity until the gimbal reports. A gimbal that never reports
        # leaves the camera pointing along the airframe, which is visibly
        # wrong in the 3D view rather than silently wrong in the numbers.
        self.q_status = (0.0, 0.0, 0.0, 1.0)
        self.q_setpoint = (0.0, 0.0, 0.0, 1.0)
        self.vehicle_q = (0.0, 0.0, 0.0, 1.0)
        self.gimbal_q = (0.0, 0.0, 0.0, 1.0)
        self.last_setpoint = None
        self.gimbal_seen = False
        self.logged_yaw_lock = False
        self.logged_setpoint = False

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)
        self._publish_static()

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_vehicle, qos_profile_sensor_data)
        if HAVE_GIMBAL_MSG:
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self._on_gimbal_status, 10)
            try:
                from mavros_msgs.msg import GimbalDeviceSetAttitude
                self.create_subscription(
                    GimbalDeviceSetAttitude,
                    "/mavros/gimbal_control/device/set_attitude",
                    self._on_gimbal_setpoint, 10)
            except ImportError:
                pass
        else:
            self.get_logger().warn(
                "mavros_msgs has no GimbalDeviceAttitudeStatus. The gimbal "
                "frame will stay fixed to the airframe.")

        self.create_timer(1.0 / GIMBAL_PUBLISH_RATE_HZ, self._publish_gimbal)
        self.create_timer(30.0, self._report)
        if self.get_parameter("log_gimbal_diagnostics").value:
            self.create_timer(1.0, self._log_diagnostics)

    # ------------------------------------------------------------------ static
    def _static(self, parent: str, child: str, xyz, q) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
        t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
        return t

    def _publish_static(self) -> None:
        mount = self.get_parameter("gimbal_mount_xyz").value
        nadir = self.get_parameter("nadir_xyz").value
        nadir_rotation = quat_from_rpy(*(math.radians(v) for v in
                                         self.get_parameter("nadir_rpy_deg").value))
        identity = (0.0, 0.0, 0.0, 1.0)

        self.static_broadcaster.sendTransform([
            self._static(self.base, "gimbal_mount", mount, identity),
            self._static("gimbal_camera_link", "gimbal_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
            self._static(self.base, "nadir_camera_link", nadir, nadir_rotation),
            self._static("nadir_camera_link", "nadir_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
        ])

    # ----------------------------------------------------------------- gimbal
    def _on_vehicle(self, msg) -> None:
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)

    def _on_gimbal_setpoint(self, msg) -> None:
        if self.gimbal_source == "status":
            return
        self.last_setpoint = self.get_clock().now().nanoseconds / 1e9

        # The setpoint is already vehicle relative on any axis whose LOCK
        # flag is clear, so it needs the body axis conversion and nothing
        # else.
        flags = int(getattr(msg, "flags", 0))
        body = body_frd_to_flu((msg.q.x, msg.q.y, msg.q.z, msg.q.w))
        if flags & YAW_LOCK:
            # Yaw is earth referenced on this axis. Take the vehicle heading
            # back out to get a rotation that belongs under gimbal_mount.
            body = quat_mul(quat_conj(yaw_only(self.vehicle_q)), body)
            if not self.logged_yaw_lock:
                self.logged_yaw_lock = True
                self.get_logger().info(
                    "gimbal yaw is earth locked, removing the vehicle heading")

        if not self.logged_setpoint:
            self.logged_setpoint = True
            self.gimbal_seen = True
            self.get_logger().info(
                f"gimbal orientation from the commanded setpoint, flags={flags}")

        self.q_setpoint = quat_mul(body, self.gimbal_offset)

    def _on_gimbal_status(self, msg) -> None:
        if not self.gimbal_seen:
            self.gimbal_seen = True
            self.get_logger().info(
                f"gimbal attitude received, flags={getattr(msg, 'flags', 0)}, "
                f"treated as {self.gimbal_reference} referenced")

        # MAVROS passes the PX4 quaternion through untouched, in aerospace
        # convention: NED reference, FRD body axes.
        absolute = aerospace_to_ros((msg.q.x, msg.q.y, msg.q.z, msg.q.w))
        if self.gimbal_reference == "earth":
            # The report is absolute, so divide the vehicle attitude off the
            # left: q_abs = q_vehicle * q_rel. Dividing on the right is a
            # conjugation, which looks perfect with the gimbal centered and
            # swaps axes once it moves. See docs/interfaces.md section 6.
            body = quat_mul(quat_conj(self.vehicle_q), absolute)
        else:
            body = absolute
        self.q_status = quat_mul(body, self.gimbal_offset)

    def _setpoint_fresh(self) -> bool:
        if self.last_setpoint is None:
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        return (now - self.last_setpoint) < SETPOINT_TIMEOUT_S

    def _publish_gimbal(self) -> None:
        if self.gimbal_source == "setpoint":
            self.gimbal_q = self.q_setpoint
        elif self.gimbal_source == "auto" and self._setpoint_fresh():
            self.gimbal_q = self.q_setpoint
        else:
            self.gimbal_q = self.q_status

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "gimbal_mount"
        t.child_frame_id = "gimbal_camera_link"
        t.transform.rotation.x = self.gimbal_q[0]
        t.transform.rotation.y = self.gimbal_q[1]
        t.transform.rotation.z = self.gimbal_q[2]
        t.transform.rotation.w = self.gimbal_q[3]
        self.dynamic_broadcaster.sendTransform(t)

    # ------------------------------------------------------------ diagnostics
    def _log_diagnostics(self) -> None:
        vehicle_yaw = quat_yaw_deg(self.vehicle_q)
        absolute_yaw = quat_yaw_deg(quat_mul(self.vehicle_q, self.gimbal_q))
        relative_yaw = quat_yaw_deg(self.gimbal_q)
        self.get_logger().info(
            f"yaw deg: vehicle {vehicle_yaw:+7.1f}  gimbal absolute "
            f"{absolute_yaw:+7.1f}  gimbal rel body {relative_yaw:+7.1f}  "
            f"source={'setpoint' if self._setpoint_fresh() else 'status'}")

    def _report(self) -> None:
        if not self.gimbal_seen:
            self.get_logger().warn(
                "no gimbal attitude yet; gimbal_camera_link is aligned with "
                "the airframe. Is the gimbal_control plugin running?")


def main() -> None:
    rclpy.init()
    node = SceneTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
