#!/usr/bin/env python3
"""Publish the frame tree for the airframe and its cameras.

MAVROS supplies map -> base_link once tf.send is on, which
`config/mavros_overrides.yaml` does. Everything below base_link is this node's
job, because MAVROS knows nothing about where the payloads sit.

The tree

    map                         local ENU, origin at the launch point
     └── base_link              FLU body frame, from MAVROS
          ├── gimbal_mount      static, where the gimbal bolts on
          │    └── gimbal_camera_link       turns with the gimbal
          │         └── gimbal_camera_optical_frame
          └── nadir_cam_link    static, pitched to look straight down
               └── nadir_camera_optical_frame

Two conventions meet here and they are easy to confuse.

  A camera *link* follows the body convention, x forward along the view axis,
  which is what the Gazebo model uses.
  A camera *optical* frame follows REP 103, x right, y down, z forward, which
  is what CameraInfo and every projection formula assume.

The fixed rotation between them is rpy (-90, 0, -90) degrees. Get it wrong and
detections land at plausible but incorrect positions, which is far harder to
notice than a crash.

The node also draws a simple airframe as markers on base_link, so the 3D view
shows an aircraft rather than a bare set of axes. Markers rather than a URDF,
because a marker needs no mesh files and no robot_description plumbing.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from mavros_msgs.msg import GimbalDeviceAttitudeStatus
    HAVE_GIMBAL_MSG = True
except ImportError:  # pragma: no cover - only when mavros_extras is absent
    HAVE_GIMBAL_MSG = False

LATCHED = QoSProfile(
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


# Body convention to REP 103 optical convention.
LINK_TO_OPTICAL = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)


class SceneTf(Node):
    def __init__(self) -> None:
        super().__init__("scene_tf")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gimbal_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("nadir_xyz", [0.10, 0.0, -0.06])
        # "vehicle" means the gimbal quaternion is relative to the airframe,
        # which is what PX4 reports unless the yaw lock flag is set. "earth"
        # means it is relative to map. Reading it wrong tilts every projection.
        self.declare_parameter("gimbal_reference", "vehicle")
        # MAVROS publishes the gimbal attitude with frame_id base_link_frd, so
        # the quaternion is in forward-right-down while every ROS frame here is
        # forward-left-up. Converting is a 180 degree turn about x, which for a
        # quaternion means negating y and z. Skip it and the camera appears to
        # roll upside down and yaw the wrong way, which still produces a
        # footprint, just the wrong one.
        self.declare_parameter("gimbal_is_frd", True)
        self.declare_parameter("gimbal_rate_hz", 30.0)
        self.declare_parameter("publish_markers", True)

        self.base = self.get_parameter("base_frame").value
        self.gimbal_reference = self.get_parameter("gimbal_reference").value
        self.gimbal_is_frd = bool(self.get_parameter("gimbal_is_frd").value)

        self.static_bc = StaticTransformBroadcaster(self)
        self.dyn_bc = TransformBroadcaster(self)
        self._publish_static()

        # Identity until the gimbal reports. A gimbal that never reports leaves
        # the camera pointing along the airframe, which is visibly wrong in the
        # 3D view rather than silently wrong in the numbers.
        self.gimbal_q = (0.0, 0.0, 0.0, 1.0)
        self.gimbal_seen = False

        if HAVE_GIMBAL_MSG:
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self._on_gimbal, 10)
        else:
            self.get_logger().warn(
                "mavros_msgs has no GimbalDeviceAttitudeStatus. The gimbal frame "
                "will stay fixed to the airframe.")

        rate = float(self.get_parameter("gimbal_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_gimbal)

        if self.get_parameter("publish_markers").value:
            self.marker_pub = self.create_publisher(
                MarkerArray, "/drone/markers", LATCHED)
            self.create_timer(1.0, self._publish_markers)

        self.create_timer(30.0, self._report)

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
        identity = (0.0, 0.0, 0.0, 1.0)

        self.static_bc.sendTransform([
            self._static(self.base, "gimbal_mount", mount, identity),
            self._static("gimbal_camera_link", "gimbal_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
            # Pitched a quarter turn so the link's x axis looks straight down,
            # matching the sensor pose in x500_recon/model.sdf.
            self._static(self.base, "nadir_cam_link", nadir,
                         quat_from_rpy(0.0, math.pi / 2, 0.0)),
            self._static("nadir_cam_link", "nadir_camera_optical_frame",
                         (0, 0, 0), LINK_TO_OPTICAL),
        ])

    # ----------------------------------------------------------------- gimbal
    def _on_gimbal(self, msg) -> None:
        q = msg.q
        self.gimbal_q = (q.x, -q.y, -q.z, q.w) if self.gimbal_is_frd \
            else (q.x, q.y, q.z, q.w)
        if not self.gimbal_seen:
            self.gimbal_seen = True
            # flags bit 32 is YAW_IN_VEHICLE_FRAME, 64 is YAW_IN_EARTH_FRAME.
            frame = "vehicle" if (getattr(msg, "flags", 0) & 32) else \
                    ("earth" if (getattr(msg, "flags", 0) & 64) else "unstated")
            self.get_logger().info(
                f"gimbal attitude received: flags={getattr(msg, 'flags', 0)} "
                f"({frame} yaw), treating input as "
                f"{'FRD' if self.gimbal_is_frd else 'FLU'}")
            if frame != "unstated" and frame != self.gimbal_reference:
                self.get_logger().warn(
                    f"gimbal reports {frame} yaw but gimbal_reference is "
                    f"'{self.gimbal_reference}'. Projections will be rotated.")

    def _publish_gimbal(self) -> None:
        parent = "gimbal_mount" if self.gimbal_reference == "vehicle" else "map"
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = "gimbal_camera_link"
        t.transform.rotation.x = self.gimbal_q[0]
        t.transform.rotation.y = self.gimbal_q[1]
        t.transform.rotation.z = self.gimbal_q[2]
        t.transform.rotation.w = self.gimbal_q[3]
        self.dyn_bc.sendTransform(t)

    # ---------------------------------------------------------------- markers
    def _publish_markers(self) -> None:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def add(idx, kind, xyz, scale, colour, frame=None, rpy=(0, 0, 0)):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame or self.base
            m.ns = "airframe"
            m.id = idx
            m.type = kind
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            q = quat_from_rpy(*rpy)
            m.pose.orientation.x, m.pose.orientation.y = q[0], q[1]
            m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color = ColorRGBA(r=colour[0], g=colour[1], b=colour[2], a=colour[3])
            arr.markers.append(m)

        body = (0.15, 0.15, 0.17, 1.0)
        add(0, Marker.CUBE, (0.0, 0.0, 0.0), (0.22, 0.16, 0.08), body)
        # Four arms and rotor discs, at the x500 geometry.
        for i, (ax, ay) in enumerate([(0.18, 0.18), (-0.18, 0.18),
                                      (-0.18, -0.18), (0.18, -0.18)]):
            add(10 + i, Marker.CUBE, (ax / 2, ay / 2, 0.0),
                (abs(ax) + 0.02, 0.02, 0.02), (0.1, 0.1, 0.1, 1.0),
                rpy=(0.0, 0.0, math.atan2(ay, ax)))
            # Front rotors red, rear ones dark, the usual orientation cue.
            colour = (0.85, 0.2, 0.2, 0.55) if ax > 0 else (0.2, 0.2, 0.25, 0.55)
            add(20 + i, Marker.CYLINDER, (ax, ay, 0.03), (0.26, 0.26, 0.01), colour)
        add(30, Marker.SPHERE, tuple(self.get_parameter("gimbal_mount_xyz").value),
            (0.07, 0.07, 0.07), (0.9, 0.75, 0.1, 1.0))
        self.marker_pub.publish(arr)

    def _report(self) -> None:
        if not self.gimbal_seen:
            self.get_logger().warn(
                "no gimbal attitude yet; gimbal_camera_link is aligned with the "
                "airframe. Is the gimbal_control plugin running?")


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
