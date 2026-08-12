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
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
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


def quat_mul(a, b):
    """Hamilton product, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


# Aerospace to ROS. NED_TO_ENU is a half turn about the (1, 1, 0) diagonal and
# swaps the reference frame; FRD_TO_FLU is a half turn about x and swaps the
# body axes. Applied as NED_TO_ENU * q * FRD_TO_FLU, which is what MAVROS does
# for every other attitude it converts.
NED_TO_ENU = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)


def aerospace_to_ros(q):
    return quat_mul(quat_mul(NED_TO_ENU, q), FRD_TO_FLU)


def quat_yaw_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y),
                                   1.0 - 2.0 * (y * y + z * z)))


# Body convention to REP 103 optical convention.
LINK_TO_OPTICAL = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)


class SceneTf(Node):
    def __init__(self) -> None:
        super().__init__("scene_tf")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gimbal_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("nadir_xyz", [0.10, 0.0, -0.06])
        # Which frame the reported gimbal quaternion is actually in.
        #
        # PX4's simulated gimbal lies about this. In
        # src/modules/simulation/gz_bridge/GZGimbal.cpp, gimbalIMUCallback()
        # builds the quaternion from the gimbal's IMU, and a Gazebo IMU reports
        # orientation relative to the world, not relative to the vehicle. The
        # value is therefore absolute. publishDeviceAttitude() then hardcodes
        #
        #     device_flags = DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME
        #
        # so the message claims to be vehicle-relative while carrying an
        # earth-relative attitude.
        #
        # Obey the flag and the vehicle attitude gets applied twice: yaw the
        # airframe by 90 degrees and the camera appears to swing 180. It looks
        # like a gimbal that over-rotates, and it is why targets land in the
        # wrong place at any heading other than zero. It affects roll and pitch
        # as well, and only yaw is obvious because aircraft fly near level.
        #
        # "earth" is therefore the default and matches what PX4 sends. This
        # node divides the vehicle attitude back out, so the transform it
        # publishes under gimbal_mount is genuinely vehicle-relative.
        # Set "vehicle" if you run a gimbal that reports honestly, or if you
        # applied patches/px4-gzgimbal-frame.patch to PX4.
        self.declare_parameter("gimbal_reference", "earth")
        # MAVROS republishes the gimbal attitude untouched, with frame_id
        # base_link_frd. That value is aerospace convention on both ends: the
        # body axes are forward-right-down and the reference is NED. Every ROS
        # frame here is forward-left-up against ENU.
        #
        # Both ends change, so this is not a single 180 degree turn. Negating y
        # and z converts the body axes alone and leaves the reference wrong,
        # which measures as the camera turning about twice as far as the
        # aircraft and in the opposite direction. The conversion needs a
        # rotation on each side, which is what NED_TO_ENU and FRD_TO_FLU do.
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
        self.gimbal_from_setpoint = False
        self.last_status = None
        # The vehicle attitude in map, needed to divide out of an earth
        # referenced gimbal report.
        self.vehicle_q = (0.0, 0.0, 0.0, 1.0)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_vehicle, qos_profile_sensor_data)

        if HAVE_GIMBAL_MSG:
            self.create_subscription(
                GimbalDeviceAttitudeStatus,
                "/mavros/gimbal_control/device/attitude_status",
                self._on_gimbal, 10)
            # The commanded attitude, used only when the device reports no
            # status of its own. A setpoint is where the gimbal was told to go
            # rather than where it is, so it lags and it never shows a stall,
            # but it beats leaving the camera pointing along the airframe.
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
    def _on_vehicle(self, msg) -> None:
        q = msg.pose.orientation
        self.vehicle_q = (q.x, q.y, q.z, q.w)

    def _on_gimbal_setpoint(self, msg) -> None:
        # Only while the device itself is silent.
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_status is not None and now - self.last_status < 2.0:
            return
        if not self.gimbal_from_setpoint:
            self.gimbal_from_setpoint = True
            self.get_logger().warn(
                "no gimbal device status, falling back to the commanded "
                "attitude. That is where the gimbal was told to go, not where "
                "it is.")
        self._store(msg.q)

    def _store(self, q) -> None:
        raw = (q.x, q.y, q.z, q.w)
        if self.gimbal_is_frd:
            raw = aerospace_to_ros(raw)
        if self.gimbal_reference == "earth":
            # The report is absolute, so remove the vehicle attitude to get the
            # rotation that belongs under gimbal_mount.
            self.gimbal_q = quat_mul(quat_conj(self.vehicle_q), raw)
        else:
            self.gimbal_q = raw

    def _on_gimbal(self, msg) -> None:
        self.last_status = self.get_clock().now().nanoseconds / 1e9
        self.gimbal_from_setpoint = False
        self._store(msg.q)
        if not self.gimbal_seen:
            self.gimbal_seen = True
            # flags bit 32 is YAW_IN_VEHICLE_FRAME, 64 is YAW_IN_EARTH_FRAME.
            frame = "vehicle" if (getattr(msg, "flags", 0) & 32) else \
                    ("earth" if (getattr(msg, "flags", 0) & 64) else "unstated")
            self.get_logger().info(
                f"gimbal attitude received: flags={getattr(msg, 'flags', 0)} "
                f"({frame} yaw claimed), treating input as "
                f"{'FRD' if self.gimbal_is_frd else 'FLU'} and as "
                f"{self.gimbal_reference} referenced")
            if frame == "vehicle" and self.gimbal_reference == "earth":
                self.get_logger().info(
                    "The flag says vehicle frame and this node is ignoring it. "
                    "PX4's simulated gimbal sets that flag while reporting an "
                    "absolute attitude. See the note in scene_tf.py.")

    def _publish_gimbal(self) -> None:
        # Always under gimbal_mount. An earth referenced report has already had
        # the vehicle attitude divided out in _store, so by this point the
        # rotation is vehicle relative either way.
        parent = "gimbal_mount"
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
