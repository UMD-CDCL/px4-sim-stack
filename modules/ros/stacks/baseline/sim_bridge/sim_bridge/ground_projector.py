#!/usr/bin/env python3
"""Draw where a camera is looking on the ground.

Takes the camera's CameraInfo and its pose from TF, casts a ray through each
image corner, and intersects those rays with a horizontal plane. The result is
the footprint the camera covers, published as a stock PolygonStamped that the
Foxglove 3D panel draws without any custom code.

Where the ground is
-------------------
The plane sits at a fixed height in the reference frame. By default that height
comes from the drone: MAVROS reports altitude above the launch point on
`/mavros/global_position/rel_alt`, and the local pose reports z in the map
frame, so the ground is at `pose.z - rel_alt`. Those two agree when the EKF is
healthy, and subtracting them cancels the drift that otherwise accumulates
between them. Set `use_rel_alt` false to pin the plane to `ground_z` instead.

This is a flat-earth assumption. Over a slope or a building the footprint is
wrong in exactly the way you would expect, and so is any detection localized
against it.

Publishes
    <camera_ns>/footprint     geometry_msgs/PolygonStamped, the covered ground
    <camera_ns>/boresight     geometry_msgs/PointStamped, where the centre looks
    /ground/plane             visualization_msgs/Marker, a reference grid patch
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA, Float64
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

from sim_bridge.projection import footprint_on_ground, image_corners, ray_in_optical, \
    intersect_ground, quat_rotate

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class GroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("ground_projector")

        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("max_range", 2000.0)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("draw_ground_grid", True)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.max_range = float(self.get_parameter("max_range").value)

        self.tf_buffer = Buffer()
        # spin_thread=True is not optional here. The listener otherwise
        # shares this node's executor, so a lookup that waits for a
        # transform blocks the very callback that would deliver it, and
        # every timeout expires. Its own thread keeps the buffer filling
        # while a lookup waits.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        self.rel_alt: float | None = None
        self.ground_z = float(self.get_parameter("ground_z").value)

        # Sensor QoS, not the default. CameraInfo and every MAVROS sensor topic
        # are published best effort, and a reliable subscription to a best
        # effort publisher is simply incompatible: it connects, reports the
        # mismatch once, and then receives nothing at all.
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        if self.get_parameter("use_rel_alt").value:
            self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)

        self.footprint_pub = self.create_publisher(PolygonStamped, "footprint", 1)
        self.boresight_pub = self.create_publisher(PointStamped, "boresight", 1)
        self.grid_pub = self.create_publisher(Marker, "/ground/plane", LATCHED)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)
        if self.get_parameter("draw_ground_grid").value:
            self.create_timer(2.0, self._publish_grid)

        self.ok = self.miss = 0
        self.create_timer(30.0, self._report)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    def _current_ground_z(self, drone_z: float) -> float:
        if self.rel_alt is None:
            return self.ground_z
        return drone_z - self.rel_alt

    def _tick(self) -> None:
        # CameraInfo.k arrives as a numpy array, so a plain truth test on it
        # raises rather than returning False. Check the length and fx.
        if self.info is None or len(self.info.k) != 9 or self.info.k[0] == 0.0:
            return
        try:
            # Latest available rather than a specific time: this draws the
            # current view, and asking for "now" races the transform.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            self.miss += 1
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rot = (r.x, r.y, r.z, r.w)
        ground_z = self._current_ground_z(t.z)

        corners = footprint_on_ground(
            image_corners(self.info.width, self.info.height),
            self.info.k, origin, rot, ground_z, self.max_range)
        if corners is None:
            # Looking at or above the horizon. Publishing nothing is the honest
            # answer; a clipped polygon would look like real coverage.
            self.miss += 1
            return

        poly = PolygonStamped()
        poly.header.stamp = tf.header.stamp
        poly.header.frame_id = self.reference
        poly.polygon.points = [
            Point32(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in corners
        ]
        self.footprint_pub.publish(poly)

        centre = ray_in_optical(self.info.width / 2.0, self.info.height / 2.0,
                                self.info.k)
        hit = intersect_ground(origin, quat_rotate(rot, centre), ground_z,
                               self.max_range)
        if hit is not None:
            p = PointStamped()
            p.header = poly.header
            p.point.x, p.point.y, p.point.z = float(hit[0]), float(hit[1]), float(hit[2])
            self.boresight_pub.publish(p)
        self.ok += 1

    def _publish_grid(self) -> None:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.reference
        m.ns = "ground"
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.z = self.ground_z if self.rel_alt is None else self.ground_z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = 400.0, 400.0, 0.02
        m.color = ColorRGBA(r=0.25, g=0.32, b=0.22, a=0.25)
        self.grid_pub.publish(m)

    def _report(self) -> None:
        if self.ok == 0 and self.miss > 0:
            self.get_logger().warn(
                f"no footprint in the last 30 s ({self.miss} attempts). Either "
                f"the transform {self.reference} -> {self.optical} is missing, "
                f"or the camera is not pointed at the ground.")
        self.ok = self.miss = 0


def main() -> None:
    rclpy.init()
    node = GroundProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
