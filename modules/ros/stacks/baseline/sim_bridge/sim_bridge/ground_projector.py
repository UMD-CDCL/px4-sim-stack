#!/usr/bin/env python3
"""Draw where a camera is looking on the ground.

Casts a ray through each image corner, intersects the ground plane, and
publishes the covered ground as a stock PolygonStamped that the Foxglove 3D
panel draws without custom code. One node runs for each camera.

The plane height comes from the drone by default: MAVROS reports altitude
above the launch point, so the ground is at pose.z minus rel_alt. Set
use_rel_alt false to pin the plane to ground_z instead. This is a flat-earth
assumption, and over a slope the footprint is wrong in the way you expect.

Publishes
    <camera_ns>/footprint   geometry_msgs/PolygonStamped
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

from sim_bridge.projection import footprint_on_ground, image_corners, intrinsics_ready

# ------------------------------------------------------------------- tunables
PUBLISH_RATE_HZ = 5.0
# Beyond this slant range a corner ray is treated as missing the ground.
MAX_RANGE = 2000.0


class GroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("ground_projector")

        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.ground_z = float(self.get_parameter("ground_z").value)

        self.tf_buffer = Buffer()
        # spin_thread=True is required. On this node's executor, a lookup that
        # waits for a transform would block the callback that delivers it.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        self.rel_alt: float | None = None

        # Sensor QoS: CameraInfo and the MAVROS topics are best effort, and a
        # reliable subscription to a best effort publisher receives nothing.
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        if self.get_parameter("use_rel_alt").value:
            self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)

        self.footprint_pub = self.create_publisher(PolygonStamped, "footprint", 1)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_footprint)

        self.published = self.missed = 0
        self.create_timer(30.0, self._report)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    def _publish_footprint(self) -> None:
        if not intrinsics_ready(self.info):
            return
        try:
            # Latest available rather than a specific time: this draws the
            # current view, and asking for "now" races the transform.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            self.missed += 1
            return

        t = tf.transform.translation
        r = tf.transform.rotation
        ground_z = self.ground_z if self.rel_alt is None else t.z - self.rel_alt

        corners = footprint_on_ground(
            image_corners(self.info.width, self.info.height),
            self.info.k, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w),
            ground_z, MAX_RANGE)
        if corners is None:
            # Looking at or above the horizon. Publishing nothing is the
            # honest answer, because a clipped polygon would look like real
            # coverage.
            self.missed += 1
            return

        footprint = PolygonStamped()
        footprint.header.stamp = tf.header.stamp
        footprint.header.frame_id = self.reference
        footprint.polygon.points = [
            Point32(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in corners
        ]
        self.footprint_pub.publish(footprint)
        self.published += 1

    def _report(self) -> None:
        if self.published == 0 and self.missed > 0:
            self.get_logger().warn(
                f"no footprint in the last 30 s ({self.missed} attempts). Either "
                f"the transform {self.reference} -> {self.optical} is missing, "
                f"or the camera is not pointed at the ground.")
        self.published = self.missed = 0


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
