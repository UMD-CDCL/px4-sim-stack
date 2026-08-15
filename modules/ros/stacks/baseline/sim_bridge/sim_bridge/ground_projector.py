#!/usr/bin/env python3
"""Draw where a camera is looking on the ground.

Casts rays through points along the image boundary, intersects the ground
plane, and publishes the covered ground as a stock PolygonStamped that the
Foxglove 3D panel draws without custom code. One node runs for each camera.

The footprint is truncated at GROUND_VIEW_MAX_DISTANCE_M from the camera. A
camera near the horizon still covers ground close to the drone, so its
footprint is the near region it sees, closed by an arc at the limit,
instead of nothing. Only a camera that sees no ground at all publishes
nothing. The projected imagery stops at the same limit, so the picture
fills the outline that frames it.

The plane height comes from the drone by default: MAVROS reports altitude
above the launch point, so the ground is at pose.z minus rel_alt. Set
use_rel_alt false to pin the plane to ground_z instead. This is a flat-earth
assumption, and over a slope the footprint is wrong in the way you expect.

The Map panel gets the same outline as one GeoJSON polygon. The Foxglove
layout colors it, with the same color it gives the 3D panel line. A camera
that sees no ground publishes an empty collection, which clears its outline
from the map.

Publishes
    <camera_ns>/footprint           geometry_msgs/PolygonStamped
    <camera_ns>/footprint_geojson   foxglove_msgs/GeoJSON, for the Map panel
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

from sim_bridge.geo import MapOrigin
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   footprint_on_ground, image_boundary,
                                   intrinsics_ready)

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

# ------------------------------------------------------------------- tunables
PUBLISH_RATE_HZ = 5.0
# Boundary rays per image edge. More makes the truncation arc smoother.
BOUNDARY_SAMPLES_PER_EDGE = 8


class GroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("ground_projector")

        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)

        self.camera = self.get_parameter("camera").value
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
        self.geojson_pub = None
        if HAVE_FOXGLOVE:
            self.geojson_pub = self.create_publisher(
                GeoJSON, "footprint_geojson", 1)
        else:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no "
                "footprint. Install ros-$ROS_DISTRO-foxglove-msgs.")
        self.origin = MapOrigin(self)
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

        outline = footprint_on_ground(
            image_boundary(self.info.width, self.info.height,
                           BOUNDARY_SAMPLES_PER_EDGE),
            self.info.k, (t.x, t.y, t.z), (r.x, r.y, r.z, r.w),
            ground_z, GROUND_VIEW_MAX_DISTANCE_M)
        if outline is None:
            # No ground within the limit is in view. Publishing nothing is
            # the honest answer, and the map outline is cleared.
            self.missed += 1
            self._publish_geojson([])
            return

        footprint = PolygonStamped()
        footprint.header.stamp = tf.header.stamp
        footprint.header.frame_id = self.reference
        footprint.polygon.points = [
            Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in outline
        ]
        self.footprint_pub.publish(footprint)
        self._publish_geojson(outline)
        self.published += 1

    def _publish_geojson(self, outline) -> None:
        """Publish the outline as one GeoJSON polygon for the Map panel.
        The Foxglove layout gives it this camera's color. An empty outline
        publishes an empty collection, which clears the map."""
        if self.geojson_pub is None or not self.origin.ready:
            return
        features = []
        if outline:
            ring = self.origin.geojson_ring([(p[0], p[1]) for p in outline])
            features.append({
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                "properties": {"name": f"{self.camera} footprint"},
            })
        msg = GeoJSON()
        msg.geojson = json.dumps(
            {"type": "FeatureCollection", "features": features})
        self.geojson_pub.publish(msg)

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
