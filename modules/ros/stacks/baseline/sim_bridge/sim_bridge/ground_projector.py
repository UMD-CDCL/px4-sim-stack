#!/usr/bin/env python3
"""Draw where a camera is looking on the ground. One node per camera.

Boundary rays through the image meet the surface sim_bridge/localization.py
selects, truncated at GROUND_VIEW_MAX_DISTANCE_M so a camera near the
horizon reports the near ground it sees, closed by an arc, instead of
nothing. The projected imagery stops at the same limit, so the picture
fills the outline that frames it.

Publishes
    <camera_ns>/footprint           geometry_msgs/PolygonStamped
    <camera_ns>/footprint_geojson   foxglove_msgs/GeoJSON, for the Map panel,
                                    empty when no ground is in view
"""

from __future__ import annotations

from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from sim_bridge.frames import CameraFrame
from sim_bridge.geo import MapOrigin
from sim_bridge.localization import GroundLocalizer
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   image_boundary, intrinsics_ready)
from sim_bridge.runtime import geojson_publisher, publish_features, spin

# ------------------------------------------------------------------- tunables
PUBLISH_RATE_HZ = 5.0
# Boundary rays per image edge. More makes the truncation arc smoother.
BOUNDARY_SAMPLES_PER_EDGE = 8
TF_TIMEOUT_S = 0.2


class GroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("ground_projector")

        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")

        self.camera = self.get_parameter("camera").value
        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.localizer = GroundLocalizer(self)
        self.camera_frame = CameraFrame(self, self.optical, self.reference)

        self.info: CameraInfo | None = None
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)

        self.footprint_pub = self.create_publisher(PolygonStamped, "footprint", 1)
        self.geojson_pub = geojson_publisher(
            self, "footprint_geojson", "the Map panel gets no footprint",
            qos=1)
        self.origin = MapOrigin(self)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_footprint)

        self.published = self.missed = 0
        self.create_timer(30.0, self._report)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _publish_footprint(self) -> None:
        if not intrinsics_ready(self.info):
            return
        # The newest pose, not one instant: this draws the current view, and
        # asking for "now" races the transform.
        pose = self.camera_frame.latest(timeout_s=TF_TIMEOUT_S)
        if pose is None:
            self.missed += 1
            return

        outline = self.localizer.footprint(
            image_boundary(self.info.width, self.info.height,
                           BOUNDARY_SAMPLES_PER_EDGE),
            self.info.k, pose.position, pose.rotation,
            GROUND_VIEW_MAX_DISTANCE_M)
        if outline is None:
            self.missed += 1
            self._publish_geojson([])
            return

        footprint = PolygonStamped()
        footprint.header.stamp = pose.stamp
        footprint.header.frame_id = self.reference
        footprint.polygon.points = [
            Point32(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in outline
        ]
        self.footprint_pub.publish(footprint)
        self._publish_geojson(outline)
        self.published += 1

    def _publish_geojson(self, outline) -> None:
        if not self.origin.ready:
            return
        features = []
        if outline:
            ring = self.origin.geojson_ring([(p[0], p[1]) for p in outline])
            features.append({
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                # fill false, not a transparent fill: without a fill layer
                # the interior takes no clicks. The Map panel merges `style`
                # over the topic color, so the stroke keeps this camera's one.
                "properties": {"name": f"{self.camera} footprint",
                               "style": {"fill": False}},
            })
        publish_features(self.geojson_pub, features)

    def _report(self) -> None:
        if self.published == 0 and self.missed > 0:
            self.get_logger().warn(
                f"no footprint in the last 30 s ({self.missed} attempts). Either "
                f"the transform {self.reference} -> {self.optical} is missing, "
                f"or the camera is not pointed at the ground.")
        self.published = self.missed = 0


def main() -> None:
    spin(GroundProjector)


if __name__ == "__main__":
    main()
