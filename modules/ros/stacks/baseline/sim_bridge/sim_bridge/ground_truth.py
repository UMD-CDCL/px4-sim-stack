#!/usr/bin/env python3
"""Publish where the targets actually are, for scoring detections against.

The simulator places targets from a scenario file, so their positions are known
exactly. Publishing them alongside the estimated ones turns "the detector seems
to work" into a number.

About the module boundary
-------------------------
This is the one node that reads a simulator file. That is deliberate and it is
safe, because ground truth is evaluation data, not vehicle data. Nothing in the
flight path reads it, no other node depends on it, and on real hardware you
simply do not run it. Keeping it here, rather than inventing a truth channel
through the message bus, keeps the plumbing honest about what it is.

Frames
------
Scenario poses are Gazebo world coordinates: x east, y north, in metres from
the world origin. PX4's local frame starts at the point where the EKF
initialized, which is where the vehicle spawned, so the two line up. When they
do not, `origin_offset_xyz` shifts them.

Latitude and longitude are derived from the drone itself. PX4 does not send
GPS_GLOBAL_ORIGIN unless something asks, so `/mavros/global_position/gp_origin`
stays silent and cannot be relied on. Instead this pairs the drone's global fix
with its local position: if the aircraft sits at local (x, y) and reports a
given latitude and longitude, then local (0, 0) is that fix walked back by
(x, y). The result tracks the same EKF the local frame comes from, so the map
view and the 3D view agree, and no coordinate is copied from .env.

Publishes
    /ground_truth/markers      visualization_msgs/MarkerArray  (3D panel)
    /ground_truth/truth_3d     vision_msgs/Detection3DArray    (for the scorer)
    /ground_truth/geojson      foxglove_msgs/GeoJSON           (Map panel)
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

try:
    from geographic_msgs.msg import GeoPointStamped
    HAVE_GEO = True
except ImportError:
    HAVE_GEO = False

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_GEOJSON = True
except ImportError:
    HAVE_GEOJSON = False

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

EARTH_R = 6378137.0


class GroundTruth(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth")

        self.declare_parameter("scenario_file", os.environ.get(
            "GROUND_TRUTH_FILE", "/scenes/scenarios/urban_casualties.yaml"))
        # The scenario file says where entities were asked to go. After
        # spawning, spawn_scenario.py reads back where Gazebo actually put them
        # and writes this file. Prefer it: a mesh whose origin is not at its
        # feet, a model that settled under gravity, or one that failed to spawn
        # and left an older copy behind all make the request and the result
        # differ, and scoring against the request measures the wrong thing.
        self.declare_parameter("resolved_file", os.environ.get(
            "RESOLVED_TRUTH_FILE", "/scenes/ground_truth_actual.yaml"))
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("origin_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("target_height", 1.7)
        self.declare_parameter("rate_hz", 1.0)
        # Only entities whose name contains one of these count as findable
        # targets. A scenario can also hold props that no detector should find.
        self.declare_parameter("target_name_filters", ["person", "casualty"])

        self.reference = self.get_parameter("reference_frame").value
        self.offset = [float(x) for x in self.get_parameter("origin_offset_xyz").value]
        self.height = float(self.get_parameter("target_height").value)
        self.filters = [s.lower() for s in self.get_parameter("target_name_filters").value]

        self.origin_lat: float | None = None
        self.origin_lon: float | None = None

        self.targets = self._load()
        if not self.targets:
            self.get_logger().warn(
                f"no targets loaded from {self.get_parameter('scenario_file').value}. "
                f"Scoring will report every detection as a false positive.")
        else:
            names = ", ".join(t["name"] for t in self.targets)
            self.get_logger().info(f"{len(self.targets)} ground truth targets: {names}")

        self.marker_pub = self.create_publisher(MarkerArray, "/ground_truth/markers", LATCHED)
        self.truth_pub = self.create_publisher(Detection3DArray, "/ground_truth/truth_3d", LATCHED)
        self.geojson_pub = (self.create_publisher(GeoJSON, "/ground_truth/geojson", LATCHED)
                            if HAVE_GEOJSON else None)
        if not HAVE_GEOJSON:
            self.get_logger().warn(
                "foxglove_msgs is missing, so the Map panel gets no ground truth. "
                "Install ros-$ROS_DISTRO-foxglove-msgs.")

        # gp_origin is preferred when it appears, and usually it does not.
        if HAVE_GEO:
            self.create_subscription(GeoPointStamped, "/mavros/global_position/gp_origin",
                                     self._on_origin, qos_profile_sensor_data)
        self.local_xy: tuple[float, float] | None = None
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)
        self.origin_from_fix = False

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish)

    # ------------------------------------------------------------------ input
    def _load(self) -> list[dict]:
        resolved = Path(self.get_parameter("resolved_file").value)
        if resolved.is_file():
            targets = self._load_file(resolved)
            if targets:
                self.get_logger().info(
                    f"using the poses read back from Gazebo, {resolved}")
                return targets
            self.get_logger().warn(
                f"{resolved} has no usable entities, falling back to the scenario")
        else:
            self.get_logger().warn(
                f"no {resolved}; using the scenario file, which records where "
                f"entities were asked to go rather than where they are. Re-run "
                f"the scenario to produce it: px4sim scenario")
        return self._load_file(Path(self.get_parameter("scenario_file").value))

    def _load_file(self, path: Path) -> list[dict]:
        if not path.is_file():
            self.get_logger().error(f"no scenario file at {path}")
            return []
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError as exc:
            self.get_logger().error(f"cannot parse {path}: {exc}")
            return []

        out = []
        for entity in data.get("entities", []):
            name = str(entity.get("name", ""))
            if self.filters and not any(f in name.lower() for f in self.filters):
                continue
            pose = list(entity.get("pose", []))
            if len(pose) < 3:
                continue
            out.append({
                "name": name,
                "x": float(pose[0]) + self.offset[0],
                "y": float(pose[1]) + self.offset[1],
                "z": float(pose[2]) + self.offset[2],
                "uri": entity.get("uri", ""),
            })
        return out

    def _on_origin(self, msg) -> None:
        # PX4 sends the origin once the EKF has one. Before that the map view
        # simply has no ground truth, which is better than placing it at 0, 0
        # in the Gulf of Guinea.
        if msg.position.latitude or msg.position.longitude:
            self.origin_lat = msg.position.latitude
            self.origin_lon = msg.position.longitude
            self.origin_from_fix = False

    def _on_local(self, msg) -> None:
        self.local_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_fix(self, msg) -> None:
        # Walk the drone's own fix back to local (0, 0). Skip it once gp_origin
        # has given a real answer, and skip a fix with no lock.
        if self.origin_lat is not None and not self.origin_from_fix:
            return
        if self.local_xy is None or msg.status.status < 0:
            return
        x, y = self.local_xy
        lat = msg.latitude - math.degrees(y / EARTH_R)
        lon = msg.longitude - math.degrees(x / (EARTH_R * math.cos(math.radians(msg.latitude))))
        first = self.origin_lat is None
        self.origin_lat, self.origin_lon = lat, lon
        self.origin_from_fix = True
        if first:
            self.get_logger().info(
                f"map origin from the vehicle fix: {lat:.7f}, {lon:.7f}")

    def _to_latlon(self, x: float, y: float) -> tuple[float, float] | None:
        """Local ENU metres to WGS84, flat-earth about the origin.

        Good to a few centimetres over the hundreds of metres a scene covers,
        which is far below the localization error being measured.
        """
        if self.origin_lat is None:
            return None
        lat = self.origin_lat + math.degrees(y / EARTH_R)
        lon = self.origin_lon + math.degrees(x / (EARTH_R * math.cos(math.radians(self.origin_lat))))
        return lat, lon

    # ---------------------------------------------------------------- output
    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        truth = Detection3DArray()
        truth.header.stamp = now
        truth.header.frame_id = self.reference
        features = []

        for i, t in enumerate(self.targets):
            # A translucent green pillar, so an estimate sitting inside it is
            # obviously a hit and one beside it is obviously not.
            pillar = Marker()
            pillar.header.stamp = now
            pillar.header.frame_id = self.reference
            pillar.ns = "ground_truth"
            pillar.id = i
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.pose.position.x = t["x"]
            pillar.pose.position.y = t["y"]
            pillar.pose.position.z = t["z"] + self.height / 2.0
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = pillar.scale.y = 0.6
            pillar.scale.z = self.height
            pillar.color = ColorRGBA(r=0.15, g=0.85, b=0.3, a=0.35)
            markers.markers.append(pillar)

            label = Marker()
            label.header = pillar.header
            label.ns = "ground_truth_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = t["x"]
            label.pose.position.y = t["y"]
            label.pose.position.z = t["z"] + self.height + 1.2
            label.pose.orientation.w = 1.0
            label.scale.z = 0.7
            label.color = ColorRGBA(r=0.5, g=1.0, b=0.6, a=0.9)
            label.text = t["name"]
            markers.markers.append(label)

            d = Detection3D()
            d.header = truth.header
            d.id = t["name"]
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "person"
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = t["x"]
            hyp.pose.pose.position.y = t["y"]
            hyp.pose.pose.position.z = t["z"]
            hyp.pose.pose.orientation.w = 1.0
            d.results.append(hyp)
            d.bbox = BoundingBox3D()
            d.bbox.center = hyp.pose.pose
            d.bbox.size.x = d.bbox.size.y = 0.6
            d.bbox.size.z = self.height
            truth.detections.append(d)

            ll = self._to_latlon(t["x"], t["y"])
            if ll is not None:
                features.append({
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [ll[1], ll[0]]},
                    "properties": {"name": t["name"], "kind": "ground_truth",
                                   "marker-color": "#2ecc71"},
                })

        self.marker_pub.publish(markers)
        self.truth_pub.publish(truth)

        if self.geojson_pub is not None and features:
            msg = GeoJSON()
            msg.geojson = json.dumps({"type": "FeatureCollection", "features": features})
            self.geojson_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
