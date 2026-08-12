#!/usr/bin/env python3
"""Put the camera coverage and the targets on the Foxglove Map panel.

The 3D panel works in the local ENU frame. The Map panel works in latitude and
longitude and reads GeoJSON. This converts one to the other so the same
information appears in both, with no second source of truth.

What it draws
    camera fields of regard  one polygon per camera, the ground each one covers
    localized detections     one point per estimate, coloured by verdict
    ground truth             one point per target

Colours match the 3D view and the image overlays:
    green   a true positive, an estimate that matched a target
    red     a false positive, an estimate with no target near it
    yellow  a false negative, a target inside the footprint that nothing found

The origin comes from the vehicle's own fix paired with its local position, the
same derivation the ground truth node uses, so the map and the 3D view agree
rather than drifting apart on separate assumptions.

Publishes
    /map_overlays/geojson    foxglove_msgs/GeoJSON
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import NavSatFix
from vision_msgs.msg import Detection3DArray

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_GEOJSON = True
except ImportError:
    HAVE_GEOJSON = False

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)

EARTH_R = 6378137.0
VERDICT_COLOUR = {"TP": "#2ecc71", "FP": "#e74c3c", "FN": "#f1c40f"}


class MapOverlays(Node):
    def __init__(self) -> None:
        super().__init__("map_overlays")

        self.declare_parameter("footprint_topics",
                               ["/camera/nadir/footprint", "/camera/gimbal/footprint"])
        self.declare_parameter("footprint_names", ["nadir", "gimbal"])
        self.declare_parameter("rate_hz", 2.0)

        self.origin: tuple[float, float] | None = None
        self.local_xy: tuple[float, float] | None = None
        self.footprints: dict[str, list[tuple[float, float]]] = {}
        self.detections: list[tuple[float, float, str, str]] = []
        self.raw: list[tuple[float, float, str]] = []
        self.raw_stamp = 0.0
        self.verdict_stamp = 0.0
        self.truth: list[tuple[float, float, str]] = []

        if not HAVE_GEOJSON:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no overlays.")
            return

        topics = list(self.get_parameter("footprint_topics").value)
        names = list(self.get_parameter("footprint_names").value)
        for topic, name in zip(topics, names):
            self.create_subscription(
                PolygonStamped, topic,
                lambda msg, n=name: self._on_footprint(n, msg), BEST_EFFORT)

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, "/scoring/verdicts",
                                 self._on_verdicts, 10)
        # Localized detections direct from the localizer, so they appear on the
        # map whether or not the scorer has anything to compare them against.
        # Scoring needs ground truth, and ground truth only exists in
        # simulation; a detection is worth plotting regardless.
        self.create_subscription(Detection3DArray, "/perception/detections_3d",
                                 self._on_detections, 10)
        self.create_subscription(Detection3DArray, "/ground_truth/truth_3d",
                                 self._on_truth, LATCHED)

        self.pub = self.create_publisher(GeoJSON, "/map_overlays/geojson", LATCHED)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.2), self._publish)

    # ------------------------------------------------------------------ input
    def _on_local(self, msg) -> None:
        self.local_xy = (msg.pose.position.x, msg.pose.position.y)

    def _on_fix(self, msg) -> None:
        if self.local_xy is None or msg.status.status < 0:
            return
        x, y = self.local_xy
        self.origin = (
            msg.latitude - math.degrees(y / EARTH_R),
            msg.longitude - math.degrees(x / (EARTH_R * math.cos(math.radians(msg.latitude)))),
        )

    def _on_footprint(self, name: str, msg: PolygonStamped) -> None:
        self.footprints[name] = [(p.x, p.y) for p in msg.polygon.points]

    def _on_verdicts(self, msg: Detection3DArray) -> None:
        out = []
        for d in msg.detections:
            verdict = d.results[0].hypothesis.class_id if d.results else "FP"
            out.append((d.bbox.center.position.x, d.bbox.center.position.y,
                        verdict, d.id))
        self.detections = out
        self.verdict_stamp = self.get_clock().now().nanoseconds / 1e9

    def _on_detections(self, msg: Detection3DArray) -> None:
        self.raw = [(d.bbox.center.position.x, d.bbox.center.position.y, d.id)
                    for d in msg.detections]
        self.raw_stamp = self.get_clock().now().nanoseconds / 1e9

    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.bbox.center.position.x, d.bbox.center.position.y, d.id)
                      for d in msg.detections]

    # ----------------------------------------------------------------- output
    def _ll(self, x: float, y: float) -> list[float]:
        """Local ENU metres to [longitude, latitude], which is GeoJSON order."""
        lat0, lon0 = self.origin
        return [lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0)))),
                lat0 + math.degrees(y / EARTH_R)]

    def _publish(self) -> None:
        if self.origin is None:
            return
        features = []

        for name, poly in self.footprints.items():
            if len(poly) < 3:
                continue
            ring = [self._ll(x, y) for x, y in poly]
            ring.append(ring[0])            # GeoJSON rings must close
            features.append({
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                "properties": {"name": f"{name} field of regard", "kind": "footprint",
                               "stroke": "#4dd0e1" if name == "nadir" else "#ffb74d",
                               "stroke-width": 2, "fill": "#4dd0e1",
                               "fill-opacity": 0.08},
            })

        for x, y, name in self.truth:
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": self._ll(x, y)},
                "properties": {"name": name, "kind": "ground_truth",
                               "marker-color": "#2ecc71", "marker-symbol": "circle"},
            })

        # Verdicts when the scorer is running and current, otherwise the raw
        # localizations, marked unjudged rather than coloured as if they had
        # been checked.
        now = self.get_clock().now().nanoseconds / 1e9
        shown = (self.detections if (now - self.verdict_stamp) < 3.0
                 else [(x, y, "", t) for x, y, t in self.raw])
        for x, y, verdict, track in shown:
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": self._ll(x, y)},
                "properties": {"name": f"{verdict} {track}".strip(),
                               "kind": "detection", "verdict": verdict or "unjudged",
                               "marker-color": VERDICT_COLOUR.get(verdict, "#bdc3c7")},
            })

        msg = GeoJSON()
        msg.geojson = json.dumps({"type": "FeatureCollection", "features": features})
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MapOverlays()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
