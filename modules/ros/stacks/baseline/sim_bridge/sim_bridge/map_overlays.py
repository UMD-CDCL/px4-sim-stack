#!/usr/bin/env python3
"""Put the camera coverage and the targets on the Foxglove Map panel.

The 3D panel works in the local ENU frame. The Map panel works in latitude and
longitude and reads GeoJSON. This converts one to the other so the same
information appears in both, with no second source of truth.

What it draws
    camera fields of regard  one polygon for each camera, the ground it covers
    localized detections     one point for each estimate, coloured by verdict
                             and named by the camera that produced it
    ground truth             one point for each target

Every camera is drawn separately. Two cameras looking at one scene disagree,
and that disagreement is the measurement, so merging their estimates into one
set of points would erase it.

Colours match the 3D view and the image overlays:
    green   a true positive, an estimate that matched a target
    red     a false positive, an estimate with no target near it
    yellow  a false negative, a target inside the footprint that nothing found

This draws the same estimates that detection_localizer publishes as NavSatFix
on /perception/<camera>/detections_navsat. Both are useful and they are not
redundant: the Map panel plots a NavSatFix topic as a live point with no
styling, and GeoJSON carries the verdict colour and the label. Turn either off
in the layout without losing the other.

Publishes
    /map_overlays/geojson    foxglove_msgs/GeoJSON
"""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from vision_msgs.msg import Detection3DArray

from sim_bridge.geo import MapOrigin

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_GEOJSON = True
except ImportError:
    HAVE_GEOJSON = False

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=5)

VERDICT_COLOUR = {"TP": "#2ecc71", "FP": "#e74c3c", "FN": "#f1c40f"}
# One outline colour for each camera, in the order they are listed. The 3D
# panel uses the same two, so a footprint is recognisable across both views.
FOOTPRINT_COLOUR = ["#4dd0e1", "#ffb74d", "#ba68c8", "#aed581"]


class MapOverlays(Node):
    def __init__(self) -> None:
        super().__init__("map_overlays")

        self.declare_parameter("cameras", ["nadir", "gimbal"])
        # Topic patterns, so a stack that names things differently can say so
        # without editing this file. {cam} expands to the camera name.
        self.declare_parameter("footprint_pattern", "/camera/{cam}/footprint")
        self.declare_parameter("verdict_pattern", "/scoring/{cam}/verdicts")
        self.declare_parameter("detection_pattern", "/perception/{cam}/detections_3d")
        self.declare_parameter("truth_topic", "/ground_truth/truth_3d")
        self.declare_parameter("rate_hz", 2.0)
        # How long a verdict stays authoritative. Past this the raw
        # localizations are drawn instead, marked unjudged.
        self.declare_parameter("verdict_timeout", 3.0)

        self.cameras = [str(c) for c in self.get_parameter("cameras").value]
        self.timeout = float(self.get_parameter("verdict_timeout").value)

        self.footprints: dict[str, list[tuple[float, float]]] = {}
        # Per camera, so one camera going quiet cannot blank the other.
        self.verdicts: dict[str, list[tuple[float, float, str, str]]] = {}
        self.verdict_stamp: dict[str, float] = {}
        self.raw: dict[str, list[tuple[float, float, str]]] = {}
        self.truth: list[tuple[float, float, str]] = []

        if not HAVE_GEOJSON:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no overlays.")
            return

        self.origin = MapOrigin(self)

        foot = self.get_parameter("footprint_pattern").value
        verdict = self.get_parameter("verdict_pattern").value
        detection = self.get_parameter("detection_pattern").value
        for cam in self.cameras:
            self.create_subscription(
                PolygonStamped, foot.format(cam=cam),
                lambda msg, c=cam: self._on_footprint(c, msg), BEST_EFFORT)
            self.create_subscription(
                Detection3DArray, verdict.format(cam=cam),
                lambda msg, c=cam: self._on_verdicts(c, msg), 10)
            # Straight from the localizer, so estimates appear whether or not
            # the scorer has anything to compare them against. Scoring needs
            # ground truth, and ground truth only exists in simulation; a
            # detection is worth plotting regardless.
            self.create_subscription(
                Detection3DArray, detection.format(cam=cam),
                lambda msg, c=cam: self._on_detections(c, msg), 10)

        self.create_subscription(Detection3DArray,
                                 self.get_parameter("truth_topic").value,
                                 self._on_truth, LATCHED)

        self.pub = self.create_publisher(GeoJSON, "/map_overlays/geojson", LATCHED)
        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.2), self._publish)
        self.get_logger().info(f"drawing {', '.join(self.cameras)} on the Map panel")

    # ------------------------------------------------------------------ input
    def _on_footprint(self, cam: str, msg: PolygonStamped) -> None:
        self.footprints[cam] = [(p.x, p.y) for p in msg.polygon.points]

    def _on_verdicts(self, cam: str, msg: Detection3DArray) -> None:
        self.verdicts[cam] = [
            (d.bbox.center.position.x, d.bbox.center.position.y,
             d.results[0].hypothesis.class_id if d.results else "FP", d.id)
            for d in msg.detections
        ]
        self.verdict_stamp[cam] = self.get_clock().now().nanoseconds / 1e9

    def _on_detections(self, cam: str, msg: Detection3DArray) -> None:
        self.raw[cam] = [(d.bbox.center.position.x, d.bbox.center.position.y, d.id)
                         for d in msg.detections]

    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.bbox.center.position.x, d.bbox.center.position.y, d.id)
                      for d in msg.detections]

    # ----------------------------------------------------------------- output
    def _ll(self, x: float, y: float) -> list[float] | None:
        """Local ENU metres to [longitude, latitude], which is GeoJSON order."""
        ll = self.origin.to_lla(x, y)
        return None if ll is None else [ll[1], ll[0]]

    def _publish(self) -> None:
        if not self.origin.ready:
            return
        features = []

        for i, cam in enumerate(self.cameras):
            poly = self.footprints.get(cam, [])
            if len(poly) < 3:
                continue
            ring = [self._ll(x, y) for x, y in poly]
            if any(p is None for p in ring):
                continue
            ring.append(ring[0])            # GeoJSON rings must close
            colour = FOOTPRINT_COLOUR[i % len(FOOTPRINT_COLOUR)]
            features.append({
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [ring]},
                "properties": {"name": f"{cam} field of regard",
                               "kind": "footprint", "camera": cam,
                               "stroke": colour, "stroke-width": 2,
                               "fill": colour, "fill-opacity": 0.08},
            })

        for x, y, name in self.truth:
            point = self._ll(x, y)
            if point is None:
                continue
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": point},
                "properties": {"name": name, "kind": "ground_truth",
                               "marker-color": "#2ecc71", "marker-symbol": "circle"},
            })

        now = self.get_clock().now().nanoseconds / 1e9
        for cam in self.cameras:
            fresh = (now - self.verdict_stamp.get(cam, 0.0)) < self.timeout
            shown = (self.verdicts.get(cam, []) if fresh
                     else [(x, y, "", t) for x, y, t in self.raw.get(cam, [])])
            for x, y, verdict, track in shown:
                point = self._ll(x, y)
                if point is None:
                    continue
                features.append({
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": point},
                    "properties": {"name": f"{cam} {verdict} {track}".strip(),
                                   "kind": "detection", "camera": cam,
                                   "verdict": verdict or "unjudged",
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
