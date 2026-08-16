#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.
One node per camera, and the results are never merged.

Scoring runs on a clock, not on detection arrival, so a camera that detects
nothing still reports its misses each tick. Each estimate is judged alone by
its viewing ray, which crosses the gate of the target the detector saw even
when the estimate lands far off, as it does for an elevated target:

    TP            the estimate lies within the gate of a target. The verdict
                  names the nearest such target, and only that one.
    MISLOCALIZED  the ray crosses the gate of a target, but the estimate lies
                  within the gate of none. The verdict names every crossed
                  target.
    FP            the ray crosses the gate of no target.
    FN            the camera sees a target and no verdict names it.

The score on a TP or MISLOCALIZED is the ground distance to the nearest named
target, the error anyone acting on the estimate would experience. Verdicts are
pure geometry over every target, so a real detection overrides what the view
alone would say; the view decides only what counts as a miss. Occlusion is not
modelled. A camera whose CameraInfo stops arriving sees nothing and misses
nothing, and estimates expire rather than freezing the last answer.

Publishes, under /scoring/<camera>/
    verdicts        vision_msgs/Detection3DArray, labelled TP, MISLOCALIZED,
                    FP or FN. A TP or MISLOCALIZED carries the target names
                    it colors as further results, for the ground truth node.
    markers         visualization_msgs/MarkerArray, per sim_bridge/verdicts.py.
                    An FN has no estimate to draw and gets no mark.
    true_positives, missed_localizations, false_positives
                    foxglove_msgs/GeoJSON for the Map panel, one message per
                    tick carrying every estimate with that verdict. Each
                    feature is named after the box label the annotator draws
                    on the image, and carries the detector's confidence and
                    the targets the verdict names. A tick with none publishes
                    an empty collection, so the panel clears instead of
                    holding the last hit on screen.
    position_error  std_msgs/Float64, meters, per matched estimate
    recall, precision
                    std_msgs/Float64, over the running window
    detection_recall, detection_precision
                    the same ratios with MISLOCALIZED counted as found, so
                    they measure the detector alone

The metrics go out only while something subscribes. The window updates either
way, so a late subscriber sees correct values.
"""

from __future__ import annotations

import math
from collections import Counter, deque
from typing import NamedTuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray

from sim_bridge.frames import CameraFrame
from sim_bridge.geo import MapOrigin
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   intrinsics_ready, point_in_view,
                                   point_to_ray_distance)
from sim_bridge.runtime import (LATCHED, geojson_publisher, now_s,
                                publish_features, spin)
from sim_bridge.verdicts import (CROSS_VERDICTS, DETECTION_CROSS_LIFT,
                                 DETECTION_CROSS_SPAN, DETECTION_DOT_DIAMETER,
                                 MAP_VERDICT_COLOR, MAP_VERDICT_TOPIC,
                                 VERDICT_COLOR, annotation_text, cross_marker,
                                 marker)

# ------------------------------------------------------------------- tunables
SCORING_RATE_HZ = 2.0
# Estimates older than this count as gone.
ESTIMATE_TIMEOUT_S = 1.0
# CameraInfo older than this counts as a camera that is down.
CAMERA_TIMEOUT_S = 2.0
# A little over one scoring period, so marks fade when scoring stops.
MARKER_LIFETIME_S = 1.0
# The metrics are computed over the last this-many verdicts.
METRIC_WINDOW = 100


class Estimate(NamedTuple):
    """One localized detection. label and confidence come from the detector
    and ride through to the map feature, so a pin names the same thing the
    image box does."""
    track_id: str
    label: str
    confidence: float
    x: float
    y: float
    z: float


class DetectionScorer(Node):
    def __init__(self) -> None:
        super().__init__("detection_scorer")

        # Names this camera in marker namespaces. Topic names come from the
        # launch namespace.
        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("detections_topic", "/perception/detections_3d")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("gate_radius", 2.0)
        self.declare_parameter("reference_frame", "map")

        self.camera = self.get_parameter("camera").value
        self.gate = float(self.get_parameter("gate_radius").value)
        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value

        self.truth: list[tuple[str, float, float, float]] = []
        self.info: CameraInfo | None = None
        self.info_stamp = 0.0
        self.estimates: list[Estimate] = []
        self.estimates_stamp = 0.0
        self.window = deque(maxlen=METRIC_WINDOW)
        self.camera_frame = CameraFrame(self, self.optical, self.reference)

        self.create_subscription(Detection3DArray, "/ground_truth/truth_3d",
                                 self._on_truth, LATCHED)
        self.create_subscription(Detection3DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._on_detections, 10)
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)

        self.verdict_pub = self.create_publisher(Detection3DArray, "verdicts", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 10)
        self.map_pub = {
            kind: geojson_publisher(self, topic,
                                    f"the Map panel gets no {topic}")
            for kind, topic in MAP_VERDICT_TOPIC.items()
        }
        self.error_pub = self.create_publisher(Float64, "position_error", 10)
        self.recall_pub = self.create_publisher(Float64, "recall", 10)
        self.precision_pub = self.create_publisher(Float64, "precision", 10)
        self.detection_recall_pub = self.create_publisher(
            Float64, "detection_recall", 10)
        self.detection_precision_pub = self.create_publisher(
            Float64, "detection_precision", 10)
        self.origin = MapOrigin(self)

        self.create_timer(1.0 / SCORING_RATE_HZ, self._score)

    def _now(self) -> float:
        return now_s(self)

    # ------------------------------------------------------------------ input
    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y,
                       d.bbox.center.position.z)
                      for d in msg.detections]

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg
        self.info_stamp = self._now()

    def _on_detections(self, msg: Detection3DArray) -> None:
        self.estimates = [
            Estimate(d.id,
                     d.results[0].hypothesis.class_id if d.results else "object",
                     d.results[0].hypothesis.score if d.results else 0.0,
                     d.bbox.center.position.x, d.bbox.center.position.y,
                     d.bbox.center.position.z)
            for d in msg.detections]
        self.estimates_stamp = self._now()

    # ------------------------------------------------------------------ score
    def _current_view(self):
        """The camera position and the names of the truth targets it sees,
        as (origin, names). No usable view means (None, an empty set)."""
        if (self._now() - self.info_stamp) > CAMERA_TIMEOUT_S \
                or not intrinsics_ready(self.info):
            return None, set()
        # The newest pose, not one instant: scoring judges the current view.
        pose = self.camera_frame.latest()
        if pose is None:
            return None, set()

        # The exact truth point decides the view, nothing around it. A
        # target whose point sits just off frame is not in view, and only
        # an actual detection can claim it.
        return pose.position, {
            target[0] for target in self.truth
            if point_in_view((target[1], target[2], target[3]),
                             self.info.k, self.info.width, self.info.height,
                             pose.position, pose.rotation,
                             GROUND_VIEW_MAX_DISTANCE_M)}

    def _score(self) -> None:
        camera, in_view = self._current_view()
        targets = self.truth
        estimates = (self.estimates
                     if (self._now() - self.estimates_stamp) <= ESTIMATE_TIMEOUT_S
                     else [])

        verdicts = Detection3DArray()
        verdicts.header.stamp = self.get_clock().now().to_msg()
        verdicts.header.frame_id = self.reference
        named: set[str] = set()
        # Every estimate of one verdict rides in one message, so the Map panel
        # shows the whole tick rather than whichever arrived last.
        map_features: dict[str, list] = {kind: [] for kind in MAP_VERDICT_TOPIC}
        for est in estimates:
            ground = {name: math.hypot(est.x - tx, est.y - ty)
                      for name, tx, ty, _ in targets}
            hit = [name for name, distance in ground.items()
                   if distance <= self.gate]
            crossed = [] if camera is None else [
                name for name, tx, ty, tz in targets
                if point_to_ray_distance((tx, ty, tz), camera,
                                         (est.x, est.y, est.z)) < self.gate]
            if hit:
                kind, names = "TP", [min(hit, key=ground.get)]
            elif crossed:
                kind, names = "MISLOCALIZED", crossed
            else:
                kind, names = "FP", []
            distance = min((ground[name] for name in names), default=0.0)
            if names and self.error_pub.get_subscription_count() > 0:
                self.error_pub.publish(Float64(data=distance))
            named.update(names)
            self.window.append(kind)
            verdicts.detections.append(
                self._verdict(kind, est.track_id, est.x, est.y, est.z,
                              score=distance, names=names))
            feature = self._map_feature(est, kind, names, distance)
            if feature is not None:
                map_features[kind].append(feature)

        for name, x, y, z in targets:
            if name in named or name not in in_view:
                continue
            self.window.append("FN")
            verdicts.detections.append(self._verdict("FN", name, x, y, z))

        self.verdict_pub.publish(verdicts)
        self.marker_pub.publish(self._verdict_markers(verdicts))
        self._publish_map(map_features)
        self._publish_metrics()

    def _map_feature(self, est: Estimate, kind: str, names: list,
                     distance: float) -> dict | None:
        """One estimate as a GeoJSON point for the Map panel, or None before
        the origin is known. The Map panel shows `name` and `metadata` in the
        hover tooltip and reads Leaflet options from `style`."""
        latlon = self.origin.to_lla(est.x, est.y)
        if latlon is None:
            return None
        color = MAP_VERDICT_COLOR[kind]
        metadata = {"camera": self.camera, "verdict": kind,
                    "confidence": round(float(est.confidence), 3)}
        if names:
            # What this estimate matched, so the pin and the truth bubble it
            # colored can be read together.
            metadata["targets"] = ", ".join(names)
            metadata["error_m"] = round(distance, 2)
        return {
            "type": "Feature",
            "geometry": {"type": "Point",
                         "coordinates": [latlon[1], latlon[0]]},
            "properties": {
                "name": annotation_text(est.label, est.track_id),
                "metadata": metadata,
                "style": {"color": color, "fillColor": color,
                          "fillOpacity": 0.9},
            },
        }

    def _publish_map(self, features: dict[str, list]) -> None:
        """Every Map panel topic, every tick. A verdict with nothing this tick
        publishes an empty collection, which clears it from the panel instead
        of leaving the last hit on screen."""
        if not self.origin.ready:
            return
        for kind, publisher in self.map_pub.items():
            publish_features(publisher, features[kind])

    def _verdict(self, kind: str, track_id: str, x: float, y: float, z: float,
                 score: float = 0.0, names: list[str] = ()) -> Detection3D:
        v = Detection3D()
        v.id = track_id
        v.bbox.center.position.x = x
        v.bbox.center.position.y = y
        v.bbox.center.position.z = z
        v.bbox.center.orientation.w = 1.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = kind
        hypothesis.hypothesis.score = float(score)
        v.results.append(hypothesis)
        for name in names:
            named = ObjectHypothesisWithPose()
            named.hypothesis.class_id = name
            v.results.append(named)
        return v

    def _verdict_markers(self, verdicts: Detection3DArray) -> MarkerArray:
        lifetime = rclpy.duration.Duration(seconds=MARKER_LIFETIME_S).to_msg()
        out = MarkerArray()
        for i, det in enumerate(verdicts.detections):
            kind = det.results[0].hypothesis.class_id if det.results else "FP"
            if kind not in VERDICT_COLOR:
                continue    # an FN has no mark, only a red truth bubble
            build, size, lift = (
                (cross_marker, DETECTION_CROSS_SPAN, DETECTION_CROSS_LIFT)
                if kind in CROSS_VERDICTS
                else (marker, DETECTION_DOT_DIAMETER,
                      DETECTION_DOT_DIAMETER / 2.0))
            p = det.bbox.center.position
            # Namespaced by camera and verdict, so the 3D panel can switch
            # off one camera's false positives without touching the other.
            out.markers.append(build(
                ns=f"{self.camera}_{kind}",
                marker_id=i,
                frame_id=verdicts.header.frame_id,
                stamp=verdicts.header.stamp,
                position=(p.x, p.y, p.z + lift),
                size_m=size,
                rgba=VERDICT_COLOR[kind],
                lifetime=lifetime))
        return out

    def _publish_metrics(self) -> None:
        # The window is cumulative state, so a Plot panel that subscribes
        # late still reads correct values from its first message.
        count = Counter(self.window)
        placed = count["TP"]
        found = placed + count["MISLOCALIZED"]
        in_view = found + count["FN"]
        claimed = found + count["FP"]
        for publisher, numerator, denominator in (
                (self.recall_pub, placed, in_view),
                (self.detection_recall_pub, found, in_view),
                (self.precision_pub, placed, claimed),
                (self.detection_precision_pub, found, claimed)):
            if denominator and publisher.get_subscription_count() > 0:
                publisher.publish(Float64(data=numerator / denominator))


def main() -> None:
    spin(DetectionScorer)


if __name__ == "__main__":
    main()
