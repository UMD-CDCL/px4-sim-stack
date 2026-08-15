#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.

Scoring runs on a clock, not on detection arrival, so a camera that detects
nothing still reports its misses: every target inside the footprint with no
estimate near it is published as an FN each tick, detections or not.

Matching is greedy nearest neighbour inside a gate: each estimate claims the
closest unclaimed truth target within gate_radius. The scene holds a handful
of targets meters apart, and at that spacing greedy agrees with optimal
assignment.

Only targets inside a fresh camera footprint count as visible. A footprint
that stops arriving means the camera sees no ground, so nothing is visible
and nothing is a miss. Estimates also expire, so a detector that goes quiet
turns its hits into misses instead of freezing the last answer.

A TP verdict carries the matched target name as a second result, so the
ground truth node can color its bubbles without matching again.

One node runs for each camera, and the results are never merged.

Publishes, under /scoring/<camera>/
    verdicts        vision_msgs/Detection3DArray, each labelled TP, FP or FN
    markers         visualization_msgs/MarkerArray, TP and FP as dots. An FN
                    has no estimate to draw and gets no dot: it appears as
                    the ground truth bubble turning yellow.
    true_positives  sensor_msgs/NavSatFix, one per TP, for the Map panel
    false_positives sensor_msgs/NavSatFix, one per FP, for the Map panel
    position_error  std_msgs/Float64, meters, one per matched estimate
    recall          std_msgs/Float64, over the running window
    precision       std_msgs/Float64, over the running window
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PolygonStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray

from sim_bridge.geo import MapOrigin
from sim_bridge.verdicts import DETECTION_DOT_DIAMETER, VERDICT_COLOR, marker

# ------------------------------------------------------------------- tunables
SCORING_RATE_HZ = 2.0
# Estimates older than this count as gone.
ESTIMATE_TIMEOUT_S = 1.0
# A footprint older than this counts as no ground in view.
FOOTPRINT_TIMEOUT_S = 2.0
# A little over one scoring period, so dots fade when scoring stops.
MARKER_LIFETIME_S = 1.0
# Recall and precision are computed over the last this-many outcomes.
METRIC_WINDOW = 100

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    """Ray casting. Works for any simple polygon, convex or not."""
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_intersect = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < x_intersect:
                inside = not inside
    return inside


class DetectionScorer(Node):
    def __init__(self) -> None:
        super().__init__("detection_scorer")

        # Names this camera in marker namespaces. Topic names come from the
        # launch namespace.
        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("detections_topic", "/perception/detections_3d")
        self.declare_parameter("footprint_topic", "/camera/nadir/footprint")
        self.declare_parameter("gate_radius", 2.0)
        self.declare_parameter("reference_frame", "map")

        self.camera = self.get_parameter("camera").value
        self.gate = float(self.get_parameter("gate_radius").value)
        self.reference = self.get_parameter("reference_frame").value

        self.truth: list[tuple[str, float, float, float]] = []
        self.footprint: list[tuple[float, float]] = []
        self.footprint_stamp = 0.0
        self.estimates: list[tuple[str, float, float, float]] = []
        self.estimates_stamp = 0.0
        self.true_positives = deque(maxlen=METRIC_WINDOW)
        self.false_positives = deque(maxlen=METRIC_WINDOW)
        self.false_negatives = deque(maxlen=METRIC_WINDOW)

        self.create_subscription(Detection3DArray, "/ground_truth/truth_3d",
                                 self._on_truth, LATCHED)
        self.create_subscription(Detection3DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._on_detections, 10)
        self.create_subscription(PolygonStamped,
                                 self.get_parameter("footprint_topic").value,
                                 self._on_footprint, BEST_EFFORT)

        self.verdict_pub = self.create_publisher(Detection3DArray, "verdicts", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 10)
        self.tp_pub = self.create_publisher(NavSatFix, "true_positives", 10)
        self.fp_pub = self.create_publisher(NavSatFix, "false_positives", 10)
        self.error_pub = self.create_publisher(Float64, "position_error", 10)
        self.recall_pub = self.create_publisher(Float64, "recall", 10)
        self.precision_pub = self.create_publisher(Float64, "precision", 10)
        self.origin = MapOrigin(self)

        self.create_timer(1.0 / SCORING_RATE_HZ, self._score)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ input
    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y,
                       d.bbox.center.position.z)
                      for d in msg.detections]

    def _on_footprint(self, msg: PolygonStamped) -> None:
        self.footprint = [(p.x, p.y) for p in msg.polygon.points]
        self.footprint_stamp = self._now()

    def _on_detections(self, msg: Detection3DArray) -> None:
        self.estimates = [(d.id, d.bbox.center.position.x,
                           d.bbox.center.position.y, d.bbox.center.position.z)
                          for d in msg.detections]
        self.estimates_stamp = self._now()

    # ------------------------------------------------------------------ score
    def _visible_truth(self) -> list[tuple[str, float, float, float]]:
        if (self._now() - self.footprint_stamp) > FOOTPRINT_TIMEOUT_S \
                or len(self.footprint) < 3:
            return []
        return [t for t in self.truth
                if point_in_polygon(t[1], t[2], self.footprint)]

    def _score(self) -> None:
        visible = self._visible_truth()
        estimates = (self.estimates
                     if (self._now() - self.estimates_stamp) <= ESTIMATE_TIMEOUT_S
                     else [])

        # Greedy matching: each estimate claims the closest unclaimed target.
        unclaimed = list(range(len(visible)))
        verdicts = Detection3DArray()
        verdicts.header.stamp = self.get_clock().now().to_msg()
        verdicts.header.frame_id = self.reference
        for track_id, x, y, z in estimates:
            best_index, best_distance = None, self.gate
            for i in unclaimed:
                distance = math.hypot(x - visible[i][1], y - visible[i][2])
                if distance < best_distance:
                    best_index, best_distance = i, distance
            if best_index is None:
                self.false_positives.append(1)
                verdicts.detections.append(self._verdict("FP", track_id, x, y, z))
                self._publish_fix(self.fp_pub, x, y, verdicts.header.stamp)
            else:
                unclaimed.remove(best_index)
                self.true_positives.append(1)
                self.error_pub.publish(Float64(data=best_distance))
                verdicts.detections.append(
                    self._verdict("TP", track_id, x, y, z,
                                  score=best_distance,
                                  matched=visible[best_index][0]))
                self._publish_fix(self.tp_pub, x, y, verdicts.header.stamp)

        for i in unclaimed:
            name, x, y, z = visible[i]
            self.false_negatives.append(1)
            verdicts.detections.append(self._verdict("FN", name, x, y, z))

        self.verdict_pub.publish(verdicts)
        self.marker_pub.publish(self._verdict_markers(verdicts))
        self._publish_metrics()

    def _publish_fix(self, publisher, x: float, y: float, stamp) -> None:
        fix = self.origin.navsat_fix(x, y, self.reference, stamp)
        if fix is not None:
            publisher.publish(fix)

    def _verdict(self, kind: str, track_id: str, x: float, y: float, z: float,
                 score: float = 0.0, matched: str = "") -> Detection3D:
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
        if matched:
            match = ObjectHypothesisWithPose()
            match.hypothesis.class_id = matched
            v.results.append(match)
        return v

    def _verdict_markers(self, verdicts: Detection3DArray) -> MarkerArray:
        lifetime = rclpy.duration.Duration(seconds=MARKER_LIFETIME_S).to_msg()
        out = MarkerArray()
        for i, det in enumerate(verdicts.detections):
            kind = det.results[0].hypothesis.class_id if det.results else "FP"
            if kind not in VERDICT_COLOR:
                continue    # an FN has no dot, only a yellow truth bubble
            # Namespaced by camera and verdict, so the 3D panel can switch
            # off one camera's false positives without touching the other.
            out.markers.append(marker(
                ns=f"{self.camera}_{kind}",
                marker_id=i,
                frame_id=verdicts.header.frame_id,
                stamp=verdicts.header.stamp,
                position=(det.bbox.center.position.x,
                          det.bbox.center.position.y,
                          det.bbox.center.position.z + DETECTION_DOT_DIAMETER / 2.0),
                size_m=DETECTION_DOT_DIAMETER,
                rgba=VERDICT_COLOR[kind],
                lifetime=lifetime))
        return out

    def _publish_metrics(self) -> None:
        tp = len(self.true_positives)
        fp = len(self.false_positives)
        fn = len(self.false_negatives)
        if tp + fn:
            self.recall_pub.publish(Float64(data=tp / (tp + fn)))
        if tp + fp:
            self.precision_pub.publish(Float64(data=tp / (tp + fp)))


def main() -> None:
    rclpy.init()
    node = DetectionScorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
