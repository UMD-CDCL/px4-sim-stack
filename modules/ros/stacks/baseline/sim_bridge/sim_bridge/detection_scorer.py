#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.

Matching is greedy nearest neighbour inside a gate: each estimate claims the
closest unclaimed truth target within gate_radius. The scene holds a handful
of targets meters apart, and at that spacing greedy agrees with optimal
assignment.

Only targets inside the camera footprint count as visible, so recall means
"of what the camera could see". Without that gate, flying away from the scene
would read as a collapse in recall.

One node runs for each camera. Each camera answers a different question, so
the results are never merged.

Publishes, under /scoring/<camera>/
    verdicts        vision_msgs/Detection3DArray, each labelled TP, FP or FN
    markers         visualization_msgs/MarkerArray, the verdicts as spheres
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
from std_msgs.msg import ColorRGBA, Float64
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from sim_bridge.verdicts import PERSON_SPHERE_DIAMETER, VERDICT_COLOR

# ------------------------------------------------------------------- tunables
MARKER_LIFETIME_S = 4.0
MARKER_ALPHA = 0.85
# Recall and precision are computed over the last this-many outcomes.
METRIC_WINDOW = 100

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def point_in_polygon(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    """Ray casting. The polygon is the camera footprint, always convex here."""
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
        self.declare_parameter("gate_radius", 8.0)
        self.declare_parameter("reference_frame", "map")

        self.camera = self.get_parameter("camera").value
        self.gate = float(self.get_parameter("gate_radius").value)
        self.reference = self.get_parameter("reference_frame").value

        self.truth: list[tuple[str, float, float]] = []
        self.footprint: list[tuple[float, float]] = []
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
        self.error_pub = self.create_publisher(Float64, "position_error", 10)
        self.recall_pub = self.create_publisher(Float64, "recall", 10)
        self.precision_pub = self.create_publisher(Float64, "precision", 10)

    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y)
                      for d in msg.detections]

    def _on_footprint(self, msg: PolygonStamped) -> None:
        self.footprint = [(p.x, p.y) for p in msg.polygon.points]

    def _visible_truth(self) -> list[tuple[str, float, float]]:
        if not self.footprint:
            return list(self.truth)
        return [t for t in self.truth if point_in_polygon(t[1], t[2], self.footprint)]

    def _on_detections(self, msg: Detection3DArray) -> None:
        visible = self._visible_truth()
        unclaimed = list(range(len(visible)))

        verdicts = Detection3DArray()
        verdicts.header = msg.header
        for det in msg.detections:
            x = det.bbox.center.position.x
            y = det.bbox.center.position.y
            best_index, best_distance = None, self.gate
            for i in unclaimed:
                _, truth_x, truth_y = visible[i]
                distance = math.hypot(x - truth_x, y - truth_y)
                if distance < best_distance:
                    best_index, best_distance = i, distance
            if best_index is None:
                self.false_positives.append(1)
                verdicts.detections.append(self._verdict("FP", det.id, x, y))
            else:
                unclaimed.remove(best_index)
                self.true_positives.append(1)
                self.error_pub.publish(Float64(data=best_distance))
                verdicts.detections.append(
                    self._verdict("TP", det.id, x, y, score=best_distance))

        for i in unclaimed:
            name, truth_x, truth_y = visible[i]
            self.false_negatives.append(1)
            # Nothing found this target, so there is no estimate to draw. The
            # verdict sits at the target itself.
            verdicts.detections.append(self._verdict("FN", name, truth_x, truth_y))

        self.verdict_pub.publish(verdicts)
        self.marker_pub.publish(self._verdict_markers(verdicts))
        self._publish_metrics()

    def _verdict(self, kind: str, track_id: str, x: float, y: float,
                 score: float = 0.0) -> Detection3D:
        v = Detection3D()
        v.id = track_id
        v.bbox.center.position.x = x
        v.bbox.center.position.y = y
        v.bbox.center.orientation.w = 1.0
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = kind
        hypothesis.hypothesis.score = float(score)
        v.results.append(hypothesis)
        return v

    def _verdict_markers(self, verdicts: Detection3DArray) -> MarkerArray:
        lifetime = rclpy.duration.Duration(seconds=MARKER_LIFETIME_S).to_msg()
        out = MarkerArray()
        for i, det in enumerate(verdicts.detections):
            kind = det.results[0].hypothesis.class_id if det.results else "FP"
            r, g, b = VERDICT_COLOR[kind]

            sphere = Marker()
            sphere.header = verdicts.header
            # Namespaced by camera and verdict, so the 3D panel can switch off
            # one camera's false positives without touching the other camera.
            sphere.ns = f"{self.camera}_{kind}"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.lifetime = lifetime
            sphere.pose.position.x = det.bbox.center.position.x
            sphere.pose.position.y = det.bbox.center.position.y
            sphere.pose.position.z = (det.bbox.center.position.z
                                      + PERSON_SPHERE_DIAMETER / 2.0)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = PERSON_SPHERE_DIAMETER
            sphere.color = ColorRGBA(r=r, g=g, b=b, a=MARKER_ALPHA)
            out.markers.append(sphere)
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
