#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.

Matching
--------
Greedy nearest neighbour inside a gate. Each localized detection claims the
closest unclaimed truth target within `gate_radius`; anything left over on
either side is a false positive or a miss. Greedy rather than Hungarian because
the scene holds a handful of targets that sit metres apart, and at that spacing
the two agree. Say so here rather than let a reader assume this is optimal
assignment.

What the numbers mean
---------------------
A miss here is not the same as a detector miss. A target outside the camera
footprint was never visible, so counting it against the detector would be
meaningless. Only targets inside the current footprint are counted, which makes
recall a statement about "what the camera could see".

That gate is why this subscribes to the footprint. Without one, flying away
from the scene would look like a collapse in recall.

Publishes
    /scoring/summary        visualization_msgs/Marker, a text overlay
    /scoring/error_lines    visualization_msgs/MarkerArray, estimate to truth
    /scoring/position_error std_msgs/Float64, metres, per matched detection
    /scoring/recall         std_msgs/Float64, over the running window
    /scoring/precision      std_msgs/Float64, over the running window
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Point, PolygonStamped
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import ColorRGBA, Float64
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)


def point_in_polygon(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    """Ray casting. The polygon is the camera footprint, always convex here."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xint:
                inside = not inside
    return inside


class DetectionScorer(Node):
    def __init__(self) -> None:
        super().__init__("detection_scorer")

        self.declare_parameter("gate_radius", 8.0)
        self.declare_parameter("window", 100)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("require_in_footprint", True)
        self.declare_parameter("footprint_topic", "/camera/nadir/footprint")

        self.gate = float(self.get_parameter("gate_radius").value)
        self.reference = self.get_parameter("reference_frame").value
        self.require_footprint = bool(self.get_parameter("require_in_footprint").value)
        window = int(self.get_parameter("window").value)

        self.truth: list[tuple[str, float, float]] = []
        self.footprint: list[tuple[float, float]] = []
        self.errors: deque[float] = deque(maxlen=window)
        self.tp = deque(maxlen=window)
        self.fp = deque(maxlen=window)
        self.fn = deque(maxlen=window)

        self.create_subscription(Detection3DArray, "/ground_truth/truth_3d",
                                 self._on_truth, LATCHED)
        self.create_subscription(Detection3DArray, "/perception/detections_3d",
                                 self._on_detections, 10)
        self.create_subscription(PolygonStamped,
                                 self.get_parameter("footprint_topic").value,
                                 self._on_footprint, BEST_EFFORT)

        self.err_pub = self.create_publisher(Float64, "/scoring/position_error", 10)
        self.recall_pub = self.create_publisher(Float64, "/scoring/recall", 10)
        self.precision_pub = self.create_publisher(Float64, "/scoring/precision", 10)
        self.summary_pub = self.create_publisher(Marker, "/scoring/summary", LATCHED)
        self.lines_pub = self.create_publisher(MarkerArray, "/scoring/error_lines", 10)

        self.create_timer(1.0, self._publish_summary)

    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y)
                      for d in msg.detections]

    def _on_footprint(self, msg: PolygonStamped) -> None:
        self.footprint = [(p.x, p.y) for p in msg.polygon.points]

    # ------------------------------------------------------------------ score
    def _visible_truth(self) -> list[tuple[str, float, float]]:
        if not self.require_footprint or not self.footprint:
            return list(self.truth)
        return [t for t in self.truth if point_in_polygon(t[1], t[2], self.footprint)]

    def _on_detections(self, msg: Detection3DArray) -> None:
        visible = self._visible_truth()
        estimates = [(d.bbox.center.position.x, d.bbox.center.position.y)
                     for d in msg.detections]

        unclaimed = list(range(len(visible)))
        pairs = []
        for ex, ey in estimates:
            best_i, best_d = None, self.gate
            for i in unclaimed:
                _, tx, ty = visible[i]
                dist = math.hypot(ex - tx, ey - ty)
                if dist < best_d:
                    best_i, best_d = i, dist
            if best_i is None:
                self.fp.append(1)
                continue
            unclaimed.remove(best_i)
            pairs.append((ex, ey, visible[best_i], best_d))
            self.errors.append(best_d)
            self.tp.append(1)
            self.err_pub.publish(Float64(data=best_d))

        for _ in unclaimed:
            self.fn.append(1)

        lines = MarkerArray()
        for n, (ex, ey, target, dist) in enumerate(pairs):
            m = Marker()
            m.header = msg.header
            m.ns = "error"
            m.id = n
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.15
            m.pose.orientation.w = 1.0
            m.lifetime = rclpy.duration.Duration(seconds=3.0).to_msg()
            m.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.9)
            m.points = [Point(x=ex, y=ey, z=0.5),
                        Point(x=target[1], y=target[2], z=0.5)]
            lines.markers.append(m)
        if lines.markers:
            self.lines_pub.publish(lines)

    # ---------------------------------------------------------------- summary
    def _publish_summary(self) -> None:
        tp, fp, fn = len(self.tp), len(self.fp), len(self.fn)
        recall = tp / (tp + fn) if (tp + fn) else float("nan")
        precision = tp / (tp + fp) if (tp + fp) else float("nan")
        mean_err = sum(self.errors) / len(self.errors) if self.errors else float("nan")

        if tp + fp + fn:
            self.recall_pub.publish(Float64(data=0.0 if recall != recall else recall))
            self.precision_pub.publish(Float64(data=0.0 if precision != precision else precision))

        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.reference
        m.ns = "scoring"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 30.0
        m.pose.orientation.w = 1.0
        m.scale.z = 2.0
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m.text = (f"visible truth {len(self._visible_truth())}/{len(self.truth)}   "
                  f"hits {tp}  false {fp}  missed {fn}\n"
                  f"recall {recall:.2f}  precision {precision:.2f}  "
                  f"mean error {mean_err:.1f} m")
        self.summary_pub.publish(m)


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
