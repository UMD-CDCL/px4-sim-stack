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

One node for each camera
------------------------
Each camera is scored on its own detections against its own footprint, and
publishes under /scoring/<camera>/. A combined number would answer a question
nobody asked: the nadir camera looking straight down and the gimbal looking out
to the horizon have different error, and merging them into one recall figure
hides which one is carrying the result.

Publishes, under /scoring/<camera>/
    verdicts        vision_msgs/Detection3DArray, every estimate and target,
                    each labelled TP, FP or FN
    markers         visualization_msgs/MarkerArray, the same verdicts drawn in
                    the 3D view
    summary         visualization_msgs/Marker, a text overlay
    error_lines     visualization_msgs/MarkerArray, estimate to truth
    position_error  std_msgs/Float64, metres, per matched detection
    recall          std_msgs/Float64, over the running window
    precision       std_msgs/Float64, over the running window
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
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
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

        # Names this camera in log lines, marker namespaces and the summary
        # text. Topic names come from the launch namespace.
        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("detections_topic", "/perception/detections_3d")
        self.declare_parameter("target_height", 1.7)
        # Height of the floating summary text. One per camera, so give each a
        # different height or they overlap into an unreadable smear.
        self.declare_parameter("summary_z", 30.0)
        self.declare_parameter("gate_radius", 8.0)
        self.declare_parameter("window", 100)
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("require_in_footprint", True)
        self.declare_parameter("footprint_topic", "/camera/nadir/footprint")

        self.gate = float(self.get_parameter("gate_radius").value)
        self.camera = self.get_parameter("camera").value
        self.target_height = float(self.get_parameter("target_height").value)
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
        self.create_subscription(Detection3DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._on_detections, 10)
        self.create_subscription(PolygonStamped,
                                 self.get_parameter("footprint_topic").value,
                                 self._on_footprint, BEST_EFFORT)

        # One message carrying every estimate and target with its verdict, so
        # the image overlay and the map colour the same things the same way
        # instead of each deciding for itself.
        # Relative names. The launch file puts this node in /scoring/<camera>.
        self.verdict_pub = self.create_publisher(Detection3DArray, "verdicts", 10)
        self.err_pub = self.create_publisher(Float64, "position_error", 10)
        self.recall_pub = self.create_publisher(Float64, "recall", 10)
        self.precision_pub = self.create_publisher(Float64, "precision", 10)
        self.summary_pub = self.create_publisher(Marker, "summary", LATCHED)
        self.lines_pub = self.create_publisher(MarkerArray, "error_lines", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "markers", 10)

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

        # Verdicts, in the vocabulary the whole stack uses:
        #   TP  an estimate that matched a target
        #   FP  an estimate with no target within the gate
        #   FN  a visible target that nothing found
        verdicts = Detection3DArray()
        verdicts.header = msg.header
        for det, (ex, ey, target, dist) in zip(msg.detections, pairs):
            v = Detection3D()
            v.header = msg.header
            v.id = det.id
            v.bbox.center.position.x, v.bbox.center.position.y = ex, ey
            v.bbox.center.orientation.w = 1.0
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = "TP"
            h.hypothesis.score = float(dist)
            v.results.append(h)
            verdicts.detections.append(v)
        matched_ids = {d.id for d in verdicts.detections}
        for det in msg.detections:
            if det.id in matched_ids:
                continue
            v = Detection3D()
            v.header = msg.header
            v.id = det.id
            v.bbox.center = det.bbox.center
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = "FP"
            v.results.append(h)
            verdicts.detections.append(v)
        for i in unclaimed:
            name, tx, ty = visible[i]
            v = Detection3D()
            v.header = msg.header
            v.id = name
            v.bbox.center.position.x, v.bbox.center.position.y = tx, ty
            v.bbox.center.orientation.w = 1.0
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = "FN"
            v.results.append(h)
            verdicts.detections.append(v)
        self.verdict_pub.publish(verdicts)
        self.markers_pub.publish(self._verdict_markers(verdicts))

        lines = MarkerArray()
        for n, (ex, ey, target, dist) in enumerate(pairs):
            m = Marker()
            m.header = msg.header
            m.ns = f"{self.camera}_error"
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

    def _verdict_markers(self, verdicts: Detection3DArray) -> MarkerArray:
        """The verdicts as pillars, in the shape ground truth already uses.

        A localization and a target are the same kind of thing in the 3D view,
        so they are drawn the same way: a pillar the height of a person, with a
        label above it. Only the colour differs, and the colour is the whole
        message. Green means this estimate landed within the gate of a real
        target. Red means it did not. Yellow is a target inside the footprint
        that nothing found, which has no estimate to draw and is therefore
        drawn at the target.

        Matching spheres to pillars was the earlier form and it read badly: a
        sphere floating beside a pillar looks like a different class of object
        rather than the same object, found or missed.
        """
        colours = {"TP": (0.18, 0.80, 0.44), "FP": (0.91, 0.30, 0.24),
                   "FN": (0.95, 0.77, 0.06)}
        life = rclpy.duration.Duration(seconds=4.0).to_msg()
        out = MarkerArray()
        for i, d in enumerate(verdicts.detections):
            v = d.results[0].hypothesis.class_id if d.results else "FP"
            r, g, b = colours.get(v, (0.6, 0.6, 0.6))
            x = d.bbox.center.position.x
            y = d.bbox.center.position.y
            z = d.bbox.center.position.z

            pillar = Marker()
            pillar.header = verdicts.header
            # Namespaced by camera and verdict, so the 3D panel can switch off
            # the gimbal's false positives without touching the nadir camera.
            pillar.ns = f"{self.camera}_{v}"
            pillar.id = i
            pillar.type = Marker.CYLINDER
            pillar.action = Marker.ADD
            pillar.lifetime = life
            pillar.pose.position.x = x
            pillar.pose.position.y = y
            pillar.pose.position.z = z + self.target_height / 2.0
            pillar.pose.orientation.w = 1.0
            pillar.scale.x = pillar.scale.y = 0.6
            pillar.scale.z = self.target_height
            # Solid enough to read against the satellite ground, translucent
            # enough to see a ground truth pillar through it when they overlap,
            # which is what a hit looks like.
            pillar.color = ColorRGBA(r=r, g=g, b=b, a=0.55)
            out.markers.append(pillar)

            label = Marker()
            label.header = verdicts.header
            label.ns = f"{self.camera}_{v}_labels"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.lifetime = life
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = z + self.target_height + 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.7
            label.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
            label.text = f"{self.camera} {v} {d.id}".strip()
            out.markers.append(label)
        return out

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
        m.ns = f"scoring_{self.camera}"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = float(self.get_parameter("summary_z").value)
        m.pose.orientation.w = 1.0
        m.scale.z = 2.0
        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        m.text = (f"{self.camera}   "
                  f"visible truth {len(self._visible_truth())}/{len(self.truth)}   "
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
