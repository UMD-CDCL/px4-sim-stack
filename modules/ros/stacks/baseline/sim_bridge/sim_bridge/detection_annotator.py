#!/usr/bin/env python3
"""Draw detection boxes over the camera image in Foxglove.

The Image panel overlays `foxglove_msgs/ImageAnnotations` on any image whose
frame matches. This converts the detections into that message, so the boxes
appear on the live video the way they do on the annotated RTSP stream, without
DeepStream having to burn them into the pixels.

Drawing them here rather than reading the burned-in stream has two advantages:
the boxes stay selectable data rather than pixels, and the overlay carries the
same timestamp as the detection, so scrubbing a recording keeps them aligned.

One node runs for each camera that produces detections.

Subscribes
    <detections_topic>   vision_msgs/Detection2DArray
Publishes
    <annotations_topic>  foxglove_msgs/ImageAnnotations
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from vision_msgs.msg import Detection2DArray, Detection3DArray

try:
    from foxglove_msgs.msg import (
        Color,
        ImageAnnotations,
        PointsAnnotation,
        Point2,
        TextAnnotation,
    )
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)

# Green for a true positive, red for a false positive. A false negative has no
# box to draw, because nothing detected it; it appears in the 3D and map views
# instead. Grey means the scorer has not judged this track yet.
VERDICT_COLOUR = {"TP": (0.18, 0.80, 0.44), "FP": (0.91, 0.30, 0.24)}
UNJUDGED = (0.75, 0.75, 0.78)

PALETTE = [
    (0.99, 0.45, 0.10),
    (0.20, 0.80, 0.95),
    (0.55, 0.90, 0.35),
    (0.95, 0.85, 0.20),
    (0.85, 0.40, 0.90),
]


class DetectionAnnotator(Node):
    def __init__(self) -> None:
        super().__init__("detection_annotator")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("annotations_topic", "annotations")
        self.declare_parameter("line_thickness", 2.0)
        self.declare_parameter("text_size", 14.0)
        self.declare_parameter("show_score", False)
        self.declare_parameter("colour_by_verdict", True)
        # Each camera is scored on its own, so read the verdicts for this one.
        # Reading another camera's would colour these boxes by whether a
        # different lens found something.
        self.declare_parameter("verdicts_topic", "/scoring/verdicts")

        self.thickness = float(self.get_parameter("line_thickness").value)
        self.text_size = float(self.get_parameter("text_size").value)
        self.show_score = bool(self.get_parameter("show_score").value)

        if not HAVE_FOXGLOVE:
            self.get_logger().error(
                "foxglove_msgs is missing, so no annotations can be published. "
                "Install ros-$ROS_DISTRO-foxglove-msgs.")
            return

        self.pub = self.create_publisher(
            ImageAnnotations, self.get_parameter("annotations_topic").value, BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, self.get_parameter("detections_topic").value,
            self._on_detections, BEST_EFFORT)

        self.verdicts: dict[str, str] = {}
        if bool(self.get_parameter("colour_by_verdict").value):
            self.create_subscription(Detection3DArray,
                                     self.get_parameter("verdicts_topic").value,
                                     self._on_verdicts, 10)
        self.count = 0
        self.create_timer(60.0, self._report)
        self.get_logger().info(
            f"annotating {self.get_parameter('detections_topic').value} -> "
            f"{self.get_parameter('annotations_topic').value}")

    def _on_verdicts(self, msg: Detection3DArray) -> None:
        self.verdicts = {
            d.id: (d.results[0].hypothesis.class_id if d.results else "FP")
            for d in msg.detections
        }

    def _colour(self, track_id: str):
        if self.verdicts:
            v = self.verdicts.get(track_id)
            r, g, b = VERDICT_COLOUR.get(v, UNJUDGED) if v else UNJUDGED
            return Color(r=r, g=g, b=b, a=1.0)
        try:
            idx = int(track_id) % len(PALETTE)
        except (TypeError, ValueError):
            idx = 0
        r, g, b = PALETTE[idx]
        return Color(r=r, g=g, b=b, a=1.0)

    def _on_detections(self, msg: Detection2DArray) -> None:
        out = ImageAnnotations()

        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            hw = det.bbox.size_x / 2.0
            hh = det.bbox.size_y / 2.0
            colour = self._colour(det.id)

            box = PointsAnnotation()
            # The stamp comes from the detection, which carries the frame time
            # DeepStream reported. Using the clock here instead would slide the
            # boxes off the frame they belong to when a recording is scrubbed.
            box.timestamp = msg.header.stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.thickness = self.thickness
            box.outline_color = colour
            box.points = [
                Point2(x=cx - hw, y=cy - hh),
                Point2(x=cx + hw, y=cy - hh),
                Point2(x=cx + hw, y=cy + hh),
                Point2(x=cx - hw, y=cy + hh),
            ]
            out.points.append(box)

            label = det.results[0].hypothesis.class_id if det.results else "object"
            if det.id:
                label = f"{label} {det.id}"
            if self.show_score and det.results:
                label = f"{label} {det.results[0].hypothesis.score:.2f}"

            text = TextAnnotation()
            text.timestamp = msg.header.stamp
            # Sit the label just above the box, and keep it inside the frame.
            text.position = Point2(x=cx - hw, y=max(cy - hh - 4.0, self.text_size))
            text.text = label
            text.font_size = self.text_size
            text.text_color = Color(r=1.0, g=1.0, b=1.0, a=1.0)
            text.background_color = Color(r=0.0, g=0.0, b=0.0, a=0.6)
            out.texts.append(text)

        self.pub.publish(out)
        self.count += len(msg.detections)

    def _report(self) -> None:
        self.get_logger().info(f"{self.count} boxes drawn in the last minute")
        self.count = 0


def main() -> None:
    rclpy.init()
    node = DetectionAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
