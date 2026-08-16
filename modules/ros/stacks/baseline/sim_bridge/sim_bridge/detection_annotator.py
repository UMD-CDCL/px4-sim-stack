#!/usr/bin/env python3
"""Draw detection boxes over the camera image in Foxglove.

Converts each Detection2DArray into foxglove_msgs/ImageAnnotations, which the
Image panel overlays on the live video. The box color comes from the scoring
verdict, so a box on the image matches the sphere for the same detection in
the 3D view. One node runs for each camera.

Subscribes
    <detections_topic>   vision_msgs/Detection2DArray
    <verdicts_topic>     vision_msgs/Detection3DArray, this camera's verdicts
Publishes
    <annotations_topic>  foxglove_msgs/ImageAnnotations
"""

from __future__ import annotations

from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection3DArray

from sim_bridge.runtime import BEST_EFFORT, HAVE_FOXGLOVE, require_foxglove, spin
from sim_bridge.verdicts import UNJUDGED_COLOR, VERDICT_COLOR

if HAVE_FOXGLOVE:
    from foxglove_msgs.msg import (
        Color,
        ImageAnnotations,
        Point2,
        PointsAnnotation,
        TextAnnotation,
    )

# ----------------------------------------------------------------- appearance
BOX_LINE_THICKNESS = 2.0
LABEL_FONT_SIZE = 14.0
LABEL_TEXT_COLOR = (1.0, 1.0, 1.0, 1.0)
LABEL_BACKGROUND = (0.0, 0.0, 0.0, 0.6)


def color(rgba, alpha: float | None = None) -> "Color":
    return Color(r=rgba[0], g=rgba[1], b=rgba[2],
                 a=rgba[3] if alpha is None else alpha)


class DetectionAnnotator(Node):
    def __init__(self) -> None:
        super().__init__("detection_annotator")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("annotations_topic", "annotations")
        # This camera's own verdicts. Another camera's verdicts would color
        # these boxes by what a different lens found.
        self.declare_parameter("verdicts_topic", "/scoring/verdicts")

        if not require_foxglove(self, "no annotations can be published"):
            return

        self.verdict_for: dict[str, str] = {}
        self.pub = self.create_publisher(
            ImageAnnotations, self.get_parameter("annotations_topic").value, BEST_EFFORT)
        self.create_subscription(
            Detection2DArray, self.get_parameter("detections_topic").value,
            self._on_detections, BEST_EFFORT)
        self.create_subscription(
            Detection3DArray, self.get_parameter("verdicts_topic").value,
            self._on_verdicts, 10)
        self.get_logger().info(
            f"annotating {self.get_parameter('detections_topic').value} -> "
            f"{self.get_parameter('annotations_topic').value}")

    def _on_verdicts(self, msg: Detection3DArray) -> None:
        self.verdict_for = {
            d.id: (d.results[0].hypothesis.class_id if d.results else "FP")
            for d in msg.detections
        }

    def _box_color(self, track_id: str) -> "Color":
        verdict = self.verdict_for.get(track_id)
        return color(VERDICT_COLOR.get(verdict, UNJUDGED_COLOR), alpha=1.0)

    def _on_detections(self, msg: Detection2DArray) -> None:
        out = ImageAnnotations()

        for det in msg.detections:
            center_x = det.bbox.center.position.x
            center_y = det.bbox.center.position.y
            half_w = det.bbox.size_x / 2.0
            half_h = det.bbox.size_y / 2.0

            box = PointsAnnotation()
            # The detection stamp is the frame time DeepStream reported. The
            # clock here would slide the boxes off their frame when a
            # recording is scrubbed.
            box.timestamp = msg.header.stamp
            box.type = PointsAnnotation.LINE_LOOP
            box.thickness = BOX_LINE_THICKNESS
            box.outline_color = self._box_color(det.id)
            box.points = [
                Point2(x=center_x - half_w, y=center_y - half_h),
                Point2(x=center_x + half_w, y=center_y - half_h),
                Point2(x=center_x + half_w, y=center_y + half_h),
                Point2(x=center_x - half_w, y=center_y + half_h),
            ]
            out.points.append(box)

            class_name = det.results[0].hypothesis.class_id if det.results else "object"
            label = TextAnnotation()
            label.timestamp = msg.header.stamp
            # Just above the box, kept inside the frame.
            label.position = Point2(x=center_x - half_w,
                                    y=max(center_y - half_h - 4.0, LABEL_FONT_SIZE))
            label.text = f"{class_name} {det.id}".strip()
            label.font_size = LABEL_FONT_SIZE
            label.text_color = color(LABEL_TEXT_COLOR)
            label.background_color = color(LABEL_BACKGROUND)
            out.texts.append(label)

        self.pub.publish(out)


def main() -> None:
    spin(DetectionAnnotator)


if __name__ == "__main__":
    main()
