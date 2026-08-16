#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.

Scoring runs on a clock, not on detection arrival, so a camera that detects
nothing still reports its misses: every target in view with no estimate
near it is published as an FN each tick, detections or not.

Each estimate is judged alone, by its viewing ray. The ray runs from the
camera through the estimate, so it crosses the gate of the target the
detector saw, an elevated target included: a detection on a roof projects
onto the ground far beyond, but the ray still passes within the gate.

The verdict follows from the estimate and its ray:
    TP            the estimate lies within the gate of a target. The
                  verdict names the nearest such target, and only that
                  one: the other targets the ray crosses keep the status
                  the view gives them.
    MISLOCALIZED  the ray crosses the gate of a target, but the estimate
                  lies within the gate of none. The verdict names every
                  crossed target, and each one turns yellow unless a TP
                  turns it green.
    FP            the ray crosses the gate of no target.
The score on a TP or MISLOCALIZED verdict is the ground distance to the
nearest named target, because that is the error anyone acting on the
estimate would experience. Without a camera pose there are no rays, so
an estimate is a TP within a gate and an FP everywhere else.

Verdicts are pure geometry over every target, in view or not. A real
detection therefore overrides what the view alone would say about a
target: one behind a roof still turns green or yellow when a ray
crosses it.

The view decides only what counts as a miss: a target is an FN when the
camera sees it and no verdict names it. A target counts as in view when
its exact point projects inside the image, in front of the camera, within
the same distance that truncates the footprint. Occlusion is not
modelled: a target behind a structure but inside the view still counts,
the same flat-scene assumption the rest of the pipeline makes.

CameraInfo that stops arriving means the camera is down, so nothing is in
view and nothing is a miss. Estimates also expire, so a detector that
goes quiet turns its hits into misses instead of freezing the last answer.

A TP or MISLOCALIZED verdict carries the target names it colors as
further results, so the ground truth node can color its bubbles without
matching again.

One node runs for each camera, and the results are never merged.

Publishes, under /scoring/<camera>/
    verdicts        vision_msgs/Detection3DArray, each labelled TP,
                    MISLOCALIZED, FP or FN
    markers         visualization_msgs/MarkerArray. A TP is a green dot, a
                    MISLOCALIZED estimate a yellow cross, an FP a red cross.
                    An FN has no estimate to draw and gets no mark: it
                    appears as the ground truth bubble turning red.
    true_positives  sensor_msgs/NavSatFix, one per TP, for the Map panel
    missed_localizations
                    sensor_msgs/NavSatFix, one per MISLOCALIZED
    false_positives sensor_msgs/NavSatFix, one per FP, for the Map panel
    position_error  std_msgs/Float64, meters, one per matched estimate,
                    mislocalized ones included
    recall          std_msgs/Float64, targets placed within the gate over
                    targets in view, across the running window
    precision       std_msgs/Float64, estimates within the gate over all
                    estimates, across the running window
    detection_recall, detection_precision
                    std_msgs/Float64, the same ratios with MISLOCALIZED
                    counted as found, so they measure the detector alone

The metrics go out only while something subscribes to them. The window
updates either way, so a late subscriber sees correct values.
"""

from __future__ import annotations

import math
from collections import Counter, deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, NavSatFix
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import MarkerArray

from sim_bridge.geo import MapOrigin
from sim_bridge.projection import (GROUND_VIEW_MAX_DISTANCE_M,
                                   intrinsics_ready, point_in_view,
                                   point_to_ray_distance)
from sim_bridge.verdicts import (CROSS_VERDICTS, DETECTION_CROSS_LIFT,
                                 DETECTION_CROSS_SPAN, DETECTION_DOT_DIAMETER,
                                 VERDICT_COLOR, cross_marker, marker)

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

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


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
        self.estimates: list[tuple[str, float, float, float]] = []
        self.estimates_stamp = 0.0
        self.window = deque(maxlen=METRIC_WINDOW)

        self.tf_buffer = Buffer()
        # spin_thread=True is required. On this node's executor, a lookup that
        # waits for a transform would block the callback that delivers it.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

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
        self.fix_pub = {
            "TP": self.create_publisher(NavSatFix, "true_positives", 10),
            "MISLOCALIZED": self.create_publisher(NavSatFix, "missed_localizations", 10),
            "FP": self.create_publisher(NavSatFix, "false_positives", 10),
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
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------------ input
    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y,
                       d.bbox.center.position.z)
                      for d in msg.detections]

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg
        self.info_stamp = self._now()

    def _on_detections(self, msg: Detection3DArray) -> None:
        self.estimates = [(d.id, d.bbox.center.position.x,
                           d.bbox.center.position.y, d.bbox.center.position.z)
                          for d in msg.detections]
        self.estimates_stamp = self._now()

    # ------------------------------------------------------------------ score
    def _current_view(self):
        """The camera position and the names of the truth targets it sees,
        as (origin, names). No usable view means (None, an empty set)."""
        if (self._now() - self.info_stamp) > CAMERA_TIMEOUT_S \
                or not intrinsics_ready(self.info):
            return None, set()
        try:
            # Latest available rather than a specific time: scoring judges
            # the current view, and asking for "now" races the transform.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, rclpy.time.Time())
        except Exception:
            return None, set()
        t, r = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rotation = (r.x, r.y, r.z, r.w)

        # The exact truth point decides the view, nothing around it. A
        # target whose point sits just off frame is not in view, and only
        # an actual detection can claim it.
        return origin, {target[0] for target in self.truth
                        if point_in_view((target[1], target[2], target[3]),
                                         self.info.k, self.info.width,
                                         self.info.height, origin, rotation,
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
        for track_id, x, y, z in estimates:
            ground = {name: math.hypot(x - tx, y - ty)
                      for name, tx, ty, _ in targets}
            hit = [name for name, distance in ground.items()
                   if distance <= self.gate]
            crossed = [] if camera is None else [
                name for name, tx, ty, tz in targets
                if point_to_ray_distance((tx, ty, tz), camera,
                                         (x, y, z)) < self.gate]
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
                self._verdict(kind, track_id, x, y, z,
                              score=distance, names=names))
            self._publish_fix(kind, x, y, verdicts.header.stamp)

        for name, x, y, z in targets:
            if name in named or name not in in_view:
                continue
            self.window.append("FN")
            verdicts.detections.append(self._verdict("FN", name, x, y, z))

        self.verdict_pub.publish(verdicts)
        self.marker_pub.publish(self._verdict_markers(verdicts))
        self._publish_metrics()

    def _publish_fix(self, kind: str, x: float, y: float, stamp) -> None:
        fix = self.origin.navsat_fix(x, y, self.reference, stamp)
        if fix is not None:
            self.fix_pub[kind].publish(fix)

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
            position = (det.bbox.center.position.x, det.bbox.center.position.y,
                        det.bbox.center.position.z)
            # Namespaced by camera and verdict, so the 3D panel can switch
            # off one camera's false positives without touching the other.
            if kind in CROSS_VERDICTS:
                out.markers.append(cross_marker(
                    ns=f"{self.camera}_{kind}",
                    marker_id=i,
                    frame_id=verdicts.header.frame_id,
                    stamp=verdicts.header.stamp,
                    position=(position[0], position[1],
                              position[2] + DETECTION_CROSS_LIFT),
                    span_m=DETECTION_CROSS_SPAN,
                    rgba=VERDICT_COLOR[kind],
                    lifetime=lifetime))
            else:
                out.markers.append(marker(
                    ns=f"{self.camera}_{kind}",
                    marker_id=i,
                    frame_id=verdicts.header.frame_id,
                    stamp=verdicts.header.stamp,
                    position=(position[0], position[1],
                              position[2] + DETECTION_DOT_DIAMETER / 2.0),
                    size_m=DETECTION_DOT_DIAMETER,
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
