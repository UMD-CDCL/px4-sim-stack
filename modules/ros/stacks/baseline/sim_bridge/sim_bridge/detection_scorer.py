#!/usr/bin/env python3
"""Score localized detections against ground truth, while the drone flies.

Scoring runs on a clock, not on detection arrival, so a camera that detects
nothing still reports its misses: every target in view with no estimate
near it is published as an FN each tick, detections or not.

Matching is greedy, closest pair first: estimates and truth targets claim
each other in order of ground distance, inside detection_radius. The scene
holds a handful of targets meters apart, and at that spacing greedy agrees
with optimal assignment. Estimates still unmatched get a second pass by
viewing ray: a detection of an elevated target projects through it onto
the ground far beyond, so an estimate whose ray from the camera passes
within the gate of a target claims that target, whatever the ground
distance.

The ground distance then splits detection from localization. Within
gate_radius the verdict is TP: the detector saw the target and placed it.
Otherwise a matched estimate is MISLOCALIZED: the detector saw the target,
but the position it reported is not good enough to act on. An unmatched
estimate is FP.

A target counts as visible when the camera sees it in 3D: some part of its
standing height projects inside the image, in front of the camera, within
the same distance that truncates the footprint. The footprint polygon would
miss an elevated target, on a roof for instance, whose ground coordinates
fall outside the polygon while the camera looks straight at it. Occlusion
is not modelled: a target behind a structure but inside the view still
counts, the same flat-scene assumption the rest of the pipeline makes.

CameraInfo that stops arriving means the camera is down, so nothing is
visible and nothing is a miss. Estimates also expire, so a detector that
goes quiet turns its hits into misses instead of freezing the last answer.

A TP or MISLOCALIZED verdict carries the matched target name as a second
result, so the ground truth node can color its bubbles without matching
again.

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
                                   PERSON_HEIGHT_M, intrinsics_ready,
                                   point_in_view, point_to_ray_distance)
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
        self.declare_parameter("detection_radius", 10.0)
        self.declare_parameter("reference_frame", "map")

        self.camera = self.get_parameter("camera").value
        self.gate = float(self.get_parameter("gate_radius").value)
        self.detection_radius = float(self.get_parameter("detection_radius").value)
        if self.detection_radius < self.gate:
            self.get_logger().warn(
                f"detection_radius {self.detection_radius} is inside the gate "
                f"{self.gate}, so it is raised to the gate. A claim within the "
                f"gate must count as a TP.")
            self.detection_radius = self.gate
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
        """The camera position and the truth targets it sees, as
        (origin, targets). No usable view means (None, [])."""
        if (self._now() - self.info_stamp) > CAMERA_TIMEOUT_S \
                or not intrinsics_ready(self.info):
            return None, []
        try:
            # Latest available rather than a specific time: scoring judges
            # the current view, and asking for "now" races the transform.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, rclpy.time.Time())
        except Exception:
            return None, []
        t, r = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rotation = (r.x, r.y, r.z, r.w)

        def sees(x: float, y: float, z: float) -> bool:
            return point_in_view((x, y, z), self.info.k, self.info.width,
                                 self.info.height, origin, rotation,
                                 GROUND_VIEW_MAX_DISTANCE_M)

        # Both ends of the standing height: a person's base can project just
        # off the frame edge while the torso shows, and the detector still
        # boxes the torso.
        return origin, [target for target in self.truth
                        if sees(target[1], target[2], target[3])
                        or sees(target[1], target[2], target[3] + PERSON_HEIGHT_M)]

    def _score(self) -> None:
        camera, visible = self._current_view()
        estimates = (self.estimates
                     if (self._now() - self.estimates_stamp) <= ESTIMATE_TIMEOUT_S
                     else [])

        # Greedy matching, closest pair first, so a far estimate cannot take
        # a target from a near one.
        pairs = sorted(
            (math.hypot(ex - tx, ey - ty), e, t)
            for e, (_, ex, ey, _) in enumerate(estimates)
            for t, (_, tx, ty, _) in enumerate(visible))
        match: dict[int, tuple[int, float]] = {}
        claimed: set[int] = set()
        for distance, e, t in pairs:
            if distance >= self.detection_radius:
                break
            if e in match or t in claimed:
                continue
            match[e] = (t, distance)
            claimed.add(t)

        # Second pass, by viewing ray: a detection of an elevated target
        # projects through it onto the ground far beyond, a roof being the
        # usual case. The estimate's ray from the camera still passes
        # within the gate of the target it saw, so claim by that distance.
        # The ground distance is kept as the score, because it is the error
        # anyone acting on the estimate would experience.
        if camera is not None:
            ray_pairs = sorted(
                (min(point_to_ray_distance((tx, ty, tz), camera, (ex, ey, ez)),
                     point_to_ray_distance((tx, ty, tz + PERSON_HEIGHT_M),
                                           camera, (ex, ey, ez))),
                 e, t)
                for e, (_, ex, ey, ez) in enumerate(estimates) if e not in match
                for t, (_, tx, ty, tz) in enumerate(visible) if t not in claimed)
            for ray_distance, e, t in ray_pairs:
                if ray_distance >= self.gate:
                    break
                if e in match or t in claimed:
                    continue
                match[e] = (t, math.hypot(estimates[e][1] - visible[t][1],
                                          estimates[e][2] - visible[t][2]))
                claimed.add(t)

        verdicts = Detection3DArray()
        verdicts.header.stamp = self.get_clock().now().to_msg()
        verdicts.header.frame_id = self.reference
        for e, (track_id, x, y, z) in enumerate(estimates):
            if e not in match:
                kind, matched, distance = "FP", "", 0.0
            else:
                t, distance = match[e]
                kind = "TP" if distance <= self.gate else "MISLOCALIZED"
                matched = visible[t][0]
                if self.error_pub.get_subscription_count() > 0:
                    self.error_pub.publish(Float64(data=distance))
            self.window.append(kind)
            verdicts.detections.append(
                self._verdict(kind, track_id, x, y, z,
                              score=distance, matched=matched))
            self._publish_fix(kind, x, y, verdicts.header.stamp)

        for t, (name, x, y, z) in enumerate(visible):
            if t in claimed:
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
