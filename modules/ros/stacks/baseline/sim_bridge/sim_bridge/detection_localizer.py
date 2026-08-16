#!/usr/bin/env python3
"""Turn image-space detections into ground positions. One node per camera.

Each detection casts a ray through its box anchor onto the surface
sim_bridge/localization.py selects, so a click and a detection through the
same pixel land on the same point.

Every transform lookup uses the detection stamp, the frame time DeepStream
reported. Detections arrive 60 to 90 ms after capture, and a moving drone or
a slewing gimbal makes "where is the camera now" the wrong question. With no
transform at the frame time the detection is dropped: a silently wrong
position is worse than a missing one.

Publishes, under /perception/<camera>/
    detections_3d   vision_msgs/Detection3DArray with covariance
"""

from __future__ import annotations

import math

from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from sim_bridge.localization import GroundLocalizer
from sim_bridge.projection import (intrinsics_ready, quat_rotate,
                                   ray_in_optical, slant_range)
from sim_bridge.runtime import BEST_EFFORT, spin, tf_buffer

# ------------------------------------------------------------------- tunables
# Diagonal of the 6x6 pose covariance, in m^2 and rad^2, as
# [x, y, z, roll, pitch, yaw]. An estimate, not a calibration: 4.0 is a two
# metre standard deviation, the right order for a pinhole model over an
# assumed flat ground.
COVARIANCE_DIAGONAL = [4.0, 4.0, 1.0, 0.0, 0.0, 0.0]
# Extra x and y variance per metre of slant range, squared. A shallow look
# angle stretches a pixel across much more ground. 0.0004 adds a two
# centimetre standard deviation for every metre of range.
RANGE_VARIANCE_SCALE = 0.0004
# Beyond this slant range a ray is treated as missing the ground.
MAX_RANGE = 2000.0
# The detection stamp is often a few tens of milliseconds newer than the
# newest transform, because the detection pipeline outruns telemetry. Within
# this bound the newest transform is close enough. Past it, drop.
FUTURE_TOLERANCE_S = 0.15


class DetectionLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("detection_localizer")

        # Names this camera in log lines. Topic names come from the launch
        # namespace.
        self.declare_parameter("camera", "gimbal")
        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        # Which point of the box to project, "bottom" or "center". Looking
        # obliquely, the feet touch the ground, so the bottom edge is right.
        # Looking straight down, the center is.
        self.declare_parameter("anchor", "bottom")
        # Publish results in this frame. Empty means the reference frame. Set
        # it to the fiducial frame to remove the surveyed bias, see
        # fiducial_alignment.py.
        self.declare_parameter("output_frame", "")

        self.camera = self.get_parameter("camera").value
        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.anchor = self.get_parameter("anchor").value
        self.output_frame = self.get_parameter("output_frame").value or self.reference
        self.localizer = GroundLocalizer(self)
        self.warned_output = False
        self.tf_buffer = tf_buffer(self, cache_time=Duration(seconds=10.0))

        self.info: CameraInfo | None = None
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._on_detections, BEST_EFFORT)

        self.det3d_pub = self.create_publisher(Detection3DArray, "detections_3d", 10)

        self.localized = self.no_tf = self.no_ground = self.clamped = 0
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f"localizing {self.camera} into {self.reference} from "
            f"{self.optical}, anchor={self.anchor}, "
            f"onto {self.localizer.description}")

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    # ------------------------------------------------------------------- work
    def _on_detections(self, msg: Detection2DArray) -> None:
        if not msg.detections or not intrinsics_ready(self.info):
            return

        stamp = Time.from_msg(msg.header.stamp)
        try:
            # The frame time, not now, and no blocking wait. The frame time
            # is already in the past when a detection arrives, so the
            # transform is in the buffer or it is not: waiting here once let
            # the full detection rate starve this node's own TF listener,
            # and then every lookup failed.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, stamp)
        except Exception as exc:
            tf = self._clamped_lookup(stamp, exc)
            if tf is None:
                return

        t, r = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rotation = (r.x, r.y, r.z, r.w)

        # The output transform is static, so one lookup serves every
        # detection in the array.
        output_tf = None
        if self.output_frame != self.reference:
            output_tf = self._output_transform()
            if output_tf is None:
                return

        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame

        for det in msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            if self.anchor == "bottom":
                v += det.bbox.size_y / 2.0

            direction = quat_rotate(rotation, ray_in_optical(u, v, self.info.k))
            ground_point = self.localizer.intersect(origin, direction, MAX_RANGE)
            if ground_point is None:
                self.no_ground += 1
                continue

            ground_point = self._to_output(ground_point, output_tf)
            distance = slant_range(origin, ground_point)
            range_variance = RANGE_VARIANCE_SCALE * distance * distance

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, ground_point)
            pose.orientation.w = 1.0

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = (
                det.results[0].hypothesis.class_id if det.results else "object")
            hypothesis.hypothesis.score = (
                det.results[0].hypothesis.score if det.results else 0.0)
            hypothesis.pose.pose = pose
            # The diagonal of a row-major 6x6. Range widens x and y only; z is
            # fixed by the surface and the angles are not estimated.
            spread = [range_variance, range_variance, 0.0, 0.0, 0.0, 0.0]
            covariance = [0.0] * 36
            for axis, (base, extra) in enumerate(zip(COVARIANCE_DIAGONAL, spread)):
                covariance[axis * 7] = base + extra
            hypothesis.pose.covariance = covariance

            d3 = Detection3D()
            d3.header = out.header
            d3.id = det.id
            d3.results.append(hypothesis)
            d3.bbox = BoundingBox3D()
            d3.bbox.center = pose
            d3.bbox.size.x = 2.0 * math.sqrt(covariance[0])
            d3.bbox.size.y = 2.0 * math.sqrt(covariance[7])
            d3.bbox.size.z = 2.0 * math.sqrt(covariance[14])
            out.detections.append(d3)
            self.localized += 1

        if out.detections:
            self.det3d_pub.publish(out)

    def _output_transform(self):
        """The transform into the output frame, or None while it is missing."""
        try:
            return self.tf_buffer.lookup_transform(
                self.output_frame, self.reference, Time())
        except Exception as exc:
            if not self.warned_output:
                self.warned_output = True
                self.get_logger().warn(
                    f"output_frame is '{self.output_frame}' but there is no "
                    f"transform from '{self.reference}' to it ({exc}). Is "
                    f"fiducial_alignment enabled? Publishing nothing until it is.")
            return None

    def _to_output(self, point, output_tf):
        """Move a point into the output frame. output_tf None means the
        output frame is the reference, so the point stays put."""
        if output_tf is None:
            return point
        t = output_tf.transform.translation
        r = output_tf.transform.rotation
        moved = quat_rotate((r.x, r.y, r.z, r.w), point)
        return (moved[0] + t.x, moved[1] + t.y, moved[2] + t.z)

    def _clamped_lookup(self, stamp: Time, first_error: Exception):
        """Fall back to the newest transform when the frame time runs ahead.

        Only within FUTURE_TOLERANCE_S, and only forward in time. A lookup
        that failed for any other reason, such as a missing frame, still fails.
        """
        try:
            latest = self.tf_buffer.lookup_transform(
                self.reference, self.optical, Time())
        except Exception:
            self._note_drop(first_error)
            return None

        latest_ns = (latest.header.stamp.sec * 1_000_000_000
                     + latest.header.stamp.nanosec)
        ahead = (stamp.nanoseconds - latest_ns) / 1e9
        if 0.0 <= ahead <= FUTURE_TOLERANCE_S:
            self.clamped += 1
            return latest
        self._note_drop(first_error)
        return None

    def _note_drop(self, exc: Exception) -> None:
        self.no_tf += 1
        if self.no_tf in (1, 200):
            self.get_logger().warn(
                f"no transform {self.reference} -> {self.optical} at the frame "
                f"time ({exc}). Those detections are dropped.")

    def _report(self) -> None:
        if self.localized or self.no_tf or self.no_ground:
            self.get_logger().info(
                f"{self.camera}: {self.localized} localized ({self.clamped} clamped to the "
                f"newest transform), {self.no_tf} dropped with no transform at "
                f"the frame time, {self.no_ground} rays that missed the ground")
        self.localized = self.no_tf = self.no_ground = self.clamped = 0


def main() -> None:
    spin(DetectionLocalizer)


if __name__ == "__main__":
    main()
