#!/usr/bin/env python3
"""Turn image-space detections into ground positions.

For each detection this casts a ray through its box anchor, intersects the
ground plane, and reports where the object stands.

Every transform lookup uses the detection stamp, which is the frame time
DeepStream reported, not the arrival time. Detections arrive 60 to 90 ms
after capture, and a moving drone or a slewing gimbal makes "where is the
camera now" the wrong question. When no transform exists at the frame time,
the detection is dropped: a silently wrong position is worse than a missing
one.

One node runs for each camera, in its own namespace. The two cameras answer
different questions, so their topics are never merged.

Publishes, under /perception/<camera>/
    detections_3d   vision_msgs/Detection3DArray with covariance
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from sim_bridge.projection import (intersect_ground, intrinsics_ready,
                                   quat_rotate, ray_in_optical, slant_range)

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
PERSON_HEIGHT = 1.7
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
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
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
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.warned_output = False

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True is required. On this node's executor, a lookup that
        # waits for a transform would block the callback that delivers it.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        self.rel_alt: float | None = None

        # Sensor QoS throughout: these topics are best effort, and a reliable
        # subscription to a best effort publisher receives nothing.
        detections_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        if self.get_parameter("use_rel_alt").value:
            self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray,
                                 self.get_parameter("detections_topic").value,
                                 self._on_detections, detections_qos)

        self.det3d_pub = self.create_publisher(Detection3DArray, "detections_3d", 10)

        self.localized = self.no_tf = self.no_ground = self.clamped = 0
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f"localizing {self.camera} into {self.reference} from "
            f"{self.optical}, anchor={self.anchor}")

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

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
        ground_z = self.ground_z if self.rel_alt is None else t.z - self.rel_alt

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
            ground_point = intersect_ground(origin, direction, ground_z, MAX_RANGE)
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
            covariance = [0.0] * 36
            covariance[0] = COVARIANCE_DIAGONAL[0] + range_variance    # x
            covariance[7] = COVARIANCE_DIAGONAL[1] + range_variance    # y
            covariance[14] = COVARIANCE_DIAGONAL[2]     # z, fixed by the plane
            covariance[21] = COVARIANCE_DIAGONAL[3]
            covariance[28] = COVARIANCE_DIAGONAL[4]
            covariance[35] = COVARIANCE_DIAGONAL[5]
            hypothesis.pose.covariance = covariance

            d3 = Detection3D()
            d3.header = out.header
            d3.id = det.id
            d3.results.append(hypothesis)
            d3.bbox = BoundingBox3D()
            d3.bbox.center = pose
            d3.bbox.size.x = 2.0 * math.sqrt(covariance[0])
            d3.bbox.size.y = 2.0 * math.sqrt(covariance[7])
            d3.bbox.size.z = PERSON_HEIGHT
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
    rclpy.init()
    node = DetectionLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
