#!/usr/bin/env python3
"""Turn image-space detections into ground positions.

For each detection this casts a ray through the bottom-centre of its box,
intersects the ground plane, and reports where the object stands.

The timestamp is the whole point
--------------------------------
Every transform lookup uses the detection header's stamp, which
`detections_bridge` copies from DeepStream's frame time, taken as the frame
enters the pipeline and before inference runs. It is not the time the message
arrived.

That distinction is the difference between a correct answer and a confident
wrong one. Detections reach ROS 60 to 90 ms after the frame was taken. A drone
at 15 m/s covers about a metre in that time, and a slewing gimbal covers
several degrees, so looking up "where is the camera now" puts the target
somewhere the camera was never pointed. Asking tf2 for the pose *at the frame
time* costs nothing and removes that error, and tf2 interpolates between
telemetry samples to answer it.

If a lookup fails because the buffer does not reach back that far, this drops
the detection rather than falling back to the latest pose. A silently wrong
position is worse than a missing one.

Why the bottom edge of the box
------------------------------
A person's feet touch the ground; their centre does not. Projecting the box
centre puts the target roughly half a body length beyond where it stands, and
the error grows as the camera tilts toward the horizon. `anchor` selects the
behaviour if a different detector suits a different choice.

Covariance
----------
Taken from parameters, as an estimate rather than a derivation. The defaults
say a couple of metres, which is the right order for a pinhole model with no
lens calibration, an assumed flat ground and an attitude estimate from a small
EKF. `range_variance_scale` optionally grows the estimate with slant range,
because a shallow look angle stretches a pixel across much more ground.

Publishes
    /perception/detections_3d   vision_msgs/Detection3DArray, for other nodes
    /perception/targets         geometry_msgs/PoseArray, one pose per target
    /perception/markers         visualization_msgs/MarkerArray, for the 3D view
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Float64
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from sensor_msgs.msg import CameraInfo

from sim_bridge.projection import intersect_ground, quat_rotate, ray_in_optical, slant_range


class DetectionLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("detection_localizer")

        self.declare_parameter("detections_topic", "/perception/detections")
        self.declare_parameter("camera_info_topic", "/camera/gimbal/camera_info")
        self.declare_parameter("optical_frame", "gimbal_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("max_range", 2000.0)
        self.declare_parameter("anchor", "bottom")          # bottom or centre
        self.declare_parameter("tf_timeout", 0.25)
        # The detection pipeline is quick. Measured here, a detection reaches
        # ROS about 16 ms after its frame, while telemetry and therefore the
        # transform tree run about 33 ms behind. So the frame time is often a
        # few tens of milliseconds *newer* than the newest transform, and an
        # exact lookup fails with "extrapolation into the future".
        #
        # Waiting handles most of it. When it does not, clamping the request
        # back to the newest transform is right as long as the gap is small:
        # over 20 ms a drone moves centimetres. Past this bound the answer
        # would be guesswork, so the detection is dropped instead.
        self.declare_parameter("future_tolerance", 0.15)
        # Diagonal of the 6x6 pose covariance, in m^2 and rad^2, as
        # [x, y, z, roll, pitch, yaw]. 4.0 is a two metre standard deviation.
        self.declare_parameter("covariance_diagonal", [4.0, 4.0, 1.0, 0.0, 0.0, 0.0])
        # Extra x and y variance per metre of slant range, squared. 0.0004 adds
        # a two centimetre standard deviation for every metre of range.
        self.declare_parameter("range_variance_scale", 0.0004)
        self.declare_parameter("target_height", 1.7)
        self.declare_parameter("marker_lifetime", 3.0)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.max_range = float(self.get_parameter("max_range").value)
        self.anchor = self.get_parameter("anchor").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
        self.future_tolerance = float(self.get_parameter("future_tolerance").value)
        self.cov_diag = [float(x) for x in self.get_parameter("covariance_diagonal").value]
        self.range_scale = float(self.get_parameter("range_variance_scale").value)
        self.target_height = float(self.get_parameter("target_height").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        # spin_thread=True is not optional here. The listener otherwise
        # shares this node's executor, so a lookup that waits for a
        # transform blocks the very callback that would deliver it, and
        # every timeout expires. Its own thread keeps the buffer filling
        # while a lookup waits.
        self.tf_listener = TransformListener(self.tf_buffer, self,
                                             spin_thread=True)

        self.info: CameraInfo | None = None
        self.rel_alt: float | None = None
        self.ground_z = float(self.get_parameter("ground_z").value)

        # Sensor QoS throughout. CameraInfo, the MAVROS sensor topics and the
        # detection stream are all best effort, and a reliable subscription to a
        # best effort publisher receives nothing.
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

        self.det3d_pub = self.create_publisher(Detection3DArray, "/perception/detections_3d", 10)
        self.pose_pub = self.create_publisher(PoseArray, "/perception/targets", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/perception/markers", 10)

        self.localized = self.no_tf = self.no_ground = self.clamped = 0
        self.create_timer(30.0, self._report)
        self.get_logger().info(
            f"localizing into {self.reference} from {self.optical}, "
            f"anchor={self.anchor}")

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    # ------------------------------------------------------------------- work
    def _on_detections(self, msg: Detection2DArray) -> None:
        if not msg.detections:
            return
        # CameraInfo.k arrives as a numpy array, so a plain truth test on it
        # raises rather than returning False. Check the length and fx.
        if self.info is None or len(self.info.k) != 9 or self.info.k[0] == 0.0:
            return

        stamp = Time.from_msg(msg.header.stamp)
        try:
            # The frame time, not now. See the module docstring.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical, stamp,
                timeout=Duration(seconds=self.tf_timeout))
        except Exception as exc:
            tf = self._clamped_lookup(stamp, exc)
            if tf is None:
                return

        t, r = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rot = (r.x, r.y, r.z, r.w)
        ground_z = self.ground_z if self.rel_alt is None else t.z - self.rel_alt

        out3d = Detection3DArray()
        out3d.header.stamp = msg.header.stamp
        out3d.header.frame_id = self.reference
        poses = PoseArray()
        poses.header = out3d.header
        markers = MarkerArray()

        for i, det in enumerate(msg.detections):
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            if self.anchor == "bottom":
                v += det.bbox.size_y / 2.0

            direction = quat_rotate(rot, ray_in_optical(u, v, self.info.k))
            hit = intersect_ground(origin, direction, ground_z, self.max_range)
            if hit is None:
                self.no_ground += 1
                continue

            rng = slant_range(origin, hit)
            extra = self.range_scale * rng * rng

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, hit)
            pose.orientation.w = 1.0

            hyp = ObjectHypothesisWithPose()
            label = det.results[0].hypothesis.class_id if det.results else "object"
            score = det.results[0].hypothesis.score if det.results else 0.0
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = score
            hyp.pose.pose = pose
            cov = [0.0] * 36
            cov[0] = self.cov_diag[0] + extra          # x
            cov[7] = self.cov_diag[1] + extra          # y
            cov[14] = self.cov_diag[2]                 # z, fixed by the plane
            cov[21], cov[28], cov[35] = self.cov_diag[3], self.cov_diag[4], self.cov_diag[5]
            hyp.pose.covariance = cov

            d3 = Detection3D()
            d3.header = out3d.header
            d3.id = det.id
            d3.results.append(hyp)
            d3.bbox = BoundingBox3D()
            d3.bbox.center = pose
            d3.bbox.size.x = 2.0 * math.sqrt(cov[0])
            d3.bbox.size.y = 2.0 * math.sqrt(cov[7])
            d3.bbox.size.z = self.target_height
            out3d.detections.append(d3)
            poses.poses.append(pose)
            markers.markers.extend(self._markers(i, det.id, label, hit, cov, msg.header.stamp))
            self.localized += 1

        if out3d.detections:
            self.det3d_pub.publish(out3d)
            self.pose_pub.publish(poses)
            self.marker_pub.publish(markers)

    def _clamped_lookup(self, stamp: Time, first_error: Exception):
        """Fall back to the newest transform when the frame time runs ahead.

        Only within future_tolerance, and only forward in time. A lookup that
        failed for any other reason, such as a missing frame, still fails.
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
        if 0.0 <= ahead <= self.future_tolerance:
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

    def _markers(self, idx, track_id, label, hit, cov, stamp):
        life = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()

        def make(ns, mid, kind):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self.reference
            m.ns = ns
            m.id = mid
            m.type = kind
            m.action = Marker.ADD
            m.lifetime = life
            m.pose.orientation.w = 1.0
            return m

        pillar = make("targets", idx, Marker.CYLINDER)
        pillar.pose.position.x, pillar.pose.position.y = float(hit[0]), float(hit[1])
        pillar.pose.position.z = float(hit[2]) + self.target_height / 2.0
        pillar.scale.x = pillar.scale.y = 0.5
        pillar.scale.z = self.target_height
        pillar.color = ColorRGBA(r=0.95, g=0.35, b=0.1, a=0.85)

        # One-sigma ellipse, flattened onto the ground.
        ellipse = make("uncertainty", idx, Marker.CYLINDER)
        ellipse.pose.position.x, ellipse.pose.position.y = float(hit[0]), float(hit[1])
        ellipse.pose.position.z = float(hit[2]) + 0.05
        ellipse.scale.x = 2.0 * math.sqrt(cov[0])
        ellipse.scale.y = 2.0 * math.sqrt(cov[7])
        ellipse.scale.z = 0.05
        ellipse.color = ColorRGBA(r=0.95, g=0.75, b=0.1, a=0.30)

        text = make("labels", idx, Marker.TEXT_VIEW_FACING)
        text.pose.position.x, text.pose.position.y = float(hit[0]), float(hit[1])
        text.pose.position.z = float(hit[2]) + self.target_height + 0.5
        text.scale.z = 0.8
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        text.text = f"{label} {track_id}" if track_id else label

        return [pillar, ellipse, text]

    def _report(self) -> None:
        if self.localized or self.no_tf or self.no_ground:
            self.get_logger().info(
                f"{self.localized} localized ({self.clamped} clamped to the "
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
