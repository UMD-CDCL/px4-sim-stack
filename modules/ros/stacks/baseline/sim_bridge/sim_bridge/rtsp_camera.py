#!/usr/bin/env python3
"""Publish an RTSP stream as a ROS 2 image topic.

This node is the video half of the drone interface. It knows one URL, not
that Gazebo exists, and it behaves the same against a real camera.

The stamp is arrival time minus the jitter buffer, plus time_offset. A frame
reaches this node well after capture, and stamping with the arrival time
would put the image later than the pose it belongs to.

Only the JPEG encoding is published. Raw frames are too large for the
foxglove_bridge websocket, and every consumer that wants pixels, the ground
projector included, decodes the JPEG at its own rate. The topic is published
only while something subscribes to it; the encode itself keeps running so a
new subscriber gets the next frame.

Parameters
    url            RTSP address to read
    frame_id       frame_id on the published messages
    latency_ms     jitter buffer depth. Lower is fresher and less forgiving.
    protocols      tcp or udp. TCP does not lose packets.
    decoder        GStreamer decoder element. avdec_h264 is software and
                   works everywhere. nvh264dec needs an NVIDIA GPU and the
                   nvcodec plugin.
    hfov           horizontal field of view in radians, for the CameraInfo
                   pinhole model
    time_offset    extra seconds added to the image stamp, for calibration

Topics
    <ns>/image_raw/compressed  sensor_msgs/CompressedImage, jpeg
    <ns>/camera_info           sensor_msgs/CameraInfo
"""

from __future__ import annotations

import math
import threading

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import CameraInfo, CompressedImage  # noqa: E402


# ------------------------------------------------------------------- tunables
JPEG_QUALITY = 75

SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)


class RtspCamera(Node):
    def __init__(self) -> None:
        super().__init__("rtsp_camera")

        self.declare_parameter("url", "rtsp://video-router:8554/gimbal")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("latency_ms", 100)
        self.declare_parameter("protocols", "tcp")
        self.declare_parameter("decoder", "avdec_h264")
        self.declare_parameter("hfov", 2.0)
        self.declare_parameter("time_offset", 0.0)

        self.url = self.get_parameter("url").value
        self.frame_id = self.get_parameter("frame_id").value
        self.hfov = float(self.get_parameter("hfov").value)
        # The jitter buffer is the largest known part of the delay, and it is
        # the one part we can name exactly.
        self.stamp_shift = -(int(self.get_parameter("latency_ms").value) / 1000.0) \
            + float(self.get_parameter("time_offset").value)

        self.jpeg_pub = self.create_publisher(
            CompressedImage, "image_raw/compressed", SENSOR_QOS)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", SENSOR_QOS)

        Gst.init(None)
        self.pipeline = None
        self.jpegsink = None
        self.info_cache: dict[tuple[int, int], CameraInfo] = {}
        self.frames = 0
        self.stop = threading.Event()

        self.get_logger().info(f"reading {self.url}")
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

        # One line a minute, so a dead stream is visible in the logs.
        self.create_timer(60.0, self._report)

    # ------------------------------------------------------------------ setup
    def _build(self) -> bool:
        latency = int(self.get_parameter("latency_ms").value)
        protocols = self.get_parameter("protocols").value
        decoder = self.get_parameter("decoder").value

        # One decode, one sink. jpegenc reads the decoder's 4:2:0 output
        # directly; the videoconvert is passthrough for I420 and keeps
        # negotiation working when a decoder outputs NV12. Encoding the JPEG
        # here rather than in a separate republish node keeps it off the
        # Python side entirely.
        desc = (
            f"rtspsrc location={self.url} latency={latency} protocols={protocols} "
            f"drop-on-latency=true "
            f"! rtph264depay ! h264parse ! {decoder} "
            f"! queue max-size-buffers=2 leaky=downstream ! videoconvert "
            f"! jpegenc quality={JPEG_QUALITY} "
            f"! appsink name=jpeg max-buffers=1 drop=true sync=false"
        )
        try:
            self.pipeline = Gst.parse_launch(desc)
        except Exception as exc:  # noqa: BLE001 - Gst raises a bare GError
            self.get_logger().error(f"cannot build the pipeline: {exc}")
            return False

        self.jpegsink = self.pipeline.get_by_name("jpeg")
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("cannot start the pipeline")
            self._teardown()
            return False
        return True

    def _teardown(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.jpegsink = None
        self.info_cache.clear()

    # ------------------------------------------------------------------- loop
    def _run(self) -> None:
        while not self.stop.is_set():
            if self.pipeline is None and not self._build():
                self.stop.wait(3.0)
                continue

            sample = self.jpegsink.emit("try-pull-sample", Gst.SECOND)
            if sample is None:
                if self._pipeline_failed():
                    self.get_logger().warn("stream dropped, reconnecting")
                    self._teardown()
                    self.stop.wait(2.0)
                continue

            self._publish_jpeg(sample)

    def _pipeline_failed(self) -> bool:
        bus = self.pipeline.get_bus()
        msg = bus.timed_pop_filtered(0, Gst.MessageType.ERROR | Gst.MessageType.EOS)
        if msg is None:
            return False
        if msg.type == Gst.MessageType.ERROR:
            err, _ = msg.parse_error()
            self.get_logger().warn(f"gstreamer: {err.message}")
        return True

    # ---------------------------------------------------------------- publish
    def _stamp(self):
        return (self.get_clock().now()
                + rclpy.duration.Duration(seconds=self.stamp_shift)).to_msg()

    def _publish_jpeg(self, sample) -> None:
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")
        stamp = self._stamp()

        self.info_pub.publish(self._camera_info(stamp, width, height))
        self.frames += 1

        if self.jpeg_pub.get_subscription_count() == 0:
            return
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return
        try:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.format = "jpeg"
            msg.data = bytes(info.data)
            self.jpeg_pub.publish(msg)
        finally:
            buf.unmap(info)

    def _camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        info = self.info_cache.get((width, height))
        if info is None:
            # A pinhole model from the field of view. It is not a calibration.
            # It is close enough to project a detection into a bearing, and it
            # is exactly right for the simulated camera, which is an ideal
            # pinhole.
            fx = (width / 2.0) / math.tan(self.hfov / 2.0)
            fy = fx
            cx = width / 2.0
            cy = height / 2.0

            info = CameraInfo()
            info.header.frame_id = self.frame_id
            info.width = width
            info.height = height
            info.distortion_model = "plumb_bob"
            info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_cache[(width, height)] = info
        info.header.stamp = stamp
        return info

    def _report(self) -> None:
        state = "up" if self.pipeline is not None else "down"
        self.get_logger().info(f"{self.url}: {state}, {self.frames} frames")
        self.frames = 0

    def destroy_node(self) -> bool:
        self.stop.set()
        self._teardown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RtspCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
