#!/usr/bin/env python3
"""Lay the live camera image flat on the ground plane, in 3D.

Foxglove's 3D panel cannot texture a surface with an image, so this projects
a subsampled grid of pixels onto the ground plane and publishes them as a
colored PointCloud2. The result is the camera view lying on the ground, next
to the detections and the ground truth.

It is the same geometry the localizer performs, applied to a grid of pixels.
When the imagery lines up with the ground truth bubbles, the localization is
correct. When it does not, the picture shows which way it is out.

The cloud stops at GROUND_VIEW_MAX_DISTANCE_M, the same limit that
truncates the footprint, so the imagery fills the outline that frames it.
Rays are cast for the whole grid, but colors are gathered and points packed
only for pixels that land inside the limit, so the cost tracks what is
displayed: a camera near the horizon pays for the near ground it shows,
not for the sky.

The frame arrives as the camera's JPEG stream, the same encoding the image
panels read, so no raw image topic has to exist. The rate limit, the
subscriber check, and the intrinsics check all run before the JPEG is
decoded, so a frame that will not be displayed costs almost nothing. The
decode runs through GStreamer, the library the camera node encodes with.
The rays through the grid depend only on the grid and the intrinsics, so
they are computed once and reused: a steady stream pays only for one decode
per period, the rotation, the intersection, and the packing.

`size` is the resolution the frame is sampled down to, and it bounds the
cost for both cameras. 640x360 is at most 230,400 points and 3.7 MB per
message, which already reads as a picture.

Subscribes
    <ns>/image_raw/compressed, <ns>/camera_info
Publishes
    <ns>/ground_projection    sensor_msgs/PointCloud2 in the reference frame
"""

from __future__ import annotations

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (HistoryPolicy, QoSProfile, ReliabilityPolicy,  # noqa: E402
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2, PointField  # noqa: E402
from tf2_ros import Buffer, TransformListener  # noqa: E402

from sim_bridge.geo import GroundPlane  # noqa: E402
from sim_bridge.projection import GROUND_VIEW_MAX_DISTANCE_M, intrinsics_ready

# The point layout, as one 16 byte record. Naming it here means point_step,
# row_step and the packing can never drift apart.
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])

# What `size` accepts. Standard 16:9 resolutions, so the sampled grid keeps
# the aspect ratio of both cameras.
STANDARD_SIZES = ("1920x1080", "1280x720", "960x540", "854x480", "640x360",
                  "480x270", "426x240")

# Keep only the newest frame. The publisher's queue is one deep, and this
# node never wants an older frame than the one it just got.
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)


def parse_size(text: str) -> tuple[int, int] | None:
    """Read "WxH" into a pair. Returns None if it does not read as one."""
    try:
        w, h = (int(part) for part in str(text).lower().split("x", 1))
    except ValueError:
        return None
    return (w, h) if w > 0 and h > 0 else None


class JpegDecoder:
    """One JPEG in, one RGB frame out, through GStreamer. A pipeline that
    errors is rebuilt on the next call, so one bad frame cannot silence the
    ground projection for good."""

    def __init__(self) -> None:
        Gst.init(None)
        self.pipeline = None
        self.src = self.sink = None
        self._build()

    def _build(self) -> None:
        self.close()
        self.pipeline = Gst.parse_launch(
            "appsrc name=src caps=image/jpeg is-live=true format=time "
            "! jpegdec ! videoconvert ! video/x-raw,format=RGB "
            "! appsink name=sink max-buffers=1 drop=true sync=false")
        self.src = self.pipeline.get_by_name("src")
        self.sink = self.pipeline.get_by_name("sink")
        self.pipeline.set_state(Gst.State.PLAYING)

    def _failed(self) -> bool:
        bus = self.pipeline.get_bus()
        return bus.timed_pop_filtered(0, Gst.MessageType.ERROR) is not None

    def decode(self, data) -> tuple | None:
        """The frame as (pixels, width, height, row_step), or None when the
        bytes do not decode. row_step comes from the buffer itself, so a
        converter that pads its rows still indexes correctly."""
        # Discard a frame left by a decode that timed out earlier, so it can
        # never come back paired with this message's stamp.
        self.sink.emit("try-pull-sample", 0)
        push = self.src.emit("push-buffer", Gst.Buffer.new_wrapped(bytes(data)))
        if push != Gst.FlowReturn.OK:
            self._build()
            return None
        sample = self.sink.emit("try-pull-sample", Gst.SECOND // 2)
        if sample is None:
            if self._failed():
                self._build()
            return None
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")
        buf = sample.get_buffer()
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            pixels = np.frombuffer(info.data, dtype=np.uint8).copy()
        finally:
            buf.unmap(info)
        if height <= 0 or width <= 0 or pixels.size < width * height * 3:
            return None
        return pixels, width, height, pixels.size // height

    def close(self) -> None:
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        self.pipeline = None
        self.src = self.sink = None


class ImageGroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("image_ground_projector")

        self.declare_parameter("image_topic", "image_raw/compressed")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("optical_frame", "nadir_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("size", "640x360")
        self.declare_parameter("rate_hz", 2.0)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value

        requested = str(self.get_parameter("size").value)
        self.size = parse_size(requested)
        if self.size is None:
            self.size = parse_size("640x360")
            self.get_logger().warn(
                f"cannot read size {requested!r}. Using 640x360. "
                f"The usual values are {', '.join(STANDARD_SIZES)}.")
        elif requested not in STANDARD_SIZES:
            # Not an error. A non-standard grid projects correctly.
            self.get_logger().info(
                f"size {requested} is not one of {', '.join(STANDARD_SIZES)}")

        # The sampled grid, and the camera size it was built for. Rebuilt only
        # when the camera resolution changes, which it does not in flight.
        self.grid_for: tuple[int, int] | None = None
        self.us = self.vs = None
        # The rays through the grid, cached with it. They depend only on the
        # grid and the intrinsics, so a steady stream reuses them.
        self.rays_for: tuple | None = None
        self.ray_x = self.ray_y = None
        # Declares use_rel_alt and ground_z, and latches the plane at the
        # takeoff altitude. See sim_bridge/geo.py.
        self.ground_plane = GroundPlane(self)
        self.min_period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)

        self.info: CameraInfo | None = None
        self.last_processed = 0.0

        self.decoder = JpegDecoder()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(CompressedImage,
                                 self.get_parameter("image_topic").value,
                                 self._on_image, IMAGE_QOS)

        self.pub = self.create_publisher(PointCloud2, "ground_projection",
                                         qos_profile_sensor_data)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_image(self, msg: CompressedImage) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_processed < self.min_period:
            return
        # The throttle does not advance here, so the first frame after a
        # client subscribes is processed at once.
        if self.pub.get_subscription_count() == 0:
            return
        if not intrinsics_ready(self.info):
            return
        # All the cheap gates passed. Advance the throttle before the work,
        # so a frame with no ground in view or no TF still counts.
        self.last_processed = now

        if "jpeg" not in msg.format.lower():
            return
        decoded = self.decoder.decode(msg.data)
        if decoded is None:
            return
        pixels, width, height, step = decoded

        try:
            # At the image's own timestamp, the same rule the localizer follows,
            # so the imagery lands where the camera was when it was taken.
            tf = self.tf_buffer.lookup_transform(
                self.reference, self.optical,
                rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except Exception:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.reference, self.optical, rclpy.time.Time())
            except Exception:
                return

        t, r = tf.transform.translation, tf.transform.rotation
        origin = (t.x, t.y, t.z)
        rot = (r.x, r.y, r.z, r.w)
        ground_z = self.ground_plane.z(t.z)

        points = self._project(pixels, width, height, step,
                               origin, rot, ground_z)
        if points is None or not len(points):
            return
        count = len(points)

        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = self.reference
        cloud.height = 1
        cloud.width = count
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = POINT_DTYPE.itemsize
        cloud.row_step = POINT_DTYPE.itemsize * count
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self.pub.publish(cloud)

    # -------------------------------------------------------------- the math
    def _sample_grid(self, width: int, height: int):
        """Pixel centers of the sampled grid, as two 1D arrays.

        The grid is the requested size, or the camera's own size when that is
        smaller: sampling a 1280x720 camera onto a 1920x1080 grid would invent
        points that carry no more information than the pixels behind them.
        """
        if self.grid_for == (width, height):
            return self.us, self.vs
        out_w = min(self.size[0], width)
        out_h = min(self.size[1], height)
        # Nearest source pixel for each output pixel, evenly spread. Integer
        # ratios (1920 -> 640, 1280 -> 640) land exactly on every third or
        # second column, which is what a plain step would have done.
        self.us = ((np.arange(out_w) + 0.5) * (width / out_w)).astype(np.int32)
        self.vs = ((np.arange(out_h) + 0.5) * (height / out_h)).astype(np.int32)
        self.grid_for = (width, height)
        self.get_logger().info(
            f"projecting {width}x{height} as {out_w}x{out_h}, up to "
            f"{out_w * out_h} points, {out_w * out_h * POINT_DTYPE.itemsize / 1e6:.1f} MB")
        return self.us, self.vs

    def _project(self, pixels, width: int, height: int, step: int,
                 origin, rot, ground_z):
        """Sampled pixels that land on the ground within the view limit.

        The same geometry as sim_bridge.projection, written over whole
        arrays. The mask comes first, so colors are gathered and points
        packed only for the pixels that will be displayed.
        """
        us, vs = self._sample_grid(width, height)

        k = self.info.k
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        if fx == 0.0 or fy == 0.0:
            return None

        # Rays through the grid, in the optical frame. Unnormalized: neither
        # the plane intersection nor the horizontal cut needs unit length.
        # Rebuilt only when the grid or the intrinsics change.
        rays_key = (self.grid_for, fx, cx, fy, cy)
        if self.rays_for != rays_key:
            x = (us[None, :].astype(np.float64) - cx) / fx
            y = (vs[:, None].astype(np.float64) - cy) / fy
            self.ray_x = np.broadcast_to(x, (len(vs), len(us)))
            self.ray_y = np.broadcast_to(y, (len(vs), len(us)))
            self.rays_for = rays_key
        x, y = self.ray_x, self.ray_y

        # quat_rotate over the grid, with d = (x, y, 1):
        # t = 2*(qv x d); d' = d + qw*t + qv x t
        qx, qy, qz, qw = rot
        tx = 2.0 * (qy - qz * y)
        ty = 2.0 * (qz * x - qx)
        tz = 2.0 * (qx * y - qy * x)
        wx = x + qw * tx + (qy * tz - qz * ty)
        wy = y + qw * ty + (qz * tx - qx * tz)
        wz = 1.0 + qw * tz + (qx * ty - qy * tx)

        # Ground intersection and the footprint limit, over the grid.
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ground_z - origin[2]) / wz
            horizontal = t * np.hypot(wx, wy)
        good = ((np.abs(wz) >= 1e-9) & (t > 0.0)
                & (horizontal <= GROUND_VIEW_MAX_DISTANCE_M))
        if not good.any():
            return None

        if pixels.size < height * step:
            return None
        # Index through the row step rather than width*3: a row can carry
        # padding.
        frame = pixels[:height * step].reshape(height, step)
        row_index, col_index = np.nonzero(good)
        rows = vs[row_index]
        cols = us[col_index] * 3
        red = frame[rows, cols].astype(np.uint32)
        green = frame[rows, cols + 1].astype(np.uint32)
        blue = frame[rows, cols + 2].astype(np.uint32)

        t = t[good]
        out = np.empty(t.size, dtype=POINT_DTYPE)
        out["x"] = origin[0] + t * wx[good]
        out["y"] = origin[1] + t * wy[good]
        out["z"] = ground_z  # every kept point lies on the ground plane
        out["rgb"] = (red << 16) | (green << 8) | blue
        return out


def main() -> None:
    rclpy.init()
    node = ImageGroundProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.decoder.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
