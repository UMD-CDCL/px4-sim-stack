#!/usr/bin/env python3
"""Lay the live camera image flat on the ground plane, in 3D.

Foxglove's 3D panel cannot texture a surface with an image, and it has no
satellite basemap. What it does render is a coloured point cloud. So instead of
drawing a textured quad, this projects a subsampled grid of pixels onto the same
ground plane the localizer uses, and publishes them as PointCloud2 with colour.
The result is the camera view lying on the ground, in the right place, at the
right scale, next to the detections and the ground truth.

It is the same geometry the localizer performs, applied to a grid of pixels
rather than to the centre of a box. Anything that misaligns one misaligns the
other by the same amount, which makes this a direct check on the projection:
if the imagery lines up with the ground truth markers, the localization is
correct, and if it does not, the picture shows which way it is out.

Cost is controlled by `size`, a standard resolution the frame is sampled down
to before projection. Both cameras project the same grid, whatever they capture
at, so one number decides the cost and neither camera can quietly become the
expensive one.

Size drives two costs at once, and both have a hard ceiling:

    size        points     message    projection
    1920x1080  2,073,600    33.2 MB      5.24 s
    960x540      518,400     8.3 MB      1.32 s
    640x360      230,400     3.7 MB      0.59 s
    480x270      129,600     2.1 MB      0.33 s

Those projection times are the measured cost of the scalar loop this node used
to run, on this machine, for one camera. At 1920x1080 it never published at
all: the loop ran longer than the interval between frames, so the node held one
core at full load and produced nothing. Two cameras did that on two cores.

The maths below is vectorized, which removes about two orders of magnitude from
that column and is why a full resolution grid is now merely expensive rather
than impossible. The default stays low anyway. A ground projection is a
backdrop to look at, not a measurement, and 640x360 already reads as a picture.

This is an approximation of the thing actually wanted, which is the frame
stretched across its footprint as a texture. Foxglove's 3D panel cannot texture
a surface: it draws markers, meshes referenced by URL, and point clouds, and
none of those take a live image. A dense coloured cloud is the closest thing it
will render, so density and point size are the two knobs that decide how much
it looks like an image.

Subscribes
    <ns>/image_raw, <ns>/camera_info
Publishes
    <ns>/ground_projection    sensor_msgs/PointCloud2 in the reference frame
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

# The point layout, as one 16 byte record. Naming it here means point_step,
# row_step and the packing can never drift apart.
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])

# What `size` accepts. Standard 16:9 resolutions, so the sampled grid keeps the
# aspect ratio of both cameras and the numbers are ones you already recognize.
STANDARD_SIZES = ("1920x1080", "1280x720", "960x540", "854x480", "640x360",
                  "480x270", "426x240")


def parse_size(text: str) -> tuple[int, int] | None:
    """Read "WxH" into a pair. Returns None if it does not read as one."""
    try:
        w, h = (int(part) for part in str(text).lower().split("x", 1))
    except ValueError:
        return None
    return (w, h) if w > 0 and h > 0 else None


class ImageGroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("image_ground_projector")

        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("optical_frame", "nadir_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("size", "640x360")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("max_range", 2000.0)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.max_range = float(self.get_parameter("max_range").value)

        requested = str(self.get_parameter("size").value)
        self.size = parse_size(requested)
        if self.size is None:
            self.size = parse_size("640x360")
            self.get_logger().warn(
                f"cannot read size {requested!r}. Using 640x360. "
                f"The usual values are {', '.join(STANDARD_SIZES)}.")
        elif requested not in STANDARD_SIZES:
            # Not an error. A non-standard grid projects correctly, it just
            # stops matching the table in the module docstring.
            self.get_logger().info(
                f"size {requested} is not one of {', '.join(STANDARD_SIZES)}")

        # The sampled grid, and the camera size it was built for. Rebuilt only
        # when the camera resolution changes, which it does not in flight.
        self.grid_for: tuple[int, int] | None = None
        self.us = self.vs = None
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.min_period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)

        self.info: CameraInfo | None = None
        self.rel_alt: float | None = None
        self.last_publish = 0.0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.create_subscription(CameraInfo,
                                 self.get_parameter("camera_info_topic").value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, qos_profile_sensor_data)
        if self.get_parameter("use_rel_alt").value:
            self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                     self._on_rel_alt, qos_profile_sensor_data)

        self.pub = self.create_publisher(PointCloud2, "ground_projection",
                                         qos_profile_sensor_data)

    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_rel_alt(self, msg: Float64) -> None:
        self.rel_alt = float(msg.data)

    def _on_image(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_publish < self.min_period:
            return
        if self.info is None or len(self.info.k) != 9 or self.info.k[0] == 0.0:
            return
        if msg.encoding != "rgb8":
            return

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
        ground_z = self.ground_z if self.rel_alt is None else t.z - self.rel_alt

        points = self._project(msg, origin, rot, ground_z)
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
        self.last_publish = now

    # ------------------------------------------------------------- the maths
    def _sample_grid(self, width: int, height: int):
        """Pixel centres of the sampled grid, as two 1D arrays.

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
            f"projecting {width}x{height} as {out_w}x{out_h}, "
            f"{out_w * out_h} points, {out_w * out_h * POINT_DTYPE.itemsize / 1e6:.1f} MB")
        return self.us, self.vs

    def _project(self, msg: Image, origin, rot, ground_z):
        """Every sampled pixel, projected onto the ground plane.

        The same geometry as sim_bridge.projection, written over whole arrays.
        A pixel whose ray misses the plane is dropped, so the result holds only
        points that a camera ray actually reaches.
        """
        us, vs = self._sample_grid(msg.width, msg.height)

        frame = np.frombuffer(msg.data, dtype=np.uint8)
        if frame.size < msg.height * msg.step:
            return None
        # Index through msg.step rather than width*3: a row can carry padding.
        frame = frame[:msg.height * msg.step].reshape(msg.height, msg.step)
        red = frame[np.ix_(vs, us * 3)].astype(np.uint32)
        green = frame[np.ix_(vs, us * 3 + 1)].astype(np.uint32)
        blue = frame[np.ix_(vs, us * 3 + 2)].astype(np.uint32)

        k = self.info.k
        fx, cx, fy, cy = k[0], k[2], k[4], k[5]
        if fx == 0.0 or fy == 0.0:
            return None

        # ray_in_optical, over the grid. The unit normalization is kept because
        # max_range below is a distance along the ray.
        x = (us[None, :].astype(np.float64) - cx) / fx
        y = (vs[:, None].astype(np.float64) - cy) / fy
        x = np.broadcast_to(x, (len(vs), len(us)))
        y = np.broadcast_to(y, (len(vs), len(us)))
        norm = np.sqrt(x * x + y * y + 1.0)
        dx, dy, dz = x / norm, y / norm, 1.0 / norm

        # quat_rotate, over the grid: t = 2*(qv x d); d' = d + qw*t + qv x t
        qx, qy, qz, qw = rot
        tx = 2.0 * (qy * dz - qz * dy)
        ty = 2.0 * (qz * dx - qx * dz)
        tz = 2.0 * (qx * dy - qy * dx)
        wx = dx + qw * tx + (qy * tz - qz * ty)
        wy = dy + qw * ty + (qz * tx - qx * tz)
        wz = dz + qw * tz + (qx * ty - qy * tx)

        # intersect_ground, over the grid.
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ground_z - origin[2]) / wz
        good = (np.abs(wz) >= 1e-9) & (t > 0.0) & (t <= self.max_range)
        if not good.any():
            return None

        t = t[good]
        out = np.empty(t.size, dtype=POINT_DTYPE)
        out["x"] = origin[0] + t * wx[good]
        out["y"] = origin[1] + t * wy[good]
        out["z"] = origin[2] + t * wz[good]
        out["rgb"] = (red[good] << 16) | (green[good] << 8) | blue[good]
        return out


def main() -> None:
    rclpy.init()
    node = ImageGroundProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
