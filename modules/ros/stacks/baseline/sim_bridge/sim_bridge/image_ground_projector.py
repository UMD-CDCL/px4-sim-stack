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

Cost is controlled by `step`. At step 8 a 1280x720 frame becomes 160x90, about
14 thousand points, which is cheap to publish at a couple of hertz.

Subscribes
    <ns>/image_raw, <ns>/camera_info
Publishes
    <ns>/ground_projection    sensor_msgs/PointCloud2 in the reference frame
"""

from __future__ import annotations

import struct

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener

from sim_bridge.projection import intersect_ground, quat_rotate, ray_in_optical


class ImageGroundProjector(Node):
    def __init__(self) -> None:
        super().__init__("image_ground_projector")

        self.declare_parameter("image_topic", "image_raw")
        self.declare_parameter("camera_info_topic", "camera_info")
        self.declare_parameter("optical_frame", "nadir_camera_optical_frame")
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("use_rel_alt", True)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("step", 8)
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("max_range", 2000.0)

        self.optical = self.get_parameter("optical_frame").value
        self.reference = self.get_parameter("reference_frame").value
        self.step = max(1, int(self.get_parameter("step").value))
        self.max_range = float(self.get_parameter("max_range").value)
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

        data = bytearray()
        count = 0
        step = self.step
        k = self.info.k
        row = msg.step
        for v in range(0, msg.height, step):
            base = v * row
            for u in range(0, msg.width, step):
                hit = intersect_ground(
                    origin, quat_rotate(rot, ray_in_optical(u, v, k)),
                    ground_z, self.max_range)
                if hit is None:
                    continue
                i = base + u * 3
                rgb = (msg.data[i] << 16) | (msg.data[i + 1] << 8) | msg.data[i + 2]
                data += struct.pack("<fffI", hit[0], hit[1], hit[2], rgb)
                count += 1

        if not count:
            return

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
        cloud.point_step = 16
        cloud.row_step = 16 * count
        cloud.is_dense = True
        cloud.data = bytes(data)
        self.pub.publish(cloud)
        self.last_publish = now


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
