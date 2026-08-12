#!/usr/bin/env python3
"""Put a detection at a pixel you choose, and see where the stack puts it.

Run it inside the ros container:

    ./px4sim shell ros
    python3 /scripts/check-projection.py

It publishes a detection payload of the shape DeepStream emits, on the same
MQTT topic, with a box at a pixel this script picks. Then it reads back what
the stack made of it and compares that against the answer worked out here from
CameraInfo and the transform tree.

Why this exists
---------------
When boxes do not sit on the target, two things can be wrong and they look
identical from a Foxglove panel:

    the ROS half     detections_bridge, the annotator and the localizer
    the DeepStream half   the pixel coordinates DeepStream reports

This script removes the second one. The box it sends is at a known pixel by
construction, so anything that comes back in the wrong place is the ROS half.
If everything here agrees and the live boxes still miss the target, the
coordinates DeepStream reports are the thing to measure, and
scripts/check-annotation-scale.py draws those against a real frame.

It needs no GPU and no detector. The perception service does not have to be
running, and it is better if it is not: a real detection arriving in the middle
of this makes the output harder to read.

Reading the result
------------------
    pixel round trip   the box that comes back out on /perception/<cam>/detections
                       against the box that went in. Any difference is a scale
                       or an offset applied between the two.
    ground position    where the localizer put it, against this script's own
                       projection of the same pixel. Any difference is in the
                       localizer's geometry, its frame, or its anchor.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
import time

import paho.mqtt.client as mqtt
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray, Detection3DArray

sys.path.insert(0, "/stacks/baseline/sim_bridge")
from sim_bridge.projection import intersect_ground, quat_rotate, ray_in_optical

DETECTION_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=10)
# Matches stack.launch.py: straight down projects from the box centre, oblique
# from the bottom edge where the feet meet the ground.
ANCHOR = {"nadir": "centre", "gimbal": "bottom"}
TOLERANCE_PX = 0.5
TOLERANCE_M = 0.5


def payload(sensor: str, left: float, top: float, right: float, bottom: float) -> str:
    now = datetime.datetime.now(datetime.timezone.utc)
    return json.dumps({
        "version": "4.0",
        "id": "0",
        "@timestamp": now.strftime("%Y-%m-%dT%H:%M:%S.") + f"{now.microsecond // 1000:03d}Z",
        "sensorId": sensor,
        "objects": [f"9001|{left}|{top}|{right}|{bottom}|person"],
    })


class Probe(Node):
    def __init__(self, camera: str) -> None:
        super().__init__("check_projection")
        self.camera = camera
        self.info: CameraInfo | None = None
        self.two_d: Detection2DArray | None = None
        self.three_d: Detection3DArray | None = None
        self.rel_alt: float | None = None

        self.create_subscription(CameraInfo, f"/camera/{camera}/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, f"/perception/{camera}/detections",
                                 self._on_2d, DETECTION_QOS)
        self.create_subscription(Detection3DArray, f"/perception/{camera}/detections_3d",
                                 self._on_3d, DETECTION_QOS)
        from std_msgs.msg import Float64
        self.create_subscription(Float64, "/mavros/global_position/rel_alt",
                                 lambda m: setattr(self, "rel_alt", float(m.data)),
                                 qos_profile_sensor_data)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

    def _on_info(self, msg): self.info = msg

    def _on_2d(self, msg):
        if any(d.id == "9001" for d in msg.detections):
            self.two_d = msg

    def _on_3d(self, msg):
        if msg.detections:
            self.three_d = msg

    def wait(self, seconds: float, done) -> None:
        end = time.time() + seconds
        while time.time() < end and not done():
            rclpy.spin_once(self, timeout_sec=0.05)


def check(node: Probe, camera: str, host: str, port: int, topic: str,
          frac_x: float, frac_y: float, box_px: float) -> bool:
    node.wait(20.0, lambda: node.info is not None)
    if node.info is None:
        print(f"  {camera}: no CameraInfo on /camera/{camera}/camera_info")
        return False

    width, height = node.info.width, node.info.height
    cx_px, cy_px = width * frac_x, height * frac_y
    left, top = cx_px - box_px / 2, cy_px - box_px / 2
    right, bottom = cx_px + box_px / 2, cy_px + box_px / 2

    # The listener starts empty, so give the tree time to arrive before
    # deciding a frame is missing.
    optical = f"{camera}_camera_optical_frame"
    node.wait(10.0, lambda: node.tf_buffer.can_transform(
        "map", optical, rclpy.time.Time()))
    try:
        tf = node.tf_buffer.lookup_transform("map", optical, rclpy.time.Time())
    except Exception as exc:
        print(f"  {camera}: no transform map -> {optical} ({type(exc).__name__})")
        return False
    t, r = tf.transform.translation, tf.transform.rotation
    origin = (t.x, t.y, t.z)
    rot = (r.x, r.y, r.z, r.w)
    ground_z = 0.0 if node.rel_alt is None else t.z - node.rel_alt

    anchor = ANCHOR.get(camera, "bottom")
    anchor_v = cy_px if anchor == "centre" else bottom
    expected = intersect_ground(
        origin, quat_rotate(rot, ray_in_optical(cx_px, anchor_v, node.info.k)),
        ground_z, 5000.0)

    node.two_d = node.three_d = None
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2) \
        if hasattr(mqtt, "CallbackAPIVersion") else mqtt.Client()
    client.connect(host, port, keepalive=10)
    client.loop_start()
    for _ in range(12):
        client.publish(topic, payload(camera, left, top, right, bottom), qos=0)
        node.wait(0.5, lambda: node.two_d is not None and node.three_d is not None)
        if node.two_d is not None and node.three_d is not None:
            break
    client.loop_stop()
    client.disconnect()

    print(f"  {camera}: image {width}x{height}, camera at "
          f"({t.x:.1f}, {t.y:.1f}, {t.z:.1f}), ground z {ground_z:.2f}, anchor {anchor}")

    if node.two_d is None:
        print(f"    pixel round trip  NO reply on /perception/{camera}/detections")
        return False
    got = next(d for d in node.two_d.detections if d.id == "9001").bbox
    dx = got.center.position.x - cx_px
    dy = got.center.position.y - cy_px
    dw = got.size_x - box_px
    ok_px = max(abs(dx), abs(dy), abs(dw)) <= TOLERANCE_PX
    print(f"    pixel round trip  sent centre ({cx_px:.1f}, {cy_px:.1f}) size {box_px:.0f}, "
          f"got ({got.center.position.x:.1f}, {got.center.position.y:.1f}) "
          f"size {got.size_x:.0f}  {'OK' if ok_px else 'OFF by '}"
          f"{'' if ok_px else f'({dx:+.1f}, {dy:+.1f}) px, size {dw:+.1f}'}")

    if expected is None:
        print("    ground position   this pixel sees no ground from here, so nothing to compare")
        return ok_px
    if node.three_d is None:
        print(f"    ground position   NO reply on /perception/{camera}/detections_3d")
        return False
    pos = node.three_d.detections[0].bbox.center.position
    err = math.dist((pos.x, pos.y), (expected[0], expected[1]))
    ok_m = err <= TOLERANCE_M
    print(f"    ground position   expected ({expected[0]:+.2f}, {expected[1]:+.2f}), "
          f"got ({pos.x:+.2f}, {pos.y:+.2f}), off by {err:.2f} m  "
          f"{'OK' if ok_m else 'FAIL'}")
    return ok_px and ok_m


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", default="nadir,gimbal")
    ap.add_argument("--host", default="message-bus")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--topic", default="perception/detections")
    ap.add_argument("--at", default="0.5,0.5",
                    help="where to put the box, as fractions of width and height")
    ap.add_argument("--box", type=float, default=80.0, help="box size in pixels")
    args = ap.parse_args()

    frac_x, frac_y = (float(v) for v in args.at.split(","))
    rclpy.init()
    ok = True
    print(f"injecting a detection at ({frac_x:.2f}, {frac_y:.2f}) of each image\n")
    try:
        for camera in [c.strip() for c in args.cameras.split(",") if c.strip()]:
            node = Probe(camera)
            try:
                ok &= check(node, camera, args.host, args.port, args.topic,
                            frac_x, frac_y, args.box)
            finally:
                node.destroy_node()
            print()
    finally:
        rclpy.try_shutdown()
    print("the ROS half is consistent" if ok else "something in the ROS half is off")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
