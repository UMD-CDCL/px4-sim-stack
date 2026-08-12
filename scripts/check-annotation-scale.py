#!/usr/bin/env python3
"""Check that detection boxes land where the objects are.

Run it inside the ros container:

    ./px4sim shell ros
    python3 /stacks/baseline/../../scripts/check-annotation-scale.py

or from the host:

    docker compose exec ros python3 /scripts/check-annotation-scale.py

For each camera it grabs one frame and the detections that go with it, draws
the boxes as reported, and writes a JPEG you can look at. It also reports the
single scale factor that would make the boxes fit the image, which is the
number to put in DS_COORD_OVERRIDES.

Why this exists
---------------
DeepStream reports boxes in its own coordinate space, and that space is not
always the image size. When it disagrees, every box is still a plausible
looking box, just on the wrong part of the image, and nothing logs an error.
The two sources in one pipeline have also been seen disagreeing with each other
at the same time, so the check is per camera.

Reading the result
------------------
    scale 1.00        the boxes are in image coordinates. Nothing to do.
    scale about 1.50  the boxes are in 1280x720 and the image is 1920x1080.
                      Set DS_COORD_OVERRIDES=<camera>=1280x720 in .env and
                      restart the ros service.

The scale is inferred from the ratio of image size to the largest box extent
seen, so it is a hint rather than a measurement. The JPEG is the real answer:
if the box sits on the object, the camera is correct.
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

DETECTION_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=10)
CANDIDATES = (1.0, 1.5, 2.0, 0.6667, 0.5)


def grab(node: Node, camera: str, seconds: float):
    state: dict = {}
    node.create_subscription(
        Image, f"/camera/{camera}/image_raw",
        lambda m: state.__setitem__("img", m), qos_profile_sensor_data)
    node.create_subscription(
        Detection2DArray, f"/perception/{camera}/detections",
        lambda m: state.__setitem__("det", m) if m.detections else None,
        DETECTION_QOS)
    for _ in range(int(seconds / 0.05)):
        rclpy.spin_once(node, timeout_sec=0.05)
        if "img" in state and "det" in state:
            break
    return state.get("img"), state.get("det")


def check(node: Node, camera: str, seconds: float, out_dir: str) -> None:
    img, det = grab(node, camera, seconds)
    if img is None:
        print(f"  {camera}: no image on /camera/{camera}/image_raw")
        return
    if det is None:
        print(f"  {camera}: image is {img.width}x{img.height}, but no detections "
              f"arrived. Point the camera at a target.")
        return

    frame = np.frombuffer(bytes(img.data), np.uint8).reshape(img.height, img.width, 3)
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR).copy()

    right = max(d.bbox.center.position.x + d.bbox.size_x / 2 for d in det.detections)
    lower = max(d.bbox.center.position.y + d.bbox.size_y / 2 for d in det.detections)
    # The boxes must fit inside the image. Of the usual ratios, take the
    # largest that still fits, which is the space they are most likely in.
    fits = [s for s in sorted(CANDIDATES)
            if right * s <= img.width + 1 and lower * s <= img.height + 1]
    guess = max(fits) if fits else 1.0

    for d in det.detections:
        b = d.bbox
        for scale, colour, tag in ((1.0, (0, 0, 255), "as reported"),
                                   (guess, (0, 255, 0), f"x{guess:.2f}")):
            if scale != 1.0 or guess == 1.0:
                cx, cy = b.center.position.x * scale, b.center.position.y * scale
                w, h = b.size_x * scale, b.size_y * scale
                p1 = (int(cx - w / 2), int(cy - h / 2))
                p2 = (int(cx + w / 2), int(cy + h / 2))
                cv2.rectangle(frame, p1, p2, colour, 3)
                cv2.putText(frame, tag, (p1[0], max(p1[1] - 8, 16)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

    path = f"{out_dir}/annotation-scale-{camera}.jpg"
    cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    verdict = ("boxes are in image coordinates" if abs(guess - 1.0) < 0.01
               else f"boxes look like {int(img.width / guess)}x{int(img.height / guess)}, "
                    f"so set DS_COORD_OVERRIDES={camera}="
                    f"{int(img.width / guess)}x{int(img.height / guess)}")
    print(f"  {camera}: image {img.width}x{img.height}, "
          f"{len(det.detections)} detections, scale {guess:.2f}")
    print(f"           {verdict}")
    print(f"           wrote {path}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cameras", default="nadir,gimbal")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="how long to wait for a frame and a detection")
    ap.add_argument("--out-dir", default="/tmp")
    args = ap.parse_args()

    rclpy.init()
    node = Node("check_annotation_scale")
    try:
        for camera in [c.strip() for c in args.cameras.split(",") if c.strip()]:
            check(node, camera, args.seconds, args.out_dir)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
