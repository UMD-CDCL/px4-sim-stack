#!/usr/bin/env python3
"""Ask one vehicle for a framing, and wait until it says it is on it.

Runs inside that vehicle's companion container, on its ROS domain:
    ./px4sim zoom 11 wide

The request goes out on zoom/preset_cmd. A Foxglove button publishes the same
topic, so this exercises the operator's path rather than a second one built for
the front door. That topic is fire and forget by design, because on the
aircraft it crosses a radio link, so the vehicle has to supply the answer.
zoom/current_preset names the framing the lens reached. camera/camera_info
carries the calibration of the picture it now delivers. Nothing here reports
success until both agree with the request.

The publisher stays alive until they do. A publisher that sends one sample and
then exits loses that sample before the reliable handshake finishes. It prints
success while the lens never moves, and every test after it runs at a framing
nobody chose.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

RELIABLE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
LATCHED_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

SETTLE_POLL_S = 0.2
# How long to wait for the zoom node to be listening. Nothing goes out before
# it is: a sample sent to nobody is the failure this tool exists to stop.
SUBSCRIBER_WAIT_S = 10.0
# How long the vehicle's latched framing and calibration have to arrive before
# the request goes out. They are what the answer is measured against, and they
# come only once this tool's own subscriptions have found their publishers.
LATCHED_WAIT_S = 5.0
# How often to ask again while the lens has not arrived. A recall is a
# position, so asking twice asks for the same place twice.
REASK_S = 2.0
# How long the calibration may lag the framing name. The zoom node publishes
# the calibration first and the name second, so this only covers delivery.
CALIBRATION_WAIT_S = 3.0
# How far the published focal length may sit from the framing's own before the
# calibration and the lens are called different pictures.
FOCAL_TOLERANCE = 0.01


def focal_of(width: int, hfov_deg: float) -> float:
    """The focal length in pixels that sees ``hfov_deg`` across ``width``."""
    return (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)


def hfov_of(width: int, focal: float) -> float:
    """The field of view in degrees a calibration describes."""
    return math.degrees(2.0 * math.atan((width / 2.0) / focal))


def recall(node: Node, args) -> int:
    namespace = f"/uas{args.number}"
    reported = {}
    node.create_subscription(
        String, f"{namespace}/zoom/current_preset",
        lambda msg: reported.__setitem__("preset", msg.data), LATCHED_QOS)
    node.create_subscription(
        CameraInfo, f"{namespace}/camera/camera_info",
        lambda msg: reported.__setitem__("info", msg), LATCHED_QOS)
    ask = node.create_publisher(
        String, f"{namespace}/zoom/preset_cmd", RELIABLE_QOS)

    def wait_until(ready, deadline_s: float, what=None) -> bool:
        """Poll for a condition. Naming ``what`` also complains when it never holds."""
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=SETTLE_POLL_S)
            if ready():
                return True
        if what:
            print(f"gave up waiting for {what} after {deadline_s:.0f}s",
                  file=sys.stderr)
        return False

    def calibrated() -> bool:
        """Does the published calibration describe the framing that was asked for?"""
        info = reported.get("info")
        if info is None or info.k[0] <= 0.0 or args.hfov <= 0.0:
            return False
        return abs(info.k[0] / focal_of(info.width, args.hfov) - 1.0) <= FOCAL_TOLERANCE

    if not wait_until(lambda: ask.get_subscription_count() > 0,
                      SUBSCRIBER_WAIT_S,
                      f"the zoom node to listen on {namespace}/zoom/preset_cmd"):
        print(f"uas{args.number} has nobody taking framing commands. Is its "
              f"zoom node running? ./px4sim logs onboard{args.number}",
              file=sys.stderr)
        return 1

    # What the vehicle says before it is asked for anything. Both are latched,
    # so they arrive on their own; a lens that has not reached a framing this
    # power cycle publishes no name at all, and that is an answer rather than a
    # failure, so this waits without insisting.
    wait_until(lambda: "preset" in reported and "info" in reported,
               LATCHED_WAIT_S)
    was_at = reported.get("preset")
    request = String(data=args.preset)
    ask.publish(request)
    began = time.monotonic()
    ask_again_at = began + REASK_S
    arrived = False
    while time.monotonic() - began < args.deadline:
        rclpy.spin_once(node, timeout_sec=SETTLE_POLL_S)
        if reported.get("preset") == args.preset:
            arrived = True
            break
        if time.monotonic() >= ask_again_at:
            ask.publish(request)
            ask_again_at = time.monotonic() + REASK_S
    travelled = time.monotonic() - began

    if not arrived:
        print(f"uas{args.number} did not take the {args.preset} framing: "
              f"{namespace}/zoom/current_preset still says "
              f"{reported.get('preset') or 'nothing'} {args.deadline:.0f}s "
              f"after asking. Read why: ./px4sim logs onboard{args.number}",
              file=sys.stderr)
        return 1

    if args.hfov > 0.0:
        wait_until(calibrated, CALIBRATION_WAIT_S)

    info = reported.get("info")
    if info is None or info.k[0] <= 0.0:
        print(f"uas{args.number} says it is at {args.preset}, but publishes no "
              f"calibration on {namespace}/camera/camera_info", file=sys.stderr)
        return 1

    if was_at == args.preset:
        print(f"uas{args.number} was already at the {args.preset} framing")
    elif was_at is None:
        # The vehicle named no framing before this ran, so the elapsed time
        # measures discovery as much as travel and is not worth reporting.
        print(f"uas{args.number} is at the {args.preset} framing")
    else:
        print(f"uas{args.number} is at the {args.preset} framing, "
              f"{travelled:.1f}s after asking (it was at {was_at})")
    print(f"  {namespace}/zoom/current_preset  {reported['preset']}")
    print(f"  {namespace}/camera/camera_info   fx {info.k[0]:.2f} at "
          f"{info.width} px wide = {hfov_of(info.width, info.k[0]):.2f} degrees")

    if args.hfov > 0.0 and not calibrated():
        print(f"but the {args.preset} framing sees {args.hfov:.2f} degrees, so "
              f"the calibration and the lens describe different pictures",
              file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("number", type=int, help="vehicle number")
    parser.add_argument("preset", help="the framing to recall, e.g. wide")
    parser.add_argument("--hfov", type=float, default=0.0,
                        help="what that framing sees, in degrees, from "
                             "scripts/zoom.sh. 0 skips the check.")
    parser.add_argument("--deadline", type=float, default=20.0,
                        help="how long the lens has to arrive")
    args = parser.parse_args()

    rclpy.init()
    node = Node("verify_zoom")
    try:
        return recall(node, args)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
