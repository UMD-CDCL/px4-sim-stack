#!/usr/bin/env python3
"""Trace the timestamps a localization depends on. No error metric.

Run it in the ros container:

    ./px4sim shell ros
    python3 /scripts/check-frame-timing.py --seconds 15

Why this exists
---------------
A detection is localized by looking up the camera pose at the detection's
stamp, so the answer is only as good as three timestamps: when the frame was
captured, when each frame of the tree last reported, and how those line up.
Measuring the localization error instead conflates all of it with the scene:
more detections near a target lower the mean error whatever the timing does.
So this measures the clocks alone.

What it reports
---------------
capture to payload   how far DeepStream's payload time trails the frame clock.
                     Ambiguous by whole frame periods on its own, which is why
                     detections_bridge snaps rather than trusting it.
report interval      how often each moving frame is published, from /tf. This
                     is the resolution of every pose lookup.
staleness            for each detection, the age of the newest report at or
                     before the capture time, per frame. This is the error
                     source that grows with slew rate: the pose used for the
                     ray is this old, and the camera kept moving.
"""

from __future__ import annotations

import argparse
import statistics
import sys
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, "/stacks/baseline/sim_bridge")
from sim_bridge.runtime import BEST_EFFORT, now_s

# ------------------------------------------------------------------- tunables
# How much report history to keep per frame, seconds.
HISTORY_S = 5.0
# Report intervals over this are called out as a stalled stream.
SLOW_REPORT_MS = 100.0


def stats(values: list, unit: str = "ms") -> str:
    if not values:
        return "no samples"
    ordered = sorted(values)
    return (f"median {statistics.median(ordered):8.1f} {unit}   "
            f"p05 {ordered[len(ordered) // 20]:8.1f}   "
            f"p95 {ordered[-max(1, len(ordered) // 20)]:8.1f}   "
            f"max {ordered[-1]:8.1f}   n={len(ordered)}")


class FrameTiming(Node):
    def __init__(self, camera: str) -> None:
        super().__init__("check_frame_timing")
        # Report stamps per child frame, from /tf itself rather than from a
        # buffer, so each hop is measured on its own instead of composed.
        self.reports: dict[str, deque] = defaultdict(lambda: deque(maxlen=4000))
        self.intervals: dict[str, list] = defaultdict(list)
        self.staleness: dict[str, list] = defaultdict(list)
        self.detections = 0

        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.create_subscription(
            Detection2DArray, f"/perception/{camera}/detections",
            self._on_detections, BEST_EFFORT)

    def _on_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            child = tf.child_frame_id
            seconds = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            history = self.reports[child]
            if history:
                self.intervals[child].append((seconds - history[-1]) * 1000.0)
            history.append(seconds)
            while history and seconds - history[0] > HISTORY_S:
                history.popleft()

    def _on_detections(self, msg: Detection2DArray) -> None:
        # Every array carries a snapped capture stamp, boxes or not, and the
        # pose age at that instant does not depend on what was in the frame.
        self.detections += 1
        capture = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        for child, history in self.reports.items():
            # The newest report at or before the capture instant: the one a
            # pose lookup would interpolate from.
            earlier = [s for s in history if s <= capture]
            if earlier:
                self.staleness[child].append((capture - earlier[-1]) * 1000.0)

    def report(self) -> None:
        print(f"\n  detection messages: {self.detections}")
        print("\n  report interval per moving frame, from /tf")
        for child in sorted(self.intervals):
            values = self.intervals[child]
            if not values:
                continue
            median = statistics.median(values)
            rate = 1000.0 / median if median else 0.0
            flag = "  <-- SLOW" if median > SLOW_REPORT_MS else ""
            print(f"    {child:28s} {stats(values)}   ~{rate:5.1f} Hz{flag}")

        print("\n  pose age at the capture instant, per frame")
        print("    the ray is cast from a pose this old; at a slew rate w the")
        print("    pointing error is w times this")
        for child in sorted(self.staleness):
            print(f"    {child:28s} {stats(self.staleness[child])}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="gimbal")
    ap.add_argument("--seconds", type=float, default=15.0)
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = FrameTiming(args.camera)
    print(f"tracing {args.camera} timestamps for {args.seconds:.0f}s")
    deadline = now_s(node) + args.seconds
    try:
        while rclpy.ok() and now_s(node) < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    main()
    sys.exit(0)
