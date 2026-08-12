#!/usr/bin/env python3
"""Measure how far DeepStream's detection timestamps lag frame capture.

DeepStream stamps a detection when nvmsgconv runs, which is after decode and
inference. The localizer needs the capture time instead, so it subtracts a
latency and then snaps to a real capture time. This measures that latency, so
the number in the config is one you measured rather than one you guessed.

How it works
------------
Detections are produced from real frames, so every detection timestamp is one
true capture time plus a nearly constant pipeline latency. Sweep a candidate
latency, subtract it from each detection timestamp, and see how close the
result lands to an actual capture time. The true latency makes those residuals
collapse to nearly zero. A wrong one leaves them spread across a frame period.

    docker compose exec perception python3 /opt/perception/app/measure-latency.py

Put the answer in modules/ros/stacks/baseline/stack.launch.py as
detection_latency, or set it as a ROS parameter at run time.
"""

from __future__ import annotations

import argparse
import datetime
import json
import statistics
import sys
import time

import paho.mqtt.client as mqtt

captures: list[float] = []
detections: list[float] = []


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload)
    except ValueError:
        return
    if msg.topic.startswith("video/frames/"):
        if payload.get("stream") == userdata["stream"]:
            captures.append(payload["capture_unix_ns"] / 1e9)
    else:
        stamp = payload.get("@timestamp")
        if stamp:
            t = datetime.datetime.fromisoformat(stamp.replace("Z", "+00:00"))
            detections.append(t.timestamp())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="message-bus")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--stream", default="gimbal")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--max-latency", type=float, default=0.40)
    args = ap.parse_args()

    if hasattr(mqtt, "CallbackAPIVersion"):
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    else:
        c = mqtt.Client()
    c.user_data_set({"stream": args.stream})
    c.on_message = on_message
    c.connect(args.host, args.port, 30)
    c.subscribe([("video/frames/#", 0), ("perception/detections", 0)])
    c.loop_start()

    print(f"listening {args.seconds:.0f}s on mqtt://{args.host}:{args.port} ...")
    time.sleep(args.seconds)
    c.loop_stop()
    c.disconnect()

    print(f"  captures:   {len(captures)}")
    print(f"  detections: {len(detections)}")
    if len(captures) < 20:
        print("\nToo few capture records. Is the simulator running with FRAME_CLOCK=1?")
        return 1
    if len(detections) < 5:
        print("\nToo few detections. Is the perception profile running?")
        return 1

    caps = sorted(captures)

    def median_residual(latency: float) -> float:
        out = []
        for t in detections:
            target = t - latency
            # Nearest capture to the corrected timestamp.
            lo, hi = 0, len(caps) - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if caps[mid] < target:
                    lo = mid + 1
                else:
                    hi = mid
            best = abs(caps[lo] - target)
            if lo > 0:
                best = min(best, abs(caps[lo - 1] - target))
            out.append(best)
        return statistics.median(out)

    step = 0.001
    grid = [i * step for i in range(int(args.max_latency / step) + 1)]
    scored = [(median_residual(l), l) for l in grid]
    best_res, best_lat = min(scored)
    worst_res = max(r for r, _ in scored)

    # Frame period, from the capture stream itself.
    gaps = [b - a for a, b in zip(caps, caps[1:]) if 0 < b - a < 1.0]
    period = statistics.median(gaps) if gaps else 0.0

    print(f"\n  frame period:      {period * 1000:6.1f} ms  ({1 / period:.1f} fps)" if period
          else "\n  frame period:      unknown")
    print(f"  best latency:      {best_lat * 1000:6.1f} ms")
    print(f"  residual at best:  {best_res * 1000:6.2f} ms")
    print(f"  residual at worst: {worst_res * 1000:6.2f} ms")

    if period and best_res < period / 8:
        print("\n  Sharp minimum, so the alignment is real. Use:")
        print(f"      detection_latency: {best_lat:.3f}")
    else:
        print("\n  No sharp minimum. The residuals are similar at every candidate,")
        print("  which means the detection timestamps are not locked to frames.")
        print("  Check that detections and captures come from the same camera.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
