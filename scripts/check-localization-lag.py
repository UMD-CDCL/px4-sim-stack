#!/usr/bin/env python3
"""Find the time offset a moving camera's localization needs.

Run it in the ros container while the gimbal slews:

    ./px4sim shell ros
    python3 /scripts/check-localization-lag.py --seconds 60

Why this exists
---------------
A detection is localized by looking up the camera pose at the detection's
stamp. When that stamp does not name the instant the frame was captured, the
pose is wrong by the camera's own motion across the difference. The error
then grows with slew rate and disappears at rest, which is what a lag looks
like from the outside.

This re-localizes every detection at a sweep of offsets around its stamp and
reports which offset puts the estimates closest to ground truth. A minimum
away from zero is the missing correction, and its depth says how much error
the lag costs. The error is also reported against slew rate, because a lag
shows as a slope there while a fixed mounting error shows as a constant.

It reads the same surface and casts the same rays as the live localizer, so
the numbers are the ones the stack would produce.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3DArray

sys.path.insert(0, "/stacks/baseline/sim_bridge")
from sim_bridge.localization import GroundLocalizer
from sim_bridge.projection import intrinsics_ready, quat_rotate, ray_in_optical
from sim_bridge.runtime import BEST_EFFORT, LATCHED, now_s, tf_buffer

# ------------------------------------------------------------------- tunables
# Offsets added to each detection stamp before the pose lookup, milliseconds.
# --sweep first:last:step narrows it once a coarse pass has found the region.
DEFAULT_SWEEP_MS = (-300, 80, 20)
# How often the boresight is sampled, for the slew rate.
POSE_SAMPLE_HZ = 50.0
# The span the slew rate is measured over.
RATE_WINDOW_S = 0.20
# A detection whose best offset still lands farther than this from every
# target is not a match, and it is dropped rather than counted as error.
MATCH_GATE_M = 20.0
# Slew rate buckets for the error table, degrees per second.
RATE_BUCKETS = (2.0, 10.0, 25.0, 50.0)
# Camera speed buckets, metres per second. Attitude lag and position lag both
# grow the error with motion, but they come from different chains, so they are
# reported apart.
SPEED_BUCKETS = (0.5, 2.0, 6.0, 12.0)
MAX_RANGE = 2000.0


class LocalizationLag(Node):
    def __init__(self, camera: str, anchor: str, sweep: list) -> None:
        super().__init__("check_localization_lag")
        self.optical = f"{camera}_camera_optical_frame"
        self.reference = "map"
        self.anchor = anchor
        self.sweep = sweep
        self.localizer = GroundLocalizer(self)
        self.tf_buffer = tf_buffer(self)

        self.info: CameraInfo | None = None
        self.truth: list[tuple[str, float, float, float]] = []
        # (time, boresight elevation, position), for the slew rate and speed.
        self.poses: deque = deque(maxlen=int(POSE_SAMPLE_HZ * 10))
        # Per detection: the slew rate, the speed, and the error at each offset.
        self.samples: list[tuple[float, float, list]] = []
        self.stamp_gaps: list[float] = []
        self.no_pose = 0
        self.unmatched = 0

        self.create_subscription(CameraInfo, f"/camera/{camera}/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, "/ground_truth/truth_3d",
                                 self._on_truth, LATCHED)
        self.create_subscription(Detection2DArray,
                                 f"/perception/{camera}/detections",
                                 self._on_detections, BEST_EFFORT)
        self.create_timer(1.0 / POSE_SAMPLE_HZ, self._sample_pose)

    # ------------------------------------------------------------------ input
    def _on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def _on_truth(self, msg: Detection3DArray) -> None:
        self.truth = [(d.id, d.bbox.center.position.x, d.bbox.center.position.y,
                       d.bbox.center.position.z) for d in msg.detections]

    def _pose_at(self, stamp: Time):
        try:
            tf = self.tf_buffer.lookup_transform(self.reference, self.optical,
                                                 stamp)
        except Exception:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def _sample_pose(self) -> None:
        """The camera's elevation and position now. The optical z axis is the
        view direction."""
        pose = self._pose_at(Time())
        if pose is None:
            return
        position, rotation = pose
        axis = quat_rotate(rotation, (0.0, 0.0, 1.0))
        elevation = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
        self.poses.append((now_s(self), elevation, position))

    def _window(self) -> list:
        if len(self.poses) < 2:
            return []
        newest = self.poses[-1][0]
        window = [s for s in self.poses if newest - s[0] <= RATE_WINDOW_S]
        return window if len(window) >= 2 else []

    def _slew_rate(self) -> float:
        """Boresight degrees per second across the last window, absolute."""
        window = self._window()
        if not window:
            return 0.0
        span = window[-1][0] - window[0][0]
        return abs(window[-1][1] - window[0][1]) / span if span > 0 else 0.0

    def _speed(self) -> float:
        """Camera metres per second across the last window. A position lag
        and an attitude lag both grow the error with motion, so the two are
        reported apart."""
        window = self._window()
        if not window:
            return 0.0
        span = window[-1][0] - window[0][0]
        if span <= 0:
            return 0.0
        first, last = window[0][2], window[-1][2]
        return math.dist(first, last) / span

    # ------------------------------------------------------------------- work
    def _ground_point(self, u: float, v: float, stamp: Time):
        pose = self._pose_at(stamp)
        if pose is None:
            return None
        origin, rotation = pose
        direction = quat_rotate(rotation, ray_in_optical(u, v, self.info.k))
        return self.localizer.intersect(origin, direction, MAX_RANGE)

    def _on_detections(self, msg: Detection2DArray) -> None:
        if not msg.detections or not intrinsics_ready(self.info) or not self.truth:
            return
        stamp = Time.from_msg(msg.header.stamp)
        self._record_stamp_gap(stamp)
        rate, speed = self._slew_rate(), self._speed()

        for det in msg.detections:
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            if self.anchor == "bottom":
                v += det.bbox.size_y / 2.0

            points = {}
            for offset_ms in self.sweep:
                shifted = stamp + rclpy.duration.Duration(
                    nanoseconds=int(offset_ms * 1e6))
                point = self._ground_point(u, v, shifted)
                if point is not None:
                    points[offset_ms] = point
            if not points:
                self.no_pose += 1
                continue

            # One target per detection, chosen at its own best offset, so the
            # curve never improves by silently switching targets.
            best = min(
                ((min(math.hypot(p[0] - tx, p[1] - ty)
                      for _, tx, ty, _ in self.truth), offset)
                 for offset, p in points.items()), key=lambda pair: pair[0])
            if best[0] > MATCH_GATE_M:
                self.unmatched += 1
                continue
            anchor_point = points[best[1]]
            target = min(self.truth,
                         key=lambda t: math.hypot(anchor_point[0] - t[1],
                                                  anchor_point[1] - t[2]))
            errors = [(offset, math.hypot(p[0] - target[1], p[1] - target[2]))
                      for offset, p in sorted(points.items())]
            self.samples.append((rate, speed, errors))

    def _record_stamp_gap(self, stamp: Time) -> None:
        """How far the detection stamp runs past the newest pose on record.
        Positive means the live localizer had to clamp or drop it."""
        try:
            latest = self.tf_buffer.lookup_transform(self.reference,
                                                     self.optical, Time())
        except Exception:
            return
        latest_ns = (latest.header.stamp.sec * 1_000_000_000
                     + latest.header.stamp.nanosec)
        self.stamp_gaps.append((stamp.nanoseconds - latest_ns) / 1e6)

    # ----------------------------------------------------------------- report
    def report(self) -> None:
        print(f"\n  detections matched: {len(self.samples)}")
        print(f"  no pose at any offset: {self.no_pose}")
        print(f"  beyond the {MATCH_GATE_M:.0f} m match gate: {self.unmatched}")
        if self.stamp_gaps:
            gaps = sorted(self.stamp_gaps)
            ahead = sum(1 for g in gaps if g > 0) / len(gaps) * 100.0
            print(f"\n  detection stamp minus newest pose, ms: "
                  f"median {statistics.median(gaps):+.1f}, "
                  f"p05 {gaps[len(gaps) // 20]:+.1f}, "
                  f"p95 {gaps[-max(1, len(gaps) // 20)]:+.1f}")
            print(f"  stamps ahead of the newest pose: {ahead:.0f}% "
                  f"(these clamp or drop in the live localizer)")
        if not self.samples:
            print("\n  No matched detections. Is the camera on a target?")
            return

        print(f"\n  {'offset ms':>10}  {'median err m':>13}  {'p90 err m':>10}")
        curve = []
        for index, offset in enumerate(sorted(
                {o for _, _, errors in self.samples for o, _ in errors})):
            values = sorted(errors[index][1] for _, _, errors in self.samples
                            if len(errors) > index and errors[index][0] == offset)
            if not values:
                continue
            median = statistics.median(values)
            curve.append((median, offset))
            print(f"  {offset:>10}  {median:>13.2f}  "
                  f"{values[int(len(values) * 0.9) - 1]:>10.2f}")
        best_err, best_offset = min(curve)
        # The uncorrected column, or the sweep's least-corrected end when the
        # sweep does not reach zero.
        reference = min((offset for _, offset in curve), key=abs)
        reference_err = next(m for m, o in curve if o == reference)
        print(f"\n  best offset: {best_offset:+d} ms (median error {best_err:.2f} m),"
              f" against {reference_err:.2f} m at {reference:+d} ms")

        # Attitude and position come through different chains, so a lag in
        # one shows against slew rate and a lag in the other against speed.
        self._bucket_table("slew deg/s", RATE_BUCKETS, lambda s: s[0],
                           reference, best_offset)
        self._bucket_table("speed m/s", SPEED_BUCKETS, lambda s: s[1],
                           reference, best_offset)

    def _bucket_table(self, title: str, buckets, measure,
                      reference: int, best_offset: int) -> None:
        print(f"\n  {title:>12}  {'n':>5}  {f'err at {reference:+d} ms':>14}"
              f"  {f'err at {best_offset:+d} ms':>14}")
        edges = [0.0, *buckets, float('inf')]
        for low, high in zip(edges, edges[1:]):
            picked = [sample[2] for sample in self.samples
                      if low <= measure(sample) < high]
            if not picked:
                continue

            def at(want, rows=picked):
                values = [e for errors in rows for o, e in errors if o == want]
                return statistics.median(values) if values else float('nan')

            label = (f"{low:g}-{high:g}" if high != float('inf')
                     else f">{low:g}")
            print(f"  {label:>12}  {len(picked):>5}  {at(reference):>14.2f}"
                  f"  {at(best_offset):>14.2f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", default="gimbal")
    ap.add_argument("--anchor", default="bottom", choices=("bottom", "center"))
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--sweep", default=":".join(str(v) for v in DEFAULT_SWEEP_MS),
                    help="first:last:step in milliseconds")
    args, _ = ap.parse_known_args()

    first, last, step = (int(v) for v in args.sweep.split(":"))
    sweep = list(range(first, last + 1, step))

    rclpy.init()
    node = LocalizationLag(args.camera, args.anchor, sweep)
    print(f"measuring {args.camera} for {args.seconds:.0f}s, "
          f"onto {node.localizer.description}. Slew the gimbal.")
    deadline = now_s(node) + args.seconds
    try:
        while rclpy.ok() and now_s(node) < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
