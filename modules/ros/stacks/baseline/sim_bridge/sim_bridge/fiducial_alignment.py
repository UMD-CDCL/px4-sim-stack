#!/usr/bin/env python3
"""Correct localization bias against a surveyed point.

Localized positions land in `map`, the vehicle's own EKF frame, which sits
meters away from any frame built from a different GPS receiver. That offset is
a frame difference, not noise, so it does not average away. A fiducial is a
point known in both frames: `surveyed` is where it truly is, `measured` is
where this pipeline put it, and the difference goes out as a static transform
from `map` to `fiducial` so tf2 does the arithmetic for every consumer.

Do not use a target you also score against. Fitting the correction to a scored
target and then reporting the error against it measures nothing but the
arithmetic.

A bad configuration falls back to the identity transform, with an error in the
log: both points at 0.0 mean one was never filled in, and a correction past
MAX_CORRECTION_M is a wrong coordinate rather than a receiver bias. The
identity keeps the fiducial frame resolvable, so the localizers keep
publishing, uncorrected.

Parameters
    enabled           off by default
    surveyed_lla      [lat, lon, alt] where the fiducial truly is
    measured_lla      [lat, lon, alt] where this pipeline put it
    fiducial_frame    name of the corrected frame, default "fiducial"
    reference_frame   the frame being corrected, default "map"
"""

from __future__ import annotations

import math

from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from sim_bridge.geo import MapOrigin, lla_to_enu
from sim_bridge.runtime import spin

# ------------------------------------------------------------------- tunables
# A GPS frame bias is metres. A larger correction is a wrong coordinate.
MAX_CORRECTION_M = 100.0

UNSET_LLA = [0.0, 0.0, 0.0]
IDENTITY_OFFSET = (0.0, 0.0, 0.0)


class FiducialAlignment(Node):
    def __init__(self) -> None:
        super().__init__("fiducial_alignment")

        self.declare_parameter("enabled", False)
        self.declare_parameter("surveyed_lla", [0.0, 0.0, 0.0])
        self.declare_parameter("measured_lla", [0.0, 0.0, 0.0])
        self.declare_parameter("fiducial_frame", "fiducial")
        self.declare_parameter("reference_frame", "map")

        self.enabled = bool(self.get_parameter("enabled").value)
        self.surveyed = [float(v) for v in self.get_parameter("surveyed_lla").value]
        self.measured = [float(v) for v in self.get_parameter("measured_lla").value]
        self.fiducial_frame = self.get_parameter("fiducial_frame").value
        self.reference = self.get_parameter("reference_frame").value

        self.bc = StaticTransformBroadcaster(self)
        self.published = False

        if not self.enabled:
            # Off means nothing to compute, so take no inputs at all.
            self.get_logger().info(
                "fiducial correction is off. Localizations stay in the vehicle's "
                "own frame. See the module docstring to turn it on.")
            return

        if self.surveyed == UNSET_LLA or self.measured == UNSET_LLA:
            self._publish_identity(
                "fiducial correction is enabled, but the surveyed or the "
                "measured point is still 0.0. Fill in both FIDUCIAL_SURVEYED_* "
                "and FIDUCIAL_MEASURED_*, or set FIDUCIAL_ENABLED=0.")
            return

        # The same origin estimate the Map panel publishers use, so the
        # correction is measured against the frame they report in.
        self.origin = MapOrigin(self)
        self.timer = self.create_timer(1.0, self._try_publish)

    def _send(self, offset) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.reference
        t.child_frame_id = self.fiducial_frame
        (t.transform.translation.x, t.transform.translation.y,
         t.transform.translation.z) = offset
        t.transform.rotation.w = 1.0
        self.bc.sendTransform(t)
        self.published = True

    def _publish_identity(self, reason: str) -> None:
        """Latch the identity transform, so the fiducial frame resolves and
        the localizers keep publishing while the configuration is unusable."""
        self.get_logger().error(
            f"{reason} Publishing the identity: positions in "
            f"'{self.fiducial_frame}' carry no correction.")
        self._send(IDENTITY_OFFSET)

    def _try_publish(self) -> None:
        origin = self.origin.lla
        if self.published or origin is None:
            return

        surveyed = lla_to_enu(*self.surveyed, *origin)
        measured = lla_to_enu(*self.measured, *origin)
        # Where the true position sits relative to where we reported it.
        correction = tuple(s - m for s, m in zip(surveyed, measured))
        distance = math.hypot(*correction)

        if distance > MAX_CORRECTION_M:
            self._publish_identity(
                f"the fiducial correction came out {distance:.0f} m, past the "
                f"{MAX_CORRECTION_M:.0f} m bound. Check the surveyed and the "
                f"measured coordinates against each other.")
        else:
            # The child frame origin goes at minus the correction, so that
            # transforming a point out of `reference` and into `fiducial` adds
            # it. Getting this sign backwards doubles the error instead of
            # removing it, which looks like the correction made things worse.
            self._send(tuple(-c for c in correction))
            self.get_logger().info(
                f"fiducial correction: east {correction[0]:+.2f} m, "
                f"north {correction[1]:+.2f} m, up {correction[2]:+.2f} m. "
                f"Positions in '{self.fiducial_frame}' carry it.")

        # Done for good: the broadcaster latches the transform.
        self.timer.cancel()


def main() -> None:
    spin(FiducialAlignment)


if __name__ == "__main__":
    main()
