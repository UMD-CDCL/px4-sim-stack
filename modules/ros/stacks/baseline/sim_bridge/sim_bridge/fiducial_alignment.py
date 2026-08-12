#!/usr/bin/env python3
"""Correct localization bias against a surveyed point.

The problem
-----------
Localized positions land in `map`, which is the vehicle's own EKF frame. That
frame is built from the vehicle's GPS and drifts relative to any other frame
built from a different receiver. Two aircraft, or an aircraft and a surveyed
map, agree on the shape of the world and disagree on where it sits, typically
by metres.

That offset is not noise and it does not average away. It is a frame
difference, and the fix is to measure it once against something whose position
is known, then subtract it.

How it works
------------
A fiducial is a physical point that appears in both frames:

  `surveyed`  where it truly is, from a survey, an RTK fix or a published
              benchmark. This defines the frame you want answers in.
  `measured`  where this pipeline says it is, read off a localization of that
              same point.

The difference is the bias. This node publishes it as a transform from `map` to
`fiducial`, so anything can be re-expressed in the corrected frame with tf2 and
nothing has to do the arithmetic itself.

Both are latitude, longitude and altitude, because that is the form a survey
comes in and the form two independent systems can actually exchange. They are
converted to local metres against the same origin the rest of the stack uses.

In simulation you do not have to fly a calibration: the true position is in the
scenario file, so `surveyed` and `measured` can both be written down.

A warning about measuring
-------------------------
Do not use a target you also score against. Fitting the correction to a scored
target and then reporting the error against that same target measures nothing
but the arithmetic. Use a separate fiducial that no detector is graded on.

Parameters
    enabled           off by default, so the frame appears only when configured
    surveyed_lla      [lat, lon, alt] where the fiducial truly is
    measured_lla      [lat, lon, alt] where this pipeline put it
    fiducial_frame    name of the corrected frame, default "fiducial"
    reference_frame   the frame being corrected, default "map"
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from tf2_ros import StaticTransformBroadcaster

EARTH_R = 6378137.0


def lla_to_enu(lat: float, lon: float, alt: float,
               ref_lat: float, ref_lon: float, ref_alt: float) -> tuple[float, float, float]:
    """Flat-earth local ENU metres, about a reference. Good over a few km."""
    east = math.radians(lon - ref_lon) * EARTH_R * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * EARTH_R
    return east, north, alt - ref_alt


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

        self.origin: tuple[float, float, float] | None = None
        self.local: tuple[float, float, float] | None = None
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)

        if not self.enabled:
            self.get_logger().info(
                "fiducial correction is off. Localizations stay in the vehicle's "
                "own frame. See the module docstring to turn it on.")
        self.create_timer(1.0, self._try_publish)

    def _on_local(self, msg) -> None:
        p = msg.pose.position
        self.local = (p.x, p.y, p.z)

    def _on_fix(self, msg) -> None:
        # The same derivation the ground truth node uses: walk the vehicle's own
        # fix back to local zero, which is where the EKF frame starts.
        if self.local is None or msg.status.status < 0:
            return
        x, y, z = self.local
        lat = msg.latitude - math.degrees(y / EARTH_R)
        lon = msg.longitude - math.degrees(x / (EARTH_R * math.cos(math.radians(msg.latitude))))
        self.origin = (lat, lon, msg.altitude - z)

    def _try_publish(self) -> None:
        if self.published or not self.enabled or self.origin is None:
            return

        s = lla_to_enu(*self.surveyed, *self.origin)
        m = lla_to_enu(*self.measured, *self.origin)
        # Where the true position sits relative to where we reported it.
        correction = (s[0] - m[0], s[1] - m[1], s[2] - m[2])

        # The child frame origin goes at minus the correction, so that
        # transforming a point out of `reference` and into `fiducial` adds it.
        # Getting this sign backwards doubles the error instead of removing it,
        # which looks like the correction made things worse.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.reference
        t.child_frame_id = self.fiducial_frame
        t.transform.translation.x = -correction[0]
        t.transform.translation.y = -correction[1]
        t.transform.translation.z = -correction[2]
        t.transform.rotation.w = 1.0
        self.bc.sendTransform(t)
        self.published = True

        self.get_logger().info(
            f"fiducial correction: east {correction[0]:+.2f} m, "
            f"north {correction[1]:+.2f} m, up {correction[2]:+.2f} m. "
            f"Positions in '{self.fiducial_frame}' carry it.")


def main() -> None:
    rclpy.init()
    node = FiducialAlignment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
