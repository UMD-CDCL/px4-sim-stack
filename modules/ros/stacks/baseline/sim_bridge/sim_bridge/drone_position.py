#!/usr/bin/env python3
"""Publish the drone position with its heading for the Map panel.

A NavSatFix has no heading field, so the Map panel's own vehicle dot cannot
say which way the drone points. Foxglove's LocationFix carries one, and the
panel draws an arrowhead that turns with it. Position and covariance are the
vehicle's own fix, untouched, so this marker cannot disagree with the fix the
panel already shows; the two schemas share the covariance convention and
constants. Heading goes out as NaN, which the schema reads as "not set",
until the first compass message.

Subscribes
    /mavros/global_position/global        sensor_msgs/NavSatFix
    /mavros/global_position/compass_hdg   std_msgs/Float64, degrees

Publishes
    /drone/position   foxglove_msgs/LocationFix, heading set when known
"""

from __future__ import annotations

import math

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from sim_bridge.runtime import HAVE_FOXGLOVE, require_foxglove, spin

if HAVE_FOXGLOVE:
    from foxglove_msgs.msg import LocationFix

PUBLISH_RATE_HZ = 5.0
# The Map panel marker color.
MARKER_RGB = (0x61 / 255, 0xCB / 255, 0xFF / 255)


class DronePosition(Node):
    def __init__(self) -> None:
        super().__init__("drone_position")

        self.fix: NavSatFix | None = None
        self.heading_rad = float("nan")

        if not require_foxglove(self, "the Map panel gets no drone position"):
            return

        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._on_fix, qos_profile_sensor_data)
        self.create_subscription(Float64,
                                 "/mavros/global_position/compass_hdg",
                                 self._on_heading, qos_profile_sensor_data)
        self.position_pub = self.create_publisher(
            LocationFix, "/drone/position", 1)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)

    def _on_fix(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self.fix = msg

    def _on_heading(self, msg: Float64) -> None:
        self.heading_rad = math.radians(float(msg.data))

    def _publish(self) -> None:
        if self.fix is None:
            return
        out = LocationFix()
        out.timestamp = self.fix.header.stamp
        out.frame_id = self.fix.header.frame_id
        out.latitude = self.fix.latitude
        out.longitude = self.fix.longitude
        out.altitude = self.fix.altitude
        out.position_covariance = self.fix.position_covariance
        out.position_covariance_type = self.fix.position_covariance_type
        out.heading = self.heading_rad
        out.color.r, out.color.g, out.color.b = MARKER_RGB
        out.color.a = 1.0
        self.position_pub.publish(out)


def main() -> None:
    spin(DronePosition)


if __name__ == "__main__":
    main()
