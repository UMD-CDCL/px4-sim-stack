#!/usr/bin/env python3
"""Publish the drone position with its heading for the Map panel.

The Map panel already draws the vehicle from /mavros/global_position/global,
but a NavSatFix has no field for heading, so that dot cannot say which way
the drone points. Foxglove's own LocationFix schema carries an optional
heading, radians clockwise from north, and the Map panel turns a marker
into an arrowhead that rotates with it. So this node pairs the vehicle's
own fix with the compass heading and republishes them as one
foxglove_msgs/LocationFix.

The position and covariance are the vehicle's own fix, untouched. No map
origin math is involved, so this marker cannot disagree with the fix the
Map panel already shows. NavSatFix and LocationFix share the covariance
convention, ENU about the reported position, and the same covariance type
constants, zero through three, so both copy straight across.

Heading is /mavros/global_position/compass_hdg, degrees clockwise from
north, converted to the radians LocationFix asks for. Until the first
compass message it goes out as NaN, which the schema defines as "not
set", and the panel falls back to a plain dot.

Subscribes
    /mavros/global_position/global        sensor_msgs/NavSatFix
    /mavros/global_position/compass_hdg   std_msgs/Float64, degrees

Publishes
    /drone/position   foxglove_msgs/LocationFix, heading set when known
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

try:
    from foxglove_msgs.msg import LocationFix
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

PUBLISH_RATE_HZ = 5.0


class DronePosition(Node):
    def __init__(self) -> None:
        super().__init__("drone_position")

        self.fix: NavSatFix | None = None
        self.heading_rad = float("nan")

        if not HAVE_FOXGLOVE:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no drone "
                "position. Install ros-$ROS_DISTRO-foxglove-msgs.")
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
        out.color.r = 0x61 / 255
        out.color.g = 0xCB / 255
        out.color.b = 0xFF / 255
        out.color.a = 1.0
        self.position_pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DronePosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
