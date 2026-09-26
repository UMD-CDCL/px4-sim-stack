#!/usr/bin/env python3
"""Offer a BEST_EFFORT mirror of a reliable MAVROS telemetry topic."""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix


class Relay(Node):
    def __init__(self, topic: str):
        super().__init__('rerecord_qos_relay')
        self.pub = self.create_publisher(
            NavSatFix, topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            NavSatFix, topic, self.pub.publish,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))


rclpy.init()
node = Relay(sys.argv[1] if len(sys.argv) > 1 else '/uas4/global_position/global')
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
