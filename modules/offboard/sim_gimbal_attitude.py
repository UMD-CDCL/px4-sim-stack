#!/usr/bin/env python3
"""Publish neutral, timestamped gimbal telemetry for simulated vehicles."""
import rclpy
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from rclpy.node import Node

class SimGimbalAttitude(Node):
    def __init__(self) -> None:
        super().__init__("sim_gimbal_attitude")
        uas = self.declare_parameter("uas", 11).value
        self.publisher = self.create_publisher(GimbalDeviceAttitudeStatus, f"/uas{int(uas)}/gimbal_control/device/attitude_status", 10)
        self.create_timer(0.1, self.publish)

    def publish(self) -> None:
        msg = GimbalDeviceAttitudeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.flags = GimbalDeviceAttitudeStatus.FLAGS_NEUTRAL
        msg.q = Quaternion(w=1.0)
        self.publisher.publish(msg)

def main() -> None:
    rclpy.init()
    node = SimGimbalAttitude()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
