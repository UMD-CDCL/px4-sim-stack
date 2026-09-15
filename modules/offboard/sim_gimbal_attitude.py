#!/usr/bin/env python3
"""Publish neutral, timestamped gimbal telemetry for simulated vehicles."""
import rclpy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform, TransformStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class SimGimbalAttitude(Node):
    def __init__(self) -> None:
        super().__init__("sim_gimbal_attitude")
        uas = self.declare_parameter("uas", 11).value
        self.publisher = self.create_publisher(GimbalDeviceAttitudeStatus, f"/uas{int(uas)}/gimbal_control/device/attitude_status", 10)
        self.tf = TransformBroadcaster(self)
        self.frame = f"d{int(uas)}_gimbal_frame"
        self.parent = f"d{int(uas)}_gimbal_frame_ref"
        self.create_timer(0.1, self.publish)

    def publish(self) -> None:
        msg = GimbalDeviceAttitudeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.flags = GimbalDeviceAttitudeStatus.FLAGS_NEUTRAL
        msg.q = Quaternion(w=1.0)
        self.publisher.publish(msg)
        edge = TransformStamped()
        edge.header.stamp = msg.header.stamp
        edge.header.frame_id = self.parent
        edge.child_frame_id = self.frame
        edge.transform = Transform(rotation=msg.q)
        self.tf.sendTransform(edge)

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
