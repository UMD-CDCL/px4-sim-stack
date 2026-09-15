#!/usr/bin/env python3
"""Publish simulator-only gimbal frame edges.

PX4/MAVROS owns the gimbal command and telemetry topics. This helper exists
only for the simulator frame tree, so it must not register a competing ROS
front door that can report synthetic success or state.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

class SimGimbalAttitude(Node):
    def __init__(self) -> None:
        super().__init__("sim_gimbal_attitude")
        uas = int(self.declare_parameter("uas", 11).value)
        self.tf = StaticTransformBroadcaster(self)
        self.frame = f"d{uas}_gimbal_frame"
        self.parent = f"d{uas}_gimbal_frame_ref"
        self.rangefinder = f"d{uas}_rangefinder_frame"
        self.create_timer(1.0, self.publish_tf)

    def publish_tf(self) -> None:
        edge = TransformStamped()
        edge.header.stamp = self.get_clock().now().to_msg()
        edge.header.frame_id = self.parent
        edge.child_frame_id = self.frame
        edge.transform.rotation.w = 1.0
        range_edge = TransformStamped()
        range_edge.header.stamp = edge.header.stamp
        range_edge.header.frame_id = self.frame
        range_edge.child_frame_id = self.rangefinder
        range_edge.transform.rotation.w = 1.0
        self.tf.sendTransform([edge, range_edge])

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
