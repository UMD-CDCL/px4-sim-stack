#!/usr/bin/env python3
"""Publish simulator-only gimbal frame edges.

PX4/MAVROS owns the gimbal command and telemetry topics. This helper exists
only for the simulator frame tree, so it must not register a competing ROS
front door that can report synthetic success or state.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from tf2_ros import TransformBroadcaster

class SimGimbalAttitude(Node):
    def __init__(self) -> None:
        super().__init__("sim_gimbal_attitude")
        uas = int(self.declare_parameter("uas", 11).value)
        self.tf = TransformBroadcaster(self)
        self.frame = f"d{uas}_gimbal_frame"
        self.parent = f"d{uas}_gimbal_frame_ref"
        self._attitude = None
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            f"/uas{uas}/gimbal_control/device/attitude_status",
            self.attitude_cb,
            10,
        )
        self.create_timer(0.05, self.publish_tf)

    def attitude_cb(self, msg: GimbalDeviceAttitudeStatus) -> None:
        self._attitude = msg.q

    def publish_tf(self) -> None:
        edge = TransformStamped()
        edge.header.stamp = self.get_clock().now().to_msg()
        edge.header.frame_id = self.parent
        edge.child_frame_id = self.frame
        if self._attitude is None:
            edge.transform.rotation.w = 1.0
        else:
            edge.transform.rotation = self._attitude
        # The MAVInsight rangefinder node owns the calibrated, measured
        # gimbal -> rangefinder edge. Publishing it here would create two TF
        # authorities and discard its sensor offset.
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
