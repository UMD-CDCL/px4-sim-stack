#!/usr/bin/env python3
"""Publish neutral, timestamped gimbal telemetry for simulated vehicles."""
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from mavros_msgs.msg import GimbalDeviceAttitudeStatus, GimbalManagerSetPitchyaw
from mavros_msgs.srv import GimbalManagerConfigure
from rclpy.node import Node
from std_msgs.msg import Float32
from tf2_ros import StaticTransformBroadcaster

class SimGimbalAttitude(Node):
    def __init__(self) -> None:
        super().__init__("sim_gimbal_attitude")
        uas = int(self.declare_parameter("uas", 11).value)
        self.publisher = self.create_publisher(GimbalDeviceAttitudeStatus, f"/uas{uas}/gimbal_control/device/attitude_status", 10)
        self.tf = StaticTransformBroadcaster(self)
        self.frame = f"d{uas}_gimbal_frame"
        self.parent = f"d{uas}_gimbal_frame_ref"
        self.pitch = 0.0
        self.yaw = 0.0
        self.create_service(GimbalManagerConfigure,
                            f"/uas{uas}/gimbal_control/manager/configure",
                            self.configure)
        self.create_subscription(Float32, f"/uas{uas}/gimbal_raw_command",
                                 self.raw_command, 10)
        self.create_subscription(GimbalManagerSetPitchyaw,
                                 f"/uas{uas}/gimbal_angle_cmd",
                                 self.angle_command, 10)
        self.create_timer(1.0, self.publish_tf)
        self.create_timer(0.1, self.publish)

    def publish(self) -> None:
        msg = GimbalDeviceAttitudeStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.flags = GimbalDeviceAttitudeStatus.FLAGS_NEUTRAL
        msg.q = self.quaternion()
        self.publisher.publish(msg)

    def raw_command(self, msg: Float32) -> None:
        if msg.data != -361.0:
            self.pitch = float(msg.data)

    def angle_command(self, msg: GimbalManagerSetPitchyaw) -> None:
        self.pitch = float(msg.pitch)
        self.yaw = float(msg.yaw)

    def configure(self, request, response):
        response.success = True
        response.result = 0
        return response

    def quaternion(self) -> Quaternion:
        # MAVROS reports FRD attitude. Negative pitch is a downward view.
        import math
        p = math.radians(-self.pitch) / 2.0
        y = math.radians(self.yaw) / 2.0
        return Quaternion(y=math.sin(p), z=math.sin(y), w=math.cos(p) * math.cos(y))

    def publish_tf(self) -> None:
        edge = TransformStamped()
        edge.header.stamp = self.get_clock().now().to_msg()
        edge.header.frame_id = self.parent
        edge.child_frame_id = self.frame
        edge.transform.rotation.w = 1.0
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
