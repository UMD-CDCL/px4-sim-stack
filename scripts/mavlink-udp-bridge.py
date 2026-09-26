#!/usr/bin/env python3
"""Turn a recorded mavros_msgs/Mavlink topic back into UDP MAVLink bytes."""
import socket
import sys

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def wire_bytes(message: Mavlink) -> bytes:
    payload = b''.join(int(word).to_bytes(8, 'little') for word in message.payload64)[:message.len]
    if message.magic == Mavlink.MAVLINK_V10:
        header = bytes((message.magic, message.len, message.seq, message.sysid,
                        message.compid, message.msgid & 0xff))
    elif message.magic == Mavlink.MAVLINK_V20:
        header = bytes((message.magic, message.len, message.incompat_flags,
                        message.compat_flags, message.seq, message.sysid,
                        message.compid)) + int(message.msgid).to_bytes(3, 'little')
    else:
        return b''
    signature = bytes(message.signature) if message.incompat_flags & 1 else b''
    return header + payload + int(message.checksum).to_bytes(2, 'little') + signature


class Bridge(Node):
    def __init__(self, topic: str, host: str, port: int):
        super().__init__('rerecord_mavlink_udp_bridge')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.endpoint = (host, port)
        self.messages = 0
        # rosbag2 replays this capture with BEST_EFFORT reliability. A
        # reliable reader never matches it and turns a healthy replay into a
        # silent MAVROS timeout.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10000,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Mavlink, topic, self.on_message, qos)

    def on_message(self, message: Mavlink) -> None:
        if message.framing_status != Mavlink.FRAMING_OK:
            return
        frame = wire_bytes(message)
        if frame:
            self.sock.sendto(frame, self.endpoint)
            self.messages += 1


def main() -> None:
    topic = sys.argv[1] if len(sys.argv) > 1 else '/uas4/mavlink_source'
    host = sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1'
    port = int(sys.argv[3]) if len(sys.argv) > 3 else 14550
    rclpy.init()
    node = Bridge(topic, host, port)
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info(f'forwarded {node.messages} MAVLink frames')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
