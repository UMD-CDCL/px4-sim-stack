import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import UInt64
from cdcl_umd_msgs.msg import TargetBoxArray
rclpy.init()
n = Node('sitl_tracking_cutoff_verifier')
q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
cutoffs = []
received = []
n.create_subscription(UInt64, '/uas11/tracking/search_session',
                      lambda m: cutoffs.append(m.data), latched)
n.create_subscription(TargetBoxArray, '/uas11/tracking/target_locations',
                      lambda m: received.append(m.seq), q)
pub = n.create_publisher(TargetBoxArray, '/uas11/target_locations', q)
start = time.monotonic()
while time.monotonic() - start < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
assert cutoffs, 'no session cutoff received'
for seq, stamp in [(990100, cutoffs[-1]-1), (990101, n.get_clock().now().nanoseconds)]:
    msg = TargetBoxArray()
    msg.seq = seq
    msg.system_id = 11
    msg.header.stamp.sec = stamp // 1_000_000_000
    msg.header.stamp.nanosec = stamp % 1_000_000_000
    pub.publish(msg)
start = time.monotonic()
while time.monotonic() - start < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
checks = {'old_capture_rejected': 990100 not in received,
          'fresh_capture_forwarded_after_second_search': 990101 in received}
print(json.dumps(checks), flush=True)
n.destroy_node()
rclpy.shutdown()
raise SystemExit(0 if all(checks.values()) else 1)
