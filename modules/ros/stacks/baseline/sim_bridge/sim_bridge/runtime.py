"""Plumbing every sim_bridge node shares: QoS profiles, TF, Foxglove, spin."""

from __future__ import annotations

import json

import rclpy
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from tf2_ros import Buffer, TransformListener

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# The mavros and camera topics publish best effort, and a reliable
# subscription to a best effort publisher receives nothing.
BEST_EFFORT = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=10)
# Keep only the newest frame, for a consumer that never wants an older one.
NEWEST_ONLY = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)


def now_s(node) -> float:
    return node.get_clock().now().nanoseconds / 1e9


def tf_buffer(node, cache_time=None) -> Buffer:
    """A TF buffer whose listener runs on its own spin thread. Without the
    thread a lookup that waits for a transform blocks the callback that
    would deliver it. The listener is kept on the node so it outlives this
    call."""
    buffer = Buffer(cache_time=cache_time)
    node.tf_listener = TransformListener(buffer, node, spin_thread=True)
    return buffer


def require_foxglove(node, consequence: str) -> bool:
    """True when foxglove_msgs is installed. Otherwise it logs what the
    absence costs and returns False."""
    if HAVE_FOXGLOVE:
        return True
    node.get_logger().error(
        f"foxglove_msgs is missing, so {consequence}. "
        f"Install ros-$ROS_DISTRO-foxglove-msgs.")
    return False


def geojson_publisher(node, topic: str, consequence: str, qos=LATCHED):
    """A GeoJSON publisher for the Map panel, or None when foxglove_msgs is
    absent. Every Map panel topic in this package comes from here."""
    if not require_foxglove(node, consequence):
        return None
    return node.create_publisher(GeoJSON, topic, qos)


def publish_features(publisher, features: list) -> None:
    """Publish one GeoJSON FeatureCollection. An empty list clears the
    panel, which is how a latched marker is taken back."""
    if publisher is None:
        return
    msg = GeoJSON()
    msg.geojson = json.dumps({"type": "FeatureCollection",
                              "features": features})
    publisher.publish(msg)


def spin(node_class) -> None:
    """Run one node until Ctrl-C. A node that holds a client or a decoder
    releases it by overriding destroy_node."""
    rclpy.init()
    node = node_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
