"""A publisher discovered after startup gets a subscription exactly once."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.qos import QoSProfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('graph_probe', ROOT / 'verify/component/graph_probe.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_late_discovery_subscribes_without_duplicating_existing_profile():
    known = {}
    endpoints = []
    node = SimpleNamespace(
        topics=['/late'], publishers_of={}, subscriptions_made=[],
        subscribed_profiles=set(),
        get_topic_names_and_types=lambda: list(known.items()),
        count_publishers=lambda topic: len(endpoints),
        get_publishers_info_by_topic=lambda topic: endpoints,
        _make_callback=lambda topic: lambda message: None,
        create_subscription=Mock(),
    )
    module.Probe.discover(node)
    node.create_subscription.assert_not_called()
    known['/late'] = ['std_msgs/msg/Bool']
    endpoints.append(SimpleNamespace(qos_profile=QoSProfile(depth=1)))
    module.Probe.discover(node)
    module.Probe.discover(node)
    assert node.create_subscription.call_count == 1
    endpoints.append(SimpleNamespace(qos_profile=QoSProfile(depth=1)))
    module.Probe.discover(node)
    assert node.create_subscription.call_count == 1
    assert node.publishers_of['/late'] == 2
