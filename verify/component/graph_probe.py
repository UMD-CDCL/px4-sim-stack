#!/usr/bin/env python3
"""Report what a running ROS graph actually carries.

For each topic named on the command line: how many publishers it has, and
whether a message arrives inside the deadline. `ros2 topic echo` answers the
second question one topic at a time and takes a discovery wait for each, so a
graph of twenty topics costs a minute of waiting and hides which ones were
merely slow.

Prints one tab separated line per topic: name, publishers, and the verdict.

With `--count SECONDS` the verdict is instead how many messages arrived in
that window, so two streams can be compared over one window rather than
sampled one after the other. A stream sampled twice in a row cannot be told
from a stream that changed in between, which is how a race reads as a fault.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from rosidl_runtime_py.utilities import get_message

DISCOVERY_S = 3.0


class Probe(Node):
    def __init__(self, topics, deadline_s):
        super().__init__("verify_graph_probe")
        self.deadline_s = deadline_s
        self.seen = set()
        self.counts = {}
        self.publishers_of = {}
        self.subscriptions_made = []
        self.topics = topics

    def discover(self):
        known = dict(self.get_topic_names_and_types())
        for topic in self.topics:
            types = known.get(topic)
            self.publishers_of[topic] = self.count_publishers(topic)
            if not types:
                continue
            message = get_message(types[0])
            # Match what each publisher offers. A probe that asks for policies
            # of its own makes every publisher log an incompatibility, which
            # reads like a fault in the stack rather than in the probe. Only
            # reliability and durability are copied: a discovered profile also
            # carries policies that no subscription may be created with.
            for endpoint in self.get_publishers_info_by_topic(topic):
                offered = endpoint.qos_profile
                self.subscriptions_made.append(self.create_subscription(
                    message, topic, self._make_callback(topic),
                    QoSProfile(reliability=offered.reliability,
                               durability=offered.durability,
                               history=HistoryPolicy.KEEP_LAST, depth=1)))

    def _make_callback(self, topic):
        def callback(_message):
            self.seen.add(topic)
            self.counts[topic] = self.counts.get(topic, 0) + 1
        return callback


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topics", nargs="+")
    parser.add_argument("--deadline", type=float, default=10.0,
                        help="seconds to wait for the first message")
    parser.add_argument("--count", type=float, default=0.0, metavar="SECONDS",
                        help="listen this long and report messages seen "
                             "instead of the verdict")
    args = parser.parse_args()

    rclpy.init()
    probe = Probe(args.topics, args.deadline)

    # Discovery is not instant, and a subscription made before the publisher is
    # known never matches.
    end = time.monotonic() + DISCOVERY_S
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
    probe.discover()

    # Counting listens out the whole window. Stopping at the first message of
    # each topic would report one message everywhere.
    end = time.monotonic() + (args.count or args.deadline)
    while time.monotonic() < end and (
            args.count or len(probe.seen) < len(args.topics)):
        rclpy.spin_once(probe, timeout_sec=0.05)

    for topic in args.topics:
        publishers = probe.publishers_of.get(topic, 0)
        if args.count:
            print(f"{topic}\t{publishers}\t{probe.counts.get(topic, 0)}")
            continue
        if topic in probe.seen:
            verdict = "data"
        elif publishers:
            verdict = "silent"
        else:
            verdict = "no publisher"
        print(f"{topic}\t{publishers}\t{verdict}")

    probe.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
