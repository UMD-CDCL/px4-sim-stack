#!/usr/bin/env python3
"""Report what Foxglove sees, over the protocol Foxglove speaks.

A ROS graph and a panel are not the same view. A topic can carry data and
still never reach an operator: the bridge offers what the graph holds, a
layout draws what it names, and either can be wrong on its own. `ros2 topic`
answers the first question and says nothing about the second. This connects
to a Foxglove bridge exactly as the app does, so what it prints is what a
panel has to work with.

Prints one tab separated line per topic: name, schema, and the verdict.

    data        a message arrived inside the deadline
    advertised  the bridge offers the topic and sent nothing
    absent      the bridge does not offer the topic at all

With no topic named it lists every channel the bridge advertises.
"""

import argparse
import asyncio
import json
import struct
import sys

import websockets

# What a bridge answers to. The Rust server the SDK ships speaks the first,
# the older C++ one the second, and a client that offers both reaches either.
SUBPROTOCOLS = ["foxglove.sdk.v1", "foxglove.websocket.v1"]
MESSAGE_DATA = 1
# The bridge advertises as it discovers, so a client that asks at once sees a
# fraction of the graph.
DISCOVERY_S = 4.0
GEOJSON_SCHEMAS = ("foxglove_msgs/msg/GeoJSON", "foxglove_msgs/GeoJSON")


def geojson_of(payload: bytes) -> str:
    """The one string a GeoJSON message carries, out of its CDR encoding.

    Four bytes of encapsulation header, then the string's length and its
    bytes. This is what the Map panel parses, so reading it here is reading
    what the panel would draw.
    """
    length = struct.unpack_from("<I", payload, 4)[0]
    return payload[8:8 + length - 1].decode("utf-8", "replace")


class Bridge:
    """One connection to a Foxglove bridge."""

    def __init__(self, url, topics, deadline_s, show):
        self.url = url
        self.topics = topics
        self.deadline_s = deadline_s
        self.show = show
        self.channels = {}
        self.subscription = {}
        self.seen = {}
        self.payloads = {}

    async def run(self):
        async with websockets.connect(
                self.url, subprotocols=SUBPROTOCOLS, max_size=None) as socket:
            await self.discover(socket)
            wanted = self.topics or sorted(
                channel["topic"] for channel in self.channels.values())
            await self.subscribe(socket, wanted)
            await self.gather(socket, self.deadline_s, len(self.subscription))
            return wanted

    async def discover(self, socket):
        await self.gather(socket, DISCOVERY_S, None)

    async def gather(self, socket, seconds, enough):
        """Read frames until the time is up, or until every subscription has
        answered. A stage that always waits out its deadline hides which topic
        was slow and pays for the ones that were not."""
        loop = asyncio.get_event_loop()
        end = loop.time() + seconds
        while enough is None or len(self.seen) < enough:
            remaining = end - loop.time()
            if remaining <= 0:
                return
            try:
                self.take(await asyncio.wait_for(socket.recv(), remaining))
            except asyncio.TimeoutError:
                return

    def take(self, frame):
        if isinstance(frame, bytes):
            self.arrived(frame)
            return
        message = json.loads(frame)
        if message.get("op") == "advertise":
            for channel in message["channels"]:
                self.channels[channel["id"]] = channel
        elif message.get("op") == "unadvertise":
            for channel_id in message["channelIds"]:
                self.channels.pop(channel_id, None)

    async def subscribe(self, socket, wanted):
        self.subscription = {}
        subscriptions = []
        for channel in self.channels.values():
            if channel["topic"] not in wanted:
                continue
            identifier = len(subscriptions)
            self.subscription[identifier] = channel
            subscriptions.append({"id": identifier, "channelId": channel["id"]})
        if subscriptions:
            await socket.send(json.dumps({"op": "subscribe",
                                          "subscriptions": subscriptions}))

    def arrived(self, frame):
        if frame[0] != MESSAGE_DATA:
            return
        identifier = struct.unpack_from("<I", frame, 1)[0]
        channel = self.subscription.get(identifier)
        if channel is None:
            return
        topic = channel["topic"]
        self.seen[topic] = self.seen.get(topic, 0) + 1
        self.payloads[topic] = frame[13:]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topics", nargs="*")
    parser.add_argument("--url", default="ws://localhost:8765")
    parser.add_argument("--seconds", type=float, default=8.0,
                        help="how long to wait for a message on each topic")
    parser.add_argument("--show", action="append", default=[],
                        help="print this topic's GeoJSON as the panel parses it")
    arguments = parser.parse_args()

    bridge = Bridge(arguments.url, arguments.topics, arguments.seconds,
                    arguments.show)
    try:
        wanted = asyncio.run(bridge.run())
    except OSError as error:
        print(f"no Foxglove bridge on {arguments.url}: {error}", file=sys.stderr)
        return 1

    schema_of = {channel["topic"]: channel.get("schemaName", "")
                 for channel in bridge.channels.values()}
    for topic in wanted:
        if topic not in schema_of:
            verdict = "absent"
        elif bridge.seen.get(topic):
            verdict = "data"
        else:
            verdict = "advertised"
        print(f"{topic}\t{schema_of.get(topic, '')}\t{verdict}")

    for topic in arguments.show:
        payload = bridge.payloads.get(topic)
        if payload is None:
            print(f"# {topic} sent nothing to show", file=sys.stderr)
            continue
        if schema_of.get(topic) in GEOJSON_SCHEMAS:
            print(f"# {topic}\n{geojson_of(payload)}")
        else:
            print(f"# {topic} is {schema_of.get(topic)}, {len(payload)} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
