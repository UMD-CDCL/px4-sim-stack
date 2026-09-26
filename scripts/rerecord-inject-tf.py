#!/usr/bin/env python3
"""Replace a rerecord bag's TF with timestamped UAS transforms from a sidecar run."""
import argparse
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from rclpy.serialization import deserialize_message, serialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from tf2_msgs.msg import TFMessage


def ros_time_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def tf_events(path: Path, first_stamp: int, uas: str):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    reader.set_filter(StorageFilter(topics=["/tf", "/tf_static"]))
    events, seen = [], set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        incoming = deserialize_message(data, TFMessage)
        retained = []
        for transform in incoming.transforms:
            parent, child = transform.header.frame_id, transform.child_frame_id
            if not (parent.startswith(uas) or child.startswith(uas) or parent == "fiducial" or child == "fiducial"):
                continue
            stamp = ros_time_ns(transform.header.stamp)
            key = (topic, parent, child, 0 if topic == "/tf_static" else stamp)
            if key in seen:
                continue
            seen.add(key)
            retained.append(transform)
        if retained:
            message = TFMessage()
            message.transforms = retained
            timestamp = first_stamp if topic == "/tf_static" else min(ros_time_ns(t.header.stamp) for t in retained)
            events.append((timestamp, topic, serialize_message(message)))
    return sorted(events, key=lambda event: event[0])


def source_metadata(path: Path):
    with path.open("rb") as handle:
        for schema, channel, _ in make_reader(handle).iter_messages():
            if channel.topic == "/tf":
                dynamic = (schema, channel)
            elif channel.topic == "/tf_static":
                static = (schema, channel)
    return dynamic, static


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--tf-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--uas", default="uas4")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    # Some ROS bags carry /tf_static at log time zero.  It is valid as a
    # static transform, but using that record as the output timestamp makes a
    # flight bag appear centuries long.  Place injected static transforms at
    # the first real ROS timestamp instead.
    with args.input.open("rb") as handle:
        first = min(
            message.log_time
            for _, _, message in make_reader(handle).iter_messages()
            if message.log_time > 1_000_000_000_000_000
        )
    dynamic, static = source_metadata(args.input)
    channels_by_topic = {"/tf": dynamic, "/tf_static": static}
    events = tf_events(args.tf_source, first, f"{args.uas}_")
    if not events:
        raise SystemExit("no UAS TF transforms found in sidecar source")

    schemas, channels = {}, {}
    def register(writer, schema, channel):
        schema_key = None if schema is None else (schema.name, schema.encoding, schema.data)
        schema_id = 0 if schema_key is None else schemas.get(schema_key)
        if schema_key is not None and schema_id is None:
            schema_id = writer.register_schema(*schema_key)
            schemas[schema_key] = schema_id
        channel_key = (channel.topic, channel.message_encoding, schema_key, tuple(sorted(channel.metadata.items())))
        channel_id = channels.get(channel_key)
        if channel_id is None:
            channel_id = writer.register_channel(channel.topic, channel.message_encoding, schema_id, dict(channel.metadata))
            channels[channel_key] = channel_id
        return channel_id

    event_index = 0
    with args.input.open("rb") as source, args.output.open("wb") as destination:
        writer = Writer(destination)
        writer.start(profile="ros2", library="px4sim-rerecord-tf-inject")
        try:
            for schema, channel, message in make_reader(source).iter_messages():
                if channel.topic in channels_by_topic:
                    continue
                while event_index < len(events) and events[event_index][0] <= message.log_time:
                    stamp, topic, data = events[event_index]
                    tf_schema, tf_channel = channels_by_topic[topic]
                    writer.add_message(register(writer, tf_schema, tf_channel), log_time=stamp,
                                       publish_time=stamp, data=data)
                    event_index += 1
                writer.add_message(register(writer, schema, channel), log_time=message.log_time,
                                   publish_time=message.publish_time, sequence=message.sequence, data=message.data)
            while event_index < len(events):
                stamp, topic, data = events[event_index]
                tf_schema, tf_channel = channels_by_topic[topic]
                writer.add_message(register(writer, tf_schema, tf_channel), log_time=stamp,
                                   publish_time=stamp, data=data)
                event_index += 1
        finally:
            writer.finish()
    print(f"wrote {args.output} with {len(events)} UAS TF events")


if __name__ == "__main__":
    main()
