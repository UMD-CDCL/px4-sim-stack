#!/usr/bin/env python3
"""Build a timestamp-faithful, offline UAS rerecord input without ROS replay.

The MAVROS bag and video names carry their offsets from the same t=0.  Raw
image timestamps from the reference bag choose the video frames to infer; the
detector is never wall-clock paced.  The output contains MAVROS telemetry,
the TF/camera data needed by tf_loc, and fresh target detections.
"""
import argparse
import re
from pathlib import Path

import cv2
from cdcl_umd_msgs.msg import TargetBox, TargetBoxArray
from mcap.reader import make_reader
from mcap.writer import Writer
from rclpy.serialization import serialize_message
from ultralytics import YOLO


OFFSET_RE = re.compile(r"^t\+(\d+)s_(\d+)ms-")


def offset_ns(path: Path) -> int:
    match = OFFSET_RE.match(path.name)
    if not match:
        raise ValueError(f"missing t+<seconds>s_<milliseconds>ms prefix: {path}")
    return int(match.group(1)) * 1_000_000_000 + int(match.group(2).ljust(3, "0")[:3]) * 1_000_000


def all_messages(path: Path):
    with path.open("rb") as handle:
        yield from make_reader(handle).iter_messages()


def reference_data(path: Path, namespace: str):
    image_topic = f"{namespace}/image"
    keep = {"/tf", "/tf_static", f"{namespace}/camera/camera_info"}
    detections = f"{namespace}/target_detections"
    frame_stamps, events, detection_channel = [], [], None
    for schema, channel, message in all_messages(path):
        if channel.topic == image_topic:
            frame_stamps.append(message.log_time)
        elif channel.topic in keep:
            events.append((message.log_time, schema, channel, message))
        elif channel.topic == detections and detection_channel is None:
            detection_channel = (schema, channel)
    if detection_channel is None:
        raise ValueError(f"{path} does not provide schema for {detections}")
    return frame_stamps, sorted(events, key=lambda event: event[0]), detection_channel


def first_log_time(path: Path) -> int:
    first = None
    for _, _, message in all_messages(path):
        first = message.log_time if first is None else min(first, message.log_time)
    if first is None:
        raise ValueError(f"empty MAVROS bag: {path}")
    return first


def make_detection(result, frame, sequence: int, stamp_ns: int, uas: int):
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise RuntimeError("failed to JPEG encode selected video frame")
    message = TargetBoxArray()
    message.seq = sequence
    message.system_id = uas
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.source_img.header.stamp = message.header.stamp
    message.source_img.format = "jpeg"
    message.source_img.data = jpeg.tobytes()
    for box in result.boxes:
        x1, y1, x2, y2 = (float(value) for value in box.xyxy[0])
        target = TargetBox()
        target.data_source_id = sequence
        target.target_bbox.center.position.x = (x1 + x2) / 2.0
        target.target_bbox.center.position.y = (y1 + y2) / 2.0
        target.target_bbox.size_x = x2 - x1
        target.target_bbox.size_y = y2 - y1
        target.detection_class = result.names[int(box.cls[0])]
        target.detection_confidence = float(box.conf[0])
        message.uav_target_boxes.append(target)
    return message


def infer(video: Path, stamps: list[int], video_start_ns: int, model_path: str,
          batch_size: int, uas: int):
    capture = cv2.VideoCapture(str(video))
    fps = capture.get(cv2.CAP_PROP_FPS)
    count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if fps <= 0 or count <= 0:
        raise RuntimeError(f"could not read video timing from {video}")
    selected = []
    seen = set()
    for stamp in stamps:
        index = round((stamp - video_start_ns) * fps / 1_000_000_000)
        if 0 <= index < count and index not in seen:
            selected.append((index, stamp))
            seen.add(index)
    if not selected:
        raise ValueError("no raw-image timestamps overlap the video")
    model = YOLO(model_path)
    output = []
    cursor = -1
    batch_frames, batch_stamps = [], []

    def flush():
        if not batch_frames:
            return
        for result, frame, stamp in zip(
            model(batch_frames, imgsz=960, device="cpu", verbose=False),
            batch_frames, batch_stamps,
        ):
            output.append((stamp, make_detection(result, frame, len(output), stamp, uas)))
        batch_frames.clear()
        batch_stamps.clear()

    for wanted, stamp in selected:
        while cursor < wanted:
            ok, frame = capture.read()
            cursor += 1
            if not ok:
                raise RuntimeError(f"video ended while reading frame {wanted}")
        batch_frames.append(frame)
        batch_stamps.append(stamp)
        if len(batch_frames) == batch_size:
            flush()
            print(f"inferred {len(output)}/{len(selected)} frames", flush=True)
    flush()
    capture.release()
    print(f"inferred {len(output)}/{len(selected)} frames at {fps:.6f} Hz video timing", flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mavros", type=Path, required=True)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True,
                        help="bag containing /tf, camera_info, and raw image timestamps")
    parser.add_argument("--model", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--namespace", default="/uas4")
    parser.add_argument("--uas", type=int, default=4)
    parser.add_argument("--batch-size", type=int, default=4)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"refusing to overwrite {args.output}")
    if args.batch_size < 1:
        raise SystemExit("--batch-size must be positive")

    raw_stamps, reference_events, detection_schema = reference_data(args.reference, args.namespace)
    bag_zero_ns = first_log_time(args.mavros) - offset_ns(args.mavros)
    video_start_ns = bag_zero_ns + offset_ns(args.video)
    eligible = [stamp for stamp in raw_stamps if stamp >= video_start_ns]
    print(f"t=0={bag_zero_ns}; raw-image frames={len(raw_stamps)}; eligible={len(eligible)}", flush=True)
    generated = infer(args.video, eligible, video_start_ns, args.model, args.batch_size, args.uas)

    generated_index = 0
    reference_index = 0
    schemas, channels = {}, {}
    target_schema, target_channel = detection_schema
    output_topic = f"{args.namespace}/target_detections"

    def register(writer, schema, channel):
        schema_id = 0
        schema_key = None
        if schema is not None:
            schema_key = (schema.name, schema.encoding, schema.data)
            schema_id = schemas.get(schema_key)
            if schema_id is None:
                schema_id = writer.register_schema(schema.name, schema.encoding, schema.data)
                schemas[schema_key] = schema_id
        channel_key = (channel.topic, channel.message_encoding, schema_key,
                       tuple(sorted(channel.metadata.items())))
        channel_id = channels.get(channel_key)
        if channel_id is None:
            channel_id = writer.register_channel(channel.topic, channel.message_encoding,
                                                 schema_id, dict(channel.metadata))
            channels[channel_key] = channel_id
        return channel_id

    def write_generated(writer, stamp, message):
        channel_id = register(writer, target_schema, target_channel)
        writer.add_message(channel_id, log_time=stamp, publish_time=stamp,
                           data=serialize_message(message))

    with args.output.open("wb") as handle:
        writer = Writer(handle)
        writer.start(profile="ros2", library="px4sim-rerecord-programmatic")
        try:
            for schema, channel, message in all_messages(args.mavros):
                while reference_index < len(reference_events) and reference_events[reference_index][0] <= message.log_time:
                    _, ref_schema, ref_channel, ref_message = reference_events[reference_index]
                    writer.add_message(register(writer, ref_schema, ref_channel),
                                       log_time=ref_message.log_time, publish_time=ref_message.publish_time,
                                       sequence=ref_message.sequence, data=ref_message.data)
                    reference_index += 1
                while generated_index < len(generated) and generated[generated_index][0] <= message.log_time:
                    write_generated(writer, *generated[generated_index])
                    generated_index += 1
                writer.add_message(register(writer, schema, channel), log_time=message.log_time,
                                   publish_time=message.publish_time, sequence=message.sequence, data=message.data)
            while reference_index < len(reference_events):
                _, schema, channel, message = reference_events[reference_index]
                writer.add_message(register(writer, schema, channel), log_time=message.log_time,
                                   publish_time=message.publish_time, sequence=message.sequence, data=message.data)
                reference_index += 1
            while generated_index < len(generated):
                write_generated(writer, *generated[generated_index])
                generated_index += 1
        finally:
            writer.finish()
    print(f"wrote {args.output} with {len(generated)} timestamped detections", flush=True)


if __name__ == "__main__":
    main()
