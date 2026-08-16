#!/usr/bin/env python3
"""Publish DeepStream detections as ROS 2 messages.

DeepStream and ROS share one MQTT topic and nothing else. This node is the
only place that knows the DeepStream payload format, so a change there costs
one file.

nvmsgbroker emits two shapes, set by payload-type in the msgconv config: the
minimal schema (payload-type=1), one message per frame with
"objects": ["<id>|<left>|<top>|<right>|<bottom>|<class>", ...], and the full
schema (payload-type=0), one message per object with a nested bbox. This
node reads both. bbox_format selects the field order, which differs across
DeepStream releases.

Each payload carries "@timestamp", set when the frame enters the DeepStream
pipeline, before inference. That instant becomes the header stamp, so the
localizer can look up the drone pose for the right moment. The arrival time
would fold in decode, inference and transport. Measure the remaining offset
with scripts/measure-latency.py and feed it back through time_offset.

DeepStream reports boxes in its own coordinate space, which can differ from
the image resolution without any error message. The boxes are scaled here
into the size the live CameraInfo reports. When the two agree the scale is 1
and nothing happens. scripts/check-annotation-scale.py finds the factor when
they do not.

Each payload's sensorId routes it to that camera's topic, so a consumer can
pick one camera without filtering.

Parameters
    host, port     the MQTT broker
    topic          MQTT topic to subscribe to
    bbox_format    ltrb (default) or ltwh
    source_width   the coordinate space DeepStream reports in. 0 disables
    source_height  scaling.
    source_size_overrides
                   per camera, as "camera=WxH". Corrects one camera without
                   disturbing the other.
    sensor_ids     DeepStream sensorId values, one per camera
    sensor_frames  the optical frame for each sensor id, same order
    time_offset    seconds added to the payload timestamp, for calibration

Topics
    /perception/<camera>/detections   vision_msgs/Detection2DArray
"""

from __future__ import annotations

import datetime
import json

import paho.mqtt.client as mqtt
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from sim_bridge.runtime import BEST_EFFORT, now_s, spin

# ------------------------------------------------------------------- tunables
# Drop a payload older than this, in seconds.
MAX_AGE_S = 2.0


class DetectionsBridge(Node):
    def __init__(self) -> None:
        super().__init__("detections_bridge")

        self.declare_parameter("host", "message-bus")
        self.declare_parameter("port", 1883)
        self.declare_parameter("topic", "perception/detections")
        self.declare_parameter("bbox_format", "ltrb")
        self.declare_parameter("source_width", 0)
        self.declare_parameter("source_height", 0)
        self.declare_parameter("source_size_overrides", [""])
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("sensor_ids", ["nadir", "gimbal"])
        self.declare_parameter("sensor_frames",
                               ["nadir_camera_optical_frame",
                                "gimbal_camera_optical_frame"])

        self.bbox_format = self.get_parameter("bbox_format").value
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.count = 0
        self.malformed = 0
        self.unrouted = 0
        self.no_stamp = 0
        self.stale = 0
        self.lag_sum = 0.0
        self.lag_n = 0

        sensor_ids = list(self.get_parameter("sensor_ids").value)
        frames = list(self.get_parameter("sensor_frames").value)
        self.frame_for = dict(zip(sensor_ids, frames))
        # Payloads without a sensorId belong to the primary camera.
        self.primary = sensor_ids[0] if sensor_ids else None

        self.source_size = (int(self.get_parameter("source_width").value),
                            int(self.get_parameter("source_height").value))
        self.source_override: dict[str, tuple[int, int]] = {}
        for entry in self.get_parameter("source_size_overrides").value:
            entry = str(entry).strip()
            if not entry or "=" not in entry:
                continue
            name, _, size = entry.partition("=")
            try:
                w, h = (int(v) for v in size.lower().split("x", 1))
            except ValueError:
                self.get_logger().warn(f"cannot read source size override {entry!r}")
                continue
            self.source_override[name.strip()] = (w, h)
            self.get_logger().info(f"{name.strip()}: DeepStream reports in {w}x{h}")

        # Live image size for each camera. CameraInfo is the authority: it is
        # what the image panels and the projection math use, and reading it
        # means a camera resolution change needs no edit here.
        self.image_size: dict[str, tuple[int, int]] = {}
        self.scale_logged: set[str] = set()
        for name in sensor_ids:
            self.create_subscription(
                CameraInfo, f"/camera/{name}/camera_info",
                lambda msg, n=name: self._on_camera_info(n, msg),
                qos_profile_sensor_data)

        self.publisher_for = {
            name: self.create_publisher(
                Detection2DArray, f"/perception/{name}/detections", BEST_EFFORT)
            for name in sensor_ids
        }

        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        topic = self.get_parameter("topic").value

        # paho-mqtt 2.x wants an explicit callback API version. Ubuntu 24.04
        # ships 1.x, which has neither the enum nor the extra callback
        # arguments. Support both, because the stack must build from apt.
        if hasattr(mqtt, "CallbackAPIVersion"):
            self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        else:
            self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.user_data_set(topic)
        # A broker that is not up yet is normal at start. Retry rather than
        # exit.
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()

        self.get_logger().info(f"reading mqtt://{host}:{port} topic '{topic}'")
        self.create_timer(60.0, self._report)

    def _on_camera_info(self, sensor: str, msg: CameraInfo) -> None:
        # The size almost never changes, so most calls return here.
        size = (msg.width, msg.height)
        if self.image_size.get(sensor) == size:
            return
        self.image_size[sensor] = size

    # ------------------------------------------------------------------- mqtt
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        # paho 1.x passes an int rc, paho 2.x a ReasonCode. Both compare to 0.
        if reason_code != 0:
            self.get_logger().warn(f"broker refused the connection: {reason_code}")
            return
        client.subscribe(userdata, qos=0)
        self.get_logger().info(f"subscribed to '{userdata}'")

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.malformed += 1
            return

        detections = self._parse(payload)
        if detections is None:
            self.malformed += 1
            return

        stamp, lag = self._frame_time(payload)
        if stamp is None:
            # Without a frame time the localizer cannot pick the right pose,
            # and a wrong pose is worse than no answer.
            self.no_stamp += 1
            return
        if lag is not None and lag > MAX_AGE_S:
            self.stale += 1
            return

        sensor = str(payload.get("sensorId") or "") or self.primary
        if sensor not in self.publisher_for:
            self.unrouted += 1
            if self.unrouted in (1, 500):
                self.get_logger().warn(
                    f"detections tagged sensorId={sensor!r}, which is not in "
                    f"sensor_ids {list(self.publisher_for)}. They are dropped. "
                    f"Check the id in the deepstream msgconv config.")
            return

        self._rescale(sensor, detections)
        array = Detection2DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_for[sensor]
        array.detections = detections
        self.publisher_for[sensor].publish(array)
        self.count += len(detections)

    # -------------------------------------------------------------- timestamp
    def _frame_time(self, payload: dict):
        """The instant the frame entered DeepStream, as a ROS stamp.

        Returns (stamp, lag_seconds). lag is how far behind the clock the
        frame is, which is the pipeline latency.
        """
        raw = payload.get("@timestamp")
        if not raw:
            return None, None
        try:
            # DeepStream writes RFC 3339 with a Z suffix, which fromisoformat
            # handles once Z becomes +00:00.
            when = datetime.datetime.fromisoformat(str(raw).replace("Z", "+00:00"))
        except ValueError:
            return None, None

        seconds = when.timestamp() + self.time_offset
        lag = now_s(self) - seconds
        self.lag_sum += lag
        self.lag_n += 1

        stamp = Time()
        stamp.sec = int(seconds)
        stamp.nanosec = int(round((seconds - int(seconds)) * 1e9))
        # Rounding can carry into the next second.
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        return stamp, lag

    # ------------------------------------------------------------------ parse
    def _parse(self, payload: dict) -> list[Detection2D] | None:
        if "objects" in payload:  # minimal schema
            out = []
            for entry in payload["objects"]:
                det = self._from_minimal(entry)
                if det is not None:
                    out.append(det)
            return out

        if "object" in payload:  # full schema, one object
            det = self._from_full(payload["object"])
            return [det] if det is not None else []

        return None

    def _from_minimal(self, entry: str) -> Detection2D | None:
        parts = str(entry).split("|")
        if len(parts) < 6:
            return None
        try:
            a, b, c, d = (float(v) for v in parts[1:5])
        except ValueError:
            return None
        return self._detection(parts[0], parts[5], a, b, c, d)

    def _from_full(self, obj: dict) -> Detection2D | None:
        box = obj.get("bbox") or {}
        try:
            left = float(box["topleftx"])
            top = float(box["toplefty"])
            right = float(box["bottomrightx"])
            bottom = float(box["bottomrighty"])
        except (KeyError, TypeError, ValueError):
            return None
        # The full schema always gives corners, whatever bbox_format says.
        label = next((k for k in obj if k not in
                      ("id", "speed", "bbox", "location", "coordinate", "orientation")),
                     "object")
        return self._detection(str(obj.get("id", "-1")), label,
                               left, top, right, bottom, corners=True)

    def _rescale(self, sensor: str, detections: list) -> None:
        """Move boxes from DeepStream's coordinate space into the image's."""
        src_w, src_h = self.source_override.get(sensor, self.source_size)
        if src_w <= 0 or src_h <= 0:
            return
        size = self.image_size.get(sensor)
        if size is None:
            return
        sx, sy = size[0] / src_w, size[1] / src_h
        same = abs(sx - 1.0) < 1e-6 and abs(sy - 1.0) < 1e-6
        if sensor not in self.scale_logged:
            self.scale_logged.add(sensor)
            if same:
                self.get_logger().info(
                    f"{sensor}: detections and image are both {size[0]}x{size[1]}")
            else:
                self.get_logger().warn(
                    f"{sensor}: DeepStream reports in {src_w}x{src_h} but the "
                    f"image is {size[0]}x{size[1]}. Scaling boxes by "
                    f"{sx:.3f}x{sy:.3f}. Check [tiled-display] in the "
                    f"deepstream-app config if that is not deliberate.")
        if same:
            return
        for det in detections:
            det.bbox.center.position.x *= sx
            det.bbox.center.position.y *= sy
            det.bbox.size_x *= sx
            det.bbox.size_y *= sy

    def _detection(self, track_id: str, label: str,
                   a: float, b: float, c: float, d: float,
                   corners: bool | None = None) -> Detection2D:
        use_corners = self.bbox_format == "ltrb" if corners is None else corners
        left, top = a, b
        width = (c - a) if use_corners else c
        height = (d - b) if use_corners else d

        det = Detection2D()
        det.id = track_id
        det.bbox = BoundingBox2D()
        det.bbox.center.position.x = left + width / 2.0
        det.bbox.center.position.y = top + height / 2.0
        det.bbox.center.theta = 0.0
        det.bbox.size_x = abs(width)
        det.bbox.size_y = abs(height)

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = label
        # The minimal schema carries no confidence. Report 1.0 rather than
        # invent a number that a planner might threshold on.
        hypothesis.hypothesis.score = 1.0
        det.results.append(hypothesis)
        return det

    def _report(self) -> None:
        lag = (self.lag_sum / self.lag_n * 1000.0) if self.lag_n else float("nan")
        self.get_logger().info(
            f"{self.count} detections in the last minute, mean pipeline lag "
            f"{lag:.0f} ms, {self.malformed} unparsed, {self.no_stamp} without a "
            f"frame time, {self.stale} stale"
        )
        self.count = self.malformed = self.no_stamp = self.stale = 0
        self.lag_sum = 0.0
        self.lag_n = 0

    def destroy_node(self) -> bool:
        self.client.loop_stop()
        self.client.disconnect()
        return super().destroy_node()


def main() -> None:
    spin(DetectionsBridge)


if __name__ == "__main__":
    main()
