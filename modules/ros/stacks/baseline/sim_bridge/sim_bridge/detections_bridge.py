#!/usr/bin/env python3
"""Publish DeepStream detections as ROS 2 messages.

DeepStream and ROS do not share a process, a language or a release schedule.
They share one MQTT topic. This node is the only place that knows the DeepStream
payload format, so a change there costs one file.

DeepStream's nvmsgbroker can emit two shapes, set by payload-type in the
message converter config:

  payload-type=1, the minimal schema, one message for each frame:
      {"version":"4.0", "id":"<frame>", "@timestamp":"...",
       "sensorId":"gimbal",
       "objects":["<id>|<left>|<top>|<right>|<bottom>|<class>", ...]}

  payload-type=0, the full schema, one message for each object, with a nested
      "object" element that holds a bbox.

This node reads both. The field order inside an object string is not identical
across DeepStream releases, so bbox_format selects it.

The timestamp matters as much as the boxes
-----------------------------------------
Each payload carries `@timestamp`, which DeepStream sets when the frame enters
its pipeline, before inference. That instant is what the header carries. Using
the arrival time instead would fold in decode, inference and transport, and the
localizer would then look up the drone pose for the wrong moment. Measured on
this stack, DeepStream's timestamp trails true capture by about 16 ms with
about 6 ms of jitter, so the remaining error is small and, more usefully,
does not grow with GPU load.

`scripts/measure-latency.py` measures that offset against the frame clock the
simulator publishes on `video/frames/<stream>`. Feed the result back through
`time_offset` if you want the last few milliseconds.

Which pixels the boxes are counted in
-------------------------------------
DeepStream reports boxes in its own coordinate space, and that space is set by
`[tiled-display]` width and height in the deepstream-app config. Not by
`[streammux]`, and it applies even when tiled display is disabled. Leave those
keys out and deepstream-app uses its built-in 1280x720, whatever the camera
sends.

That is a quiet failure. With 1080p cameras and no tiled-display size, every
box arrived at two thirds of its true position and size: still a plausible
looking box, on the wrong part of the image, and no error anywhere.

So the boxes are scaled here, into the size the live CameraInfo reports. When
the two agree the scale is 1 and nothing happens, which is the normal case;
when they disagree the boxes still land in the right place.

The correction is per camera. Each camera has its own DeepStream pipeline at
its own resolution, so the two should already agree and the scale should be 1;
`source_size_overrides` exists because the failure it corrects is silent, and
naming the camera means one can be fixed without disturbing the other.

The scale factor is logged for each camera the first time it is used. If it is
not 1.0, something has changed and it is worth knowing why.
`scripts/check-annotation-scale.py` draws the boxes on a live frame and reports
the factor that would fit, which is the quickest way to find out.

Parameters
    host, port     the MQTT broker
    topic          MQTT topic to subscribe to
    bbox_format    ltrb (default) or ltwh
    source_width   the coordinate space DeepStream reports in
    source_height
    source_size_overrides
                   per camera, as "camera=WxH". Corrects one camera without
                   disturbing the other.
    frame_id       frame_id on the published messages
    time_offset    seconds added to the payload timestamp, for calibration
    max_age        drop a payload older than this, in seconds

Topics
    /perception/detections   vision_msgs/Detection2DArray
"""

from __future__ import annotations

import datetime
import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

DETECTION_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class DetectionsBridge(Node):
    def __init__(self) -> None:
        super().__init__("detections_bridge")

        self.declare_parameter("host", "message-bus")
        self.declare_parameter("port", 1883)
        self.declare_parameter("topic", "perception/detections")
        self.declare_parameter("bbox_format", "ltrb")
        # The coordinate space DeepStream reports boxes in. 0 means "assume it
        # matches the image", which disables scaling.
        self.declare_parameter("source_width", 0)
        self.declare_parameter("source_height", 0)
        # Per camera overrides, as "camera=WxH". The two sources in one
        # DeepStream pipeline do not always report in the same space, so a
        # single global size cannot always be right. Anything not named here
        # falls back to source_width and source_height.
        self.declare_parameter("source_size_overrides", [""])
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("max_age", 2.0)
        # DeepStream runs a pipeline for each camera and tags each
        # payload with sensorId. These two lists map that id to the camera's
        # optical frame, and detections are republished per camera so a
        # consumer can pick one without filtering. The first entry is the
        # primary camera, which also keeps the plain /perception/detections
        # topic that the localizer and the older layout expect.
        # The unqualified /perception/detections topic, which carried the
        # primary camera's detections before every stage became per camera.
        # Off by default: with it on, the primary camera's detections appear on
        # two topics, and anything left on the old default subscribes to a feed
        # that silently holds one camera out of two.
        self.declare_parameter("publish_unqualified", False)
        self.declare_parameter("sensor_ids", ["nadir", "gimbal"])
        self.declare_parameter("sensor_frames",
                               ["nadir_camera_optical_frame",
                                "gimbal_camera_optical_frame"])

        self.bbox_format = self.get_parameter("bbox_format").value
        self.frame_id = self.get_parameter("frame_id").value
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.max_age = float(self.get_parameter("max_age").value)
        self.count = 0
        self.malformed = 0
        self.unrouted = 0
        self.no_stamp = 0
        self.stale = 0
        self.lag_sum = 0.0
        self.lag_n = 0

        ids = list(self.get_parameter("sensor_ids").value)
        frames = list(self.get_parameter("sensor_frames").value)
        self.frame_for = dict(zip(ids, frames))
        self.primary = ids[0] if ids else None

        self.publish_unqualified = bool(self.get_parameter("publish_unqualified").value)
        self.pub = (self.create_publisher(
            Detection2DArray, "/perception/detections", DETECTION_QOS)
            if self.publish_unqualified else None)
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
        # Live image size for each camera, from CameraInfo, and the scale that
        # follows from it. CameraInfo is the authority: it is what the image
        # panels and the projection maths use.
        self.image_size: dict[str, tuple[int, int]] = {}
        self.scale_logged: set[str] = set()

        self.per_camera = {
            name: self.create_publisher(
                Detection2DArray, f"/perception/{name}/detections", DETECTION_QOS)
            for name in ids
        }

        # CameraInfo says how big the image really is. Reading it rather than
        # trusting a configured number means a camera resolution change needs
        # no edit here.
        for name in ids:
            self.create_subscription(
                CameraInfo, f"/camera/{name}/camera_info",
                lambda msg, n=name: self.image_size.__setitem__(n, (msg.width, msg.height)),
                qos_profile_sensor_data)

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
        # A broker that is not up yet is normal at start. Retry rather than exit.
        self.client.connect_async(host, port, keepalive=30)
        self.client.loop_start()

        self.get_logger().info(f"reading mqtt://{host}:{port} topic '{topic}'")
        self.create_timer(60.0, self._report)

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
            # Without a frame time the localizer cannot pick the right pose, and
            # a wrong pose is worse than no answer. Drop it and say so.
            self.no_stamp += 1
            return
        if lag is not None and lag > self.max_age:
            self.stale += 1
            return

        sensor = str(payload.get("sensorId") or "") or self.primary
        self._rescale(sensor, detections)
        array = Detection2DArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_for.get(sensor, self.frame_id)
        array.detections = detections

        if sensor in self.per_camera:
            self.per_camera[sensor].publish(array)
        # A camera with no publisher of its own has nowhere else to go, so it
        # falls back to the unqualified topic when that is enabled at all.
        if self.pub is not None and (sensor == self.primary
                                     or sensor not in self.per_camera):
            self.pub.publish(array)
        elif sensor not in self.per_camera:
            self.unrouted += 1
            if self.unrouted in (1, 500):
                self.get_logger().warn(
                    f"detections tagged sensorId={sensor!r}, which is not in "
                    f"sensor_ids {list(self.per_camera)}. They are dropped. "
                    f"Check the id in the deepstream msgconv config.")
        self.count += len(detections)

    # -------------------------------------------------------------- timestamp
    def _frame_time(self, payload: dict):
        """The instant the frame entered DeepStream, as a ROS stamp.

        Returns (stamp, lag_seconds). lag is how far behind the clock the frame
        is, which is the pipeline latency and is worth watching.
        """
        raw = payload.get("@timestamp")
        if not raw:
            return None, None
        try:
            # DeepStream writes RFC 3339 with a Z suffix and millisecond
            # precision, which fromisoformat handles once Z becomes +00:00.
            when = datetime.datetime.fromisoformat(str(raw).replace("Z", "+00:00"))
        except ValueError:
            return None, None

        seconds = when.timestamp() + self.time_offset
        now = self.get_clock().now()
        lag = now.nanoseconds / 1e9 - seconds
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
        if sensor not in self.scale_logged:
            self.scale_logged.add(sensor)
            if abs(sx - 1.0) < 1e-6 and abs(sy - 1.0) < 1e-6:
                self.get_logger().info(
                    f"{sensor}: detections and image are both {size[0]}x{size[1]}")
            else:
                self.get_logger().warn(
                    f"{sensor}: DeepStream reports in {src_w}x{src_h} but the "
                    f"image is {size[0]}x{size[1]}. Scaling boxes by "
                    f"{sx:.3f}x{sy:.3f}. Check [tiled-display] in the "
                    f"deepstream-app config if that is not deliberate.")
        if abs(sx - 1.0) < 1e-6 and abs(sy - 1.0) < 1e-6:
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
        # The minimal schema carries no confidence. Report 1.0 and say so here
        # rather than invent a number that a planner might threshold on.
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
    rclpy.init()
    node = DetectionsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
