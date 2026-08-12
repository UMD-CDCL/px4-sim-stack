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

Parameters
    host, port     the MQTT broker
    topic          MQTT topic to subscribe to
    bbox_format    ltrb (default) or ltwh
    frame_id       frame_id on the published messages

Topics
    /perception/detections   vision_msgs/Detection2DArray
"""

from __future__ import annotations

import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
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
        self.declare_parameter("frame_id", "camera_optical_frame")

        self.bbox_format = self.get_parameter("bbox_format").value
        self.frame_id = self.get_parameter("frame_id").value
        self.count = 0
        self.malformed = 0

        self.pub = self.create_publisher(
            Detection2DArray, "/perception/detections", DETECTION_QOS
        )

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

        array = Detection2DArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.frame_id
        array.detections = detections
        self.pub.publish(array)
        self.count += len(detections)

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
        self.get_logger().info(
            f"{self.count} detections in the last minute, {self.malformed} unparsed"
        )
        self.count = 0
        self.malformed = 0

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
