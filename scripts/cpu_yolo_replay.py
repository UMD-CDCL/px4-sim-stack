#!/usr/bin/env python3
"""Replay RGB frames through YOLO and publish native TargetBoxArray detections."""
import argparse
import time

import cv2
import rclpy
from builtin_interfaces.msg import Time
from cdcl_umd_msgs.msg import TargetBox, TargetBoxArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ultralytics import YOLO


def stamp(ns: int) -> Time:
    value = Time()
    value.sec = ns // 1_000_000_000
    value.nanosec = ns % 1_000_000_000
    return value


class ReplayDetector(Node):
    def __init__(self, args):
        super().__init__('rerecord_yolo26_detector')
        self.args = args
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(TargetBoxArray, args.topic, qos)
        self.model = YOLO(args.model)

    def run(self):
        cap = cv2.VideoCapture(self.args.video)
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        step = max(1, round(fps / self.args.rate))
        index = int(self.args.skip_seconds * fps)
        cap.set(cv2.CAP_PROP_POS_FRAMES, index)
        first_index = index
        sequence = 0
        while rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                break
            if index % step:
                index += 1
                continue
            pts_ns = self.args.start_ns + int((self.args.offset + index / fps) * 1e9)
            target_wall = self.args.wall_start + (0 if self.args.start_now else self.args.offset) + (index - first_index) / fps
            delay = target_wall - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            result = self.model(frame, imgsz=960, device='cpu', verbose=False)[0]
            ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok:
                index += 1
                continue
            message = TargetBoxArray()
            message.seq = sequence
            message.system_id = self.args.uas
            message.header.stamp = stamp(pts_ns)
            message.source_img.header.stamp = message.header.stamp
            message.source_img.format = 'jpeg'
            message.source_img.data = encoded.tobytes()
            for box in result.boxes:
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                target = TargetBox()
                target.data_source_id = sequence
                target.target_bbox.center.position.x = (x1 + x2) / 2
                target.target_bbox.center.position.y = (y1 + y2) / 2
                target.target_bbox.size_x = x2 - x1
                target.target_bbox.size_y = y2 - y1
                target.detection_class = self.model.names[int(box.cls[0])]
                target.detection_confidence = float(box.conf[0])
                message.uav_target_boxes.append(target)
            self.publisher.publish(message)
            sequence += 1
            index += 1
        cap.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--video', required=True)
    parser.add_argument('--model', required=True)
    parser.add_argument('--start-ns', type=int, required=True)
    parser.add_argument('--offset', type=float, required=True)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--uas', type=int, default=4)
    parser.add_argument('--topic', default='/uas4/target_detections')
    parser.add_argument('--start-now', action='store_true',
                        help='begin processing immediately while retaining video timestamps')
    parser.add_argument('--skip-seconds', type=float, default=0.0,
                        help='seek this far into the video before processing')
    args = parser.parse_args()
    args.wall_start = time.monotonic()
    rclpy.init()
    node = ReplayDetector(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
