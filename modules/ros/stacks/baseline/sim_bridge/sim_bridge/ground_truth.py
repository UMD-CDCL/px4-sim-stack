#!/usr/bin/env python3
"""Publish where the targets actually are, for scoring detections against.

The simulator places targets from a scenario file, so their positions are
known exactly. This is the one node that reads a simulator file, and that is
safe: ground truth is evaluation data, nothing in the flight path reads it,
and on real hardware this node does not run.

Scenario poses are Gazebo world coordinates, x east and y north in meters.
PX4's local frame starts where the vehicle spawned, so the two line up.
origin_offset_xyz shifts them when they do not.

Publishes
    /ground_truth/markers      visualization_msgs/MarkerArray, blue spheres
    /ground_truth/truth_3d     vision_msgs/Detection3DArray, for the scorer
"""

from __future__ import annotations

import os
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from sim_bridge.verdicts import GROUND_TRUTH_COLOR, PERSON_SPHERE_DIAMETER

# ------------------------------------------------------------------- tunables
# Translucent, so a detection sphere at the same spot stays visible through it.
MARKER_ALPHA = 0.45
# Only entities whose name contains one of these count as findable targets.
# A scenario can also hold props that no detector should find.
TARGET_NAME_FILTERS = ["person", "casualty"]

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class GroundTruth(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth")

        self.declare_parameter("scenario_file", os.environ.get(
            "GROUND_TRUTH_FILE", "/scenes/scenarios/urban_casualties.yaml"))
        # The scenario file records where entities were asked to go. After
        # spawning, spawn_scenario.py reads back where Gazebo actually put
        # them and writes this file. Prefer it, because a model that settled
        # under gravity or failed to spawn makes the two differ.
        self.declare_parameter("resolved_file", os.environ.get(
            "RESOLVED_TRUTH_FILE", "/scenes/ground_truth_actual.yaml"))
        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("origin_offset_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("rate_hz", 1.0)

        self.reference = self.get_parameter("reference_frame").value
        self.offset = [float(x) for x in self.get_parameter("origin_offset_xyz").value]

        self.targets = self._load()
        if not self.targets:
            self.get_logger().warn(
                f"no targets loaded from {self.get_parameter('scenario_file').value}. "
                f"Scoring will report every detection as a false positive.")
        else:
            names = ", ".join(t["name"] for t in self.targets)
            self.get_logger().info(f"{len(self.targets)} ground truth targets: {names}")

        self.marker_pub = self.create_publisher(MarkerArray, "/ground_truth/markers", LATCHED)
        self.truth_pub = self.create_publisher(Detection3DArray, "/ground_truth/truth_3d", LATCHED)

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish)

    # ------------------------------------------------------------------ input
    def _load(self) -> list[dict]:
        resolved = Path(self.get_parameter("resolved_file").value)
        if resolved.is_file():
            targets = self._load_file(resolved)
            if targets:
                self.get_logger().info(
                    f"using the poses read back from Gazebo, {resolved}")
                return targets
            self.get_logger().warn(
                f"{resolved} has no usable entities, falling back to the scenario")
        else:
            self.get_logger().warn(
                f"no {resolved}; using the scenario file, which records where "
                f"entities were asked to go rather than where they are. Re-run "
                f"the scenario to produce it: px4sim scenario")
        return self._load_file(Path(self.get_parameter("scenario_file").value))

    def _load_file(self, path: Path) -> list[dict]:
        if not path.is_file():
            self.get_logger().error(f"no scenario file at {path}")
            return []
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except yaml.YAMLError as exc:
            self.get_logger().error(f"cannot parse {path}: {exc}")
            return []

        out = []
        for entity in data.get("entities", []):
            name = str(entity.get("name", ""))
            if not any(f in name.lower() for f in TARGET_NAME_FILTERS):
                continue
            pose = list(entity.get("pose", []))
            if len(pose) < 3:
                continue
            out.append({
                "name": name,
                "x": float(pose[0]) + self.offset[0],
                "y": float(pose[1]) + self.offset[1],
                "z": float(pose[2]) + self.offset[2],
            })
        return out

    # ---------------------------------------------------------------- output
    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        truth = Detection3DArray()
        truth.header.stamp = now
        truth.header.frame_id = self.reference

        for i, target in enumerate(self.targets):
            sphere = Marker()
            sphere.header.stamp = now
            sphere.header.frame_id = self.reference
            sphere.ns = "ground_truth"
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = target["x"]
            sphere.pose.position.y = target["y"]
            sphere.pose.position.z = target["z"] + PERSON_SPHERE_DIAMETER / 2.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = PERSON_SPHERE_DIAMETER
            r, g, b = GROUND_TRUTH_COLOR
            sphere.color = ColorRGBA(r=r, g=g, b=b, a=MARKER_ALPHA)
            markers.markers.append(sphere)

            d = Detection3D()
            d.header = truth.header
            d.id = target["name"]
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "person"
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose.position.x = target["x"]
            hypothesis.pose.pose.position.y = target["y"]
            hypothesis.pose.pose.position.z = target["z"]
            hypothesis.pose.pose.orientation.w = 1.0
            d.results.append(hypothesis)
            d.bbox = BoundingBox3D()
            d.bbox.center = hypothesis.pose.pose
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = PERSON_SPHERE_DIAMETER
            truth.detections.append(d)

        self.marker_pub.publish(markers)
        self.truth_pub.publish(truth)


def main() -> None:
    rclpy.init()
    node = GroundTruth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
