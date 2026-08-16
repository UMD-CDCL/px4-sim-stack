#!/usr/bin/env python3
"""Publish where the targets actually are, for scoring detections against.

The simulator places targets from a scenario file, so their positions are
known exactly. This is the one node that reads a simulator file, and that is
safe: ground truth is evaluation data, nothing in the flight path reads it,
and on real hardware this node does not run.

Each target is drawn as a bubble with the scoring gate's radius, colored by
its scene status across the cameras the `cameras` parameter selects, the
gimbal alone by default: green when some camera placed an
estimate within the gate of it, yellow when some camera detected it but
every estimate failed the gate, red when some camera's view covers it but
nothing detected it, grey when no camera sees it and nothing matched it.
Green and yellow need no view: a detection overrides what the view alone
would say. The status comes from the scorers' verdicts: an FN names a
target in view but undetected, and a TP or MISLOCALIZED verdict carries
the name of the target it matched.

A billboard label names each target. The Map panel gets the same gate
circle and colors from one GeoJSON message that carries every target.
The tooltip on a target shows its name and altitude.

Markers and truth_3d go out on every timer tick. Ticks are cheap: the
messages are built once, and a tick only restamps them and recolors the
bubbles. The GeoJSON topic is latched, so it goes out only when a status
or the origin changes, and the Map panel still sees the current state.

The scenario can change while this node runs: px4sim scenario places a
new layout, and spawn_scenario.py rewrites the resolved file. A timer
watches both files and reloads on any change, so the published truth
follows the world with no restart. Every marker message leads with a
DELETEALL, so targets that left the scenario also leave the display.

Scenario poses are Gazebo world coordinates, x east and y north in meters.
PX4's local frame starts where the vehicle spawned, so the two line up.
origin_offset_xyz shifts them when they do not.

Publishes
    /ground_truth/markers      visualization_msgs/MarkerArray, the bubbles
                               and the name labels
    /ground_truth/truth_3d     vision_msgs/Detection3DArray, for the scorers
    /ground_truth/geojson      foxglove_msgs/GeoJSON, every target in one
                               message, for the Map panel. Each target is a
                               gate circle plus a pin. The tooltip shows the
                               name and the altitude. Sent only on a change.
"""

from __future__ import annotations

import json
import math
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

from sim_bridge.geo import MapOrigin
from sim_bridge.verdicts import (GROUND_TRUTH_BUBBLE_RADIUS,
                                 GROUND_TRUTH_COLOR,
                                 GROUND_TRUTH_LABEL_COLOR,
                                 GROUND_TRUTH_LABEL_HEIGHT, hex_rgb, marker)

try:
    from foxglove_msgs.msg import GeoJSON
    HAVE_FOXGLOVE = True
except ImportError:
    HAVE_FOXGLOVE = False

# ------------------------------------------------------------------- tunables
# Only entities whose name contains one of these count as findable targets.
# A scenario can also hold props that no detector should find.
TARGET_NAME_FILTERS = ["person", "casualty"]
# Verdicts older than this no longer color the bubbles. The scorers publish
# at 2 Hz, so a camera gone quiet turns its targets grey within a few ticks.
VERDICT_TIMEOUT_S = 3.0
# Vertices of the gate circle on the Map panel. GeoJSON has no circle
# primitive, so the circle is a polygon.
GATE_CIRCLE_SEGMENTS = 24
GATE_CIRCLE_ANGLES = [2.0 * math.pi * k / GATE_CIRCLE_SEGMENTS
                      for k in range(GATE_CIRCLE_SEGMENTS)]
# The fix-fallback origin re-estimates on every fix, so its last decimals
# jitter constantly. Only a move the Map panel could show counts as a
# change. One millionth of a degree is about 0.1 m.
ORIGIN_CHANGE_DEG = 1e-6
# How often to look for a changed scenario or resolved file on disk. Two
# stat calls a second cost nothing, and a re-placed scenario shows up in
# the published truth within this long.
RELOAD_CHECK_S = 1.0

LATCHED = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=1)

# One ColorRGBA per status, shared by every bubble that has that status.
BUBBLE_COLOR = {status: ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
                for status, rgba in GROUND_TRUTH_COLOR.items()}


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
        self.declare_parameter("rate_hz", 10.0)
        # Whose verdicts color the bubbles. The launch file sets it from
        # GROUND_TRUTH_CAMERAS, the gimbal alone by default.
        self.declare_parameter("cameras", ["gimbal"])

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

        # Per camera: which targets its verdicts put in each status, and
        # when. Stale entries stop counting.
        self.detected_by: dict[str, set[str]] = {}
        self.mislocalized_by: dict[str, set[str]] = {}
        self.visible_by: dict[str, set[str]] = {}
        self.verdict_stamp: dict[str, float] = {}
        for cam in [str(c) for c in self.get_parameter("cameras").value]:
            self.create_subscription(
                Detection3DArray, f"/scoring/{cam}/verdicts",
                lambda msg, c=cam: self._on_verdicts(c, msg), 10)

        self.marker_pub = self.create_publisher(MarkerArray, "/ground_truth/markers", LATCHED)
        self.truth_pub = self.create_publisher(Detection3DArray, "/ground_truth/truth_3d", LATCHED)
        self.geojson_pub = None
        if HAVE_FOXGLOVE:
            self.geojson_pub = self.create_publisher(
                GeoJSON, "/ground_truth/geojson", LATCHED)
        else:
            self.get_logger().error(
                "foxglove_msgs is missing, so the Map panel gets no ground "
                "truth. Install ros-$ROS_DISTRO-foxglove-msgs.")
        self.origin = MapOrigin(self)

        # Positions, labels and sizes never change, so the tick messages are
        # built once here. A tick only restamps and recolors them.
        self.marker_msg, self.bubbles = self._build_markers()
        self.truth_msg = self._build_truth()
        self.last_geojson_statuses: dict[str, str] | None = None
        self.last_geojson_origin: tuple[float, float] | None = None
        self.target_shapes: list[list] = []

        rate = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self._source_stamps = self._read_source_stamps()
        self.create_timer(RELOAD_CHECK_S, self._maybe_reload)

    # ------------------------------------------------------------------ input
    def _read_source_stamps(self) -> tuple:
        stamps = []
        for name in ("resolved_file", "scenario_file"):
            path = Path(self.get_parameter(name).value)
            try:
                stamps.append(path.stat().st_mtime_ns)
            except OSError:
                stamps.append(None)
        return tuple(stamps)

    def _maybe_reload(self) -> None:
        """Follow the files: a re-placed scenario rewrites the resolved
        file, and the published truth must follow it with no restart."""
        stamps = self._read_source_stamps()
        if stamps == self._source_stamps:
            return
        self._source_stamps = stamps
        self.targets = self._load()
        names = ", ".join(t["name"] for t in self.targets) or "none"
        self.get_logger().info(
            f"scenario changed on disk; {len(self.targets)} targets now: {names}")
        self.marker_msg, self.bubbles = self._build_markers()
        self.truth_msg = self._build_truth()
        # Clear the change detectors, so the next tick republishes the
        # GeoJSON and rebuilds the gate circles.
        self.last_geojson_statuses = None
        self.last_geojson_origin = None

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

    def _on_verdicts(self, cam: str, msg: Detection3DArray) -> None:
        detected, mislocalized, visible = set(), set(), set()
        for det in msg.detections:
            kind = det.results[0].hypothesis.class_id if det.results else ""
            if kind == "FN":
                visible.add(det.id)
            elif len(det.results) > 1:
                # The second result names the matched target.
                name = det.results[1].hypothesis.class_id
                if kind == "TP":
                    detected.add(name)
                elif kind == "MISLOCALIZED":
                    mislocalized.add(name)
        self.detected_by[cam] = detected
        self.mislocalized_by[cam] = mislocalized
        self.visible_by[cam] = visible
        self.verdict_stamp[cam] = self.get_clock().now().nanoseconds / 1e9

    def _statuses(self) -> dict[str, str]:
        """The scene status of every target, from one clock read. The best
        answer from any camera wins."""
        now = self.get_clock().now().nanoseconds / 1e9
        detected: set[str] = set()
        mislocalized: set[str] = set()
        visible: set[str] = set()
        for cam, stamp in self.verdict_stamp.items():
            if (now - stamp) <= VERDICT_TIMEOUT_S:
                detected |= self.detected_by[cam]
                mislocalized |= self.mislocalized_by[cam]
                visible |= self.visible_by[cam]
        out = {}
        for target in self.targets:
            name = target["name"]
            if name in detected:
                out[name] = "detected"
            elif name in mislocalized:
                out[name] = "mislocalized"
            elif name in visible:
                out[name] = "visible"
            else:
                out[name] = "out_of_view"
        return out

    # ---------------------------------------------------------------- output
    def _build_markers(self) -> tuple[MarkerArray, list]:
        """Build the marker set once. The second return pairs each bubble
        with its target name, for recoloring."""
        markers = MarkerArray()
        bubbles = []
        stamp = self.get_clock().now().to_msg()
        # A leading DELETEALL wipes the display before the adds, so a target
        # that left a reloaded scenario does not linger on screen. Within one
        # message the actions apply in order, and the wipe-then-add pair is
        # idempotent on every tick.
        wipe = Marker()
        wipe.header.frame_id = self.reference
        wipe.action = Marker.DELETEALL
        markers.markers.append(wipe)
        for i, target in enumerate(self.targets):
            # The bubble's equator sits at ground level, so a detection dot
            # inside the visible half is within the gate by construction.
            bubble = marker(
                ns="ground_truth",
                marker_id=i,
                frame_id=self.reference,
                stamp=stamp,
                position=(target["x"], target["y"], target["z"]),
                size_m=2.0 * GROUND_TRUTH_BUBBLE_RADIUS,
                rgba=GROUND_TRUTH_COLOR["out_of_view"])
            markers.markers.append(bubble)
            bubbles.append((bubble, target["name"]))
            # A marker cannot follow the camera, so the label sits on top
            # of the bubble. From the usual overhead view, that is centered
            # on the target and one gate radius toward the camera.
            markers.markers.append(marker(
                ns="ground_truth_labels",
                marker_id=i,
                frame_id=self.reference,
                stamp=stamp,
                position=(target["x"], target["y"],
                          target["z"] + GROUND_TRUTH_BUBBLE_RADIUS),
                size_m=GROUND_TRUTH_LABEL_HEIGHT,
                rgba=GROUND_TRUTH_LABEL_COLOR,
                text=target["name"]))
        return markers, bubbles

    def _build_truth(self) -> Detection3DArray:
        """Build truth_3d once. Every detection shares the array header, so
        one restamp per tick covers the whole message."""
        truth = Detection3DArray()
        truth.header.frame_id = self.reference
        for target in self.targets:
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
            d.bbox.size.x = d.bbox.size.y = d.bbox.size.z = 2.0 * GROUND_TRUTH_BUBBLE_RADIUS
            truth.detections.append(d)
        return truth

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        statuses = self._statuses()
        for m in self.marker_msg.markers:
            m.header.stamp = now
        for bubble, name in self.bubbles:
            bubble.color = BUBBLE_COLOR[statuses[name]]
        self.truth_msg.header.stamp = now
        self.marker_pub.publish(self.marker_msg)
        self.truth_pub.publish(self.truth_msg)
        self._publish_geojson(statuses)

    def _origin_moved(self, origin: tuple[float, float]) -> bool:
        last = self.last_geojson_origin
        return (last is None
                or abs(origin[0] - last[0]) > ORIGIN_CHANGE_DEG
                or abs(origin[1] - last[1]) > ORIGIN_CHANGE_DEG)

    def _target_shape(self, target: dict) -> list:
        """The gate ring of one target. It depends only on the target
        position and the origin."""
        return self.origin.geojson_ring(
            [(target["x"] + GROUND_TRUTH_BUBBLE_RADIUS * math.cos(a),
              target["y"] + GROUND_TRUTH_BUBBLE_RADIUS * math.sin(a))
             for a in GATE_CIRCLE_ANGLES])

    def _publish_geojson(self, statuses: dict[str, str]) -> None:
        """Publish every target in one GeoJSON message for the Map panel.
        Each target becomes a gate ring in its status color. The tooltip
        on the ring shows the name and the altitude.

        The topic is latched, so the message goes out only when a status or
        the origin changed since the last publish."""
        if self.geojson_pub is None or not self.origin.ready:
            return
        origin = (self.origin.lat, self.origin.lon)
        moved = self._origin_moved(origin)
        if statuses == self.last_geojson_statuses and not moved:
            return
        if moved:
            self.target_shapes = [self._target_shape(t) for t in self.targets]
        features = []
        for target, ring in zip(self.targets, self.target_shapes):
            status = statuses[target["name"]]
            rgba = GROUND_TRUTH_COLOR[status]
            # The Map panel shows `name` and `metadata` in the hover tooltip,
            # and reads Leaflet path options from `style`. A LineString, not
            # a polygon: the gate is a ring, not an area, and a line has no
            # interior to fill, to tint, or to take clicks.
            tooltip = {
                "name": target["name"],
                "metadata": {"altitude_m": round(target["z"], 1),
                             "status": status},
            }
            features.append({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": ring},
                "properties": dict(tooltip, style={
                    "color": hex_rgb(rgba), "weight": 2}),
            })
        msg = GeoJSON()
        msg.geojson = json.dumps(
            {"type": "FeatureCollection", "features": features})
        self.geojson_pub.publish(msg)
        self.last_geojson_statuses = statuses
        # Anchor the origin only when it moved, so a slow drift cannot ratchet
        # under the threshold forever.
        if moved:
            self.last_geojson_origin = origin


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
