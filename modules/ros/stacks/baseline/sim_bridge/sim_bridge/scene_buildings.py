#!/usr/bin/env python3
"""Publish the scene's buildings for the Foxglove 3D panel.

scenegen build writes worlds/<scene>_buildings.json next to the world: every
building as triangles, roofs with a satellite color per vertex, walls in one
flat gray. Buildings only, on purpose: vehicle props would clutter the panel
without telling the operator anything.

A timer watches the file's mtime, so a rebuilt scene shows up within a few
seconds and no restart. A missing file means an empty scene until it appears,
which is normal for a hand-written world. Every publish leads with a
DELETEALL, so buildings that left the scene also leave the display.

Publishes
    /scene/buildings    visualization_msgs/MarkerArray, latched
"""

from __future__ import annotations

import json
import os
from pathlib import Path

from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from sim_bridge.runtime import LATCHED, spin

# ------------------------------------------------------------------- tunables
BUILDINGS_FORMAT = "scenegen-buildings/1"
# How often to look for a changed file on disk. One stat call costs
# nothing, and a rebuilt scene shows up in the panel within this long.
RELOAD_CHECK_S = 2.0


class SceneBuildings(Node):
    def __init__(self) -> None:
        super().__init__("scene_buildings")
        self.declare_parameter("buildings_file", os.environ.get(
            "SCENE_BUILDINGS_FILE", ""))
        self.declare_parameter("reference_frame", "map")
        self.reference = self.get_parameter("reference_frame").value
        configured = str(self.get_parameter("buildings_file").value)
        self.path = Path(configured) if configured else None
        self.publisher = self.create_publisher(MarkerArray, "/scene/buildings",
                                               LATCHED)
        self.published_stamp: int | None = 0
        if self.path is None:
            self.get_logger().warn(
                "no buildings_file configured (SCENE is empty); the 3D "
                "panel shows no buildings")
            self.publisher.publish(self._markers())
            return
        self._maybe_publish()
        self.create_timer(RELOAD_CHECK_S, self._maybe_publish)

    def _maybe_publish(self) -> None:
        try:
            stamp = self.path.stat().st_mtime_ns
        except OSError:
            stamp = None
        if stamp == self.published_stamp:
            return
        self.published_stamp = stamp
        if stamp is None:
            self.get_logger().info(
                f"no buildings file at {self.path}; the 3D panel shows "
                f"none until scenegen build writes it next to the world")
        self.publisher.publish(self._markers())

    def _wipe(self) -> Marker:
        wipe = Marker()
        wipe.header.frame_id = self.reference
        wipe.action = Marker.DELETEALL
        return wipe

    def _markers(self) -> MarkerArray:
        """The buildings on disk, led by the wipe. With no usable file the
        wipe goes out alone, which clears the panel."""
        markers = MarkerArray()
        markers.markers.append(self._wipe())
        if self.path is None or not self.path.is_file():
            return markers
        try:
            data = json.loads(self.path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            self.get_logger().error(f"cannot read {self.path}: {error}")
            return markers
        if data.get("format") != BUILDINGS_FORMAT:
            self.get_logger().error(
                f"{self.path} carries format {data.get('format')!r}, this "
                f"node reads {BUILDINGS_FORMAT!r}. Rebuild the scene.")
            return markers
        stamp = self.get_clock().now().to_msg()
        for index, building in enumerate(data.get("buildings", [])):
            markers.markers.append(self._building_marker(index, building, stamp))
        self.get_logger().info(
            f"{len(markers.markers) - 1} buildings from {self.path}")
        return markers

    def _building_marker(self, index: int, building: dict, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.reference
        marker.header.stamp = stamp
        marker.ns = "buildings"
        marker.id = index
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        roof = building["roof"]
        wall = building["walls"]
        wall_color = ColorRGBA(r=float(wall["color"][0]),
                               g=float(wall["color"][1]),
                               b=float(wall["color"][2]), a=1.0)
        marker.points = [Point(x=float(x), y=float(y), z=float(z))
                         for x, y, z in roof["points"] + wall["points"]]
        marker.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)
                         for r, g, b in roof["colors"]]
        marker.colors += [wall_color] * len(wall["points"])
        return marker


def main() -> None:
    spin(SceneBuildings)


if __name__ == "__main__":
    main()
