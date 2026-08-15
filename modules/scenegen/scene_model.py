#!/usr/bin/env python3
"""The scene description: one editable scene.json per scene.

scene.json is the contract between the pipeline stages. `create` writes it
from fetched data, `detect` adds vehicles to it, the editor changes it, and
`build` turns it into a Gazebo world. Hand-editing it in a text editor is
supported; the graphical editor is a convenience over the same file.

All positions are scene ENU meters relative to the center coordinate:
east_m grows east, north_m grows north. Yaw and heading are degrees
counterclockwise from east, the Gazebo ENU convention. Heights are meters.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import shapely

SCENE_FORMAT = "scenegen-scene/1"
# Keep buildings whose footprint touches the square grown by this margin,
# so a wall on the boundary still stands.
BUILDING_KEEP_MARGIN_M = 5.0
FIDUCIAL_DIAMETER_M = 0.5


@dataclass
class Vehicle:
    id: str
    cls: str                      # "car" or "bus"
    east_m: float
    north_m: float
    length_m: float
    width_m: float
    heading_deg: float            # long axis, may be off by 180 (see DEFERRED.md)
    confidence: float = 1.0
    source: str = "manual"        # "auto" from the detector, "manual" from a person
    model_uri: str | None = None  # None picks from the class pool at build time
    enabled: bool = True


@dataclass
class Building:
    id: str
    east_m: float                 # center of the oriented bounding rectangle
    north_m: float
    length_m: float
    width_m: float
    yaw_deg: float
    height_m: float
    height_source: str = "default"
    name: str = ""
    outline_m: list = field(default_factory=list)   # [[east, north], ...] closed
    source: str = "osm"
    model_uri: str | None = None  # a higher-detail stand-in replaces the box
    enabled: bool = True


@dataclass
class FlattenZone:
    """A polygon where the terrain gets one height. This is the fix for an
    overhang or a bridge that the elevation data recorded as solid ground."""
    id: str
    polygon_m: list               # [[east, north], ...]
    mode: str = "min"             # "min" | "mean" | "manual"
    height_m: float | None = None  # used when mode is "manual", meters above origin


@dataclass
class SceneSpec:
    name: str
    center_lat: float
    center_lon: float
    side_m: float
    origin_alt_m: float           # terrain AMSL at the center; world z=0 sits here
    imagery: dict = field(default_factory=dict)
    elevation: dict = field(default_factory=dict)
    fiducial: dict = field(default_factory=lambda: {
        "east_m": 0.0, "north_m": 0.0, "diameter_m": FIDUCIAL_DIAMETER_M})
    buildings: list[Building] = field(default_factory=list)
    vehicles: list[Vehicle] = field(default_factory=list)
    flatten_zones: list[FlattenZone] = field(default_factory=list)
    format: str = SCENE_FORMAT

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=1)

    @staticmethod
    def from_json(text: str) -> "SceneSpec":
        data = json.loads(text)
        if data.get("format") != SCENE_FORMAT:
            raise ValueError(f"scene.json format is {data.get('format')!r}, "
                             f"this code reads {SCENE_FORMAT!r}")
        data["buildings"] = [Building(**b) for b in data.get("buildings", [])]
        data["vehicles"] = [Vehicle(**v) for v in data.get("vehicles", [])]
        data["flatten_zones"] = [FlattenZone(**z) for z in data.get("flatten_zones", [])]
        return SceneSpec(**data)


def load_scene(path: Path) -> SceneSpec:
    return SceneSpec.from_json(path.read_text())


def save_scene(scene: SceneSpec, path: Path) -> None:
    path.write_text(scene.to_json())


def oriented_rectangle(points_en: list[tuple[float, float]]) -> tuple[float, float, float, float, float]:
    """(center east, center north, length, width, yaw degrees) of the
    minimum-area oriented rectangle. Length is the long side; yaw points
    along it, normalized to (-90, 90]."""
    rectangle = shapely.oriented_envelope(shapely.Polygon(points_en))
    if rectangle.geom_type == "Point":
        x, y = rectangle.x, rectangle.y
        return x, y, 0.0, 0.0, 0.0
    corners = list(rectangle.exterior.coords) if rectangle.geom_type == "Polygon" \
        else list(rectangle.coords)
    if len(corners) < 3:
        (x0, y0), (x1, y1) = corners[0], corners[-1]
        return ((x0 + x1) / 2, (y0 + y1) / 2, math.dist(corners[0], corners[-1]),
                0.0, math.degrees(math.atan2(y1 - y0, x1 - x0)))

    edge_a = math.dist(corners[0], corners[1])
    edge_b = math.dist(corners[1], corners[2])
    if edge_a >= edge_b:
        length, width = edge_a, edge_b
        (x0, y0), (x1, y1) = corners[0], corners[1]
    else:
        length, width = edge_b, edge_a
        (x0, y0), (x1, y1) = corners[1], corners[2]
    yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
    if yaw <= -90.0:
        yaw += 180.0
    elif yaw > 90.0:
        yaw -= 180.0
    center = rectangle.centroid
    return center.x, center.y, length, width, yaw


def buildings_from_osm(raw_buildings: list[dict], frame, side_m: float) -> tuple[list[Building], int]:
    """Fetched footprints to Building entries in scene ENU. Returns the
    buildings and how many fell outside the square and were dropped."""
    half = side_m / 2.0 + BUILDING_KEEP_MARGIN_M
    square = shapely.box(-half, -half, half, half)
    kept: list[Building] = []
    dropped = 0
    for raw in raw_buildings:
        outline_en = [tuple(frame.latlon_to_enu(lat, lon)[:2])
                      for lat, lon in raw["outline"]]
        polygon = shapely.Polygon(outline_en)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if polygon.is_empty or not polygon.intersects(square):
            dropped += 1
            continue
        east, north, length, width, yaw = oriented_rectangle(outline_en)
        kept.append(Building(
            id="b_" + raw["id"].replace("/", "_"),
            east_m=round(east, 2), north_m=round(north, 2),
            length_m=round(length, 2), width_m=round(width, 2),
            yaw_deg=round(yaw, 2), height_m=raw["height_m"],
            height_source=raw["height_source"], name=raw["name"],
            outline_m=[[round(e, 2), round(n, 2)] for e, n in outline_en]))
    return kept, dropped
