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
import yaml

import geo

SCENE_FORMAT = "scenegen-scene/1"
# Keep buildings whose footprint touches the square grown by this margin,
# so a wall on the boundary still stands.
BUILDING_KEEP_MARGIN_M = 5.0
# Below this fitted length or width, an outline is a line or a point, and
# the build falls back to the rectangle fields.
MIN_FOOTPRINT_EXTENT_M = 0.5
FIDUCIAL_DIAMETER_M = 0.5
# A target without a designated model draws from this pool at build time,
# by a stable hash of its name, so a rebuild never reshuffles the draw.
# Every OpenRobotics person model on Fuel that stands free of furniture,
# verified against the catalog on 2026-08-16: standing, walking, seated
# and lying poses. Yaw is also drawn at build when the target has none,
# so a default target gets a random pose and a random heading.
#
# Two kinds of person model stay out on purpose. "Male visitor" and
# "MaleVisitorPhone" are Fuel actors, not models: an actor animates in
# place and never reports into /world/<world>/pose/info, so the spawner
# cannot read its pose back and the target gets no ground truth.
# "Survivor Male" and "Survivor Female" read as manikins on camera, not
# as people.
_FUEL_OPENROBOTICS = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
CASUALTY_MODEL_POOL = [_FUEL_OPENROBOTICS + name for name in (
    "Standing person", "Walking person", "Casual female",
    "FemaleVisitor", "MaleVisitorOnPhone", "Nurse",
    "Scrubs", "OpScrubs", "MaleVisitorSit", "FemaleVisitorSit",
    "VisitorKidSit", "PatientFSit", "PatientWheelChair", "Rescue Randy",
    "Rescue Randy Sitting")]
# Every working tree model on Fuel, with its natural height and canopy
# diameter measured in-sim against reference poles (2026-08-16). Nothing
# rescales a tree: a height range selects among these as they are. The
# editor shows the same table through /tree_models.json.
TREE_MODEL_POOL = [
    {"name": "Juniper Tree", "height_m": 1.8, "canopy_m": 0.8,
     "uri": "https://fuel.gazebosim.org/1.0/shrijitsingh99/models/Juniper Tree"},
    {"name": "Pine Tree", "height_m": 5.0, "canopy_m": 2.2,
     "uri": _FUEL_OPENROBOTICS + "Pine Tree"},
    {"name": "Oak tree", "height_m": 6.3, "canopy_m": 10.4,
     "uri": _FUEL_OPENROBOTICS + "Oak tree"},
]


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
    agl_m: float | None = None    # meters above the floor; None sits on it
    on_building: bool = True      # floor = the building top under it, else terrain
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
    # The height the map gave, frozen at create, so an edited height can
    # go back to it. None on a hand-placed building.
    map_height_m: float | None = None
    map_height_source: str = ""
    name: str = ""
    outline_m: list = field(default_factory=list)   # [[east, north], ...] closed
    holes_m: list = field(default_factory=list)     # inner rings, same shape
    source: str = "osm"
    model_uri: str | None = None  # a higher-detail stand-in replaces the box
    enabled: bool = True


@dataclass
class Tree:
    """One tree, placed exactly. model_uri None draws from the pool by a
    stable hash of the id, so a rebuild keeps the species."""
    id: str
    east_m: float
    north_m: float
    model_uri: str | None = None
    source: str = "manual"        # "osm" from a natural=tree node
    enabled: bool = True


@dataclass
class TreeArea:
    """A polygon the build fills with trees: a jittered grid at the given
    density, each position drawn from the pool inside the height range.
    Everything derives from stable hashes of the id and the grid cell, so
    the spacing looks random and a rebuild is byte-identical."""
    id: str
    polygon_m: list               # [[east, north], ...]
    density_per_ha: float = 150.0
    min_height_m: float = 1.0
    max_height_m: float = 10.0
    source: str = "manual"        # "osm" from a wood or forest polygon
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
class Target:
    """A ground-truth casualty. The scene is the source of truth for these:
    a lat/lon file only imports into this list, the editor places and moves
    them, and the build projects them into the scenario that
    spawn_scenario.py places and the ROS scorer reads."""
    id: str
    name: str                     # the world entity name the scorer sees
    east_m: float
    north_m: float
    agl_m: float | None = None    # meters above the floor; None sits on it
    on_building: bool = True      # floor = the building top under it, else terrain
    yaw_deg: float | None = None  # None draws a stable pseudo-random yaw at build
    model_uri: str | None = None  # None draws from the pool at build, stably
    enabled: bool = True
    source: str = "manual"        # "import" from a file, "manual" from the editor


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
    targets: list[Target] = field(default_factory=list)
    trees: list[Tree] = field(default_factory=list)
    tree_areas: list[TreeArea] = field(default_factory=list)
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
        targets = []
        for entry in data.get("targets", []):
            # Scenes from before terrain-relative heights carry alt_m. The
            # key moves over; a non-null value was AMSL and needs a look.
            if "agl_m" not in entry and "alt_m" in entry:
                entry["agl_m"] = entry.pop("alt_m")
            entry.pop("alt_m", None)
            targets.append(Target(**entry))
        data["targets"] = targets
        data["trees"] = [Tree(**t) for t in data.get("trees", [])]
        data["tree_areas"] = [TreeArea(**a) for a in data.get("tree_areas", [])]
        data["flatten_zones"] = [FlattenZone(**z) for z in data.get("flatten_zones", [])]
        return SceneSpec(**data)


def fnv1a(text: str) -> int:
    """32-bit FNV-1a. The stable randomness behind tree placement. The
    editor mirrors it in JS with Math.imul, so both sides draw the same
    positions from the same keys; change one only with the other."""
    value = 2166136261
    for byte in text.encode():
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def unit_hash(key: str) -> float:
    """A stable draw in [0, 1) from a string key."""
    return fnv1a(key) / 4294967296.0


def polygon_contains(points: list, east: float, north: float) -> bool:
    """Even-odd test, arithmetic identical to the editor's inPolygon, so
    a generated tree sits inside on both sides or on neither."""
    inside = False
    j = len(points) - 1
    for i in range(len(points)):
        xi, yi = points[i]
        xj, yj = points[j]
        if (yi > north) != (yj > north) \
                and east < (xj - xi) * (north - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def tree_models_in_range(min_height_m: float, max_height_m: float) -> list:
    """The pool entries whose natural height fits the range. An empty
    match falls back to the model nearest the range, so an area never
    silently loses all its trees to a narrow range."""
    fits = [m for m in TREE_MODEL_POOL
            if min_height_m <= m["height_m"] <= max_height_m]
    if fits:
        return fits
    middle = (min_height_m + max_height_m) / 2.0
    return [min(TREE_MODEL_POOL, key=lambda m: abs(m["height_m"] - middle))]


def tree_model_for(key: str, min_height_m: float, max_height_m: float) -> dict:
    candidates = tree_models_in_range(min_height_m, max_height_m)
    return candidates[fnv1a(key + ":model") % len(candidates)]


def area_tree_points(area: TreeArea) -> list:
    """The tree positions an area generates: a grid at the requested
    density, each point jittered inside its cell by stable hashes, kept
    when it lands inside the polygon. Somewhat random spacing, no order,
    and the same answer in the editor preview and the build."""
    if len(area.polygon_m) < 3 or area.density_per_ha <= 0:
        return []
    cell = math.sqrt(10000.0 / area.density_per_ha)
    xs = [p[0] for p in area.polygon_m]
    ys = [p[1] for p in area.polygon_m]
    columns = int((max(xs) - min(xs)) / cell) + 1
    rows = int((max(ys) - min(ys)) / cell) + 1
    # The editor carries the same bound: past it an area generates
    # nothing on both sides, instead of freezing one of them.
    if columns * rows > 262144:
        return []
    points = []
    for j in range(rows):
        for i in range(columns):
            east = min(xs) + (i + 0.15 + 0.7
                              * unit_hash(f"{area.id}:{i}:{j}:x")) * cell
            north = min(ys) + (j + 0.15 + 0.7
                               * unit_hash(f"{area.id}:{i}:{j}:y")) * cell
            if polygon_contains(area.polygon_m, east, north):
                points.append((east, north, f"{area.id}:{i}:{j}"))
    return points


def scoreable_name(name: str) -> str:
    """The ROS ground-truth scorer counts only entity names that carry
    "person" or "casualty" (sim_bridge/ground_truth.py). Fix any other."""
    return name if ("casualty" in name or "person" in name) else f"casualty_{name}"


def import_casualty_file(scene: SceneSpec, path: Path) -> tuple[int, int]:
    """Load a lat/lon casualty list into the scene's targets.

    The file is an on-ramp, not the source of truth. After the import the
    targets live in scene.json, the editor moves them, and the build reads
    only the scene. A re-import replaces earlier imported targets and
    keeps hand-placed ones. Returns (imported, hand-placed kept).

    Each entry: lat and lon (required, WGS84 degrees), agl (optional
    meters above the terrain, absent sits on it), model (optional URI,
    absent draws from the pool at build), name (optional).
    """
    data = yaml.safe_load(path.read_text())
    entries = data.get("casualties", data) if isinstance(data, dict) else data
    if not isinstance(entries, list):
        raise ValueError(f"{path} holds no casualty list")
    frame = geo.GeoFrame(scene.center_lat, scene.center_lon, scene.origin_alt_m)
    imported: list[Target] = []
    for index, entry in enumerate(entries, start=1):
        if "lat" not in entry or "lon" not in entry:
            raise ValueError(f"casualty {index} in {path} has no lat/lon")
        east, north, _ = frame.latlon_to_enu(entry["lat"], entry["lon"])
        imported.append(Target(
            id=f"t_import_{index}",
            name=scoreable_name(entry.get("name") or f"casualty_{index:02d}"),
            east_m=round(east, 2), north_m=round(north, 2),
            agl_m=entry.get("agl", entry.get("alt")),
            model_uri=entry.get("model"), source="import"))
    kept = [t for t in scene.targets if t.source != "import"]
    scene.targets = kept + imported
    return len(imported), len(kept)


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


def _rectangle_ring(east: float, north: float, length: float, width: float,
                    yaw_deg: float) -> list[list[float]]:
    yaw = math.radians(yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_l, half_w = length / 2.0, width / 2.0
    return [[east + dx * cos_yaw - dy * sin_yaw, north + dx * sin_yaw + dy * cos_yaw]
            for dx, dy in [(-half_l, -half_w), (half_l, -half_w),
                           (half_l, half_w), (-half_l, half_w)]]


def placed_footprint(building: Building) -> tuple[list, list]:
    """The footprint the build extrudes, in scene ENU: (outer ring, holes),
    rings open (no repeated last point), outer counterclockwise and holes
    clockwise, the winding building_mesh.extrude expects.

    The editor edits only the rectangle fields (center, yaw, length,
    width) and never touches the outline. So the delta between the
    outline's own fitted rectangle and the stored one is exactly the
    user's edit, and this maps the outline and its holes through it. An
    unedited building maps through the identity. A building without a
    usable outline gets the rectangle itself.
    """
    ring = [p for p in building.outline_m]
    if len(ring) >= 2 and ring[0] == ring[-1]:
        ring = ring[:-1]
    if len(ring) < 3:
        return _rectangle_ring(building.east_m, building.north_m,
                               building.length_m, building.width_m,
                               building.yaw_deg), []
    fit_east, fit_north, fit_length, fit_width, fit_yaw = oriented_rectangle(ring)
    if fit_length < MIN_FOOTPRINT_EXTENT_M or fit_width < MIN_FOOTPRINT_EXTENT_M:
        return _rectangle_ring(building.east_m, building.north_m,
                               building.length_m, building.width_m,
                               building.yaw_deg), []
    scale_length = building.length_m / fit_length
    scale_width = building.width_m / fit_width
    fit = math.radians(fit_yaw)
    placed = math.radians(building.yaw_deg)
    cos_fit, sin_fit = math.cos(fit), math.sin(fit)
    cos_placed, sin_placed = math.cos(placed), math.sin(placed)

    def transform(east: float, north: float) -> list[float]:
        de, dn = east - fit_east, north - fit_north
        local_x = (de * cos_fit + dn * sin_fit) * scale_length
        local_y = (-de * sin_fit + dn * cos_fit) * scale_width
        return [building.east_m + local_x * cos_placed - local_y * sin_placed,
                building.north_m + local_x * sin_placed + local_y * cos_placed]

    outer = [transform(e, n) for e, n in ring]
    if not shapely.LinearRing(outer).is_ccw:
        outer.reverse()
    holes = []
    for hole in building.holes_m:
        hole_ring = [p for p in hole]
        if len(hole_ring) >= 2 and hole_ring[0] == hole_ring[-1]:
            hole_ring = hole_ring[:-1]
        if len(hole_ring) < 3:
            continue
        placed_hole = [transform(e, n) for e, n in hole_ring]
        if shapely.LinearRing(placed_hole).is_ccw:
            placed_hole.reverse()
        holes.append(placed_hole)
    return outer, holes


def vegetation_from_osm(raw_trees: list, raw_areas: list, frame,
                        side_m: float) -> tuple[list[Tree], list[TreeArea]]:
    """Fetched vegetation to scene entries in ENU. Individual trees
    outside the square drop; an area stays when it reaches into the
    square, and the build clips its fill there anyway."""
    half = side_m / 2.0
    trees = []
    for index, (lat, lon) in enumerate(raw_trees, start=1):
        east, north, _ = frame.latlon_to_enu(lat, lon)
        if abs(east) > half or abs(north) > half:
            continue
        trees.append(Tree(id=f"tr_osm_{index}", east_m=round(east, 2),
                          north_m=round(north, 2), source="osm"))
    square = shapely.box(-half, -half, half, half)
    areas = []
    for index, ring in enumerate(raw_areas, start=1):
        points = [tuple(frame.latlon_to_enu(lat, lon)[:2])
                  for lat, lon in ring[:-1]]
        if len(points) < 3:
            continue
        polygon = shapely.Polygon(points)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if polygon.is_empty or not polygon.intersects(square):
            continue
        areas.append(TreeArea(
            id=f"ta_osm_{index}",
            polygon_m=[[round(e, 2), round(n, 2)] for e, n in points],
            source="osm"))
    return trees, areas


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
        holes_en = [[tuple(frame.latlon_to_enu(lat, lon)[:2]) for lat, lon in ring]
                    for ring in raw.get("holes", [])]
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
            height_source=raw["height_source"],
            map_height_m=raw["height_m"], map_height_source=raw["height_source"],
            name=raw["name"],
            outline_m=[[round(e, 2), round(n, 2)] for e, n in outline_en],
            holes_m=[[[round(e, 2), round(n, 2)] for e, n in ring]
                     for ring in holes_en]))
    return kept, dropped
