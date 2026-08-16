#!/usr/bin/env python3
"""scene.json to Gazebo: the world, the terrain and building models, the
Foxglove building payload and the scenario.

Outputs, all into modules/sim/scenes/ (SCENES_DIR):

  worlds/<name>.sdf                     the world. <world name> equals the
                                        file name; the sim entrypoint
                                        refuses a mismatch.
  worlds/<name>_surface.json            terrain heights and roof polygons,
                                        for scene-based localization
                                        (sim_bridge/scene_surface.py)
  worlds/<name>_buildings.json          building triangles with satellite
                                        colors, for the Foxglove 3D panel
                                        (sim_bridge/scene_buildings.py)
  models/<name>_terrain/                textured terrain mesh model
  models/<name>_buildings/              extruded buildings, roofs textured
                                        with the same satellite image
  scenarios/<name>_casualties.yaml      always; spawn_scenario.py places
                                        the targets at run time, so they
                                        can change with no sim restart

Ground truth flows one way: a casualty file imports into scene.json, the
editor adjusts targets there, and this build projects them into the
scenario. The build draws nothing at random, so a rebuild of an unchanged
scene is byte-identical.

The scenario also carries home_* and fiducial_* lines. The front doors
read them (scripts/scenario-env.sh), so SCENE and SCENARIO in .env select
everything; nothing else is copied by hand.

The world's <spherical_coordinates> ties ENU (0,0,0) to the scene center
at the terrain's own altitude. The printed .env lines carry the same
numbers to PX4 and QGC; apply them, or the map and the world disagree.
"""

from __future__ import annotations

import json
import math
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import shapely
import yaml
from PIL import Image
from shapely.geometry.polygon import orient

import building_mesh
import collada
import geo
import scene_model
import terrain_mesh

# ------------------------------------------------------------------- tunables
# Where a detected vehicle's model comes from when the scene does not name
# one. Chosen per vehicle by a stable hash, so a rebuild keeps its choices.
# Every real vehicle model OpenRobotics has on Fuel (catalog checked
# 2026-08-16): cars, vans and light trucks in the car pool, the heavies in
# the bus pool. Fuel has no school bus, semi or tanker from a credible
# owner, so those stay absent rather than stand in as toys.
_FUEL_VEHICLES = "https://fuel.gazebosim.org/1.0/OpenRobotics/models/"
VEHICLE_MODEL_POOLS = {
    "car": [_FUEL_VEHICLES + name for name in (
        "Hatchback", "Hatchback red", "Hatchback blue", "hatchback_2",
        "SUV", "Pickup", "TruckDelivery", "Ambulance2")],
    "bus": [_FUEL_VEHICLES + name for name in (
        "Bus", "TruckBox", "Fire truck", "Ambulance")],
}
# Each Fuel model has its own forward axis, so a model can spawn sideways
# in a heading that is correct for its bounding box. This offset turns the
# model, not the box: scene.json and the editor keep the true box heading.
# Measured in a nadir render of every pool model at yaw 0 (2026-08-16):
# only the Bus and the Fire truck lie 90 degrees off their boxes.
VEHICLE_MODEL_YAW_OFFSET_DEG = {
    _FUEL_VEHICLES + "Bus": 90.0,
    _FUEL_VEHICLES + "Fire truck": 90.0,
}
# The pool a target without a model draws from lives in scene_model
# (CASUALTY_MODEL_POOL), next to the Target type and the import.
FIDUCIAL_THICKNESS_M = 0.02
FIDUCIAL_COLOR = "1 0.45 0.05 1"
# Walls get a slight per-building gray so adjacent buildings read as
# separate ones from the air. Roofs carry the satellite texture instead.
BUILDING_GRAY_RANGE = (0.55, 0.72)
# Roof triangles split down to this edge length before the Foxglove
# payload samples a satellite color at each vertex. The world mesh stays
# coarse; only the sampling density rides on this.
ROOF_SAMPLE_EDGE_M = 8.0
# A building keeps only what stands inside the scene square: the terrain
# and the imagery end there, and the cut part would float in the void
# with no pixels to wear. A clipped remnant under this area drops whole.
MIN_CLIPPED_AREA_M2 = 1.0
# Footprints that overlap by at least this area merge into one building:
# one wall gray, and the roofs cut to the upper envelope so every point
# has one roof. Edge-to-edge neighbors stay separate buildings.
OVERLAP_MIN_AREA_M2 = 1.0
# Envelope remnants under this area are slivers of numeric noise, not
# roofs.
MIN_ROOF_PIECE_M2 = 0.05


def _pose(x: float, y: float, z: float, yaw_rad: float = 0.0) -> str:
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw_rad:.4f}"


def _stable_choice(options: list[str], key: str) -> str:
    return options[sum(key.encode()) % len(options)]


def _wall_rgba(building_id: str) -> tuple[float, float, float, float]:
    gray_low, gray_high = BUILDING_GRAY_RANGE
    gray = gray_low + (sum(building_id.encode()) % 100) / 100.0 * (gray_high - gray_low)
    return (gray, gray, gray * 0.97, 1.0)


@dataclass
class PlacedBuilding:
    """One enabled building grounded in the scene: the placed footprint
    clipped to the scene square, and the base at the lowest ground under
    it so no wall floats on a slope. A footprint the square cuts in two
    carries one ring pair per piece. Meshes live in BuildingCluster."""
    building: scene_model.Building
    polygon: shapely.Geometry
    pieces: list                      # [(outer ring, holes), ...], rings open
    base_z: float
    roof_z: float


@dataclass
class BuildingCluster:
    """Overlapping extruded buildings merged into one visual building:
    one wall gray, and the roofs cut to the upper envelope, so a typical
    building mapped as several overlapping parts shows exactly one roof
    surface over every point, at the height of its tallest part there."""
    id: str
    name: str
    wall_rgba: tuple
    roof: np.ndarray                  # (n, 3, 3) triangles, per-part heights
    walls: np.ndarray
    holes: int


def _polygon_parts(geometry) -> list:
    if geometry.geom_type == "Polygon":
        return [geometry]
    return [part for part in getattr(geometry, "geoms", [])
            if part.geom_type == "Polygon"]


def _oriented_rings(polygon) -> tuple[list, list]:
    """(outer, holes) of one polygon, rings open, outer counterclockwise
    and holes clockwise, the winding building_mesh.extrude expects."""
    oriented = orient(polygon)
    outer = [list(point) for point in oriented.exterior.coords[:-1]]
    holes = [[list(point) for point in ring.coords[:-1]]
             for ring in oriented.interiors]
    return outer, holes


def place_buildings(scene: scene_model.SceneSpec, grid,
                    meta) -> tuple[list[PlacedBuilding], int]:
    """Returns (placed buildings, buildings dropped as outside the
    square). The scene square clips every footprint; a building whose
    remnant is under MIN_CLIPPED_AREA_M2 drops whole."""
    origin_alt = scene.origin_alt_m
    half = scene.side_m / 2.0
    square = shapely.box(-half, -half, half, half)
    placed = []
    dropped = 0
    for building in scene.buildings:
        if not building.enabled:
            continue
        outer, holes = scene_model.placed_footprint(building)
        polygon = shapely.Polygon(outer, holes)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if not square.covers(polygon):
            polygon = polygon.intersection(square)
        if polygon.is_empty or polygon.area < MIN_CLIPPED_AREA_M2:
            dropped += 1
            continue
        parts = _polygon_parts(polygon)
        pieces = [_oriented_rings(part) for part in parts]
        samples = [point for piece_outer, piece_holes in pieces
                   for ring in [piece_outer, *piece_holes] for point in ring]
        samples += [[part.centroid.x, part.centroid.y] for part in parts]
        base = min(_ground_at(grid, meta, east, north, origin_alt)
                   for east, north in samples)
        placed.append(PlacedBuilding(building, polygon, pieces, base,
                                     base + building.height_m))
    return placed, dropped


def cluster_buildings(placed: list[PlacedBuilding]) -> list[BuildingCluster]:
    """Group the extruded buildings into overlap clusters and mesh each.

    Members sort by (roof height, id), and each roof keeps only the part
    no later member's footprint covers: the upper envelope, with equal
    heights broken deterministically by id. Walls stay full height; a
    wall segment inside a taller neighbor is enclosed by it and never
    seen. Model-override buildings stand outside the clustering."""
    meshed = [entry for entry in placed if not entry.building.model_uri]
    parent = list(range(len(meshed)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(len(meshed)):
        for j in range(i + 1, len(meshed)):
            if meshed[i].polygon.intersection(meshed[j].polygon).area \
                    > OVERLAP_MIN_AREA_M2:
                parent[find(i)] = find(j)

    groups: dict[int, list[PlacedBuilding]] = {}
    for index, entry in enumerate(meshed):
        groups.setdefault(find(index), []).append(entry)

    clusters = []
    for members in groups.values():
        members.sort(key=lambda entry: (entry.roof_z, entry.building.id))
        cluster_id = min(entry.building.id for entry in members)
        name = next((entry.building.name for entry in members
                     if entry.building.name), "")
        roofs, walls = [], []
        holes = 0
        for index, entry in enumerate(members):
            covering = [m.polygon for m in members[index + 1:]
                        if m.polygon.intersects(entry.polygon)]
            exposed = entry.polygon.difference(shapely.union_all(covering)) \
                if covering else entry.polygon
            for part in _polygon_parts(exposed):
                if part.area < MIN_ROOF_PIECE_M2:
                    continue
                outer, hole_rings = _oriented_rings(part)
                roofs.append(building_mesh.roof_at(outer, hole_rings,
                                                   entry.roof_z))
            for outer, hole_rings in entry.pieces:
                walls.append(building_mesh.extrude_walls(
                    outer, hole_rings, entry.base_z, entry.roof_z))
                holes += len(hole_rings)
        empty = np.zeros((0, 3, 3))
        clusters.append(BuildingCluster(
            cluster_id, name, _wall_rgba(cluster_id),
            np.concatenate(roofs) if roofs else empty,
            np.concatenate(walls) if walls else empty, holes))
    return sorted(clusters, key=lambda cluster: cluster.id)


def _include(name: str, uri: str, pose: str) -> str:
    return f"""    <include>
      <uri>{uri}</uri>
      <name>{name}</name>
      <pose>{pose}</pose>
      <static>true</static>
    </include>
"""


def _ground_at(grid, meta, east: float, north: float, origin_alt: float) -> float:
    return terrain_mesh.sample_height(grid, meta, east, north) - origin_alt


def _roof_above(placed: list[PlacedBuilding], east: float,
                north: float) -> float | None:
    """The top of the highest building whose footprint covers the point,
    in scene z. None when no building stands there."""
    point = shapely.Point(east, north)
    tops = [entry.roof_z for entry in placed if entry.polygon.covers(point)]
    return max(tops) if tops else None


def _floor_z(placed: list[PlacedBuilding], grid, meta, origin_alt: float,
             east: float, north: float, on_building: bool) -> float:
    """The surface something stands on, scene z: the terrain, or the top
    of the building under the point when on_building asks for it. With no
    building there it falls back to the terrain."""
    floor = _ground_at(grid, meta, east, north, origin_alt)
    if on_building:
        roof = _roof_above(placed, east, north)
        if roof is not None:
            floor = roof
    return floor


def _rounded_ring(ring: list) -> list:
    return [[round(east, 2), round(north, 2)] for east, north in ring]


def surface_json(scene: scene_model.SceneSpec, placed: list[PlacedBuilding],
                 grid, meta) -> str:
    """The localization surface: terrain heights and roof polygons, scene z.

    sim_bridge/scene_surface.py reads this to intersect detection rays with
    the scene instead of a flat plane. Walls are absent on purpose: the
    surface holds only what a camera above looks down onto. A building with
    a model override keeps its footprint roof, the best height on record.
    """
    origin_alt = scene.origin_alt_m
    n = meta["grid_n"]
    terrain = [[round(float(grid[j, i]) - origin_alt, 2) for i in range(n + 1)]
               for j in range(n + 1)]
    buildings = [{
        "footprint": _rounded_ring(outer),
        "holes": [_rounded_ring(ring) for ring in holes],
        "roof_z": round(entry.roof_z, 2),
    } for entry in placed for outer, holes in entry.pieces]
    return json.dumps({
        "format": "scenegen-surface/2",
        "side_m": meta["side_m"],
        "grid_n": n,
        "row0": meta.get("row0", "south"),
        "terrain_z": terrain,
        "buildings": buildings,
    })


def build_target_scenario(scene: scene_model.SceneSpec,
                          placed: list[PlacedBuilding], grid, meta,
                          geo_meta: dict, out_path: Path) -> list[str]:
    """The scene's ground-truth targets, projected into the scenario that
    spawn_scenario.py places, plus the home_* and fiducial_* lines the
    front doors read. Pure: every value comes from the scene, and the
    fallbacks (pool model, yaw) are stable hashes, so a rebuild of an
    unchanged scene writes the same bytes."""
    lines = []
    entities = []
    used_names: set[str] = set()
    for target in scene.targets:
        if not target.enabled:
            continue
        name = scene_model.scoreable_name(target.name)
        base, counter = name, 2
        while name in used_names:
            name = f"{base}_{counter}"
            counter += 1
        used_names.add(name)
        floor = _floor_z(placed, grid, meta, scene.origin_alt_m,
                         target.east_m, target.north_m, target.on_building)
        z = floor + (target.agl_m or 0.0)
        uri = target.model_uri or _stable_choice(scene_model.CASUALTY_MODEL_POOL,
                                                 scene.name + name)
        yaw_deg = target.yaw_deg if target.yaw_deg is not None \
            else float(sum((scene.name + name).encode()) % 360)
        entities.append({"name": name, "uri": uri,
                         "pose": [round(target.east_m, 3), round(target.north_m, 3),
                                  round(z, 3), 0, 0,
                                  round(math.radians(yaw_deg), 3)],
                         "static": True})
        lines.append(f"{name:24s} ({target.east_m:8.1f}, {target.north_m:8.1f}, "
                     f"{z:6.2f})  {uri}")
    scenario = {"name": out_path.stem,
                "description": f"{len(entities)} ground-truth targets from "
                               f"scene {scene.name}.",
                **geo_meta,
                "entities": entities}
    out_path.write_text(yaml.safe_dump(scenario, sort_keys=False, allow_unicode=True))
    return lines


def tree_instances(scene: scene_model.SceneSpec) -> list[dict]:
    """Every tree the scene asks for: the individual ones, then each
    area's fill. Species draw from the pool by stable hashes of the id or
    the grid cell; an area filters the pool to its height range first,
    and nothing rescales a model."""
    instances = []
    for tree in scene.trees:
        if not tree.enabled:
            continue
        uri = tree.model_uri or scene_model.tree_model_for(
            tree.id, 0.0, 1000.0)["uri"]
        instances.append({"id": tree.id, "east": tree.east_m,
                          "north": tree.north_m, "uri": uri, "key": tree.id})
    for area in scene.tree_areas:
        if not area.enabled:
            continue
        for east, north, key in scene_model.area_tree_points(area):
            model = scene_model.tree_model_for(key, area.min_height_m,
                                               area.max_height_m)
            instances.append({"id": "tr_" + key.replace(":", "_"),
                              "east": east, "north": north,
                              "uri": model["uri"], "key": key})
    return instances


def build_world_sdf(scene: scene_model.SceneSpec, frame: geo.GeoFrame,
                    placed: list[PlacedBuilding], grid, meta) -> tuple[str, dict]:
    origin_alt = scene.origin_alt_m
    counts = {"buildings": 0, "building_overrides": 0, "vehicles": 0}

    building_includes = []
    for entry in placed:
        if entry.building.model_uri:
            building_includes.append(_include(
                entry.building.id, entry.building.model_uri,
                _pose(entry.building.east_m, entry.building.north_m,
                      entry.base_z, math.radians(entry.building.yaw_deg))))
            counts["building_overrides"] += 1
        else:
            counts["buildings"] += 1

    vehicle_includes = []
    for vehicle in scene.vehicles:
        if not vehicle.enabled:
            continue
        uri = vehicle.model_uri or _stable_choice(
            VEHICLE_MODEL_POOLS.get(vehicle.cls, VEHICLE_MODEL_POOLS["car"]), vehicle.id)
        floor = _floor_z(placed, grid, meta, origin_alt,
                         vehicle.east_m, vehicle.north_m, vehicle.on_building)
        model_yaw = vehicle.heading_deg + VEHICLE_MODEL_YAW_OFFSET_DEG.get(uri, 0.0)
        vehicle_includes.append(_include(
            vehicle.id, uri,
            _pose(vehicle.east_m, vehicle.north_m,
                  floor + (vehicle.agl_m or 0.0),
                  math.radians(model_yaw))))
        counts["vehicles"] += 1

    # Trees stand on the terrain. One under a building or outside the
    # square would poke through a roof or float in the void, so it drops.
    tree_includes = []
    counts["trees"] = counts["trees_skipped"] = 0
    half = scene.side_m / 2.0
    building_union = shapely.union_all(
        [entry.polygon for entry in placed]) if placed else None
    if building_union is not None and not building_union.is_empty:
        shapely.prepare(building_union)
    for instance in tree_instances(scene):
        east, north = instance["east"], instance["north"]
        blocked = abs(east) > half or abs(north) > half or (
            building_union is not None
            and building_union.covers(shapely.Point(east, north)))
        if blocked:
            counts["trees_skipped"] += 1
            continue
        ground = _ground_at(grid, meta, east, north, origin_alt)
        yaw = scene_model.unit_hash(instance["key"] + ":yaw") * 2.0 * math.pi
        tree_includes.append(_include(instance["id"], instance["uri"],
                                      _pose(east, north, ground, yaw)))
        counts["trees"] += 1

    fiducial = scene.fiducial
    # The disk floats just over the highest surface under its rim, a roof
    # included, so a slope never buries an edge, the whole 0.5 m circle
    # stays visible from the air, and a fiducial placed on a building
    # sits on that building.
    radius = fiducial["diameter_m"] / 2.0
    rim_ground = max(
        _floor_z(placed, grid, meta, origin_alt, fiducial["east_m"] + dx,
                 fiducial["north_m"] + dy, True)
        for dx, dy in [(0, 0), (radius, 0), (-radius, 0), (0, radius), (0, -radius)])
    fiducial_pose = _pose(fiducial["east_m"], fiducial["north_m"],
                          rim_ground + FIDUCIAL_THICKNESS_M / 2.0 + 0.005)

    buildings_model = ""
    if counts["buildings"]:
        buildings_model = _include(f"{scene.name}_buildings",
                                   f"model://{scene.name}_buildings",
                                   _pose(0.0, 0.0, 0.0))

    world = f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  {scene.name} - generated by modules/scenegen from real map data.

  Center {scene.center_lat:.6f}, {scene.center_lon:.6f}, side {scene.side_m:.0f} m.
  Ground truth for the layout is data/{scene.name}/scene.json in the
  scenegen module. Edit that and rebuild; an edit here does not survive
  the next build.

  The <world name> must match the file name. The sim entrypoint checks it.
-->
<sdf version="1.9">
  <world name="{scene.name}">

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <scene>
      <grid>false</grid>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.62 0.72 0.85 1</background>
      <shadows>true</shadows>
    </scene>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <intensity>1</intensity>
      <direction>0.3 0.5 -0.81</direction>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <attenuation>
        <range>2000</range>
        <linear>0</linear>
        <constant>1</constant>
        <quadratic>0</quadratic>
      </attenuation>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>

    <include>
      <uri>model://{scene.name}_terrain</uri>
      <name>{scene.name}_terrain</name>
      <pose>0 0 0 0 0 0</pose>
    </include>

{buildings_model}{"".join(building_includes)}{"".join(vehicle_includes)}{"".join(tree_includes)}
    <!-- The survey fiducial: a flat orange 0.5 m disk. Its coordinate
         rides in the scenario as fiducial_*; the drone measures it to
         align frames. -->
    <model name="fiducial_marker">
      <static>true</static>
      <pose>{fiducial_pose}</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{fiducial["diameter_m"] / 2.0:.3f}</radius>
              <length>{FIDUCIAL_THICKNESS_M}</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>{FIDUCIAL_COLOR}</ambient>
            <diffuse>{FIDUCIAL_COLOR}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <!-- A touch of emissive keeps the marker orange in shadow. -->
            <emissive>0.35 0.14 0.02 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{scene.center_lat:.7f}</latitude_deg>
      <longitude_deg>{scene.center_lon:.7f}</longitude_deg>
      <elevation>{origin_alt:.2f}</elevation>
    </spherical_coordinates>

  </world>
</sdf>
"""
    return world, counts


def _model_config(model_name: str, description: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


def _model_sdf(model_name: str, mesh_file: str) -> str:
    mesh_uri = f"model://{model_name}/meshes/{mesh_file}"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
        <surface><friction><ode/></friction><bounce/><contact/></surface>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>{mesh_uri}</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
"""


def write_buildings_model(scene: scene_model.SceneSpec, frame: geo.GeoFrame,
                          clusters: list[BuildingCluster], scene_data_dir: Path,
                          scenes_dir: Path) -> dict:
    """The merged buildings as one static model: satellite-textured
    envelope roofs, one flat wall gray per cluster. Returns numbers for
    the build report."""
    model_name = f"{scene.name}_buildings"
    model_dir = scenes_dir / "models" / model_name
    (model_dir / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    shutil.copy2(scene_data_dir / scene.imagery["file"],
                 model_dir / "materials" / "textures" / "satellite.jpg")

    materials = [collada.Material(id="satellite",
                                  texture="../materials/textures/satellite.jpg")]
    geometries = []
    triangles = 0
    for cluster in clusters:
        wall_id = f"{cluster.id}-wall"
        materials.append(collada.Material(id=wall_id, rgba=cluster.wall_rgba))
        roof_points = cluster.roof.reshape(-1, 3)
        wall_points = cluster.walls.reshape(-1, 3)
        positions = np.concatenate([roof_points, wall_points])
        normals = np.concatenate([building_mesh.face_normals(cluster.roof),
                                  building_mesh.face_normals(cluster.walls)])
        # A footprint on the fetch margin can poke past the imagery crop.
        # Clamped coordinates smear the edge pixel over the overhang;
        # wrapped ones would paint the far side of the image onto the roof.
        roof_uvs = np.clip(
            terrain_mesh.imagery_uv(frame, scene.imagery, roof_points[:, :2]),
            0.0, 1.0)
        uvs = np.concatenate([roof_uvs, np.zeros((wall_points.shape[0], 2))])
        indices = np.arange(positions.shape[0]).reshape(-1, 3)
        roof_count = cluster.roof.shape[0]
        geometries.append(collada.Geometry(
            id=cluster.id, positions=positions, normals=normals, uvs=uvs,
            groups=[("satellite", indices[:roof_count]),
                    (wall_id, indices[roof_count:])]))
        triangles += int(indices.shape[0])

    collada.write_dae(model_dir / "meshes" / "buildings.dae", materials, geometries)
    (model_dir / "model.config").write_text(_model_config(
        model_name, f"Buildings of the {scene.name} scene, extruded from "
                    f"map footprints with the satellite image over the "
                    f"roofs. Generated by modules/scenegen."))
    (model_dir / "model.sdf").write_text(_model_sdf(model_name, "buildings.dae"))
    return {"holes": sum(cluster.holes for cluster in clusters),
            "triangles": triangles}


def buildings_viz_json(scene: scene_model.SceneSpec, frame: geo.GeoFrame,
                       clusters: list[BuildingCluster],
                       scene_data_dir: Path) -> str:
    """The Foxglove payload: building triangles in the map frame, roofs
    with a satellite color per vertex, walls in their flat gray.
    sim_bridge/scene_buildings.py turns this into markers; nothing else
    reads it. Buildings only, on purpose: the 3D panel shows the drone's
    surroundings, not the vehicle props."""
    image = np.asarray(
        Image.open(scene_data_dir / scene.imagery["file"]).convert("RGB"))
    entries = []
    for cluster in clusters:
        roof_points = building_mesh.subdivide(
            cluster.roof, ROOF_SAMPLE_EDGE_M).reshape(-1, 3)
        pixels = terrain_mesh.imagery_pixels(frame, scene.imagery,
                                             roof_points[:, :2])
        columns = np.clip(pixels[:, 0].astype(int), 0, image.shape[1] - 1)
        rows = np.clip(pixels[:, 1].astype(int), 0, image.shape[0] - 1)
        roof_colors = image[rows, columns] / 255.0
        wall_points = cluster.walls.reshape(-1, 3)
        entries.append({
            "id": cluster.id,
            "name": cluster.name,
            "roof": {"points": [[round(float(v), 2) for v in p]
                                for p in roof_points],
                     "colors": [[round(float(c), 3) for c in color]
                                for color in roof_colors]},
            "walls": {"points": [[round(float(v), 2) for v in p]
                                 for p in wall_points],
                      "color": [round(c, 3) for c in cluster.wall_rgba[:3]]}})
    return json.dumps({"format": "scenegen-buildings/1", "frame": "map",
                       "buildings": entries})


def run(scene_data_dir: Path, scenes_dir: Path) -> int:
    scene_path = scene_data_dir / "scene.json"
    if not scene_path.is_file():
        print(f"No scene at {scene_path}. Run create first.", file=sys.stderr)
        return 1
    scene = scene_model.load_scene(scene_path)
    if not scene.name.replace("_", "").isalnum() or not scene.name[0].isalpha():
        print(f"Scene name {scene.name!r} must be letters, digits and _ and "
              f"start with a letter: it becomes the Gazebo world name.",
              file=sys.stderr)
        return 1

    frame = geo.GeoFrame(scene.center_lat, scene.center_lon, scene.origin_alt_m)
    raw_grid, meta = terrain_mesh.load_elevation(scene_data_dir)
    grid = terrain_mesh.apply_flatten_zones(raw_grid, meta, scene.flatten_zones,
                                            scene.origin_alt_m)

    model_dir = scenes_dir / "models" / f"{scene.name}_terrain"
    (model_dir / "materials" / "textures").mkdir(parents=True, exist_ok=True)
    shutil.copy2(scene_data_dir / scene.imagery["file"],
                 model_dir / "materials" / "textures" / "satellite.jpg")
    mesh_stats = terrain_mesh.write_terrain_dae(
        frame, grid, meta, scene.imagery, scene.origin_alt_m,
        "../materials/textures/satellite.jpg", model_dir / "meshes" / "terrain.dae")
    (model_dir / "model.config").write_text(_model_config(
        f"{scene.name}_terrain",
        f"Terrain for the {scene.name} scene: elevation grid with the "
        f"satellite image draped over it. Generated by modules/scenegen."))
    (model_dir / "model.sdf").write_text(_model_sdf(f"{scene.name}_terrain",
                                                    "terrain.dae"))

    placed, dropped_outside = place_buildings(scene, grid, meta)
    clusters = cluster_buildings(placed)
    building_stats = {"holes": 0, "triangles": 0}
    if clusters:
        building_stats = write_buildings_model(scene, frame, clusters,
                                               scene_data_dir, scenes_dir)

    world, counts = build_world_sdf(scene, frame, placed, grid, meta)
    world_path = scenes_dir / "worlds" / f"{scene.name}.sdf"
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world)
    surface_path = world_path.with_name(f"{scene.name}_surface.json")
    surface_path.write_text(surface_json(scene, placed, grid, meta))
    buildings_viz_path = world_path.with_name(f"{scene.name}_buildings.json")
    buildings_viz_path.write_text(
        buildings_viz_json(scene, frame, clusters, scene_data_dir))

    fiducial_lat, fiducial_lon, fiducial_alt = frame.enu_to_latlon(
        scene.fiducial["east_m"], scene.fiducial["north_m"],
        _floor_z(placed, grid, meta, scene.origin_alt_m,
                 scene.fiducial["east_m"], scene.fiducial["north_m"], True))
    geo_meta = {"home_lat": round(scene.center_lat, 7),
                "home_lon": round(scene.center_lon, 7),
                "home_alt": round(scene.origin_alt_m, 2),
                "fiducial_lat": round(fiducial_lat, 7),
                "fiducial_lon": round(fiducial_lon, 7),
                "fiducial_alt": round(fiducial_alt, 2)}

    # The scenario always exists, even with zero targets, because it is
    # what carries home and fiducial: SCENE and SCENARIO in .env select
    # everything, and nothing else moves by hand.
    scenario_path = scenes_dir / "scenarios" / f"{scene.name}_casualties.yaml"
    scenario_path.parent.mkdir(parents=True, exist_ok=True)
    target_lines = build_target_scenario(scene, placed, grid, meta, geo_meta,
                                         scenario_path)

    env_lines = [f"SCENE={scene.name}", f"SCENARIO={scenario_path.stem}"]
    (scene_data_dir / "env.snippet").write_text(
        "\n".join(env_lines) + "\n"
        + "# Home and fiducial ride inside the scenario file; px4sim and\n"
        + "# make read them from it. For reference:\n"
        + "".join(f"# {key}={value}\n" for key, value in geo_meta.items()))

    report = [f"world      {world_path}",
              f"surface    {surface_path}",
              f"terrain    {mesh_stats['vertices']} vertices, "
              f"{mesh_stats['triangles']} triangles, "
              f"z {mesh_stats['z_min']} to {mesh_stats['z_max']} m",
              f"buildings  {counts['buildings']} extruded into "
              f"{len(clusters)} merged buildings "
              f"({building_stats['holes']} courtyard holes, "
              f"{building_stats['triangles']} triangles), "
              f"{counts['building_overrides']} model overrides, "
              f"{dropped_outside} outside the square dropped",
              f"vehicles   {counts['vehicles']}",
              f"trees      {counts['trees']} placed, {counts['trees_skipped']} "
              f"under buildings or outside the square dropped"]
    if scenario_path:
        report.append(f"scenario   {scenario_path}")
        report += ["  " + line for line in target_lines]
    report.append("")
    report.append("Put these two in .env; home and fiducial ride inside the "
                  "scenario file:")
    report += ["  " + line for line in env_lines]
    if abs(mesh_stats["z_min"]) > 5 or abs(mesh_stats["z_max"]) > 5:
        report.append("")
        report.append(f"NOTE: terrain spans {mesh_stats['z_min']} to "
                      f"{mesh_stats['z_max']} m around the origin. With "
                      "LOCALIZATION_MODE=plane the ROS localizers project "
                      "onto a flat plane at the takeoff altitude; expect "
                      "offsets over ground far above or below the takeoff "
                      "point. LOCALIZATION_MODE=scene reads the surface "
                      "file instead and follows the terrain and the roofs.")
    text = "\n".join(report)
    (scene_data_dir / "build_report.txt").write_text(text + "\n")
    print(text)
    return 0
