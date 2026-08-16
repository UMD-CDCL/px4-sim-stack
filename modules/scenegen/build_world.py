#!/usr/bin/env python3
"""scene.json to Gazebo: the world, the terrain model and the scenario.

Outputs, all into modules/sim/scenes/ (SCENES_DIR):

  worlds/<name>.sdf                     the world. <world name> equals the
                                        file name; the sim entrypoint
                                        refuses a mismatch.
  models/<name>_terrain/                textured terrain mesh model
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

import math
import shutil
import sys
from pathlib import Path

import yaml

import geo
import scene_model
import terrain_mesh

# ------------------------------------------------------------------- tunables
# Where a detected vehicle's model comes from when the scene does not name
# one. Chosen per vehicle by a stable hash, so a rebuild keeps its choices.
VEHICLE_MODEL_POOLS = {
    "car": ["https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback red",
            "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback blue",
            "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV",
            "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pickup"],
    "bus": ["https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bus"],
}
# The pool a target without a model draws from lives in scene_model
# (CASUALTY_MODEL_POOL), next to the Target type and the import.
FIDUCIAL_THICKNESS_M = 0.02
FIDUCIAL_COLOR = "1 0.45 0.05 1"
# Building faces get a slight per-building tint so adjacent boxes read as
# separate buildings from the air.
BUILDING_GRAY_RANGE = (0.55, 0.72)


def _pose(x: float, y: float, z: float, yaw_rad: float = 0.0) -> str:
    return f"{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw_rad:.4f}"


def _stable_choice(options: list[str], key: str) -> str:
    return options[sum(key.encode()) % len(options)]


def _building_link(building: scene_model.Building, base_z: float) -> str:
    gray_low, gray_high = BUILDING_GRAY_RANGE
    gray = gray_low + (sum(building.id.encode()) % 100) / 100.0 * (gray_high - gray_low)
    size = f"{building.length_m:.2f} {building.width_m:.2f} {building.height_m:.2f}"
    pose = _pose(building.east_m, building.north_m, base_z + building.height_m / 2.0,
                 math.radians(building.yaw_deg))
    label = f" ({building.name})" if building.name else ""
    return f"""    <!-- {building.id}{label}, height from {building.height_source} -->
    <link name="{building.id}">
      <pose>{pose}</pose>
      <collision name="collision">
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{gray:.2f} {gray:.2f} {gray * 0.97:.2f} 1</ambient>
          <diffuse>{gray + 0.08:.2f} {gray + 0.08:.2f} {gray * 0.97 + 0.08:.2f} 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
        </material>
      </visual>
    </link>
"""


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


def _building_base(grid, meta, building: scene_model.Building, origin_alt: float) -> float:
    """The lowest ground under the footprint's rectangle, so no corner of
    the box floats on a slope."""
    yaw = math.radians(building.yaw_deg)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_l, half_w = building.length_m / 2.0, building.width_m / 2.0
    samples = []
    for dx, dy in [(0, 0), (half_l, half_w), (half_l, -half_w),
                   (-half_l, half_w), (-half_l, -half_w)]:
        east = building.east_m + dx * cos_yaw - dy * sin_yaw
        north = building.north_m + dx * sin_yaw + dy * cos_yaw
        samples.append(_ground_at(grid, meta, east, north, origin_alt))
    return min(samples)


def _building_top_at(scene: scene_model.SceneSpec, grid, meta, east: float,
                     north: float, origin_alt: float) -> float | None:
    """The top surface of the highest enabled building whose box covers the
    point, in scene z. None when no building stands there."""
    top = None
    for building in scene.buildings:
        if not building.enabled:
            continue
        yaw = math.radians(building.yaw_deg)
        dx = east - building.east_m
        dy = north - building.north_m
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if abs(local_x) > building.length_m / 2 or abs(local_y) > building.width_m / 2:
            continue
        roof = _building_base(grid, meta, building, origin_alt) + building.height_m
        top = roof if top is None else max(top, roof)
    return top


def build_target_scenario(scene: scene_model.SceneSpec, grid, meta,
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
        floor = _ground_at(grid, meta, target.east_m, target.north_m,
                           scene.origin_alt_m)
        if target.on_building:
            roof = _building_top_at(scene, grid, meta, target.east_m,
                                    target.north_m, scene.origin_alt_m)
            if roof is not None:
                floor = roof
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


def build_world_sdf(scene: scene_model.SceneSpec, frame: geo.GeoFrame,
                    grid, meta) -> tuple[str, dict]:
    origin_alt = scene.origin_alt_m
    counts = {"buildings": 0, "building_overrides": 0, "vehicles": 0}

    building_links = []
    building_includes = []
    for building in scene.buildings:
        if not building.enabled:
            continue
        base = _building_base(grid, meta, building, origin_alt)
        if building.model_uri:
            building_includes.append(_include(
                building.id, building.model_uri,
                _pose(building.east_m, building.north_m, base,
                      math.radians(building.yaw_deg))))
            counts["building_overrides"] += 1
        else:
            building_links.append(_building_link(building, base))
            counts["buildings"] += 1

    vehicle_includes = []
    for vehicle in scene.vehicles:
        if not vehicle.enabled:
            continue
        uri = vehicle.model_uri or _stable_choice(
            VEHICLE_MODEL_POOLS.get(vehicle.cls, VEHICLE_MODEL_POOLS["car"]), vehicle.id)
        ground = _ground_at(grid, meta, vehicle.east_m, vehicle.north_m, origin_alt)
        vehicle_includes.append(_include(
            vehicle.id, uri,
            _pose(vehicle.east_m, vehicle.north_m, ground,
                  math.radians(vehicle.heading_deg))))
        counts["vehicles"] += 1

    fiducial = scene.fiducial
    # The disk floats just over the highest ground under its rim, so a
    # slope never buries an edge and the whole 0.5 m circle stays visible
    # from the air.
    radius = fiducial["diameter_m"] / 2.0
    rim_ground = max(
        _ground_at(grid, meta, fiducial["east_m"] + dx, fiducial["north_m"] + dy,
                   origin_alt)
        for dx, dy in [(0, 0), (radius, 0), (-radius, 0), (0, radius), (0, -radius)])
    fiducial_pose = _pose(fiducial["east_m"], fiducial["north_m"],
                          rim_ground + FIDUCIAL_THICKNESS_M / 2.0 + 0.005)

    buildings_model = ""
    if building_links:
        buildings_model = (f'    <model name="{scene.name}_buildings">\n'
                           "      <static>true</static>\n"
                           + "".join(building_links)
                           + "    </model>\n")

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

{buildings_model}{"".join(building_includes)}{"".join(vehicle_includes)}
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


def _terrain_model_config(name: str) -> str:
    return f"""<?xml version="1.0"?>
<model>
  <name>{name}_terrain</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Terrain for the {name} scene: elevation grid with the
  satellite image draped over it. Generated by modules/scenegen.</description>
</model>
"""


def _terrain_model_sdf(name: str) -> str:
    mesh_uri = f"model://{name}_terrain/meshes/terrain.dae"
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}_terrain">
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
    (model_dir / "model.config").write_text(_terrain_model_config(scene.name))
    (model_dir / "model.sdf").write_text(_terrain_model_sdf(scene.name))

    world, counts = build_world_sdf(scene, frame, grid, meta)
    world_path = scenes_dir / "worlds" / f"{scene.name}.sdf"
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_text(world)

    fiducial_lat, fiducial_lon, fiducial_alt = frame.enu_to_latlon(
        scene.fiducial["east_m"], scene.fiducial["north_m"],
        _ground_at(grid, meta, scene.fiducial["east_m"], scene.fiducial["north_m"],
                   scene.origin_alt_m))
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
    target_lines = build_target_scenario(scene, grid, meta, geo_meta, scenario_path)

    env_lines = [f"SCENE={scene.name}", f"SCENARIO={scenario_path.stem}"]
    (scene_data_dir / "env.snippet").write_text(
        "\n".join(env_lines) + "\n"
        + "# Home and fiducial ride inside the scenario file; px4sim and\n"
        + "# make read them from it. For reference:\n"
        + "".join(f"# {key}={value}\n" for key, value in geo_meta.items()))

    report = [f"world      {world_path}",
              f"terrain    {mesh_stats['vertices']} vertices, "
              f"{mesh_stats['triangles']} triangles, "
              f"z {mesh_stats['z_min']} to {mesh_stats['z_max']} m",
              f"buildings  {counts['buildings']} boxes, "
              f"{counts['building_overrides']} model overrides",
              f"vehicles   {counts['vehicles']}"]
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
                      f"{mesh_stats['z_max']} m around the origin. The ROS "
                      "localization nodes project onto a flat plane at the "
                      "takeoff altitude; expect offsets over ground far above "
                      "or below the takeoff point. Flatten zones can level "
                      "the areas you fly over.")
    text = "\n".join(report)
    (scene_data_dir / "build_report.txt").write_text(text + "\n")
    print(text)
    return 0
