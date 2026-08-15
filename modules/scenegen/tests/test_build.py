#!/usr/bin/env python3
"""Ground-truth tests for the build stage. No network.

Run: python3 tests/test_build.py

A synthetic scene with hand-computable geometry goes through the real
build: flat ground at 100 m AMSL with one 10 m bump, one 20x10x7 building
rotated 30 degrees, one car heading 45 degrees, one flatten zone over the
bump, and two casualties at known offsets. Every expected number below is
computed by hand from those inputs.
"""

from __future__ import annotations

import math
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import json

import numpy as np
import yaml
from PIL import Image

import build_world
import geo
import scene_model
import sources
import terrain_mesh

CENTER_LAT, CENTER_LON = 38.9869, -76.9426
SIDE_M = 200.0
GRID_N = 8
ORIGIN_ALT = 100.0
BUMP_I, BUMP_J, BUMP_M = 2, 6, 10.0   # vertex east -50, north 50

CHECKS = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, condition, detail))
    mark = "ok  " if condition else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))


def make_synthetic_scene(data_dir: Path) -> scene_model.SceneSpec:
    grid = np.full((GRID_N + 1, GRID_N + 1), ORIGIN_ALT, dtype=np.float32)
    grid[BUMP_J, BUMP_I] = ORIGIN_ALT + BUMP_M
    np.save(data_dir / "elevation.npy", grid)
    meta = {"grid_n": GRID_N, "side_m": SIDE_M, "zoom": 15, "row0": "south",
            "origin_alt_m": ORIGIN_ALT, "min_m": ORIGIN_ALT,
            "max_m": ORIGIN_ALT + BUMP_M}
    (data_dir / "elevation.json").write_text(json.dumps(meta))

    frame = geo.GeoFrame(CENTER_LAT, CENTER_LON, ORIGIN_ALT)
    zoom = 19
    min_x, min_y, max_x, max_y = sources._square_mercator_bounds(frame, SIDE_M, zoom)
    left, top = math.floor(min_x), math.floor(min_y)
    width = math.ceil(max_x) - left
    height = math.ceil(max_y) - top
    Image.new("RGB", (width, height), (90, 120, 80)).save(data_dir / "satellite.jpg")
    imagery = {"source": "synthetic", "zoom": zoom, "file": "satellite.jpg",
               "width_px": width, "height_px": height,
               "m_per_px": geo.ground_resolution_m_per_px(CENTER_LAT, zoom),
               "origin_px": float(left), "origin_py": float(top)}

    scene = scene_model.SceneSpec(
        name="synthtest", center_lat=CENTER_LAT, center_lon=CENTER_LON,
        side_m=SIDE_M, origin_alt_m=ORIGIN_ALT, imagery=imagery, elevation=meta,
        buildings=[scene_model.Building(
            id="b_test", east_m=30.0, north_m=-20.0, length_m=20.0, width_m=10.0,
            yaw_deg=30.0, height_m=7.0)],
        vehicles=[scene_model.Vehicle(
            id="v_1", cls="car", east_m=10.0, north_m=5.0, length_m=4.5,
            width_m=1.9, heading_deg=45.0, source="manual")],
        flatten_zones=[scene_model.FlattenZone(
            id="fz_1", polygon_m=[[-60, 40], [-40, 40], [-40, 60], [-60, 60]],
            mode="manual", height_m=0.0)])
    scene_model.save_scene(scene, data_dir / "scene.json")
    return scene


def make_casualty_file(path: Path) -> None:
    frame = geo.GeoFrame(CENTER_LAT, CENTER_LON, ORIGIN_ALT)
    lat_east, lon_east, _ = frame.enu_to_latlon(100.0, 0.0)
    yaml_text = yaml.safe_dump({"casualties": [
        {"lat": lat_east, "lon": lon_east, "model": "model://casualty_prone",
         "name": "casualty_alpha"},
        {"lat": CENTER_LAT, "lon": CENTER_LON, "alt": ORIGIN_ALT + 5.0},
    ]})
    path.write_text(yaml_text)


def run_build(tmp: Path) -> tuple[Path, Path]:
    data_dir = tmp / "data" / "synthtest"
    data_dir.mkdir(parents=True)
    scenes_dir = tmp / "scenes"
    make_synthetic_scene(data_dir)
    casualties = tmp / "casualties.yaml"
    make_casualty_file(casualties)
    code = build_world.run(data_dir, scenes_dir, casualties, seed=7)
    check("build exits 0", code == 0)
    return data_dir, scenes_dir


def test_world(scenes_dir: Path) -> None:
    print("world SDF")
    world_path = scenes_dir / "worlds" / "synthtest.sdf"
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    check("<world name> matches the file name", world.get("name") == "synthtest")

    spherical = world.find("spherical_coordinates")
    check("spherical coordinates carry the center and altitude",
          abs(float(spherical.find("latitude_deg").text) - CENTER_LAT) < 1e-6
          and abs(float(spherical.find("elevation").text) - ORIGIN_ALT) < 0.01)

    buildings = [m for m in world.findall("model") if m.get("name") == "synthtest_buildings"]
    check("one buildings model", len(buildings) == 1)
    link = buildings[0].find("link")
    size = link.find("visual/geometry/box/size").text.split()
    check("building box is 20 x 10 x 7",
          [float(v) for v in size] == [20.0, 10.0, 7.0], " ".join(size))
    pose = [float(v) for v in link.find("pose").text.split()]
    check("building sits on flat ground, center at half height",
          abs(pose[0] - 30) < 0.01 and abs(pose[1] + 20) < 0.01
          and abs(pose[2] - 3.5) < 0.01, str(pose[:3]))
    check("building yaw is 30 degrees in radians",
          abs(pose[5] - math.radians(30)) < 1e-3, f"{pose[5]:.4f}")

    includes = {inc.find("name").text: inc for inc in world.findall("include")}
    check("terrain include present", "synthtest_terrain" in includes)
    vehicle = includes.get("v_1")
    check("vehicle include present", vehicle is not None)
    if vehicle is not None:
        vehicle_pose = [float(v) for v in vehicle.find("pose").text.split()]
        check("vehicle pose and heading",
              abs(vehicle_pose[0] - 10) < 0.01 and abs(vehicle_pose[2]) < 0.01
              and abs(vehicle_pose[5] - math.radians(45)) < 1e-3)
        check("vehicle model drawn from the car pool",
              vehicle.find("uri").text in build_world.VEHICLE_MODEL_POOLS["car"])

    fiducial = [m for m in world.findall("model") if m.get("name") == "fiducial_marker"]
    check("fiducial marker present", len(fiducial) == 1)
    if fiducial:
        radius = float(fiducial[0].find("link/visual/geometry/cylinder/radius").text)
        check("fiducial is a 0.5 m circle", abs(radius - 0.25) < 1e-6, str(radius))


def test_terrain(scenes_dir: Path) -> None:
    print("terrain mesh")
    dae_path = scenes_dir / "models" / "synthtest_terrain" / "meshes" / "terrain.dae"
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(dae_path).getroot()
    check("mesh states Z_UP", root.find("c:asset/c:up_axis", ns).text == "Z_UP")

    arrays = {a.get("id"): a for a in root.iter("{http://www.collada.org/2005/11/COLLADASchema}float_array")}
    positions = np.fromstring(arrays["terrain-positions-array"].text, sep=" ").reshape(-1, 3)
    expected_vertices = (GRID_N + 1) ** 2
    check("vertex count matches the grid", positions.shape[0] == expected_vertices,
          str(positions.shape))

    center_index = (GRID_N // 2) * (GRID_N + 1) + GRID_N // 2
    check("origin vertex sits at z=0", abs(positions[center_index, 2]) < 1e-3,
          f"{positions[center_index]}")
    bump_index = BUMP_J * (GRID_N + 1) + BUMP_I
    check("flatten zone leveled the bump", abs(positions[bump_index, 2]) < 1e-3,
          f"z={positions[bump_index, 2]}")
    check("east axis spans the square",
          abs(positions[:, 0].min() + SIDE_M / 2) < 1e-3
          and abs(positions[:, 0].max() - SIDE_M / 2) < 1e-3)

    uvs = np.fromstring(arrays["terrain-uvs-array"].text, sep=" ").reshape(-1, 2)
    check("texture coordinates stay in [0,1]",
          uvs.min() > -0.01 and uvs.max() < 1.01, f"{uvs.min():.3f}..{uvs.max():.3f}")
    check("origin vertex maps near the texture center",
          abs(uvs[center_index, 0] - 0.5) < 0.02 and abs(uvs[center_index, 1] - 0.5) < 0.02,
          str(uvs[center_index]))
    # North edge vertices (last row) must map to the image top (v near 1):
    # satellite row 0 is north.
    north_row = uvs[-(GRID_N + 1):, 1]
    check("north edge maps to the top of the texture", north_row.min() > 0.95,
          f"v {north_row.min():.3f}")

    texture = scenes_dir / "models" / "synthtest_terrain" / "materials" / "textures" / "satellite.jpg"
    check("texture copied into the model", texture.is_file())


def test_flatten_modes() -> None:
    print("flatten zone modes")
    grid = np.array([[100.0, 100], [110, 120]], dtype=np.float32)
    meta = {"grid_n": 1, "side_m": 100.0}
    zone = scene_model.FlattenZone(id="z", polygon_m=[[-60, -60], [60, -60], [60, 60], [-60, 60]],
                                   mode="min")
    result = terrain_mesh.apply_flatten_zones(grid, meta, [zone], 100.0)
    check("min mode takes the lowest vertex", float(result.max()) == 100.0,
          str(result.tolist()))
    zone.mode = "mean"
    result = terrain_mesh.apply_flatten_zones(grid, meta, [zone], 100.0)
    check("mean mode averages", abs(float(result[0, 0]) - 107.5) < 1e-6,
          str(result.tolist()))


def test_scenario(scenes_dir: Path, tmp: Path) -> None:
    print("casualty scenario")
    scenario_path = scenes_dir / "scenarios" / "synthtest_casualties.yaml"
    data = yaml.safe_load(scenario_path.read_text())
    entities = data["entities"]
    check("two casualties placed", len(entities) == 2)

    alpha = entities[0]
    check("designated name and model survive",
          alpha["name"] == "casualty_alpha" and alpha["uri"] == "model://casualty_prone")
    check("100 m east converts to x=100",
          abs(alpha["pose"][0] - 100.0) < 0.05 and abs(alpha["pose"][1]) < 0.05,
          str(alpha["pose"][:3]))
    check("no alt means ground height", abs(alpha["pose"][2]) < 0.01,
          str(alpha["pose"][2]))

    second = entities[1]
    check("generated name is scoreable",
          "casualty" in second["name"] or "person" in second["name"], second["name"])
    check("given AMSL alt becomes scene z", abs(second["pose"][2] - 5.0) < 0.01,
          str(second["pose"][2]))
    check("random model comes from the pool",
          second["uri"] in build_world.CASUALTY_MODEL_POOL, second["uri"])

    # Same seed, same draw: rebuild into a second directory and compare.
    rerun_dir = tmp / "scenes2"
    build_world.run(tmp / "data" / "synthtest", rerun_dir, tmp / "casualties.yaml", seed=7)
    again = yaml.safe_load((rerun_dir / "scenarios" / "synthtest_casualties.yaml").read_text())
    check("seeded assignment is repeatable",
          again["entities"][1]["uri"] == second["uri"]
          and again["entities"][1]["pose"] == second["pose"])


def test_env_snippet(tmp: Path) -> None:
    print("env snippet")
    text = (tmp / "data" / "synthtest" / "env.snippet").read_text()
    lines = dict(line.split("=", 1) for line in text.strip().splitlines())
    check("scene and home lines present",
          lines.get("SCENE") == "synthtest"
          and abs(float(lines["HOME_ALT"]) - ORIGIN_ALT) < 0.01)
    check("fiducial surveyed altitude is the ground AMSL",
          abs(float(lines["FIDUCIAL_SURVEYED_ALT"]) - ORIGIN_ALT) < 0.05,
          lines.get("FIDUCIAL_SURVEYED_ALT", "missing"))
    check("fiducial surveyed position is the center",
          abs(float(lines["FIDUCIAL_SURVEYED_LAT"]) - CENTER_LAT) < 1e-6
          and abs(float(lines["FIDUCIAL_SURVEYED_LON"]) - CENTER_LON) < 1e-6)


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp_name:
        tmp = Path(tmp_name)
        print("build")
        _, scenes_dir = run_build(tmp)
        test_world(scenes_dir)
        test_terrain(scenes_dir)
        test_flatten_modes()
        test_scenario(scenes_dir, tmp)
        test_env_snippet(tmp)

    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
