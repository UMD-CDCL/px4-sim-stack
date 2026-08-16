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
import subprocess
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
            mode="manual", height_m=0.0)],
        targets=[
            scene_model.Target(id="t_manual_1", name="person_manual",
                               east_m=-50.0, north_m=0.0),
            scene_model.Target(id="t_manual_2", name="casualty_off",
                               east_m=10.0, north_m=10.0, enabled=False),
            scene_model.Target(id="t_manual_3", name="casualty_roof",
                               east_m=30.0, north_m=-20.0, on_building=True),
            scene_model.Target(id="t_manual_4", name="casualty_offbuilding",
                               east_m=80.0, north_m=80.0, on_building=True,
                               agl_m=1.5)])
    scene_model.save_scene(scene, data_dir / "scene.json")
    return scene


def make_casualty_file(path: Path) -> None:
    frame = geo.GeoFrame(CENTER_LAT, CENTER_LON, ORIGIN_ALT)
    lat_east, lon_east, _ = frame.enu_to_latlon(100.0, 0.0)
    yaml_text = yaml.safe_dump({"casualties": [
        {"lat": lat_east, "lon": lon_east, "model": "model://casualty_prone",
         "name": "casualty_alpha"},
        {"lat": CENTER_LAT, "lon": CENTER_LON, "agl": 5.0},
        {"lat": CENTER_LAT, "lon": CENTER_LON, "name": "plain"},
    ]})
    path.write_text(yaml_text)


def run_build(tmp: Path) -> tuple[Path, Path]:
    data_dir = tmp / "data" / "synthtest"
    data_dir.mkdir(parents=True)
    scenes_dir = tmp / "scenes"
    scene = make_synthetic_scene(data_dir)
    casualties = tmp / "casualties.yaml"
    make_casualty_file(casualties)
    # The file imports into the scene; the scene is what the build reads.
    imported, kept = scene_model.import_casualty_file(scene, casualties)
    check("import adds file targets and keeps hand-placed ones",
          imported == 3 and kept == 4, f"imported {imported}, kept {kept}")
    imported, kept = scene_model.import_casualty_file(scene, casualties)
    check("a re-import replaces imports, not hand-placed targets",
          imported == 3 and kept == 4 and len(scene.targets) == 7)
    scene_model.save_scene(scene, data_dir / "scene.json")
    code = build_world.run(data_dir, scenes_dir)
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
    print("target scenario, projected from the scene")
    scenario_path = scenes_dir / "scenarios" / "synthtest_casualties.yaml"
    data = yaml.safe_load(scenario_path.read_text())
    entities = {e["name"]: e for e in data["entities"]}
    check("enabled targets are in, the disabled one is out",
          len(entities) == 6 and "casualty_off" not in entities,
          str(sorted(entities)))
    check("hand-placed target came through", "person_manual" in entities)

    roof = entities.get("casualty_roof")
    check("snap to building lands on the roof of the 7 m box",
          roof is not None and abs(roof["pose"][2] - 7.0) < 0.01,
          str(roof["pose"][2] if roof else None))
    fallback = entities.get("casualty_offbuilding")
    check("snap with no building there falls back to terrain plus offset",
          fallback is not None and abs(fallback["pose"][2] - 1.5) < 0.01,
          str(fallback["pose"][2] if fallback else None))

    alpha = entities.get("casualty_alpha")
    check("designated name and model survive",
          alpha is not None and alpha["uri"] == "model://casualty_prone")
    check("100 m east converts to x=100",
          alpha is not None and abs(alpha["pose"][0] - 100.0) < 0.05
          and abs(alpha["pose"][1]) < 0.05, str(alpha["pose"][:3] if alpha else None))
    check("no alt means ground height",
          alpha is not None and abs(alpha["pose"][2]) < 0.01)

    second = entities.get("casualty_02")
    check("agl height rides on the terrain",
          second is not None and abs(second["pose"][2] - 5.0) < 0.01)
    check("a pool model resolves for an undesignated target",
          second is not None and second["uri"] in scene_model.CASUALTY_MODEL_POOL,
          second["uri"] if second else "missing")
    check("a plain name gets a scoreable prefix", "casualty_plain" in entities)

    # No randomness at build: a rebuild of the same scene is byte-identical.
    rerun_dir = tmp / "scenes2"
    build_world.run(tmp / "data" / "synthtest", rerun_dir)
    check("a rebuild writes the same bytes",
          (rerun_dir / "scenarios" / "synthtest_casualties.yaml").read_bytes()
          == scenario_path.read_bytes())


def test_legacy_alt_key() -> None:
    print("legacy scene.json compatibility")
    legacy = json.dumps({
        "format": scene_model.SCENE_FORMAT, "name": "old", "center_lat": 38.9,
        "center_lon": -76.9, "side_m": 100, "origin_alt_m": 10.0,
        "targets": [{"id": "t_import_1", "name": "casualty_01",
                     "east_m": 1.0, "north_m": 2.0, "alt_m": 3.0}]})
    scene = scene_model.SceneSpec.from_json(legacy)
    check("a pre-rename alt_m key loads as agl_m",
          scene.targets[0].agl_m == 3.0, str(scene.targets[0]))


def test_env_snippet(tmp: Path) -> None:
    print("env snippet")
    text = (tmp / "data" / "synthtest" / "env.snippet").read_text()
    lines = [line for line in text.strip().splitlines() if not line.startswith("#")]
    check("only SCENE and SCENARIO remain to copy",
          lines == ["SCENE=synthtest", "SCENARIO=synthtest_casualties"], str(lines))


def test_scenario_metadata(scenes_dir: Path) -> None:
    print("home and fiducial ride in the scenario")
    data = yaml.safe_load(
        (scenes_dir / "scenarios" / "synthtest_casualties.yaml").read_text())
    check("home matches the scene origin",
          abs(data["home_lat"] - CENTER_LAT) < 1e-6
          and abs(data["home_lon"] - CENTER_LON) < 1e-6
          and abs(data["home_alt"] - ORIGIN_ALT) < 0.01)
    check("fiducial sits at the center on the ground",
          abs(data["fiducial_lat"] - CENTER_LAT) < 1e-6
          and abs(data["fiducial_lon"] - CENTER_LON) < 1e-6
          and abs(data["fiducial_alt"] - ORIGIN_ALT) < 0.05)


def test_scenario_env_script(scenes_dir: Path, tmp: Path) -> None:
    print("scenario-env.sh derives the .env values")
    script = Path(__file__).resolve().parents[3] / "scripts" / "scenario-env.sh"
    scenario = scenes_dir / "scenarios" / "synthtest_casualties.yaml"
    reply = subprocess.run(["bash", str(script), str(scenario)],
                           capture_output=True, text=True)
    values = dict(line.split("=", 1) for line in reply.stdout.strip().splitlines())
    check("a generated scenario yields home and fiducial",
          values.get("FIDUCIAL_ENABLED") == "1"
          and abs(float(values["HOME_ALT"]) - ORIGIN_ALT) < 0.01
          and abs(float(values["HOME_LAT"]) - CENTER_LAT) < 1e-6
          and abs(float(values["FIDUCIAL_SURVEYED_LAT"]) - CENTER_LAT) < 1e-6,
          reply.stdout)
    handwritten = tmp / "handwritten.yaml"
    handwritten.write_text("name: x\nentities: []\n")
    reply = subprocess.run(["bash", str(script), str(handwritten)],
                           capture_output=True, text=True)
    check("a hand-written scenario yields nothing, .env stands",
          reply.stdout == "", reply.stdout)


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp_name:
        tmp = Path(tmp_name)
        print("build")
        _, scenes_dir = run_build(tmp)
        test_world(scenes_dir)
        test_terrain(scenes_dir)
        test_flatten_modes()
        test_scenario(scenes_dir, tmp)
        test_legacy_alt_key()
        test_env_snippet(tmp)
        test_scenario_metadata(scenes_dir)
        test_scenario_env_script(scenes_dir, tmp)

    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
