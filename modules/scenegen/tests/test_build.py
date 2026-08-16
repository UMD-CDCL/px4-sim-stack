#!/usr/bin/env python3
"""Ground-truth tests for the build stage. No network.

Run: python3 tests/test_build.py

A synthetic scene with hand-computable geometry goes through the real
build: flat ground at 100 m AMSL with one 10 m bump, one 20x10x7 building
rotated 30 degrees, one L-shaped building, one building with a courtyard
hole, one building straddling the square edge, one building fully outside
the square, one car heading 45 degrees, one flatten zone over the bump,
and casualties at known offsets. Every expected number below is computed
by hand from those inputs.
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
import building_mesh
import geo
import scene_model
import sources
import terrain_mesh

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "modules" / "ros"
                       / "stacks" / "baseline" / "sim_bridge"))
from sim_bridge import scene_surface  # noqa: E402 - path set just above

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

    # The L covers a 20 x 20 square minus its 12 x 12 northeast notch, so
    # its area is 400 - 144 = 256. The courtyard ring is a 16 x 16 square
    # minus a 4 x 4 hole, 240. Rectangle fields come from the same fit the
    # create stage uses, so the build maps the outlines through identity.
    l_outline = [[-40.0, -70.0], [-20.0, -70.0], [-20.0, -62.0], [-32.0, -62.0],
                 [-32.0, -50.0], [-40.0, -50.0], [-40.0, -70.0]]
    court_outline = [[52.0, 32.0], [68.0, 32.0], [68.0, 48.0], [52.0, 48.0],
                     [52.0, 32.0]]
    court_hole = [[58.0, 38.0], [62.0, 38.0], [62.0, 42.0], [58.0, 42.0],
                  [58.0, 38.0]]

    def outlined(building_id: str, outline: list, holes: list,
                 height_m: float) -> scene_model.Building:
        east, north, length, width, yaw = scene_model.oriented_rectangle(
            [tuple(p) for p in outline])
        return scene_model.Building(
            id=building_id, east_m=round(east, 2), north_m=round(north, 2),
            length_m=round(length, 2), width_m=round(width, 2),
            yaw_deg=round(yaw, 2), height_m=height_m,
            outline_m=outline, holes_m=holes)

    scene = scene_model.SceneSpec(
        name="synthtest", center_lat=CENTER_LAT, center_lon=CENTER_LON,
        side_m=SIDE_M, origin_alt_m=ORIGIN_ALT, imagery=imagery, elevation=meta,
        buildings=[scene_model.Building(
            id="b_test", east_m=30.0, north_m=-20.0, length_m=20.0, width_m=10.0,
            yaw_deg=30.0, height_m=7.0),
            outlined("b_lshape", l_outline, [], 5.0),
            outlined("b_court", court_outline, [court_hole], 10.0),
            # Straddles the east edge of the 200 m square: x 90..110 clips
            # to 90..100, so half the 20 x 10 rectangle survives.
            scene_model.Building(
                id="b_edge", east_m=100.0, north_m=-80.0, length_m=20.0,
                width_m=10.0, yaw_deg=0.0, height_m=6.0),
            scene_model.Building(
                id="b_outside", east_m=140.0, north_m=140.0, length_m=10.0,
                width_m=10.0, yaw_deg=0.0, height_m=6.0),
            # A 20 m tower inside a 6 m podium: they merge into one
            # building whose roof is the upper envelope, 100 m2 at 20 m
            # and 320 m2 at 6 m, 420 m2 in total, the union footprint.
            scene_model.Building(
                id="b_tower", east_m=-70.0, north_m=-20.0, length_m=10.0,
                width_m=10.0, yaw_deg=0.0, height_m=20.0),
            scene_model.Building(
                id="b_podium", east_m=-70.0, north_m=-20.0, length_m=30.0,
                width_m=14.0, yaw_deg=0.0, height_m=6.0),
            # Two equal 5 m boxes overlapping by 32 m2: the envelope
            # assigns the overlap to one of them, 96 m2 in total, no
            # doubled roof to z-fight.
            scene_model.Building(
                id="b_eq1", east_m=78.0, north_m=70.0, length_m=8.0,
                width_m=8.0, yaw_deg=0.0, height_m=5.0),
            scene_model.Building(
                id="b_eq2", east_m=82.0, north_m=70.0, length_m=8.0,
                width_m=8.0, yaw_deg=0.0, height_m=5.0)],
        vehicles=[scene_model.Vehicle(
            id="v_1", cls="car", east_m=10.0, north_m=5.0, length_m=4.5,
            width_m=1.9, heading_deg=45.0, source="manual"),
            scene_model.Vehicle(
                id="v_2", cls="bus", east_m=-20.0, north_m=30.0, length_m=12.0,
                width_m=2.5, heading_deg=10.0, source="manual"),
            scene_model.Vehicle(
                id="v_3", cls="car", east_m=30.0, north_m=-20.0, length_m=4.5,
                width_m=1.9, heading_deg=0.0, source="manual", on_building=True),
            scene_model.Vehicle(
                id="v_4", cls="car", east_m=60.0, north_m=-60.0, length_m=4.5,
                width_m=1.9, heading_deg=0.0, source="manual", agl_m=2.0),
            # No on_building given: snapping is the default, and the
            # courtyard ring under it is 10 m tall.
            scene_model.Vehicle(
                id="v_5", cls="car", east_m=55.0, north_m=44.0, length_m=4.5,
                width_m=1.9, heading_deg=0.0, source="manual"),
            scene_model.Vehicle(
                id="v_6", cls="bus", east_m=-60.0, north_m=60.0, length_m=12.0,
                width_m=2.5, heading_deg=10.0, source="manual",
                model_uri=build_world.VEHICLE_MODEL_POOLS["bus"][0])],
        trees=[
            scene_model.Tree(id="tr_1", east_m=-90.0, north_m=-90.0),
            scene_model.Tree(id="tr_oak", east_m=-80.0, north_m=-90.0,
                             model_uri=scene_model.TREE_MODEL_POOL[-1]["uri"]),
            scene_model.Tree(id="tr_under", east_m=30.0, north_m=-20.0),
            scene_model.Tree(id="tr_out", east_m=150.0, north_m=0.0)],
        # 35 x 35 m at 300 per hectare, heights 4 to 7: pine and oak only.
        tree_areas=[scene_model.TreeArea(
            id="ta_1", polygon_m=[[-95.0, 60.0], [-60.0, 60.0],
                                  [-60.0, 95.0], [-95.0, 95.0]],
            density_per_ha=300.0, min_height_m=4.0, max_height_m=7.0)],
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
                               agl_m=1.5),
            scene_model.Target(id="t_manual_5", name="casualty_notch",
                               east_m=-25.0, north_m=-55.0, on_building=True),
            scene_model.Target(id="t_manual_6", name="casualty_court",
                               east_m=60.0, north_m=40.0, on_building=True),
            scene_model.Target(id="t_manual_7", name="casualty_ring",
                               east_m=55.0, north_m=40.0, on_building=True),
            scene_model.Target(id="t_manual_8", name="casualty_onclip",
                               east_m=95.0, north_m=-80.0, on_building=True),
            scene_model.Target(id="t_manual_9", name="casualty_offclip",
                               east_m=105.0, north_m=-80.0, on_building=True)])
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
          imported == 3 and kept == 9, f"imported {imported}, kept {kept}")
    imported, kept = scene_model.import_casualty_file(scene, casualties)
    check("a re-import replaces imports, not hand-placed targets",
          imported == 3 and kept == 9 and len(scene.targets) == 12)
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

    includes = {inc.find("name").text: inc for inc in world.findall("include")}
    check("terrain include present", "synthtest_terrain" in includes)
    check("buildings model include present", "synthtest_buildings" in includes
          and includes["synthtest_buildings"].find("uri").text
          == "model://synthtest_buildings")
    vehicle = includes.get("v_1")
    check("vehicle include present", vehicle is not None)
    if vehicle is not None:
        vehicle_pose = [float(v) for v in vehicle.find("pose").text.split()]
        check("vehicle pose and heading",
              abs(vehicle_pose[0] - 10) < 0.01 and abs(vehicle_pose[2]) < 0.01
              and abs(vehicle_pose[5] - math.radians(45)) < 1e-3)
        check("vehicle model drawn from the car pool",
              vehicle.find("uri").text in build_world.VEHICLE_MODEL_POOLS["car"])
    bus = includes.get("v_2")
    check("bus include present and drawn from the bus pool",
          bus is not None
          and bus.find("uri").text in build_world.VEHICLE_MODEL_POOLS["bus"])
    pinned = includes.get("v_6")
    check("the Bus model turns its measured 90 degrees onto the box heading",
          pinned is not None
          and abs(float(pinned.find("pose").text.split()[5])
                  - math.radians(10.0 + 90.0)) < 1e-3,
          pinned.find("pose").text if pinned is not None else "missing")
    roof_car = includes.get("v_3")
    check("a vehicle snapped to the building rides on its 7 m roof",
          roof_car is not None
          and abs(float(roof_car.find("pose").text.split()[2]) - 7.0) < 0.01,
          roof_car.find("pose").text if roof_car is not None else "missing")
    default_car = includes.get("v_5")
    check("snapping is the default: an unmarked vehicle rides the 10 m ring",
          default_car is not None
          and abs(float(default_car.find("pose").text.split()[2]) - 10.0) < 0.01,
          default_car.find("pose").text if default_car is not None else "missing")
    raised_car = includes.get("v_4")
    check("a vehicle offset rides that far above the terrain",
          raised_car is not None
          and abs(float(raised_car.find("pose").text.split()[2]) - 2.0) < 0.01,
          raised_car.find("pose").text if raised_car is not None else "missing")

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


def test_trees(scenes_dir: Path) -> None:
    print("trees: placement, height range, drops")
    check("the placement hash is frozen across languages",
          scene_model.fnv1a("") == 2166136261
          and scene_model.fnv1a("a") == 3826002220,
          str(scene_model.fnv1a("a")))

    root = ET.parse(scenes_dir / "worlds" / "synthtest.sdf").getroot()
    includes = {inc.find("name").text: inc
                for inc in root.find("world").findall("include")}
    pool_uris = {m["uri"] for m in scene_model.TREE_MODEL_POOL}

    tree = includes.get("tr_1")
    check("an individual tree stands on the terrain, model from the pool",
          tree is not None
          and abs(float(tree.find("pose").text.split()[2])) < 0.01
          and tree.find("uri").text in pool_uris,
          tree.find("uri").text if tree is not None else "missing")
    oak = includes.get("tr_oak")
    check("a designated tree model survives",
          oak is not None
          and oak.find("uri").text == scene_model.TREE_MODEL_POOL[-1]["uri"])
    check("a tree under a building is dropped", "tr_under" not in includes)
    check("a tree outside the square is dropped", "tr_out" not in includes)

    area = scene_model.TreeArea(
        id="ta_1", polygon_m=[[-95.0, 60.0], [-60.0, 60.0],
                              [-60.0, 95.0], [-95.0, 95.0]],
        density_per_ha=300.0, min_height_m=4.0, max_height_m=7.0)
    expected = scene_model.area_tree_points(area)
    filled = [name for name in includes if name.startswith("tr_ta_1_")]
    check("the area fills with exactly the generated trees",
          len(filled) == len(expected) > 10,
          f"{len(filled)} placed, {len(expected)} generated")
    inside = all(scene_model.polygon_contains(
        area.polygon_m, *[float(v) for v in
                          includes[name].find("pose").text.split()[:2]])
        for name in filled)
    check("every generated tree stands inside its area", inside)
    in_range = {m["uri"] for m in scene_model.TREE_MODEL_POOL
                if 4.0 <= m["height_m"] <= 7.0}
    species = {includes[name].find("uri").text for name in filled}
    check("the height range selects models, no rescaling",
          species and species <= in_range and len(in_range) == 2,
          str(sorted(u.rsplit("/", 1)[-1] for u in species)))


def test_buildings(scenes_dir: Path) -> None:
    print("buildings: mesh, texture, viz payload, surface")
    model_dir = scenes_dir / "models" / "synthtest_buildings"
    check("satellite texture copied into the buildings model",
          (model_dir / "materials" / "textures" / "satellite.jpg").is_file())

    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    root = ET.parse(model_dir / "meshes" / "buildings.dae").getroot()
    check("buildings mesh states Z_UP",
          root.find("c:asset/c:up_axis", ns).text == "Z_UP")
    geometries = root.findall("c:library_geometries/c:geometry", ns)
    geometry_ids = [g.get("id") for g in geometries]
    check("one geometry per merged building, the outside one dropped",
          len(geometries) == 6
          and "b_outside-geometry" not in geometry_ids
          and "b_tower-geometry" not in geometry_ids
          and "b_eq2-geometry" not in geometry_ids
          and "b_podium-geometry" in geometry_ids,
          str(geometry_ids))
    for geometry in geometries:
        groups = geometry.findall("c:mesh/c:triangles", ns)
        symbols = [g.get("material") for g in groups]
        check(f"{geometry.get('id')} carries a textured roof and a wall group",
              "satellite-symbol" in symbols
              and any(s.endswith("-wall-symbol") for s in symbols), str(symbols))
        for group in groups:
            indices = group.find("c:p", ns).text.split()
            check(f"{geometry.get('id')}/{group.get('material')} triples "
                  f"its indices",
                  len(indices) == 9 * int(group.get("count")))
    arrays = {a.get("id"): a for a in
              root.iter("{http://www.collada.org/2005/11/COLLADASchema}float_array")}
    positions = np.fromstring(arrays["b_test-positions-array"].text,
                              sep=" ").reshape(-1, 3)
    check("rectangle fallback tops out at its height and reaches the ground",
          abs(positions[:, 2].max() - 7.0) < 1e-3
          and abs(positions[:, 2].min()) < 1e-3,
          f"z {positions[:, 2].min()}..{positions[:, 2].max()}")
    normal_count = np.fromstring(arrays["b_test-normals-array"].text,
                                 sep=" ").size
    check("per-vertex normals match the vertex count",
          normal_count == positions.size)

    viz = json.loads(
        (scenes_dir / "worlds" / "synthtest_buildings.json").read_text())
    check("viz payload format", viz["format"] == "scenegen-buildings/1"
          and viz["frame"] == "map")
    entries = {b["id"]: b for b in viz["buildings"]}
    check("viz carries one entry per merged building",
          sorted(entries) == ["b_court", "b_edge", "b_eq1", "b_lshape",
                              "b_podium", "b_test"],
          str(sorted(entries)))

    def roof_area(entry: dict) -> float:
        triangles = np.array(entry["roof"]["points"]).reshape(-1, 3, 3)
        return building_mesh.triangle_area_m2(triangles[:, :, :2])

    check("L roof area is the L, not its bounding box",
          abs(roof_area(entries["b_lshape"]) - 256.0) < 1.0,
          f"{roof_area(entries['b_lshape']):.1f}")
    check("courtyard roof area excludes the hole",
          abs(roof_area(entries["b_court"]) - 240.0) < 1.0,
          f"{roof_area(entries['b_court']):.1f}")
    check("the straddling building keeps only its inside half",
          abs(roof_area(entries["b_edge"]) - 100.0) < 0.5,
          f"{roof_area(entries['b_edge']):.1f}")
    merged = np.array(entries["b_podium"]["roof"]["points"]).reshape(-1, 3, 3)
    check("the merged building's roof is the union footprint",
          abs(roof_area(entries["b_podium"]) - 420.0) < 1.0,
          f"{roof_area(entries['b_podium']):.1f}")
    tower_part = merged[np.isclose(merged[:, :, 2], 20.0).all(axis=1)]
    check("the tower part of the envelope rides at its own 20 m",
          abs(building_mesh.triangle_area_m2(tower_part[:, :, :2]) - 100.0) < 0.5,
          f"{building_mesh.triangle_area_m2(tower_part[:, :, :2]):.1f}")
    check("equal-height overlap yields one roof, not two",
          abs(roof_area(entries["b_eq1"]) - 96.0) < 0.5,
          f"{roof_area(entries['b_eq1']):.1f}")
    edge_points = np.array(entries["b_edge"]["roof"]["points"]
                           + entries["b_edge"]["walls"]["points"])
    check("no clipped vertex pokes past the square",
          float(edge_points[:, 0].max()) <= 100.0 + 0.01,
          f"max east {edge_points[:, 0].max():.2f}")
    surface_data = json.loads(
        (scenes_dir / "worlds" / "synthtest_surface.json").read_text())
    largest = max(abs(value) for b in surface_data["buildings"]
                  for ring in [b["footprint"], *b["holes"]]
                  for point in ring for value in point)
    check("surface footprints stay inside the square", largest <= 100.01,
          f"max |coordinate| {largest:.2f}")
    check("a satellite color pairs with every roof vertex",
          all(len(b["roof"]["points"]) % 3 == 0
              and len(b["roof"]["colors"]) == len(b["roof"]["points"])
              for b in viz["buildings"]))
    sampled = entries["b_test"]["roof"]["colors"][0]
    check("the synthetic satellite color lands on the roof, JPEG noise aside",
          all(abs(sampled[i] - [90 / 255, 120 / 255, 80 / 255][i]) < 0.02
              for i in range(3)),
          str(sampled))

    surface = scene_surface.SceneSurface.load(
        str(scenes_dir / "worlds" / "synthtest_surface.json"))
    down = (0.0, 0.0, -1.0)
    courtyard = surface.intersect((60.0, 40.0, 50.0), down, 200.0)
    check("a ray into the courtyard reaches the terrain",
          courtyard is not None and abs(courtyard[2]) < 0.05, str(courtyard))
    ring = surface.intersect((55.0, 40.0, 50.0), down, 200.0)
    check("a ray onto the courtyard ring lands on the 10 m roof",
          ring is not None and abs(ring[2] - 10.0) < 0.01, str(ring))
    notch = surface.intersect((-25.0, -55.0, 50.0), down, 200.0)
    check("a ray into the L notch reaches the terrain",
          notch is not None and abs(notch[2]) < 0.05, str(notch))


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
          len(entities) == 11 and "casualty_off" not in entities,
          str(sorted(entities)))
    check("hand-placed target came through", "person_manual" in entities)

    roof = entities.get("casualty_roof")
    check("snap to building lands on the roof of the 7 m building",
          roof is not None and abs(roof["pose"][2] - 7.0) < 0.01,
          str(roof["pose"][2] if roof else None))
    fallback = entities.get("casualty_offbuilding")
    check("snap with no building there falls back to terrain plus offset",
          fallback is not None and abs(fallback["pose"][2] - 1.5) < 0.01,
          str(fallback["pose"][2] if fallback else None))
    notch = entities.get("casualty_notch")
    check("snap inside the L notch falls to the terrain",
          notch is not None and abs(notch["pose"][2]) < 0.01,
          str(notch["pose"][2] if notch else None))
    court = entities.get("casualty_court")
    check("snap inside the courtyard falls to the terrain",
          court is not None and abs(court["pose"][2]) < 0.01,
          str(court["pose"][2] if court else None))
    ring = entities.get("casualty_ring")
    check("snap on the courtyard ring lands on the 10 m roof",
          ring is not None and abs(ring["pose"][2] - 10.0) < 0.01,
          str(ring["pose"][2] if ring else None))
    onclip = entities.get("casualty_onclip")
    check("snap on the kept half of a clipped building lands on its roof",
          onclip is not None and abs(onclip["pose"][2] - 6.0) < 0.01,
          str(onclip["pose"][2] if onclip else None))
    offclip = entities.get("casualty_offclip")
    check("snap on the cut-away half falls to the terrain",
          offclip is not None and abs(offclip["pose"][2]) < 0.01,
          str(offclip["pose"][2] if offclip else None))

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
        test_buildings(scenes_dir)
        test_trees(scenes_dir)
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
