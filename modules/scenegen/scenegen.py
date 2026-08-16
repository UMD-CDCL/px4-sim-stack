#!/usr/bin/env python3
"""Generate a Gazebo scene from a coordinate and a side length.

The pipeline is four stages, each resumable, all keyed by --name:

  create   fetch satellite imagery, elevation and OSM buildings for a
           square around --center, and write data/<name>/scene.json
  detect   find cars and buses in the imagery and add them to scene.json
  edit     serve a browser editor for scene.json: move, add and remove
           vehicles, ground-truth targets, trees, tree areas and terrain
           flatten zones, adjust buildings
  build    write the Gazebo world, the terrain model and the target
           scenario into modules/sim/scenes/

scene.json is the source of truth for ground-truth targets. A casualty
file only imports into it (--casualties on create or all, or the
import-casualties command); after that the editor owns them, and build
reads nothing but the scene.

`all` runs the stages in order and waits at the editor: the browser's
"Save & build" button lets the build stage run, Ctrl-C stops without
building.

    scenegen.py all --name campus --center 38.9869,-76.9426 --side 600 \\
                    --casualties casualties.yaml

    scenegen.py create --name campus --center 38.9869,-76.9426 --side 600
    scenegen.py import-casualties --name campus --casualties casualties.yaml
    scenegen.py detect --name campus
    scenegen.py edit   --name campus
    scenegen.py build  --name campus

build-all builds every scene in the data directory in one run. A fresh
clone carries the scene sources but none of the build products, so this
is the setup step that fills modules/sim/scenes.

Then: SCENE=campus and the printed HOME_* values in .env, and restart the
sim. See README.md.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

import geo
import scene_model
import sources

# ------------------------------------------------------------------- tunables
# Zoom 19 is about 0.22 m per pixel at 39 degrees latitude: a car spans
# 20 pixels. Zoom 20 doubles that where Google has it.
DEFAULT_IMAGERY_ZOOM = 19
DEFAULT_IMAGERY_SOURCE = "google"
# 128 cells across the scene. At 600 m that is a 4.7 m mesh step, matching
# the elevation data's own resolution.
DEFAULT_TERRAIN_GRID = 128
MAX_SIDE_M = 5000.0
MIN_SIDE_M = 50.0
SATELLITE_JPEG_QUALITY = 92

MODULE_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.environ.get("DATA_DIR", MODULE_DIR / "data"))
SCENES_DIR = Path(os.environ.get("SCENES_DIR", MODULE_DIR.parent / "sim" / "scenes"))


def scene_dir(name: str) -> Path:
    return DATA_DIR / name


def parse_center(text: str) -> tuple[float, float]:
    try:
        lat_text, lon_text = text.split(",")
        lat, lon = float(lat_text), float(lon_text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"--center wants LAT,LON, got {text!r}")
    if not (-85.0 < lat < 85.0 and -180.0 <= lon <= 180.0):
        raise argparse.ArgumentTypeError(f"center {lat},{lon} is off the map")
    return lat, lon


def cmd_create(args: argparse.Namespace) -> int:
    lat, lon = args.center
    directory = scene_dir(args.name)
    scene_path = directory / "scene.json"
    if scene_path.exists() and not args.force:
        print(f"{scene_path} exists. Your edits would be lost.\n"
              f"Use --force to refetch and start over.", file=sys.stderr)
        return 1
    directory.mkdir(parents=True, exist_ok=True)
    cache_dir = directory / "tiles"

    # Two frames on purpose. The elevation fetch needs a frame before the
    # origin altitude is known; heights are absolute AMSL, so a zero-alt
    # frame samples the same points.
    flat_frame = geo.GeoFrame(lat, lon, 0.0)

    print(f"[1/4] elevation, zoom {sources.ELEVATION_ZOOM}, "
          f"grid {args.terrain_grid}x{args.terrain_grid}")
    elevation = sources.fetch_elevation_grid(flat_frame, args.side, args.terrain_grid,
                                             cache_dir)
    center_index = args.terrain_grid // 2
    origin_alt = float(elevation[center_index, center_index])
    np.save(directory / "elevation.npy", elevation)
    elevation_meta = {"grid_n": args.terrain_grid, "side_m": args.side,
                      "zoom": sources.ELEVATION_ZOOM, "row0": "south",
                      "origin_alt_m": round(origin_alt, 2),
                      "min_m": round(float(elevation.min()), 2),
                      "max_m": round(float(elevation.max()), 2)}
    (directory / "elevation.json").write_text(json.dumps(elevation_meta, indent=1))
    print(f"      origin altitude {origin_alt:.1f} m AMSL, "
          f"relief {elevation.min():.1f} to {elevation.max():.1f} m")

    frame = geo.GeoFrame(lat, lon, origin_alt)

    print(f"[2/4] satellite imagery from {args.imagery}, zoom {args.imagery_zoom}")
    image, georef = sources.fetch_satellite_image(frame, args.side, args.imagery_zoom,
                                                  args.imagery, cache_dir)
    image.save(directory / "satellite.jpg", quality=SATELLITE_JPEG_QUALITY)
    resolution = geo.ground_resolution_m_per_px(lat, args.imagery_zoom)
    imagery_meta = {"source": args.imagery, "zoom": args.imagery_zoom,
                    "file": "satellite.jpg",
                    "width_px": image.width, "height_px": image.height,
                    "m_per_px": round(resolution, 4),
                    "origin_px": georef.origin_px, "origin_py": georef.origin_py}
    print(f"      {image.width}x{image.height} px, {resolution:.2f} m/px")

    print("[3/4] OSM buildings")
    raw_buildings, report = sources.fetch_osm_buildings(frame, args.side)
    sources.write_buildings_geojson(raw_buildings, directory / "buildings.geojson")
    buildings, dropped = scene_model.buildings_from_osm(raw_buildings, frame, args.side)
    print(f"      {len(buildings)} kept ({report['ways']} ways, "
          f"{report['relation_rings']} relation rings with {report['holes']} holes, "
          f"{dropped} outside the square)")

    print("[4/4] OSM trees and woods")
    raw_trees, raw_areas, veg_report = sources.fetch_osm_vegetation(frame, args.side)
    trees, tree_areas = scene_model.vegetation_from_osm(
        raw_trees, raw_areas, frame, args.side)
    print(f"      {len(trees)} trees, {len(tree_areas)} wooded areas kept "
          f"(of {veg_report['trees']} and {veg_report['areas']} fetched)")

    scene = scene_model.SceneSpec(
        name=args.name, center_lat=lat, center_lon=lon, side_m=args.side,
        origin_alt_m=round(origin_alt, 2), imagery=imagery_meta,
        elevation=elevation_meta, buildings=buildings, trees=trees,
        tree_areas=tree_areas)
    if args.casualties:
        imported, _ = scene_model.import_casualty_file(scene, Path(args.casualties))
        print(f"      {imported} ground-truth targets imported from {args.casualties}")
    scene_model.save_scene(scene, scene_path)
    print(f"\nWrote {scene_path}")
    print("Next: detect, then edit, then build. Or edit first and place "
          "vehicles by hand.")
    return 0


def cmd_detect(args: argparse.Namespace) -> int:
    import detect_vehicles
    return detect_vehicles.run(scene_dir(args.name), args.confidence,
                               args.replace_auto)


def cmd_edit(args: argparse.Namespace) -> int:
    import editor_server
    outcome = editor_server.serve(scene_dir(args.name), args.port)
    return 1 if outcome == "missing" else 0


def cmd_build(args: argparse.Namespace) -> int:
    import build_world
    return build_world.run(scene_dir(args.name), SCENES_DIR)


def cmd_import(args: argparse.Namespace) -> int:
    scene_path = scene_dir(args.name) / "scene.json"
    if not scene_path.is_file():
        print(f"No scene at {scene_path}. Run create first.", file=sys.stderr)
        return 1
    scene = scene_model.load_scene(scene_path)
    imported, kept = scene_model.import_casualty_file(scene, Path(args.casualties))
    scene_model.save_scene(scene, scene_path)
    print(f"{imported} ground-truth targets imported into {scene_path} "
          f"({kept} hand-placed targets kept). The editor can adjust them; "
          f"build writes the scenario from the scene.")
    return 0


def cmd_all(args: argparse.Namespace) -> int:
    import build_world
    import editor_server

    directory = scene_dir(args.name)
    scene_path = directory / "scene.json"
    if scene_path.exists() and not args.force:
        print(f"[all 1/4] keeping {scene_path} and your edits in it. "
              "--force refetches from scratch.")
        if args.casualties:
            reused = scene_model.load_scene(scene_path)
            imported, kept = scene_model.import_casualty_file(
                reused, Path(args.casualties))
            scene_model.save_scene(reused, scene_path)
            print(f"      {imported} ground-truth targets imported "
                  f"({kept} hand-placed kept)")
    else:
        if args.center is None or args.side is None:
            print("A new scene needs --center LAT,LON and --side METERS.",
                  file=sys.stderr)
            return 1
        print("[all 1/4] create")
        code = cmd_create(args)
        if code:
            return code

    scene = scene_model.load_scene(scene_path)
    if args.skip_detect:
        print("[all 2/4] detect skipped (--skip-detect)")
    elif scene.vehicles:
        print(f"[all 2/4] detect skipped: the scene already holds "
              f"{len(scene.vehicles)} vehicles, and a re-run would replace "
              "edited auto detections. Use the detect command to redo them.")
    else:
        import detect_vehicles
        print("[all 2/4] detect")
        code = detect_vehicles.run(directory, args.confidence, True)
        if code:
            return code

    print("[all 3/4] edit")
    outcome = editor_server.serve(directory, args.port, finish_enabled=True)
    if outcome != "finished":
        print(f"\nStopped before the build. When the scene is ready:\n"
              f"  scenegen.py build --name {args.name}")
        return 1

    print("[all 4/4] build")
    return build_world.run(directory, SCENES_DIR)


def cmd_build_all(_args: argparse.Namespace) -> int:
    """Build every scene that has a scene.json in DATA_DIR. One failure
    does not stop the rest; the exit code reports whether any failed."""
    import build_world

    scene_dirs = sorted(path for path in DATA_DIR.iterdir()
                        if (path / "scene.json").is_file()) \
        if DATA_DIR.is_dir() else []
    if not scene_dirs:
        print(f"No scenes to build. DATA_DIR is {DATA_DIR}")
        return 0
    failures = []
    for directory in scene_dirs:
        print(f"=== build {directory.name}")
        try:
            code = build_world.run(directory, SCENES_DIR)
        except Exception as error:  # noqa: BLE001 - report, then keep building
            print(f"{directory.name}: {error}", file=sys.stderr)
            code = 1
        if code:
            failures.append(directory.name)
        print()
    print(f"{len(scene_dirs) - len(failures)} of {len(scene_dirs)} scenes "
          f"built into {SCENES_DIR}")
    if failures:
        print("failed: " + ", ".join(failures), file=sys.stderr)
    return 1 if failures else 0


def cmd_list(_args: argparse.Namespace) -> int:
    if not DATA_DIR.is_dir():
        print(f"No scenes yet. DATA_DIR is {DATA_DIR}")
        return 0
    for path in sorted(DATA_DIR.iterdir()):
        scene_path = path / "scene.json"
        if not scene_path.is_file():
            continue
        scene = scene_model.load_scene(scene_path)
        vehicles = sum(1 for v in scene.vehicles if v.enabled)
        buildings = sum(1 for b in scene.buildings if b.enabled)
        targets = sum(1 for t in scene.targets if t.enabled)
        print(f"  {scene.name:20s} {scene.center_lat:.4f},{scene.center_lon:.4f}  "
              f"{scene.side_m:.0f} m  {buildings} buildings  {vehicles} vehicles  "
              f"{targets} targets")
    return 0


def _fetch_options(parser, *, required: bool) -> None:
    """The create arguments, shared with `all`. `all` takes them optional,
    because an existing scene.json is reused as is."""
    parser.add_argument("--center", type=parse_center, required=required,
                        help="LAT,LON" if required else
                             "LAT,LON. Needed for a new scene; an existing "
                             "scene.json is reused as is.")
    parser.add_argument("--side", type=float, required=required,
                        help="square side, meters")
    parser.add_argument("--imagery", default=DEFAULT_IMAGERY_SOURCE,
                        choices=sorted(sources.IMAGERY_URL_TEMPLATES))
    parser.add_argument("--imagery-zoom", type=int, default=DEFAULT_IMAGERY_ZOOM)
    parser.add_argument("--terrain-grid", type=int, default=DEFAULT_TERRAIN_GRID)
    parser.add_argument("--casualties",
                        help="YAML casualty list to import as ground-truth "
                             "targets; scene.json owns them afterwards")
    parser.add_argument("--force", action="store_true",
                        help="refetch and overwrite an existing scene.json")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)

    def command(name: str, handler, help: str):
        parser = sub.add_parser(name, help=help)
        parser.set_defaults(handler=handler)
        return parser

    everything = command(
        "all", cmd_all,
        "create, detect, edit and build in one run; the build starts when "
        "the browser sends Save & build")
    everything.add_argument("--name", required=True)
    _fetch_options(everything, required=False)
    everything.add_argument("--skip-detect", action="store_true",
                            help="no vehicle detection; place vehicles by hand")
    everything.add_argument("--confidence", type=float, default=0.10)
    everything.add_argument("--port", type=int, default=8090)

    create = command("create", cmd_create, "fetch data and write scene.json")
    create.add_argument("--name", required=True,
                        help="scene name, also the world name")
    _fetch_options(create, required=True)

    importer = command("import-casualties", cmd_import,
                       "load a lat/lon casualty file into the scene's "
                       "ground-truth targets")
    importer.add_argument("--name", required=True)
    importer.add_argument("--casualties", required=True,
                          help="YAML list; replaces earlier imported targets, "
                               "keeps hand-placed ones")

    detect = command("detect", cmd_detect, "find vehicles in the imagery")
    detect.add_argument("--name", required=True)
    # 0.10 found about 3x the vehicles of 0.25 on a campus test with few
    # false boxes, and deleting a false box costs less than placing a
    # missed one. Raise it if your imagery earns junk detections.
    detect.add_argument("--confidence", type=float, default=0.10)
    detect.add_argument("--keep-auto", dest="replace_auto", action="store_false",
                        help="add to earlier detections instead of replacing them")

    edit = command("edit", cmd_edit, "serve the browser editor")
    edit.add_argument("--name", required=True)
    edit.add_argument("--port", type=int, default=8090)

    build = command("build", cmd_build,
                    "write the world and the target scenario into "
                    "modules/sim/scenes, all from scene.json")
    build.add_argument("--name", required=True)

    command("build-all", cmd_build_all,
            "build every scene in the data directory; the setup step for a "
            "fresh clone")
    command("list", cmd_list, "show the scenes on disk")

    args = ap.parse_args()
    if getattr(args, "side", None) is not None \
            and not (MIN_SIDE_M <= args.side <= MAX_SIDE_M):
        ap.error(f"--side must be {MIN_SIDE_M:.0f} to {MAX_SIDE_M:.0f} meters")
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
