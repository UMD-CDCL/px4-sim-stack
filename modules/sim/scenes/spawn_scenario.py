#!/usr/bin/env python3
"""Place scenario entities into a running Gazebo world.

A scene is the terrain. A scenario is what stands on it. They are separate so
that one scene can carry many test cases, and so that a target layout can
change without a simulator restart.

    spawn_scenario.py --world lorton --scenario scenarios/lorton_casualties.yaml
    spawn_scenario.py --world lorton --clear
    spawn_scenario.py --list
    spawn_scenario.py --world lorton --fiducial 6 -9

The last one stands the survey marker away from the coordinate the vehicles
were given, which is how the simulator holds a frame error. See
`move_fiducial`.

The script talks to Gazebo through the `gz service` command line tool, so it
needs no Python bindings.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import yaml

STATE_FILE = Path("/tmp/px4simstack-scenario.json")
SCENES_DIR = Path(os.environ.get("SCENES_DIR", "/scenes"))
# Where the spawner records the poses Gazebo actually gave each entity. The
# ROS ground truth node prefers this over the scenario file.
RESOLVED_FILE = Path(os.environ.get("RESOLVED_TRUTH_FILE", "/tmp/ground_truth_actual.yaml"))
# How many `gz service` calls run at the same time. The calls are independent
# and Gazebo accepts them concurrently.
GZ_SERVICE_WORKERS = 4
# The longest wait for spawned entities to show up in the pose stream, and how
# often to look. The old fixed sleep paid the full ceiling every time.
SPAWN_SETTLE_CEILING_S = 1.5
POSE_POLL_INTERVAL_S = 0.2
# The survey marker in every generated world. modules/scenegen/build_world.py
# writes it, at the coordinate the scenario hands the vehicles as fiducial_lat
# and fiducial_lon.
FIDUCIAL_MODEL = "fiducial_marker"


def gz_service(service: str, reqtype: str, req: str, timeout_ms: int = 8000,
               label: str = "") -> bool:
    """Call one Gazebo service. Return True when Gazebo answers with data: true.

    Calls run concurrently, so `label` names the entity in error lines.
    """
    prefix = f"[{label}] " if label else ""
    cmd = [
        "gz", "service", "-s", service,
        "--reqtype", reqtype,
        "--reptype", "gz.msgs.Boolean",
        "--timeout", str(timeout_ms),
        "--req", req,
    ]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    except subprocess.TimeoutExpired:
        print(f"    {prefix}timed out calling {service}", file=sys.stderr)
        return False
    if "data: true" in out.stdout:
        return True
    detail = (out.stdout + out.stderr).strip().replace("\n", " ")
    print(f"    {prefix}service {service} refused: {detail[:200]}", file=sys.stderr)
    return False


def pose_element(pose) -> str:
    if not pose:
        return ""
    values = list(pose) + [0] * (6 - len(pose))
    return "<pose>" + " ".join(str(v) for v in values[:6]) + "</pose>"


def spawn(world: str, entity: dict) -> bool:
    name = entity["name"]
    uri = entity["uri"]
    static = "<static>true</static>" if entity.get("static", True) else ""
    # Double quotes only. The whole block goes inside a single-quoted protobuf
    # text field below, so a single quote here would end the field early.
    sdf = (
        '<sdf version="1.9"><include>'
        f"<uri>{uri}</uri><name>{name}</name>"
        f"{pose_element(entity.get('pose'))}{static}"
        "</include></sdf>"
    )
    req = f"name: \"{name}\", allow_renaming: false, sdf: '{sdf}'"
    return gz_service(f"/world/{world}/create", "gz.msgs.EntityFactory", req, label=name)


def remove(world: str, name: str) -> bool:
    req = f'name: "{name}", type: MODEL'
    return gz_service(f"/world/{world}/remove", "gz.msgs.Entity", req, label=name)


def read_world_poses(world: str) -> dict[str, tuple[float, float, float]]:
    """Ask Gazebo where every model actually is.

    The scenario file says where entities were *asked* to go. This reads back
    where they ended up. A model whose mesh origin is not at its feet, or one
    that settles under gravity, or one that failed to spawn and left an older
    copy in place, all show up as a difference between the two. Scoring against
    the request rather than the result quietly measures the wrong thing.
    """
    cmd = ["gz", "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {}

    poses: dict[str, tuple[float, float, float]] = {}
    name = None
    pending: dict[str, float] = {}
    in_position = False
    for line in out.splitlines():
        stripped = line.strip()
        if stripped.startswith("name:"):
            # Only the outermost name of each pose block matters; link names
            # appear too, so keep the first and let the next block replace it.
            name = stripped.split('"')[1] if '"' in stripped else None
            pending, in_position = {}, False
        elif stripped.startswith("position"):
            in_position = True
        elif in_position and stripped.startswith(("x:", "y:", "z:")):
            key, _, value = stripped.partition(":")
            try:
                pending[key.strip()] = float(value)
            except ValueError:
                pass
            if len(pending) == 3 and name:
                poses.setdefault(name, (pending["x"], pending["y"], pending["z"]))
                in_position = False
    return poses


def wait_for_poses(world: str, names: list[str]) -> dict[str, tuple[float, float, float]]:
    """Poll the world pose data until every spawned name appears.

    A spawned model shows up in the pose stream a physics step later. The poll
    returns as soon as Gazebo reports every entity, and it never waits past
    the ceiling that the old fixed sleep paid in full every run.
    """
    wanted = set(names)
    deadline = time.monotonic() + SPAWN_SETTLE_CEILING_S
    while True:
        poses = read_world_poses(world)
        if wanted <= poses.keys() or time.monotonic() >= deadline:
            return poses
        time.sleep(POSE_POLL_INTERVAL_S)


def write_resolved(world: str, entities: list[dict],
                   actual: dict[str, tuple[float, float, float]]) -> None:
    """Record the actual pose of every entity we placed."""
    if not actual:
        print("    could not read back poses from Gazebo; scoring will use the "
              "scenario file", file=sys.stderr)
        return
    resolved = []
    for entity in entities:
        name = entity["name"]
        if name not in actual:
            continue
        x, y, z = actual[name]
        asked = list(entity.get("pose", [0, 0, 0]))[:3]
        drift = max(abs(a - b) for a, b in zip(asked + [0, 0, 0], [x, y, z]))
        resolved.append({"name": name, "uri": entity.get("uri", ""),
                         "pose": [x, y, z], "requested": asked,
                         "drift_m": round(drift, 3)})
    RESOLVED_FILE.write_text(yaml.safe_dump(
        {"world": world, "entities": resolved}, sort_keys=False))
    worst = max((e["drift_m"] for e in resolved), default=0.0)
    print(f"    recorded {len(resolved)} actual poses to {RESOLVED_FILE} "
          f"(largest difference from the request: {worst:.2f} m)")


def world_file(world: str) -> Path:
    return SCENES_DIR / "worlds" / f"{world}.sdf"


def surveyed_pose(world: str) -> tuple[float, float, float] | None:
    """Where the world file stands the survey marker.

    That pose IS the surveyed coordinate: scenegen writes the disk and the
    scenario's fiducial_lat and fiducial_lon from one place in the scene, so
    the world file is the record of where the vehicles were told the marker
    is.
    """
    path = world_file(world)
    if not path.is_file():
        print(f"No world file at {path}", file=sys.stderr)
        return None
    text = path.read_text()
    block = re.search(rf'<model name="{FIDUCIAL_MODEL}">(.*?)</model>', text,
                      re.DOTALL)
    pose = re.search(r"<pose>([^<]*)</pose>", block.group(1)) if block else None
    if pose is None:
        print(f"{path} holds no {FIDUCIAL_MODEL}. Rebuild the scene: "
              f"modules/scenegen writes it.", file=sys.stderr)
        return None
    values = [float(v) for v in pose.group(1).split()]
    return values[0], values[1], values[2]


def terrain_height(world: str, east: float, north: float) -> float | None:
    """The scene surface's own height at a point, world z.

    Read from the same `<scene>_surface.json` the vehicles cast their rays
    at, so the marker sits on the ground the localization believes in. None
    when the file is missing or the point is outside the square.
    """
    path = world_file(world).with_name(f"{world}_surface.json")
    if not path.is_file():
        return None
    surface = json.loads(path.read_text())
    grid, n, side = surface["terrain_z"], surface["grid_n"], surface["side_m"]
    x = (east + side / 2.0) / (side / n)
    y = (north + side / 2.0) / (side / n)
    if not (0.0 <= x <= n and 0.0 <= y <= n):
        return None
    i, j = min(int(x), n - 1), min(int(y), n - 1)
    fx, fy = x - i, y - j
    return float(
        grid[j][i] * (1 - fx) * (1 - fy) + grid[j][i + 1] * fx * (1 - fy)
        + grid[j + 1][i] * (1 - fx) * fy + grid[j + 1][i + 1] * fx * fy)


def move_fiducial(world: str, east: float, north: float) -> int:
    """Stand the survey marker this far from the coordinate it was surveyed at.

    A vehicle cannot tell a displaced marker from a frame of its own that is
    displaced the other way, so this is how the simulator holds a frame error
    without touching the autopilot. The vehicle photographs the marker,
    localizes it, compares it against the coordinate it was given, and must
    publish a correction of minus this offset. Every later localization then
    moves by that correction.

    Zeros put the marker back where the scene surveyed it, which is the state
    every scoring run needs.
    """
    base = surveyed_pose(world)
    if base is None:
        return 1
    east_at, north_at = base[0] + east, base[1] + north
    height, was = terrain_height(world, east_at, north_at), \
        terrain_height(world, base[0], base[1])
    z = base[2] if height is None or was is None else base[2] + height - was
    ok = gz_service(
        f"/world/{world}/set_pose", "gz.msgs.Pose",
        f'name: "{FIDUCIAL_MODEL}", position: {{x: {east_at}, y: {north_at}, '
        f"z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}",
        label=FIDUCIAL_MODEL)
    if not ok:
        return 1
    print(f"{FIDUCIAL_MODEL} stands at ({east_at:.2f}, {north_at:.2f}, "
          f"{z:.2f}), which is ({east:+.2f}, {north:+.2f}) m from the "
          f"coordinate the vehicles were given.")
    if east or north:
        print(f"A survey of it must publish a correction of "
              f"({-east:+.2f}, {-north:+.2f}) m east and north, and every "
              f"later localization must move by that.")
    else:
        print("A survey of it must publish a correction of about zero.")
    if height is None:
        print("    no scene surface to read, so the marker keeps its old "
              "height. Check it is not buried or floating.", file=sys.stderr)
    return 0


def load_state() -> dict:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text())
        except json.JSONDecodeError:
            pass
    return {"world": None, "names": []}


def save_state(world: str, names: list[str]) -> None:
    STATE_FILE.write_text(json.dumps({"world": world, "names": names}, indent=2))


def cmd_list() -> int:
    directory = SCENES_DIR / "scenarios"
    if not directory.is_dir():
        print(f"No scenario directory at {directory}", file=sys.stderr)
        return 1
    for path in sorted(directory.glob("*.yaml")):
        data = yaml.safe_load(path.read_text()) or {}
        count = len(data.get("entities", []))
        print(f"  {path.stem:24s} {count:3d} entities  {data.get('description', '')}")
    return 0


def cmd_clear(world: str) -> int:
    state = load_state()
    names = state.get("names", [])
    if not names:
        print("Nothing recorded as spawned.")
        return 0
    target_world = world or state.get("world")
    with ThreadPoolExecutor(max_workers=GZ_SERVICE_WORKERS) as pool:
        removed = sum(pool.map(lambda n: remove(target_world, n), names))
    print(f"Removed {removed} of {len(names)} entities from '{target_world}'.")
    save_state(target_world, [])
    return 0


def cmd_spawn(world: str, path: Path) -> int:
    data = yaml.safe_load(path.read_text()) or {}
    entities = data.get("entities", [])
    if not entities:
        # A valid case, not an error: docs/development.md lists the empty
        # scenario as a test ("a detector that reports objects here is
        # wrong"), and scenegen writes one for a scene without targets.
        # The previous targets still clear and the resolved file resets.
        print(f"{path} lists no entities; clearing what was placed.")

    # Replace whatever the previous scenario left behind.
    previous = load_state()
    previous_world = previous.get("world") or world
    stale_names = previous.get("names", [])
    if stale_names:
        with ThreadPoolExecutor(max_workers=GZ_SERVICE_WORKERS) as pool:
            list(pool.map(lambda n: remove(previous_world, n), stale_names))

    valid = []
    for entity in entities:
        if "name" not in entity or "uri" not in entity:
            print(f"    skipping an entry without name and uri: {entity}", file=sys.stderr)
            continue
        valid.append(entity)

    placed = []
    with ThreadPoolExecutor(max_workers=GZ_SERVICE_WORKERS) as pool:
        futures = {pool.submit(spawn, world, e): e for e in valid}
        for future in as_completed(futures):
            entity = futures[future]
            # One bad entity must not abort the batch: the siblings still
            # spawn, and the state file must record them.
            try:
                ok = future.result()
            except Exception as exc:  # noqa: BLE001 - report and continue
                print(f"    [{entity['name']}] spawn failed: {exc}",
                      file=sys.stderr)
                continue
            if ok:
                placed.append(entity["name"])
                print(f"    placed {entity['name']:24s} {entity['uri']}")

    save_state(world, placed)
    actual = wait_for_poses(world, placed)
    write_resolved(world, [e for e in entities if e.get("name") in placed], actual)
    print(f"Scenario '{data.get('name', path.stem)}': {len(placed)} of {len(entities)} entities placed.")
    if len(placed) < len(entities):
        print("A Fuel model downloads on first use. Check the network and try again.",
              file=sys.stderr)
        return 1
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=os.environ.get("SCENE", "lorton"))
    ap.add_argument("--scenario", type=Path)
    ap.add_argument("--clear", action="store_true", help="remove the entities that are placed")
    ap.add_argument("--list", action="store_true", help="show the scenarios on disk")
    ap.add_argument("--fiducial", nargs=2, type=float, metavar=("EAST", "NORTH"),
                    help="stand the survey marker this far, in metres, from "
                         "the coordinate the vehicles were given. Zeros put "
                         "it back.")
    args = ap.parse_args()

    if args.list:
        return cmd_list()
    if args.fiducial is not None:
        return move_fiducial(args.world, *args.fiducial)
    if args.clear:
        return cmd_clear(args.world)
    if not args.scenario:
        ap.error("give --scenario, --clear, --list or --fiducial")
    if not args.scenario.is_file():
        print(f"No scenario file at {args.scenario}", file=sys.stderr)
        return 1
    return cmd_spawn(args.world, args.scenario)


if __name__ == "__main__":
    sys.exit(main())
