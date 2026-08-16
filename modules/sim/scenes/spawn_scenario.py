#!/usr/bin/env python3
"""Place scenario entities into a running Gazebo world.

A scene is the terrain. A scenario is what stands on it. They are separate so
that one scene can carry many test cases, and so that a target layout can
change without a simulator restart.

    spawn_scenario.py --world recon_field --scenario scenarios/urban_casualties.yaml
    spawn_scenario.py --world recon_field --clear
    spawn_scenario.py --list

The script talks to Gazebo through the `gz service` command line tool, so it
needs no Python bindings.
"""

from __future__ import annotations

import argparse
import json
import os
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
        # A valid case, not an error: docs/scenarios.md lists the empty
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
    ap.add_argument("--world", default=os.environ.get("SCENE", "recon_field"))
    ap.add_argument("--scenario", type=Path)
    ap.add_argument("--clear", action="store_true", help="remove the entities that are placed")
    ap.add_argument("--list", action="store_true", help="show the scenarios on disk")
    args = ap.parse_args()

    if args.list:
        return cmd_list()
    if args.clear:
        return cmd_clear(args.world)
    if not args.scenario:
        ap.error("give --scenario, --clear or --list")
    if not args.scenario.is_file():
        print(f"No scenario file at {args.scenario}", file=sys.stderr)
        return 1
    return cmd_spawn(args.world, args.scenario)


if __name__ == "__main__":
    sys.exit(main())
