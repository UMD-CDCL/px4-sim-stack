#!/usr/bin/env python3
"""Tests for the editor server contract, including the finish flow that
lets `all` continue to the build. No browser involved.

Run: python3 tests/test_editor_server.py
"""

from __future__ import annotations

import json
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import tempfile

from PIL import Image

import editor_server
import scene_model

PORT_FINISH = 8123
PORT_PLAIN = 8124

CHECKS = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, condition, detail))
    mark = "ok  " if condition else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))


def get(port: int, path: str) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(f"http://localhost:{port}{path}", timeout=5) as reply:
            return reply.status, reply.read()
    except urllib.error.HTTPError as error:
        return error.code, b""


def post(port: int, path: str, payload: bytes) -> tuple[int, bytes]:
    request = urllib.request.Request(f"http://localhost:{port}{path}", payload,
                                     {"Content-Type": "application/json"},
                                     method="POST")
    try:
        with urllib.request.urlopen(request, timeout=5) as reply:
            return reply.status, reply.read()
    except urllib.error.HTTPError as error:
        return error.code, b""


def make_scene_dir(tmp: Path) -> Path:
    directory = tmp / "editortest"
    directory.mkdir()
    Image.new("RGB", (16, 16), (90, 120, 80)).save(directory / "satellite.jpg")
    scene = scene_model.SceneSpec(
        name="editortest", center_lat=38.9869, center_lon=-76.9426, side_m=100,
        origin_alt_m=10.0,
        imagery={"source": "synthetic", "zoom": 19, "file": "satellite.jpg",
                 "width_px": 16, "height_px": 16, "m_per_px": 6.25,
                 "origin_px": 0.0, "origin_py": 0.0})
    scene_model.save_scene(scene, directory / "scene.json")
    return directory


def wait_up(port: int) -> bool:
    for _ in range(50):
        try:
            get(port, "/config.json")
            return True
        except OSError:
            time.sleep(0.1)
    return False


def test_finish_flow(directory: Path) -> None:
    print("finish-enabled server (the `all` pipeline)")
    outcome: list[str] = []
    thread = threading.Thread(
        target=lambda: outcome.append(
            editor_server.serve(directory, PORT_FINISH, finish_enabled=True)))
    thread.start()
    check("server comes up", wait_up(PORT_FINISH))

    status, body = get(PORT_FINISH, "/config.json")
    check("config.json advertises the finish button",
          status == 200 and json.loads(body)["finish_enabled"] is True)

    scene = json.loads(get(PORT_FINISH, "/scene.json")[1])
    scene["fiducial"]["east_m"] = 7.0
    status, _ = post(PORT_FINISH, "/save", json.dumps(scene).encode())
    saved = json.loads((directory / "scene.json").read_text())
    check("save still works before finish",
          status == 200 and saved["fiducial"]["east_m"] == 7.0)

    status, body = post(PORT_FINISH, "/finish", b"")
    check("finish is accepted", status == 200 and b"ok" in body)
    thread.join(timeout=5)
    check("server stops after finish", not thread.is_alive())
    check("serve reports 'finished'", outcome == ["finished"], str(outcome))


def test_plain_flow(directory: Path) -> None:
    print("plain server (the edit command)")
    thread = threading.Thread(
        target=lambda: editor_server.serve(directory, PORT_PLAIN), daemon=True)
    thread.start()
    check("server comes up", wait_up(PORT_PLAIN))

    status, body = get(PORT_PLAIN, "/config.json")
    check("config.json hides the finish button",
          status == 200 and json.loads(body)["finish_enabled"] is False)

    status, _ = post(PORT_PLAIN, "/finish", b"")
    check("finish is refused", status == 404)
    # The daemon thread dies with the test process; plain serve only stops
    # on Ctrl-C, which a test cannot send.


def main() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        directory = make_scene_dir(Path(tmp))
        test_finish_flow(directory)
        test_plain_flow(directory)
    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
