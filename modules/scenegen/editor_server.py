#!/usr/bin/env python3
"""Serve the browser editor for one scene.

A plain HTTP server, no framework:

  GET  /              editor.html, the whole editor in one file
  GET  /scene.json    the current scene description
  GET  /satellite.jpg the imagery the editor draws under everything
  GET  /config.json   what this serve run allows, e.g. the finish button
  GET  /tree_models.json  the tree pool with measured heights and canopy
                      diameters, so the editor draws true footprints
  POST /save          the edited scene.json. The previous version moves
                      to scene.json.bak first, so one bad save loses
                      nothing.
  POST /finish        only in the `all` pipeline: stop serving and let
                      the build stage run. The editor's "Save & build"
                      button sends this after a save.

Stop it with Ctrl-C. Rebuild the world after editing:
  scenegen.py build --name <name>
"""

from __future__ import annotations

import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import scene_model

MODULE_DIR = Path(__file__).resolve().parent


def serve(scene_data_dir: Path, port: int, finish_enabled: bool = False) -> str:
    """Serve until the user is done. Returns how it ended:

      "missing"      no scene.json to edit
      "finished"     the browser sent /finish (Save & build)
      "interrupted"  Ctrl-C
    """
    scene_path = scene_data_dir / "scene.json"
    if not scene_path.is_file():
        print(f"No scene at {scene_path}. Run create first.", file=sys.stderr)
        return "missing"
    imagery_file = json.loads(scene_path.read_text())["imagery"]["file"]
    finished = threading.Event()

    class EditorHandler(BaseHTTPRequestHandler):
        def _send(self, payload: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self) -> None:  # noqa: N802 - http.server API
            if self.path in ("/", "/editor.html"):
                self._send((MODULE_DIR / "editor.html").read_bytes(),
                           "text/html; charset=utf-8")
            elif self.path == "/scene.json":
                self._send(scene_path.read_bytes(), "application/json")
            elif self.path == "/satellite.jpg":
                self._send((scene_data_dir / imagery_file).read_bytes(), "image/jpeg")
            elif self.path == "/config.json":
                self._send(json.dumps({"finish_enabled": finish_enabled}).encode(),
                           "application/json")
            elif self.path == "/tree_models.json":
                self._send(json.dumps(scene_model.TREE_MODEL_POOL).encode(),
                           "application/json")
            else:
                self.send_error(404)

        def do_POST(self) -> None:  # noqa: N802 - http.server API
            if self.path == "/finish" and finish_enabled:
                finished.set()
                self._send(b'{"ok": true}', "application/json")
                # shutdown() blocks until serve_forever returns, so it runs
                # on its own thread rather than inside this handler.
                threading.Thread(target=server.shutdown, daemon=True).start()
                return
            if self.path != "/save":
                self.send_error(404)
                return
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)
            try:
                scene_model.SceneSpec.from_json(body.decode("utf-8"))
            except (ValueError, TypeError, KeyError) as error:
                self.send_error(400, f"scene.json rejected: {error}")
                return
            if scene_path.exists():
                scene_path.replace(scene_path.with_suffix(".json.bak"))
            scene_path.write_bytes(body)
            self._send(b'{"ok": true}', "application/json")

        def log_message(self, format: str, *args) -> None:  # noqa: A002
            pass  # one line per tile drag-save would drown the terminal

    server = ThreadingHTTPServer(("0.0.0.0", port), EditorHandler)
    print(f"Editor for '{scene_path.parent.name}' on http://localhost:{port}/")
    if finish_enabled:
        print("Edits save straight into scene.json. Click 'Save & build' in the "
              "browser to continue, or Ctrl-C to stop without building.")
    else:
        print("Edits save straight into scene.json. Ctrl-C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
        return "interrupted"
    finally:
        server.server_close()
    if finished.is_set():
        print("Finish received from the browser.")
        return "finished"
    return "interrupted"
