#!/usr/bin/env python3
"""Print the live video paths.

Two sources. With no flag, the MediaMTX API of the simulated video router.
With `--rtsp BASE NAME...`, a plain RTSP server that has no API (lcam on the
ground station, rcam on the aircraft): gst-discoverer-1.0 probes each name.
`--json` prints the probed rows instead of the table, which is how
scripts/state.py reads the same answer.

`px4sim streams` calls this. It is a file rather than a line inside the shell
script because the quoting of nested JSON in a heredoc is not worth the trouble.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
import urllib.error
import urllib.request

API = next((word for word in sys.argv[1:] if not word.startswith("-")),
             "http://localhost:9997")
PROBE_SECONDS = 5
# gst-discoverer text when no server or no mount accepts the connection.
REFUSED_TEXT = "Could not open resource for reading"


def main() -> int:
    try:
        with urllib.request.urlopen(f"{API}/v3/paths/list", timeout=5) as r:
            data = json.load(r)
    except urllib.error.HTTPError as e:
        print(f"  the video router API answered {e.code}.")
        if e.code == 401:
            print("  It needs the api permission. See authInternalUsers in")
            print("  modules/video-router/mediamtx.yml.")
        return 1
    except (urllib.error.URLError, OSError) as e:
        print(f"  cannot reach {API}: {e}")
        print("  Is the video router running?  ./px4sim status")
        return 1

    items = data.get("items", [])
    if not items:
        print("  No paths configured.")
        return 0

    print(f"  {'PATH':<20} {'STATE':<9} {'READERS':<8} SOURCE")
    live = 0
    for p in items:
        ready = bool(p.get("ready"))
        live += ready
        source = (p.get("source") or {}).get("type", "-")
        print(f"  {p.get('name', '?'):<20} {'online' if ready else 'offline':<9} "
              f"{len(p.get('readers', [])):<8} {source}")

    if live == 0:
        print()
        print("  Nothing is publishing. Every vehicle stream comes from the")
        print("  simulator, so check it with:  ./px4sim logs sim")
    return 0


def describe_video(report: str) -> str:
    """Codec, size and rate of the first video stream in a gst-discoverer report, or ''.

    gst-discoverer-1.0 exits 0 on a refused or silent path, so only its text says
    whether frames flow. The stream line reads `video #1: H.265 (Main Profile)`.
    """
    def field(name: str) -> str:
        m = re.search(rf"^\s*{name}: (\S+)", report, re.M)
        return m.group(1) if m else ""

    codec = re.search(r"^\s*video(?: #\d+)?: (.+)$", report, re.M)
    if not codec:
        return ""
    size = "x".join(v for v in (field("Width"), field("Height")) if v)
    rate = field("Frame rate")
    rate = rate[:-2] if rate.endswith("/1") else rate
    return " ".join(part for part in (codec.group(1), size, rate and f"{rate} fps") if part)


def probe_rtsp(base: str, names: list[str]) -> list[dict]:
    """One row for each mount. `source` is empty where no frames arrived."""
    rows = []
    for name in names:
        try:
            result = subprocess.run(["gst-discoverer-1.0", "-t", str(PROBE_SECONDS), f"{base}/{name}"],
                                    capture_output=True, text=True, timeout=PROBE_SECONDS * 3)
            video = describe_video(result.stdout)
            refused = REFUSED_TEXT in result.stdout
        except subprocess.TimeoutExpired:
            video, refused = "", False
        rows.append({"name": name, "ready": bool(video),
                     "source": video, "refused": refused})
    return rows


def print_rtsp(base: str, rows: list[dict]) -> int:
    print(f"  {'PATH':<20} {'STATE':<9} SOURCE")
    for row in rows:
        print(f"  {row['name']:<20} {'online' if row['ready'] else 'offline':<9} "
              f"{row['source'] or '-'}")
    if any(row["ready"] for row in rows):
        return 0
    print()
    if rows and all(row["refused"] for row in rows):
        print(f"  Nothing answers at {base}. The ground station serves it with lcam.service,")
        print("  the aircraft with rcam.service:  systemctl status lcam rcam")
    else:
        print(f"  {base} answers, but no stream sends frames.")
        print("  Check the camera on the vehicle.")
    return 0


if __name__ == "__main__":
    words = [word for word in sys.argv[1:] if word != "--json"]
    if words[:1] == ["--rtsp"]:
        if len(words) < 2:
            sys.exit(f"usage: {sys.argv[0]} [--json] --rtsp rtsp://HOST:PORT NAME...")
        try:
            probed = probe_rtsp(words[1], words[2:])
        except FileNotFoundError:
            sys.exit("gst-discoverer-1.0 is not installed."
                     " It comes with gstreamer1.0-plugins-base-apps.")
        if "--json" in sys.argv[1:]:
            json.dump(probed, sys.stdout)
            sys.exit(0)
        sys.exit(print_rtsp(words[1], probed))
    sys.exit(main())
