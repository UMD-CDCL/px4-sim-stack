#!/usr/bin/env python3
"""Print the live video paths, read from the MediaMTX API.

`px4sim streams` calls this. It is a file rather than a line inside the shell
script because the quoting of nested JSON in a heredoc is not worth the trouble.
"""

from __future__ import annotations

import json
import sys
import urllib.error
import urllib.request

API = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:9997"


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


if __name__ == "__main__":
    sys.exit(main())
