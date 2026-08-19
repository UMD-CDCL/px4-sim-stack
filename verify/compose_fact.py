#!/usr/bin/env python3
"""One resolved fact from `docker compose config --format json`.

The stage compares these against scripts/fleet.sh, so a number that drifted
between the front door and the compose file shows up as a mismatch instead of
as a container that starts and talks to nobody.
"""

import json
import sys


def main(argv):
    path, query, service = argv[1], argv[2], argv[3]
    config = json.load(open(path))
    services = config["services"]
    if service not in services:
        return f"no service {service}"
    entry = services[service]

    if query == "address":
        networks = entry.get("networks") or {}
        return next((n.get("ipv4_address", "") for n in networks.values() if n), "")
    if query == "netmode":
        return entry.get("network_mode", "")
    if query == "port":
        target = int(argv[4])
        return next((p["published"] for p in entry.get("ports") or []
                     if p["target"] == target), "")
    if query == "publishes":
        published = argv[4]
        return "yes" if any(str(p["published"]) == published
                            for p in entry.get("ports") or []) else "no"
    if query == "env":
        return (entry.get("environment") or {}).get(argv[4], "")
    return f"unknown query {query}"


if __name__ == "__main__":
    print(main(sys.argv))
