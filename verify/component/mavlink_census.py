#!/usr/bin/env python3
"""Count the MAVLink messages on a link, by message and by system.

Says what a link really carries, which is the question when telemetry reaches
one consumer and not another. It reads the frames with scripts/mavlink.py,
which is the one frame reader this stack has, so run it from this checkout.

    mavlink_census.py tcp://localhost:5761 --seconds 5
    mavlink_census.py udp://:14561 --seconds 5
"""

import argparse
import collections
import socket
import sys
import time
from pathlib import Path
from urllib.parse import urlparse

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from mavlink import frames  # noqa: E402

# The handful worth naming. Everything else is reported by number.
NAMES = {
    0: "HEARTBEAT", 24: "GPS_RAW_INT", 30: "ATTITUDE", 32: "LOCAL_POSITION_NED",
    33: "GLOBAL_POSITION_INT", 74: "VFR_HUD", 132: "DISTANCE_SENSOR",
    141: "ALTITUDE", 242: "HOME_POSITION", 245: "EXTENDED_SYS_STATE",
    1: "SYS_STATUS", 285: "GIMBAL_DEVICE_ATTITUDE_STATUS",
    281: "GIMBAL_MANAGER_STATUS", 280: "GIMBAL_MANAGER_INFORMATION",
}


def open_link(url: str):
    parsed = urlparse(url)
    if parsed.scheme == "tcp":
        link = socket.create_connection((parsed.hostname, parsed.port), timeout=5)
        return link, True
    link = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    link.bind((parsed.hostname or "0.0.0.0", parsed.port))
    return link, False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("url", help="tcp://host:port or udp://:port")
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()

    link, streaming = open_link(args.url)
    link.settimeout(0.5)
    seen = collections.Counter()
    pending = b""
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        try:
            data = link.recv(65535)
        except socket.timeout:
            continue
        if streaming and not data:
            break
        found, pending = frames(pending + data)
        for frame in found:
            seen[(frame.system, frame.message)] += 1

    if not seen:
        print("nothing arrived", file=sys.stderr)
        return 1
    for (system, message), count in sorted(seen.items(), key=lambda item: -item[1]):
        print(f"{count:6}  system {system:3}  {message:3} {NAMES.get(message, '')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
