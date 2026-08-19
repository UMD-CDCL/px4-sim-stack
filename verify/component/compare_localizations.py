#!/usr/bin/env python3
"""Compare the localizations two graphs published, joined on sequence number.

The vehicle and the ground station watch one live stream, not one message, so
sampling each in turn compares different frames. Both sides are collected at
once and only the sequence numbers they share are compared: those must match
exactly, because the ground shows what the vehicle worked out rather than its
own approximation of it.
"""

import pathlib
import sys


def load(path):
    rows = {}
    for line in pathlib.Path(path).read_text().splitlines():
        fields = line.split("\t")
        if len(fields) == 6:
            rows[(fields[0], fields[1])] = tuple(fields[2:])
    return rows


def main(argv) -> int:
    vehicle, ground = load(argv[1]), load(argv[2])
    shared = sorted(set(vehicle) & set(ground))
    if not shared:
        print("no sequence number reached both sides")
        return 1
    differing = [key for key in shared if vehicle[key] != ground[key]]
    print(f"{len(shared)} boxes on both sides, {len(shared) - len(differing)} identical")
    for key in differing[:3]:
        print(f"  seq {key[0]} box {key[1]}: vehicle {vehicle[key]} ground {ground[key]}")
    return 1 if differing else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
