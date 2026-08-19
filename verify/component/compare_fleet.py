#!/usr/bin/env python3
"""How far apart the fleet puts the same target.

Each file is one vehicle's localizations, tab separated, named by the recorded
target each landed nearest. A target more than one vehicle saw is one the fleet
can be asked about twice, and the answers should be one answer: every vehicle
casts its rays at the same ground, anchored the same way.

    compare_fleet.py <tolerance-m> <file> <file> [file ...]
"""

import itertools
import math
import pathlib
import sys

import pymap3d as pm


def load(path):
    found = {}
    for line in pathlib.Path(path).read_text().splitlines():
        fields = line.split("\t")
        if len(fields) >= 3 and fields[0]:
            found[fields[0]] = (float(fields[1]), float(fields[2]))
    return found


def main(argv) -> int:
    tolerance = float(argv[1])
    seen = {pathlib.Path(path).name: load(path) for path in argv[2:]}
    seen = {name: found for name, found in seen.items() if found}
    if len(seen) < 2:
        print(f"only {len(seen)} vehicles localized anything")
        return 1

    apart = {}
    for (one, first), (other, second) in itertools.combinations(seen.items(), 2):
        for target in set(first) & set(second):
            east, north, _ = pm.geodetic2enu(*first[target], 0.0,
                                             *second[target], 0.0, deg=True)
            apart[(target, one, other)] = math.hypot(east, north)
    if not apart:
        print(f"{len(seen)} vehicles localized targets, but no target twice")
        return 1

    worst, distance = max(apart.items(), key=lambda item: item[1])
    targets = {key[0] for key in apart}
    print(f"{len(seen)} vehicles agree on {len(targets)} targets, worst "
          f"{distance:.2f} m on {worst[0]} between uas{worst[1]} and uas{worst[2]}")
    return 0 if distance <= tolerance else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
