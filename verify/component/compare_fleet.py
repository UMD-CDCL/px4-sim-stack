#!/usr/bin/env python3
"""How far apart two vehicles put the same target.

Each file is one vehicle's localizations, tab separated, named by the recorded
target each landed nearest. A target both vehicles saw is one the fleet can be
asked about twice, and the two answers should be one answer.
"""

import math
import sys

import pymap3d as pm


def load(path):
    found = {}
    for line in open(path, encoding="utf-8"):
        fields = line.rstrip("\n").split("\t")
        if len(fields) >= 3 and fields[0]:
            found[fields[0]] = (float(fields[1]), float(fields[2]))
    return found


def main(argv) -> int:
    first, second = load(argv[1]), load(argv[2])
    tolerance = float(argv[3])
    shared = sorted(set(first) & set(second))
    if not shared:
        print("the two vehicles saw no target in common")
        return 1

    apart = {}
    for name in shared:
        east, north, _ = pm.geodetic2enu(*first[name], 0.0, *second[name], 0.0, deg=True)
        apart[name] = math.hypot(east, north)
    worst_name = max(apart, key=apart.get)
    worst = apart[worst_name]
    print(f"two vehicles agree on {len(shared)} targets, worst {worst:.2f} m "
          f"apart on {worst_name}")
    return 0 if worst <= tolerance else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
