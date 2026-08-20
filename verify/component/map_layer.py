#!/usr/bin/env python3
"""Check a layer the Map panel was handed.

Reads the GeoJSON a Foxglove bridge delivered, on standard input. Two shapes
of layer are drawn on that panel and each has to hold up on its own:

    --outlines  a ring around every known target. A filled or part transparent
                shape hides the imagery an operator is reading the target off,
                so either fails, as does a ring that is not the size it says.
    --verdicts  a pin for every estimate. Each one has to account for itself
                the same way, whatever its verdict: what it is, what it
                touched, and how far the nearest target was.

Prints one tab separated line per feature.
"""

import argparse
import json
import math
import sys

EARTH_RADIUS_M = 6378137.0


def radius_m(ring):
    """How far the ring stands from its own centre, in metres. A ring this
    small is flat enough that the meridian and the parallel are straight.

    A closed ring repeats its first corner, and counting it twice pulls the
    centre towards it and reports a ring wider than it is.
    """
    ring = ring[:-1] if ring[0] == ring[-1] else ring
    longitudes = [point[0] for point in ring]
    latitudes = [point[1] for point in ring]
    centre = (sum(longitudes) / len(ring), sum(latitudes) / len(ring))
    metres_per_degree = math.pi * EARTH_RADIUS_M / 180.0
    east_scale = metres_per_degree * math.cos(math.radians(centre[1]))
    return max(math.hypot((point[0] - centre[0]) * east_scale,
                          (point[1] - centre[1]) * metres_per_degree)
               for point in ring)


def outline_faults(feature, name, arguments):
    style = feature.get("properties", {}).get("style", {})
    ring = feature["geometry"]["coordinates"]
    drawn = radius_m(ring)
    print(f"{name}\t{style.get('color')}\t{drawn:.2f}")
    if abs(drawn - arguments.radius) > arguments.tolerance:
        yield f"{name} is {drawn:.2f} m across the middle, not {arguments.radius} m"
    if style.get("fill") is not False:
        yield f"{name} is a filled shape, not an outline"
    if style.get("opacity") != 1.0:
        yield f"{name} is drawn part transparent"
    if feature["geometry"]["type"] != "LineString":
        yield f"{name} is a {feature['geometry']['type']}, not a line"


def verdict_faults(feature, name, _arguments):
    """Every estimate accounts for itself the same way. A false positive names
    no target, and still has to say how far from one it fell."""
    metadata = feature.get("properties", {}).get("metadata", {})
    print(f"{name}\t{metadata.get('verdict')}\t"
          f"{metadata.get('nearest', '-')}\t{metadata.get('nearest_error_m', '-')}")
    for key in ("verdict", "nearest", "nearest_error_m"):
        if key not in metadata:
            yield f"{name} does not say its {key.replace('_', ' ')}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outlines", action="store_true",
                        help="a ring around every known target")
    parser.add_argument("--verdicts", action="store_true",
                        help="a pin for every estimate")
    parser.add_argument("--radius", type=float, default=2.0,
                        help="how far each ring stands from its target")
    parser.add_argument("--tolerance", type=float, default=0.1)
    parser.add_argument("--allow-empty", action="store_true",
                        help="a layer with nothing in it is not a fault. A "
                             "verdict kind that did not happen has none.")
    arguments = parser.parse_args()

    body = sys.stdin.read()
    # The probe labels what it shows; the collection is the rest.
    body = body[body.index("{"):] if "{" in body else body
    features = json.loads(body).get("features", [])
    if not features:
        if arguments.allow_empty:
            print("empty")
            return 0
        print("the layer carries no features", file=sys.stderr)
        return 1

    check = verdict_faults if arguments.verdicts else outline_faults
    faults = []
    for feature in features:
        name = feature.get("properties", {}).get("name", "?")
        faults += list(check(feature, name, arguments))
    for fault in faults:
        print(f"fault\t{fault}", file=sys.stderr)
    return 1 if faults else 0


if __name__ == "__main__":
    sys.exit(main())
