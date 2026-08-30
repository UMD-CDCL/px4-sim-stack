#!/usr/bin/env python3
"""Where to fly to get the scene's targets in the camera, for any scene.

A check that flies to a place written into it only works in the scene it was
written for. The campus scenario puts its casualties about 100 m north of the
origin and the Lorton one puts them within 10 m of it, so a hardcoded leg is
either a long flight to empty ground or a view of nothing.

This reads the scenario the stack is flying and prints one line:

    east north up heading

Standing there at that heading, with the gimbal at the depression asked for,
puts the middle of the targets on the boresight. The vehicle is placed short of
the targets rather than over them, because a person is found at an oblique
angle rather than from straight above.
"""

import argparse
import math
import re
import sys

import yaml


def targets(scenario: dict, first: int):
    """The east and north of the targets worth aiming at.

    The LOWEST NUMBERED ones, and not all of them. A scenario places casualties
    across the whole scene and many of them stand under a vehicle or behind
    something, where no camera sees them from any one place. Averaging those in
    drags the aim off the group that IS in the open: at Lorton the centre of all
    32 is 18 m from the centre of the first 12, which at a 10 m footprint means
    the clear group is out of frame entirely.
    """
    numbered = []
    for entity in scenario.get("entities", []):
        pose = entity.get("pose")
        if not pose or len(pose) < 2:
            continue
        found = re.search(r"(\d+)$", str(entity.get("name", "")))
        numbered.append((int(found.group(1)) if found else 10 ** 6,
                         float(pose[0]), float(pose[1])))
    numbered.sort()
    for _, east, north in numbered[:first] or numbered:
        yield east, north


def aim(places, spread):
    """The point to put on the boresight: the middle of the BIGGEST GROUP.

    Not the middle of all of them. A mean is only a place to look when the
    targets surround it, and a scenario is free to put them in two groups with
    nothing between. uroc does exactly that -- two casualties about 6 m west of
    the origin and three about 14 m east -- and their mean falls in the empty
    car park between the two, 9.5 m from the nearest of them. At the mid
    framing that is a 10 m footprint aimed at bare tarmac, so the check that
    asks whether casualties localize saw no boxes at all and read as a broken
    detector.

    `spread` is how far apart two targets can be and still share one view. Each
    target proposes the group within that distance of it, the fullest group
    wins, and the tightest wins a tie so the aim does not drift to the edge of
    an equally large but looser one.
    """
    def group_of(centre):
        return [p for p in places
                if math.dist(p, centre) <= spread]

    def score(group):
        east = sum(p[0] for p in group) / len(group)
        north = sum(p[1] for p in group) / len(group)
        return len(group), -max(math.dist(p, (east, north)) for p in group)

    best = max((group_of(centre) for centre in places), key=score)
    return (sum(p[0] for p in best) / len(best),
            sum(p[1] for p in best) / len(best),
            len(best))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scenario")
    parser.add_argument("--height", type=float, default=20.0,
                        help="metres over home. Lower is a better look at a "
                             "person, and closer to what a vehicle really flies")
    parser.add_argument("--depression", type=float, default=45.0,
                        help="the gimbal angle the view is planned for")
    parser.add_argument("--first", type=int, default=12,
                        help="how many of the lowest numbered targets to aim "
                             "at. The rest are the obstructed ones")
    parser.add_argument("--spread", type=float, default=5.0,
                        help="how far apart two targets can be and still share "
                             "one view, in metres. The default is the half "
                             "footprint of the mid framing at the default "
                             "height and depression, which is the narrowest "
                             "view the checks use")
    parser.add_argument("--centre", action="store_true",
                        help="print the middle of the targets instead, as "
                             "east north, for a caller that plans its own view")
    arguments = parser.parse_args()

    with open(arguments.scenario) as handle:
        scenario = yaml.safe_load(handle) or {}
    places = list(targets(scenario, arguments.first))
    if not places:
        print(f"{arguments.scenario} names no entity with a pose", file=sys.stderr)
        return 1

    east, north, held = aim(places, arguments.spread)
    if held < len(places):
        print(f"aiming at {held} of {len(places)} targets, the biggest group "
              f"within {arguments.spread:.0f} m", file=sys.stderr)
    # Ground range at that depression, so the boresight lands on the middle of
    # them. Heading 0 keeps the offset due south and the arithmetic readable.
    if arguments.centre:
        print(f"{east:.2f} {north:.2f}")
        return 0
    reach = arguments.height / math.tan(math.radians(arguments.depression))
    print(f"{east:.1f} {north - reach:.1f} {arguments.height:.1f} 0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
