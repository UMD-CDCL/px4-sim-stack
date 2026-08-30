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
import json
import math
import re
import sys

import yaml


def roofs(path):
    """The buildings, as (footprint, roof height) pairs. None reads as no scene."""
    if not path:
        return []
    with open(path) as handle:
        scene = json.load(handle)
    return [(b["footprint"], max((l.get("z", 0.0) for l in b["levels"]), default=0.0))
            for b in scene.get("buildings", []) if b.get("footprint")]


def within(point, polygon):
    """Ray casting, because a footprint is a plain polygon in the same frame."""
    x, y = point
    inside = False
    for i, (x1, y1) in enumerate(polygon):
        x2, y2 = polygon[(i + 1) % len(polygon)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def sees(eye, target, buildings, step=0.5):
    """Whether the eye can see a target on the ground, past the roofs between.

    Marched rather than solved: a footprint is an arbitrary polygon and the only
    question is whether the sight line is under a roof while it is over that
    roof's outline. Half a metre is finer than the smallest building here.
    """
    span = math.dist(eye[:2], target)
    for i in range(1, max(2, int(span / step))):
        along = i / max(2, int(span / step))
        height = eye[2] * (1 - along)
        place = (eye[0] + (target[0] - eye[0]) * along,
                 eye[1] + (target[1] - eye[1]) * along)
        for footprint, roof in buildings:
            if height < roof and within(place, footprint):
                return False
    return True


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
            best)


def standoff(aim_point, group, reach, height, buildings):
    """Where to stand to see the group, and the heading to face it.

    Due south of the targets is the readable answer and the one this always gave:
    the vehicle sits at (east, north - reach) on heading 0. It is only right when
    nothing is in the way, and a scene with buildings in it does not promise
    that. uroc puts a 5.8 m roof 10 m from its eastern casualties, which hides
    all three of them from anywhere between bearing 45 and 165, and the same roof
    hides two of them from due south as soon as the vehicle stands 40 m off
    instead of 20 -- occlusion follows the range as much as the bearing, because
    what matters is whether the sight line is still above the roof when it
    crosses it.

    So south is tried first and kept when it works, and otherwise the bearing
    turns away from it in even steps until every target in the group is visible.
    Answers the position, the heading, and how far it had to turn.
    """
    for turn in range(0, 181, 5):
        for bearing in ({180 - turn, 180 + turn} if turn else {180}):
            radians = math.radians(bearing)
            eye = (aim_point[0] + reach * math.sin(radians),
                   aim_point[1] + reach * math.cos(radians),
                   height)
            if all(sees(eye, target, buildings) for target in group):
                return eye[0], eye[1], (bearing + 180) % 360, turn
    return (aim_point[0], aim_point[1] - reach, 0.0, None)


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
    parser.add_argument("--buildings", default="",
                        help="the scene's <scene>_buildings.json, so the "
                             "bearing chosen is one the targets are visible "
                             "from. Without it every bearing looks clear")
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

    east, north, group = aim(places, arguments.spread)
    if len(group) < len(places):
        print(f"aiming at {len(group)} of {len(places)} targets, the biggest "
              f"group within {arguments.spread:.0f} m", file=sys.stderr)
    if arguments.centre:
        print(f"{east:.2f} {north:.2f}")
        return 0

    # Ground range at that depression, so the boresight lands on the middle of
    # them, and then a bearing from which they can actually be seen.
    reach = arguments.height / math.tan(math.radians(arguments.depression))
    buildings = roofs(arguments.buildings)
    stand_east, stand_north, heading, turn = standoff(
        (east, north), group, reach, arguments.height, buildings)
    if turn is None:
        print("no bearing at this range and height sees the whole group past "
              "the buildings; standing due south anyway", file=sys.stderr)
    elif turn:
        print(f"standing {turn} degrees off due south: a building hides the "
              f"group from there", file=sys.stderr)
    print(f"{stand_east:.1f} {stand_north:.1f} {arguments.height:.1f} "
          f"{heading:.0f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
