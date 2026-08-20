#!/usr/bin/env python3
"""How far a simulated vehicle leans from upright, in degrees.

Reads a Gazebo `pose/info` stream on standard input and answers for one model.
A vehicle that ended a test on its side looks healthy to every ROS topic:
MAVROS still publishes, the graph is complete, and the next takeoff simply
never climbs. The lean is the one measurement that says so.

`gz model -p` is not used on purpose. It answered with a pose that never
changed across four commands while the live stream showed the model moving,
so it is not to be trusted for this.
"""

import argparse
import math
import re
import sys

# One "name: ... pose { position {...} orientation {...} }" block, flattened.
NUMBER = r"[-+0-9.eE]+"


def lean_degrees(text: str, model: str):
    """Degrees between the model's own up axis and the world's."""
    start = text.find(f'name: "{model}"')
    if start < 0:
        return None
    window = text[start:start + 2000]
    found = re.search(
        r"orientation\s*{\s*x:\s*(" + NUMBER + r")\s*y:\s*(" + NUMBER +
        r")\s*z:\s*(" + NUMBER + r")\s*w:\s*(" + NUMBER + r")", window)
    if not found:
        return None
    x, y, z, w = (float(v) for v in found.groups())
    # The z row of the rotation matrix is where the body's up axis points.
    up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", help="the Gazebo entity, such as uas11_10")
    arguments = parser.parse_args()
    lean = lean_degrees(sys.stdin.read(), arguments.model)
    if lean is None:
        print(f"{arguments.model} said nothing", file=sys.stderr)
        return 1
    print(f"{lean:.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
