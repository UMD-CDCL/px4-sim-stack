#!/usr/bin/env python3
"""Rebuild dtc_g1_sim while preserving its surveyed narrow structures.

Run from the repository root:
    python3 modules/scenegen/data/dtc_g1_sim/build.py

The generic builder discards footprints below 1 m². This scene contains
intentional 0.1 m wide walls; use its roof sliver tolerance instead, only
for this process. No other scene or shared build setting is changed.
"""

from pathlib import Path
import sys


def main():
    directory = Path(__file__).resolve().parent
    module = directory.parent.parent
    sys.path.insert(0, str(module))
    import build_world
    import scene_model

    scene = scene_model.load_scene(directory / "scene.json")
    if scene.name != "dtc_g1_sim":
        raise ValueError("This build profile is only for dtc_g1_sim")
    original_limit = build_world.MIN_CLIPPED_AREA_M2
    try:
        build_world.MIN_CLIPPED_AREA_M2 = build_world.MIN_ROOF_PIECE_M2
        return build_world.run(directory, module.parent / "sim" / "scenes")
    finally:
        build_world.MIN_CLIPPED_AREA_M2 = original_limit


if __name__ == "__main__":
    raise SystemExit(main())
