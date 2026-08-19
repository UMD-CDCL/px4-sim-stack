#!/usr/bin/env python3
"""Ground-truth tests for footprint triangulation and extrusion.

Run: python3 tests/test_building_mesh.py

Every polygon here has a hand-computed area, so coverage checks catch a
triangulation that leaks outside the footprint or misses a piece of it.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

import building_mesh
import scene_model

CHECKS = []


def check(name: str, condition: bool, detail: str = "") -> None:
    CHECKS.append((name, condition, detail))
    mark = "ok  " if condition else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))


SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
# A 20 x 20 square minus its 12 x 12 northeast notch: area 256.
L_SHAPE = [(0.0, 0.0), (20.0, 0.0), (20.0, 8.0), (8.0, 8.0),
           (8.0, 20.0), (0.0, 20.0)]
# Clockwise hole, the winding placed_footprint hands over.
HOLE = [(4.0, 4.0), (4.0, 6.0), (6.0, 6.0), (6.0, 4.0)]


def test_triangulate() -> None:
    print("triangulation")
    triangles = building_mesh.triangulate(SQUARE, [])
    check("a square becomes two triangles of full area",
          len(triangles) == 2
          and abs(building_mesh.triangle_area_m2(triangles) - 100.0) < 1e-6)

    triangles = building_mesh.triangulate(L_SHAPE, [])
    check("the L triangulates to its own area, not the bounding box",
          abs(building_mesh.triangle_area_m2(triangles) - 256.0) < 1e-6,
          f"{building_mesh.triangle_area_m2(triangles):.2f}")

    triangles = building_mesh.triangulate(SQUARE, [HOLE])
    area = building_mesh.triangle_area_m2(triangles)
    check("a courtyard hole leaves the ring area", abs(area - 96.0) < 1e-6,
          f"{area:.2f}")
    centers = [((a[0] + b[0] + c[0]) / 3.0, (a[1] + b[1] + c[1]) / 3.0)
               for a, b, c in triangles]
    check("no triangle sits in the hole",
          not any(4.0 < x < 6.0 and 4.0 < y < 6.0 for x, y in centers))

    collinear = [(0.0, 0.0), (5.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    triangles = building_mesh.triangulate(collinear, [])
    check("a collinear edge point does not break coverage",
          abs(building_mesh.triangle_area_m2(triangles) - 100.0) < 1e-6)


def test_extrude() -> None:
    print("extrusion")
    roof, walls = building_mesh.extrude(SQUARE, [HOLE], 1.0, 7.0)
    check("roof rides at the top", float(roof[:, :, 2].min()) == 7.0)
    check("two wall triangles per ring edge", walls.shape[0] == 16,
          str(walls.shape))
    check("walls span base to top",
          float(walls[:, :, 2].min()) == 1.0 and float(walls[:, :, 2].max()) == 7.0)

    roof_normals = building_mesh.face_normals(roof)
    check("roof normals point up", bool((roof_normals[:, 2] > 0.99).all()))
    wall_normals = building_mesh.face_normals(walls)
    check("wall normals are horizontal",
          bool((np.abs(wall_normals[:, 2]) < 1e-9).all()))
    check("normals pair one to one with vertices",
          wall_normals.shape[0] == walls.shape[0] * 3)

    south_wall = walls[0]
    normal = building_mesh.face_normals(south_wall.reshape(1, 3, 3))[0]
    check("the south wall faces south", abs(normal[1] + 1.0) < 1e-9, str(normal))


def test_slab() -> None:
    print("floor slab")
    top = building_mesh.roof_at(SQUARE, [HOLE], 9.0)
    shell = building_mesh.slab_shell(SQUARE, [HOLE], 9.0, 0.3)
    check("the slab hangs at its level and is that thick",
          float(shell[:, :, 2].max()) == 9.0
          and abs(float(shell[:, :, 2].min()) - 8.7) < 1e-9,
          f"z {shell[:, :, 2].min()}..{shell[:, :, 2].max()}")
    underside = shell[shell[:, :, 2].max(axis=1) == 8.7]
    check("the underside covers the footprint, hole cut out",
          abs(building_mesh.triangle_area_m2(underside[:, :, :2])
              - (100.0 - 4.0)) < 1e-6,
          f"{building_mesh.triangle_area_m2(underside[:, :, :2]):.2f}")
    normals = building_mesh.face_normals(underside)
    check("the underside faces down", bool((normals[:, 2] < -0.99).all()))
    check("the top faces up",
          bool((building_mesh.face_normals(top)[:, 2] > 0.99).all()))


def test_subdivide() -> None:
    print("subdivision")
    roof, _ = building_mesh.extrude(L_SHAPE, [], 0.0, 5.0)
    fine = building_mesh.subdivide(roof, 4.0)
    check("subdivision keeps the area",
          abs(building_mesh.triangle_area_m2(fine[:, :, :2]) - 256.0) < 1e-6)
    edges = np.linalg.norm(np.roll(fine, -1, axis=1) - fine, axis=2)
    check("no edge is longer than the limit", float(edges.max()) <= 4.0 + 1e-9,
          f"max {edges.max():.2f}")


def test_placed_footprint() -> None:
    print("placed footprint follows the rectangle fields")
    outline = [[e, n] for e, n in L_SHAPE] + [[L_SHAPE[0][0], L_SHAPE[0][1]]]
    east, north, length, width, yaw = scene_model.oriented_rectangle(L_SHAPE)

    def building(**overrides) -> scene_model.Building:
        fields = dict(id="b", east_m=round(east, 2), north_m=round(north, 2),
                      length_m=round(length, 2), width_m=round(width, 2),
                      yaw_deg=round(yaw, 2), height_m=5.0, outline_m=outline,
                      holes_m=[[[p[0], p[1]] for p in HOLE]])
        fields.update(overrides)
        return scene_model.Building(**fields)

    outer, holes = scene_model.placed_footprint(building())
    check("an unedited building keeps its outline",
          all(math.dist(a, b) < 0.02 for a, b in zip(outer, L_SHAPE))
          and len(holes) == 1, str(outer[:3]))

    moved_outer, _ = scene_model.placed_footprint(
        building(east_m=round(east, 2) + 5.0, north_m=round(north, 2) - 3.0))
    check("a nudged building carries its outline along",
          all(math.dist(a, (b[0] + 5.0, b[1] - 3.0)) < 0.02
              for a, b in zip(moved_outer, L_SHAPE)))

    turned_outer, _ = scene_model.placed_footprint(
        building(yaw_deg=round(yaw, 2) + 90.0))
    area = building_mesh.triangle_area_m2(
        building_mesh.triangulate(turned_outer, []))
    turned_corner = min(turned_outer, key=lambda p: (p[0], p[1]))
    check("a rotated building turns in place with its area intact",
          abs(area - 256.0) < 0.5, f"{area:.2f}, corner {turned_corner}")

    grown_outer, _ = scene_model.placed_footprint(
        building(length_m=round(length, 2) * 2.0))
    grown_area = building_mesh.triangle_area_m2(
        building_mesh.triangulate(grown_outer, []))
    check("a resized building scales its outline",
          abs(grown_area - 512.0) < 1.0, f"{grown_area:.2f}")

    fallback_outer, fallback_holes = scene_model.placed_footprint(
        scene_model.Building(id="b2", east_m=3.0, north_m=4.0, length_m=8.0,
                             width_m=6.0, yaw_deg=0.0, height_m=5.0))
    fallback_area = building_mesh.triangle_area_m2(
        building_mesh.triangulate(fallback_outer, []))
    check("no outline falls back to the rectangle",
          len(fallback_outer) == 4 and not fallback_holes
          and abs(fallback_area - 48.0) < 1e-6)


def main() -> int:
    test_triangulate()
    test_extrude()
    test_slab()
    test_subdivide()
    test_placed_footprint()
    failed = [name for name, ok, _ in CHECKS if not ok]
    print(f"\n{len(CHECKS) - len(failed)} of {len(CHECKS)} checks passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
