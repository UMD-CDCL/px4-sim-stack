"""The localization surface: terrain and roofs, with walls left out.

Loads the worlds/<scene>_surface.json that scenegen build writes next to
the world, and answers where a ray first meets the scene. The surface is
2.5D: a terrain height grid, plus one horizontal roof polygon per
building, with courtyard holes cut out. A wall has no horizontal surface,
so a ray aimed at one lands on the terrain behind it on purpose. Format 1
files carry roof rectangles instead of polygons; both load.

Heights are scene z, which equals the map frame under the same convention
the ground truth node relies on: the world origin is the spawn point.

Pure arithmetic with no ROS dependency, like projection.py, so it can be
read, reasoned about and tested on its own.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

SURFACE_FORMATS = ("scenegen-surface/1", "scenegen-surface/2")

# ------------------------------------------------------------------- tunables
# Terrain march step along the ray, meters. Under half a grid cell, so a
# crossing cannot slip between samples on any slope the grid can hold.
MARCH_STEP_M = 2.0
# Bisection passes on the crossing bracket. Twenty passes on a 2 m bracket
# resolve the hit to a millimeter.
BISECTION_PASSES = 20


def _rectangle_covers(building: dict):
    yaw = math.radians(building["yaw_deg"])
    east, north = building["east_m"], building["north_m"]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_length = building["length_m"] / 2.0
    half_width = building["width_m"] / 2.0

    def covers(hit_east: float, hit_north: float) -> bool:
        de, dn = hit_east - east, hit_north - north
        local_x = de * cos_yaw + dn * sin_yaw
        local_y = -de * sin_yaw + dn * cos_yaw
        return abs(local_x) <= half_length and abs(local_y) <= half_width

    return covers


def _polygon_covers(building: dict):
    """Even-odd over the footprint and its holes together, so a point in a
    courtyard crosses an even number of edges and counts as outside."""
    rings = [building["footprint"]] + building.get("holes", [])
    edges = []
    for ring in rings:
        for i, (east1, north1) in enumerate(ring):
            east2, north2 = ring[(i + 1) % len(ring)]
            if north1 != north2:
                edges.append((east1, north1, east2, north2))

    def covers(hit_east: float, hit_north: float) -> bool:
        inside = False
        for east1, north1, east2, north2 in edges:
            if (north1 > hit_north) != (north2 > hit_north) \
                    and hit_east < east1 + (hit_north - north1) \
                    / (north2 - north1) * (east2 - east1):
                inside = not inside
        return inside

    return covers


def _rectangle_ring(building: dict) -> list:
    yaw = math.radians(building["yaw_deg"])
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_length = building["length_m"] / 2.0
    half_width = building["width_m"] / 2.0
    return [[building["east_m"] + dx * cos_yaw - dy * sin_yaw,
             building["north_m"] + dx * sin_yaw + dy * cos_yaw]
            for dx, dy in [(-half_length, -half_width), (half_length, -half_width),
                           (half_length, half_width), (-half_length, half_width)]]


def _batch_roof(building: dict) -> tuple:
    """(roof_z, bbox, edges) for the vectorized pass: every non-horizontal
    ring edge as one row of (x1, y1, x2, y2), and the outer bounding box
    as a cheap prefilter. A format-1 rectangle becomes its corner ring."""
    if "footprint" in building:
        rings = [building["footprint"]] + building.get("holes", [])
    else:
        rings = [_rectangle_ring(building)]
    edges = []
    for ring in rings:
        for i, (x1, y1) in enumerate(ring):
            x2, y2 = ring[(i + 1) % len(ring)]
            if y1 != y2:
                edges.append((x1, y1, x2, y2))
    outer = rings[0]
    bbox = (min(p[0] for p in outer), min(p[1] for p in outer),
            max(p[0] for p in outer), max(p[1] for p in outer))
    return building["roof_z"], bbox, np.asarray(edges, dtype=np.float64)


class SceneSurface:
    def __init__(self, side_m: float, grid_n: int,
                 terrain: list[list[float]], roofs: list[tuple],
                 batch_roofs: list[tuple] | None = None) -> None:
        self.side = float(side_m)
        self.n = int(grid_n)
        self.terrain = np.asarray(terrain, dtype=np.float64)
        # (roof_z, covers), covers a callable on (east, north)
        self.roofs = roofs
        # (roof_z, bbox, edges) per roof, for intersect_batch
        self.batch_roofs = batch_roofs or []
        self.lowest = float(self.terrain.min())

    @classmethod
    def load(cls, path: str) -> "SceneSurface":
        """Raises OSError on a missing file and ValueError on a wrong one."""
        data = json.loads(Path(path).read_text())
        if data.get("format") not in SURFACE_FORMATS:
            raise ValueError(f"{path} carries format {data.get('format')!r}, "
                             f"this code reads {SURFACE_FORMATS}")
        roofs = []
        batch_roofs = []
        for building in data["buildings"]:
            covers = _polygon_covers(building) if "footprint" in building \
                else _rectangle_covers(building)
            roofs.append((building["roof_z"], covers))
            batch_roofs.append(_batch_roof(building))
        return cls(data["side_m"], data["grid_n"], data["terrain_z"], roofs,
                   batch_roofs)

    def terrain_z(self, east: float, north: float) -> float:
        """Bilinear terrain height. Points outside the square clamp to the
        edge, the same rule the mesh build uses."""
        n = self.n
        fi = (east + self.side / 2.0) / self.side * n
        fj = (north + self.side / 2.0) / self.side * n
        fi = min(max(fi, 0.0), n - 1e-9)
        fj = min(max(fj, 0.0), n - 1e-9)
        i, j = int(fi), int(fj)
        di, dj = fi - i, fj - j
        t = self.terrain
        return (t[j][i] * (1 - di) * (1 - dj) + t[j][i + 1] * di * (1 - dj)
                + t[j + 1][i] * (1 - di) * dj + t[j + 1][i + 1] * di * dj)

    def intersect(self, origin, direction, max_range: float):
        """Where the ray first meets a roof or the terrain, or None.

        direction must be a unit vector, so the ray parameter is meters.
        Only descending rays intersect: every surface here faces up, and a
        climbing ray could only meet a roof from below.
        """
        if direction[2] >= -1e-9:
            return None
        roof = self._nearest_roof(origin, direction, max_range)
        ground = self._terrain_crossing(origin, direction,
                                        roof if roof is not None else max_range)
        t = ground if ground is not None else roof
        if t is None:
            return None
        return (origin[0] + t * direction[0],
                origin[1] + t * direction[1],
                origin[2] + t * direction[2])

    def _nearest_roof(self, origin, direction, max_range: float):
        best = None
        for roof_z, covers in self.roofs:
            t = (roof_z - origin[2]) / direction[2]
            if t <= 0.0 or t > max_range or (best is not None and t >= best):
                continue
            if covers(origin[0] + t * direction[0], origin[1] + t * direction[1]):
                best = t
        return best

    def _terrain_crossing(self, origin, direction, limit: float):
        """March along the ray until it drops below the terrain, then
        bisect the bracket. A ray already below the terrain at its origin
        returns None: the camera is under the surface and the input is
        nonsense."""
        def clearance(t: float) -> float:
            east = origin[0] + t * direction[0]
            north = origin[1] + t * direction[1]
            return origin[2] + t * direction[2] - self.terrain_z(east, north)

        if clearance(0.0) <= 0.0:
            return None
        above = 0.0
        t = MARCH_STEP_M
        while t < limit:
            if clearance(t) <= 0.0:
                return self._bisect(clearance, above, t)
            # Below the lowest terrain and still descending: no crossing
            # remains ahead.
            if origin[2] + t * direction[2] < self.lowest:
                return None
            above = t
            t += MARCH_STEP_M
        if clearance(limit) <= 0.0:
            return self._bisect(clearance, above, limit)
        return None

    @staticmethod
    def _bisect(clearance, above: float, below: float) -> float:
        for _ in range(BISECTION_PASSES):
            middle = (above + below) / 2.0
            if clearance(middle) <= 0.0:
                below = middle
            else:
                above = middle
        return below

    # ------------------------------------------------------------------ batch
    def _terrain_z_batch(self, east: np.ndarray, north: np.ndarray) -> np.ndarray:
        """terrain_z over arrays, same clamping, same bilinear weights."""
        n = self.n
        fi = np.clip((east + self.side / 2.0) / self.side * n, 0.0, n - 1e-9)
        fj = np.clip((north + self.side / 2.0) / self.side * n, 0.0, n - 1e-9)
        i = fi.astype(np.int64)
        j = fj.astype(np.int64)
        di, dj = fi - i, fj - j
        t = self.terrain
        return (t[j, i] * (1 - di) * (1 - dj) + t[j, i + 1] * di * (1 - dj)
                + t[j + 1, i] * (1 - di) * dj + t[j + 1, i + 1] * di * dj)

    def intersect_batch(self, origin, dirs: np.ndarray,
                        max_range: np.ndarray) -> np.ndarray:
        """intersect() for one origin and many unit rays at once.

        dirs is (n, 3), max_range per ray. Returns the ray parameter per
        ray, np.inf where a ray misses. Same rules as the scalar path:
        only descending rays intersect, roofs are horizontal, and walls
        stay out, so a ray past a roof edge lands on the terrain behind
        it. The image mosaic feeds a whole pixel grid through here.
        """
        ox, oy, oz = float(origin[0]), float(origin[1]), float(origin[2])
        ux, uy, uz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
        misses = np.full(ux.shape, np.inf)
        descending = uz < -1e-9
        if not descending.any() or (oz - self.terrain_z(ox, oy)) <= 0.0:
            return misses

        roof_t = np.full(ux.shape, np.inf)
        for roof_z, (x0, y0, x1, y1), edges in self.batch_roofs:
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (roof_z - oz) / uz
            candidates = np.nonzero(descending & (t > 0.0) & (t <= max_range)
                                    & (t < roof_t))[0]
            if not candidates.size:
                continue
            px = ox + t[candidates] * ux[candidates]
            py = oy + t[candidates] * uy[candidates]
            in_box = (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1)
            candidates, px, py = candidates[in_box], px[in_box], py[in_box]
            if not candidates.size:
                continue
            inside = np.zeros(px.shape, dtype=bool)
            for ex1, ey1, ex2, ey2 in edges:
                crosses = (ey1 > py) != (ey2 > py)
                with np.errstate(divide="ignore", invalid="ignore"):
                    cross_x = ex1 + (py - ey1) / (ey2 - ey1) * (ex2 - ex1)
                inside ^= crosses & (px < cross_x)
            hit = candidates[inside]
            roof_t[hit] = t[hit]

        # The terrain march, every ray in step, capped per ray at its
        # range, its roof, and the depth below the lowest terrain.
        with np.errstate(divide="ignore", invalid="ignore"):
            floor_t = (self.lowest - oz) / uz
        limit = np.minimum(np.minimum(max_range, roof_t),
                           np.where(descending, floor_t, 0.0))
        alive = np.nonzero(descending & (limit > 0.0))[0]
        ground_t = np.full(ux.shape, np.inf)
        march = MARCH_STEP_M
        top = float(limit[alive].max()) if alive.size else 0.0
        while alive.size and march <= top + MARCH_STEP_M:
            t_step = np.minimum(march, limit[alive])
            px = ox + t_step * ux[alive]
            py = oy + t_step * uy[alive]
            pz = oz + t_step * uz[alive]
            crossed = (pz - self._terrain_z_batch(px, py)) <= 0.0
            ground_t[alive[crossed]] = t_step[crossed]
            alive = alive[~(crossed | (t_step >= limit[alive]))]
            march += MARCH_STEP_M
        found = np.nonzero(np.isfinite(ground_t))[0]
        if found.size:
            below = ground_t[found]
            above = np.maximum(below - MARCH_STEP_M, 0.0)
            fx, fy, fz = ux[found], uy[found], uz[found]
            for _ in range(BISECTION_PASSES):
                middle = (above + below) / 2.0
                under = (oz + middle * fz
                         - self._terrain_z_batch(ox + middle * fx,
                                                 oy + middle * fy)) <= 0.0
                below = np.where(under, middle, below)
                above = np.where(under, above, middle)
            ground_t[found] = below
        return np.where(np.isfinite(ground_t), ground_t, roof_t)
