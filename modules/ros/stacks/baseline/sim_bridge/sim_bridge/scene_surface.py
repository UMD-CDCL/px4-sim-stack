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


class SceneSurface:
    def __init__(self, side_m: float, grid_n: int,
                 terrain: list[list[float]], roofs: list[tuple]) -> None:
        self.side = float(side_m)
        self.n = int(grid_n)
        self.terrain = terrain
        # (roof_z, covers), covers a callable on (east, north)
        self.roofs = roofs
        self.lowest = min(min(row) for row in terrain)

    @classmethod
    def load(cls, path: str) -> "SceneSurface":
        """Raises OSError on a missing file and ValueError on a wrong one."""
        data = json.loads(Path(path).read_text())
        if data.get("format") not in SURFACE_FORMATS:
            raise ValueError(f"{path} carries format {data.get('format')!r}, "
                             f"this code reads {SURFACE_FORMATS}")
        roofs = []
        for building in data["buildings"]:
            covers = _polygon_covers(building) if "footprint" in building \
                else _rectangle_covers(building)
            roofs.append((building["roof_z"], covers))
        return cls(data["side_m"], data["grid_n"], data["terrain_z"], roofs)

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
