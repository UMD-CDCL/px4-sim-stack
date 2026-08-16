"""The localization surface: terrain and roofs, with walls left out.

Loads the worlds/<scene>_surface.json that scenegen build writes next to
the world, and answers where a ray first meets the scene. The surface is
2.5D: a terrain height grid, plus one horizontal roof rectangle per
building. A wall has no horizontal surface, so a ray aimed at one lands
on the terrain behind it on purpose.

Heights are scene z, which equals the map frame under the same convention
the ground truth node relies on: the world origin is the spawn point.

Pure arithmetic with no ROS dependency, like projection.py, so it can be
read, reasoned about and tested on its own.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

SURFACE_FORMAT = "scenegen-surface/1"

# ------------------------------------------------------------------- tunables
# Terrain march step along the ray, meters. Under half a grid cell, so a
# crossing cannot slip between samples on any slope the grid can hold.
MARCH_STEP_M = 2.0
# Bisection passes on the crossing bracket. Twenty passes on a 2 m bracket
# resolve the hit to a millimeter.
BISECTION_PASSES = 20


class SceneSurface:
    def __init__(self, side_m: float, grid_n: int,
                 terrain: list[list[float]], roofs: list[tuple]) -> None:
        self.side = float(side_m)
        self.n = int(grid_n)
        self.terrain = terrain
        # (east, north, cos_yaw, sin_yaw, half_length, half_width, roof_z)
        self.roofs = roofs
        self.lowest = min(min(row) for row in terrain)

    @classmethod
    def load(cls, path: str) -> "SceneSurface":
        """Raises OSError on a missing file and ValueError on a wrong one."""
        data = json.loads(Path(path).read_text())
        if data.get("format") != SURFACE_FORMAT:
            raise ValueError(f"{path} carries format {data.get('format')!r}, "
                             f"this code reads {SURFACE_FORMAT!r}")
        roofs = []
        for b in data["buildings"]:
            yaw = math.radians(b["yaw_deg"])
            roofs.append((b["east_m"], b["north_m"], math.cos(yaw),
                          math.sin(yaw), b["length_m"] / 2.0,
                          b["width_m"] / 2.0, b["roof_z"]))
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
        for east, north, cos_yaw, sin_yaw, half_length, half_width, roof_z in self.roofs:
            t = (roof_z - origin[2]) / direction[2]
            if t <= 0.0 or t > max_range or (best is not None and t >= best):
                continue
            hit_east = origin[0] + t * direction[0] - east
            hit_north = origin[1] + t * direction[1] - north
            local_x = hit_east * cos_yaw + hit_north * sin_yaw
            local_y = -hit_east * sin_yaw + hit_north * cos_yaw
            if abs(local_x) <= half_length and abs(local_y) <= half_width:
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
