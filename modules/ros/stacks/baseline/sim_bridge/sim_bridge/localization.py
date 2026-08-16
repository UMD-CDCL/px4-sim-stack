"""One ray-to-ground localizer for every node that casts one.

The detection localizers and the gimbal click both turn a camera ray
into a ground position. This class is that answer in one place, so the
two can never disagree: a click on a detection's pixel holds the gimbal
on the point the localizer reported for it, roofs included.

Two modes select what the ray meets, the LOCALIZATION_MODE contract
from stack.launch.py. "plane" intersects the flat plane latched at the
takeoff altitude (sim_bridge/geo.py, GroundPlane). "scene" intersects
the terrain and the roof polygons from the surface file scenegen build
writes next to the world; walls are not in it, so a ray onto a wall
lands on the terrain behind it. A missing or unreadable surface file
falls back to the plane, with an error in the log.
"""

from __future__ import annotations

import math

import numpy as np

from sim_bridge.geo import GroundPlane
from sim_bridge.projection import intersect_ground, quat_rotate, ray_in_optical
from sim_bridge.scene_surface import SceneSurface


class GroundLocalizer:
    """Attach one to a node. Declares localization_mode and surface_file,
    plus GroundPlane's use_rel_alt and ground_z, loads the surface once,
    and answers intersect() for every ray after that. ground_plane stays
    readable for owners that need the raw plane or rel_alt."""

    def __init__(self, node) -> None:
        node.declare_parameter("localization_mode", "plane")
        node.declare_parameter("surface_file", "")
        self.node = node
        self.ground_plane = GroundPlane(node)
        self.surface = self._load_surface()

    def _load_surface(self) -> SceneSurface | None:
        """The scene surface when localization_mode asks for it and the
        file loads. None means the ground plane."""
        mode = self.node.get_parameter("localization_mode").value
        if mode == "plane":
            return None
        path = self.node.get_parameter("surface_file").value
        if mode != "scene":
            self.node.get_logger().warn(
                f"localization_mode {mode!r} is not 'plane' or 'scene'. "
                f"Using the ground plane.")
            return None
        try:
            surface = SceneSurface.load(path)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.node.get_logger().error(
                f"localization_mode is 'scene' but no usable surface at "
                f"'{path}' ({exc}). Using the ground plane. Re-run scenegen "
                f"build to write the surface next to the world.")
            return None
        self.node.get_logger().info(
            f"scene surface: {len(surface.roofs)} roofs, terrain "
            f"{surface.side:.0f} m square")
        return surface

    @property
    def description(self) -> str:
        """For startup log lines: what a ray meets."""
        return "the scene surface" if self.surface else "the ground plane"

    def intersect(self, origin, direction,
                  max_range: float) -> tuple[float, float, float] | None:
        """Where the ray first meets the scene, or None within max_range.

        direction must be a unit vector. The origin's z also feeds the
        plane latch, so the first call with rel_alt in hand pins the
        plane to the takeoff altitude.
        """
        ground_z = self.ground_plane.z(origin[2])
        if self.surface is not None:
            return self.surface.intersect(origin, direction, max_range)
        return intersect_ground(origin, direction, ground_z, max_range)

    def intersect_batch(self, origin, dirs: np.ndarray,
                        max_range: np.ndarray) -> np.ndarray:
        """intersect() for one origin and many unit rays at once, for the
        image mosaic's pixel grid. dirs is (n, 3), max_range per ray.
        Returns the ray parameter per ray, np.inf where a ray misses.
        In plane mode a ray on either side of the plane can meet it, the
        same rule intersect_ground applies."""
        ground_z = self.ground_plane.z(origin[2])
        if self.surface is not None:
            return self.surface.intersect_batch(origin, dirs, max_range)
        uz = dirs[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ground_z - origin[2]) / uz
        keep = (np.abs(uz) >= 1e-9) & (t > 0.0) & (t <= max_range)
        return np.where(keep, t, np.inf)

    def ground_z_at(self, east: float, north: float,
                    pose_z: float | None = None) -> float:
        """The ground under a point: the terrain in scene mode, the
        latched plane otherwise. Roofs stay out on purpose: this answers
        where a clamped footprint arc point rests, and that is a ground
        line even beside a building."""
        if self.surface is not None:
            return self.surface.terrain_z(east, north)
        return self.ground_plane.z(pose_z)

    def footprint(self, boundary, k, origin, rotation,
                  max_distance: float) -> list | None:
        """The camera's covered ground: each image-boundary ray's surface
        hit, truncated at max_distance horizontal meters.

        A ray that misses the surface, or hits beyond the limit, clamps
        to the limit circle in the direction it looks, resting on the
        ground there. A camera near the horizon therefore still reports
        the near ground it sees, bounded by an arc, instead of nothing.
        Returns the outline as (x, y, z) tuples, or None when no ray hits
        within the limit: no ground in view. Hits follow the same surface
        the detections localize onto, so the outline drapes over terrain
        and roofs in scene mode.
        """
        rays = [quat_rotate(rotation, ray_in_optical(u, v, k))
                for u, v in boundary]
        dirs = np.asarray(rays, dtype=np.float64)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        horizontal = np.hypot(dirs[:, 0], dirs[:, 1])
        with np.errstate(divide="ignore", invalid="ignore"):
            max_range = max_distance / np.maximum(horizontal, 1e-12)
        t = self.intersect_batch(origin, dirs, max_range)
        # Only a downward hit counts, the rule the plane footprint always
        # had: a camera under the latched plane clamps instead of drawing
        # a footprint overhead.
        t = np.where(dirs[:, 2] < -1e-9, t, np.inf)

        out = []
        hits = 0
        for index in range(dirs.shape[0]):
            if math.isfinite(t[index]):
                out.append((origin[0] + t[index] * dirs[index, 0],
                            origin[1] + t[index] * dirs[index, 1],
                            origin[2] + t[index] * dirs[index, 2]))
                hits += 1
                continue
            if horizontal[index] < 1e-9:
                continue    # straight up: no direction to clamp along
            east = origin[0] + dirs[index, 0] / horizontal[index] * max_distance
            north = origin[1] + dirs[index, 1] / horizontal[index] * max_distance
            out.append((east, north, self.ground_z_at(east, north, origin[2])))
        return out if hits else None
