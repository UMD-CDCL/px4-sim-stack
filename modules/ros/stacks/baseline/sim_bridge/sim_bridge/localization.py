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

from sim_bridge.geo import GroundPlane
from sim_bridge.projection import intersect_ground
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
