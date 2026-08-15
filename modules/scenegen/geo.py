#!/usr/bin/env python3
"""Geodesy for the scene generator.

A scene is a square of ground centered on one WGS84 coordinate. Three
conversions cover everything the pipeline does:

  lat/lon  <->  scene ENU meters. Exact, through ECEF. Gazebo resolves
                <spherical_coordinates> the same way, so a GPS fix taken in
                the simulator over a placed object returns the coordinate
                the object came from.
  lat/lon  <->  web mercator pixels and tile indices, for imagery and
                elevation tiles.
  yaw           radians counterclockwise from east, the Gazebo ENU
                convention. Stored in scene.json as degrees.

Standard library only, so every other module imports this one freely.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

WGS84_SEMI_MAJOR_M = 6378137.0
WGS84_FLATTENING = 1.0 / 298.257223563
WGS84_ECC2 = WGS84_FLATTENING * (2.0 - WGS84_FLATTENING)
TILE_PX = 256
# Web mercator is defined to this latitude. The pipeline refuses scenes
# closer to a pole long before the projection degrades.
MERCATOR_LAT_LIMIT_DEG = 85.05112878


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    prime_vertical = WGS84_SEMI_MAJOR_M / math.sqrt(1.0 - WGS84_ECC2 * math.sin(lat) ** 2)
    x = (prime_vertical + alt_m) * math.cos(lat) * math.cos(lon)
    y = (prime_vertical + alt_m) * math.cos(lat) * math.sin(lon)
    z = (prime_vertical * (1.0 - WGS84_ECC2) + alt_m) * math.sin(lat)
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Iterative inverse of geodetic_to_ecef. Converges below a millimeter
    in a few rounds for any point between the ground and orbit."""
    lon = math.atan2(y, x)
    ground_distance = math.hypot(x, y)
    lat = math.atan2(z, ground_distance * (1.0 - WGS84_ECC2))
    alt = 0.0
    for _ in range(6):
        prime_vertical = WGS84_SEMI_MAJOR_M / math.sqrt(1.0 - WGS84_ECC2 * math.sin(lat) ** 2)
        alt = ground_distance / math.cos(lat) - prime_vertical
        lat = math.atan2(z, ground_distance * (1.0 - WGS84_ECC2 * prime_vertical / (prime_vertical + alt)))
    return math.degrees(lat), math.degrees(lon), alt


@dataclass(frozen=True)
class GeoFrame:
    """A local tangent frame: x east, y north, z up, origin at one lat/lon.

    origin_alt_m is the AMSL height of the frame origin, so that "up = 0"
    means "at the scene origin's ground height", which is where the Gazebo
    world puts z = 0.
    """

    origin_lat: float
    origin_lon: float
    origin_alt_m: float = 0.0

    def latlon_to_enu(self, lat_deg: float, lon_deg: float,
                      alt_m: float | None = None) -> tuple[float, float, float]:
        if alt_m is None:
            alt_m = self.origin_alt_m
        ox, oy, oz = geodetic_to_ecef(self.origin_lat, self.origin_lon, self.origin_alt_m)
        px, py, pz = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        dx, dy, dz = px - ox, py - oy, pz - oz
        sin_lat = math.sin(math.radians(self.origin_lat))
        cos_lat = math.cos(math.radians(self.origin_lat))
        sin_lon = math.sin(math.radians(self.origin_lon))
        cos_lon = math.cos(math.radians(self.origin_lon))
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up

    def enu_to_latlon(self, east: float, north: float,
                      up: float = 0.0) -> tuple[float, float, float]:
        """Returns (lat, lon, AMSL altitude)."""
        ox, oy, oz = geodetic_to_ecef(self.origin_lat, self.origin_lon, self.origin_alt_m)
        sin_lat = math.sin(math.radians(self.origin_lat))
        cos_lat = math.cos(math.radians(self.origin_lat))
        sin_lon = math.sin(math.radians(self.origin_lon))
        cos_lon = math.cos(math.radians(self.origin_lon))
        dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
        dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
        dz = cos_lat * north + sin_lat * up
        return ecef_to_geodetic(ox + dx, oy + dy, oz + dz)


def latlon_to_mercator_px(lat_deg: float, lon_deg: float, zoom: int) -> tuple[float, float]:
    """Global web mercator pixel coordinates. x grows east, y grows south."""
    world_px = TILE_PX * (1 << zoom)
    x = (lon_deg + 180.0) / 360.0 * world_px
    lat = math.radians(lat_deg)
    y = (1.0 - math.log(math.tan(lat) + 1.0 / math.cos(lat)) / math.pi) / 2.0 * world_px
    return x, y


def mercator_px_to_latlon(x: float, y: float, zoom: int) -> tuple[float, float]:
    world_px = TILE_PX * (1 << zoom)
    lon = x / world_px * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / world_px))))
    return lat, lon


def tile_index(px: float) -> int:
    return int(math.floor(px / TILE_PX))


def ground_resolution_m_per_px(lat_deg: float, zoom: int) -> float:
    return (math.cos(math.radians(lat_deg)) * 2.0 * math.pi * WGS84_SEMI_MAJOR_M
            / (TILE_PX * (1 << zoom)))
