#!/usr/bin/env python3
"""Terrain: the elevation grid becomes a textured COLLADA mesh.

Texture coordinates come from the imagery georeference, vertex by vertex,
not from a linear stretch. The crop's sub-pixel edges and the slight
mercator curvature across the square are absorbed here, so a feature in
the texture sits over its own coordinates. The same mapping serves the
building roofs, so a roof shows the pixels that sit over its footprint.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import shapely

import collada
import geo
import sources
from scene_model import FlattenZone


def load_elevation(scene_data_dir: Path) -> tuple[np.ndarray, dict]:
    grid = np.load(scene_data_dir / "elevation.npy")
    meta = json.loads((scene_data_dir / "elevation.json").read_text())
    return grid, meta


def grid_coordinates(meta: dict) -> tuple[np.ndarray, np.ndarray]:
    """East and north coordinates of every grid vertex, shape (n+1, n+1).
    Row 0 is the south edge, matching sources.fetch_elevation_grid."""
    n = meta["grid_n"]
    side = meta["side_m"]
    axis = np.linspace(-side / 2.0, side / 2.0, n + 1)
    east, north = np.meshgrid(axis, axis)
    return east, north


def sample_height(grid: np.ndarray, meta: dict, east: float, north: float) -> float:
    """Bilinear height at one ENU point, AMSL meters. Points outside the
    square clamp to the edge."""
    n = meta["grid_n"]
    side = meta["side_m"]
    fi = (east + side / 2.0) / side * n
    fj = (north + side / 2.0) / side * n
    fi = min(max(fi, 0.0), n - 1e-9)
    fj = min(max(fj, 0.0), n - 1e-9)
    i, j = int(fi), int(fj)
    di, dj = fi - i, fj - j
    return float(grid[j, i] * (1 - di) * (1 - dj) + grid[j, i + 1] * di * (1 - dj)
                 + grid[j + 1, i] * (1 - di) * dj + grid[j + 1, i + 1] * di * dj)


def apply_flatten_zones(grid: np.ndarray, meta: dict, zones: list[FlattenZone],
                        origin_alt_m: float) -> np.ndarray:
    """A copy of the grid with each zone's vertices set to one height.
    This is how a bridge or an overhang that the elevation data recorded
    as solid ground becomes level again."""
    flattened = grid.copy()
    east, north = grid_coordinates(meta)
    for zone in zones:
        if len(zone.polygon_m) < 3:
            continue
        polygon = shapely.Polygon([(e, n) for e, n in zone.polygon_m])
        mask = shapely.contains_xy(polygon, east.ravel(), north.ravel()).reshape(east.shape)
        if not mask.any():
            continue
        if zone.mode == "min":
            flattened[mask] = flattened[mask].min()
        elif zone.mode == "mean":
            flattened[mask] = flattened[mask].mean()
        elif zone.mode == "manual" and zone.height_m is not None:
            flattened[mask] = origin_alt_m + zone.height_m
    return flattened


def imagery_pixels(frame: geo.GeoFrame, imagery: dict,
                   east_north: np.ndarray) -> np.ndarray:
    """The raster pixel over each ENU point, shape (n, 2)."""
    georef = sources.RasterGeoref(imagery["zoom"], imagery["origin_px"],
                                  imagery["origin_py"])
    pixels = np.zeros((len(east_north), 2))
    for index, (east, north) in enumerate(east_north):
        lat, lon, _ = frame.enu_to_latlon(float(east), float(north))
        pixels[index] = georef.latlon_to_raster_px(lat, lon)
    return pixels


def imagery_uv(frame: geo.GeoFrame, imagery: dict,
               east_north: np.ndarray) -> np.ndarray:
    """Texture coordinates over each ENU point, shape (n, 2). COLLADA
    texture space puts v=0 at the image bottom; pixel rows count from the
    top."""
    pixels = imagery_pixels(frame, imagery, east_north)
    return np.column_stack([pixels[:, 0] / imagery["width_px"],
                            1.0 - pixels[:, 1] / imagery["height_px"]])


def _vertex_normals(z: np.ndarray, step_m: float) -> np.ndarray:
    grad_north, grad_east = np.gradient(z, step_m)
    normals = np.dstack([-grad_east, -grad_north, np.ones_like(z)])
    lengths = np.linalg.norm(normals, axis=2, keepdims=True)
    return (normals / lengths).reshape(-1, 3)


def _triangle_indices(n: int) -> np.ndarray:
    """Two counterclockwise-from-above triangles per grid cell."""
    triangles = []
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i
            b = a + 1
            c = b + (n + 1)
            d = a + (n + 1)
            triangles.append((a, b, c))
            triangles.append((a, c, d))
    return np.array(triangles, dtype=np.int64)


def write_terrain_dae(frame: geo.GeoFrame, grid_amsl: np.ndarray, meta: dict,
                      imagery: dict, origin_alt_m: float, texture_rel_path: str,
                      out_path: Path) -> dict:
    """The terrain mesh, z = 0 at the scene origin's ground. Returns a few
    numbers for the build report."""
    east, north = grid_coordinates(meta)
    z = grid_amsl - origin_alt_m
    n = meta["grid_n"]
    step = meta["side_m"] / n

    positions = np.column_stack([east.ravel(), north.ravel(), z.ravel()])
    normals = _vertex_normals(z, step)
    uvs = imagery_uv(frame, imagery,
                     np.column_stack([east.ravel(), north.ravel()]))
    triangles = _triangle_indices(n)

    collada.write_dae(
        out_path,
        [collada.Material(id="terrain-material", texture=texture_rel_path)],
        [collada.Geometry(id="terrain", positions=positions, normals=normals,
                          uvs=uvs, groups=[("terrain-material", triangles)])])
    return {"vertices": positions.shape[0], "triangles": int(triangles.shape[0]),
            "z_min": round(float(z.min()), 2), "z_max": round(float(z.max()), 2)}
