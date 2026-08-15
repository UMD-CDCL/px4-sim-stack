#!/usr/bin/env python3
"""Terrain: the elevation grid becomes a textured COLLADA mesh.

COLLADA rather than OBJ because it states its own up axis. Gazebo reads
<up_axis>Z_UP</up_axis> and the mesh lands in the world the way the grid
meant it, with no guess about axis conventions.

Texture coordinates come from the imagery georeference, vertex by vertex,
not from a linear stretch. The crop's sub-pixel edges and the slight
mercator curvature across the square are absorbed here, so a feature in
the texture sits over its own coordinates.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import shapely

import geo
import sources
from scene_model import FlattenZone

# ------------------------------------------------------------------- tunables
# A visual sits exactly on the collision surface and z-fights nothing,
# because the terrain is the only thing at its height. No offset needed.
DAE_FLOAT_FORMAT = "{:.3f}"


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


def _vertex_uvs(frame: geo.GeoFrame, imagery: dict,
                east: np.ndarray, north: np.ndarray) -> np.ndarray:
    georef = sources.RasterGeoref(imagery["zoom"], imagery["origin_px"],
                                  imagery["origin_py"])
    uvs = np.zeros((east.size, 2))
    flat_east, flat_north = east.ravel(), north.ravel()
    for index in range(east.size):
        lat, lon, _ = frame.enu_to_latlon(float(flat_east[index]), float(flat_north[index]))
        px, py = georef.latlon_to_raster_px(lat, lon)
        # COLLADA texture space puts v=0 at the image bottom; pixel rows
        # count from the top.
        uvs[index] = (px / imagery["width_px"], 1.0 - py / imagery["height_px"])
    return uvs


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
    uvs = _vertex_uvs(frame, imagery, east, north)
    triangles = _triangle_indices(n)

    fmt = DAE_FLOAT_FORMAT.format
    positions_text = " ".join(fmt(v) for v in positions.ravel())
    normals_text = " ".join(fmt(v) for v in normals.ravel())
    uvs_text = " ".join("{:.6f}".format(v) for v in uvs.ravel())
    # One index stream per input, Blender style. Position, normal and uv
    # share indices here, so each corner repeats its index three times.
    # Gazebo's collada-to-physics path needs this layout: inputs that share
    # offset 0 reach ODE with no index array and crash the server.
    corner_indices = np.repeat(triangles.ravel(), 3).reshape(-1, 3)
    indices_text = " ".join(str(i) for i in corner_indices.ravel())
    vertex_count = positions.shape[0]

    dae = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset>
  <unit name="meter" meter="1"/>
  <up_axis>Z_UP</up_axis>
 </asset>
 <library_images>
  <image id="satellite-image"><init_from>{texture_rel_path}</init_from></image>
 </library_images>
 <library_effects>
  <effect id="terrain-effect">
   <profile_COMMON>
    <newparam sid="satellite-surface">
     <surface type="2D"><init_from>satellite-image</init_from></surface>
    </newparam>
    <newparam sid="satellite-sampler">
     <sampler2D><source>satellite-surface</source></sampler2D>
    </newparam>
    <technique sid="common">
     <lambert>
      <diffuse><texture texture="satellite-sampler" texcoord="UVMAP"/></diffuse>
     </lambert>
    </technique>
   </profile_COMMON>
  </effect>
 </library_effects>
 <library_materials>
  <material id="terrain-material"><instance_effect url="#terrain-effect"/></material>
 </library_materials>
 <library_geometries>
  <geometry id="terrain-geometry">
   <mesh>
    <source id="terrain-positions">
     <float_array id="terrain-positions-array" count="{vertex_count * 3}">{positions_text}</float_array>
     <technique_common>
      <accessor source="#terrain-positions-array" count="{vertex_count}" stride="3">
       <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor>
     </technique_common>
    </source>
    <source id="terrain-normals">
     <float_array id="terrain-normals-array" count="{vertex_count * 3}">{normals_text}</float_array>
     <technique_common>
      <accessor source="#terrain-normals-array" count="{vertex_count}" stride="3">
       <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor>
     </technique_common>
    </source>
    <source id="terrain-uvs">
     <float_array id="terrain-uvs-array" count="{vertex_count * 2}">{uvs_text}</float_array>
     <technique_common>
      <accessor source="#terrain-uvs-array" count="{vertex_count}" stride="2">
       <param name="S" type="float"/><param name="T" type="float"/>
      </accessor>
     </technique_common>
    </source>
    <vertices id="terrain-vertices">
     <input semantic="POSITION" source="#terrain-positions"/>
    </vertices>
    <triangles material="terrain-material-symbol" count="{triangles.shape[0]}">
     <input semantic="VERTEX" source="#terrain-vertices" offset="0"/>
     <input semantic="NORMAL" source="#terrain-normals" offset="1"/>
     <input semantic="TEXCOORD" source="#terrain-uvs" offset="2" set="0"/>
     <p>{indices_text}</p>
    </triangles>
   </mesh>
  </geometry>
 </library_geometries>
 <library_visual_scenes>
  <visual_scene id="terrain-scene">
   <node id="terrain-node">
    <instance_geometry url="#terrain-geometry">
     <bind_material>
      <technique_common>
       <instance_material symbol="terrain-material-symbol" target="#terrain-material">
        <bind_vertex_input semantic="UVMAP" input_semantic="TEXCOORD" input_set="0"/>
       </instance_material>
      </technique_common>
     </bind_material>
    </instance_geometry>
   </node>
  </visual_scene>
 </library_visual_scenes>
 <scene><instance_visual_scene url="#terrain-scene"/></scene>
</COLLADA>
"""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(dae)
    return {"vertices": vertex_count, "triangles": int(triangles.shape[0]),
            "z_min": round(float(z.min()), 2), "z_max": round(float(z.max()), 2)}
