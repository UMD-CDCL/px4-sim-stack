#!/usr/bin/env python3
"""One COLLADA writer for every mesh the build emits.

COLLADA rather than OBJ because it states its own up axis. Gazebo reads
<up_axis>Z_UP</up_axis> and the mesh lands in the world the way its
coordinates meant it, with no guess about axis conventions.

Two facts, established by experiment in the sim container, are
load-bearing and live only here:

- <triangles> inputs use Blender-style per-input offsets (VERTEX 0,
  NORMAL 1, TEXCOORD 2) with tripled indices. Inputs that share offset 0
  reach ODE with no index array and crash the server.
- dartsim requires per-vertex normals that match the vertex count. A
  mesh without normals segfaults DART.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

POSITION_FORMAT = "{:.3f}"
UV_FORMAT = "{:.6f}"


@dataclass
class Material:
    """A texture path makes a UV-mapped diffuse material; without one the
    rgba color paints the surface flat."""
    id: str
    texture: str | None = None
    rgba: tuple = (0.7, 0.7, 0.7, 1.0)


@dataclass
class Geometry:
    """One mesh: shared vertex arrays, split into triangle groups so parts
    of one mesh can carry different materials."""
    id: str
    positions: np.ndarray                        # (n, 3)
    normals: np.ndarray                          # (n, 3)
    uvs: np.ndarray                              # (n, 2)
    groups: list = field(default_factory=list)   # [(material id, (m, 3) indices)]


def _effect(material: Material) -> str:
    if material.texture:
        shading = (f'     <diffuse><texture texture="{material.id}-sampler" '
                   f'texcoord="UVMAP"/></diffuse>\n')
        params = f"""    <newparam sid="{material.id}-surface">
     <surface type="2D"><init_from>{material.id}-image</init_from></surface>
    </newparam>
    <newparam sid="{material.id}-sampler">
     <sampler2D><source>{material.id}-surface</source></sampler2D>
    </newparam>
"""
    else:
        color = " ".join(f"{channel:.3f}" for channel in material.rgba)
        shading = (f"     <ambient><color>{color}</color></ambient>\n"
                   f"     <diffuse><color>{color}</color></diffuse>\n")
        params = ""
    return f"""  <effect id="{material.id}-effect">
   <profile_COMMON>
{params}    <technique sid="common">
     <lambert>
{shading}     </lambert>
    </technique>
   </profile_COMMON>
  </effect>
"""


def _source(source_id: str, values: np.ndarray, params: str) -> str:
    stride = values.shape[1]
    value_format = UV_FORMAT if stride == 2 else POSITION_FORMAT
    text = " ".join(value_format.format(v) for v in values.ravel())
    axes = "".join(f'<param name="{p}" type="float"/>' for p in params)
    return f"""    <source id="{source_id}">
     <float_array id="{source_id}-array" count="{values.size}">{text}</float_array>
     <technique_common>
      <accessor source="#{source_id}-array" count="{values.shape[0]}" stride="{stride}">
       {axes}
      </accessor>
     </technique_common>
    </source>
"""


def _triangles(geometry: Geometry, material_id: str, indices: np.ndarray) -> str:
    corner_indices = np.repeat(np.asarray(indices, dtype=np.int64).ravel(), 3)
    indices_text = " ".join(str(i) for i in corner_indices)
    return f"""    <triangles material="{material_id}-symbol" count="{len(indices)}">
     <input semantic="VERTEX" source="#{geometry.id}-vertices" offset="0"/>
     <input semantic="NORMAL" source="#{geometry.id}-normals" offset="1"/>
     <input semantic="TEXCOORD" source="#{geometry.id}-uvs" offset="2" set="0"/>
     <p>{indices_text}</p>
    </triangles>
"""


def _geometry(geometry: Geometry) -> str:
    groups = "".join(_triangles(geometry, material_id, indices)
                     for material_id, indices in geometry.groups)
    return f"""  <geometry id="{geometry.id}-geometry">
   <mesh>
{_source(f"{geometry.id}-positions", geometry.positions, "XYZ")}\
{_source(f"{geometry.id}-normals", geometry.normals, "XYZ")}\
{_source(f"{geometry.id}-uvs", geometry.uvs, "ST")}\
    <vertices id="{geometry.id}-vertices">
     <input semantic="POSITION" source="#{geometry.id}-positions"/>
    </vertices>
{groups}   </mesh>
  </geometry>
"""


def _node(geometry: Geometry) -> str:
    bindings = "".join(
        f"""       <instance_material symbol="{material_id}-symbol" target="#{material_id}">
        <bind_vertex_input semantic="UVMAP" input_semantic="TEXCOORD" input_set="0"/>
       </instance_material>
"""
        for material_id, _ in geometry.groups)
    return f"""   <node id="{geometry.id}-node">
    <instance_geometry url="#{geometry.id}-geometry">
     <bind_material>
      <technique_common>
{bindings}      </technique_common>
     </bind_material>
    </instance_geometry>
   </node>
"""


def write_dae(path: Path, materials: list[Material],
              geometries: list[Geometry]) -> None:
    images = "".join(
        f'  <image id="{m.id}-image"><init_from>{m.texture}</init_from></image>\n'
        for m in materials if m.texture)
    effects = "".join(_effect(m) for m in materials)
    material_items = "".join(
        f'  <material id="{m.id}"><instance_effect url="#{m.id}-effect"/></material>\n'
        for m in materials)
    geometry_items = "".join(_geometry(g) for g in geometries)
    nodes = "".join(_node(g) for g in geometries)
    document = f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
 <asset>
  <unit name="meter" meter="1"/>
  <up_axis>Z_UP</up_axis>
 </asset>
 <library_images>
{images} </library_images>
 <library_effects>
{effects} </library_effects>
 <library_materials>
{material_items} </library_materials>
 <library_geometries>
{geometry_items} </library_geometries>
 <library_visual_scenes>
  <visual_scene id="scene">
{nodes}  </visual_scene>
 </library_visual_scenes>
 <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(document)
