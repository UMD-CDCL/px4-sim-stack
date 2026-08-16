#!/usr/bin/env python3
"""Buildings: a footprint polygon becomes an extruded mesh.

The footprint can be concave and can hold courtyard holes. Shapely splits
a footprint with holes along a line through each hole, which turns the
hole into boundary notches, so the ear clipper only ever sees a simple
polygon. Triangles come out as point triples with no shared indices: the
walls are flat-shaded, so shared vertices would only blur the normals.

Coordinate contract, from scene_model.placed_footprint: the outer ring is
counterclockwise and holes are clockwise, rings open. With that winding
one wall emitter serves both: the outward face of an outer wall and the
courtyard face of a hole wall.
"""

from __future__ import annotations

import numpy as np
import shapely
from shapely.ops import split as split_polygon

# ------------------------------------------------------------------- tunables
# Triangles under this area are collinear slivers: never emitted, and a
# corner this flat counts as an ear so clipping cannot stall on it.
SLIVER_AREA_M2 = 1e-6
# Roof subdivision stops here. Only the Foxglove color sampling wants the
# density; the world mesh stays coarse. The cap bounds a degenerate input.
MAX_SUBDIVIDED_TRIANGLES = 2048


def _cross(a, b, c) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _strictly_inside(p, a, b, c) -> bool:
    return (_cross(a, b, p) > SLIVER_AREA_M2
            and _cross(b, c, p) > SLIVER_AREA_M2
            and _cross(c, a, p) > SLIVER_AREA_M2)


def _ear_clip(ring: list) -> list[tuple]:
    """Triangles of a simple counterclockwise polygon, as point triples.

    Sliver ears are clipped but not emitted. When numeric noise leaves no
    clean ear, the most convex corner goes anyway: on a valid simple
    polygon that only ever discards a sliver, and it guarantees an end.
    """
    points = [tuple(p) for p in ring]
    points = [p for i, p in enumerate(points) if p != points[i - 1]]
    triangles: list[tuple] = []

    def corner(i: int) -> tuple:
        n = len(points)
        return points[(i - 1) % n], points[i], points[(i + 1) % n]

    while len(points) > 3:
        ear = None
        for i in range(len(points)):
            a, b, c = corner(i)
            if _cross(a, b, c) < -SLIVER_AREA_M2:
                continue
            if any(_strictly_inside(p, a, b, c) for p in points
                   if p not in (a, b, c)):
                continue
            ear = i
            break
        if ear is None:
            ear = max(range(len(points)), key=lambda i: _cross(*corner(i)))
        a, b, c = corner(ear)
        if _cross(a, b, c) > SLIVER_AREA_M2:
            triangles.append((a, b, c))
        points.pop(ear)
    if len(points) == 3 and _cross(*points) > SLIVER_AREA_M2:
        triangles.append(tuple(points))
    return triangles


def _simple_pieces(geometry, pieces: list, depth: int = 0) -> None:
    """Split a polygon until no piece keeps an interior ring. Each cut runs
    through one hole, so the hole opens into two boundary notches. A cut
    that changes nothing drops the remaining holes rather than loop."""
    if geometry.is_empty:
        return
    if geometry.geom_type != "Polygon":
        for part in getattr(geometry, "geoms", []):
            _simple_pieces(part, pieces, depth)
        return
    if not geometry.interiors:
        pieces.append(geometry)
        return
    if depth > 2 * len(geometry.interiors) + 8:
        pieces.append(shapely.Polygon(geometry.exterior))
        return
    hole_center = geometry.interiors[0].centroid
    min_x, min_y, max_x, max_y = geometry.bounds
    cut = shapely.LineString([(hole_center.x, min_y - 1.0),
                              (hole_center.x, max_y + 1.0)])
    parts = split_polygon(geometry, cut)
    if parts.geom_type != "Polygon" and len(parts.geoms) > 1:
        _simple_pieces(parts, pieces, depth + 1)
    else:
        pieces.append(shapely.Polygon(geometry.exterior))


def triangulate(outer: list, holes: list) -> list[tuple]:
    """Triangles that cover the footprint, as counterclockwise point
    triples in the footprint's own plane."""
    polygon = shapely.Polygon(outer, holes)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    pieces: list = []
    _simple_pieces(polygon, pieces)
    triangles: list[tuple] = []
    for piece in pieces:
        ring = list(piece.exterior.coords)[:-1]
        if not piece.exterior.is_ccw:
            ring.reverse()
        triangles += _ear_clip(ring)
    return triangles


def extrude(outer: list, holes: list, base_z: float,
            top_z: float) -> tuple[np.ndarray, np.ndarray]:
    """(roof, walls) of the extruded footprint, each (n, 3, 3): triangles
    of 3D points. The roof faces up. Walls face out of the solid, into a
    courtyard for a hole ring. No floor: nothing ever sees one."""
    roof = np.array([[(x, y, top_z) for x, y in triangle]
                     for triangle in triangulate(outer, holes)]
                    ).reshape(-1, 3, 3)
    walls = []
    for ring in [outer] + holes:
        for i, (ax, ay) in enumerate(ring):
            bx, by = ring[(i + 1) % len(ring)]
            if (ax, ay) == (bx, by):
                continue
            walls.append([(ax, ay, base_z), (bx, by, base_z), (bx, by, top_z)])
            walls.append([(ax, ay, base_z), (bx, by, top_z), (ax, ay, top_z)])
    return roof, np.array(walls).reshape(-1, 3, 3)


def face_normals(triangles: np.ndarray) -> np.ndarray:
    """Per-vertex normals for flat shading, (3n, 3): each vertex takes its
    face normal. dartsim requires normals to match the vertex count."""
    normals = np.cross(triangles[:, 1] - triangles[:, 0],
                       triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths[lengths == 0.0] = 1.0
    return np.repeat(normals / lengths, 3, axis=0)


def subdivide(triangles: np.ndarray, max_edge_m: float) -> np.ndarray:
    """The triangles with every edge split down to max_edge_m, at long-edge
    midpoints. Same coverage, denser vertices to sample colors at."""
    queue = [triangles[i] for i in range(triangles.shape[0])]
    done: list[np.ndarray] = []
    while queue:
        triangle = queue.pop()
        edges = [np.linalg.norm(triangle[(i + 1) % 3] - triangle[i])
                 for i in range(3)]
        longest = int(np.argmax(edges))
        if edges[longest] <= max_edge_m \
                or len(queue) + len(done) >= MAX_SUBDIVIDED_TRIANGLES:
            done.append(triangle)
            continue
        a, b = triangle[longest], triangle[(longest + 1) % 3]
        c = triangle[(longest + 2) % 3]
        middle = (a + b) / 2.0
        queue.append(np.array([a, middle, c]))
        queue.append(np.array([middle, b, c]))
    return np.array(done).reshape(-1, 3, 3)


def triangle_area_m2(triangles) -> float:
    """The summed area, for coverage checks against the source polygon."""
    return sum(abs(_cross(a, b, c)) / 2.0 for a, b, c in triangles)
