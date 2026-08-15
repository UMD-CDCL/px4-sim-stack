"""Pixel to ground-plane geometry, shared by the projector and the localizer.

Everything here is pure arithmetic with no ROS dependency, so it can be read,
reasoned about and tested on its own.

Conventions
    Optical frame is REP 103: x right, y down, z forward along the view axis.
    A CameraInfo K is the pinhole matrix in that frame.
    The ground is a horizontal plane at a fixed height in the reference frame.
"""

from __future__ import annotations

import math
from typing import Iterable, Sequence


def intrinsics_ready(info) -> bool:
    """True when a CameraInfo carries a usable pinhole matrix.

    CameraInfo.k arrives as a numpy array, so a plain truth test on it raises
    rather than returning False. Test the length and fx instead.
    """
    return info is not None and len(info.k) == 9 and info.k[0] != 0.0


def ray_in_optical(u: float, v: float, k: Sequence[float]) -> tuple[float, float, float]:
    """Unit direction, in the optical frame, of the ray through pixel (u, v).

    k is the row-major 3x3 CameraInfo K: [fx, 0, cx, 0, fy, cy, 0, 0, 1].
    """
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    if fx == 0.0 or fy == 0.0:
        raise ValueError("CameraInfo K has a zero focal length")
    x = (u - cx) / fx
    y = (v - cy) / fy
    norm = math.sqrt(x * x + y * y + 1.0)
    return (x / norm, y / norm, 1.0 / norm)


def quat_rotate(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    """Rotate vector v by quaternion q, given as (x, y, z, w)."""
    qx, qy, qz, qw = q
    # t = 2 * (q_vec x v); v' = v + qw * t + q_vec x t
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (
        v[0] + qw * tx + (qy * tz - qz * ty),
        v[1] + qw * ty + (qz * tx - qx * tz),
        v[2] + qw * tz + (qx * ty - qy * tx),
    )


def intersect_ground(
    origin: Sequence[float],
    direction: Sequence[float],
    ground_z: float,
    max_range: float = 5000.0,
) -> tuple[float, float, float] | None:
    """Where a ray meets the horizontal plane z = ground_z.

    Returns None when the ray points away from the plane, runs parallel to it,
    or would meet it beyond max_range. Each of those means "this pixel does not
    see the ground", and a caller must not invent a point for it. A camera at
    the horizon produces rays that technically intersect thousands of meters
    away, and those answers are worthless.
    """
    dz = direction[2]
    if abs(dz) < 1e-9:
        return None
    t = (ground_z - origin[2]) / dz
    if t <= 0.0 or t > max_range:
        return None
    return (origin[0] + t * direction[0],
            origin[1] + t * direction[1],
            origin[2] + t * direction[2])


def slant_range(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def image_boundary(width: int, height: int, per_edge: int,
                   inset: float = 0.5) -> list[tuple[float, float]]:
    """Points along the image boundary, in the order that draws an outline.

    Each edge contributes per_edge points, corner included, so the list
    holds 4 * per_edge points. More points make the truncated footprint's
    arc smoother.
    """
    x0, y0 = inset, inset
    x1, y1 = width - inset, height - inset
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    out = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        for i in range(per_edge):
            f = i / per_edge
            out.append((ax + (bx - ax) * f, ay + (by - ay) * f))
    return out


def footprint_on_ground(
    boundary: Iterable[tuple[float, float]],
    k: Sequence[float],
    origin: Sequence[float],
    rotation: Sequence[float],
    ground_z: float,
    max_distance: float,
) -> list[tuple[float, float, float]] | None:
    """Project image boundary points onto the ground, truncated at a range.

    Each boundary ray keeps its ground hit when that hit lies within
    max_distance (horizontal meters) of the camera. A ray that misses the
    ground, or hits beyond the limit, is clamped to the max_distance circle
    in the direction it looks. A camera near the horizon therefore still
    reports the near ground it sees, bounded by an arc, instead of nothing.

    Returns None when no ray hits the ground within the limit, which means
    no ground within max_distance is in view.
    """
    out = []
    hits = 0
    for u, v in boundary:
        d = quat_rotate(rotation, ray_in_optical(u, v, k))
        horizontal = math.hypot(d[0], d[1])

        point = None
        if d[2] < -1e-9:
            t = (ground_z - origin[2]) / d[2]
            if t > 0.0:
                hx = origin[0] + t * d[0]
                hy = origin[1] + t * d[1]
                if math.hypot(hx - origin[0], hy - origin[1]) <= max_distance:
                    point = (hx, hy, ground_z)
                    hits += 1
        if point is None:
            if horizontal < 1e-9:
                continue    # straight up: no direction to clamp along
            point = (origin[0] + d[0] / horizontal * max_distance,
                     origin[1] + d[1] / horizontal * max_distance,
                     ground_z)
        out.append(point)

    return out if hits else None
