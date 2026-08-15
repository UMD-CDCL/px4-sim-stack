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

# Both the footprint polygon and the projected imagery stop at this
# horizontal distance from the camera, so the picture always fills the
# outline that frames it.
GROUND_VIEW_MAX_DISTANCE_M = 100.0


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


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) from roll, pitch, yaw in radians, applied
    intrinsically in z-y-x order, the aerospace convention."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    """Hamilton product, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q: Sequence[float]) -> tuple[float, float, float, float]:
    """The inverse of a unit quaternion (x, y, z, w)."""
    return (-q[0], -q[1], -q[2], q[3])


def rpy_from_quat(q: Sequence[float]) -> tuple[float, float, float]:
    """Roll, pitch, yaw in radians from a quaternion (x, y, z, w). The
    inverse of quat_from_rpy, in the same intrinsic z-y-x convention."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(angle: float) -> float:
    """The same angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


# Aerospace to ROS. NED_TO_ENU swaps the reference frame; FRD_TO_FLU swaps
# the body axes. Applied on both sides because both ends change, the same
# conversion MAVROS applies to every attitude.
NED_TO_ENU = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)


def aerospace_to_ros(q: Sequence[float]) -> tuple[float, float, float, float]:
    """An absolute attitude, NED reference and FRD body, into ENU and FLU."""
    return quat_mul(quat_mul(NED_TO_ENU, q), FRD_TO_FLU)


def ros_to_aerospace(q: Sequence[float]) -> tuple[float, float, float, float]:
    """An absolute attitude, ENU reference and FLU body, into NED and FRD."""
    return quat_mul(quat_mul(quat_conj(NED_TO_ENU), q), quat_conj(FRD_TO_FLU))


def body_frd_to_flu(q: Sequence[float]) -> tuple[float, float, float, float]:
    """A rotation relative to the body, from FRD axes to FLU axes.

    A relative rotation has no reference frame to change, so only the axis
    convention converts. This is not the absolute NED to ENU conversion:
    using either on the wrong quantity produces a frame that tracks the
    aircraft incorrectly.
    """
    return (q[0], -q[1], -q[2], q[3])


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


def pointing_rpy_ned(direction_enu: Sequence[float]) -> tuple[float, float, float]:
    """The earth-referenced attitude that puts the view axis on a direction.

    direction_enu is a unit vector in the ENU reference frame. The result is
    roll, pitch, yaw in radians in the NED convention MAVLink attitude
    commands use. Roll is zero, so the image stays level. Straight up or
    down leaves yaw underdetermined, and atan2 then returns an arbitrary
    stable angle, which is harmless with the view axis vertical.
    """
    east, north, up = direction_enu
    pitch = math.asin(max(-1.0, min(1.0, up)))
    yaw = math.atan2(east, north)
    return 0.0, pitch, yaw


def pointing_rpy_body(direction_flu: Sequence[float]) -> tuple[float, float, float]:
    """The vehicle-relative attitude that puts the view axis on a direction.

    direction_flu is a unit vector in the FLU body frame. The result is
    roll, pitch, yaw in radians in the FRD convention vehicle-relative
    MAVLink attitude commands use. Roll is zero, so the image stays level.
    """
    forward, left, up = direction_flu
    # Right, forward, up against the body play east, north, up in the world.
    return pointing_rpy_ned((-left, forward, up))


# Body convention to REP 103 optical convention.
LINK_TO_OPTICAL = quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2)


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
