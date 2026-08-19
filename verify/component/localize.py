#!/usr/bin/env python3
"""Localize hardcoded detection boxes and check where they land.

No detector and no simulator. The box, the camera pose, the fix and the
calibration are all written here, so the answer follows from tf_loc's own
arithmetic and from the scene surface on disk. Each case states the position it
expects from geometry that is worked out independently of the node.

Run inside the onboard image: ./px4sim verify localize
"""

import json
import math
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.time import Time as RclpyTime

from builtin_interfaces.msg import Time as TimeMsg
from cdcl_umd_msgs.msg import TargetBox, TargetBoxArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, NavSatFix
from vision_msgs.msg import BoundingBox2D

from umd_uas.tf_loc import TfLocalizationNode

ORIGIN_FRAME = "uas11_home_position"
DRONE_FRAME = "d11_base_link"
GIMBAL_FRAME = "d11_gimbal_frame"
CAMERA_FRAME = "d11_rgb_offset"
RANGEFINDER_FRAME = "d11_rangefinder"

CALIBRATION_SIZE = (1920, 1080)
HFOV_DEG = 27.45
FLIGHT_ALTITUDE_M = 50.0
STAMP = TimeMsg(sec=1000, nanosec=0)

TOLERANCE_M = 0.05


def camera_info(width: int, height: int, hfov_deg: float) -> CameraInfo:
    """A pinhole calibration with no distortion, from a field of view."""
    fx = fy = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx, cy = width / 2.0, height / 2.0
    info = CameraInfo()
    info.width, info.height = width, height
    info.distortion_model = "plumb_bob"
    info.d = [0.0] * 5
    info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return info


def jpeg_of_size(width: int, height: int) -> bytes:
    """Real JPEG bytes, because tf_loc reads the box pixel space out of them."""
    ok, buffer = cv2.imencode(".jpg", np.zeros((height, width, 3), np.uint8))
    assert ok
    return buffer.tobytes()


def transform(parent: str, child: str, translation, rotation: R) -> TransformStamped:
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        float(value) for value in translation)
    q = rotation.as_quat()
    t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
    t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
    return t


def boresight(azimuth_deg: float, depression_deg: float) -> R:
    """Camera body rotation whose forward axis points at that bearing and dip.

    Composed rather than given as an Euler sequence: pitch the nose down off
    east, then yaw onto the bearing. Written out because the two orders differ
    and a single sequence string hides which one it means.
    """
    yaw = R.from_euler("z", 90.0 - azimuth_deg, degrees=True)
    pitch = R.from_euler("y", depression_deg, degrees=True)
    return yaw * pitch


def pose_the_vehicle(node, azimuth_deg: float, depression_deg: float,
                     altitude_m: float = FLIGHT_ALTITUDE_M) -> None:
    """One camera pose, written into the transform buffer as the whole tree."""
    attitude = boresight(azimuth_deg, depression_deg)
    at = (0.0, 0.0, altitude_m)
    slant = altitude_m / max(math.sin(math.radians(depression_deg)), 1e-9)
    for t in (transform(ORIGIN_FRAME, DRONE_FRAME, at, R.identity()),
              transform(ORIGIN_FRAME, GIMBAL_FRAME, at, attitude),
              transform(ORIGIN_FRAME, CAMERA_FRAME, at, attitude),
              transform(GIMBAL_FRAME, RANGEFINDER_FRAME, (slant, 0.0, 0.0), R.identity())):
        node.tf_buffer.set_transform_static(t, "verify")


def detection(pixel_uv, image_size) -> TargetBoxArray:
    """A TargetBoxArray carrying one box at one pixel of one image size."""
    msg = TargetBoxArray()
    msg.seq = 1
    msg.header.stamp = STAMP
    msg.header.frame_id = CAMERA_FRAME
    msg.source_img.header.stamp = STAMP
    msg.source_img.format = "jpeg"
    msg.source_img.data = jpeg_of_size(*image_size)
    box = TargetBox()
    box.target_bbox = BoundingBox2D()
    box.target_bbox.center.position.x = float(pixel_uv[0])
    box.target_bbox.center.position.y = float(pixel_uv[1])
    box.target_bbox.size_x, box.target_bbox.size_y = 20.0, 20.0
    msg.uav_target_boxes = [box]
    return msg


def build_node(anchor_lla, terrain_dir: str, ground_altitude_m: float = 0.0):
    """A tf_loc that reads no topics: the tree, the fix and the calibration are
    written straight in, which is what `spin_tf_listener=False` is for."""
    settings = {
        "bbox_anchor": "center",
        "camera_frame": CAMERA_FRAME,
        "drone_frame": DRONE_FRAME,
        "gimbal_frame": GIMBAL_FRAME,
        "rangefinder_frame": RANGEFINDER_FRAME,
        "localization_origin_frame": ORIGIN_FRAME,
        "localization.ground_altitude_m": ground_altitude_m,
        "terrain.cache_dir": terrain_dir,
        "tf_lookup_timeout_duration_sec": 0,
        "tf_lookup_timeout_duration_nanosec": 0,
    }
    args = ["--ros-args"]
    for name, value in settings.items():
        args += ["-p", f"{name}:={value}"]
    rclpy.init(args=args)

    node = TfLocalizationNode(spin_tf_listener=False)
    node.cam_info_cb(camera_info(*CALIBRATION_SIZE, HFOV_DEG))
    fix = NavSatFix()
    fix.header.stamp = STAMP
    fix.latitude, fix.longitude, fix.altitude = anchor_lla
    node.origin.fix = fix
    node.gps_cache.add(fix)
    return node


def localize(node, msg: TargetBoxArray):
    """Where one box lands, as (lat, lon, alt), or None when it does not."""
    out = node.unlocalized_tba_cb(msg)
    if out is None or not out.uav_target_boxes:
        return None
    landed = out.uav_target_boxes[0].target_location_altimeter_plane
    return (landed.latitude, landed.longitude, landed.altitude)


def metres_apart(a, b, anchor) -> float:
    return float(np.linalg.norm(
        np.array(pm.geodetic2enu(*a, *anchor, deg=True))
        - np.array(pm.geodetic2enu(*b, *anchor, deg=True))))


def roof_over(surface: dict):
    """A building roof from the scene, and a point on it, read from the file
    rather than from the code under test."""
    for building in surface.get("buildings", []):
        ring = np.asarray(building["footprint"], dtype=np.float64)
        centre = ring.mean(axis=0)
        # A convex-enough footprint has its centroid inside it. Take the first
        # that also sits well inside the tile, so the ray starts over the roof.
        if np.hypot(*centre) < surface["side_m"] / 2.0 - 50.0:
            return float(building["roof_z"]), (float(centre[0]), float(centre[1]))
    return None, None


def main() -> int:
    surface_path = Path(sys.argv[1])
    terrain_dir = str(surface_path.parent)
    surface = json.loads(surface_path.read_text())
    anchor = tuple(surface["origin_lla"])
    results = []

    def record(name, want_enu, got_lla, tolerance=TOLERANCE_M):
        if got_lla is None:
            results.append((False, name, "no localization"))
            return
        want = pm.enu2geodetic(*want_enu, *anchor, deg=True)
        error = metres_apart(want, got_lla, anchor)
        results.append((error <= tolerance, name, f"{error:.3f} m from where geometry puts it"))

    # 1. Straight down over the anchor, on the flat plane. The ground altitude
    #    is the anchor's own, so the box lands on the anchor itself.
    node = build_node(anchor, tempfile.mkdtemp(), ground_altitude_m=anchor[2])
    pose_the_vehicle(node, azimuth_deg=0.0, depression_deg=90.0)
    centre_pixel = (CALIBRATION_SIZE[0] / 2.0, CALIBRATION_SIZE[1] / 2.0)
    record("a centred box straight down lands on the anchor",
           (0.0, 0.0, 0.0), localize(node, detection(centre_pixel, CALIBRATION_SIZE)))

    # 2. Depressed 45 degrees due north from 50 m: 50 m north of the anchor.
    pose_the_vehicle(node, azimuth_deg=0.0, depression_deg=45.0)
    record("a centred box 45 degrees down, due north, lands 50 m north",
           (0.0, FLIGHT_ALTITUDE_M, 0.0),
           localize(node, detection(centre_pixel, CALIBRATION_SIZE)))

    # 3. The same detection reported in a 640x360 preview must land where the
    #    full-size one does. The two spaces have disagreed by a factor of three.
    pose_the_vehicle(node, azimuth_deg=0.0, depression_deg=45.0)
    corner_full = (CALIBRATION_SIZE[0] * 0.75, CALIBRATION_SIZE[1] * 0.25)
    full = localize(node, detection(corner_full, CALIBRATION_SIZE))
    preview = localize(node, detection((640 * 0.75, 360 * 0.25), (640, 360)))
    if full is None or preview is None:
        results.append((False, "a preview box lands where the full size box lands",
                        "no localization"))
    else:
        error = metres_apart(full, preview, anchor)
        results.append((error <= TOLERANCE_M,
                        "a preview box lands where the full size box lands",
                        f"{error:.3f} m apart"))
    node.destroy_node()
    rclpy.shutdown()

    # 4. With the scene surface loaded and no altitude configured, the terrain
    #    supplies the fleet's ground.
    node = build_node(anchor, terrain_dir, ground_altitude_m=0.0)
    results.append((node._ground_altitude is not None
                    and abs(node._ground_altitude - anchor[2]) < 0.01,
                    "an unset ground altitude comes from the terrain tiles",
                    f"{node._ground_altitude}"))

    # 5. A ray straight down onto a building roof stops on the roof, at the
    #    height the scene file gives it.
    roof_z, roof_point = roof_over(surface)
    if roof_z is None:
        results.append((False, "a box on a roof lands on the roof", "no usable building"))
    else:
        altitude = roof_z + 80.0
        node.tf_buffer.set_transform_static(
            transform(ORIGIN_FRAME, DRONE_FRAME, (*roof_point, altitude), R.identity()), "verify")
        node.tf_buffer.set_transform_static(
            transform(ORIGIN_FRAME, GIMBAL_FRAME, (*roof_point, altitude),
                      boresight(0.0, 90.0)), "verify")
        node.tf_buffer.set_transform_static(
            transform(ORIGIN_FRAME, CAMERA_FRAME, (*roof_point, altitude),
                      boresight(0.0, 90.0)), "verify")
        node.tf_buffer.set_transform_static(
            transform(GIMBAL_FRAME, RANGEFINDER_FRAME, (altitude - roof_z, 0.0, 0.0),
                      R.identity()), "verify")
        record("a box on a roof lands on the roof, not on the ground beside it",
               (*roof_point, roof_z),
               localize(node, detection(centre_pixel, CALIBRATION_SIZE)))
    node.destroy_node()
    rclpy.shutdown()

    for ok, name, detail in results:
        print(f"{'ok' if ok else 'FAIL'}\t{name}\t{detail}")
    return 0 if all(ok for ok, _, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
