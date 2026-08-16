"""The baseline stack.

Every stack in this directory must have a stack.launch.py at its root. The
ros container builds the directory with colcon and launches this file. That
is the whole contract.

What runs here

  mavros              MAVLink from the hub, and the map -> base_link transform
  scene_tf            the rest of the frame tree
  rtsp_camera x2      the gimbal and nadir streams, as ROS images
  detections_bridge   DeepStream detections, stamped with the frame time
  detection_annotator boxes over the video, colored by verdict, one per camera
  ground_projector    the camera footprint on the ground, one per camera
  detection_localizer detections placed on the ground, one per camera
  ground_truth        where the targets really are, simulation only
  scene_buildings     the scene's buildings for the 3D panel, from the
                      file scenegen build writes next to the world
  detection_scorer    estimates against truth, one per camera
  image_ground_projector  the live image draped on the localization
                      surface, one per camera
  fiducial_alignment  frame correction against a surveyed point, off by default
  click_to_gimbal     a click on the gimbal image becomes a gimbal command
  drone_position      the vehicle fix with its heading, for the Map panel
  foxglove_bridge     a websocket for the browser, on port 8765

Every per-camera stage runs once for each camera and nothing merges the two.
The cameras answer different questions, and one combined figure would
describe neither.

Everything the Foxglove layout reads is a stock ROS message, so it needs no
custom extension.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# ---------------------------------------------------- addresses, from compose
FCU_URL = os.environ.get("FCU_URL", "udp://:14555@mavlink-hub:14551")
RTSP_BASE = os.environ.get("RTSP_BASE", "rtsp://video-router:8554")
MQTT_HOST = os.environ.get("MQTT_HOST", "message-bus")

# ------------------------------------------------------------------- tunables
# Horizontal field of view in radians, from the sensor definitions in
# modules/sim/scenes/models/x500_recon/model.sdf. rtsp_camera builds its
# CameraInfo from these, and every projection depends on them.
#
# The anchor is which point of the box the localizer projects. Looking
# obliquely, a person's feet touch the ground, so the bottom edge is right.
# Looking straight down, the box center is: the bottom edge there shifts
# every estimate by half a box.
CAMERA = {
    "nadir": {
        "optical_frame": "nadir_camera_optical_frame",
        "hfov": float(os.environ.get("NADIR_HFOV", "1.74")),
        "anchor": "center",
    },
    "gimbal": {
        "optical_frame": "gimbal_camera_optical_frame",
        "hfov": float(os.environ.get("GIMBAL_HFOV", "2.0")),
        "anchor": "bottom",
    },
}

# Which camera feeds detection first. It must match RTSP_IN on the perception
# service, or detections get projected through the wrong lens from the wrong
# frame. Compose sets both from PERCEPTION_CAMERA for that reason. The first
# camera is also where payloads without a sensorId are routed.
PERCEPTION_CAMERA = os.environ.get("PERCEPTION_CAMERA", "nadir")
PERCEPTION_CAMERA_2 = os.environ.get("PERCEPTION_CAMERA_2", "gimbal")
CAMERAS = [PERCEPTION_CAMERA, PERCEPTION_CAMERA_2]

# The RTSP jitter buffer, in milliseconds. rtsp_camera also subtracts it when
# stamping, because it is the largest known part of the capture delay.
RTSP_LATENCY_MS = 100

# The GStreamer element that decodes each H.264 stream. nvh264dec moves the
# decode to NVDEC.
RTSP_DECODER = os.environ.get("RTSP_DECODER", "avdec_h264")

# An estimate counts as finding a target only within this many meters. Two
# meters means "a position good enough to send someone to". A wider gate
# scores geometry that is not actually useful as a success.
SCORING_GATE_M = float(os.environ.get("SCORING_GATE_M", "2.0"))
# Between the gate and this radius, an estimate still proves the detector
# saw the target, and the verdict is MISLOCALIZED instead of FP. Geometry
# faults, a wrong ground plane or a stale transform, put estimates in this
# band. Keep it under the spacing between targets, or a drifted estimate
# claims a neighbour.
SCORING_DETECTION_RADIUS_M = float(
    os.environ.get("SCORING_DETECTION_RADIUS_M", "10.0"))

# How a localization ray becomes a ground position, for the detection
# localizers and the gimbal ROI click alike. "plane" intersects the flat
# plane latched at the takeoff altitude. "scene" intersects the terrain
# and the roofs from the surface file scenegen build writes next to the
# world; walls are not in it on purpose. A missing surface file falls
# back to the plane, with an error in the log.
LOCALIZATION_MODE = os.environ.get("LOCALIZATION_MODE", "plane")
SCENE = os.environ.get("SCENE", "")
SURFACE_FILE = f"/scenes/worlds/{SCENE}_surface.json" if SCENE else ""
# Buildings for the 3D panel, written by the same build. Buildings only:
# vehicle props stay out of the panel on purpose.
BUILDINGS_FILE = f"/scenes/worlds/{SCENE}_buildings.json" if SCENE else ""

# Whose verdicts color the truth bubbles. The nadir camera keeps most of
# the scene in view from above, so counting it marks every target it sees
# as a miss even while only the gimbal hunts. The gimbal alone is the
# default. Set GROUND_TRUTH_CAMERAS=nadir,gimbal to combine both.
GROUND_TRUTH_CAMERAS = [
    cam.strip() for cam in
    os.environ.get("GROUND_TRUTH_CAMERAS", "gimbal").split(",") if cam.strip()]

# The resolution each frame is sampled down to before it is projected onto
# the ground, one entry for each camera. The gimbal mosaic is the one the
# operator reads, so it keeps more pixels; the nadir mosaic is context.
# 480x270 is 129,600 points and 2.1 MB per message, 240x135 a quarter of
# that. GROUND_IMAGE_SIZE sets both cameras at once, and
# GROUND_IMAGE_SIZE_GIMBAL or GROUND_IMAGE_SIZE_NADIR sets one.
GROUND_IMAGE_SIZE = {
    "gimbal": (os.environ.get("GROUND_IMAGE_SIZE_GIMBAL")
               or os.environ.get("GROUND_IMAGE_SIZE") or "480x270"),
    "nadir": (os.environ.get("GROUND_IMAGE_SIZE_NADIR")
              or os.environ.get("GROUND_IMAGE_SIZE") or "240x135"),
}
GROUND_IMAGE_RATE_HZ = float(os.environ.get("GROUND_IMAGE_RATE", "1.0"))

# How often ground truth publishes, in hertz. The markers project into the
# image panels, and 10 Hz keeps them smooth there. Do not lower it.
GROUND_TRUTH_RATE_HZ = float(os.environ.get("GROUND_TRUTH_RATE", "10.0"))

# Every sim_bridge node comes back this many seconds after a crash. Without
# respawn a dead per-camera node vanishes silently and takes its half of the
# layout with it.
RESPAWN_DELAY_S = 2.0

REFERENCE_FRAME = "map"

FIDUCIAL_ENABLED = os.environ.get("FIDUCIAL_ENABLED", "0") == "1"
# Empty means the localizers publish in the reference frame. With the
# fiducial enabled they publish in the corrected frame instead.
OUTPUT_FRAME = "fiducial" if FIDUCIAL_ENABLED else ""


def mavros_configs() -> list:
    """The PX4 plugin list and parameter file that ship with mavros.

    Returned as a list so a mavros release that renames or drops them
    degrades to defaults instead of failing to launch.
    """
    try:
        share = get_package_share_directory("mavros")
    except Exception:  # noqa: BLE001 - package missing is the case we handle
        return []
    found = []
    for name in ("px4_pluginlists.yaml", "px4_config.yaml"):
        path = os.path.join(share, "launch", name)
        if os.path.isfile(path):
            found.append(path)
    return found


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        # ------------------------------------------------------- MAVLink
        # No name and no namespace. mavros_node is a container that starts
        # several nodes of its own, and forcing a name makes two of them ask
        # for the same one. Its topics land under /mavros without any help.
        Node(
            package="mavros",
            executable="mavros_node",
            output="screen",
            parameters=[
                {
                    "fcu_url": FCU_URL,
                    "gcs_url": "",
                    "fcu_protocol": "v2.0",
                    "target_system_id": 1,
                    "target_component_id": 1,
                },
                *mavros_configs(),
                # Ours goes last, so it wins over the upstream defaults. It
                # turns on the map -> base_link transform and the rangefinder.
                os.path.join(os.path.dirname(__file__), "sim_bridge", "config",
                             "mavros_overrides.yaml"),
            ],
        ),

        # -------------------------------------------------------- frames
        Node(
            package="sim_bridge",
            executable="scene_tf",
            name="scene_tf",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            parameters=[{
                "base_frame": "base_link",
                "gimbal_mount_xyz": [0.0, 0.0, 0.10],
                "nadir_xyz": [
                    float(os.environ.get("NADIR_X", "0.10")),
                    float(os.environ.get("NADIR_Y", "0.0")),
                    float(os.environ.get("NADIR_Z", "-0.06")),
                ],
                "nadir_rpy_deg": [
                    float(os.environ.get("NADIR_ROLL", "0.0")),
                    float(os.environ.get("NADIR_PITCH", "90.0")),
                    float(os.environ.get("NADIR_YAW", "0.0")),
                ],
                "gimbal_reference": "earth",
                "gimbal_source": os.environ.get("GIMBAL_SOURCE", "auto"),
                # Any constant mounting rotation left after the frame
                # handling. The scene_tf diagnostics log prints the number to
                # put here.
                "gimbal_offset_rpy_deg": [
                    float(os.environ.get("GIMBAL_OFFSET_ROLL", "0.0")),
                    float(os.environ.get("GIMBAL_OFFSET_PITCH", "0.0")),
                    float(os.environ.get("GIMBAL_OFFSET_YAW", "0.0")),
                ],
                "log_gimbal_diagnostics":
                    os.environ.get("GIMBAL_DIAGNOSTICS", "0") == "1",
            }],
        ),

        # --------------------------------------------------------- video
        # The namespace carries the camera name, so two cameras never publish
        # to one topic.
        *[
            Node(
                package="sim_bridge",
                executable="rtsp_camera",
                name=f"{cam}_camera",
                namespace=f"camera/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "url": f"{RTSP_BASE}/{cam}",
                    "frame_id": CAMERA[cam]["optical_frame"],
                    "latency_ms": RTSP_LATENCY_MS,
                    "protocols": "tcp",
                    "decoder": RTSP_DECODER,
                    "hfov": CAMERA[cam]["hfov"],
                }],
            )
            for cam in CAMERAS
        ],

        # ---------------------------------------------------- detections
        Node(
            package="sim_bridge",
            executable="detections_bridge",
            name="detections_bridge",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            parameters=[{
                "host": MQTT_HOST,
                "port": 1883,
                "topic": "perception/detections",
                "bbox_format": "ltrb",
                # The coordinate space DeepStream reports boxes in. Compose
                # sets both from DS_WIDTH and DS_HEIGHT. The bridge scales
                # into the live image size, so a disagreement still lands the
                # boxes in the right place and a warning says so.
                "source_width": int(os.environ.get("DS_WIDTH", "1920")),
                "source_height": int(os.environ.get("DS_HEIGHT", "1080")),
                # Per camera corrections, as "camera=WxH", comma separated.
                # Normally empty. The failure it corrects is silent, and one
                # env var beats a code change at 2 am.
                "source_size_overrides": [
                    part.strip() for part in
                    os.environ.get("DS_COORD_OVERRIDES", "").split(",")
                ] or [""],
                "sensor_ids": CAMERAS,
                "sensor_frames": [CAMERA[cam]["optical_frame"] for cam in CAMERAS],
                # DeepStream's frame time trails true capture by about 16 ms
                # on this hardware. Measure yours with
                # scripts/measure-latency.py and put the difference here.
                "time_offset": 0.0,
            }],
        ),

        # ------------------------------------------- boxes on the video
        # Each annotator reads its own camera's verdicts, so the box colors
        # match the spheres for the same camera in the 3D view.
        *[
            Node(
                package="sim_bridge",
                executable="detection_annotator",
                name=f"{cam}_annotator",
                namespace=f"camera/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "detections_topic": f"/perception/{cam}/detections",
                    "annotations_topic": "annotations",
                    "verdicts_topic": f"/scoring/{cam}/verdicts",
                }],
            )
            for cam in CAMERAS
        ],

        # ------------------------------------------------------- ground
        # What each camera covers on the ground, as a polygon.
        *[
            Node(
                package="sim_bridge",
                executable="ground_projector",
                name=f"{cam}_ground_projector",
                namespace=f"camera/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "camera": cam,
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": CAMERA[cam]["optical_frame"],
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    # The footprint drapes over the same surface everything
                    # else localizes onto: one localizer class.
                    "localization_mode": LOCALIZATION_MODE,
                    "surface_file": SURFACE_FILE,
                }],
            )
            for cam in CAMERAS
        ],

        # One localizer for each camera, each in its own namespace, so the
        # topics come out as /perception/<camera>/detections_3d.
        *[
            Node(
                package="sim_bridge",
                executable="detection_localizer",
                name=f"{cam}_localizer",
                namespace=f"perception/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "camera": cam,
                    "detections_topic": f"/perception/{cam}/detections",
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": CAMERA[cam]["optical_frame"],
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    "anchor": CAMERA[cam]["anchor"],
                    "output_frame": OUTPUT_FRAME,
                    "localization_mode": LOCALIZATION_MODE,
                    "surface_file": SURFACE_FILE,
                }],
            )
            for cam in CAMERAS
        ],

        # ---------------------------------------- ground truth and scoring
        # Simulation only. These exist to measure the detector, not to fly
        # the aircraft. Drop them from a stack you intend to run on hardware.
        Node(
            package="sim_bridge",
            executable="ground_truth",
            name="ground_truth",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            parameters=[{
                "scenario_file": os.environ.get(
                    "GROUND_TRUTH_FILE",
                    "/scenes/scenarios/urban_casualties.yaml"),
                "reference_frame": REFERENCE_FRAME,
                "rate_hz": GROUND_TRUTH_RATE_HZ,
                # Scenario poses are Gazebo world coordinates and PX4's local
                # frame starts where the vehicle spawned, which is the same
                # point. Shift this if you spawn somewhere else.
                "origin_offset_xyz": [0.0, 0.0, 0.0],
                "cameras": GROUND_TRUTH_CAMERAS,
            }],
        ),

        # The scene's buildings as one latched MarkerArray for the 3D
        # panel. Reads the file scenegen build writes next to the world;
        # without one it publishes an empty scene and waits for it.
        Node(
            package="sim_bridge",
            executable="scene_buildings",
            name="scene_buildings",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            parameters=[{
                "buildings_file": BUILDINGS_FILE,
                "reference_frame": REFERENCE_FRAME,
            }],
        ),

        # One scorer for each camera. Each judges its own estimates against
        # the targets its own camera sees, so recall means "of what this
        # camera could see". The gate is shared, because it is a statement
        # about the task rather than about a lens.
        *[
            Node(
                package="sim_bridge",
                executable="detection_scorer",
                name=f"{cam}_scorer",
                namespace=f"scoring/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "camera": cam,
                    "detections_topic": f"/perception/{cam}/detections_3d",
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": CAMERA[cam]["optical_frame"],
                    "gate_radius": SCORING_GATE_M,
                    "detection_radius": SCORING_DETECTION_RADIUS_M,
                    "reference_frame": REFERENCE_FRAME,
                }],
            )
            for cam in CAMERAS
        ],

        # Frame alignment against a surveyed point. Off until you have one.
        # Do not survey a target you also score against.
        Node(
            package="sim_bridge",
            executable="fiducial_alignment",
            name="fiducial_alignment",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
            parameters=[{
                "enabled": FIDUCIAL_ENABLED,
                "surveyed_lla": [
                    float(os.environ.get("FIDUCIAL_SURVEYED_LAT", "0.0")),
                    float(os.environ.get("FIDUCIAL_SURVEYED_LON", "0.0")),
                    float(os.environ.get("FIDUCIAL_SURVEYED_ALT", "0.0")),
                ],
                "measured_lla": [
                    float(os.environ.get("FIDUCIAL_MEASURED_LAT", "0.0")),
                    float(os.environ.get("FIDUCIAL_MEASURED_LON", "0.0")),
                    float(os.environ.get("FIDUCIAL_MEASURED_ALT", "0.0")),
                ],
                "fiducial_frame": "fiducial",
                "reference_frame": REFERENCE_FRAME,
            }],
        ),

        # The live image draped on the localization surface, so the 3D view
        # shows the camera's own picture where the localizer thinks it is.
        *[
            Node(
                package="sim_bridge",
                executable="image_ground_projector",
                name=f"{cam}_ground_image",
                namespace=f"camera/{cam}",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "image_topic": f"/camera/{cam}/image_raw/compressed",
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": CAMERA[cam]["optical_frame"],
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    # The mosaic drapes over the same surface detections
                    # and ROI clicks localize onto: one localizer class.
                    "localization_mode": LOCALIZATION_MODE,
                    "surface_file": SURFACE_FILE,
                    "size": GROUND_IMAGE_SIZE[cam],
                    "rate_hz": GROUND_IMAGE_RATE_HZ,
                }],
            )
            for cam in CAMERAS
        ],

        # ------------------------------------------------ click to point
        # A click on the gimbal image in Foxglove becomes a MAVLink gimbal
        # attitude command. Gimbal only: the nadir camera is bolted down.
        # Skipped when the gimbal camera is not in the camera set, so the
        # node never claims gimbal control for a camera that is not
        # streaming.
        *([
            Node(
                package="sim_bridge",
                executable="click_to_gimbal",
                name="click_to_gimbal",
                output="screen",
                respawn=True,
                respawn_delay=RESPAWN_DELAY_S,
                parameters=[{
                    "click_topic": "/foxglove/cursor/click",
                    "camera_info_topic": "/camera/gimbal/camera_info",
                    "optical_frame": CAMERA["gimbal"]["optical_frame"],
                    "reference_frame": REFERENCE_FRAME,
                    # What a click does: roi holds the camera on the ground
                    # point, point turns the camera once, off ignores clicks.
                    # Runtime switchable with ros2 param set or the Foxglove
                    # Parameters panel.
                    "click_mode": os.environ.get("CLICK_MODE", "roi"),
                    "use_rel_alt": True,
                    # A click localizes exactly like a detection: the same
                    # mode, the same surface file, one localizer class.
                    "localization_mode": LOCALIZATION_MODE,
                    "surface_file": SURFACE_FILE,
                    # "gz_sim" matches the simulated gimbal, whose joints
                    # ride on the airframe: the node commands vehicle
                    # relative. Set "mavlink" for a gimbal that obeys the
                    # MAVLink frame flags: the node commands earth
                    # referenced with the lock flags set.
                    "gimbal_convention": "gz_sim",
                }],
            ),
        ] if "gimbal" in CAMERAS else []),

        # ------------------------------------------------- observability
        # The vehicle fix paired with the compass heading, as one
        # foxglove_msgs/LocationFix. NavSatFix cannot carry a heading, and
        # with one the Map panel draws an arrowhead that turns with the
        # drone.
        Node(
            package="sim_bridge",
            executable="drone_position",
            name="drone_position",
            output="screen",
            respawn=True,
            respawn_delay=RESPAWN_DELAY_S,
        ),

        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "asset_uri_allowlist": ["^package://(?!.*\\.\\.).*"],
                # The default is 10 MB, and the ground projection clouds are
                # megabytes each. Over the limit the bridge drops the channel
                # silently. See docs/troubleshooting.md.
                "send_buffer_limit": int(os.environ.get(
                    "FOXGLOVE_SEND_BUFFER", str(200 * 1024 * 1024))),
            }],
        ),
    ])
