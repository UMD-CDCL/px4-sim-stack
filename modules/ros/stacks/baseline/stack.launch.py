"""The baseline stack.

Every stack in this directory must have a stack.launch.py at its root. The ros
container builds the directory with colcon and launches this file. That is the
whole contract, so a stack can hold any packages it likes.

What runs here

  mavros              MAVLink, from the hub. Telemetry, commands, missions, and
                      the map -> base_link transform.
  rtsp_camera x2      the gimbal and nadir streams, as ROS images.
  scene_tf            the rest of the frame tree, and an airframe to look at.
  detections_bridge   DeepStream detections, stamped with the frame time.
  ground_projector    what the gimbal covers on the ground.
  detection_localizer detections placed on the ground, with covariance.
  foxglove_bridge     a websocket for the browser, on port 8765.

Everything the 3D view needs is a stock ROS message: /tf, MarkerArray,
PolygonStamped, PoseArray. Nothing here invents a schema, so the Foxglove
layout in foxglove/ needs no custom extension to read it.

Frame names appear in several nodes and they must agree. They are defined once
below rather than repeated as literals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

FCU_URL = os.environ.get("FCU_URL", "udp://:14555@mavlink-hub:14551")
RTSP_BASE = os.environ.get("RTSP_BASE", "rtsp://video-router:8554")
MQTT_HOST = os.environ.get("MQTT_HOST", "message-bus")

REFERENCE_FRAME = "map"
GIMBAL_OPTICAL = "gimbal_camera_optical_frame"
NADIR_OPTICAL = "nadir_camera_optical_frame"

# Which camera feeds detection and localization. It must match RTSP_IN on the
# perception service, or detections get projected through the wrong lens from
# the wrong frame and land somewhere plausible and wrong. Compose sets both
# from PERCEPTION_CAMERA for that reason.
PERCEPTION_CAMERA = os.environ.get("PERCEPTION_CAMERA", "nadir")
PERCEPTION_OPTICAL = NADIR_OPTICAL if PERCEPTION_CAMERA == "nadir" else GIMBAL_OPTICAL

# Which point of the box to project.
#
# Looking obliquely, a person's feet touch the ground and their centre does
# not, so the bottom edge is the right anchor. Looking straight down, the whole
# body projects around the point where they stand and the box centre is the
# right anchor: taking the bottom edge there just shifts every estimate by half
# a box in one image direction, which shows up as a constant offset in metres.
PERCEPTION_ANCHOR = "centre" if PERCEPTION_CAMERA == "nadir" else "bottom"

# Field of view of each camera, in radians, from the sensor definitions in
# modules/sim/scenes/models/x500_recon/model.sdf. rtsp_camera builds its
# CameraInfo from these, and every projection depends on them being right.
GIMBAL_HFOV = 2.0
NADIR_HFOV = 1.74

# The RTSP jitter buffer, in milliseconds. It is also the largest known part of
# the delay between capture and an image reaching ROS, so rtsp_camera subtracts
# it when stamping. Change the two together.
RTSP_LATENCY_MS = 100


def mavros_configs() -> list:
    """The PX4 plugin list and parameter file that ship with mavros.

    Returned as a list so a mavros release that renames or drops them degrades
    to defaults instead of failing to launch.
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


def camera(name: str, hfov: float, optical_frame: str) -> Node:
    # The namespace carries the camera name. A node name does not affect topic
    # names, so two cameras in the same namespace would both publish to
    # <ns>/image_raw and one would silently overwrite the other.
    return Node(
        package="sim_bridge",
        executable="rtsp_camera",
        name=f"{name}_camera",
        namespace=f"camera/{name}",
        output="screen",
        parameters=[{
            "url": f"{RTSP_BASE}/{name}",
            "frame_id": optical_frame,
            "latency_ms": RTSP_LATENCY_MS,
            "protocols": "tcp",
            "decoder": "avdec_h264",
            "hfov": hfov,
        }],
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([

        # ------------------------------------------------------- MAVLink
        # No name and no namespace. mavros_node is a container that starts
        # several nodes of its own, including mavros_router and one for each
        # plugin. Forcing a name makes two of them ask for the same name, and
        # the process aborts with an RCLError that does not say which. Its
        # topics land under /mavros without any help.
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
                    # Do not add a name with a slash in it. ROS 2 rejects it,
                    # and mavros_node then aborts with an RCLError that does not
                    # name the parameter. Connection timing comes from
                    # px4_config.yaml below.
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
            parameters=[{
                "base_frame": "base_link",
                "gimbal_mount_xyz": [0.0, 0.0, 0.10],
                "nadir_xyz": [0.10, 0.0, -0.06],
                "gimbal_reference": "vehicle",
                "gimbal_rate_hz": 30.0,
            }],
        ),

        # --------------------------------------------------------- video
        camera("gimbal", GIMBAL_HFOV, GIMBAL_OPTICAL),
        camera("nadir", NADIR_HFOV, NADIR_OPTICAL),

        # ---------------------------------------------------- detections
        Node(
            package="sim_bridge",
            executable="detections_bridge",
            name="detections_bridge",
            output="screen",
            parameters=[{
                "host": MQTT_HOST,
                "port": 1883,
                "topic": "perception/detections",
                "bbox_format": "ltrb",
                "frame_id": PERCEPTION_OPTICAL,
                # DeepStream's frame time trails true capture by about 16 ms on
                # this hardware. Measure yours with scripts/measure-latency.py
                # and put the difference here if it matters to you.
                "time_offset": 0.0,
                "max_age": 2.0,
            }],
        ),

        # ------------------------------------------------------- ground
        Node(
            package="sim_bridge",
            executable="ground_projector",
            name="gimbal_ground_projector",
            namespace="camera/gimbal",
            output="screen",
            parameters=[{
                "camera_info_topic": "/camera/gimbal/camera_info",
                "optical_frame": GIMBAL_OPTICAL,
                "reference_frame": REFERENCE_FRAME,
                "use_rel_alt": True,
                "ground_z": 0.0,
                "rate_hz": 5.0,
            }],
        ),

        Node(
            package="sim_bridge",
            executable="ground_projector",
            name="nadir_ground_projector",
            namespace="camera/nadir",
            output="screen",
            parameters=[{
                "camera_info_topic": "/camera/nadir/camera_info",
                "optical_frame": NADIR_OPTICAL,
                "reference_frame": REFERENCE_FRAME,
                "use_rel_alt": True,
                "ground_z": 0.0,
                "rate_hz": 5.0,
                # One ground marker is enough, and the gimbal projector draws it.
                "draw_ground_grid": False,
            }],
        ),

        Node(
            package="sim_bridge",
            executable="detection_localizer",
            name="detection_localizer",
            output="screen",
            parameters=[{
                "detections_topic": "/perception/detections",
                "camera_info_topic": f"/camera/{PERCEPTION_CAMERA}/camera_info",
                "optical_frame": PERCEPTION_OPTICAL,
                "reference_frame": REFERENCE_FRAME,
                "use_rel_alt": True,
                "anchor": PERCEPTION_ANCHOR,
                # A two metre standard deviation in x and y, one metre in z.
                # These are estimates, not derived from a calibration. Raise
                # them before you fuse this with anything that trusts them.
                "covariance_diagonal": [4.0, 4.0, 1.0, 0.0, 0.0, 0.0],
                # Two centimetres of extra standard deviation per metre of
                # slant range, squared.
                "range_variance_scale": 0.0004,
                "target_height": 1.7,
                "marker_lifetime": 3.0,
                # Empty means the reference frame. Set FIDUCIAL_ENABLED=1
                # and this becomes the corrected frame.
                "output_frame": ("fiducial"
                                 if os.environ.get("FIDUCIAL_ENABLED", "0") == "1"
                                 else ""),
            }],
        ),

        # -------------------------------------------- ground truth and scoring
        # Simulation only. These two exist to measure the detector, not to fly
        # the aircraft. Drop them from a stack you intend to run on hardware.
        Node(
            package="sim_bridge",
            executable="ground_truth",
            name="ground_truth",
            output="screen",
            parameters=[{
                "scenario_file": os.environ.get(
                    "GROUND_TRUTH_FILE",
                    "/scenes/scenarios/urban_casualties.yaml"),
                "reference_frame": REFERENCE_FRAME,
                # Scenario poses are Gazebo world coordinates and PX4's local
                # frame starts where the vehicle spawned, which is the same
                # point. Shift this if you spawn somewhere else.
                "origin_offset_xyz": [0.0, 0.0, 0.0],
                "target_height": 1.7,
                "rate_hz": 1.0,
            }],
        ),

        # Frame alignment against a surveyed point. Off until you have one.
        # Two GPS-derived frames disagree by metres, and this is where that gets
        # removed. Do not survey a target you also score against.
        Node(
            package="sim_bridge",
            executable="fiducial_alignment",
            name="fiducial_alignment",
            output="screen",
            parameters=[{
                "enabled": os.environ.get("FIDUCIAL_ENABLED", "0") == "1",
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

        Node(
            package="sim_bridge",
            executable="detection_scorer",
            name="detection_scorer",
            output="screen",
            parameters=[{
                # Eight metres is generous. It is wide enough that a correct
                # detection is never called a miss on a bad frame, and narrow
                # enough that two targets cannot be confused: the closest pair
                # in the shipped scenario is further apart than that.
                "gate_radius": 8.0,
                "window": 100,
                "reference_frame": REFERENCE_FRAME,
                "footprint_topic": f"/camera/{PERCEPTION_CAMERA}/footprint",
                # Only score targets the camera can actually see.
                "require_in_footprint": True,
            }],
        ),

        # ------------------------------------------------- observability
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                # The 3D panel needs the transforms, and asset serving lets it
                # fetch anything a future URDF refers to.
                "asset_uri_allowlist": ["^package://(?!.*\\.\\.).*"],
            }],
        ),
    ])
