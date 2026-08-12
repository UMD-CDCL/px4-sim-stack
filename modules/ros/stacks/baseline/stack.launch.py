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
  ground_projector    what each camera covers on the ground.
  detection_localizer detections placed on the ground, with covariance.
  detection_scorer    those positions against ground truth.
  foxglove_bridge     a websocket for the browser, on port 8765.

One pipeline for each camera
----------------------------
DeepStream runs one pipeline for each camera, and every stage after it runs
once for each camera too: an annotator, a ground projector, a localizer and a
scorer. Nothing merges the two.

That is deliberate. The nadir camera looks straight down over a small patch and
localizes it well; the gimbal looks out and gets worse as it tilts toward the
horizon. One combined recall figure would average those into a number that
describes neither, so each camera keeps its own topics under
/perception/<camera>/ and /scoring/<camera>/, and the layout shows both.

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
def anchor_for(camera: str) -> str:
    return "centre" if camera == "nadir" else "bottom"


def hfov_for(camera: str) -> float:
    return NADIR_HFOV if camera == "nadir" else GIMBAL_HFOV


def optical_for(camera: str) -> str:
    return CAMERA_OPTICAL.get(camera, GIMBAL_OPTICAL)


PERCEPTION_ANCHOR = anchor_for(PERCEPTION_CAMERA)

# The second camera DeepStream runs. Each has its own pipeline, and the
# payload's sensorId says which one a detection came from.
PERCEPTION_CAMERA_2 = os.environ.get("PERCEPTION_CAMERA_2", "gimbal")
CAMERA_OPTICAL = {"gimbal": GIMBAL_OPTICAL, "nadir": NADIR_OPTICAL}

# Every per-camera node is built from this list. Add a third camera here and to
# the DeepStream source list, and the rest follows.
CAMERAS = [PERCEPTION_CAMERA, PERCEPTION_CAMERA_2]

# Field of view of each camera, in radians, from the sensor definitions in
# modules/sim/scenes/models/x500_recon/model.sdf. rtsp_camera builds its
# CameraInfo from these, and every projection depends on them being right.
# Horizontal field of view in radians, from the sensor definitions in
# modules/sim/scenes/models/x500_recon/model.sdf. Both are overridable, so the
# nadir camera can be trimmed without touching the gimbal, whose projection is
# already correct.
GIMBAL_HFOV = float(os.environ.get("GIMBAL_HFOV", "2.0"))
NADIR_HFOV = float(os.environ.get("NADIR_HFOV", "1.74"))

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
                "gimbal_compose": os.environ.get("GIMBAL_COMPOSE", "left"),
                # Any constant left after the frame handling. The scene_tf
                # diagnostic prints the number to put here: with the gimbal
                # centred, "gimbal rel body" should read near zero.
                "gimbal_offset_rpy_deg": [
                    float(os.environ.get("GIMBAL_OFFSET_ROLL", "0.0")),
                    float(os.environ.get("GIMBAL_OFFSET_PITCH", "0.0")),
                    float(os.environ.get("GIMBAL_OFFSET_YAW", "0.0")),
                ],
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
                # The coordinate space DeepStream reports boxes in, which is
                # [tiled-display] in its config, not [streammux]. Compose sets
                # both from DS_WIDTH and DS_HEIGHT. The bridge scales into the
                # live image size, so if these ever disagree the boxes still
                # land in the right place and a warning says so.
                "source_width": int(os.environ.get("DS_WIDTH", "1920")),
                "source_height": int(os.environ.get("DS_HEIGHT", "1080")),
                # Per camera corrections, as "camera=WxH", comma separated.
                # Each camera now has its own pipeline at its own resolution,
                # so the boxes should already match the image and this should
                # stay empty. It is kept because the failure it corrects is
                # silent, and one env var beats a code change at 2 am.
                "source_size_overrides": [
                    part.strip() for part in
                    os.environ.get("DS_COORD_OVERRIDES", "").split(",")
                ] or [""],
                "frame_id": PERCEPTION_OPTICAL,
                "sensor_ids": [PERCEPTION_CAMERA, PERCEPTION_CAMERA_2],
                "sensor_frames": [
                    CAMERA_OPTICAL.get(PERCEPTION_CAMERA, PERCEPTION_OPTICAL),
                    CAMERA_OPTICAL.get(PERCEPTION_CAMERA_2, GIMBAL_OPTICAL),
                ],
                # DeepStream's frame time trails true capture by about 16 ms on
                # this hardware. Measure yours with scripts/measure-latency.py
                # and put the difference here if it matters to you.
                "time_offset": 0.0,
                "max_age": 2.0,
            }],
        ),

        # ------------------------------------------- boxes on the video
        # One annotator for each camera, so the Image panels show the same
        # boxes the detector produced.
        *[
            Node(
                package="sim_bridge",
                executable="detection_annotator",
                name=f"{cam}_annotator",
                namespace=f"camera/{cam}",
                output="screen",
                parameters=[{
                    "detections_topic": f"/perception/{cam}/detections",
                    "annotations_topic": "annotations",
                    # This camera's own verdicts. Reading the other camera's
                    # would colour these boxes by what a different lens found.
                    "verdicts_topic": f"/scoring/{cam}/verdicts",
                    "line_thickness": 2.0,
                    "text_size": 14.0,
                }],
            )
            for cam in CAMERAS
        ],

        # ------------------------------------------------------- ground
        # What each camera covers on the ground, as a polygon and a boresight.
        *[
            Node(
                package="sim_bridge",
                executable="ground_projector",
                name=f"{cam}_ground_projector",
                namespace=f"camera/{cam}",
                output="screen",
                parameters=[{
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": optical_for(cam),
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    "ground_z": 0.0,
                    "rate_hz": 5.0,
                    # One ground marker is enough, so only the first draws it.
                    "draw_ground_grid": i == 0,
                }],
            )
            for i, cam in enumerate(CAMERAS)
        ],

        # One localizer for each camera, each in its own namespace, so the
        # topics come out as /perception/<camera>/detections_3d and nothing
        # overwrites anything.
        *[
            Node(
                package="sim_bridge",
                executable="detection_localizer",
                name=f"{cam}_localizer",
                namespace=f"perception/{cam}",
                output="screen",
                parameters=[{
                    "camera": cam,
                    "detections_topic": f"/perception/{cam}/detections",
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": optical_for(cam),
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    "anchor": anchor_for(cam),
                    # A two metre standard deviation in x and y, one metre in z.
                    # These are estimates, not derived from a calibration. Raise
                    # them before you fuse this with anything that trusts them.
                    "covariance_diagonal": [4.0, 4.0, 1.0, 0.0, 0.0, 0.0],
                    # Two centimetres of extra standard deviation per metre of
                    # slant range, squared.
                    "range_variance_scale": 0.0004,
                    "target_height": 1.7,
                    "marker_lifetime": 3.0,
                    # Each estimate also as a NavSatFix, which is what the Map
                    # panel plots without any conversion of its own.
                    "publish_navsat": True,
                    # Empty means the reference frame. Set FIDUCIAL_ENABLED=1
                    # and this becomes the corrected frame.
                    "output_frame": ("fiducial"
                                     if os.environ.get("FIDUCIAL_ENABLED", "0") == "1"
                                     else ""),
                }],
            )
            for cam in CAMERAS
        ],

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

        # One scorer for each camera. Each judges its own estimates against the
        # targets inside its own footprint, so recall means "of what this
        # camera could see". The gate is shared, because it is a statement
        # about the task rather than about a lens.
        *[
            Node(
                package="sim_bridge",
                executable="detection_scorer",
                name=f"{cam}_scorer",
                namespace=f"scoring/{cam}",
                output="screen",
                parameters=[{
                    "camera": cam,
                    "detections_topic": f"/perception/{cam}/detections_3d",
                    # An estimate counts as finding a target only if it lands
                    # within this many metres of it. Two metres is a statement
                    # about what the localization is for: a position good enough
                    # to send someone to. A wider gate scores geometry that is
                    # not actually useful as a success.
                    "gate_radius": float(os.environ.get("SCORING_GATE_M", "2.0")),
                    "window": 100,
                    "reference_frame": REFERENCE_FRAME,
                    "footprint_topic": f"/camera/{cam}/footprint",
                    # Only score targets the camera can actually see.
                    "require_in_footprint": True,
                    "target_height": 1.7,
                    # Stack the summaries rather than let them overlap.
                    "summary_z": 30.0 + 6.0 * i,
                }],
            )
            for i, cam in enumerate(CAMERAS)
        ],

        Node(
            package="sim_bridge",
            executable="map_overlays",
            name="map_overlays",
            output="screen",
            parameters=[{
                "cameras": CAMERAS,
                "footprint_pattern": "/camera/{cam}/footprint",
                "verdict_pattern": "/scoring/{cam}/verdicts",
                "detection_pattern": "/perception/{cam}/detections_3d",
                "rate_hz": 2.0,
            }],
        ),

        # The live image laid flat on the ground plane, so the 3D view shows
        # the camera's own picture in the place the localizer thinks it is.
        *[
            Node(
                package="sim_bridge",
                executable="image_ground_projector",
                name=f"{cam}_ground_image",
                namespace=f"camera/{cam}",
                output="screen",
                parameters=[{
                    "image_topic": f"/camera/{cam}/image_raw",
                    "camera_info_topic": f"/camera/{cam}/camera_info",
                    "optical_frame": optical_for(cam),
                    "reference_frame": REFERENCE_FRAME,
                    "use_rel_alt": True,
                    # The resolution each frame is sampled down to before it is
                    # projected. Both cameras use the same grid, so this one
                    # number decides the cost for all of them.
                    #
                    # 640x360 is 230 thousand points and 3.7 MB for each
                    # camera. The full 1920x1080 grid is 2.07 million points
                    # and 33 MB, which held a core at full load and published
                    # nothing at all. Raise it if you want a sharper backdrop
                    # and have measured that the link carries it.
                    "size": os.environ.get("GROUND_IMAGE_SIZE", "640x360"),
                    "rate_hz": float(os.environ.get("GROUND_IMAGE_RATE", "1.0")),
                }],
            )
            for cam in CAMERAS
        ],

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
                # The default is 10 MB, and the ground projection clouds are
                # megabytes each. Over that limit the bridge drops the channel
                # silently: the topic still advertises and the panel stays
                # empty, which is a slow thing to diagnose. See
                # docs/troubleshooting.md.
                "send_buffer_limit": int(os.environ.get(
                    "FOXGLOVE_SEND_BUFFER", str(200 * 1024 * 1024))),
            }],
        ),
    ])
