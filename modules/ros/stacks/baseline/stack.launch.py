"""The baseline stack.

Every stack in this directory must have a stack.launch.py at its root. The ros
container builds the directory with colcon and launches this file. That is the
whole contract, so a stack can hold any packages it likes.

This one brings up the drone interface and nothing else:

  mavros              MAVLink, from the hub. Telemetry, commands, missions.
  rtsp_camera x2      the gimbal and nadir streams, as ROS images.
  detections_bridge   DeepStream detections, as vision_msgs.
  foxglove_bridge     a websocket for the browser, on port 8765.

It flies nothing. It is the floor that a real autonomy stack builds on, and it
is the thing to run when you want to know whether the plumbing works.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

FCU_URL = os.environ.get("FCU_URL", "udp://:14555@mavlink-hub:14551")
RTSP_BASE = os.environ.get("RTSP_BASE", "rtsp://video-router:8554")
MQTT_HOST = os.environ.get("MQTT_HOST", "message-bus")


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


def camera(name: str, hfov: float, ns: str) -> Node:
    # The namespace carries the camera name. A node name does not affect topic
    # names, so two cameras in the same namespace would both publish to
    # <ns>/image_raw and one would silently overwrite the other.
    return Node(
        package="sim_bridge",
        executable="rtsp_camera",
        name=f"{name}_camera",
        namespace=f"{ns}/{name}",
        output="screen",
        parameters=[{
            "url": f"{RTSP_BASE}/{name}",
            "frame_id": f"{name}_optical_frame",
            "latency_ms": 100,
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
                # Ours goes last, so it wins over the upstream defaults.
                os.path.join(os.path.dirname(__file__), "sim_bridge", "config",
                             "mavros_overrides.yaml"),
            ],
        ),

        # -------------------------------------------------------- video
        # 2.0 rad matches the gimbal camera, 1.74 rad the nadir camera.
        # Both values come from the sensor definitions in x500_recon/model.sdf.
        camera("gimbal", 2.0, "camera"),
        camera("nadir", 1.74, "camera"),

        # --------------------------------------------------- detections
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
                "frame_id": "gimbal_optical_frame",
            }],
        ),

        # ------------------------------------------------- observability
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{"port": 8765, "address": "0.0.0.0"}],
        ),
    ])
