from setuptools import find_packages, setup

package_name = "sim_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="px4-sim-stack",
    maintainer_email="ctitus@umd.edu",
    description="RTSP video and MQTT detections as ROS 2 topics.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "rtsp_camera = sim_bridge.rtsp_camera:main",
            "detections_bridge = sim_bridge.detections_bridge:main",
        ],
    },
)
