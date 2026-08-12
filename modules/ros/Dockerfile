# The autonomy module: ROS 2 Jazzy.
#
# What this image deliberately does NOT contain:
#   - Gazebo, or ros_gz. The stack sees video and MAVLink, not the simulator.
#     Leaving ros_gz out also avoids the version clash between the ROS-vendored
#     Gazebo packages and the upstream Gazebo in the sim image.
#   - px4_msgs or the uXRCE-DDS client. MAVROS is the drone interface. Turn on
#     the xrce profile if a stack you want to run needs the uORB topics.
#
# ROS 2 Jazzy is the long term release for Ubuntu 24.04, supported to May 2029.
# Kilted and Lyrical are newer. Jazzy has the binary packages this stack needs,
# which Lyrical does not yet have for every one of them.

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-mavros-msgs \
      ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-foxglove-msgs \
      ros-${ROS_DISTRO}-geographic-msgs \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-camera-info-manager \
      ros-${ROS_DISTRO}-gscam \
      python3-colcon-common-extensions \
      python3-gi python3-gi-cairo gir1.2-glib-2.0 \
      gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
      gstreamer1.0-tools \
      python3-paho-mqtt \
      libgl1 libglx0 libegl1 libglvnd0 \
      curl vim less iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# MAVROS needs the GeographicLib geoid model to convert AMSL to ellipsoid
# height. Without it, every mavros_node start logs a fatal error and quits.
#
# MAVROS ships install_geographiclib_datasets.sh for this. That script refuses
# to run here, with "This script require root privileges", even though the build
# is root. Calling the tool directly does the same work and does not depend on
# the script surviving the next MAVROS release.
RUN apt-get update && apt-get install -y --no-install-recommends \
      geographiclib-tools \
    && geographiclib-get-geoids egm96-5 \
    && rm -rf /var/lib/apt/lists/*

RUN if getent passwd ${UID} >/dev/null; then \
      userdel -rf "$(getent passwd ${UID} | cut -d: -f1)" || true; \
    fi \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev \
    && mkdir -p /ws/src && chown -R ${UID}:${GID} /ws

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && printf '%s\n' \
      'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      > /etc/profile.d/ros.sh \
    && cat /etc/profile.d/ros.sh >> /home/dev/.bashrc \
    && chown ${UID}:${GID} /home/dev/.bashrc

USER dev
WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
