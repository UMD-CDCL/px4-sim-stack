# The ground station, as an image.
#
# One container for the whole fleet. It is the mirror of modules/onboard: the
# same ROS 2 Humble base, the same workspace, MAVROS from apt. It runs a
# listening MAVROS, a frame tree and a preview ds_node for each vehicle, and
# one Foxglove bridge for the operator.
#
# DeepStream is here for ds_node alone, which runs in preview mode. The ground
# carries no detector and no model artifacts. The base still has to be
# DeepStream, because ds_node reads the stream through it, and it still has to
# be Ubuntu 22.04: every container that carries cdcl_umd_msgs must run ROS 2
# Humble, and Jazzy adds a field to sensor_msgs/Range, which sits inside
# TargetBoxArray before the box array, so a mixed pair decodes an empty box
# list and reports no error. The guard below stops a wrong base here, where the
# message is readable.
#
#   docker compose build offboard                     # DeepStream 7.1
#   DS_VERSION=7.0 docker compose build offboard      # DeepStream 7.0
ARG DS_VERSION=7.1
ARG DS_IMAGE_FLAVOR=samples-multiarch

FROM nvcr.io/nvidia/deepstream:${DS_VERSION}-${DS_IMAGE_FLAVOR}

# An ARG before FROM reaches FROM only. Declare it again to use it below.
ARG DS_VERSION

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all
# GIO asks a proxy resolver which proxy to use for every connection it opens.
# libproxy in this base image faults while it unwinds, which kills the process
# with SIGSEGV and no message. rtspsrc opens such a connection, so every RTSP
# source died at startup. The dummy resolver skips libproxy.
ENV GIO_USE_PROXY_RESOLVER=dummy

ARG USERNAME=user
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG WORKSPACE_DIR=/home/user/ros2_ws
# px4_msgs is not released to apt, and models/vehicle.py and models/sensor.py in
# MAVInsight import it at module level.
ARG PX4_MSGS_REF=main
# Overrides the per-release pyds wheel resolved below.
ARG PYDS_WHEEL_URL=

RUN . /etc/os-release; \
    if [ "$VERSION_CODENAME" != "jammy" ]; then \
        echo "DeepStream ${DS_VERSION} gives an Ubuntu $VERSION_ID base." >&2; \
        echo "ROS 2 Humble needs 22.04 (jammy). See docs/uas-contract.md section 7." >&2; \
        exit 1; \
    fi

# ---------------------------------------------------------------------------
# ROS 2 Humble. The list is the onboard list plus what only the ground runs:
# foxglove_bridge, which is the operator interface, and rqt, which USPI is a
# plugin of. rosdep is not used: umd_uas declares roboscout_uas_extras, which
# is not in this workspace, and rosdep stops on the unknown key.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg2 \
      locales \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      geographiclib-tools \
      git \
      iproute2 \
      iputils-ping \
      nano \
      python3-colcon-common-extensions \
      python3-gi \
      python3-gst-1.0 \
      python3-matplotlib \
      python3-opencv \
      python3-pandas \
      python3-pil \
      python3-pip \
      python3-scipy \
      python3-serial \
      python3-yaml \
      ros-humble-cv-bridge \
      ros-humble-diagnostic-msgs \
      ros-humble-domain-bridge \
      ros-humble-foxglove-bridge \
      ros-humble-foxglove-msgs \
      ros-humble-image-geometry \
      ros-humble-image-transport \
      ros-humble-mavros \
      ros-humble-mavros-extras \
      ros-humble-message-filters \
      ros-humble-nav-msgs \
      ros-humble-rosbag2-py \
      ros-humble-rosidl-runtime-py \
      ros-humble-ros-base \
      ros-humble-rqt-gui \
      ros-humble-rqt-gui-py \
      ros-humble-tf2-geometry-msgs \
      ros-humble-tf2-ros \
      ros-humble-vision-msgs \
      ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# The DeepStream image records ffmpeg and codec packages as installed and ships
# none of their shared objects. That leaves cv2 unimportable, which takes
# image_geometry with it, and so every node that localizes. Reinstall whatever
# the OpenCV module cannot find, repeatedly, because each restored library
# names further ones. The import at the end fails the build if it is still
# broken, rather than leaving a vehicle that starts and localizes nothing.
RUN set -eu; \
    apt-get update; \
    while missing=$(ldd /usr/lib/python3/dist-packages/cv2*.so \
                    | awk '/not found/ {print $1}' | sort -u); [ -n "$missing" ]; do \
      packages=$(for lib in $missing; do dpkg -S "$lib" | cut -d: -f1; done | sort -u); \
      apt-get install -y --reinstall $packages; \
    done; \
    rm -rf /var/lib/apt/lists/*; \
    python3 -c "import cv2"


# MAVROS refuses to convert between AMSL and ellipsoid height without these.
RUN /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# pymap3d is what the footprint, the scoring and MAVInsight convert LLA with.
# numpy stays below 2, because the Humble cv_bridge and the apt scipy build
# against numpy 1.
RUN python3 -m pip install --no-cache-dir \
      "numpy==1.24.4" \
      "pymap3d"

# ---------------------------------------------------------------------------
# DeepStream Python bindings. NVIDIA locks each wheel's ABI tag to the
# interpreter of the matching DeepStream image, so the wheel and the base go
# together: 7.1 gives Python 3.10 and pyds 1.2.0. ds_node imports pyds for the
# stream timestamps, which a preview reader needs as much as a detector does.
# ---------------------------------------------------------------------------
RUN set -eux; \
    ARCH="$(uname -m)"; \
    PYDS_URL="${PYDS_WHEEL_URL}"; \
    if [ -z "${PYDS_URL}" ]; then \
        REL_BASE="https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download"; \
        case "${DS_VERSION}" in \
            7.1*) PYDS_URL="${REL_BASE}/v1.2.0/pyds-1.2.0-cp310-cp310-linux_${ARCH}.whl" ;; \
            7.0*) PYDS_URL="${REL_BASE}/v1.1.11/pyds-1.1.11-py3-none-linux_${ARCH}.whl" ;; \
            *)    echo "No pyds wheel is known for DeepStream ${DS_VERSION}. Pass PYDS_WHEEL_URL." >&2; exit 1 ;; \
        esac; \
    fi; \
    pip3 install --no-cache-dir "${PYDS_URL}"; \
    python3 -c "import importlib.util, sys; \
spec = importlib.util.find_spec('pyds'); \
sys.exit('pyds did not install') if spec is None else print('pyds installed:', spec.origin)"

# The check uses find_spec and not an import. pyds links against libcuda.so.1,
# which the NVIDIA container runtime injects at run time, so a real import
# always fails during a build even when the install is correct.

RUN set -eux; \
    groupadd -f -g "${HOST_GID}" "${USERNAME}"; \
    useradd -m -d /home/user -u "${HOST_UID}" -g "${HOST_GID}" -s /bin/bash "${USERNAME}"; \
    mkdir -p "${WORKSPACE_DIR}/src" /logs /terrain; \
    chown -R "${HOST_UID}:${HOST_GID}" /home/user "${WORKSPACE_DIR}" /logs /terrain

# ---------------------------------------------------------------------------
# The workspace. `ros2_ws` is a named build context, so the sources stay where
# they are checked out and no copy of them lives in this repository.
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch "${PX4_MSGS_REF}" \
      https://github.com/PX4/px4_msgs.git "${WORKSPACE_DIR}/src/px4_msgs" \
    && rm -rf "${WORKSPACE_DIR}/src/px4_msgs/.git"

COPY --from=ros2_ws --chown=${HOST_UID}:${HOST_GID} src ${WORKSPACE_DIR}/src

RUN chown -R "${HOST_UID}:${HOST_GID}" "${WORKSPACE_DIR}"

USER ${USERNAME}
WORKDIR ${WORKSPACE_DIR}

RUN . /opt/ros/humble/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

USER root
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
EXPOSE 8765/tcp
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
