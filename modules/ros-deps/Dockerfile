# Dependency preparation only: ./px4sim deps (requires network on cache misses).
# Ordinary source builds derive from the resulting local image.
ARG DS_VERSION=7.1
# What this build is tagged as. Not read here; declared so the one set of
# build arguments serves this image and the two derived from it.
ARG DS_TAG=
ARG DS_IMAGE_FLAVOR=samples-multiarch
ARG DS_IMAGE=nvcr.io/nvidia/deepstream:${DS_VERSION}-${DS_IMAGE_FLAVOR}

# ---------------------------------------------------------------------------
# The DeepStream Python bindings, on their own, so the compilers and headers
# one release needs do not end up in the image that flies.
#
# NVIDIA locks each wheel's ABI tag to the interpreter of the matching
# DeepStream image, so the wheel and the base go together: 7.1 gives Python
# 3.10 and pyds 1.2.0, 8.0 gives Python 3.12 and pyds 1.2.2. From DeepStream
# 9.0 NVIDIA publishes no wheel at all -- the bindings are deprecated in favour
# of PyServiceMaker -- so that one is compiled here from the sources that ship
# for it. Same base image, so the interpreter and the DeepStream libraries the
# bindings link against are the ones that will load them.
# ---------------------------------------------------------------------------
FROM ${DS_IMAGE} AS pyds

ARG DS_VERSION
# Overrides the per-release wheel resolved below.
ARG PYDS_WHEEL_URL=
ARG PYDS_SOURCE_REF=master

ENV DEBIAN_FRONTEND=noninteractive

RUN set -eux; \
    ARCH="$(uname -m)"; \
    REL_BASE="https://github.com/NVIDIA-AI-IOT/deepstream_python_apps/releases/download"; \
    PYDS_URL="${PYDS_WHEEL_URL}"; \
    if [ -z "${PYDS_URL}" ]; then \
        case "${DS_VERSION}" in \
            9.0*) PYDS_URL=build-from-source ;; \
            8.0*) PYDS_URL="${REL_BASE}/v1.2.2/pyds-1.2.2-cp312-cp312-linux_${ARCH}.whl" ;; \
            7.1*) PYDS_URL="${REL_BASE}/v1.2.0/pyds-1.2.0-cp310-cp310-linux_${ARCH}.whl" ;; \
            *) echo "No pyds wheel is known for DeepStream ${DS_VERSION}." >&2; \
               echo "Pass PYDS_WHEEL_URL, or add the release to scripts/ds-select.sh." >&2; \
               exit 1 ;; \
        esac; \
    fi; \
    mkdir -p /wheels; \
    if [ "${PYDS_URL}" = build-from-source ]; then \
        apt-get update; \
        apt-get install -y --no-install-recommends \
            autoconf automake build-essential cmake g++ git libcairo2-dev \
            libgirepository1.0-dev libglib2.0-dev libglib2.0-dev-bin \
            libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libtool m4 \
            ninja-build pkg-config python3-build python3-dev python3-gi \
            python3-gst-1.0 python3-pip python3-setuptools python3-venv \
            python3-wheel; \
        git clone --depth 1 --branch "${PYDS_SOURCE_REF}" \
            https://github.com/NVIDIA-AI-IOT/deepstream_python_apps.git /tmp/pyds-src; \
        git -C /tmp/pyds-src submodule update --init --depth 1; \
        cd /tmp/pyds-src/bindings; \
        CMAKE_BUILD_PARALLEL_LEVEL="$(nproc)" python3 -m build --wheel; \
        cp dist/pyds-*.whl /wheels/; \
    else \
        apt-get update; \
        apt-get install -y --no-install-recommends ca-certificates curl; \
        curl -fsSL -o "/wheels/$(basename "${PYDS_URL}")" "${PYDS_URL}"; \
    fi; \
    ls -la /wheels

FROM ${DS_IMAGE}

# An ARG before FROM reaches FROM only. Declare them again to use them below.
ARG DS_VERSION
# The ROS 2 distribution that belongs to this base, and the Ubuntu that carries
# both. scripts/ds-select.sh decides; the check below refuses a pair that does
# not match the image that actually arrived.
ARG ROS_DISTRO=humble
# The live ROS 2 Humble repo lost ros-humble-mavros and ros-humble-mavros-extras
# after this date: only mavros-msgs advanced, to 2.15.1. This snapshot carries
# all three at 2.14.0, the versions the aircraft runs.
ARG ROS_SNAPSHOT=2026-08-07
ARG DS_CODENAME=jammy

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
# What is inside, for anything that has to tell one build from another at run
# time. The entry point compares it against the parser it finds on the model
# volume, which belongs to one release.
ENV DS_RELEASE=${DS_VERSION}
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
# Pinned upstream ROS 2 storage plugin. The ROS Humble apt index ships the
# SQLite plugin but not MCAP, so this is built once in the prepared dependency
# image and inherited by ros-base, onboard, and offboard.
ARG MCAP_STORAGE_REF=c1c2159679921a4f4392f370172c28e7b3b01452


# ---------------------------------------------------------------------------
# Base check. Fail here, where the message is readable.
#
# The build arguments say which release and which distribution this is meant to
# be. If the image that arrived disagrees with them, everything below installs
# the wrong ROS packages against the wrong libc and the first symptom is a node
# that will not start.
# ---------------------------------------------------------------------------
RUN set -eu; \
    . /etc/os-release; \
    if [ "$VERSION_CODENAME" != "${DS_CODENAME}" ]; then \
        echo "This base is Ubuntu $VERSION_ID ($VERSION_CODENAME), and DeepStream" >&2; \
        echo "${DS_VERSION} is recorded as ${DS_CODENAME}. Fix the table in" >&2; \
        echo "scripts/ds-select.sh, which is where that pair is decided." >&2; \
        exit 1; \
    fi; \
    have=$(sed -n 's/^Version: *//p' /opt/nvidia/deepstream/deepstream/version 2>/dev/null | head -1); \
    case "${have}" in \
        "${DS_VERSION}"*) : ;; \
        "") echo "No DeepStream version file in this base. Is DS_IMAGE a DeepStream image?" >&2; exit 1 ;; \
        *) echo "This base carries DeepStream ${have}, and the build was told ${DS_VERSION}." >&2; \
           echo "The pyds bindings are picked from that number, so they would not load." >&2; \
           exit 1 ;; \
    esac; \
    echo "DeepStream ${DS_VERSION} on Ubuntu $VERSION_ID, building for ROS 2 ${ROS_DISTRO}."

# ---------------------------------------------------------------------------
# ROS 2 and the packages the workspace links against.
#
# The list comes from the package.xml files of umd_uas and cdcl_umd_msgs, plus
# the imports in MAVInsight, whose package.xml declares no dependency at all.
# rosdep is not used: umd_uas declares roboscout_uas_extras, which is not in
# this workspace, and rosdep stops on the unknown key.
# ---------------------------------------------------------------------------
# snapshots.ros.org is signed by "ROS Snapshot builder <rosbuild@ros.org>"
# (AD19BAB3CBF125EA), which rosdistro's ros.key does not carry. Vendored so the
# build does not depend on a keyserver.
COPY ros-snapshot.key /tmp/ros-snapshot.key
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg2 \
      locales \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && gpg --dearmor < /tmp/ros-snapshot.key \
         >> /usr/share/keyrings/ros-archive-keyring.gpg \
    && . /etc/os-release \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://snapshots.ros.org/humble/${ROS_SNAPSHOT}/ubuntu $VERSION_CODENAME main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      cmake \
      geographiclib-tools \
      libgeographic-dev \
      git \
      libasio-dev \
      iproute2 \
      iputils-ping \
      nano \
      python3-colcon-common-extensions \
      python3-gi \
      python3-gst-1.0 \
      gstreamer1.0-tools \
      python3-matplotlib \
      python3-opencv \
      python3-pandas \
      python3-pil \
      python3-pip \
      python3-scipy \
      python3-serial \
      python3-yaml \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-domain-bridge \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-foxglove-msgs \
      ros-${ROS_DISTRO}-eigen-stl-containers \
      ros-${ROS_DISTRO}-image-geometry \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-mavlink \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-rosbag2-py \
      ros-${ROS_DISTRO}-zstd-vendor \
      ros-${ROS_DISTRO}-rqt-gui \
      ros-${ROS_DISTRO}-rqt-gui-py \
      ros-${ROS_DISTRO}-rosidl-runtime-py \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-visualization-msgs \
    && geographiclib-get-geoids egm96-5 \
    && rm -rf /var/lib/apt/lists/*

# ROS Humble has no binary rosbag2 MCAP plugin in the configured repository.
# Build the pinned upstream plugin here, where network access is already part
# of the explicit `px4sim prepare` operation; all derived runtime images stay
# offline and reuse the installed result.
RUN set -eux; \
    mkdir -p /tmp/rosbag2_storage_mcap; \
    git clone --depth 1 https://github.com/ros-tooling/rosbag2_storage_mcap.git \
      /tmp/rosbag2_storage_mcap; \
    git -C /tmp/rosbag2_storage_mcap fetch --depth 1 origin "${MCAP_STORAGE_REF}"; \
    git -C /tmp/rosbag2_storage_mcap checkout --detach "${MCAP_STORAGE_REF}"; \
    # Humble's setup scripts read optional variables under nounset. Source the
    # environment with nounset briefly disabled, then restore strict mode for
    # the actual build and cleanup.
    set +u; . /opt/ros/${ROS_DISTRO}/setup.sh; set -u; \
    colcon build --merge-install \
      --base-paths /tmp/rosbag2_storage_mcap \
      --install-base /opt/ros/${ROS_DISTRO} \
      --build-base /tmp/rosbag2_storage_mcap-build \
      --packages-select mcap_vendor rosbag2_storage_mcap_testdata \
      --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    set +u; . /opt/ros/${ROS_DISTRO}/setup.sh; set -u; \
    colcon build --merge-install \
      --base-paths /tmp/rosbag2_storage_mcap \
      --install-base /opt/ros/${ROS_DISTRO} \
      --build-base /tmp/rosbag2_storage_mcap-build \
      --packages-select rosbag2_storage_mcap \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF; \
    rm -rf /tmp/rosbag2_storage_mcap /tmp/rosbag2_storage_mcap-build \
      /root/.colcon

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

# The same stripping takes the GStreamer software codecs. The image records
# gstreamer1.0-plugins-good, -bad and -ugly as installed with their plugin
# objects removed, ships no gstreamer1.0-libav at all, and strips the codec
# libraries those plugins load in the same way, so a plain install restores
# nothing and a reinstalled plugin still fails to load. On a GPU with video
# engines nothing ever misses them; on a machine without (T500-class laptop
# GPUs: CUDA intact, NVENC and NVDEC fused off) they are the whole video
# path -- ds_node decodes with avdec, records with x264enc and previews
# with jpegenc, while inference stays on the GPU. Reinstall the plugins,
# then whatever the restored objects cannot find, repeatedly, exactly as
# for OpenCV above. The inspects at the end fail the build while it can
# still say why, rather than leaving a companion that starts and shows
# nothing.
RUN set -eu; \
    apt-get update; \
    apt-get install -y --reinstall \
      gstreamer1.0-libav \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly; \
    plugins="/usr/lib/$(uname -m)-linux-gnu/gstreamer-1.0"; \
    while missing=$(ldd "$plugins/libgstlibav.so" "$plugins/libgstx264.so" \
                        "$plugins/libgstx265.so" "$plugins/libgstjpeg.so" \
                    | awk '/not found/ {print $1}' | sort -u); [ -n "$missing" ]; do \
      packages=$(for lib in $missing; do dpkg -S "$lib" | cut -d: -f1; done | sort -u); \
      apt-get install -y --reinstall $packages; \
    done; \
    rm -rf /var/lib/apt/lists/*; \
    for element in avdec_h264 avdec_h265 x264enc x265enc jpegenc; do \
      gst-inspect-1.0 "$element" >/dev/null; \
    done


# The TensorRT this machine needs, when the one in the base is not it.
#
# Two cases. A GPU newer than Hopper: DeepStream 7.1 ships TensorRT 10.3,
# which stops at Hopper. On a newer card it parses the ONNX, reports
# `Unsupported SM`, builds no engine and takes the process down, while the
# streams keep decoding. A newer TensorRT for jammy from the CUDA repository,
# which the x86 base already lists, fixes it. A Jetson: the generic arm64 base
# carries 10.3.0.26 and the host runs 10.3.0.30. An engine loads only under
# the build that made it, and 10.3.0.26 cannot build one on the Orin at all
# (a 5 MB tactic limit). The host's own packages fix it. They come from the
# JetPack repository, which TRT_APT_SOURCE names and this block adds.
# TRT_APT_PUBLIC_KEY is the URL of the key that signs that repository.
#
# TRT_CUDA names the CUDA runtime the x86 upgrade links. A Jetson leaves it
# empty, because its base already carries CUDA 12.6.
#
# Empty means the base's own TensorRT is right for this machine, which is the
# case for every x86 GPU up to Hopper. scripts/ds-select.sh decides all four.
ARG TRT_VERSION=
ARG TRT_CUDA=
ARG TRT_APT_SOURCE=
ARG TRT_APT_PUBLIC_KEY=
RUN if [ -n "${TRT_VERSION}" ]; then \
        set -eux; \
        apt-get update; \
        apt-get install -y --no-install-recommends ca-certificates curl; \
        if [ -n "${TRT_APT_SOURCE}" ]; then \
            curl -fsSL "${TRT_APT_PUBLIC_KEY}" -o /etc/apt/trusted.gpg.d/tensorrt-host.asc; \
            echo "${TRT_APT_SOURCE}" > /etc/apt/sources.list.d/tensorrt-host.list; \
            apt-get update; \
        fi; \
        cuda=""; \
        if [ -n "${TRT_CUDA}" ]; then cuda="cuda-cudart-${TRT_CUDA} libcublas-${TRT_CUDA}"; fi; \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer10=${TRT_VERSION}" \
            "libnvinfer-plugin10=${TRT_VERSION}" \
            "libnvonnxparsers10=${TRT_VERSION}" \
            "libnvinfer-dispatch10=${TRT_VERSION}" \
            "libnvinfer-lean10=${TRT_VERSION}" \
            "libnvinfer-vc-plugin10=${TRT_VERSION}" \
            "libnvinfer-bin=${TRT_VERSION}" \
            ${cuda}; \
        rm -rf /var/lib/apt/lists/*; \
        echo "TensorRT is now $(dpkg -l libnvinfer10 | awk '/^ii/ {print $3}')"; \
    fi

# Ubuntu 24.04 marks the system interpreter externally managed, so every pip
# install below refuses unless it is told the distribution's packages may move.
# There is no virtual environment here on purpose: this image is the runtime,
# and ROS 2 loads its own Python packages from the system path. Written as
# configuration rather than a flag on each call so a new pip line cannot forget
# it. Ubuntu 22.04 ships a pip that has no such option, and gets no file.
RUN if python3 -m pip install --help 2>/dev/null | grep -q -- --break-system-packages; then \
        printf '[install]\nbreak-system-packages = true\n' > /etc/pip.conf; \
    fi

# ---------------------------------------------------------------------------

# pymap3d is what tf_loc and MAVInsight convert LLA with. folium is the mosaic
# map export.
#
# numpy stays below 2 on every base. pyds does not support numpy 2 at all, and
# each distribution's cv_bridge and apt scipy are built against the numpy that
# ships with it: 1.24 on 22.04, 1.26 on 24.04. So the pin is a ceiling here and
# an exact version there, and neither one moves the distribution's own numpy.
RUN python3 -m pip install --no-cache-dir \
      "$([ "${ROS_DISTRO}" = humble ] && echo 'numpy==1.24.4' || echo 'numpy<2')" \
      "pymap3d" \
      "folium" \
      "utm" \
      "onnxruntime==1.18.1" \
      "tensorrt==10.3.0"

# The bindings from the first stage. Installing the wheel, rather than copying
# pyds*.so into site-packages, makes a wrong artifact fail here instead of at
# the first import.
COPY --from=pyds /wheels /tmp/wheels
RUN set -eux; \
    pip3 install --no-cache-dir /tmp/wheels/pyds-*.whl; \
    rm -rf /tmp/wheels; \
    python3 -c "import importlib.util, sys; \
spec = importlib.util.find_spec('pyds'); \
sys.exit('pyds did not install') if spec is None else print('pyds installed:', spec.origin)"

# The check uses find_spec and not an import. pyds links against libcuda.so.1,
# which the NVIDIA container runtime injects at run time, so a real import
# always fails during a build even when the install is correct.

# ---------------------------------------------------------------------------
# A user whose ids match the host, so a bind mount stays writable from both
# sides. The home is /home/user because onboard_common_params.yaml points
# model.dir and model.custom_lib_path at /home/user/ros2_ws.
#
# Ubuntu 24.04 already has a uid 1000 account named `ubuntu`, and it holds the
# id the host user wants. Remove it first, or useradd fails the build on a
# machine whose host uid is the usual 1000.
# ---------------------------------------------------------------------------
RUN set -eux; \
    if existing=$(getent passwd "${HOST_UID}" | cut -d: -f1) && [ -n "${existing}" ]; then \
        userdel -r "${existing}" 2>/dev/null || userdel "${existing}"; \
    fi; \
    groupadd -f -g "${HOST_GID}" "${USERNAME}"; \
    useradd -m -d /home/user -u "${HOST_UID}" -g "${HOST_GID}" -s /bin/bash "${USERNAME}"; \
    mkdir -p "${WORKSPACE_DIR}/src" /models /logs /terrain /camera; \
    chown -R "${HOST_UID}:${HOST_GID}" /home/user "${WORKSPACE_DIR}" /models /logs /terrain /camera

# What verify/component/foxglove_probe.py speaks. It reads the Foxglove bridge
# over the same protocol the panels do, which is the only way to see a layout
# the way an operator's Foxglove sees it. Its own layer, above everything slow
# and below the workspace, so an edit to a node never pays for it.
# From pip rather than apt: jammy ships websockets 9.1, which passes a loop to
# asyncio.Lock and so cannot run on the Python 3.10 beside it.
RUN python3 -m pip install --no-cache-dir "websockets==12.0"

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
      patch python3-colcon-override-check ccache \
    && rm -rf /var/lib/apt/lists/*
