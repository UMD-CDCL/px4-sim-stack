# Dependency preparation only: ./px4sim prepare (requires network on cache misses).
# Ordinary source builds (./px4sim restart) derive from the tagged local images.
#
# ===========================================================================
# LAYOUT AND CACHE RULES -- read before adding anything
# ===========================================================================
# One Dockerfile, four targets, built together by `px4sim prepare` so the
# shared stages are built once:
#
#   trt ═══ yolo-deps  (same TensorRT step)  -> px4simstack/yolo-deps:$DS_TAG
#    └─ system ── ros-core ─┐
#                           │                -> px4simstack/ros-core:$DS_TAG
#                           └─ ros-deps      -> px4simstack/ros-deps:$DS_TAG
#   pyds (independent, copied into ros-deps)
#
# ros-core is what MAVROS (modules/ros-base) compiles against. Nothing added
# to ros-deps can reach it, so an extra pip or apt package never recompiles
# MAVROS. A source-built dependency is its own stage copied in with
# `COPY --link`, so bumping one rebuilds that stage and nothing else.
#
# Layers are ordered by how often they change (rarest first) and weighted by
# how much they cost to redo (TensorRT/cuBLAS alone is ~10 GB of downloads):
#
#   T0 never    DeepStream base, TensorRT for this GPU           (stage trt)
#   T1 rare     system apt + stripped-codec repair               (stage system)
#   T2 rare     ROS core: what MAVROS / MCAP build against        (stage ros-core)
#   T3 rare     host user, heavy pinned pip, pyds                 (stage ros-deps)
#   T4 often    ROS / apt extras            <- ADD NEW APT PACKAGES HERE
#   T5 often    light pip packages          <- ADD NEW PIP PACKAGES HERE
#   T6 medium   source-built deps (COPY --link from their stages; none now)
#
# Rules that keep the cache intact:
#   * Declare an ARG immediately above the one RUN that reads it. Every RUN
#     after an ARG gets it as an environment variable, so a changed value
#     misses the cache from the declaration on, whether or not a RUN uses it.
#     (This is why the MCAP_STORAGE_REF bump used to reinstall ROS, TensorRT
#     and pip: the ARG sat above all of them.)
#   * Never edit a lower tier just to add a package. Add to T4/T5; fold it
#     down into T1/T2 only when you are deliberately taking a cold rebuild.
#   * Do not reword the trt RUN or the pyds stage. They are byte-for-byte the
#     former yolo-deps/ros-deps text, which is what makes them cache hits.
#
# What survives when a lower layer DOES change:
#   * Downloads. Every apt and pip step mounts a persistent BuildKit cache
#     (px4sim-apt-*, px4sim-pip-*), so a re-run layer reinstalls from local
#     .debs and wheels instead of the network. docker-clean is switched off
#     in T1 for this. The cache lives outside the image and outside layers.
#   * Sources. Fetched in stages FROM scratch, which depend on
#     nothing below them, and bind-mounted into the build that uses them.
#   * Compiler output. Source builds use a ccache mount, so a rebuilt parent
#     recompiles only what its headers actually changed.
#   These are build cache: `docker builder prune` with no filter wipes them.
#   Prune images, or prune build cache with `--filter until=...`, instead.
# ===========================================================================
ARG DS_VERSION=7.1
# What this build is tagged as. Not read here; declared so the one set of
# build arguments serves this image and the two derived from it.
ARG DS_TAG=
ARG DS_IMAGE_FLAVOR=samples-multiarch
ARG DS_IMAGE=nvcr.io/nvidia/deepstream:${DS_VERSION}-${DS_IMAGE_FLAVOR}

# ===========================================================================
# T0  trt -- the DeepStream base with the TensorRT this machine needs.
#     Changes only with DS_VERSION or the GPU (scripts/ds-select.sh).
#     Shared by yolo-deps and ros-deps, so it is downloaded once.
# ===========================================================================
FROM ${DS_IMAGE} AS trt

ARG DS_VERSION
ENV DEBIAN_FRONTEND=noninteractive

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

# ===========================================================================
# yolo-deps -- CUDA compiler and TensorRT headers for the YOLO parser,
# prepared independently of detector source. Target of the yolo-deps service.
# ===========================================================================
FROM ${DS_IMAGE} AS yolo-deps
# One stage, exactly the former modules/yolo-deps/Dockerfile: a stage that
# derives FROM trt gets a different cache key for the nvcc step even with
# identical text. Its TensorRT RUN is byte-identical to the one in `trt`, so
# BuildKit solves both as one step -- still one download, one layer.
# KEEP THE TWO TensorRT BLOCKS IDENTICAL; if they drift, both still work, but
# TensorRT is downloaded and stored twice.

ARG DS_VERSION
ENV DEBIAN_FRONTEND=noninteractive

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

ARG YOLO_CUDA_VERSION=12.6
RUN set -eux; \
    CUDA_VER="${YOLO_CUDA_VERSION}"; \
    apt-get update; \
    apt-get install -y --no-install-recommends build-essential; \
    if [ ! -x "/usr/local/cuda-${CUDA_VER}/bin/nvcc" ]; then \
        apt-get install -y --no-install-recommends \
            "cuda-nvcc-$(echo "${CUDA_VER}" | tr . -)" \
            "cuda-cudart-dev-$(echo "${CUDA_VER}" | tr . -)" || true; \
    fi; \
    if [ ! -x "/usr/local/cuda-${CUDA_VER}/bin/nvcc" ]; then \
        found=$(ls -d /usr/local/cuda-*/bin/nvcc 2>/dev/null | sort -V | tail -1); \
        [ -n "${found}" ] || { echo "No CUDA compiler here and none installable. The parser cannot be built." >&2; exit 1; }; \
        CUDA_VER=$(echo "${found}" | sed "s|.*/cuda-||; s|/bin/nvcc||"); \
        echo "Falling back to the CUDA this base already carries: ${CUDA_VER}."; \
    fi; \
    if [ -n "${TRT_VERSION}" ]; then \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer-headers-dev=${TRT_VERSION}" \
            "libnvinfer-dev=${TRT_VERSION}" \
            "libnvinfer-headers-plugin-dev=${TRT_VERSION}" \
            "libnvinfer-plugin-dev=${TRT_VERSION}" \
            "libnvonnxparsers-dev=${TRT_VERSION}"; \
    fi; \
    for lib in /usr/local/cuda-*/lib64; do \
        [ -d "$lib" ] || continue; \
        for versioned in "$lib"/lib*.so.[0-9]*; do \
            plain="${versioned%%.so.*}.so"; \
            [ -e "$plain" ] || ln -s "$(basename "$versioned")" "$plain"; \
        done; \
    done; \
    mkdir -p /opt/ds-yolo; \
    printf '%s\n' "${YOLO_CUDA_VERSION}" > /opt/ds-yolo/cuda-requested; \
    printf '%s\n' "${CUDA_VER}" > /opt/ds-yolo/cuda-version; \
    rm -rf /var/lib/apt/lists/*

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

# ===========================================================================
# T1  system -- Ubuntu packages and the repair of what the DeepStream image
#     strips. Nothing from the ROS repository. Changes rarely.
#     ADD HERE only a system library that T2 (ROS core) must build against;
#     anything else goes in T4.
# ===========================================================================
FROM trt AS system

# An ARG before FROM reaches FROM only. Declare them again to use them below.
ARG TARGETARCH
ARG DS_VERSION
# The ROS 2 distribution that belongs to this base, and the Ubuntu that carries
# both. scripts/ds-select.sh decides; the check below refuses a pair that does
# not match the image that actually arrived.
ARG ROS_DISTRO=humble
ARG DS_CODENAME=jammy

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

# Keep every .deb apt downloads. The base's docker-clean hook deletes them
# after each install, which would leave the apt cache mount below empty. The
# mount is outside the layer, so the image does not grow.
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
         > /etc/apt/apt.conf.d/keep-cache

# Keep a later `apt-get install` in T4 from ever moving the TensorRT that T0
# chose for this GPU. A no-op where the base's own TensorRT is used.
ARG TRT_VERSION=
RUN if [ -n "${TRT_VERSION}" ]; then \
        apt-mark hold libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10 \
            libnvinfer-dispatch10 libnvinfer-lean10 libnvinfer-vc-plugin10 \
            libnvinfer-bin; \
    fi

# System packages. The Python ones are here, not in T4, because the codec
# repair below is what makes python3-opencv importable, and because numpy
# comes from them.
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      ccache \
      cmake \
      curl \
      geographiclib-tools \
      git \
      gnupg2 \
      gstreamer1.0-tools \
      iproute2 \
      iputils-ping \
      libasio-dev \
      libgeographic-dev \
      locales \
      nano \
      patch \
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
    && locale-gen en_US.UTF-8 \
    && geographiclib-get-geoids egm96-5 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# The DeepStream image records ffmpeg and codec packages as installed and ships
# none of their shared objects. That leaves cv2 unimportable, which takes
# image_geometry with it, and so every node that localizes. Reinstall whatever
# the OpenCV module cannot find, repeatedly, because each restored library
# names further ones. The import at the end fails the build if it is still
# broken, rather than leaving a vehicle that starts and localizes nothing.
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    set -eu; \
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
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    set -eu; \
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

# ===========================================================================
# T2  ros-core -- ROS 2 and exactly what MAVROS and the MCAP plugin compile
#     against. Target of the ros-core service; modules/ros-base builds MAVROS
#     from this image, so anything added here recompiles MAVROS (ccache keeps
#     that short, but it is still a cold link).
#     ADD HERE only a build dependency of a source-built package (MAVROS,
#     MCAP, a new T6 stage). A runtime-only ROS package goes in T4.
# ===========================================================================
FROM system AS ros-core
ARG TARGETARCH

# The MCAP storage plugin is installed here, in the same apt run as rosbag2,
# so the two are always one release. It was once built from source (the
# archived ros-tooling v0.6.0, then rosbag2 0.15.10): v0.6.0 has no lock on
# its topic->channel map, so `ros2 bag record -a` on a graph still appearing
# died with "Channel reference not found", and 0.15.10 fixed that but
# loaded an old plugin into a newer core. The lock (rosbag2 #1561) is in
# every Humble release since 0.15.10.
# rosdep is not used: umd_uas declares roboscout_uas_extras, which is not in
# this workspace, and rosdep stops on the unknown key. The MAVROS set is its
# package.xml files (mavros, mavros_extras, libmavconn, mavros_msgs) minus
# what ros-base already carries.
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && . /etc/os-release \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
         > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-colcon-override-check \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-diagnostic-msgs \
      ros-${ROS_DISTRO}-diagnostic-updater \
      ros-${ROS_DISTRO}-eigen-stl-containers \
      ros-${ROS_DISTRO}-mavlink \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-rosbag2-py \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-rosidl-runtime-py \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-visualization-msgs \
      ros-${ROS_DISTRO}-zstd-vendor \
    && rm -rf /var/lib/apt/lists/*

# ===========================================================================
# ros-deps -- the runtime every ROS image derives from. Target of the
# ros-deps service; modules/ros-base builds the workspace FROM it.
# ===========================================================================
FROM ros-core AS ros-deps
ARG TARGETARCH

# ---------------------------------------------------------------------------
# T3  rare: host user, heavy pinned Python, DeepStream bindings.
# ---------------------------------------------------------------------------
# A user whose ids match the host, so a bind mount stays writable from both
# sides. The home is /home/user because onboard_common_params.yaml points
# model.dir and model.custom_lib_path at /home/user/ros2_ws.
#
# Ubuntu 24.04 already has a uid 1000 account named `ubuntu`, and it holds the
# id the host user wants. Remove it first, or useradd fails the build on a
# machine whose host uid is the usual 1000.
ARG USERNAME=user
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG WORKSPACE_DIR=/home/user/ros2_ws
RUN set -eux; \
    if existing=$(getent passwd "${HOST_UID}" | cut -d: -f1) && [ -n "${existing}" ]; then \
        userdel -r "${existing}" 2>/dev/null || userdel "${existing}"; \
    fi; \
    groupadd -f -g "${HOST_GID}" "${USERNAME}"; \
    useradd -m -d /home/user -u "${HOST_UID}" -g "${HOST_GID}" -s /bin/bash "${USERNAME}"; \
    mkdir -p "${WORKSPACE_DIR}/src" /models /logs /terrain /camera; \
    chown -R "${HOST_UID}:${HOST_GID}" /home/user "${WORKSPACE_DIR}" /models /logs /terrain /camera

# Ubuntu 24.04 marks the system interpreter externally managed, so every pip
# install below refuses unless it is told the distribution's packages may move.
# There is no virtual environment here on purpose: this image is the runtime,
# and ROS 2 loads its own Python packages from the system path. Written as
# configuration rather than a flag on each call so a new pip line cannot forget
# it. Ubuntu 22.04 ships a pip that has no such option, and gets no file.
RUN if python3 -m pip install --help 2>/dev/null | grep -q -- --break-system-packages; then \
        printf '[install]\nbreak-system-packages = true\n' > /etc/pip.conf; \
    fi

# The heavy, pinned wheels (~3 GB: onnxruntime and the tensorrt wheel with its
# bundled libraries). Kept apart from the light packages in T5 so adding a
# small dependency never downloads these again.
#
# numpy stays below 2 on every base. pyds does not support numpy 2 at all, and
# each distribution's cv_bridge and apt scipy are built against the numpy that
# ships with it: 1.24 on 22.04, 1.26 on 24.04. So the pin is a ceiling here and
# an exact version there, and neither one moves the distribution's own numpy.
RUN --mount=type=cache,id=px4sim-pip-${TARGETARCH},target=/root/.cache/pip \
    python3 -m pip install \
      "$([ "${ROS_DISTRO}" = humble ] && echo 'numpy==1.24.4' || echo 'numpy<2')" \
      "onnxruntime==1.18.1" \
      "tensorrt==10.3.0"

# The bindings from the pyds stage. Installing the wheel, rather than copying
# pyds*.so into site-packages, makes a wrong artifact fail here instead of at
# the first import.
#
# The check uses find_spec and not an import. pyds links against libcuda.so.1,
# which the NVIDIA container runtime injects at run time, so a real import
# always fails during a build even when the install is correct.
COPY --from=pyds /wheels /tmp/wheels
RUN --mount=type=cache,id=px4sim-pip-${TARGETARCH},target=/root/.cache/pip \
    set -eux; \
    pip3 install /tmp/wheels/pyds-*.whl; \
    rm -rf /tmp/wheels; \
    python3 -c "import importlib.util, sys; \
spec = importlib.util.find_spec('pyds'); \
sys.exit('pyds did not install') if spec is None else print('pyds installed:', spec.origin)"

# ---------------------------------------------------------------------------
# T4  often: ROS runtime packages and apt tools nothing compiles against.
#     >>> ADD NEW apt / ros-${ROS_DISTRO}-* PACKAGES HERE <<<
#     Costs one apt run from the local apt cache; T0-T3, MAVROS and MCAP
#     stay cached.
#     Keep the list sorted so a merge conflict is a one-line conflict.
# ---------------------------------------------------------------------------
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-domain-bridge \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-foxglove-msgs \
      ros-${ROS_DISTRO}-image-geometry \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-rqt-gui \
      ros-${ROS_DISTRO}-rqt-gui-py \
      ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# T5  often: light Python packages.
#     >>> ADD NEW PIP PACKAGES HERE <<<  (pin them; a heavy wheel goes in T3)
#     Costs seconds; nothing below T5 reruns.
#
#     pymap3d: tf_loc and MAVInsight convert LLA with it. folium: the mosaic
#     map export. utm: tracking. websockets: verify/component/foxglove_probe.py
#     reads the Foxglove bridge with it -- from pip rather than apt because
#     jammy ships websockets 9.1, which passes a loop to asyncio.Lock and so
#     cannot run on the Python 3.10 beside it.
# ---------------------------------------------------------------------------
RUN --mount=type=cache,id=px4sim-pip-${TARGETARCH},target=/root/.cache/pip \
    python3 -m pip install \
      "folium" \
      "pymap3d" \
      "utm" \
      "websockets==12.0"

# ---------------------------------------------------------------------------
# T6  source-built dependencies: one FROM ros-core stage each (network only
#     in FROM scratch source stages, the build itself --network=none), then
#     one COPY --link here, so a T4/T5 edit reuses the built result. There
#     are none now; prefer a matching apt package in T2 whenever one exists.
# ---------------------------------------------------------------------------
# Last, and cheap: prove that nothing added above undid the codec repair in
# T1 or lost the MCAP storage plugin. A failure here means a T4 package pulled in a
# stripped library; move that package into T1 (before the repair).
RUN set -eu; \
    python3 -c "import cv2"; \
    for element in avdec_h264 avdec_h265 x264enc x265enc jpegenc; do \
      gst-inspect-1.0 "$element" >/dev/null; \
    done; \
    set +u; . /opt/ros/${ROS_DISTRO}/setup.sh; set -u; \
    ros2 pkg prefix rosbag2_storage_mcap >/dev/null

USER root
