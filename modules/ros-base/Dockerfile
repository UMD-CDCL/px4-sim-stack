# ---------------------------------------------------------------------------
# What the companion and the ground station are both made of.
#
# The two images were the same file twice: the same DeepStream base, the same
# ROS 2 packages, the same OpenCV repair, the same user and the same
# workspace, differing by three apt packages and two directories. Building it
# once and deriving both from it halves the work and leaves one list to keep in
# step instead of two.
#
# The DeepStream release and the ROS distribution are one choice, not two. A
# DeepStream base carries its Ubuntu with it, and an Ubuntu carries exactly one
# ROS 2 distribution: 7.1 is 22.04 and Humble, 8.0 and 9.0 are 24.04 and Jazzy.
# scripts/ds-select.sh holds that table and reads the machine, and everything
# here arrives as a build argument from it. Nothing below picks a version.
#
# Layer order is cheapest and most stable first, so an edit costs only the
# layers after it. The nodes are built last here, and each derived image adds
# nothing but its entry point.
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# The DeepStream-Yolo bounding box parser, the .so nvinfer loads to turn the
# detector's output tensor into boxes.
#
# ds_node builds this from its vendored sources at first start when the file is
# missing. That works where the base image carries a CUDA compiler, and no base
# does any more: 7.1 ships nvcc for 12.1 while the parser wants 12.6, 8.0 ships
# 12.5 and wants 12.8, and 9.0 ships none at all. So it is compiled here, once,
# against the release the image is built from, and the entry point puts it
# where ds_node looks. No compiler and no CUDA headers reach the image that
# flies.
#
# Which CUDA a release wants is the flight code's answer, not this file's:
# _DS_CUDA_VERSIONS in ds_ros_pipeline/infer_configs.py is what ds_node would
# have used, so it is read from there rather than written down again.
#
# The CUDA compiler comes from apt only where the base has none for the version
# the flight code names. A Jetson gets CUDA from JetPack rather than from the
# x86 CUDA repository, so that install can fail there; the fallback takes the
# newest nvcc the base already carries and builds against that instead of
# stopping. An x86 DeepStream 7.1 ships nvcc for 12.1 and the parser wants
# 12.6, which is why the install is tried at all.
#
# Compiling against NvInfer.h needs TensorRT headers at the version of the
# libraries beside them, so this stage installs them where an upgrade is in
# play. Only this stage compiles; the image that flies carries the runtime and
# no headers.
#
# The loop over lib*.so.N makes the unversioned names a linker needs. They come
# from the CUDA dev packages, and asking apt for the one that carries -lcublas
# pulls a newer libcublas than the base pins, which apt then refuses. Only the
# link is missing -- nothing here includes a cuBLAS header -- so it is made
# rather than moving the library DeepStream was built against.
# ---------------------------------------------------------------------------
FROM ${DS_IMAGE} AS dsyolo

ARG DS_VERSION
ENV DEBIAN_FRONTEND=noninteractive

# The TensorRT this GPU needs, when the release does not ship one.
#
# DeepStream 7.1 carries TensorRT 10.3, which emits kernels up to Hopper. On a
# newer card it parses the ONNX, reports `Unsupported SM`, builds no engine and
# takes the process down with it -- while the streams keep decoding, so the
# only visible symptom is a detector that never fires. NVIDIA packages newer
# TensorRT for jammy, the soname does not change, and DeepStream's own nvinfer
# builds and loads engines against it.
#
# Empty means the base's own TensorRT is right for this machine, which is the
# case for every GPU up to Hopper -- the Orin included, so the aircraft image
# is byte for byte what it always was. scripts/ds-select.sh decides.
ARG TRT_VERSION=
ARG TRT_CUDA=
RUN if [ -n "${TRT_VERSION}" ]; then \
        set -eux; \
        apt-get update; \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer10=${TRT_VERSION}" \
            "libnvinfer-plugin10=${TRT_VERSION}" \
            "libnvonnxparsers10=${TRT_VERSION}" \
            "libnvinfer-dispatch10=${TRT_VERSION}" \
            "libnvinfer-lean10=${TRT_VERSION}" \
            "libnvinfer-vc-plugin10=${TRT_VERSION}" \
            "libnvinfer-bin=${TRT_VERSION}" \
            "cuda-cudart-${TRT_CUDA}" \
            "libcublas-${TRT_CUDA}"; \
        rm -rf /var/lib/apt/lists/*; \
        echo "TensorRT is now $(dpkg -l libnvinfer10 | awk '/^ii/ {print $3}')"; \
    fi

COPY --from=ros2_ws src/5g_drone/config/deepstream/nvdsinfer_custom_impl_Yolo /src
COPY --from=ros2_ws src/5g_drone/umd_uas/ds_ros_pipeline/infer_configs.py /tmp/infer_configs.py

RUN set -eux; \
    CUDA_VER="$(python3 -c "\
import ast, re, sys; \
text = open('/tmp/infer_configs.py').read(); \
table = ast.literal_eval(re.search(r'_DS_CUDA_VERSIONS\s*=\s*(\{[^}]*\})', text).group(1)); \
match = [v for k, v in table.items() if '${DS_VERSION}'.startswith(k)]; \
sys.exit('infer_configs.py names no CUDA for DeepStream ${DS_VERSION}') if not match else print(match[0])")"; \
    echo "DeepStream ${DS_VERSION} builds the parser against CUDA ${CUDA_VER}."; \
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
    cp -r /src /build; \
    make -C /build "CUDA_VER=${CUDA_VER}" -j"$(nproc)"; \
    mkdir -p /out; \
    cp /build/libnvdsinfer_custom_impl_Yolo.so /out/

# ---------------------------------------------------------------------------
# The image itself.
# ---------------------------------------------------------------------------
FROM ${DS_IMAGE}

# An ARG before FROM reaches FROM only. Declare them again to use them below.
ARG DS_VERSION
# The ROS 2 distribution that belongs to this base, and the Ubuntu that carries
# both. scripts/ds-select.sh decides; the check below refuses a pair that does
# not match the image that actually arrived.
ARG ROS_DISTRO=humble
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
# px4_msgs is not released to apt, and models/vehicle.py and models/sensor.py in
# MAVInsight import it at module level. Pin this to the PX4 release if anyone
# ever puts the uXRCE bridge on this stack; nothing here reads the field values.
ARG PX4_MSGS_REF=main

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
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      curl \
      gnupg2 \
      locales \
    && locale-gen en_US.UTF-8 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && . /etc/os-release \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
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
      ros-${ROS_DISTRO}-domain-bridge \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-foxglove-msgs \
      ros-${ROS_DISTRO}-image-geometry \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-message-filters \
      ros-${ROS_DISTRO}-nav-msgs \
      ros-${ROS_DISTRO}-rosbag2-py \
      ros-${ROS_DISTRO}-rqt-gui \
      ros-${ROS_DISTRO}-rqt-gui-py \
      ros-${ROS_DISTRO}-rosidl-runtime-py \
      ros-${ROS_DISTRO}-ros-base \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-visualization-msgs \
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


# The TensorRT this GPU needs, when the release does not ship one.
#
# DeepStream 7.1 carries TensorRT 10.3, which emits kernels up to Hopper. On a
# newer card it parses the ONNX, reports `Unsupported SM`, builds no engine and
# takes the process down with it -- while the streams keep decoding, so the
# only visible symptom is a detector that never fires. NVIDIA packages newer
# TensorRT for jammy, the soname does not change, and DeepStream's own nvinfer
# builds and loads engines against it.
#
# Empty means the base's own TensorRT is right for this machine, which is the
# case for every GPU up to Hopper -- the Orin included, so the aircraft image
# is byte for byte what it always was. scripts/ds-select.sh decides.
ARG TRT_VERSION=
ARG TRT_CUDA=
RUN if [ -n "${TRT_VERSION}" ]; then \
        set -eux; \
        apt-get update; \
        apt-get install -y --no-install-recommends --allow-downgrades \
            "libnvinfer10=${TRT_VERSION}" \
            "libnvinfer-plugin10=${TRT_VERSION}" \
            "libnvonnxparsers10=${TRT_VERSION}" \
            "libnvinfer-dispatch10=${TRT_VERSION}" \
            "libnvinfer-lean10=${TRT_VERSION}" \
            "libnvinfer-vc-plugin10=${TRT_VERSION}" \
            "libnvinfer-bin=${TRT_VERSION}" \
            "cuda-cudart-${TRT_CUDA}" \
            "libcublas-${TRT_CUDA}"; \
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

# MAVROS refuses to convert between AMSL and ellipsoid height without these.
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

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
      "folium"

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

# The parser from the stage above. The entry point copies it into the model
# directory when that directory has none, because model.custom_lib_path in
# onboard_sim_params.yaml names a path on the mounted volume and ds_node uses
# any file it finds there in preference to building one.
COPY --from=dsyolo /out/libnvdsinfer_custom_impl_Yolo.so \
     /opt/ds-yolo/libnvdsinfer_custom_impl_Yolo.so

# ---------------------------------------------------------------------------
# The workspace. `ros2_ws` is a named build context, so the sources stay where
# they are checked out and no copy of them lives in this repository.
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch "${PX4_MSGS_REF}" \
      https://github.com/PX4/px4_msgs.git "${WORKSPACE_DIR}/src/px4_msgs" \
    && rm -rf "${WORKSPACE_DIR}/src/px4_msgs/.git"

COPY --from=ros2_ws --chown=${HOST_UID}:${HOST_GID} src/cdcl_umd_msgs \
     ${WORKSPACE_DIR}/src/cdcl_umd_msgs

RUN chown -R "${HOST_UID}:${HOST_GID}" "${WORKSPACE_DIR}"

USER ${USERNAME}
WORKDIR ${WORKSPACE_DIR}

# The interfaces first, on their own. Generating and compiling them is five
# minutes and they change when a .msg does, which is seldom. Built with the
# Python packages, as they were, a one line edit to a node paid those five
# minutes again.
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Then the nodes. colcon finds the interfaces already built and this costs
# seconds, which is what an edit to a node should cost.
USER root
# The source, and nothing a working copy accumulates around it.
#
# perception_models is 5g_drone's tree of ONNX exports and TensorRT engines,
# gigabytes of it, and the onboard service bind-mounts that same directory at
# /models -- the only path the node reads, because model.dir names
# /models/local. A copy here is one nothing opens.
#
# .git and .vscode are the working copy's own. .vscode costs more than its
# size: the IntelliSense browse database is rewritten when a file is merely
# opened, and this COPY is the layer the colcon build below sits on, so
# browsing the flight code would invalidate it and buy back the five minutes
# the split above exists to save.
#
# Excluded here rather than ignored in the context, because the context is a
# checkout this repository does not own and must not leave files in.
COPY --exclude=perception_models --exclude=.git --exclude=.vscode \
     --from=ros2_ws --chown=${HOST_UID}:${HOST_GID} \
     src/5g_drone ${WORKSPACE_DIR}/src/5g_drone
COPY --exclude=.git --exclude=.vscode \
     --from=ros2_ws --chown=${HOST_UID}:${HOST_GID} \
     src/MAVInsight ${WORKSPACE_DIR}/src/MAVInsight
USER ${USERNAME}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# What both entry points work the site out with. Last, because it is the
# cheapest thing here to edit and nothing above it depends on it.
USER root
COPY site-params.sh /usr/local/bin/site-params.sh
USER ${USERNAME}
