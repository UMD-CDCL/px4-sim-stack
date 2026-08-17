# The sim module: PX4 SITL, Gazebo Harmonic, and the camera encoders.
#
# The image holds the toolchain only. The PX4 source is bind-mounted at /px4 and
# builds at container start, so a PX4 edit needs a restart, not a rebuild.

FROM ubuntu:24.04

ARG PX4_REF=v1.17.0
ARG GZ_DISTRO=harmonic
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    TZ=UTC

# ---------------------------------------------------------------------------
# Base toolchain.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      bc \
      build-essential \
      ca-certificates \
      ccache \
      cmake \
      curl \
      file \
      gawk \
      git \
      gnupg \
      lsb-release \
      ninja-build \
      python3 \
      python3-pip \
      rsync \
      sudo \
      tzdata \
      wget \
      xz-utils \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# PX4 dependencies. This runs the setup script from the pinned PX4 release, so
# the dependency set always matches PX4_REF. --no-nuttx skips the embedded
# cross-compiler, which SITL does not need.
# ---------------------------------------------------------------------------
RUN mkdir -p /tmp/px4-setup && cd /tmp/px4-setup \
    && curl -fsSLO "https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_REF}/Tools/setup/ubuntu.sh" \
    && curl -fsSLO "https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_REF}/Tools/setup/requirements.txt" \
    && bash ubuntu.sh --no-nuttx \
    && rm -rf /tmp/px4-setup /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Gazebo development headers and the GStreamer set the video streamer needs.
# ubuntu.sh already added the osrfoundation apt repository.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgz-transport13-dev \
      libgz-msgs10-dev \
      libgz-common5-dev \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav \
      gstreamer1.0-rtsp \
      gstreamer1.0-tools \
      libglvnd0 libgl1 libglx0 libegl1 libgles2 \
      libxext6 libx11-6 libxrandr2 libxi6 libxcursor1 libxinerama1 \
      mesa-utils x11-utils \
      python3-yaml \
      python3-paho-mqtt \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# The video streamer. One camera in, one H.264 stream out, for each camera.
# ---------------------------------------------------------------------------
COPY gz_video_streamer /src/gz_video_streamer
RUN cmake -S /src/gz_video_streamer -B /src/gz_video_streamer/build \
      -G Ninja -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /src/gz_video_streamer/build \
    && cmake --install /src/gz_video_streamer/build --prefix /usr/local \
    && rm -rf /src/gz_video_streamer/build

# ---------------------------------------------------------------------------
# A user whose id matches the host, so the bind-mounted PX4 build output does
# not end up owned by root.
# ---------------------------------------------------------------------------
RUN if getent passwd ${UID} >/dev/null; then \
      userdel -rf "$(getent passwd ${UID} | cut -d: -f1)" || true; \
    fi \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev \
    # Create the named-volume mount points here, owned by dev. Docker seeds a
    # new named volume from the image, ownership included. Leave the directory
    # out of the image and docker makes an empty root-owned volume instead,
    # which fails the build with "ccache: error: Permission denied".
    && mkdir -p /home/dev/.ccache /home/dev/.gz \
    && chown -R ${UID}:${GID} /home/dev

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY px4-rcS /opt/sim/px4-rcS
COPY hold-stream-rates.sh /opt/sim/hold-stream-rates.sh
COPY frame_clock.py /opt/sim/frame_clock.py
RUN chmod +x /usr/local/bin/entrypoint.sh /opt/sim/frame_clock.py \
    && chmod 0644 /opt/sim/px4-rcS

USER dev
WORKDIR /px4
ENV CCACHE_DIR=/home/dev/.ccache \
    PATH=/usr/lib/ccache:$PATH

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
