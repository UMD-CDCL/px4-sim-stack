# The sim module: PX4 SITL, Gazebo Harmonic, and the camera encoders.
#
# The image holds the toolchain only. The PX4 source is bind-mounted at /px4 and
# builds at container start, so a PX4 edit needs a restart, not a rebuild.

FROM ubuntu:24.04

# Each ARG is declared directly above the RUN that reads it. A build argument
# reaches every later RUN as its environment, so one declared up here would
# make a PX4 bump or another host uid reinstall the whole toolchain too.
#
# WHERE THINGS GO
#   apt package for the streamer or Gazebo   the Gazebo/GStreamer RUN
#   PX4 release                              PX4_REF (compose / .env)
#   a new script the container runs          a COPY in the last block

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    TZ=UTC

# Keep downloaded packages in the apt cache mount every apt step uses, so a
# re-run layer installs from local .debs. docker-clean would delete them.
ARG TARGETARCH
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
    && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
         > /etc/apt/apt.conf.d/keep-cache

# ---------------------------------------------------------------------------
# Base toolchain.
# ---------------------------------------------------------------------------
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
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
ARG PX4_REF=v1.17.0
ARG GZ_DISTRO=harmonic
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=px4sim-pip-${TARGETARCH},target=/root/.cache/pip \
    mkdir -p /tmp/px4-setup && cd /tmp/px4-setup \
    && curl -fsSLO "https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_REF}/Tools/setup/ubuntu.sh" \
    && curl -fsSLO "https://raw.githubusercontent.com/PX4/PX4-Autopilot/${PX4_REF}/Tools/setup/requirements.txt" \
    && bash ubuntu.sh --no-nuttx \
    && rm -rf /tmp/px4-setup /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Gazebo development headers and the GStreamer set the video streamer needs.
# ubuntu.sh already added the osrfoundation apt repository.
# ---------------------------------------------------------------------------
RUN --mount=type=cache,id=px4sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
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
      gettext-base \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# The video streamer. One camera in, one H.265 stream out, for each stream.
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
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN if getent passwd ${HOST_UID} >/dev/null; then \
      userdel -rf "$(getent passwd ${HOST_UID} | cut -d: -f1)" || true; \
    fi \
    && groupadd -f -g ${HOST_GID} dev \
    && useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev \
    && chmod 0440 /etc/sudoers.d/dev \
    # Create the named-volume mount points here, owned by dev. Docker seeds a
    # new named volume from the image, ownership included. Leave the directory
    # out of the image and docker makes an empty root-owned volume instead,
    # which fails the build with "ccache: error: Permission denied".
    && mkdir -p /home/dev/.ccache /home/dev/.gz \
    && chown -R ${HOST_UID}:${HOST_GID} /home/dev

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY px4-rcS /opt/sim/px4-rcS
COPY hold-stream-rates.sh /opt/sim/hold-stream-rates.sh
COPY gimbal_rangefinder.py /opt/sim/gimbal_rangefinder.py
COPY scf4_emulator.py /opt/sim/scf4_emulator.py
# The framing table, from the repository root. A build context cannot reach
# outside itself, so compose gives this image a second one rather than the
# stack keeping a copy of the table for each image that reads it.
COPY --from=shared zoom.sh /opt/sim/zoom.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /opt/sim/gimbal_rangefinder.py \
      /opt/sim/hold-stream-rates.sh /opt/sim/scf4_emulator.py \
    && chmod 0644 /opt/sim/px4-rcS

USER dev
WORKDIR /px4
ENV CCACHE_DIR=/home/dev/.ccache \
    PATH=/usr/lib/ccache:$PATH

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
