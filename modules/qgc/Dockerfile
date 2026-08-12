# QGroundControl, the released build.
#
# This image runs the official AppImage. It is the fast path: no Qt, no compile,
# about two minutes to build. To change QGroundControl itself, use the qgc-dev
# profile instead, which gives you a Qt toolchain and the source tree.
#
# The AppImage is extracted at build time. That avoids FUSE, which a container
# cannot use without extra privileges.

FROM ubuntu:24.04

ARG QGC_REF=v5.0.8
ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl socat \
      libfontconfig1 libfreetype6 \
      libx11-6 libx11-xcb1 libxext6 libxfixes3 libxi6 libxrender1 \
      libxcb1 libxcb-glx0 libxcb-keysyms1 libxcb-image0 libxcb-shm0 \
      libxcb-icccm4 libxcb-sync1 libxcb-xfixes0 libxcb-shape0 libxcb-randr0 \
      libxcb-render-util0 libxcb-util1 libxcb-xinerama0 libxcb-xkb1 \
      libxcb-cursor0 libxkbcommon0 libxkbcommon-x11-0 \
      libgl1 libglx0 libegl1 libglvnd0 libopengl0 \
      libglib2.0-0t64 libdbus-1-3 libnss3 \
      libasound2t64 libpulse0 libudev1 libusb-1.0-0 libsdl2-2.0-0 \
      libspeechd2 libssl3t64 libsm6 libice6 \
      libwayland-client0 libwayland-cursor0 libwayland-egl1 \
      libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav \
      gstreamer1.0-x gstreamer1.0-gl \
      fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL -o /tmp/qgc.AppImage \
      "https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_REF}/QGroundControl-x86_64.AppImage" \
    && chmod +x /tmp/qgc.AppImage \
    && cd /opt && /tmp/qgc.AppImage --appimage-extract >/dev/null \
    && mv /opt/squashfs-root /opt/qgc \
    && rm -f /tmp/qgc.AppImage

# The serial group lets QGroundControl reach a USB radio if you pass one in.
RUN if getent passwd ${UID} >/dev/null; then \
      userdel -rf "$(getent passwd ${UID} | cut -d: -f1)" || true; \
    fi \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -G dialout -s /bin/bash dev \
    # Docker seeds a new named volume from the image, ownership included. A
    # mount point that the image does not have becomes a root-owned volume that
    # QGroundControl cannot write its settings into.
    && mkdir -p /home/dev/.config /home/dev/Documents/QGroundControl \
    && chown -R ${UID}:${GID} /home/dev

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER dev
WORKDIR /home/dev

ENV QT_X11_NO_MITSHM=1 \
    QGC_VIDEO_URL=rtsp://video-router:8554/gimbal

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
