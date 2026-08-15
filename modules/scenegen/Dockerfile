# The scenegen module: builds a Gazebo scene from real map data.
#
# Satellite imagery, elevation tiles and OSM buildings come in; a textured
# terrain model, a world SDF and a casualty scenario come out, written into
# modules/sim/scenes. The module runs on demand and exits. It is not part
# of the running stack and the sim never depends on it.
#
# The code is bind-mounted at /app over the copy baked in here, so an edit
# needs no rebuild. The copy keeps the image usable on its own.

FROM ubuntu:24.04

ARG UID=1000
ARG GID=1000

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    TZ=UTC

RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      libgl1 \
      libglib2.0-0t64 \
      python3 \
      python3-numpy \
      python3-pil \
      python3-pip \
      python3-requests \
      python3-shapely \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*

# The vehicle detector. Ultralytics has no apt package, so this is a pip
# install, against the stack's apt-only habit. The CPU torch wheel keeps
# the image several GB smaller than the CUDA default, and one satellite
# image a run does not need a GPU.
#
# The pins keep pip inside the apt-installed numpy: opencv 5 demands
# numpy 2, and pip then tries to uninstall Debian's numpy 1.26 and fails
# with "RECORD file not found".
RUN pip3 install --no-cache-dir --break-system-packages \
      --extra-index-url https://download.pytorch.org/whl/cpu \
      "numpy<2" "opencv-python<5" ultralytics \
    && rm -rf /root/.cache

# The container writes into bind-mounted repo directories, so it runs as
# the host user or the output belongs to root.
RUN if getent passwd ${UID} >/dev/null; then \
      userdel -rf "$(getent passwd ${UID} | cut -d: -f1)" || true; \
    fi \
    && groupadd -f -g ${GID} dev \
    && useradd -m -u ${UID} -g ${GID} -s /bin/bash dev

COPY . /app
RUN chown -R ${UID}:${GID} /app

USER dev
WORKDIR /app
EXPOSE 8090

ENTRYPOINT ["python3", "/app/scenegen.py"]
