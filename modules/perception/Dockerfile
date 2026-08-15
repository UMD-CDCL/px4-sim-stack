# The perception module: DeepStream inference.
#
# Why 8.0 and not 9.0
#   DeepStream 9.0 needs NVIDIA driver 590.48. DeepStream 8.0 needs 570.133.
#   Check `nvidia-smi` before you raise DS_IMAGE in .env. The stack has no way
#   to work around a driver that is older than the SDK.
#
# There is no application code in this module. deepstream-app reads a config
# file and does the work. Adding Python bindings would buy nothing that the
# config file cannot express, and it would tie the module to a DeepStream
# release.

ARG DS_IMAGE=nvcr.io/nvidia/deepstream:8.0-samples-multiarch
FROM ${DS_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# gettext-base supplies envsubst, which puts the addresses from .env into the
# config file. mosquitto-clients is for checking the bus by hand.
RUN apt-get update && apt-get install -y --no-install-recommends \
      gettext-base \
      mosquitto-clients \
      python3-paho-mqtt \
      iputils-ping \
      vim less \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/perception/configs /opt/perception/models/cache /opt/perception/logs

COPY --chmod=0755 entrypoint.sh /usr/local/bin/entrypoint.sh

WORKDIR /opt/perception

# The RTSP server that deepstream-app starts. The video router pulls from it.
EXPOSE 8554/tcp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
