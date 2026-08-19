# ---------------------------------------------------------------------------
# The ground station, as an image.
#
# One container for the whole fleet, and the mirror of modules/onboard: the same
# base, the same workspace, the same MAVROS. It runs a listening MAVROS, a frame
# tree and a preview ds_node for each vehicle, and one Foxglove bridge for the
# operator.
#
# Everything it is made of is modules/ros-base. What this file adds is what only
# the ground has.
# ---------------------------------------------------------------------------
ARG DS_VERSION=7.1

FROM px4simstack/ros-base:${DS_VERSION}

ARG USERNAME=user

USER root
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
EXPOSE 8765/tcp
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
