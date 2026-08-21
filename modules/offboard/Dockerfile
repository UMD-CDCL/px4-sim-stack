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
# The tag of the base to derive from. It is the release plus whatever
# accommodation this machine needed -- 7.1 on an Orin, 7.1-trt10.9 on a card
# too new for the TensorRT that release ships. Following DS_VERSION instead
# would silently derive from a stock base and undo the accommodation.
# scripts/ds-select.sh --tag is where it comes from.
ARG DS_TAG=7.1

FROM px4simstack/ros-base:${DS_TAG}

ARG USERNAME=user

USER root
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
EXPOSE 8765/tcp
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
