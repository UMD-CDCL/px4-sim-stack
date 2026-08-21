# ---------------------------------------------------------------------------
# The companion computer, as an image.
#
# One container for each simulated vehicle. It carries what the Orin carries:
# DeepStream, ROS 2 Humble, MAVROS from apt, and the built ros2_ws. All of that
# is modules/ros-base, which the ground station is made of too.
#
# Nothing heavy belongs here. What this file adds is what only a vehicle has,
# and it is last on purpose: an edit costs the layers after it, and there are
# none.
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
COPY entrypoint.sh sim_calibration.py /usr/local/bin/
# The framing table, from the repository root. A build context cannot reach
# outside itself, so compose gives this image a second one rather than the
# stack keeping a copy of the table for each image that reads it.
COPY --from=shared zoom.sh /usr/local/bin/zoom.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
