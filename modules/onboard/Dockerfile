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
ARG DS_VERSION=7.1

FROM px4simstack/ros-base:${DS_VERSION}

ARG USERNAME=user

USER root
COPY entrypoint.sh sim_calibration.py /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
