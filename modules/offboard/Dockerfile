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
# Compose supplies the exact ros-base build result as a service context.
FROM ros_base

ARG USERNAME=user

USER root
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
COPY --from=shared sim_gimbal_attitude.py /usr/local/bin/sim_gimbal_attitude.py
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/sim_gimbal_attitude.py

USER ${USERNAME}
EXPOSE 8765/tcp
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["launch"]
