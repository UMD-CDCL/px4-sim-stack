# shellcheck shell=bash
# Every ROS overlay this image carries, lowest first. Both entry points and
# every `docker exec` that runs a ros2 command source this one file, so an
# overlay added here reaches all of them.
#
# The ROS setup scripts read variables they have not set, so nounset comes
# off around them and goes back on only where the caller had it.
case $- in *u*) _had_nounset=1 ;; *) _had_nounset= ;; esac
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# MAVROS, patched for PX4 v1.18, when the image was built with it. Between
# /opt/ros and the workspace, so it wins over the apt package.
if [ -f /opt/mavros/install/setup.bash ]; then
	# shellcheck disable=SC1091
	source /opt/mavros/install/setup.bash
fi
# shellcheck disable=SC1091
source /home/user/ros2_ws/install/setup.bash
if [ -n "${_had_nounset}" ]; then
	set -u
fi
unset _had_nounset
