# shellcheck shell=bash
# The pure arithmetic: terrain rays, roofs, the shared ground and the drawn map
#
# These run against the sources as built into the companion image, so they check
# the code the vehicle will actually run rather than a copy on this host.

onboard_image=px4simstack/onboard:${DS_TAG:-${DS_VERSION:-7.1}}
workspace=$(readlink -f "${ROS2_WS_DIR:-../ros2_ws}")

if ! docker image inspect "$onboard_image" >/dev/null 2>&1; then
	fail "$onboard_image is not built. Run ./px4sim build onboard"
	return 0
fi

run_tests() {
	local what=$1 package=$2; shift 2
	output=$(docker run --rm --entrypoint bash \
		-v "$workspace/src/$package:/src:ro" "$onboard_image" -c "
			source /opt/ros/\$ROS_DISTRO/setup.bash
			source /home/user/ros2_ws/install/setup.bash
			cd /src && python3 -m pytest $* -q 2>&1
		" 2>&1) || true
	summary=$(printf '%s' "$output" | grep -oE '[0-9]+ (passed|failed)[^,]*' | tr '\n' ' ')
	if printf '%s' "$output" | grep -qE '^[0-9]+ passed'; then
		pass "$what: ${summary% }"
	else
		fail "$what: ${summary:-no result}"
		note "$(printf '%s' "$output" | grep -E '^FAILED' | head -5)"
	fi
}

run_tests "terrain and ground frame" 5g_drone test/test_terrain.py test/test_ground_frame.py
# Which way up and which way round the ground is drawn. A model that is turned
# or mirrored publishes exactly as convincingly as a correct one.
run_tests "the drawn map's axes and texture" MAVInsight test/test_gltf.py
