# shellcheck shell=bash
# A hardcoded detection box lands where the geometry says it should
#
# Runs tf_loc itself, with the transform tree, the fix and the calibration
# written in rather than read from topics. No detector and no simulator, so a
# failure here is the localization arithmetic and nothing else.

onboard_image=px4simstack/onboard:${DS_VERSION_ONBOARD:-7.1}

if ! docker image inspect "$onboard_image" >/dev/null 2>&1; then
	fail "$onboard_image is not built. Run ./px4sim build onboard"
	return 0
fi

surface=modules/sim/scenes/worlds/${SCENE:-campus}_surface.json
if [ ! -f "$surface" ]; then
	fail "scene '${SCENE:-campus}' has no surface to localize against"
	note "expected $surface. Build it with ./px4sim genscene"
	return 0
fi

# The terrain cache reads every file in its directory, so the scene under test
# gets one of its own.
# The image supplies the environment and the working tree supplies the code, so
# a source change is checked without a rebuild. `./px4sim build` then `./px4sim
# start` is what proves the baked image agrees.
installed=/home/user/ros2_ws/install/umd_uas/lib/python3.10/site-packages/umd_uas
output=$(docker run --rm --entrypoint bash \
	-e UAS_ZOOM_PRESETS="$UAS_ZOOM_PRESETS" \
	-v ./verify:/verify:ro -v ./modules/sim/scenes:/scenes:ro \
	-v "$(readlink -f "${ROS2_WS_DIR:-../ros2_ws}")/src/5g_drone/umd_uas:$installed:ro" \
	"$onboard_image" -c "
		source /opt/ros/humble/setup.bash
		source /home/user/ros2_ws/install/setup.bash
		mkdir -p /tmp/terrain && cp /scenes/worlds/$(basename "$surface") /tmp/terrain/
		python3 /verify/component/localize.py /tmp/terrain/$(basename "$surface") 2>/tmp/log
	" 2>&1) || true

if ! printf '%s' "$output" | grep -q $'\t'; then
	fail "the localization harness ran"
	note "$(printf '%s' "$output" | tail -3)"
	return 0
fi

while IFS=$'\t' read -r verdict what detail; do
	[ -n "$what" ] || continue
	if [ "$verdict" = ok ]; then pass "$what"; else fail "$what"; note "$detail"; fi
done <<< "$(printf '%s' "$output" | grep -E $'^(ok|FAIL)\t')"
