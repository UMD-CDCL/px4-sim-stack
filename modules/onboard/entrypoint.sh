#!/usr/bin/env bash
# Start the onboard stack for one vehicle.
#
# UAS_NUM is the whole identity. Everything else comes from it, the way it does
# on the aircraft: MAVLink system id, ROS namespace /uas${N}, ROS domain 6${N},
# and the d${N}_ and uas${N}_ frame prefixes. See docs/uas-contract.md.
set -euo pipefail

UAS_NUM=${UAS_NUM:-11}
SCENE=${SCENE:-lorton}
TERRAIN_DIR=${TERRAIN_DIR:-/terrain}
# The airframe model. UAS_FLEET is the one source of truth for the whole fleet,
# the same list the simulator builds its models from, so the companion reads its
# own entry rather than carrying a second copy that can disagree. A disagreement
# would be silent: the simulator would serve pilot<N> while ds_node opened
# rgb<N>, and the detector would simply never receive a frame.
# Set MODEL to override one vehicle.
UAS_BASE=${UAS_BASE:-10}
UAS_FLEET=${UAS_FLEET:-"chimera_v3 chimera_v3 chimera_v2 chimera_v2"}
read -r -a _fleet <<< "${UAS_FLEET}"
SLOT=$((UAS_NUM - UAS_BASE - 1))
AIRFRAME=${_fleet[$SLOT]:-}
case "${AIRFRAME}" in
*_v2) MODEL=${MODEL:-v2} ;;
*_v3) MODEL=${MODEL:-v3} ;;
"") echo "uas${UAS_NUM} has no entry in UAS_FLEET ('${UAS_FLEET}')." >&2; exit 1 ;;
*) echo "uas${UAS_NUM} names airframe '${AIRFRAME}', which is neither v2 nor v3." >&2; exit 1 ;;
esac

if ! [[ "${UAS_NUM}" =~ ^([1-9]|1[1-9])$ ]]; then
	echo "UAS_NUM must be 1 to 9 for a real vehicle or 11 to 19 for a simulated one, not '${UAS_NUM}'." >&2
	exit 1
fi

export ROS_DOMAIN_ID=$((60 + UAS_NUM))
export ROS_LOCALHOST_ONLY=0
# The air imagery profiles are Fast DDS XML, so the bridge needs this
# implementation. The profiles themselves are set on the image bridge process
# alone, by the launch file. Do not export them here: this shell is the parent
# of every node, and a launch-wide profile would give mavros and ds_node the
# flow controller as well.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# The ROS setup scripts read variables they have not set, so nounset stops the
# entry point on the first line of the first one. Nothing below reads a name
# they leave unset, so relax it for the two lines that need it.
set +u
# ROS_DISTRO is set in the image, from the DeepStream release it was built
# on: 7.1 carries Humble and 8.0 and 9.0 carry Jazzy. Naming a distribution
# here would be a second answer to a question modules/ros-base already
# settled. See scripts/ds-select.sh.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source /home/user/ros2_ws/install/setup.bash
set -u

# The terrain cache reads every *.json in one directory and takes the first
# tile whose square holds the ray origin, so two scenes in there would make the
# choice arbitrary. /scenes holds a surface for every scene, so link the one
# scene in play into a directory of its own. An empty directory is not an
# error: every ray then meets the flat plane.
mkdir -p "${TERRAIN_DIR}"
rm -f "${TERRAIN_DIR}"/*.json
SURFACE="/scenes/worlds/${SCENE}_surface.json"
if [ -f "${SURFACE}" ]; then
	ln -s "${SURFACE}" "${TERRAIN_DIR}/"
	echo "terrain: ${SURFACE}"
else
	echo "terrain: no surface for scene '${SCENE}'. Localization uses the flat plane." >&2
fi

# The bounding box parser nvinfer loads to turn the detector's output tensor
# into boxes. onboard_sim_params.yaml points model.custom_lib_path at the model
# volume, and ds_node compiles one there when it finds none -- which needs a
# CUDA compiler that no DeepStream base ships any more. modules/ros-base built
# it against this release instead, so put it where ds_node looks.
#
# A parser belongs to the release it was compiled against, and the model volume
# outlives the image, so the one left there by another release is replaced
# rather than loaded. The stamp beside it says which release it came from.
PARSER_SRC=/opt/ds-yolo/libnvdsinfer_custom_impl_Yolo.so
PARSER_DIR=${MODEL_DIR:-/models}
PARSER_DEST="${PARSER_DIR}/libnvdsinfer_custom_impl_Yolo.so"
PARSER_STAMP="${PARSER_DIR}/.parser-deepstream"
if [ -f "${PARSER_SRC}" ] && [ -w "${PARSER_DIR}" ]; then
	want=${DS_RELEASE:-unknown}
	have=$(cat "${PARSER_STAMP}" 2>/dev/null || echo none)
	if [ ! -e "${PARSER_DEST}" ] || [ "${have}" != "${want}" ]; then
		cp "${PARSER_SRC}" "${PARSER_DEST}"
		echo "${want}" > "${PARSER_STAMP}"
		echo "parser: ${PARSER_DEST} is the DeepStream ${want} build (was ${have})"
	fi
elif [ ! -e "${PARSER_DEST}" ]; then
	echo "parser: no ${PARSER_DEST} and none to install. ds_node will try to" >&2
	echo "        compile one, which needs a CUDA compiler this image has not got." >&2
fi

# The calibration of the camera that really made the picture. A simulated camera
# is an ideal pinhole at the field of view its airframe was rendered with, so
# the calibration is a file written here rather than one of the aircraft's real
# lens files, which describe real optics and carry a real lens's distortion.
#
# Everything written here lands in CAMERA_DIR, a simulator volume. None of it
# goes near the package the aircraft loads.
read -r -a _gimbal_hfov <<< "${UAS_GIMBAL_HFOV_DEG:-56.06 56.06 85.25 25.98}"
CAMERA_HFOV_DEG=${_gimbal_hfov[$SLOT]:-${_gimbal_hfov[0]}}
CAMERA_DIR=${CAMERA_DIR:-/camera}
mkdir -p "${CAMERA_DIR}"

# shellcheck disable=SC1091
. /usr/local/bin/zoom.sh

# The aircraft calibrates at the preview size: uas1_params.yaml names
# uas1-mid-640x360-d1.yaml. The detector reports its boxes in that space and a
# panel draws the preview, so a calibration at the full sensor size puts every
# annotation a factor of three from the thing it marks.
calibrate() {   # <field of view, degrees> <name> <output file>
	python3 /usr/local/bin/sim_calibration.py \
		--model "/scenes/models/${AIRFRAME}/model.sdf" \
		--sensor gimbal_camera \
		--hfov-deg "$1" --name "$2" \
		--width "${CAMERA_PREVIEW_WIDTH:-640}" \
		--height "${CAMERA_PREVIEW_HEIGHT:-360}" \
		--out "$3"
}

# A v3 carries a zoom lens, and each framing is its own optic: the simulator
# renders a camera for each one, so each one has its own intrinsics. The zoom
# node publishes the current framing's CameraInfo, so it needs all of them.
#
# The parameters that point at these files, and at the emulated controller,
# are written beside them and loaded last. They are simulator values -- a
# framing table of 1x, 3x and 10x, and a lens on a TCP port -- and a simulated
# uas11 loads the real uas1_params.yaml, so they must not be in it.
LENS_PARAMS=""
if [ "${MODEL}" = v3 ]; then
	LENS_PARAMS="${CAMERA_DIR}/lens.yaml"
	ZOOM_SERIAL_HOST=${ZOOM_SERIAL_HOST:-sim}
	{
		echo "# Written by modules/onboard/entrypoint.sh from scripts/zoom.sh."
		echo "# The simulated lens: which framings it reaches, where they put"
		echo "# the motors, and where the controller answers. Loaded last, so"
		echo "# it beats the aircraft's own values for the same node."
		echo "/**/*:"
		echo "  zoom:"
		echo "    ros__parameters:"
		echo "      # An emulated SCF4 in the sim container, in place of the"
		echo "      # USB controller the aircraft finds by its USB identity."
		echo "      # Setting a port is what skips that search."
		printf '      zoom.serial.port: "socket://%s:%s"\n' \
			"${ZOOM_SERIAL_HOST}" "$(zoom_port "${UAS_NUM}")"
		echo "      # The emulated lens has no optics, so sharpness does not"
		echo "      # follow the focus axis and a sweep would find a peak that"
		echo "      # means nothing -- and leave the framing off its preset."
		echo '      zoom.autofocus.topic.image: ""'
		for _axis in zoom focus; do
			printf '      zoom.home.%s: %s\n' "${_axis}" "$(zoom_datum "${_axis}")"
			printf '      zoom.limits.%s: [%s]\n' "${_axis}" "$(zoom_travel "${_axis}")"
		done
		for _preset in $(zoom_presets_widest_first); do
			printf '      zoom.presets.%s: [%s]\n' "${_preset}" "$(zoom_steps "${_preset}")"
			printf '      zoom.camera_info.calibration.%s: "%s/gimbal-%s.yaml"\n' \
				"${_preset}" "${CAMERA_DIR}" "${_preset}"
		done
		if _boot=$(zoom_preset_of_slot "${SLOT}"); then
			printf '      zoom.boot.preset: "%s"\n' "${_boot}"
		fi
	} > "${LENS_PARAMS}"

	for _preset in $(zoom_presets_widest_first); do
		calibrate "$(zoom_hfov_deg "${_preset}")" "${_preset}" \
			"${CAMERA_DIR}/gimbal-${_preset}.yaml"
	done
	echo "lens: $(echo "${UAS_ZOOM_PRESETS}" | tr ' ' ',') on socket://${ZOOM_SERIAL_HOST}:$(zoom_port "${UAS_NUM}")"
else
	# One fixed lens, one calibration, published by cam_info.
	calibrate "${CAMERA_HFOV_DEG}" simulated "${CAMERA_DIR}/gimbal.yaml"
fi

SITE_PARAMS=${SITE_PARAMS:-/camera/site.yaml}
source /usr/local/bin/site-params.sh

# Where the survey marker really is. A fiducial capture is localized and the
# difference between that and this is the correction the whole fleet's frame
# moves by, so an unset marker surveys against latitude zero and moves the
# frame across the planet. The scene carries the coordinates. The altitude is
# left at zero on purpose: tf_loc takes it from the ground model, which is
# right by construction on flat ground.
if [ "${FIDUCIAL_ENABLED:-0}" = 1 ] && [ -n "${FIDUCIAL_SURVEYED_LAT:-}" ]; then
	printf '  tf_loc:\n    ros__parameters:\n      fiducial_lla: [%s, %s, 0.0]\n' \
		"${FIDUCIAL_SURVEYED_LAT}" "${FIDUCIAL_SURVEYED_LON}" >> "${SITE_PARAMS}"
	echo "site: fiducial surveyed at ${FIDUCIAL_SURVEYED_LAT}, ${FIDUCIAL_SURVEYED_LON}"
fi

# The detector opens its camera once and dies if it is not there. On the
# aircraft the camera is a local socket that exists at boot; here it is a stream
# the simulator publishes after Gazebo has loaded the world and placed the
# scenario, which takes minutes on a large scene. So wait until a frame can be
# pulled from it, with the same GStreamer the detector opens it with.
GIMBAL_STREAM=${GIMBAL_STREAM:-$([ "${MODEL}" = v3 ] && echo "rgb${UAS_NUM}" || echo "pilot${UAS_NUM}")}
CAMERA_URI="${RTSP_BASE:-rtsp://video-router:8554}/${GIMBAL_STREAM}"
STREAM_WAIT_S=${STREAM_WAIT_S:-300}
waited=0
until timeout 15 gst-launch-1.0 -q rtspsrc "location=${CAMERA_URI}" latency=100 \
	! fakesink num-buffers=1 >/dev/null 2>&1; do
	if [ "${waited}" -ge "${STREAM_WAIT_S}" ]; then
		echo "uas${UAS_NUM}: ${CAMERA_URI} never appeared after ${STREAM_WAIT_S}s." >&2
		echo "The detector will start anyway and fail to open its camera." >&2
		break
	fi
	[ "${waited}" = 0 ] && echo "waiting for ${GIMBAL_STREAM}"
	sleep 5
	waited=$((waited + 5))
done
# Said whether or not there was a wait. `./px4sim fly` blocks until this line
# appears in this container's log, so a camera that was ready on the first probe
# used to leave that wait with nothing to find: it timed out after its whole
# budget and reported a vehicle that never reached the air, on a stack where
# every part of it was working. A warm video router makes the no-wait case the
# usual one, which is why a fleet started earlier hits it every time.
if [ "${waited}" -gt 0 ]; then
	echo "camera: ${GIMBAL_STREAM} ready after ${waited}s"
else
	echo "camera: ${GIMBAL_STREAM} ready, first probe"
fi

if [ "${1:-launch}" = "launch" ]; then
	shift || true
	# One file per machine, at the same path on every machine:
	# /models/local/params.yaml, where `local` is the symlink pointing at this
	# machine's engine group. A machine with nothing of its own to say has no
	# such file, and ros2 launch refuses a params: entry that names one that is
	# not there, so an absent file is dropped rather than being an error.
	if [ -n "${ONBOARD_PARAMS_FILE}" ] && [ ! -f "${ONBOARD_PARAMS_FILE}" ]; then
		echo "params: no ${ONBOARD_PARAMS_FILE} on this machine; using the common files only"
		ONBOARD_PARAMS_FILE=
	fi

	exec ros2 launch umd_uas onboard.launch.py \
		uas:="${UAS_NUM}" \
		model:="${MODEL}" \
		sim:=true \
		params:="${SITE_PARAMS}${LENS_PARAMS:+,${LENS_PARAMS}}${ONBOARD_PARAMS_FILE:+,${ONBOARD_PARAMS_FILE}}" \
		"$@"
fi

exec "$@"
