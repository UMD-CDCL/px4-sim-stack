#!/usr/bin/env bash
# Start the onboard stack for one vehicle.
#
# UAS_NUM is the whole identity. Everything else comes from it, the way it does
# on the aircraft: MAVLink system id, ROS namespace /uas${N}, ROS domain 6${N},
# and the d${N}_ and uas${N}_ frame prefixes. See docs/uas-contract.md.
set -euo pipefail

UAS_NUM=${UAS_NUM:-11}
SCENE=${SCENE:-recon_field}
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
source /opt/ros/humble/setup.bash
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

# The calibration of the camera that really made the picture. A simulated camera
# is an ideal pinhole at the field of view its airframe was rendered with, so
# cam_info reads a file written here rather than one of the aircraft's real lens
# files. config/param_files/sim/onboard_sim_params.yaml names this path.
read -r -a _gimbal_hfov <<< "${UAS_GIMBAL_HFOV_DEG:-56.06 56.06 85.25 25.98}"
CAMERA_HFOV_DEG=${_gimbal_hfov[$SLOT]:-${_gimbal_hfov[0]}}

# A v3 lens zooms. The simulator crops its camera to the preset the vehicle
# flies at, so the calibration is the preset's field of view rather than the
# widest the camera renders. One table says what a preset is, and both read it.
# shellcheck disable=SC1091
. /usr/local/bin/zoom.sh
if [ "${MODEL}" = v3 ] && _preset=$(zoom_preset_of_slot "${SLOT}"); then
	if _preset_hfov=$(zoom_hfov_deg "${_preset}"); then
		CAMERA_HFOV_DEG=${_preset_hfov}
		echo "lens: ${_preset} preset, ${_preset_hfov} degrees"
	else
		echo "uas${UAS_NUM} asks for zoom preset '${_preset}', which UAS_ZOOM_PRESETS does not name." >&2
	fi
fi

python3 /usr/local/bin/sim_calibration.py \
	--model "/scenes/models/${AIRFRAME}/model.sdf" \
	--sensor gimbal_camera \
	--hfov-deg "${CAMERA_HFOV_DEG}" \
	--out "${CAMERA_DIR:-/camera}/gimbal.yaml"

# The site, worked out from the scene rather than configured. A terrain tile is
# anchored above mean sea level and a NavSatFix altitude is above the WGS84
# ellipsoid; GeoidEval says how far apart those are here, and mavros already
# installs the datasets it reads.
SITE_PARAMS=${SITE_PARAMS:-/camera/site.yaml}
if [ -f "${SURFACE}" ]; then
	geoid_height=$(python3 -c "
import json, subprocess, sys
latitude, longitude, _ = json.load(open('${SURFACE}'))['origin_lla']
print(subprocess.run(['GeoidEval'], input=f'{latitude} {longitude}',
                     capture_output=True, text=True).stdout.strip())")
	printf '/**/*:\n  ros__parameters:\n    localization.geoid_height_m: %s\n' \
		"${geoid_height}" > "${SITE_PARAMS}"
	echo "site: geoid height ${geoid_height} m"
else
	printf '/**/*:\n  ros__parameters:\n    localization.geoid_height_m: 0.0\n' \
		> "${SITE_PARAMS}"
fi

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
[ "${waited}" -gt 0 ] && echo "camera: ${GIMBAL_STREAM} ready after ${waited}s"

if [ "${1:-launch}" = "launch" ]; then
	shift || true
	exec ros2 launch umd_uas onboard.launch.py \
		uas:="${UAS_NUM}" \
		model:="${MODEL}" \
		sim:=true \
		params:="${SITE_PARAMS}${ONBOARD_PARAMS_FILE:+,${ONBOARD_PARAMS_FILE}}" \
		"$@"
fi

exec "$@"
