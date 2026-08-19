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
read -r -a _gimbal_hfov <<< "${UAS_GIMBAL_HFOV_DEG:-27.45 27.45 85.25 25.98}"
python3 /usr/local/bin/sim_calibration.py \
	--model "/scenes/models/${AIRFRAME}/model.sdf" \
	--sensor gimbal_camera \
	--hfov-deg "${_gimbal_hfov[$SLOT]:-${_gimbal_hfov[0]}}" \
	--out "${CAMERA_DIR:-/camera}/gimbal.yaml"

if [ "${1:-launch}" = "launch" ]; then
	shift || true
	exec ros2 launch umd_uas onboard.launch.py \
		uas:="${UAS_NUM}" \
		model:="${MODEL}" \
		sim:=true \
		params:="${ONBOARD_PARAMS_FILE:-}" \
		"$@"
fi

exec "$@"
