#!/usr/bin/env bash
# Start the ground station for the whole fleet.
#
# UAS_FLEET is the identity, the way UAS_NUM is on the vehicle. It names one
# airframe model for each vehicle in UAS number order, so the fleet is written
# once and this derives the vehicle numbers, the namespaces and the models from
# it. The ground station runs on domain 60 for every vehicle.
# See docs/uas-contract.md.
set -euo pipefail

UAS_FLEET=${UAS_FLEET:-chimera_v3 chimera_v3 chimera_v2 chimera_v2}
SCENE=${SCENE:-recon_field}
TERRAIN_DIR=${TERRAIN_DIR:-/terrain}

UAS_BASE=${UAS_BASE:-10}
numbers=""
models=""
index=0
for airframe in ${UAS_FLEET}; do
	index=$((index + 1))
	case "${airframe}" in
		*v3) model=v3 ;;
		*v2) model=v2 ;;
		*)
			echo "UAS_FLEET entry '${airframe}' names no known model. Use chimera_v2 or chimera_v3." >&2
			exit 1
			;;
	esac
	# A simulated vehicle is its real counterpart plus UAS_BASE, so the ground
	# station namespaces match the vehicles. Without the offset this launched
	# /uas1 to /uas4 while the fleet published /uas11 to /uas14, and the two
	# sides simply never met.
	numbers="${numbers},$((UAS_BASE + index))"
	models="${models},${model}"
done

if [ "${index}" -lt 1 ] || [ "${index}" -gt 9 ]; then
	echo "UAS_FLEET has ${index} vehicles. The simulator numbers them 11 to 19." >&2
	exit 1
fi

# 70, not the fielded 60. A simulated vehicle is its real counterpart plus ten
# everywhere, and its ground station follows, so a simulator and the fleet can
# share one network without discovering each other.
export ROS_DOMAIN_ID=${GROUND_DOMAIN:-70}
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

# The same surface the vehicles localize against. The camera footprint and the
# live view projection meet the ground here, so a ground station on another
# surface would draw an outline that the vehicle's own detections fall outside
# of. An empty directory is not an error: every ray then meets the flat plane.
mkdir -p "${TERRAIN_DIR}"
rm -f "${TERRAIN_DIR}"/*.json
SURFACE="/scenes/worlds/${SCENE}_surface.json"
if [ -f "${SURFACE}" ]; then
	ln -s "${SURFACE}" "${TERRAIN_DIR}/"
	echo "terrain: ${SURFACE}"
else
	echo "terrain: no surface for scene '${SCENE}'. The footprint uses the flat plane." >&2
fi

# The same site the vehicles work out. The station draws the scene against the
# vehicle's home fix and recomputes the camera footprint, so it needs the datum
# the vehicle has.
SITE_PARAMS=${SITE_PARAMS:-/camera/site.yaml}
source /usr/local/bin/site-params.sh

if [ "${1:-launch}" = "launch" ]; then
	shift || true
	exec ros2 launch umd_uas offboard.launch.py \
		uas:="${numbers#,}" \
		models:="${models#,}" \
		params:="${SITE_PARAMS}" \
		"$@"
fi

exec "$@"
