#!/usr/bin/env bash
# Start the ground station for the whole fleet.
#
# UAS_FLEET is the identity, the way UAS_NUM is on the vehicle. It names one
# airframe model for each vehicle in UAS number order, so the fleet is written
# once and this derives the vehicle numbers, the namespaces and the models from
# it. UAS_BASE says which fleet: 10 is the simulated one, numbered from uas11
# and heard through ground-router. 0 is the real one, numbered from uas1 and
# heard through the native mavlink-router. See docs/uas-contract.md.
set -euo pipefail

UAS_FLEET=${UAS_FLEET:-chimera_v3 chimera_v3 chimera_v2 chimera_v2}
# The dash form keeps an empty value empty: SCENE= draws no terrain.
SCENE=${SCENE-lorton}
TERRAIN_DIR=${TERRAIN_DIR:-/terrain}

UAS_BASE=${UAS_BASE:-10}
# A simulated fleet is numbered from 11. The same number says whether the
# ground station scores against a scenario or against the real course.
if [ "${UAS_BASE}" -ge 10 ]; then SIM=true; else SIM=false; fi
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
	echo "UAS_FLEET has ${index} vehicles. A fleet is 1 to 9 of them." >&2
	exit 1
fi

# 60 + UAS_BASE: 70 beside a simulated fleet, 60 beside the real one, so a
# simulator and the fleet can share one network without discovering each
# other. scripts/fleet.sh derives the same number for the host.
export ROS_DOMAIN_ID=${GROUND_DOMAIN:-$((60 + UAS_BASE))}
export ROS_LOCALHOST_ONLY=0
# The air imagery profiles are Fast DDS XML, so the bridge needs this
# implementation. The profiles themselves are set on the image bridge process
# alone, by the launch file. Do not export them here: this shell is the parent
# of every node, and a launch-wide profile would give mavros and ds_node the
# flow controller as well.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# shellcheck disable=SC1091
. /usr/local/bin/ros-env.sh

# The same surface the vehicles localize against. The camera footprint and the
# live view projection meet the ground here, so a ground station on another
# surface would draw an outline that the vehicle's own detections fall outside
# of. An empty directory is not an error: every ray then meets the flat plane.
mkdir -p "${TERRAIN_DIR}"
rm -f "${TERRAIN_DIR}"/*.json
SURFACE="/scenes/worlds/${SCENE}_surface.json"
if [ -z "${SCENE}" ]; then
	echo "terrain: no scene. The footprint uses the flat plane."
elif [ -f "${SURFACE}" ]; then
	ln -s "${SURFACE}" "${TERRAIN_DIR}/"
	echo "terrain: ${SURFACE}"
else
	echo "terrain: no surface for scene '${SCENE}'. The footprint uses the flat plane." >&2
fi

# The same site the vehicles work out. The station draws the scene against the
# vehicle's home fix and recomputes the camera footprint, so it needs the datum
# the vehicle has. With no scene the geoid height is 0.0.
SITE_PARAMS=${SITE_PARAMS:-/camera/site.yaml}
source /usr/local/bin/site-params.sh

if [ "${1:-launch}" = "launch" ]; then
	shift || true
	exec ros2 launch umd_uas offboard.launch.py \
		uas:="${numbers#,}" \
		models:="${models#,}" \
		sim:="${SIM}" \
		params:="${SITE_PARAMS}" \
		"$@"
fi

exec "$@"
