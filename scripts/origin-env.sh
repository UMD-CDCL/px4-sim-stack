#!/usr/bin/env bash
# Emit the world origin and the survey marker as shell assignments.
#
# The origin is scene and scenario data. It is not .env data. A generated
# scenario carries home_* and fiducial_* lines, and every world file carries
# <spherical_coordinates>. This script reads the scenario first and falls
# back to the world, so SCENE and SCENARIO in .env carry the origin with
# them and no coordinate is written twice. px4sim evals the output.
#
#   origin-env.sh <scenario> [<scene>]
#
# Each argument is a name or a path. A name resolves under SCENES_DIR.
# Output, when an origin is found:
#
#   HOME_LAT, HOME_LON, HOME_ALT
#   FIDUCIAL_ENABLED, and FIDUCIAL_SURVEYED_LAT, _LON, _ALT when it is 1
#
# A source with no usable coordinates emits nothing, and the caller reports
# that. Silence is better than a wrong origin: a wrong one flies the whole
# mission somewhere else and looks correct while it does it.
set -euo pipefail

SCENES_DIR=${SCENES_DIR:-modules/sim/scenes}
# The sim merges the PX4 worlds with ours, so a stock world such as baylands
# is a valid scene. Each one carries its own <spherical_coordinates>, so the
# same search finds the origin of both kinds. Keep this list in the order the
# entrypoint links them: ours wins a name it shares. See modules/sim/entrypoint.sh.
WORLD_DIRS=${WORLD_DIRS:-"$SCENES_DIR/worlds src/PX4-Autopilot/Tools/simulation/gz/worlds"}
SCENARIO_DIRS=${SCENARIO_DIRS:-"$SCENES_DIR/scenarios"}

# Resolve a name or a path to a file. Prints nothing when neither exists.
resolve() {
	local given=$1 extension=$2 dir
	shift 2
	[ -n "$given" ] || return 0
	if [ -f "$given" ]; then
		echo "$given"
		return 0
	fi
	for dir in "$@"; do
		if [ -f "$dir/$given$extension" ]; then
			echo "$dir/$given$extension"
			return 0
		fi
	done
}

# shellcheck disable=SC2086
scenario_file=$(resolve "${1:-}" .yaml $SCENARIO_DIRS)
# shellcheck disable=SC2086
scene_file=$(resolve "${2:-}" .sdf $WORLD_DIRS)

# Drop the space around a value. Hand-written world files pad an element,
# and a padded number is not numeric.
trim() {
	sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//'
}

# A top-level `key: value` line from a scenario.
scenario_value() {
	[ -n "$scenario_file" ] || return 0
	sed -n "s/^$1:[[:space:]]*//p" "$scenario_file" | head -1 | trim
}

# The text of an element from a world file, such as <latitude_deg>.
scene_value() {
	[ -n "$scene_file" ] || return 0
	sed -n "s:.*<$1>\(.*\)</$1>.*:\1:p" "$scene_file" | head -1 | trim
}

# Only digits, dot and minus may reach eval. Anything else, including an
# empty value, fails and keeps its whole group silent.
numeric() {
	for value in "$@"; do
		case "$value" in
			'' | *[!0-9.-]*) return 1 ;;
		esac
	done
}

home_lat=$(scenario_value home_lat)
home_lon=$(scenario_value home_lon)
home_alt=$(scenario_value home_alt)

if ! numeric "$home_lat" "$home_lon" "$home_alt"; then
	home_lat=$(scene_value latitude_deg)
	home_lon=$(scene_value longitude_deg)
	home_alt=$(scene_value elevation)
fi

numeric "$home_lat" "$home_lon" "$home_alt" || exit 0

echo "HOME_LAT=$home_lat"
echo "HOME_LON=$home_lon"
echo "HOME_ALT=$home_alt"

# The marker is its own group. A scene can have an origin and no survey
# point, and then the frame correction stays off.
fiducial_lat=$(scenario_value fiducial_lat)
fiducial_lon=$(scenario_value fiducial_lon)
fiducial_alt=$(scenario_value fiducial_alt)

if numeric "$fiducial_lat" "$fiducial_lon" "$fiducial_alt"; then
	echo "FIDUCIAL_ENABLED=1"
	echo "FIDUCIAL_SURVEYED_LAT=$fiducial_lat"
	echo "FIDUCIAL_SURVEYED_LON=$fiducial_lon"
	echo "FIDUCIAL_SURVEYED_ALT=$fiducial_alt"
else
	echo "FIDUCIAL_ENABLED=0"
fi
