#!/usr/bin/env bash
# Clone the upstream sources into ./src at the versions pinned in .env.
# These trees are yours to edit. The containers build them in place.
#
#   ./scripts/bootstrap.sh            px4 and the scenes
#   ./scripts/bootstrap.sh qgc        add the QGroundControl source
#   ./scripts/bootstrap.sh all        everything
set -euo pipefail

cd "$(dirname "$0")/.."
[ -f .env ] && set -a && . ./.env && set +a

PX4_REF=${PX4_REF:-v1.17.0}
QGC_REF=${QGC_REF:-v5.0.8}
WHAT=${1:-default}

mkdir -p src logs/{px4,qgc,onboard,offboard}

clone_at() { # url dir ref
	local url=$1 dir=$2 ref=$3
	if [ -d "$dir/.git" ]; then
		echo "==> $dir exists. Leaving it alone."
		echo "    Current ref: $(git -C "$dir" describe --tags --always 2>/dev/null || echo unknown)"
		return
	fi
	echo "==> Cloning $url at $ref into $dir"
	git clone --branch "$ref" --recurse-submodules --shallow-submodules \
	          --jobs 8 "$url" "$dir"
}

if [ "$WHAT" = "default" ] || [ "$WHAT" = "all" ] || [ "$WHAT" = "px4" ]; then
	clone_at https://github.com/PX4/PX4-Autopilot.git src/PX4-Autopilot "$PX4_REF"
fi

# The flight code is not cloned here. The onboard and offboard images build
# 5g_drone, cdcl_umd_msgs and MAVInsight from ROS2_WS_DIR, a checkout outside
# this repository, so it stays where its own git remote put it.
if [ ! -d "${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone" ]; then
	echo "==> No 5g_drone at ${ROS2_WS_DIR:-../ros2_ws}/src."
	echo "    The onboard and offboard images build from there. Check it out,"
	echo "    or set ROS2_WS_DIR in .env to where it already is."
fi

if [ "$WHAT" = "all" ] || [ "$WHAT" = "qgc" ]; then
	clone_at https://github.com/mavlink/qgroundcontrol.git src/qgroundcontrol "$QGC_REF"
fi

# Git carries the scene sources (modules/scenegen/data), not the worlds
# built from them. Build them all here, so a fresh clone starts with every
# scene in place. The first run builds the scenegen image, which takes a
# few minutes.
if [ "$WHAT" = "default" ] || [ "$WHAT" = "all" ]; then
	if ls modules/scenegen/data/*/scene.json >/dev/null 2>&1; then
		echo "==> Building the scenes in modules/scenegen/data"
		./px4sim genscene build-all \
			|| echo "==> Scene build failed. Run it later: ./px4sim genscene build-all"
	fi
fi

echo ""
echo "Done. Sources:"
for d in src/*/; do
	[ -d "$d" ] || continue
	ref=$(git -C "$d" describe --tags --always 2>/dev/null || echo "not a git tree")
	printf '  %-28s %s\n' "$d" "$ref"
done
