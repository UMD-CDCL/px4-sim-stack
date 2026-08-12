#!/usr/bin/env bash
# Clone the upstream sources into ./src at the versions pinned in .env.
# These trees are yours to edit. The containers build them in place.
#
#   ./scripts/bootstrap.sh            px4 and the ROS workspace
#   ./scripts/bootstrap.sh qgc        add the QGroundControl source
#   ./scripts/bootstrap.sh all        everything
set -euo pipefail

cd "$(dirname "$0")/.."
[ -f .env ] && set -a && . ./.env && set +a

PX4_REF=${PX4_REF:-v1.17.0}
QGC_REF=${QGC_REF:-v5.0.8}
WHAT=${1:-default}

mkdir -p src logs/{px4,qgc,ros,perception}

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

if [ "$WHAT" = "default" ] || [ "$WHAT" = "all" ] || [ "$WHAT" = "ros" ]; then
	if [ ! -d src/ros2_ws ]; then
		echo "==> Creating the ROS 2 workspace at src/ros2_ws"
		mkdir -p src/ros2_ws/src
	else
		echo "==> src/ros2_ws exists. Leaving it alone."
	fi
fi

if [ "$WHAT" = "all" ] || [ "$WHAT" = "qgc" ]; then
	clone_at https://github.com/mavlink/qgroundcontrol.git src/qgroundcontrol "$QGC_REF"
fi

echo ""
echo "Done. Sources:"
for d in src/*/; do
	[ -d "$d" ] || continue
	ref=$(git -C "$d" describe --tags --always 2>/dev/null || echo "not a git tree")
	printf '  %-28s %s\n' "$d" "$ref"
done
