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

# The simulated gimbal is wrong in three ways, and each one produces plausible
# wrong pointing rather than an error. patches/ corrects them and
# docs/px4-simulated-gimbal.md says what each one does. They are applied here,
# every time this step runs, so a tree cloned before a patch existed picks it
# up.
#
# What has been applied is RECORDED rather than detected. Two of these patches
# change the same lines of the same file, so once the later one is in, the
# earlier one no longer reverses cleanly and a tree that is correctly patched
# reads as unpatched. The record is a checksum for each patch, so an edited
# patch is applied again and an unchanged one is left alone.
patch_record() { echo "$1/.px4sim-patches"; }

apply_patches() { # dir
	local dir=$1 record patch name sum
	record=$(patch_record "$dir")
	touch "$record"
	for patch in "$PWD"/patches/px4-*.patch; do
		name=$(basename "$patch")
		sum=$(md5sum "$patch" | cut -d' ' -f1)
		if grep -qx "$sum $name" "$record"; then
			echo "    $name is already applied"
		elif git -C "$dir" apply "$patch" 2>/dev/null; then
			echo "$sum $name" >> "$record"
			echo "    applied $name"
		elif git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
			echo "$sum $name" >> "$record"
			echo "    $name was already in the tree. Recorded."
		else
			echo "    $name does not apply. See docs/px4-simulated-gimbal.md" >&2
		fi
	done
}

if [ "$WHAT" = "default" ] || [ "$WHAT" = "all" ] || [ "$WHAT" = "px4" ]; then
	clone_at https://github.com/PX4/PX4-Autopilot.git src/PX4-Autopilot "$PX4_REF"
	echo "==> Patching the simulated gimbal"
	apply_patches src/PX4-Autopilot
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
