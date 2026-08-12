#!/usr/bin/env bash
# Choose the DeepStream release this machine can run.
#
# The driver sets the ceiling. DeepStream 9.0 needs NVIDIA driver 590.48, and
# DeepStream 8.0 needs 570.133. A DeepStream newer than the driver does not
# degrade, it fails to initialize CUDA, so the choice is a fact about the
# machine rather than a preference.
#
# That fact used to live in .env as one pinned image name. It travelled with
# the repository to machines with a different driver, where it was wrong and
# said nothing about being wrong. This script reads the driver instead.
#
# Precedence, highest first:
#   DS_IMAGE     an explicit image. Honoured as given, and the version and tag
#                are read back out of it.
#   DS_VERSION   8.0 or 9.0. Pins the release without naming an image, which is
#                what reproduces a result on another machine.
#   auto         read the driver and take the newest release it supports.
#
# Prints three shell assignments. Both front doors evaluate them, so the ROS
# and perception services agree on which DeepStream is in play:
#
#   DS_VERSION=9.0
#   DS_IMAGE=nvcr.io/nvidia/deepstream:9.0-samples-multiarch
#   DS_TAG=ds9
#
# DS_TAG names the built image. Without it, a machine that moves between
# releases builds a DeepStream 9.0 image over its DeepStream 8.0 one under the
# same name, and the only symptom is a container that will not start.
set -uo pipefail

DS_FLAVOUR=${DS_FLAVOUR:-samples-multiarch}

# release:minimum driver, newest first.
SUPPORTED=("9.0:590.48" "8.0:570.133")
FALLBACK=8.0

die() { echo "ds-select: $*" >&2; exit 1; }

# Is version $1 at least version $2? Compares field by field, so 595.84 beats
# 590.48 and 1000.1 does not lose to 99.1 the way a string compare would.
at_least() {
	[ "$(printf '%s\n%s\n' "$2" "$1" | sort -V | head -1)" = "$2" ]
}

driver_version() {
	command -v nvidia-smi >/dev/null 2>&1 || return 1
	nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1
}

image_for() { echo "nvcr.io/nvidia/deepstream:$1-$DS_FLAVOUR"; }
tag_for()   { echo "ds${1%%.*}"; }

resolve() {
	# An explicit image wins. Read the release back out of its tag so the built
	# image is still named for the release inside it.
	if [ -n "${DS_IMAGE:-}" ] && [ "${DS_IMAGE}" != "auto" ]; then
		local ref=${DS_IMAGE##*:}
		version=${ref%%-*}
		case "$version" in
		[0-9]*.[0-9]*) : ;;
		*) version=$FALLBACK ;;   # an image whose tag says nothing about a release
		esac
		image=$DS_IMAGE
		reason="DS_IMAGE names it"
		return
	fi

	local want=${DS_VERSION:-auto}
	if [ "$want" != "auto" ] && [ -n "$want" ]; then
		local known=0
		for entry in "${SUPPORTED[@]}"; do
			[ "${entry%%:*}" = "$want" ] && known=1
		done
		[ "$known" = 1 ] || die "DS_VERSION=$want is not one of ${SUPPORTED[*]%%:*}"
		version=$want
		image=$(image_for "$version")
		reason="DS_VERSION pins it"
		return
	fi

	local drv
	drv=$(driver_version)
	if [ -z "$drv" ]; then
		version=$FALLBACK
		image=$(image_for "$version")
		reason="no nvidia-smi, so the oldest supported release"
		return
	fi

	for entry in "${SUPPORTED[@]}"; do
		local release=${entry%%:*} minimum=${entry##*:}
		if at_least "$drv" "$minimum"; then
			version=$release
			image=$(image_for "$version")
			reason="driver $drv supports it, and it is the newest that does"
			return
		fi
	done

	version=$FALLBACK
	image=$(image_for "$version")
	reason="driver $drv is below ${SUPPORTED[-1]##*:}, so DeepStream will not start"
}

version=""; image=""; reason=""
resolve
tag=$(tag_for "$version")

# A caller that already resolved this exports the answer, and this script then
# runs again inside it and sees its own DS_IMAGE. Deriving the reason a second
# time would report "DS_IMAGE names it" for a choice that came from the driver.
# Carrying the reason keeps the second answer equal to the first.
reason=${DS_REASON:-$reason}

case "${1:-}" in
--version) echo "$version" ;;
--image)   echo "$image" ;;
--tag)     echo "$tag" ;;
--explain) echo "DeepStream $version: $reason" ;;
"")
	echo "DS_VERSION=$version"
	echo "DS_IMAGE=$image"
	echo "DS_TAG=$tag"
	printf 'DS_REASON=%q\n' "$reason"
	;;
*) die "unknown option $1" ;;
esac
