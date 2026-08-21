#!/usr/bin/env bash
# Choose what this machine needs to run DeepStream, and print it.
#
# The stack is ROS 2 Humble on Ubuntu 22.04, everywhere, because that is what
# the aircraft is: a Jetson Orin on 22.04 with DeepStream 7.1. The ground
# station has to decode what that aircraft sends, and cdcl_umd_msgs does not
# survive a distribution change -- Jazzy adds a field to sensor_msgs/Range,
# which sits inside TargetBoxArray ahead of the box array, so a mixed pair
# reports every frame with an empty box list and no error. So the DeepStream
# release and the ROS distribution are not choices this script makes.
#
# One thing does vary, and it is TensorRT.
#
# TensorRT only emits kernels for the GPU architectures its release knows.
# DeepStream 7.1 ships TensorRT 10.3, which stops at Hopper. On a Blackwell
# card it parses the ONNX, says `Unsupported SM: 0xc00`, builds no engine and
# takes the process down. Everything around it carries on: the streams decode,
# the operator sees video, and no box is ever drawn.
#
# The fix is not a newer DeepStream -- 8.0 and 9.0 are Ubuntu 24.04, which is
# Jazzy, which breaks the fleet. It is a newer TensorRT inside the same 22.04
# DeepStream 7.1 image. NVIDIA packages TensorRT 10.16 for jammy, the soname
# is unchanged, and DeepStream's own nvinfer builds and loads engines against
# it. So a machine too new for the stock TensorRT gets a newer one, and
# nothing else about it moves.
#
# Precedence, highest first:
#   DS_IMAGE     an explicit image. Honoured as given.
#   DS_VERSION   pins the release. 8.0 and 9.0 are reachable this way and
#                bring Jazzy with them, which this script says out loud.
#   auto         DeepStream 7.1, plus a TensorRT the GPU can use.
#
# Prints shell assignments that both front doors evaluate:
#
#   DS_VERSION=7.1
#   DS_IMAGE=nvcr.io/nvidia/deepstream:7.1-samples-multiarch
#   DS_TAG=7.1-trt10.16
#   ROS_DISTRO=humble
#   DS_TRT_VERSION=10.16.1.11-1+cuda12.9
#
set -uo pipefail

DS_FLAVOUR=${DS_FLAVOUR:-samples-multiarch}

# release : ROS distro : Ubuntu codename : minimum driver
#
# 7.1 is the last release on 22.04 and so the last that carries Humble. The
# other two are here to be pinned deliberately, not chosen.
SUPPORTED=(
	"7.1:humble:jammy:535.183"
	"8.0:jazzy:noble:570.133"
	"9.0:jazzy:noble:590.48"
)
DEFAULT_RELEASE=7.1

# The TensorRT a GPU needs, newest architecture first. Compute capabilities are
# times ten, so 90 is Hopper and 120 is Blackwell.
#
#   up to  90  the TensorRT 10.3 DeepStream 7.1 already ships. Nothing to do,
#              and the Orin (87) lands here, so the aircraft image is untouched.
#   above  90  TensorRT 10.9 for CUDA 12.8, from the jammy CUDA repository.
#
# 10.9 and not the newest, deliberately. 10.8 is the first release that knows
# Blackwell and 10.9 is the one DeepStream 8.0 itself ships, so it is the least
# distance to travel from the 10.3 this image was built around. Newer releases
# keep deleting API that DeepStream 7.1-era code still calls: at 10.16 the
# vendored DeepStream-Yolo parser no longer compiles, because
# NetworkDefinitionCreationFlag::kEXPLICIT_BATCH, IBuilder::platformHasFastFp16
# and BuilderFlag::kINT8 are all gone. Take the oldest TensorRT that covers the
# card, not the newest that exists.
#
# The second field is the CUDA runtime that TensorRT build links, as its apt
# package suffix. Both are installed together or neither is.
TRT_STOCK_MAX_CAP=90
TRT_UPGRADE_VERSION=10.9.0.34-1+cuda12.8
TRT_UPGRADE_CUDA=12-8
# The newest architecture the upgrade knows. Past this, say so rather than
# build an image that cannot infer.
TRT_UPGRADE_MAX_CAP=120

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

# The compute capability of the first GPU, times ten: 8.9 becomes 89 and 12.0
# becomes 120, so the table compares with a plain integer test.
gpu_capability() {
	command -v nvidia-smi >/dev/null 2>&1 || return 1
	local cap
	cap=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1)
	case "${cap:-}" in
	[0-9]*.[0-9]*) echo "$(( ${cap%%.*} * 10 + ${cap##*.} ))" ;;
	*) return 1 ;;
	esac
}

row_field() { echo "$1" | cut -d: -f"$2"; }
image_for() { echo "nvcr.io/nvidia/deepstream:$1-$DS_FLAVOUR"; }

row_for() {
	local want=$1 row
	for row in "${SUPPORTED[@]}"; do
		[ "$(row_field "$row" 1)" = "$want" ] && { echo "$row"; return 0; }
	done
	return 1
}

RELEASES=$(for row in "${SUPPORTED[@]}"; do row_field "$row" 1; done | paste -sd' ' -)

version=""; image=""; distro=""; codename=""; minimum=""
trt=""; trt_short=""; reason=""; cap=""; drv=""

drv=$(driver_version) || true
cap=$(gpu_capability) || true

# ------------------------------------------------------------ which release
if [ -n "${DS_IMAGE:-}" ] && [ "${DS_IMAGE}" != "auto" ]; then
	ref=${DS_IMAGE##*:}
	version=${ref%%-*}
	row=$(row_for "$version") ||
		die "DS_IMAGE=$DS_IMAGE has the tag '$ref', which names no release this stack knows ($RELEASES)."
	image=$DS_IMAGE
	reason="DS_IMAGE names it"
elif [ -n "${DS_VERSION:-}" ] && [ "${DS_VERSION}" != "auto" ]; then
	row=$(row_for "$DS_VERSION") || die "DS_VERSION=$DS_VERSION is not one of $RELEASES"
	version=$DS_VERSION
	image=$(image_for "$version")
	reason="DS_VERSION pins it"
else
	row=$(row_for "$DEFAULT_RELEASE")
	version=$DEFAULT_RELEASE
	image=$(image_for "$version")
	reason="the release the aircraft runs, and the last one that carries Humble"
fi

distro=$(row_field "$row" 2)
codename=$(row_field "$row" 3)
minimum=$(row_field "$row" 4)

# ------------------------------------------------------------ which TensorRT
# Only 7.1 is ever short of a TensorRT the GPU can use. 8.0 and 9.0 ship 10.9
# and 10.14, which already know Blackwell.
if [ "$version" = 7.1 ] && [ -n "${cap:-}" ] && [ "$cap" -gt "$TRT_STOCK_MAX_CAP" ]; then
	trt=$TRT_UPGRADE_VERSION
	trt_short=$(echo "${trt%%-*}" | cut -d. -f1,2)
	reason="$reason; TensorRT ${trt_short} because compute capability ${cap%?}.${cap#${cap%?}} is past the 10.3 it ships"
fi

tag=$version
[ -n "$trt" ] && tag="$version-trt${trt_short}"

# ------------------------------------------------------------------ warnings
if [ -n "${drv:-}" ] && ! at_least "$drv" "$minimum"; then
	echo "ds-select: DeepStream $version needs driver $minimum and this machine has $drv." >&2
	echo "        CUDA will not initialize and the container stops at its first call." >&2
fi
if [ "$distro" != humble ]; then
	echo "ds-select: DeepStream $version is Ubuntu $codename, so this builds ROS 2 ${distro^}." >&2
	echo "        cdcl_umd_msgs does not decode between Humble and ${distro^}: a TargetBoxArray" >&2
	echo "        crossing that line arrives with an empty box list and no error, so a vehicle" >&2
	echo "        and a ground station either side of it silently share no detections." >&2
	echo "        Unset DS_VERSION to build 7.1 and Humble, which is what the aircraft runs." >&2
fi
if [ -n "${cap:-}" ] && [ "$cap" -gt "$TRT_UPGRADE_MAX_CAP" ]; then
	echo "ds-select: compute capability ${cap%?}.${cap#${cap%?}} is past TensorRT ${TRT_UPGRADE_VERSION%%-*}." >&2
	echo "        Raise TRT_UPGRADE_VERSION in this file to one that knows this GPU." >&2
fi

reason=${DS_REASON:-$reason}

case "${1:-}" in
--version)  echo "$version" ;;
--image)    echo "$image" ;;
--tag)      echo "$tag" ;;
--distro)   echo "$distro" ;;
--codename) echo "$codename" ;;
--trt)      echo "$trt" ;;
--explain)
	printf 'DeepStream %s with ROS 2 %s' "$version" "${distro^}"
	[ -n "$trt" ] && printf ', TensorRT %s' "${trt_short}"
	printf ': %s\n' "$reason"
	;;
"")
	echo "DS_VERSION=$version"
	echo "DS_IMAGE=$image"
	echo "DS_TAG=$tag"
	echo "ROS_DISTRO=$distro"
	echo "DS_CODENAME=$codename"
	echo "DS_TRT_VERSION=$trt"
	echo "DS_TRT_CUDA=$([ -n "$trt" ] && echo "$TRT_UPGRADE_CUDA")"
	printf 'DS_REASON=%q\n' "$reason"
	;;
*) die "unknown option $1" ;;
esac
