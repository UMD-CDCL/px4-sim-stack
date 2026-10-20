#!/usr/bin/env bash
set -euo pipefail

stack_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source_dir=$(realpath "${QGC_SOURCE_DIR:-$stack_dir/../qgroundcontrol-5.1.4}")
build_image=${QGC_BUILD_IMAGE:-qgc-planner-build:5.1.4}

docker image inspect "$build_image" >/dev/null
test -f "$source_dir/custom-planner/CMakeLists.txt"
docker run --rm --user "$(id -u):$(id -g)" \
    --network "${PX4SIM_BUILD_NETWORK:-none}" \
    --mount "type=bind,source=$HOME,target=$HOME" \
    -e HOME -w "$source_dir" "$build_image" bash -euc '
        cmake -S . -B build -DQGC_STABLE_BUILD=ON
        cmake --build build --parallel "$1"
        stage=$(mktemp -d "$PWD/build/runtime.XXXXXX")
        trap '\''rm -rf "$stage"'\'' EXIT
        cmake --install build --prefix "$stage" --component Unspecified
        rm -rf build/runtime
        mv "$stage" build/runtime
    ' _ "${QGC_BUILD_JOBS:-12}"
