#!/usr/bin/env bash
# Stage only build inputs. Never traverse the surrounding workspace or copy
# model artifacts. rsync preserves unchanged files and removes deleted inputs.
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")/.."
command -v rsync >/dev/null || { echo 'Install rsync before building.' >&2; exit 1; }
ws=${ROS2_WS_DIR:-../ros2_ws}
deploy=${CHIMERA_DEPLOY_DIR:-../chimera-deploy}
root="$PWD/.build-contexts"
[ ! -L "$root" ] || { echo "Unexpected symlink: $root" >&2; exit 1; }
mkdir -p "$root"

stage() {
    local name=$1 source=$2
    shift 2
    [ -d "$source" ] || { echo "Missing build source: $source" >&2; exit 1; }
    # Refuse links at our generated destination before using --delete.
    [ ! -L "$root/$name" ] || { echo "Unexpected symlink: $root/$name" >&2; exit 1; }
    mkdir -p "$root/$name"
    rsync -a --delete --delete-excluded --safe-links \
        --exclude='.git' --exclude='.git*' --exclude='.vscode' \
        --exclude='__pycache__' --exclude='*.pyc' --exclude='.pytest_cache' \
        --exclude='build' --exclude='install' --exclude='log' --exclude='logs' \
        --exclude='*.egg-info' --exclude='*.engine' --exclude='*.onnx' \
        --exclude='*.pt' --exclude='*.o' --exclude='*.so' \
        "$@" "$source/" "$root/$name/"
}

# Python package inputs follow setup.py. MAVInsight's models/ is Python code;
# 5g_drone's models/ and perception_models/ are runtime data, not package inputs.
stage umd_uas "$ws/src/5g_drone" \
    --include='/umd_uas/***' --include='/config/***' --include='/launch/***' \
    --include='/resource/***' --include='/setup.*' --include='/package.xml' \
    --include='/pyproject.toml' --include='/LICENSE*' --exclude='*'
stage mavinsight "$ws/src/MAVInsight" \
    --include='/mavinsight/***' --include='/models/***' --include='/launch/***' \
    --include='/resource/***' --include='/vehicles/***' --include='/sensors/***' \
    --include='/sites/***' --include='/setup.*' --include='/package.xml' \
    --include='/pyproject.toml' --include='/LICENSE*' --exclude='*'
stage tracking_test "$ws/src/tracking_test_5g" \
    --include='/tracking_test/***' --include='/launch/***' --include='/config/***' \
    --include='/resource/***' --include='/setup.*' --include='/package.xml' \
    --include='/pyproject.toml' --include='/LICENSE*' --exclude='*'
for package in px4_msgs cdcl_umd_msgs; do
    stage "$package" "$ws/src/$package" \
        --include='/msg/***' --include='/srv/***' --include='/action/***' \
        --include='/cmake/***' --include='/CMakeLists.txt' --include='/package.xml' \
        --include='/LICENSE*' --exclude='*'
done
stage mavros "$deploy/submodules/mavros"
stage angles "$deploy/submodules/angles"
stage geographic_msgs "$deploy/submodules/geographic_info/geographic_msgs"
stage mavros_patch "$deploy/remote/mavros_patch"
stage yolo "$ws/src/5g_drone/config/deepstream/nvdsinfer_custom_impl_Yolo"

# Only the CUDA mapping affects compiler selection; unrelated edits to this
# module must not invalidate the parser. stdout supplies the deps build ARG.
cuda=$(python3 - "$ws/src/5g_drone/umd_uas/ds_ros_pipeline/infer_configs.py" "${DS_VERSION:-7.1}" <<'PY'
import ast
import sys
from pathlib import Path

tree = ast.parse(Path(sys.argv[1]).read_text())
for node in tree.body:
    if isinstance(node, ast.Assign) and any(
        isinstance(t, ast.Name) and t.id == '_DS_CUDA_VERSIONS' for t in node.targets
    ):
        versions = ast.literal_eval(node.value)
        for release, cuda in versions.items():
            if sys.argv[2].startswith(release):
                print(cuda)
                sys.exit(0)
raise SystemExit('No CUDA version declared for DeepStream ' + sys.argv[2])
PY
)
printf '%s\n' "$cuda" > "$root/yolo/cuda-version"
printf '%s\n' "$cuda"
