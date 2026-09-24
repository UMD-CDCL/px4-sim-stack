#!/usr/bin/env bash
set -euo pipefail

# Simulator-only compatibility shim.  The Blackwell engines are built with
# TensorRT 10.9, while the optional Python bindings in the onboard image are
# 10.3 and cannot inspect them.  DeepStream's native nvinfer can load them;
# let it perform the authoritative validation.
infer_configs=/home/user/ros2_ws/install/umd_uas/lib/python3.10/site-packages/umd_uas/ds_ros_pipeline/infer_configs.py

python3 - "$infer_configs" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()
old = '''        if "No module named \'tensorrt\'" in output:
            raise _EngineInspectionUnavailable(ending)
        raise RuntimeError(f"could not inspect TensorRT engine {engine}: {ending}")'''
new = '''        if "No module named \'tensorrt\'" in output or "could not deserialize the engine" in output:
            raise _EngineInspectionUnavailable(ending)
        raise RuntimeError(f"could not inspect TensorRT engine {engine}: {ending}")'''
if old not in text:
    raise SystemExit("unexpected infer_configs.py layout")
path.write_text(text.replace(old, new, 1))
PY

export HOME=/home/user
export USER=user
export LOGNAME=user
exec runuser --preserve-environment -u user -- /usr/local/bin/entrypoint.sh "$@"
