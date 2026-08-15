#!/usr/bin/env python3
"""Does this file load as a TensorRT engine?

Exit 0 if it does, 1 if it does not. Nothing on stdout.

The perception entrypoint asks this before it starts the second camera. That
the file exists answers nothing: nvinfer streams the plan to disk at the end
of a build, so a half-written plan has a size and a date like a finished one,
and an engine built for another GPU or another TensorRT is a complete file
that still cannot be read. Deserializing is the only honest test.

The DeepStream samples image ships no tensorrt Python module, so trtexec is
the primary path and the module is the fallback.
"""

import os
import subprocess
import sys

TRTEXEC = "/usr/src/tensorrt/bin/trtexec"


def loads_with_trtexec(path: str) -> bool:
    # One inference at batch settings from the plan. Loading the plan is the
    # test; the single run costs little and proves the engine executes.
    result = subprocess.run(
        [TRTEXEC, f"--loadEngine={path}",
         "--warmUp=0", "--iterations=1", "--duration=0", "--avgRuns=1"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)
    return result.returncode == 0


def loads_with_module(path: str) -> bool:
    import tensorrt as trt
    try:
        with open(path, "rb") as f:
            blob = f.read()
    except OSError:
        return False
    if not blob:
        return False
    # ERROR rather than WARNING: a plan that does not load is the expected
    # answer here, not a fault, and the caller reports it in its own words.
    logger = trt.Logger(trt.Logger.ERROR)
    try:
        return trt.Runtime(logger).deserialize_cuda_engine(blob) is not None
    except Exception:  # noqa: BLE001 - TensorRT raises its own types
        return False


def loads(path: str) -> bool:
    if not os.path.isfile(path):
        return False
    if os.path.isfile(TRTEXEC):
        try:
            return loads_with_trtexec(path)
        except (OSError, subprocess.TimeoutExpired):
            pass
    try:
        return loads_with_module(path)
    except ImportError:
        print("neither trtexec nor the tensorrt module is available",
              file=sys.stderr)
        return False


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <engine-file>", file=sys.stderr)
        return 2
    return 0 if loads(sys.argv[1]) else 1


if __name__ == "__main__":
    sys.exit(main())
