#!/usr/bin/env python3
"""Does this file load as a TensorRT engine?

Exit 0 if it does, 1 if it does not. Nothing on stdout.

The perception entrypoint asks this before it starts the second camera. That
the file exists answers nothing: nvinfer streams the plan to disk at the end of
a build, so a half-written plan has a size and a date like a finished one, and
an engine built for another GPU or another TensorRT is a complete file that
still cannot be read. Both fail the same way, several seconds into the next
start, as:

    Serialization assertion plan.header.size == blobSize failed

Deserializing is the only honest test, and it costs about a second.
"""

import sys

import tensorrt as trt


def loads(path: str) -> bool:
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


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <engine-file>", file=sys.stderr)
        return 2
    return 0 if loads(sys.argv[1]) else 1


if __name__ == "__main__":
    sys.exit(main())
