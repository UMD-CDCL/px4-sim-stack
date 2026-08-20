#!/usr/bin/env python3
"""Check parameter files the way rcl reads them, not the way YAML does.

A file that `yaml.safe_load` accepts can still stop a node dead. rcl's own
parser refuses a node whose `ros__parameters` holds nothing, and a comment is
nothing: delete the last parameter under a node and the file still parses,
still looks right, and the node exits with "Cannot have a value before
ros__parameters". That failure arrives at launch, once, in a log nobody is
watching yet.

Prints one line for each fault and answers non-zero when there is any.
"""

import argparse
import sys

import yaml


def empty_nodes(config, path=()):
    """Node paths whose ros__parameters block holds no parameter."""
    if not isinstance(config, dict):
        return
    if "ros__parameters" in config:
        body = config["ros__parameters"]
        if not isinstance(body, dict) or not body:
            yield "/".join(path) or "<root>"
        return
    for key, value in config.items():
        yield from empty_nodes(value, path + (str(key),))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+")
    arguments = parser.parse_args()

    faults = 0
    for name in arguments.files:
        try:
            with open(name) as handle:
                config = yaml.safe_load(handle) or {}
        except yaml.YAMLError as error:
            print(f"{name}: is not YAML: {error}")
            faults += 1
            continue
        for node in empty_nodes(config):
            print(f"{name}: {node} has a ros__parameters block with nothing in "
                  f"it, which rcl refuses")
            faults += 1
    return 1 if faults else 0


if __name__ == "__main__":
    sys.exit(main())
