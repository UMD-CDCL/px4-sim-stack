#!/usr/bin/env python3
"""Check a Foxglove layout, and list the topics it draws.

A panel's settings are validated whole. One value outside the set Foxglove
accepts fails the parse, and the panel then falls back to its defaults: a Map
panel with a bad point shape draws a street map with no colors and follows
nothing, which reads as a panel that was never configured rather than as a
value that was rejected. Nothing warns, so this does.

The sets below are Foxglove's own, read out of the app it ships
(resources/app.asar). Refresh them from a new version the same way.

With --topics it prints every topic the layout names instead, one per line,
so a stage can ask the bridge whether each one is really there.
"""

import argparse
import json
import sys

MAP_LAYERS = ("map", "satellite", "custom", "shaded-relief")
POINT_DISPLAY_MODES = ("dot", "pin", "diamond", "square", "plus", "cross",
                       "arrowhead")
HISTORY_MODES = ("all", "none", "previous", "last-n-seconds")
MESH_UP_AXES = ("y_up", "z_up")
# Keys the Map panel used to take and no longer reads. A layout that still
# carries one looks configured and is not.
MAP_DEAD_KEYS = ("customTileProviders",)


def map_problems(name, config):
    if config.get("layer") not in MAP_LAYERS:
        yield f"{name}: layer {config.get('layer')!r} is not one of {MAP_LAYERS}"
    for key in MAP_DEAD_KEYS:
        if key in config:
            yield f"{name}: {key} is not read; the custom layer takes customTileUrl"
    for topic, settings in config.get("topicConfig", {}).items():
        mode = settings.get("pointDisplayMode")
        if mode is not None and mode not in POINT_DISPLAY_MODES:
            yield (f"{name}: {topic} pointDisplayMode {mode!r} is not one of "
                   f"{POINT_DISPLAY_MODES}, so the whole panel falls back")
        history = settings.get("historyMode")
        if history is not None and history not in HISTORY_MODES:
            yield f"{name}: {topic} historyMode {history!r} is not one of {HISTORY_MODES}"


def scene_problems(name, config):
    axis = config.get("scene", {}).get("meshUpAxis")
    if axis is not None and axis not in MESH_UP_AXES:
        yield f"{name}: meshUpAxis {axis!r} is not one of {MESH_UP_AXES}"


def problems(layout):
    for name, config in layout.get("configById", {}).items():
        if name.startswith("map!"):
            yield from map_problems(name, config)
        elif name.startswith("3D!"):
            yield from scene_problems(name, config)


def topics(layout):
    """Every topic the layout draws, in the order the panels are declared."""
    found = []
    disabled = set()

    def remember(value):
        if isinstance(value, str) and value.startswith("/") and value not in found:
            found.append(value)

    for name, config in layout.get("configById", {}).items():
        if name.startswith("map!"):
            disabled.update(config.get("disabledTopics", []))
            for key in ("topicColors", "topicConfig"):
                for topic in config.get(key, {}):
                    remember(topic)
            remember(config.get("followTopic"))
        elif name.startswith("3D!"):
            for topic in config.get("topics", {}):
                remember(topic)
        elif name.startswith("Image!"):
            remember(config.get("imageTopic"))
            remember(config.get("calibrationTopic"))
        elif name.startswith("Plot!"):
            for path in config.get("paths", []):
                remember(str(path.get("value", "")).split(".")[0])
        elif name.startswith("RawMessages!"):
            remember(str(config.get("topicPath", "")).split(".")[0])
    return [topic for topic in found if topic not in disabled]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout")
    parser.add_argument("--topics", action="store_true",
                        help="list the topics the layout draws")
    arguments = parser.parse_args()

    with open(arguments.layout) as handle:
        layout = json.load(handle)

    if arguments.topics:
        for topic in topics(layout):
            print(topic)
        return 0

    found = list(problems(layout))
    for problem in found:
        print(problem)
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
