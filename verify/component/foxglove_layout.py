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
so a stage can ask the bridge whether each one is really there. With
--services it prints the services its buttons call, which a stage asks the
same question about: a button that names a service nobody offers looks
configured and fails under the operator's hand.
"""

import argparse
import json
import re
import sys

MAP_LAYERS = ("map", "satellite", "custom", "shaded-relief")
POINT_DISPLAY_MODES = ("dot", "pin", "diamond", "square", "plus", "cross",
                       "arrowhead")
HISTORY_MODES = ("all", "none", "previous", "last-n-seconds")
MESH_UP_AXES = ("y_up", "z_up")
# Keys the Map panel used to take and no longer reads. A layout that still
# carries one looks configured and is not.
MAP_DEAD_KEYS = ("customTileProviders",)
# A Publish panel names the message type the way the bridge does, with the
# msg folder in the middle: std_msgs/msg/Float32, not std_msgs/Float32.
DATATYPE = re.compile(r"^[a-z][a-z0-9_]*/msg/[A-Za-z][A-Za-z0-9]*$")


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


def button_problems(name, config, field, payload_field):
    """What stops a button panel from doing anything when it is pressed.

    Its target has to be a real name, and the body it sends has to be JSON.
    Foxglove writes both by hand here, and it reports neither as wrong.
    """
    target = config.get(field)
    if not isinstance(target, str) or not target.startswith("/"):
        yield f"{name}: {field} {target!r} is not an absolute name"
    payload = config.get(payload_field)
    if payload is not None:
        try:
            json.loads(payload)
        except (TypeError, ValueError) as error:
            yield f"{name}: {payload_field} is not JSON, so the button sends nothing ({error})"


def publish_problems(name, config):
    yield from button_problems(name, config, "topicName", "value")
    datatype = config.get("datatype")
    if not isinstance(datatype, str) or not DATATYPE.match(datatype):
        yield f"{name}: datatype {datatype!r} is not a pkg/msg/Type name"


def placement_problems(layout):
    """Panels the tree and the settings disagree about.

    A panel with settings and no place in the tree is invisible. A panel in the
    tree with no settings draws its defaults, which is the same silence a
    rejected value produces.
    """
    placed = set()

    def walk(node):
        if isinstance(node, str):
            placed.add(node)
        elif isinstance(node, dict):
            walk(node.get("first"))
            walk(node.get("second"))

    walk(layout.get("layout"))
    configured = set(layout.get("configById", {}))
    for name in sorted(configured - placed):
        yield f"{name}: has settings but no place in the layout, so it is not drawn"
    for name in sorted(placed - configured):
        yield f"{name}: is in the layout with no settings, so it draws its defaults"


def problems(layout):
    for name, config in layout.get("configById", {}).items():
        if name.startswith("map!"):
            yield from map_problems(name, config)
        elif name.startswith("3D!"):
            yield from scene_problems(name, config)
        elif name.startswith("Publish!"):
            yield from publish_problems(name, config)
        elif name.startswith("CallService!"):
            yield from button_problems(name, config, "serviceName",
                                       "requestPayload")
    yield from placement_problems(layout)


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
            # An image panel is the 3D panel in image mode, so it draws
            # annotations and 3D topics too. Missing either leaves the picture
            # bare with nothing said.
            for key in ("annotations", "topics"):
                for topic in config.get(key, {}):
                    remember(topic)
            for topic in config.get("imageMode", {}).get("annotations", {}):
                remember(topic)
        elif name.startswith("Plot!"):
            for path in config.get("paths", []):
                remember(str(path.get("value", "")).split(".")[0])
        elif name.startswith("RawMessages!"):
            remember(str(config.get("topicPath", "")).split(".")[0])
    return [topic for topic in found if topic not in disabled]


def services(layout):
    """Every service the layout's buttons call, in the order they are declared."""
    found = []
    for name, config in layout.get("configById", {}).items():
        if not name.startswith("CallService!"):
            continue
        called = config.get("serviceName")
        if isinstance(called, str) and called.startswith("/") and called not in found:
            found.append(called)
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("layout")
    parser.add_argument("--topics", action="store_true",
                        help="list the topics the layout draws")
    parser.add_argument("--services", action="store_true",
                        help="list the services the layout's buttons call")
    arguments = parser.parse_args()

    with open(arguments.layout) as handle:
        layout = json.load(handle)

    if arguments.topics:
        for topic in topics(layout):
            print(topic)
        return 0

    if arguments.services:
        for service in services(layout):
            print(service)
        return 0

    found = list(problems(layout))
    for problem in found:
        print(problem)
    return 1 if found else 0


if __name__ == "__main__":
    sys.exit(main())
