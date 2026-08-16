"""How detections and ground truth are drawn, in every view at once.

The color rules and marker shapes live here, and only here, because they
change often. Edit this file and the image boxes, the detection dots, and
the truth bubbles all follow. None of it is a ROS parameter on purpose.
"""

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# ------------------------------------------------------- detections, per camera
# TP: the estimate lies within the gate of a ground truth target.
# MISLOCALIZED: the viewing ray to the estimate crosses the gate of a
#   target, but the estimate lies within the gate of none.
# FP: the ray crosses the gate of no target.
# An FN, a target in view that no verdict names, has no estimate and gets
# no mark. It appears as the ground truth bubble turning red.
# The Map panel colors in foxglove/px4-sim-stack.json mirror these values.
VERDICT_COLOR = {
    "TP": (0.18, 0.80, 0.44, 0.95),            # green
    "MISLOCALIZED": (0.95, 0.77, 0.06, 0.95),  # yellow
    "FP": (0.91, 0.30, 0.24, 0.95),            # red
}
# Verdicts drawn as a flat cross on the ground instead of a dot.
CROSS_VERDICTS = frozenset({"MISLOCALIZED", "FP"})
# Image boxes before the first verdict arrives.
UNJUDGED_COLOR = (0.75, 0.75, 0.78, 1.0)

# The same three verdicts on the Map panel, as CSS colors for GeoJSON. True
# red, yellow and green rather than the muted tones above: a flat map pin
# carries no shading to soften, and the operator reads it at a glance.
MAP_VERDICT_COLOR = {
    "TP": "#00ff00",
    "MISLOCALIZED": "#ffff00",
    "FP": "#ff0000",
}
# Which Map panel topic each verdict goes to. An FN has no estimate to place,
# so it has no entry: it shows as the ground truth pin turning red.
MAP_VERDICT_TOPIC = {
    "TP": "true_positives",
    "MISLOCALIZED": "missed_localizations",
    "FP": "false_positives",
}


def annotation_text(label: str, track_id: str) -> str:
    """What one detection is called, in every view at once: the box label on
    the image and the feature name on the map. One function, so an operator
    can match a map pin to the box it came from."""
    return f"{label} {track_id}".strip()

# A detection is a small dot on the ground, not a person-sized shape.
DETECTION_DOT_DIAMETER = 0.4
# A cross is wider than the dot, because it marks a problem worth a look.
DETECTION_CROSS_SPAN = 1.2
DETECTION_CROSS_LINE_WIDTH = 0.15
# The cross floats this far above the ground, clear of z fighting.
DETECTION_CROSS_LIFT = 0.05

# --------------------------------------------------- ground truth, whole scene
# A target's scene status combines the cameras GROUND_TRUTH_CAMERAS selects,
# the gimbal alone by default. Green beats yellow beats red beats grey.
# Green and yellow need no view: a detection overrides what the view alone
# would say.
#   detected      an estimate lies within the gate of it
#   mislocalized  a viewing ray crosses the gate of it, but that ray's
#                 estimate lies within the gate of none
#   visible       in some camera's view, and no verdict names it
#   out_of_view   no camera sees it, and no verdict names it
GROUND_TRUTH_COLOR = {
    "detected": (0.18, 0.80, 0.44, 0.30),      # green
    "mislocalized": (0.95, 0.77, 0.06, 0.30),  # yellow
    "visible": (0.91, 0.30, 0.24, 0.30),       # red
    "out_of_view": (0.55, 0.55, 0.58, 0.25),   # grey
}

# The bubble radius equals the scoring gate, so a detection dot inside the
# bubble is a hit by construction. Change the two together.
GROUND_TRUTH_BUBBLE_RADIUS = 2.0

# The name label over each bubble. Height is the text size in meters.
GROUND_TRUTH_LABEL_HEIGHT = 0.7
GROUND_TRUTH_LABEL_COLOR = (0.92, 0.94, 0.96, 0.9)


def marker(ns: str, marker_id: int, frame_id: str, stamp,
           position, size_m: float, rgba, text: str = "",
           lifetime=None) -> Marker:
    """One marker for the 3D panels. position is (x, y, z) of the center.
    The default is a sphere with size_m as its diameter. Give text to get
    billboard text with size_m as its height."""
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    if text:
        m.text = text
        m.scale.z = float(size_m)
    else:
        m.scale.x = m.scale.y = m.scale.z = float(size_m)
    m.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    if lifetime is not None:
        m.lifetime = lifetime
    return m


def cross_marker(ns: str, marker_id: int, frame_id: str, stamp,
                 position, size_m: float, rgba, lifetime=None) -> Marker:
    """A flat X centered on position, size_m across. Same signature as
    marker(), so a caller can pick either builder by verdict."""
    m = marker(ns, marker_id, frame_id, stamp, position, size_m, rgba,
               lifetime=lifetime)
    m.type = Marker.LINE_LIST
    half = size_m / 2.0
    m.points = [Point(x=-half, y=-half), Point(x=half, y=half),
                Point(x=-half, y=half), Point(x=half, y=-half)]
    m.scale.x = DETECTION_CROSS_LINE_WIDTH
    m.scale.y = m.scale.z = 0.0
    return m


def hex_rgb(rgba) -> str:
    """The rgba tuple as a #rrggbb string, for the Map panel's GeoJSON styles."""
    return "#%02x%02x%02x" % tuple(round(255 * c) for c in rgba[:3])
