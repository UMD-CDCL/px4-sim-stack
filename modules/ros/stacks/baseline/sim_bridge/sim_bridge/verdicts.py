"""How detections and ground truth are drawn, in every view at once.

The color rules and marker shapes live here, and only here, because they
change often. Edit this file and the image boxes, the detection dots, and
the truth bubbles all follow. None of it is a ROS parameter on purpose.
"""

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

# ------------------------------------------------------- detections, per camera
# TP: the estimate landed within the gate of a ground truth target.
# FP: no target within the gate.
# An FN, a target in view that this camera did not detect, has no estimate
# and gets no dot. It appears as the ground truth bubble turning yellow.
VERDICT_COLOR = {
    "TP": (0.18, 0.80, 0.44, 0.95),   # green
    "FP": (0.91, 0.30, 0.24, 0.95),   # red
}
# Image boxes before the first verdict arrives.
UNJUDGED_COLOR = (0.75, 0.75, 0.78, 1.0)

# A detection is a small dot on the ground, not a person-sized shape.
DETECTION_DOT_DIAMETER = 0.4

# --------------------------------------------------- ground truth, whole scene
# A target's scene status combines every camera:
#   detected     some camera's estimate landed within the gate of it
#   visible      inside some camera's footprint, but nothing detected it
#   out_of_view  no camera's footprint covers it
GROUND_TRUTH_COLOR = {
    "detected": (0.18, 0.80, 0.44, 0.30),      # green
    "visible": (0.95, 0.77, 0.06, 0.30),       # yellow
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


def hex_rgb(rgba) -> str:
    """The rgba tuple as a #rrggbb string, for the Map panel's GeoJSON styles."""
    return "#%02x%02x%02x" % tuple(round(255 * c) for c in rgba[:3])
