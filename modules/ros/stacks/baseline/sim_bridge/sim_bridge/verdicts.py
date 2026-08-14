"""Verdict names and colors, shared by every view that draws them.

The image boxes and the 3D spheres read the same table, so a detection has
one color everywhere.
"""

# TP: the estimate landed within the gate of a ground truth target.
# FP: no target within the gate.
# FN: a visible target that nothing found. It has no box to draw, so it
#     appears only in the 3D view, at the target.
VERDICT_COLOR = {
    "TP": (0.18, 0.80, 0.44),   # green
    "FP": (0.91, 0.30, 0.24),   # red
    "FN": (0.95, 0.77, 0.06),   # yellow
}
UNJUDGED_COLOR = (0.75, 0.75, 0.78)     # grey, before the first verdict
GROUND_TRUTH_COLOR = (0.20, 0.55, 0.95)  # blue, where the target really is

# Every person marker is a sphere of this diameter, in meters.
PERSON_SPHERE_DIAMETER = 1.0
