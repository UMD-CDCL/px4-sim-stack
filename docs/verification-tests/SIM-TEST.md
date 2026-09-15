# Sim test

## Scope

End-to-end simulated flight: PX4 state transitions, MAVLink routing, 5G
companion behavior, MAVInsight, camera streams, gimbal and zoom in motion,
perception/tracking, mission/survey, localization, and live Foxglove panels.

## Isolation

This stage owns its world, vehicle state, scenario, startup, flight actions,
evidence, and teardown. A container being `Up` is not a pass; require the
specific readiness log, topic/service data, and user-facing result for every
feature record. Record future flight capabilities as `PLANNED` with their
intended checkpoint and dependencies; do not count them as baseline failures.

## Current checkpoint

On 2026-09-15 at root commit `c1d8785`, the live flight/ground/Foxglove
sequence reached 40 passes and 4 failures. PX4 takeoff, gimbal motion, zoom
calibration, detector box production, vehicle and ground scoring, click/ROI
behavior, capture delivery, map rendering, and Foxglove live topics/services
passed. Target localization, verdict-count consistency, fiducial survey
framing, and verdict-pin population failed. The stage remains `BLOCKED`; this
is evidence for implementation progress, not a sim-test pass. QGC remained a
singleton throughout the run.
