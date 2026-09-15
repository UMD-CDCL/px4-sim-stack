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

## Tested checkpoint

On 2026-09-15 at root commit `b5d0b43`, the live sequence completed **33
passed, 0 failed**. It verified PX4 readiness and flight, MAVLink routing,
5G companion behavior, MAVInsight gimbal/zoom/click/ROI behavior, target
localization and scoring, capture delivery and mosaic rendering, fiducial
survey correction, and every Foxglove topic, service, image, calibration, map,
outline, and verdict layer exercised by the packet. QGC remained a singleton.

The evidence metadata records the worktree state at test start. Generated
evidence is committed only when this packet is green; older failed evidence is
historical and is not reclassified by this checkpoint.
