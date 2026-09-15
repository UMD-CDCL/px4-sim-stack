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
