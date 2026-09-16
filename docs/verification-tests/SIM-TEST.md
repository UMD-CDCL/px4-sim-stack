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

Follow-up live packet on 2026-09-16 at root commit `3a1135b`: **32 passed, 1
failed**. Flight controls, gimbal/ROI behavior, capture delivery, map
rendering, Foxglove topic/service contracts, and live image delivery passed.
The remaining failure is verdict/box timing consistency (`15` verdicts,
`21` boxes, `79` marks in 8 seconds). This is a targeted perception timing
defect and does not replace the last green sim checkpoint. The camera
supervisor source is also not present in the prepared image used by this run.

Corrected follow-up packet: **33 passed, 0 failed** on 2026-09-16 at root
commit `5819048`. The annotation check now follows the source contract:
verdicts judge a subset of recently published boxes, while `scoring_viz.py`
keeps unjudged boxes visible. Flight, gimbal/ROI, captures, map,
localization, and Foxglove checks all passed. The camera-supervisor recovery
change still awaits a rebuilt simulator image.

Final raw-ROI follow-up packet: **46 passed, 0 failed** on 2026-09-16. This
checkpoint additionally verified the canonical raw coordinate ROI command,
retention of the exact target after movement, and all five scoring metrics
(`recall`, `precision`, `detection_recall`, `detection_precision`, and
`position_error`) through the Foxglove path. The packet used root commit
`39c1581` as its evidence checkpoint; the retained manifest records the
source commits used by the running images.

During isolated restarts, `ds_node` sometimes logged GStreamer
`Could not open resource for reading` with `No supported authentication
protocol was found` while the simulator publisher was temporarily absent.
The configured source was the expected `rtsp://video-router:8554/rgbl11`, a
direct GStreamer read succeeded after recovery, and the ground-station
`respawn` contract restored the viewer. Treat this as a transient publisher
availability observation, not a failed baseline assertion; repeat it at the
next critical checkpoint if startup reliability is changed.
