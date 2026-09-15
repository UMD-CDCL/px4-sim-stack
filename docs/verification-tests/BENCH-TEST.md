# Bench test

## Scope

Ground operation with GPS unavailable or intentionally withheld. Verify the
operator/front-door paths, communications, cameras, gimbal, zoom, DeepStream,
tracking inputs, status, services, and safe failure of GPS-dependent features.

## Isolation

The test owns setup, fixture data, environment, startup, teardown, and its
evidence directory. It must not consume a GPS or flight result from another
stage. Record every applicable ledger feature and every observed refusal as
`CURRENT`, `PLANNED`, `BLOCKED`, or `UNKNOWN` with the reason. Planned bench
features are tracked for design impact but do not fail the current baseline.

## Checkpoint

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `9a5d7bb` in the
`5g_drone` repository on `feature/ubuntu24-compat`; the root checkout was
clean before this ledger update. The latest live run produced 16 passes and 5
failures. Ground telemetry, heading/TF, casualty truth, Foxglove layout,
topic/service contracts, live image data, calibration, and verdict-layer
structure passed. ReID loaded from the installed tracking package, but the
detector reported `0 pixel det(s)` and the localization node reported
`localized 0 box(es)` throughout the run, so neither scoring check can pass.
The other failures are click-distance behavior and a 5.1 m drawn-map height
offset. QGC was observed as exactly one host process after restart. No bench
pass is claimed.

The follow-up implementation checkpoint `d132a50` adds the simulator's
`d11_gimbal_frame -> d11_rangefinder_frame` edge, which `tf_loc` had reported
missing. After restart, `/tf_static` was inspected directly and contained both
the gimbal edge and this rangefinder edge. This fix still requires a fresh live
bench result before changing the tested status.

The fresh post-fix run confirms the simulator-only temporal fallback is active:
the node warns that historical TF is unavailable and uses the newest simulator
transform. Live `/tf` contains `d11_gimbal_frame` and `/tf_static` contains the
rangefinder edge. This removes the earlier exception path, but it does not
create detections; the remaining localization/scoring blocker is currently
upstream in detector input or inference, not proven to be a TF frame-name
problem.
