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

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `cdad465` on
`feature/px4sim-baseline-implementation`; Git status was clean before this
ledger update. The latest live run produced 12 passes and 9 failures. Ground
telemetry, heading/TF, casualty truth, both scoring checks, and Foxglove
layout/topic/service contracts passed. ReID loaded from the installed tracking
package and the detector produced enough output for scoring. Remaining
failures are localization publication, click-distance behavior, Foxglove image
data/calibration, target rings and verdict layers, and a 5.1 m drawn-map height
offset. The run still exposes a gimbal bridge durability warning. QGC was
observed as exactly one host process after restart. No bench pass is claimed.

The follow-up implementation checkpoint `d132a50` adds the simulator's
`d11_gimbal_frame -> d11_rangefinder_frame` edge, which `tf_loc` had reported
missing. After restart, `/tf_static` was inspected directly and contained both
the gimbal edge and this rangefinder edge. This fix still requires a fresh live
bench result before changing the tested status.
