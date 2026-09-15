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

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `95e71bb` on
`feature/px4sim-baseline-implementation`; Git status was clean. The ground
station received camera info, position, status, and casualty truth. The
simulator now publishes timestamped neutral gimbal telemetry and the live TF
lookup `uas11_home_position -> d11_rgb_offset` succeeds. The bench verifier
Both companion and ground heading/TF checks now pass, as do ground camera
info, position, status, and casualty truth. The router no longer reports RTSP
authentication failures and both `rgb11` and `rgbl11` reach online state, but
the run still stalls before reliable camera-FOV/localization output while
DeepStream respawns during stream startup. No bench pass is claimed until
camera-FOV, detection, and localization output are reliable. QGC was observed
as one process.
