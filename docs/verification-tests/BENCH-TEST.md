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

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `9e68756` on
`feature/px4sim-baseline-implementation`; Git status was clean. The ground
station received camera info, position, status, and casualty truth. The
simulator now publishes timestamped neutral gimbal telemetry and the live TF
lookup `uas11_home_position -> d11_rgb_offset` succeeds. The bench verifier
Both companion and ground heading/TF checks now pass (about 69 deg compass,
airframe and camera). Ground camera info, position, status, and casualty truth
also pass. The remaining failure is vehicle localization after the scripted
gimbal-pointing/detection action; DeepStream continues respawning while RTSP
streams initialize. No bench pass is claimed until localization and stream
contracts are resolved and rerun. QGC was observed as one process.
