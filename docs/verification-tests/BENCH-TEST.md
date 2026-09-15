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

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `e1a4aa6` on
`feature/px4sim-baseline-implementation`; Git status was clean. The ground
station received camera info, position, status, and casualty truth. The
simulator now publishes timestamped neutral gimbal telemetry and the live TF
lookup `uas11_home_position -> d11_rgb_offset` succeeds. The bench verifier
still fails its `d11_gimbal_frame` lookup intermittently and reports a 48 deg
drawn-versus-0 deg flying heading discrepancy; DeepStream also respawns while
RTSP streams initialize. No bench pass is claimed until those runtime and
heading contracts are resolved and rerun. QGC was observed as one process.
