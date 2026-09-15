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

Tested: BLOCKED. Date: 2026-09-15. Commit at test start: `2e23566` on
`feature/px4sim-baseline-implementation`; Git status was clean. The ground
station received camera info, position, status, and casualty truth, but both
vehicle and ground frame checks failed because
`uas11_home_position -> d11_gimbal_frame` did not become available. The run
also observed repeated DeepStream exits with `Could not open resource for
reading` while the RTSP publisher was still coming online. No bench pass is
claimed until the frame contract and stream readiness behavior are fixed and
rerun.
