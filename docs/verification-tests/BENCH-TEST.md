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

Tested: BLOCKED. Date: 2026-09-15. Latest test commit: `af115e3` on
`feature/px4sim-baseline-implementation`; Git status was clean before this
ledger update. A live run produced 16 passes and 5 failures. Ground camera,
position, status, heading, casualty truth, Foxglove layout/topic/service/frame
contracts, and QGC singleton behavior passed. The rebuilt portable image also
contains the ReID model at the installed tracking package path. Remaining
failures are localization/detection/scoring output, click-distance behavior,
and map north-up orientation. The run also reported incompatible durability
QoS for the bridged gimbal command topics; this is an implementation issue to
resolve, not evidence that the command path is correct. QGC was observed as
exactly one host process after restart. No bench pass is claimed.
