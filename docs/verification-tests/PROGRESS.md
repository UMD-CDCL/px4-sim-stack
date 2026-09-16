# Implementation progress

Checkpoint: 2026-09-16

The implementation branch is `feature/px4sim-baseline-implementation`.
The compatibility mirror is `feature/ubuntu24-compat-derived`. External
repositories changed for this work remain on and pushed from
`feature/ubuntu24-compat`.

## Green stages

| Stage | Result | Evidence |
|---|---:|---|
| Code | 20/0 live | `verify/evidence/code/` |
| Bench | 21/0 live | `verify/evidence/bench/` |
| GPS | 21/0 live | `verify/evidence/gps/` |
| Sim | 46/0 live | `verify/evidence/sim/` |

The previously certified bench checkpoint remains **21/0**. A later live
retry after the onboard image refresh passed ground camera, position, status,
and heading readiness, but did not complete: the simulator takeoff/reposition
command remained stuck and was stopped after its bounded diagnostic window.
The ground stage now waits for all three telemetry topics before probing and
wraps both reposition attempts in a timeout; a fresh full bench checkpoint is
still required.

The sim checkpoint covers QGC singleton behavior, PX4/MAVLink readiness,
gimbal and zoom control, raw coordinate ROI, localization, capture/mosaic,
fiducial correction, Foxglove contracts, and all five scoring metrics.

The portability manifest also fingerprints the 5G Drone model manifest, so a
checkpoint can detect model-selection drift independently of source and image
commits. The current machine exposes a configuration mismatch: `.env` points
to `/home/user/deepstream-work/models`, which has no manifest, while the
canonical source manifest is present; the manifest now reports both facts.

The 5G Drone scorer now publishes a zero `position_error` for a valid frame
with no localization errors. Runtime certification is pending a clean full
sim restart: an isolated offboard recreation left `/uas11/home_position/fix`
without a sample, preventing ground-truth placement and making all scoring
metrics correctly silent.

The subsequent clean restart restored `/uas11/home_position/fix`, but the
focused Foxglove run still found the scenario messages had no usable position:
the simulator logged 322 entities placed while the scorer logged zero targets
placed against the origin. This is now a distinct scenario-truth data-contract
issue to trace; the scoring implementation remains runtime-unverified.

## Source checkpoints

- Root: `fc3acf6` (code-stage evidence recorded at this source checkpoint)
- 5G Drone: `8296fe0` on `feature/ubuntu24-compat`
- MAVInsight: `0fb189b` on `feature/ubuntu24-compat`
- PX4-Autopilot: `639154f` on `feature/ubuntu24-compat`

The working tree is clean apart from generated evidence directories that are
intentionally excluded from commits. The next implementation item is paused:
live-test the RTSP preflight deadline fix after rebuilding the onboard image.
The cache-preserving rebuild and live startup passed (`rgb11 ready after 57s`,
then `ds_node: pipelines PLAYING, services up`); deliberate deadline-expiry
coverage is now covered by a deterministic source-contract test, while a live
timeout-expiration run remains pending. The transient RTSP source-loss observation remains
documented in `SIM-TEST.md`. No deployment-only Chimera work was started.

Verification is a critical-checkpoint activity, not a per-edit requirement.
