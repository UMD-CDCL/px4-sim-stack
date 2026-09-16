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

The sim checkpoint covers QGC singleton behavior, PX4/MAVLink readiness,
gimbal and zoom control, raw coordinate ROI, localization, capture/mosaic,
fiducial correction, Foxglove contracts, and all five scoring metrics.

## Source checkpoints

- Root: `39c1581`
- 5G Drone: `0bb58e8` on `feature/ubuntu24-compat`
- MAVInsight: `0fb189b` on `feature/ubuntu24-compat`
- PX4-Autopilot: `639154f` on `feature/ubuntu24-compat`

The working tree is clean apart from generated evidence directories that are
intentionally excluded from commits. The next implementation item is paused:
investigate whether the transient RTSP source-loss log merits a separate
startup/readiness assertion. No deployment-only Chimera work was started.

Verification is a critical-checkpoint activity, not a per-edit requirement.
