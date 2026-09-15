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

Tested: BLOCKED. Date: 2026-09-15. Latest root test commit: `03d3e0b` on
`feature/px4sim-baseline-implementation`; the root checkout was clean before
this ledger update. The latest live run produced 17 passes and 4 failures.
Ground telemetry, heading/TF, casualty truth, ground scoring, Foxglove
layout/topic/service contracts, live image data, calibration, and
verdict-layer structure passed. ReID loaded from the installed tracking
package, but the detector still reported `0 pixel det(s)` and the vehicle
localization check failed. The other failures are click-distance behavior and
a 5.1 m drawn-map height offset. QGC was observed as exactly one host process
after restart. No bench pass is claimed.

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

The `0bea3dd` gimbal-mode change was tested first by copying the committed
`modules/sim/px4-rcS` into the already-built simulator container, because the
image rebuild was blocked by BuildKit DNS failures reaching Ubuntu and Debian
repositories. With `MNT_MODE_IN=4` and `MNT_MODE_OUT=2`, a `-60` command changed
the captured Gazebo camera view from a horizon view to a downward view. The
source change is committed and pushed; a clean image rebuild remains required
before this result is promoted to a persistent deployment checkpoint.

The persistent component checkpoint is now improved: `chimera-deploy` commit
`72ba138` builds `mavros_extras` into the MAVROS overlay, and the rebuilt
`ros-base`, `onboard`, and `offboard` images were started on 2026-09-15. The
simulator helper no longer publishes synthetic gimbal status or registers the
configure service. Live checks found one MAVROS gimbal status publisher,
`gimbal control held by 11/191`, PX4 primary control `11/191`, and a valid
quaternion after `./px4sim uas 11 gimbal -60`. This is a component pass, not a
bench pass: detector/localization, click-distance, and map-height checks still
need a fresh complete run. The 19-pass/2-failure result is retained in
`verify/evidence/bench/` and was captured before the subsequent sim restart;
the authenticated MediaMTX readiness correction is now in `03d3e0b`.

Fresh persistent-image bench run after this checkpoint: 19 passed, 2 failed.
Vehicle gimbal control, both scoring checks, click-distance behavior, all
Foxglove topic/service/layout contracts, live image/calibration, and
verdict-layer structure passed. Vehicle localization still fails because the
detector produced no usable pixel detections in this viewpoint, and the
satellite terrain draw remains 5.1 m above the surface. The bench stage stays
blocked and must not advance to GPS testing yet.
