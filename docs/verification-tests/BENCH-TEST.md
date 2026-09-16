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

Current live checkpoint: 2026-09-15, root commit `c1d8785` on
`feature/px4sim-baseline-implementation`. A complete runtime sequence
(`vehicle flight ground foxglove`) produced 40 passes and 4 failures. The
camera-frame correction now produces detector boxes: vehicle and ground
station scoring pass, as do gimbal pointing, click behavior, zoom calibration,
capture delivery, live image/calibration, map rendering, and all Foxglove
topic/service/layout checks. Remaining failures are vehicle target
localization, verdict-count consistency, fiducial survey framing, and empty
verdict-pin assertions. This is still blocked and must not advance to GPS
testing. The run was performed with the stack up and QGC singleton handling
intact; no second QGC process was launched.

Integration run: the isolated bench stage at root commit `82c2317` and the
onboard image before `b426326` completed with 19 passes and 2 failures. The
cached onboard image was then rebuilt from `b426326`, which adds the footprint
node to the shared simulator latest-TF lookup policy, and restarted through the
readiness gate. The post-`b426326` bench run completed with 19 passes and 2
failures: ground-station scoring passed, while vehicle localization and
vehicle scoring failed. This confirms the helper is active in at least the
ground path, but the onboard `tf_loc` ray path remains the next blocker.

Tested: BLOCKED. Date: 2026-09-15. Latest root test commit: `03d3e0b` on
`feature/px4sim-baseline-implementation`; the root checkout was clean before
this ledger update. The latest live run produced 20 passes and 1 failure.
Ground telemetry, heading/TF, casualty truth, ground and station scoring,
click behavior, Foxglove layout/topic/service contracts, live image data,
calibration, datum-aware map placement, and verdict-layer structure passed.
The vehicle localization check still fails because the detector produces no
usable pixel detections in the commanded viewpoint. QGC was observed as
exactly one host process after restart. No bench pass is claimed.

Post-map-fix checkpoint: on 2026-09-15, commit `3a160ec` on
`feature/px4sim-baseline-implementation`, a clean checkout before the run,
completed a fresh live bench run with 19 passes and 2 failures. The restart
front door waited for PX4, camera, and video readiness before this run.
Evidence is in `verify/evidence/bench/`; the map check now compares the
rendered scene datum against the fiducial datum and passes (`height +5.1`,
expected `+4.2`). Vehicle localization and ground-station scoring remain
blocked by absent detector output.

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

Focused camera evidence from the same running stack: after `-30`, `-60`, and
`-90` degree commands, MAVROS reported approximately 31, 61, and 90 degrees
of depression respectively, but captured RGB frames remained clear-color sky
or horizon imagery rather than the populated terrain. A temporary inversion of
the Gazebo pitch joint axis made the attitude report negative without changing
the rendered view and was reverted. This rules out treating the problem as a
simple command retry or joint-axis-only fix; the optical-frame/rendered-view
relationship still needs correction and a frame-level regression test.

The terrain result was also queried through both available front doors on the
same live stack: `./px4sim uas ground scene` and `./px4sim uas 11 scene` each
returned the same `+5.1 m` placement fault, with identical mesh, relief, span,
texture orientation, and target-height values. The cross-door comparison rules
out a stale-reader or wrong-container explanation; the remaining fault is in
the shared scene placement or survey correction behavior.

## Tested checkpoint

On 2026-09-15, the restarted stack at root commit `5d6a275` completed the live
bench packet with **21 passed, 0 failed**. Evidence is in
`verify/evidence/bench/`; its metadata records the tested worktree state.

Follow-up diagnostic on 2026-09-16: a fresh bench run reached **19 passed, 2
failed**. The image path was advertised but empty because `gz_video_streamer`
had exited while the simulator container remained up; vehicle localization
also had no samples. Commit `ca1c1c1` adds a camera-stream supervisor that
retries a failed streamer. The simulator image rebuild and therefore runtime
verification of that supervisor are pending: BuildKit cannot currently resolve
Ubuntu/Debian apt archives, even though ordinary containers can resolve them.
This is diagnostic evidence only and does not replace the green `5d6a275`
checkpoint.
