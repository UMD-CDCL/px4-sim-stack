# Implementation progress

Checkpoint: 2026-09-16

The implementation branch is `feature/px4sim-baseline-implementation`.
The compatibility mirror is `feature/ubuntu24-compat-derived`. External
repositories changed for this work remain on and pushed from
`feature/ubuntu24-compat`.

## Green stages

| Stage | Result | Evidence |
|---|---:|---|
| Code | 21/0 live | `verify/evidence/code/` |
| Bench | 26/0 live | `verify/evidence/bench/` |
| GPS | 21/0 live | `verify/evidence/gps/` |
| Sim | 36/4 live | `verify/evidence/sim/` |

The earlier incomplete bench retry is retained in git history as a diagnostic
checkpoint. The current rebuilt-image bench result supersedes it: **26/0**.
The ground stage waits for all three telemetry topics before probing and wraps
both reposition attempts in a timeout.

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

The pushed scorer fix was confirmed present in the staged build context, but
not in the existing `ros-base` image. An offline `docker compose build
--pull=false ros-base` still attempted to resolve the locally tagged
`px4simstack/ros-deps:7.1` and `yolo-deps:7.1` parents through Docker Hub. The
machine therefore needs a later build-portability fix or an available local
BuildKit image import before the scorer can be runtime-certified. No green
result is claimed from this attempt.

On 2026-09-16, `./px4sim restart --no-build` completed and reported all seven
runtime services up. PX4 logged `Ready for takeoff`, MediaMTX eventually
reported `rgb11` and `rgbl11` publishing, and HTTP probes for `rgb11`,
`pilot11`, and `thermal11` succeeded. The QGC container contained exactly one
`QGroundControl` process. Initialization is still incomplete: onboard and
offboard DeepStream repeatedly exit with `Could not open resource for reading`,
and `/uas11/camera/camera_info` produced no sample during a ten-second probe;
the resulting gimbal, footprint, and scoring warnings are expected until that
camera contract is restored.

## Source checkpoints

- Root: `baa8af6` (source checkpoint for this test run)
- 5G Drone: `b269aa0` on `feature/ubuntu24-compat`
- MAVInsight: `0fb189b` on `feature/ubuntu24-compat`
- PX4-Autopilot: `639154f` on `feature/ubuntu24-compat`

The working tree is clean apart from generated evidence directories that are
intentionally excluded from commits. The RTSP preflight fix and persistent
rebuilt-image startup are covered by the current bench and sim checkpoints;
the transient RTSP source-loss observation remains documented in `SIM-TEST.md`.
No deployment-only Chimera work was started.

Verification is a critical-checkpoint activity, not a per-edit requirement.

The 5G Drone launch fix at `889c277` was runtime-probed on the live stack by
overlaying the checked-out launch file into the running onboard container.
When the simulator lens emulator was initially late, `zoom` exited; its
respawn then retried after the emulator became available. The retry reached
`zoom node ready`, published camera info, and onboard gimbal plus target
preprocessing logged successful camera-info consumption. This confirms the
startup-race fix, but a rebuilt image is still required for persistent
certification.

The scorer follow-up at `496a825` publishes an initial zero position error
when truth is ready but no detector frame has arrived. After rebuilding
`ros-base`, onboard, and offboard from local registry parents, the live bench
stage completed **26 passed, 0 failed** at 2026-09-16T17:10:23Z. This covered
ground camera/position/status/heading, localization transfer, target scoring,
Foxglove layout topics/services/images/map/pins, and all five scoring metrics.

The ROS base Dockerfile now accepts explicit `DEPS_IMAGE` and
`YOLO_DEPS_IMAGE` build arguments, defaulting to the existing parent names.
This permits a prepared machine to reuse local-registry parents while keeping
the normal defaults unchanged; the tested build used `localhost:5000`.

The prior full sim stage completed **40 passed, 0 failed** on 2026-09-16 at
17:34:42Z. The current rebuilt-image sim stage completed **39 passed, 2
failed** at 18:50:05Z: narrow framing camera-info was silent and returning to
mid framing did not complete. All other flight, localization, ROI, capture,
survey, Foxglove, and scoring checks passed. The narrow-zoom regression is the
next implementation target; no current sim pass is claimed.

A warmed rerun completed **36 passed, 4 failed** at 19:15:09Z: initial
vehicle state was silent and raw ROI acceptance plus coordinate retention
failed. Narrow zoom passed in this run; all other flight, localization,
capture, survey, Foxglove, and scoring checks passed. The readiness and
raw-ROI contract is the next target, and no current sim pass is claimed.

At 19:19:31Z another live sim run was started from commit `d6006e2` and was
paused after the vehicle checks and the first flight checks passed because the
harness hung waiting for a later gimbal-state read. A direct
`./px4sim uas 11 raw-roi 38.9869 -76.9426 39.9` probe during the same running
stack was accepted by the onboard gimbal, which logged three `holding ...
(fix)` callbacks. This confirms the px4sim raw-ROI front door and onboard
subscription work in isolation, but it does not certify the complete sim
stage; the harness readiness/state wait remains open. No implementation change
was made for this diagnostic run. Generated evidence is retained as an
incomplete checkpoint, and implementation is paused here pending the next
work session.

The next bounded fix updates the `uas topic` front door to poll DDS graph
discovery inside its deadline before subscribing. This addresses the reproduced
startup hang where `/uas11/gimbal/state` was absent from the first graph
snapshot even though the gimbal was publishing it. Static/unit verification is
green at **21 passed, 0 failed**, and a live probe now returns the latched ROI
state. The full sim stage still needs a fresh end-to-end run at a later
checkpoint.

After the DDS discovery fix, the complete live sim stage passed **40 passed,
0 failed** at `2026-09-16T19:54:26Z` from commit `bd7122b`. This is the current
sim-tested checkpoint: vehicle readiness, flight and gimbal behavior, click
and raw-ROI retention through movement, capture, survey, Foxglove panels and
all five scoring metrics completed successfully.

The post-MCAP rebuilt-image sim run on 2026-09-16 completed **36 passed, 4
failed** at `2026-09-16T21:05:29Z`. Vehicle telemetry, gimbal framing,
perception annotations, capture delivery, scoring, and all Foxglove layout/data
checks passed. The failures are bounded to raw-ROI acceptance/retention and one
survey-position assertion (`fiducial -> uas11_home_position -0.29,0.69,0.52`
versus expected `-6.00,9.00`); no code change is being claimed until those
paths are reconciled against the current source and scenario.

Source inspection found the raw-ROI failures were a QoS race: the verifier's
latched publisher was paired with the gimbal's volatile subscription. The fix
was committed and pushed to 5G Drone `feature/ubuntu24-compat` as `62c95a9`,
rebuilt with cached root layers, and focused live verification passed after
restart on 2026-09-16: the target was retained across discovery and exactly one
QGC container was running. The survey-position assertion remains open.

The 5G Drone ROI safety slice was implemented on `feature/ubuntu24-compat`
as commit `db780ed`. The affected images rebuilt from local cached parents in
about seconds of package compilation, and a restarted live stack rejected a
`nan` raw-ROI input while accepting `38.9869000,-76.9426000,39.9` and reporting
that exact target in `gimbal/state`.

The recording slice is now `PARTIAL`: `./px4sim record start|stop|status`
resolves the canonical ground container, persists output under
`logs/recordings`, validates names, and starts the pinned upstream rosbag2 MCAP
plugin in the rebuilt image. Live start, status, named stop, output inspection,
and `ros2 bag info` passed for `mcap_checkpoint`; the result was
`mcap_checkpoint_0.mcap` with populated status and scene topics. Synchronized
video capture remains planned, so this does not yet promote the whole recording
feature to complete.

The code stage completed **21 passed, 0 failed** on 2026-09-16 at 17:54:21Z.
It verified airframe expansion, service/address/MAVLink contracts, the
px4sim UI helper, QGC singleton behavior, the recording front door and
persistent mount contract, including named recording stop selection, 43
terrain/frame checks, 137 5G Drone functional checks, and four
map-axis/texture checks.

The GPS stage completed **21 passed, 0 failed** on 2026-09-16 at 17:18:25Z.
It recorded 203 identical localization samples across vehicle and ground
front doors and passed terrain/roof geometry, multiple camera framings,
horizon gating, heading/TF, casualty truth, scoring, and click behavior.

The SQLite fallback lifecycle was exercised live on 2026-09-16: start,
active-PID status, clean stop, and output inspection all passed. The 8-second
`smoke6` bag contains 5,122 messages and a 23 MB SQLite database. This is
recording-front-door evidence only; MCAP is now verified, while synchronized
video remains unavailable.

Named recording selection was exercised live on 2026-09-16: two simultaneous
SQLite bags (`named_a` and `named_b`) were started, stopping `named_a` left
`named_b` active, and the remaining bag was then stopped cleanly. This proves
selective lifecycle control while the MCAP and synchronized-video limitations
remain unchanged for video synchronization.

The 5G Drone health wiring slice was source- and bench-verified and pushed as `b269aa0`
on `feature/ubuntu24-compat`: the gimbal heartbeat default now agrees with
the status consumer, invalid GPS clears GPS and RTK bits, and a non-RTK fix
clears stale RTK state. Python syntax validation passed, and the rebuilt-image
bench stage completed **26 passed, 0 failed** at 2026-09-16T18:16:33Z. Package
pytest remains unavailable on the host; the live bench result is the runtime
evidence for this slice.

The GPS stage first produced 20/1 while the detector was still warming after
restart. A clean retry completed **21 passed, 0 failed** at
2026-09-16T18:33:07Z, including localization, GPS/RTK-dependent geometry,
heading/TF, casualty truth, scoring, and click behavior. The failed attempt is
retained in the stage evidence as a diagnostic run; the retry is the current
GPS checkpoint.

The survey fiducial relocation contract was source-reviewed and live-verified
on 2026-09-16. The scenegen generator now emits a pose-controllable,
gravity-free, kinematic fiducial because `px4sim fiducial` uses Gazebo
`set_pose`; the generated campus world reported the requested `(+6, -9) m`
offset. The associated scenegen test was extended. A full flight retry reached
the gimbal/ROI checks but was stopped after a later ROI polling loop exceeded
its useful test window; it remains incomplete evidence and does not promote
the flight stage. One pre-existing gimbal-pointing assertion remains failing.

After correcting the verification pitch sign, a clean flight-stage retry was
started on the same stack. The router reported live vehicle traffic, but the
stage remained in `uas11 takeoff 20` beyond its useful observation window and
was stopped. This is recorded as a runtime takeoff-readiness failure; no
flight-stage pass is claimed from this attempt. The direct gimbal front-door
probe passed with `+45` commanded and `46.19` degrees reported.
