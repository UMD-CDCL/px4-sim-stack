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
| Sim | 39/2 live | `verify/evidence/sim/` |

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

The recording slice is currently `PARTIAL`: `./px4sim record start|stop|status`
now resolves the canonical ground container, persists output under
`logs/recordings`, validates recording names, and preflights the MCAP storage
plugin. The installed image reports only `sqlite3` and a test plugin, so the
front door correctly refuses to start and identifies the required rosbag2
MCAP dependency. Video synchronization and MCAP image support remain planned.

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
recording-front-door evidence only; it does not promote the feature beyond
`PARTIAL` because MCAP and synchronized video are still unavailable.

Named recording selection was exercised live on 2026-09-16: two simultaneous
SQLite bags (`named_a` and `named_b`) were started, stopping `named_a` left
`named_b` active, and the remaining bag was then stopped cleanly. This proves
selective lifecycle control while the MCAP and synchronized-video limitations
remain unchanged.

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
