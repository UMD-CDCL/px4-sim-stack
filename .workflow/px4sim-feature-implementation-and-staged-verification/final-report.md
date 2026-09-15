# Final Report: px4sim feature implementation and staged verification

## Outcome
Implementation checkpoint complete on `feature/px4sim-baseline-implementation`.

Start tag: `ubuntu24-compat-start-20260915` at `3188ce1`.
Implementation commit: `5edc0ad` (`implement px4sim front door and staged verification`).

## Accepted Results
The px4sim front door now treats `restart` as an offline prepared-image restart and supports `restart --build` for cache-preserving rebuilds. The TUI action text and restart feed reset match that contract.

The code, bench, GPS, and sim verification wrappers are separate, dry-run by default, checkpointed, and capable of invoking existing stage assertions in live mode without lifecycle side effects.

## Rejected Results
The MAVInsight parser change was rejected because it exists only in the ignored generated `.build-contexts` tree and is not portable in a clean checkout.

## Conflicts Resolved

## Verification Evidence
Passed: `./px4sim check`; `./px4sim help`; Python compilation; shell syntax checks; all four wrapper dry runs; workflow artifact validation; `git diff --check`; simulator range-reading unit tests; startup/frame contract checks.
The live code stage passed 17/17 assertions. The bench stage remains blocked by the camera/detector path: telemetry, Foxglove contracts, images, calibration, datum-aware map checks, and target layers pass, while vehicle localization and ground-station scoring receive no usable pixel detections.

Follow-up compatibility implementation: `5g_drone` commit `80d7dba` on
`feature/ubuntu24-compat` adds one shared timestamped-TF lookup helper and
wires it into scoring, ground projection, and mosaic. Simulation enables its
latest-transform fallback through the existing `onboard_sim_params.yaml`
layer; real-aircraft defaults remain strict historical lookup. This is source
verified and pushed, but not yet runtime verified because the dependent onboard
image still needs rebuilding.

At the 2026-09-15 checkpoint, `./px4sim restart --no-build` completed through the readiness gate. All seven services were running, `video-router` was healthy, simulator startup reported Gazebo world readiness, and `pgrep -x QGroundControl` reported exactly one process. `./px4sim probe 11 --deadline 3` completed successfully; topics without publishers are reported explicitly by the probe rather than treated as a launch failure.

## Remaining Risks
The runtime feature packet remains to be reworked against tracked source. Existing fixture coverage still reports a missing `/src/tracking_test_5g` path, and `pytest` was unavailable on the host. The rebuilt onboard image containing `80d7dba` still needs a successful build and restart before live bench rerun; the simulator image rebuild is separately blocked by Docker container DNS resolving Ubuntu mirrors.

QGroundControl launch serialization is enforced by the persistent config-volume
flock and the compose restart policy. The restart checkpoint directly verified
the running singleton after a full front-door restart. A forced application
crash/restart remains unverified because manually killing the packaged QGC
process did not exercise the container's normal unexpected-exit path.

## Reusable Follow-up
