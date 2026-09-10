# Real-drone fixes

Bench target: `uas1` on the connected Chimera v3. The requested changes live
on `feature/real-fixes`, branched from `flight_testing` in `px4-sim-stack`,
`chimera-deploy`, `5g_drone`, and `MAVInsight`.

| # | Issue | State | Evidence or remaining proof |
|---|---|---|---|
| 1 | `rcam` stream dies after about seven minutes | Watchdog implemented; hardware follow-up open. | rcam now retires stalled producer pipelines and systemd recreates the service. A post-deployment check still found pilot unavailable while other streams were present. The kernel also logged UVC `-71` errors and a Boson USB disconnect. Test all mounts beyond the observed failure interval and inspect the hub, power, cable, and camera subsets. |
| 2 | Real gimbal pose appears composed with vehicle pose twice | Implemented and merged; live deployment proof pending. | MAVInsight `feature/real-fixes-gimbal-test` was fast-forwarded into `feature/real-fixes` at `321ace0`. Tests cover explicit earth/vehicle flags, legacy fallback, and the v3 override. Validate the merged image on the aircraft. |
| 3 | Onboard container starts, stops, then starts after hard power loss | Fixed. Power-cycle proof pending. | Docker restored the exit-255 container under `on-failure:5`. Systemd then removed and recreated it. The aircraft container now has no Docker restart policy. `onboard.service` starts it once. A hard power cycle must prove the final boot path. |
| 4 | Center/down commands must center body yaw and use follow mode | Fixed. Deploy pending. | Look Forward and Look Down set yaw zero with explicit vehicle-frame flags. Ordinary commands and reasserts select body-follow yaw. Only held ROI commands select earth-frame yaw. Unit and layout tests pass. |
| 5 | Terrain must treat unsurveyed home as ground level | Fixed. Flight proof pending. | Localization and Foxglove now anchor the reported home on its terrain. They ignore raw home altitude. A standing fiducial correction is applied afterward and remains authoritative. Tests cover varying home altitudes and fiducial vertical offsets. |
| 6 | VLM capture returns unrelated bounding boxes | Parser and engine validation fixed; target proof pending. | Startup rejects incompatible plans and quarantines them for rebuild. Capture results preserve the selected bbox and detection index. The current magazine test produced a deterministic empty result, so validate next with a known detectable person or casualty target. |
| 7 | Confirm real detections and localizations | Partially confirmed | The fresh live capture proves inference executes, but its pre-fix boxes were corrupt: blank labels, confidence 36.7 to 65.4, and overlapping top-left boxes. Rangefinder localization returned a valid home-frame position. Terrain/box localization still needs a downward ground view after the corrected plan builds. |
| 8 | `./px4sim ui` and `chimera_real.json` are the real-system front doors | Implemented; final live verification pending. | The live TUI rendered and exited cleanly on both ground and aircraft. Real configurations now select and validate `chimera_real.json`. Simulated configurations keep `chimera_sim.json`. The verifier follows panels nested inside Foxglove tabs and stacks. A local layout adjustment remains uncommitted in `5g_drone`. |

## Additional findings

| Finding | State | Evidence or action |
|---|---|---|
| Corrected Orin plan needs a one-time build | Completed on the bench; aircraft retest open | The compatible TensorRT plan was built from the portable 1280 ONNX with the required `input -> output [1,19320,6]` contract. Repeat the capture on the aircraft after the next image restart. |
| Aircraft image is older than its checked-out flight code | Open | Rebuild and restart `onboard` after the merged MAVInsight and latest `5g_drone` changes before accepting new hardware results. |
| Jetson is in 25 W power mode | Observe. | Do not change aircraft power policy implicitly. Account for it when scheduling the TensorRT build. |

## Acceptance boundary

Bench tests may prove service lifecycle, frame conventions, fresh detections,
localization plumbing, and both front doors. Sustained streaming beyond the
observed failure interval and behavior after takeoff or an airborne fiducial
correction require a powered duration test or a restrained flight test.
