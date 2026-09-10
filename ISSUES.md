# Real-drone fixes

Bench target: `uas1` on the connected Chimera v3. The requested changes live
on `feature/real-fixes`, branched from `flight_testing` in `px4-sim-stack`,
`chimera-deploy`, `5g_drone`, and `MAVInsight`.

| # | Issue | State | Evidence or remaining proof |
|---|---|---|---|
| 1 | `rcam` stream dies after about seven minutes | Watchdog implemented; hardware follow-up open. | rcam now retires stalled producer pipelines and systemd recreates the service. After the rebuilt aircraft stack came up, RGB, RGB-low, pilot, and pilot-low were online; thermal remained offline. The kernel also logged UVC `-71` errors and a Boson USB disconnect. Test all mounts beyond the observed failure interval and inspect the hub, power, cable, and camera subsets. |
| 2 | Real gimbal pose appears composed with vehicle pose twice | Implemented, merged, and deployed; flight proof pending. | MAVInsight `feature/real-fixes-gimbal-test` was fast-forwarded into `feature/real-fixes` at `321ace0`. The aircraft rebuilt and loaded that package. Live gimbal status reports flags `44`, which the merged v3 earth-reference override handles. Validate alignment while changing vehicle yaw. |
| 3 | Onboard container starts, stops, then starts after hard power loss | Fixed. Power-cycle proof pending. | Docker restored the exit-255 container under `on-failure:5`. Systemd then removed and recreated it. The aircraft container now has no Docker restart policy. `onboard.service` starts it once. A hard power cycle must prove the final boot path. |
| 4 | Center/down commands must center body yaw and use follow mode | Fixed. Deploy pending. | Look Forward and Look Down set yaw zero with explicit vehicle-frame flags. Ordinary commands and reasserts select body-follow yaw. Only held ROI commands select earth-frame yaw. Unit and layout tests pass. |
| 5 | Terrain must treat unsurveyed home as ground level | Code fixed; GPS-denied bench proof blocked. | Localization and Foxglove ground the reported home on terrain and ignore raw home altitude. A standing fiducial correction remains authoritative. On the current aircraft bench there is no GPS/home fix, so the frame tree has no geographic anchor and waits rather than inventing one. Tests cover varying home altitudes and fiducial vertical offsets. |
| 6 | VLM capture returns unrelated bounding boxes | Parser and engine validation fixed; target proof pending. | Startup rejects incompatible plans and quarantines them for rebuild. Capture results preserve the selected bbox and detection index. The current magazine test produced a deterministic empty result, so validate next with a known detectable person or casualty target. |
| 7 | Confirm real detections and localizations | Partially confirmed; current bench has no GPS fix | The rebuilt aircraft deserialized the compatible detector engines and a live VLM capture returned a fresh image with a deterministic empty box list. No localization was published because `home_position/fix` is absent and the rangefinder TF tree cannot connect to the home frame. Repeat with GPS or a known home/fiducial anchor and a detectable target. |
| 8 | `./px4sim ui` and `chimera_real.json` are the real-system front doors | Implemented; final live verification pending. | The live TUI rendered and exited cleanly on both ground and aircraft. Real configurations now select and validate `chimera_real.json`. Simulated configurations keep `chimera_sim.json`. The verifier follows panels nested inside Foxglove tabs and stacks. A local layout adjustment remains uncommitted in `5g_drone`. |

## Additional findings

| Finding | State | Evidence or action |
|---|---|---|
| Corrected Orin plan needs a one-time build | Completed on the bench; aircraft retest open | The compatible TensorRT plan was built from the portable 1280 ONNX with the required `input -> output [1,19320,6]` contract. Repeat the capture on the aircraft after the next image restart. |
| Aircraft image is older than its checked-out flight code | Rebuilt and restarted; repeat after future changes | The aircraft pulled `feature/real-fixes` for all four repositories, rebuilt `px4simstack/onboard:7.1-trt10.3`, and restarted `onboard.service`. The image loaded the merged MAVInsight package and compatible TensorRT engines. |
| Jetson is in 25 W power mode | Observe. | Do not change aircraft power policy implicitly. Account for it when scheduling the TensorRT build. |

## Acceptance boundary

Bench tests may prove service lifecycle, frame conventions, fresh detections,
localization plumbing, and both front doors. Sustained streaming beyond the
observed failure interval and behavior after takeoff or an airborne fiducial
correction require a powered duration test or a restrained flight test.
