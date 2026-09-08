# Real-drone fixes

Bench target: `uas1` on the connected Chimera v3. The requested changes live
on `feature/real-fixes`, branched from `flight_testing` in `px4-sim-stack`,
`chimera-deploy`, `5g_drone`, and `MAVInsight`.

| # | Issue | State | Evidence or remaining proof |
|---|---|---|---|
| 1 | `rcam` stream dies after about seven minutes | Hardware follow-up. | rcam stayed active. The kernel logged UVC `-71` errors and then a Boson USB disconnect. rcam correctly retired only thermal. RGB and pilot continued. No camera-server change was justified. Test the hub, power, cable, and camera subsets over a sustained interval. |
| 2 | Real gimbal pose appears composed with vehicle pose twice | Fixed. Deploy pending. | The aligned live state was internally consistent. Vehicle yaw was -27.18 degrees, relative gimbal yaw was -3.26 degrees, and world gimbal yaw was -30.52 degrees. MAVInsight did ignore the authoritative vehicle/earth yaw-frame bits. An earth-frame report could therefore be composed twice. It now prefers bits 32/64 and retains the legacy fallback. |
| 3 | Onboard container starts, stops, then starts after hard power loss | Fixed. Power-cycle proof pending. | Docker restored the exit-255 container under `on-failure:5`. Systemd then removed and recreated it. The aircraft container now has no Docker restart policy. `onboard.service` starts it once. A hard power cycle must prove the final boot path. |
| 4 | Center/down commands must center body yaw and use follow mode | Fixed. Deploy pending. | Look Forward and Look Down set yaw zero with explicit vehicle-frame flags. Ordinary commands and reasserts select body-follow yaw. Only held ROI commands select earth-frame yaw. Unit and layout tests pass. |
| 5 | Terrain must treat unsurveyed home as ground level | Fixed. Flight proof pending. | Localization and Foxglove now anchor the reported home on its terrain. They ignore raw home altitude. A standing fiducial correction is applied afterward and remains authoritative. Tests cover varying home altitudes and fiducial vertical offsets. |
| 6 | VLM capture returns unrelated bounding boxes | Fixed. Post-build capture pending. | A live request produced fresh, timestamp-matched inference. The Orin plan exposed `output0 [1,84,33600]`. The parser expected `output [batch,N,6]`. The invalid plan is no longer an artifact. Startup rejects incompatible plans and rebuilds from compatible ONNX under a separate cache name. The casualty result also preserves the selected bbox and detection index. |
| 7 | Confirm real detections and localizations | Partially confirmed | The fresh live capture proves inference executes, but its pre-fix boxes were corrupt: blank labels, confidence 36.7 to 65.4, and overlapping top-left boxes. Rangefinder localization returned a valid home-frame position. Terrain/box localization still needs a downward ground view after the corrected plan builds. |
| 8 | `./px4sim ui` and `chimera_real.json` are the real-system front doors | Fixed. Deployed verification pending. | The live TUI rendered and exited cleanly on both ground and aircraft. Real configurations now select and validate `chimera_real.json`. Simulated configurations keep `chimera_sim.json`. The verifier follows panels nested inside Foxglove tabs and stacks. |

## Additional findings

| Finding | State | Evidence or action |
|---|---|---|
| Corrected Orin plan needs a one-time build | Open | The portable 1280 ONNX has the required `input -> output [1,19320,6]` contract. Its first TensorRT build on the 25 W Orin may take substantially longer than an ordinary container rebuild and requires stable bench power. |
| Aircraft image is older than its checked-out flight code | Open | `./px4sim doctor` reports that `onboard` needs rebuilding. Validate fixes in a fresh image before accepting hardware results. |
| Jetson is in 25 W power mode | Observe. | Do not change aircraft power policy implicitly. Account for it when scheduling the TensorRT build. |

## Acceptance boundary

Bench tests may prove service lifecycle, frame conventions, fresh detections,
localization plumbing, and both front doors. Sustained streaming beyond the
observed failure interval and behavior after takeoff or an airborne fiducial
correction require a powered duration test or a restrained flight test.
