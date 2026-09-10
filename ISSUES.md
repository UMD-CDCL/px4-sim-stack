# Real-drone fixes

Bench target: `uas1` on the connected Chimera v3. The changes live on
`feature/real-fixes` in `px4-sim-stack`, `chimera-deploy`, `5g_drone`, and
`MAVInsight`.

| # | Issue | State | Evidence or remaining proof |
|---|---|---|---|
| 1 | `rcam` stream dies after about seven minutes | Hardware issue being resolved. | The watchdog retires stalled pipelines. Systemd recreates the service. RGB, RGB-low, pilot, and pilot-low came online after reboot. Thermal remains offline. UVC `-71` errors and a Boson USB disconnect point to hardware. |
| 2 | Onboard container starts twice after hard power loss | Fixed and verified by hard reboot. | The aircraft booted at `2026-09-10 18:26:18 UTC`. `onboard.service` started once at 18:27:01. The container has restart count 0 and Docker policy `no`. |
| 3 | Center and down commands need body yaw and follow mode | Fixed and deployed. | Look Forward and Look Down set zero yaw in the vehicle frame. Other commands and reasserts select body-follow yaw. Held ROI commands select earth-frame yaw. |
| 4 | Terrain must treat an unsurveyed home as ground level | Deferred for this campaign. | Tests cover the implementation. GPS-dependent terrain placement will not receive a flight test here. |
| 5 | VLM capture returns unrelated bounding boxes | Complete. | Plan checks and capture indexing preserve the selected detection and box. The detector threshold stayed unchanged. |
| 6 | Confirm real detections and localizations | Detection complete. GPS localization deferred. | Detector startup and capture plumbing work on the aircraft. Geographic localization needs an outdoor GPS or fiducial anchor. |
| 7 | `./px4sim ui` and `chimera_real.json` are the real-system front doors | Working. Polish can follow later. | The UI and real Foxglove layout work in air and ground configurations. A later pass can improve presentation. |

## Additional findings

| Finding | State | Evidence or action |
|---|---|---|
| Corrected Orin plan needs a one-time build | Complete on the bench. Aircraft retest open. | The plan uses the portable 1280 ONNX. It has the required `input -> output [1,19320,6]` contract. Repeat the capture after the next image restart. |
| Aircraft image can lag behind checked-out flight code | Rebuilt and restarted. | The aircraft pulled `feature/real-fixes` for all four repositories. It rebuilt `px4simstack/onboard:7.1-trt10.3`. It restarted `onboard.service`. |
| Jetson is in 25 W power mode | Observe. | Keep the aircraft power policy unchanged. Account for it when you schedule a TensorRT build. |
| Docker ROS workspace build context is oversized | Improved and rechecked. | The workspace ignore removes model assets, generated workspaces, Git data, and unused packages. The aircraft build transferred about 103 MB. The cached ROS compile took 27.5 seconds. |
| Mosaic auto-growth crashed after the second angled capture | Fixed and verified. | The frame store keeps its correction and source canvas transform. Three captures grew from 475x505 to 742x789 without errors. Ground `terrain_viz` projected each overlay onto the UROC terrain. |

## Acceptance boundary

Bench tests can prove service lifecycle, frame conventions, detector startup,
fresh capture plumbing, and both front doors. GPS-dependent localization,
terrain placement, scoring, long streaming tests, takeoff behavior, and airborne
fiducial correction remain deferred until an outdoor or restrained flight test.
