# Real-drone fixes

Bench target: `uas1` on the connected Chimera v3. The requested changes live
on `feature/real-fixes`, branched from `flight_testing` in `px4-sim-stack`,
`chimera-deploy`, `5g_drone`, and `MAVInsight`.

| # | Issue | State | Evidence or remaining proof |
|---|---|---|---|
| 1 | `rcam` stream dies after about seven minutes | Hardware issue being resolved. | The watchdog retires stalled producer pipelines and systemd recreates the service. RGB, RGB-low, pilot, and pilot-low came online after reboot; thermal remains offline. Kernel UVC `-71` errors and a Boson USB disconnect point to the camera, hub, power, or cable hardware. |
| 2 | Onboard container starts, stops, then starts after hard power loss | Fixed and verified by hard reboot. | The aircraft booted at `2026-09-10 18:26:18 UTC`; `onboard.service` started once at 18:27:01, created the container once, and it is running with restart count 0 and Docker restart policy `no`. |
| 3 | Center/down commands must center body yaw and use follow mode | Fixed and deployed. | Look Forward and Look Down set yaw zero with explicit vehicle-frame flags. Ordinary commands and reasserts select body-follow yaw; only held ROI commands select earth-frame yaw. |
| 4 | Terrain must treat unsurveyed home as ground level | Deferred/out of scope for this campaign. | The implementation is covered by tests, but GPS-dependent terrain placement will not be flight-tested here. |
| 5 | VLM capture returns unrelated bounding boxes | Complete; working as intended. | Plan compatibility checks and capture indexing preserve the selected detection and bbox. No detector threshold was lowered. |
| 6 | Confirm real detections and localizations | Detection complete; GPS localization deferred. | Detector startup and live capture plumbing work on the aircraft. Geographic localization remains intentionally untested without an outdoor GPS/fiducial anchor. |
| 7 | `./px4sim ui` and `chimera_real.json` are the real-system front doors | Working; polish follow-up later. | The UI and real Foxglove layout work in the air and ground configurations. A later pass can improve presentation without changing the control path. |

## Additional findings

| Finding | State | Evidence or action |
|---|---|---|
| Corrected Orin plan needs a one-time build | Completed on the bench; aircraft retest open | The compatible TensorRT plan was built from the portable 1280 ONNX with the required `input -> output [1,19320,6]` contract. Repeat the capture on the aircraft after the next image restart. |
| Aircraft image is older than its checked-out flight code | Rebuilt and restarted; repeat after future changes | The aircraft pulled `feature/real-fixes` for all four repositories, rebuilt `px4simstack/onboard:7.1-trt10.3`, and restarted `onboard.service`. The image loaded the merged MAVInsight package and compatible TensorRT engines. |
| Jetson is in 25 W power mode | Observe. | Do not change aircraft power policy implicitly. Account for it when scheduling the TensorRT build. |
| Docker ROS workspace build context is oversized | Improved and re-verified | The workspace ignore now omits runtime model assets, generated workspaces, git metadata, and unused packages. The aircraft rebuild transferred about 103 MB (55.83 MB + 47.38 MB named-context payloads) and completed the ROS compile in 27.5 seconds with cached base layers. |
| Mosaic auto-growth crashed after the second angled capture | Fixed and verified | The frame store now retains `correction` plus its source canvas transform; auto-growth no longer assumes a stale `H` field. A clean aircraft restart accepted captures at pitch -70°/yaw 0°, -30°, and +30°, growing from 475x505 to 742x789 without errors. Ground `terrain_viz` projected each overlay onto the UROC terrain. |

## Acceptance boundary

Bench tests may prove service lifecycle, frame conventions, detector startup,
fresh capture plumbing, and both front doors. GPS-dependent localization,
terrain placement, scoring, sustained streaming beyond the observed failure
interval, and behavior after takeoff or an airborne fiducial correction are
deferred until a powered outdoor or restrained flight test.
