# 5G Drone feature ledger

This ledger covers the whole `5g_drone` package, including active and legacy
launch paths. It intentionally includes features that may be abandoned or
only available on real hardware; use `ORPHANED`, `MISSING`, `PLANNED`, or
`BLOCKED` rather than deleting them during the first review.

`Status` records intended scope, not verification. Add `Tested: YES|NO|BLOCKED`
with date, commit, Git status, and evidence; this inventory defaults to
`Tested: NO`.

| Intent | Feature | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `CURRENT` | Onboard launch composition | `launch/onboard.launch.py` | All active nodes, params, namespaces, and remaps resolve |
| `CURRENT` | Ground-station launch composition | `launch/ground_station*.launch.py` | Ground bridge, HIL, video, and viz nodes work |
| `CURRENT` | Offboard companion launch | `launch/offboard.launch.py` | Companion services and links start |
| `ORPHANED` | Per-UAS launch variants: orphaned if not used by the px4sim front door, but if they are they should be kept and activated by the px4sim front door | `launch/uas1..uas4.launch.py` | Vehicle-specific configurations remain valid |
| `ORPHANED` | Thermal launch variants: orphaned if not used by the px4sim front door, but if they are they should be kept and activated by the px4sim front door | `launch/*thermal.launch.py` | Thermal stream/calibration/path works |
| `ORPHANED` | No-GPS assessment variants | `launch/*no_gps*.launch.py` | GPS-dependent nodes fail safely or are omitted |
| `ORPHANED` | Postrun/replay variants | `launch/postrun*.launch.py` | Recorded/postrun inputs are consumed correctly |
| `CURRENT` | MAVROS connection and plugin configuration | `config/param_files/px4_*.yaml`, launch files | State, telemetry, commands, and services work |
| `ORPHANED` | MAVLink system/heartbeat telemetry | MAVROS and router configs | Heartbeats/state are timely and correctly namespaced |
| `ORPHANED` | Global GPS position | params, `legacy_nodes/detect.py`, consumers | NavSatFix reaches all intended consumers |
| `CURRENT` | Local position/odometry | params and localization consumers | Local pose and covariance are valid |
| `CURRENT` | Compass heading | MAVROS params and detection code | Heading drives geometry correctly |
| `CURRENT` | IMU/attitude | MAVROS params and vehicle nodes | Attitude is available with expected frame |
| `CURRENT` | Rangefinder | gimbal rangefinder params and consumers | Range data is valid and bounded |
| `TESTED (code 2026-09-16, 5g_drone 7f92ac5)` | Gimbal command/state | gimbal node, params, services/topics, canonical `config/foxglove/chimera_sim.json` | Full functional code suite passed 137 tests; canonical sim gimbal command now carries `FOLLOW_BODY_YAW` flags (`44`) and matches the gimbal implementation. Live command/feedback remains covered by the staged sim gate. |
| `CURRENT` | Camera info/calibration | calibration files, `cam_info` launch | Camera info matches each stream |
| `CURRENT` | RGB/gimbal/down/thermal cameras | launch params and camera bridges | Images publish at declared resolution/rate |
| `CURRENT` | Camera zoom and framing | `config/foxglove`, zoom/gimbal code | Presets and continuous framing affect image |
| `CURRENT` | DeepStream primary detection | `umd_uas/ds_ros_pipeline`, deepstream configs | Engine loads and detections publish |
| `TESTED (live 2026-09-16, commit 0bb58e8)` | Scoring metric publication | `umd_uas/scoring.py`, canonical Foxglove plot topics | Empty rate denominators publish explicit zeroes, and the full staged sim observed all five scoring metrics through the Foxglove bridge. Position error remains conditional on localized estimates, which were present in the flight path. |
| `TESTED (live 2026-09-16, commit 0bb58e8)` | Raw coordinate ROI hold | `umd_uas/gimbal.py`, `raw_roi_point_cmd` | Full flight-stage test accepted a NavSatFix coordinate and retained the exact target in ROI mode after movement. |
| `CURRENT` | Secondary injury/VLM inference | `config/deepstream/*secondary*`, pipeline code | Secondary results are emitted and correlated |
| `CURRENT` | Detection enable/disable | pipeline services/topics and layout | State changes and data flow respond |
| `CURRENT` | Detection preview image | `ds_ros_pipeline/ros_io.py` | Preview is encoded and reaches ground |
| `CURRENT` | TargetBoxArray detections | `ros_io.py`, message definitions | Boxes, labels, timestamps, IDs are coherent |
| `CURRENT` | Mosaic detections | `ros_io.py`, mosaic consumers | Mosaic output has correct geometry |
| `CURRENT` | Fiducial capture/detections | fiducial config, pipeline services | Fiducial path is distinct and observable |
| `CURRENT` | VLM capture/detections | capture services/topics, VLM config | Captures reach VLM path without ambiguity |
| `CURRENT` | Target preprocessing | tracking launch and node | Input normalization produces expected observations |
| `CURRENT` | Target tracking | tracking launch/package | Tracks persist, merge, expire, and publish |
| `CURRENT` | Video annotation | tracking package annotator nodes | Boxes/tracks are drawn on output image |
| `CURRENT` | Target location/localization | localization nodes, GPS/TF inputs | Target coordinates match source geometry |
| `CURRENT` | Ignore zones | `config/ignore_zones/ignore_zones.yaml` | Suppressed regions are honored |
| `CURRENT` | Mission management | `umd_uas_mission`, mission params | Mission state and advance command work |
| `CURRENT` | Survey operation | survey node/launch/config | Survey starts, progresses, and reports status |
| `CURRENT` | Mosaic capture | mosaic node and service/topic | Capture creates expected product/status |
| `PARTIAL` | Status/health reporting: the status node publishes a consolidated node-status bitmask, GPS/RTK freshness, MAVROS state, heading, and telemetry; the gimbal heartbeat now matches the status consumer, invalid GPS clears GPS/RTK health, and stale RTK is cleared on non-RTK fixes; richer lifecycle states (starting, initialization, intentional stop, and error reason) are not yet represented | `umd_uas/status.py`, `umd_uas/gimbal.py`, `test/test_status_health.py`, `ChimeraStatus.msg` | Source and rebuilt-image bench verified in 5G Drone commit `b269aa0` on 2026-09-16; the richer lifecycle contract remains `PLANNED` |
| `CURRENT` | TF and frame publication | `tf_loc`, gimbal/camera configs | Frames are connected and physically consistent |
| `CURRENT` | Domain bridges send/receive | `*domain_bridge*.yaml`, launch files | Only intended topics/services cross domains, and they all do so successfully |
| `CURRENT` | Foxglove bridge | `foxglove_bridge.launch.py`, layouts | Operator topics/services are advertised and live |
| `CURRENT` | Service-domain forwarding | `service_domain_bridge` launch paths | Remote call-service actions reach owning node |
| `PLANNED` | 5G network transport | docs/networking, router configs, offboard launch | Loss, reconnect, and routing behavior are bounded |
| `CURRENT` | QoS choices for sensor/control data | node source and params | Data is not lost or blocked by incompatible QoS |
| `ORPHANED` | Recording and topic lists (lists should in general record all, the specific lists became largely unnecessary due to smaller image topics and domain ids/domain bridge) | docs/record_topic_list.txt, launch tools | Required evidence can be recorded/replayed |
| `ORPHANED` | Legacy perception/detection nodes | `legacy_nodes/*.py` | Determine active, replaceable, or abandoned paths |
| `CURRENT` | Hardware serial/controller paths | docs, launch params, controller code | Real-drone adaptation boundary is documented |
| `CURRENT` | Calibration and model asset fetch | calibration files, model scripts | Deployment has deterministic required assets |
| `CURRENT` | Parameter precedence | common/container/sim/vehicle YAMLs | Last-wins behavior is intentional and tested |

## Tested checkpoint

On 2026-09-16, 5G Drone commit `a03f9b1` on `feature/ubuntu24-compat`
passed **133 functional tests, 1 skipped** in the onboard `7.1` runtime after
sourcing its MAVROS overlay. Two legacy style-only tests were excluded because
the suite reports pre-existing lint/docstring violations; behavior and package
tests were exercised.

## Future additions to consider

| Intent | Planned feature | Design concern |
|---|---|---|
| `CURRENT` | Explicit real-drone profile from sim profile | Preserve topics, services, and Foxglove layout, and maintain parity between sim and real |
| `PLANNED` | 5G loss/reconnect test mode | Define acceptable stale-data and command behavior |
| `CURRENT` / `Tested: YES (2026-09-16, root 8bd53ee)` | Model/version manifest fingerprint | `px4sim manifest` records the configured model-manifest path and SHA-256, plus the canonical 5G Drone source manifest when the configured artifact bundle differs. Current machine evidence shows `/home/user/deepstream-work/models/manifest.json` is unavailable while the source manifest exists; artifact/source alignment remains follow-up work. |
| `CURRENT` | Deterministic synthetic camera fixtures | Make perception tests repeatable without Gazebo |
| `CURRENT` | Safety interlocks for gimbal | Gimbal pitch roll and yaw should all have chimera version specific bounds, including ROI (our gimbal doesn't set these in hardware unfortunately, so if roi yaw exceeds limit convert it loudly to hold the max angle) |
| `CURRENT` | Standard status heartbeats from nodes for use in the system | Nodes should be able to tell services/px4sim their status with standard states shared across the system |
| `MISSING` | Intermediate fiducial frames | Prevent silent engine/config drift |
| `MISSING` | Scoring frames | The scoring stuff in 3D should not move due to fiducial correction, it should be locked to the known fiducial, make sure this applies to the markers and the backend that does the verdict, I've observed a verdict marker that falls outside the 2m radius scoring bubble in the 3d vis scored correct despite visually being outside the bubble |
| `MISSING` | Useful logging | 5g_drone (and some other repos) have unnecessarily verbose logging. Preserve these on debug logging if they aren't critical for flight, but info and above should only be things needed to confirm a working system while flying (ie detectsions happening, localizations good, mosaic updated, etc), and they should be short |
