# 5G Drone feature ledger

This ledger covers the whole `5g_drone` package, including active and legacy
launch paths. It intentionally includes features that may be abandoned or
only available on real hardware; use `ORPHANED`, `MISSING`, `PLANNED`, or
`BLOCKED` rather than deleting them during the first review.

| Status | Feature | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Onboard launch composition | `launch/onboard.launch.py` | All active nodes, params, namespaces, and remaps resolve |
| `UNKNOWN` | Ground-station launch composition | `launch/ground_station*.launch.py` | Ground bridge, HIL, video, and viz nodes work |
| `UNKNOWN` | Offboard companion launch | `launch/offboard.launch.py` | Companion services and links start |
| `UNKNOWN` | Per-UAS launch variants | `launch/uas1..uas4.launch.py` | Vehicle-specific configurations remain valid |
| `UNKNOWN` | Thermal launch variants | `launch/*thermal.launch.py` | Thermal stream/calibration/path works |
| `UNKNOWN` | No-GPS assessment variants | `launch/*no_gps*.launch.py` | GPS-dependent nodes fail safely or are omitted |
| `UNKNOWN` | Postrun/replay variants | `launch/postrun*.launch.py` | Recorded/postrun inputs are consumed correctly |
| `UNKNOWN` | MAVROS connection and plugin configuration | `config/param_files/px4_*.yaml`, launch files | State, telemetry, commands, and services work |
| `UNKNOWN` | MAVLink system/heartbeat telemetry | MAVROS and router configs | Heartbeats/state are timely and correctly namespaced |
| `UNKNOWN` | Global GPS position | params, `legacy_nodes/detect.py`, consumers | NavSatFix reaches all intended consumers |
| `UNKNOWN` | Local position/odometry | params and localization consumers | Local pose and covariance are valid |
| `UNKNOWN` | Compass heading | MAVROS params and detection code | Heading drives geometry correctly |
| `UNKNOWN` | IMU/attitude | MAVROS params and vehicle nodes | Attitude is available with expected frame |
| `UNKNOWN` | Rangefinder | gimbal rangefinder params and consumers | Range data is valid and bounded |
| `UNKNOWN` | Gimbal command/state | gimbal node, params, services/topics | Ownership, commands, attitude, and feedback work |
| `UNKNOWN` | Camera info/calibration | calibration files, `cam_info` launch | Camera info matches each stream |
| `UNKNOWN` | RGB/gimbal/down/thermal cameras | launch params and camera bridges | Images publish at declared resolution/rate |
| `UNKNOWN` | Camera zoom and framing | `config/foxglove`, zoom/gimbal code | Presets and continuous framing affect image |
| `UNKNOWN` | DeepStream primary detection | `umd_uas/ds_ros_pipeline`, deepstream configs | Engine loads and detections publish |
| `UNKNOWN` | Secondary injury/VLM inference | `config/deepstream/*secondary*`, pipeline code | Secondary results are emitted and correlated |
| `UNKNOWN` | Detection enable/disable | pipeline services/topics and layout | State changes and data flow respond |
| `UNKNOWN` | Detection preview image | `ds_ros_pipeline/ros_io.py` | Preview is encoded and reaches ground |
| `UNKNOWN` | TargetBoxArray detections | `ros_io.py`, message definitions | Boxes, labels, timestamps, IDs are coherent |
| `UNKNOWN` | Mosaic detections | `ros_io.py`, mosaic consumers | Mosaic output has correct geometry |
| `UNKNOWN` | Fiducial capture/detections | fiducial config, pipeline services | Fiducial path is distinct and observable |
| `UNKNOWN` | VLM capture/detections | capture services/topics, VLM config | Captures reach VLM path without ambiguity |
| `UNKNOWN` | Target preprocessing | tracking launch and node | Input normalization produces expected observations |
| `UNKNOWN` | Target tracking | tracking launch/package | Tracks persist, merge, expire, and publish |
| `UNKNOWN` | Video annotation | tracking package annotator nodes | Boxes/tracks are drawn on output image |
| `UNKNOWN` | Target location/localization | localization nodes, GPS/TF inputs | Target coordinates match source geometry |
| `UNKNOWN` | Ignore zones | `config/ignore_zones/ignore_zones.yaml` | Suppressed regions are honored |
| `UNKNOWN` | Mission management | `umd_uas_mission`, mission params | Mission state and advance command work |
| `UNKNOWN` | Survey operation | survey node/launch/config | Survey starts, progresses, and reports status |
| `UNKNOWN` | Mosaic capture | mosaic node and service/topic | Capture creates expected product/status |
| `UNKNOWN` | Status/health reporting | status node and params | Health, rates, GPS, and link state are visible |
| `UNKNOWN` | TF and frame publication | `tf_loc`, gimbal/camera configs | Frames are connected and physically consistent |
| `UNKNOWN` | Domain bridges | `*domain_bridge*.yaml`, launch files | Only intended topics/services cross domains |
| `UNKNOWN` | Foxglove bridge | `foxglove_bridge.launch.py`, layouts | Operator topics/services are advertised and live |
| `UNKNOWN` | Service-domain forwarding | `service_domain_bridge` launch paths | Remote call-service actions reach owning node |
| `UNKNOWN` | 5G network transport | docs/networking, router configs, offboard launch | Loss, reconnect, and routing behavior are bounded |
| `UNKNOWN` | QoS choices for sensor/control data | node source and params | Data is not lost or blocked by incompatible QoS |
| `UNKNOWN` | Recording and topic lists | docs/record_topic_list.txt, launch tools | Required evidence can be recorded/replayed |
| `UNKNOWN` | Legacy perception/detection nodes | `legacy_nodes/*.py` | Determine active, replaceable, or abandoned paths |
| `UNKNOWN` | Hardware serial/controller paths | docs, launch params, controller code | Real-drone adaptation boundary is documented |
| `UNKNOWN` | Calibration and model asset fetch | calibration files, model scripts | Deployment has deterministic required assets |
| `UNKNOWN` | Parameter precedence | common/container/sim/vehicle YAMLs | Last-wins behavior is intentional and tested |

## Future additions to consider

| Status | Planned feature | Design concern |
|---|---|---|
| `PLANNED` | Explicit real-drone profile from sim profile | Preserve topics, services, and Foxglove layout |
| `PLANNED` | 5G loss/reconnect test mode | Define acceptable stale-data and command behavior |
| `PLANNED` | Model/version manifest | Prevent silent engine/config drift |
| `PLANNED` | Deterministic synthetic camera fixtures | Make perception tests repeatable without Gazebo |
| `PLANNED` | Safety interlocks for operator actions | Keep capture, gimbal, mission, and arm controls bounded |
