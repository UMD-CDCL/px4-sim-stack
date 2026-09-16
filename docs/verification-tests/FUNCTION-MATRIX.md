# Verification function matrix

This is the review document for the staged test plan. The source column names
the code or protocol that must be checked, rather than an assumed behavior.

| Stage | Function to verify | Source of contract | Evidence / pass condition | Current confidence |
|---|---|---|---|---|
| Code | `px4sim` front door commands, profiles, variables, and restart path | `px4sim`, `compose.yaml`, `docs/front-doors.md` | `./px4sim check` passes and each named command resolves | Confirmed at baseline |
| Code | Foxglove layout parses and all panel settings are accepted | `config/foxglove/chimera_sim.json`, `foxglove_layout.py` | Layout validation passes; no unknown panel setting | Confirmed at baseline |
| Code | Layout topics and services have implementation owners | `chimera_sim.json`, 5G Drone launch files, MAVInsight sources | Every drawn topic/service maps to a launch node, bridge, or documented external source | Needs complete mapping |
| Code | 5G Drone launch composition | `5g_drone/launch/onboard.launch.py`, `ground_station2.launch.py` | All expected executables and parameter files exist | Partially confirmed |
| Code | MAVInsight build and entry points | MAVInsight package metadata and launch files | Package builds; executable names resolve | Needs explicit test |
| Code | Models and runtime assets | 5G Drone model fetch/config code, compose mounts | Required assets exist; optional assets are labeled | ReID asset currently missing |
| Bench | `px4sim` starts, reports status, and restarts cleanly | `px4sim`, Compose | All expected containers remain running after restart; QGC remains singleton; restart waits for PX4, camera, video, and QGC readiness | Confirmed 2026-09-16 at `3402e88`; one QGC container |
| Bench | Foxglove ground bridge accepts the Foxglove protocol | `verify/component/foxglove_probe.py`, bridge launch | Bridge advertises expected layout topics/services and returns data where applicable | Partially confirmed |
| Bench | Foxglove camera front door | `chimera_sim.json`, video-router config, camera nodes | Configured image and camera-info topics carry frames/calibration | RGB pipeline confirmed; synchronized recording remains open |
| Bench | 5G Drone camera, zoom, framing, and SCF4 emulation | `scf4_emulator.py`, camera launch/config | Commands change observable zoom/framing state without hardware | Needs scripted action/assertion |
| Bench | Gimbal control path without GPS | gimbal node, PX4 gimbal bridge, `GZGimbal.*`, MAVROS services | Gimbal command/service produces state feedback or a documented bounded failure | Needs explicit test |
| Bench | Perception pipeline startup without GPS | DeepStream node and deployment config | `pipelines PLAYING, services up`; detection topics emit data from test frames | DeepStream startup confirmed; data assertion pending |
| Bench | Mission/status/vehicle nodes and domain bridges | `onboard.launch.py`, node sources, bridge YAML | Nodes stay alive and expected local topics/services are present | Needs explicit inventory |
| GPS | MAVROS connection and vehicle state | MAVROS launch/config, mavlink-router, PX4 SITL | State, heartbeat, command, and telemetry topics produce valid data | MAVROS startup confirmed; assertions pending |
| GPS | Geographic datum/geoid behavior | MAVROS GeographicLib use, site params, `egm96-5` dataset | Geoid file exists and reported altitude/datum matches configured site | Dataset fixed; value assertion pending |
| GPS | Global/local position and transforms | MAVROS, TF, localization nodes, site config | GPS fix, pose, TF, and map placement agree within declared tolerance | Needs tolerance review |
| GPS | GPS-dependent target localization and map visualization | tracking/localization sources and `chimera_sim.json` | GPS-dependent topics carry data and Foxglove panels render them | Needs source-to-panel mapping |
| Sim | PX4 SITL flight lifecycle | PX4 startup scripts, sim compose service | Arm/takeoff/flight/land or equivalent scripted state transitions complete | Needs test controls defined |
| Sim | MAVLink routing and 5G companion path in flight | router configs, 5G Drone launch, MAVInsight | Commands and telemetry traverse configured UDP/TCP paths | Needs packet/topic evidence |
| Sim | In-flight camera/perception loop | Gazebo camera topics, video router, DeepStream, tracking | Live image, detections, tracks, and annotations reach Foxglove | Full sim passed at `bd7122b`; repeat after current image checkpoint |
| Sim | Mission and survey behavior | mission/survey nodes and configured services | Mission action/service completes and status is visible at the front door | Needs service contract review |
| Sim | Foxglove operator surface during flight | `chimera_sim.json`, Foxglove bridge | Every panel has advertised data, valid schema, and live updates | Layout validation only at baseline |
| Sim | Restart/repeatability after flight | `px4sim restart`, stage teardown | Clean teardown and repeat start preserve the same contracts | Restart container test confirmed |
