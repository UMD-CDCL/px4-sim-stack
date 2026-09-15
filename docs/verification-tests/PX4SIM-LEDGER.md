# px4sim feature ledger

This ledger covers the shell front door, Compose stack, simulator helpers,
scene generation, portability, and operator diagnostics in the current
`px4-sim-stack` source. It is intentionally broad; classify each row after
tracing implementation and running the narrowest possible assertion.

| Status | Feature | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Host doctor/preflight checks | `scripts/preflight.sh`, `px4sim doctor` | Reports actionable host readiness |
| `UNKNOWN` | Portable bootstrap/setup | `scripts/bootstrap.sh`, `px4sim setup` | Fresh checkout can prepare required sources |
| `UNKNOWN` | Dependency/image preparation | `px4sim prepare`, Compose build targets | Online preparation creates all local images |
| `UNKNOWN` | Offline/repeatable restart | `px4sim restart`, Compose profiles | Restart reuses images and avoids network |
| `UNKNOWN` | Start/up/core lifecycle aliases | `px4sim` command dispatch | Aliases select intended profiles |
| `UNKNOWN` | Stop/down/clean/nuke lifecycle | `px4sim`, cleanup scripts | Teardown removes owned resources safely |
| `UNKNOWN` | Status and fleet facts | `px4sim status`, `scripts/fleet.sh` | Addresses, ports, models, streams are truthful |
| `UNKNOWN` | Dynamic fleet add/remove | `px4sim fleet`, `scripts/fleet.sh` | Fleet edits persist and recreate correct services |
| `UNKNOWN` | Fleet numbering/sysid/domain allocation | `scripts/fleet.sh`, `compose.yaml` | UAS 11-19 remain collision-free |
| `UNKNOWN` | Scene/origin/scenario selection | `.env`, `scripts/origin-env.sh`, scenegen | Selected scene and origin reach sim/runtime |
| `UNKNOWN` | World/scenario generation | `modules/scenegen/*.py` | World, models, casualties, terrain are reproducible |
| `UNKNOWN` | Scene editor/server | `editor_server.py`, `editor.html` | Editor loads and changes supported scene data |
| `UNKNOWN` | Terrain and building source ingestion | `sources.py`, `terrain_mesh.py`, `building_mesh.py` | Source assets convert with declared errors |
| `UNKNOWN` | Geographic coordinate conversion | `geo.py`, origin helpers | Coordinates and altitude datum are consistent |
| `UNKNOWN` | PX4 SITL startup | `modules/sim/px4-rcS`, sim entrypoint | PX4 boots with expected instance and params |
| `UNKNOWN` | Gazebo Sim world startup | `modules/sim/entrypoint.sh`, sim Dockerfile | World reaches ready state |
| `UNKNOWN` | Simulated gimbal bridge | PX4 `GZGimbal.*`, patches | Commands and state pass between PX4/Gazebo |
| `UNKNOWN` | SCF4 lens-controller emulation | `scf4_emulator.py`, `scf4_lens_check.py` | Zoom/limits/status emulate hardware contract |
| `UNKNOWN` | Gimbal rangefinder emulation | `gimbal_rangefinder.py` | Range data follows simulated geometry |
| `UNKNOWN` | Camera stream startup | `hold-stream-rates.sh`, sim entrypoint | RGB/low-rate/pilot/thermal streams appear |
| `UNKNOWN` | Stream listing/checking | `scripts/list-streams.py`, `px4sim streams` | Published paths and dimensions are discoverable |
| `UNKNOWN` | Media routing/RTSP/WebRTC | video-router config, Compose | Stream reaches declared RTSP/HTTP endpoints |
| `UNKNOWN` | MAVLink router configuration | router entrypoints/templates | PX4, GCS, ROS, TCP, and keepalive routes work |
| `UNKNOWN` | MAVLink keepalive | `keepalive.py` | Heartbeat policy is isolated and observable |
| `UNKNOWN` | Offboard companion path | `modules/offboard/*`, Compose | Offboard services launch and communicate |
| `UNKNOWN` | Onboard container launch | `modules/onboard/entrypoint.sh` | Runtime assets, env, launch, and teardown work |
| `UNKNOWN` | ROS environment/front door | `ros-env.sh`, `site-params.sh` | Commands source the intended workspace/config |
| `UNKNOWN` | DeepStream/Yolo parser installation | `install-parser.sh`, yolo modules | Parser ABI matches DeepStream and engine |
| `UNKNOWN` | TensorRT/Python runtime compatibility | ROS dependency Dockerfiles | TensorRT import and engine validation work |
| `UNKNOWN` | Foxglove layout rendering | `verify/stages/90-foxglove.sh`, layout tools | Layout settings, topics, services, and frames pass |
| `UNKNOWN` | Foxglove protocol probing | `foxglove_probe.py` | Advertised channels/services carry data |
| `UNKNOWN` | Capture front door | `px4sim capture`, capture scripts | Each supported capture kind reaches consumer |
| `UNKNOWN` | PX4 command front door | `px4sim px4` | Commands are scoped to selected simulated UAS |
| `UNKNOWN` | UAS topic/node inspection | `px4sim uas`, `scripts/state.py` | Status, heading, detections, and state are readable |
| `UNKNOWN` | Gimbal/zoom helper commands | `scripts/zoom.sh`, `sweep-gimbal.py` | Helpers exercise declared control paths |
| `UNKNOWN` | X11/QGC setup | `scripts/x11-allow.sh`, QGC modules | QGC starts with display/auth and video |
| `UNKNOWN` | QGC configuration/autoconnect | `modules/qgc/entrypoint.sh`, Compose | QGC receives intended MAVLink/video endpoints |
| `UNKNOWN` | Documentation/front-door consistency | `docs/front-doors.md`, `px4sim check` | Named commands and variables exist |
| `UNKNOWN` | Cache/layer reuse | Dockerfiles, ccache mounts, build scripts | Incremental source edit avoids unrelated rebuilds |
| `UNKNOWN` | Branch portability contract | `.env`, build contexts, docs | Changed repos are on `feature/ubuntu24-compat` |
| `UNKNOWN` | Logs and readiness markers | Compose logs, entrypoints | “Up” is distinguished from initialized |
| `UNKNOWN` | Evidence retention | `logs/`, verification test design | Test artifacts include date, commits, and status |

## Future additions to consider

| Status | Planned feature | Design concern |
|---|---|---|
| `PLANNED` | Hardware-in-the-loop mode | Keep simulator-only commands from reaching real vehicles |
| `PLANNED` | Real-drone transport profile | Reuse addresses/contracts while replacing SITL/Gazebo |
| `PLANNED` | Automated checkpoint snapshots | Capture image digests, branches, env, and logs together |
| `PLANNED` | Fast targeted rebuild command | Preserve caches while rebuilding one changed service |
| `PLANNED` | CI static front-door verification | Detect abandoned docs/config before deployment |
