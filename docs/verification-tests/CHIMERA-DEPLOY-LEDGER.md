# chimera-deploy feature ledger

This is an intentionally loose inventory of the whole `chimera-deploy`
repository. It includes current deployment code, hardware integrations,
operator conveniences, old procedures, and features suggested by filenames or
comments. Do not remove a questionable row during the first pass. Classify
intent with the status tags in the verification README, then add the separate
`Tested: YES|NO|BLOCKED` marker.

The source basis is the repository at the checkpoint recorded in the parent
verification README. Paths below are relative to `chimera-deploy` unless
stated otherwise.

## Deployment and lifecycle

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Full one-command setup | `setup.sh`, `README.md` | A supported aircraft can be installed from documented prerequisites |
| `UNKNOWN` | Local workstation setup | `local_setup.sh` | Laptop dependencies, workspace, router, QGC, Docker, and camera tools install |
| `UNKNOWN` | Remote aircraft setup | `deploy.sh`, `remote/deploy_onboard.sh` | Orin receives the intended files, packages, users, groups, and services |
| `UNKNOWN` | Onboard systemd boot service | `remote/onboard.service`, `deploy_onboard.sh` | Onboard stack starts at boot with correct working directory/environment |
| `UNKNOWN` | Remote camera systemd service | `remote/rcam.service`, camera scripts | Camera server starts, restarts, and exposes expected streams |
| `UNKNOWN` | Local camera systemd service | `local/lcam.service`, `local/start_camera_server.sh` | Laptop camera server can be enabled independently |
| `UNKNOWN` | Deployment idempotence | setup/deploy scripts | Re-running setup does not destroy working data or duplicate services |
| `UNKNOWN` | Package installation manifests | `remote/*install-packages.txt`, `local_setup.sh` | Required apt packages are complete and version-compatible |
| `UNKNOWN` | CUDA/Jetson installation | `remote/cuda-install-packages.txt`, setup docs | Correct CUDA/TensorRT stack is installed for board image |
| `UNKNOWN` | Docker installation and runtime | `local_setup.sh`, Docker files | Docker, Compose, NVIDIA runtime, and permissions work |
| `UNKNOWN` | Docker ROS development container | `docker/ros-dev/*` | Workspace builds interactively with mounted source/config |
| `UNKNOWN` | ROS dependency source refresh | `docker/scripts/refresh_rosdep_src.sh` | Dependency source copies stay synchronized and reproducible |
| `UNKNOWN` | VCS repository import | `docker/chimera.repos`, `local_setup.sh` | All declared repositories and branches are obtained |
| `UNKNOWN` | Git submodule initialization | `.gitmodules`, setup scripts | MAVROS, angles, geographic info, and nested sources resolve |
| `UNKNOWN` | Deployment status/reporting | `deploy.sh`, README commands | Operator can tell installed, running, and failed states apart |
| `UNKNOWN` | Remote update/redeploy | `remote/deploy_onboard.sh` | Code/config updates reach the aircraft without stale artifacts |
| `UNKNOWN` | Log directory ownership/setup | `remote/deploy_onboard.sh` | Docker/systemd logs are writable and retained |
| `UNKNOWN` | Environment persistence | `/etc/environment` writes in deploy script | UAS number, paths, domain, and network values survive reboot |
| `UNKNOWN` | USB/device permission setup | deploy scripts, udev references | Camera, serial, and controller devices are accessible |

## Source, branch, and portability

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Portable source checkout | `docker/chimera.repos`, setup scripts | A new checkout can reproduce the workspace without local paths |
| `UNKNOWN` | Local git mirror/server | `setup_git_server.sh` | Bare mirrors, links, daemon, and client config work offline |
| `UNKNOWN` | Multi-machine source sync | `setup_git_server.sh sync/deploy` | Laptop and aircraft revisions converge and report failures |
| `UNKNOWN` | Concurrent aircraft sync | `setup_git_server.sh` sync logic | Multiple clients update without hiding an individual failure |
| `UNKNOWN` | Branch selection/pinning | repos files and setup scripts | Intended branches, commits, and forks are explicit |
| `UNKNOWN` | Offline deployment after sync | git daemon and local mirrors | Aircraft can rebuild using local sources without GitHub |
| `UNKNOWN` | Ubuntu/JetPack portability | install docs, Dockerfiles, package lists | Host/board release differences are detected or documented |
| `UNKNOWN` | Real versus simulated source reuse | package/config layout | Real aircraft can use contracts without simulator-only paths |
| `UNKNOWN` | Configuration migration | deploy scripts and `.env` writes | Old UAS naming/config is migrated without silent loss |
| `UNKNOWN` | Secret/credential handling | scripts and config files | Keys, tokens, and private endpoints are not accidentally copied |

## Aircraft identity, networking, and time

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | UAS number and identity | `remote/deploy_onboard.sh`, env files | Identity selects correct namespace, ports, and model |
| `UNKNOWN` | Ethernet/wired network setup | networking docs, `main.conf` | Wired aircraft link reaches companion and ground station |
| `UNKNOWN` | Wi-Fi setup | `setup.sh`, networking docs | Required SSID/addressing procedure is reproducible |
| `UNKNOWN` | 5G modem/network setup | `docs/echopilot_ai/network_setup_5G_modem.md` | Modem connects, routes, and reconnects |
| `UNKNOWN` | 5G link QoS/failure behavior | `docs/air_link_qos.md`, router configs | Loss, delay, and reconnect behavior are bounded |
| `UNKNOWN` | Static IP/address allocation | `remote/main.conf.template`, local config | Endpoint addresses are correct for each deployment |
| `UNKNOWN` | MAVLink router endpoints | `remote/main.conf.template`, `local/main.conf` | PX4, ROS, GCS, TCP tools, and keepalive routes work |
| `UNKNOWN` | Router source filtering | `AllowSrcSysIn` config | Unwanted system IDs cannot inject into filtered links |
| `UNKNOWN` | Router keepalive | router configs/scripts | Heartbeat remains isolated from mission/control traffic |
| `UNKNOWN` | Chrony/NTP time synchronization | `deploy-chrony.sh`, `local/chrony.conf` | Aircraft and ground clocks converge and remain healthy |
| `UNKNOWN` | Time source fallback | chrony config/docs | Loss of preferred source does not produce unsafe timestamps |
| `UNKNOWN` | Host firewall/port exposure | setup docs and configs | Required ports work without exposing unintended services |
| `UNKNOWN` | Network diagnostics | useful commands/docs, scripts | Operator can distinguish link, route, router, and ROS failures |

## MAVLink, PX4, and vehicle control

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | PX4 serial connection | `remote/main.conf.template`, setup docs | Flight controller is reachable through the expected device |
| `UNKNOWN` | PX4 firmware deployment | `local/*.px4`, setup/deploy docs | Correct firmware variant is installed and identifiable |
| `UNKNOWN` | CRSF RC modification | `local/echomav_echopilot-ai_default_crsf_rc_mod.px4` | RC behavior matches aircraft wiring/configuration |
| `UNKNOWN` | Rangefinder firmware modification | `local/echomav_echopilot-ai_default-rangefinder-mod.px4` | Rangefinder parameters and driver behavior are correct |
| `UNKNOWN` | MAVROS source build/patch | `remote/mavros_patch/*` | Pinned MAVROS builds against intended PX4 protocol |
| `UNKNOWN` | MAVROS GeographicLib dataset | `remote/install_geographiclib_datasets.sh` | Required geoid data is present on aircraft |
| `UNKNOWN` | MAVROS plugin configuration | ROS source/config copied by deploy | Required plugins load and expose services/topics |
| `UNKNOWN` | MAVLink command routing | main configs and launch docs | Commands reach the intended UAS and acknowledgements return |
| `UNKNOWN` | Ground station endpoint | router config, QGC setup | QGC sees telemetry and can issue intended commands |
| `UNKNOWN` | MAVSDK/pymavlink TCP access | router TCP server/docs | Tools can connect without disrupting flight path |
| `UNKNOWN` | Vehicle heartbeat and health | router/MAVROS/status docs | Health is visible locally and over the air link |

## Cameras, video, and recording

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Onboard CSI camera startup | `remote/remote_start_camera_server.sh`, `rtsp_config.py` | Camera opens with correct sensor and format |
| `UNKNOWN` | Local camera startup | `local/start_camera_server.sh` | Laptop camera pipeline starts independently |
| `UNKNOWN` | Argus camera source | `rtsp_config.py` | Jetson hardware source and controls work |
| `UNKNOWN` | V4L2 device discovery | `remote/v4l2_devices.py` | Cameras are identified by stable properties, not enumeration order |
| `UNKNOWN` | Multi-camera stream creation | `multi_rtsp_server.py`, forked server | All configured cameras publish distinct paths |
| `UNKNOWN` | RGB/pilot/thermal stream naming | recording/view scripts and configs | Names agree with consumers and operator docs |
| `UNKNOWN` | RTSP server | `remote/forked_rtsp_server.py`, `local/multi_rtsp_server.py` | RTSP clients connect and receive frames |
| `UNKNOWN` | RTSP producer watchdog | forked server/tests | Dead producers are detected and recovered/reported |
| `UNKNOWN` | Encoder selection and bitrate | camera/server scripts | Hardware/software encoder fallback is explicit |
| `UNKNOWN` | Stream resolution/framerate | `rtsp_config.py`, scripts | Advertised media matches calibration and expected load |
| `UNKNOWN` | Camera flip/orientation | camera scripts | Image orientation matches vehicle frame |
| `UNKNOWN` | Camera calibration selection | configs and 5G Drone integration | Camera info matches sensor/resolution/profile |
| `UNKNOWN` | RTSP viewing | `view_rtsp_streams.sh` | Operator can inspect every configured stream |
| `UNKNOWN` | RTSP recording | `record_rtsp_streams.sh`, `record_nv_streams.sh` | Recordings start, rotate/stop, and are timestamped |
| `UNKNOWN` | Local/remote all-stream recording | `*_record_all.sh` | One command records required ROS/video evidence |
| `UNKNOWN` | TS-to-MP4 conversion | `convert-ts-to-mp4-batch.sh` | Recorded transport streams convert without loss/error |
| `UNKNOWN` | YOLO video processing | `local/yolo-video.py` | Offline inference consumes recorded video |
| `UNKNOWN` | YOLO MCAP processing | `local/yolo-mcap.py` | Offline inference consumes ROS recordings |

## ROS, perception, and operator integration

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | ROS 2 Humble workspace installation | package lists, `docker/ros-dev` | Workspace resolves and builds |
| `UNKNOWN` | 5G Drone package deployment | `docker/rosdep_src/5g_drone`, repos | Correct package reaches aircraft |
| `UNKNOWN` | MAVInsight deployment | repos/config and ROS workspace | Visualization package builds and launches |
| `UNKNOWN` | Tracking package deployment | repos and package lists | Tracking code and models reach aircraft |
| `UNKNOWN` | Domain bridge configuration | 5G Drone configs copied by deploy | Intended vehicle/ground topics cross domains |
| `UNKNOWN` | Foxglove bridge availability | launch/config and README | Operator can connect to aircraft bridge |
| `UNKNOWN` | ROS topic recording | `local_record_all.sh`, useful commands | Required topics are recorded with MCAP |
| `UNKNOWN` | Detection model deployment | model fetch references, install scripts | Engine and labels match runtime |
| `UNKNOWN` | YOLO/TensorRT dependency deployment | package lists and Docker files | Inference starts on target hardware |
| `UNKNOWN` | Target detections and tracking | 5G Drone package references | Detections, tracks, and health reach ground |
| `UNKNOWN` | Mission/survey integration | launch/config references | Mission actions and survey state are usable |
| `UNKNOWN` | Manual detector GUI | package manifests and old launch refs | Determine whether GUI is active or abandoned |
| `UNKNOWN` | Injury/VLM inference | model/config references | Determine hardware, model, and operator contract |
| `UNKNOWN` | Capture/mosaic workflows | scripts/docs and package refs | Capture commands create expected products |

## Hardware and maintenance

| Status | Feature | Evidence to inspect | Intended evidence |
|---|---|---|---|
| `UNKNOWN` | Jetson Orin BSP installation | `setup.sh`, deploy docs | Board boots with supported BSP |
| `UNKNOWN` | Kernel/bootloader flashing | `setup.sh` L4T commands | Flash procedure is safe and reproducible |
| `UNKNOWN` | USB serial rules | setup docs | Flight controller/controller device names are stable |
| `UNKNOWN` | USB camera rules | setup docs, V4L2 code | Camera identity survives re-enumeration |
| `UNKNOWN` | Ethernet adapter configuration | setup docs | Wired link has intended address/routes |
| `UNKNOWN` | TP-Link adapter setup | hardware docs | Adapter is configured for field network |
| `UNKNOWN` | EchoPilot AI hardware setup | `docs/echopilot_ai/*` | Board-specific peripherals are installed |
| `UNKNOWN` | Rangefinder hardware | firmware/config/docs | Rangefinder data is calibrated and available |
| `UNKNOWN` | Lens/gimbal controller hardware | 5G Drone and local docs | Controller serial/USB path is operational |
| `UNKNOWN` | Reboot and recovery procedure | useful commands, systemd units | Operator can recover services without corrupting data |
| `UNKNOWN` | Service watchdog/restart policy | systemd files and scripts | Crashed camera/onboard services recover predictably |
| `UNKNOWN` | Disk/storage management | recording scripts and setup docs | Recording cannot silently fill the aircraft disk |
| `UNKNOWN` | Log rotation/retention | systemd/docker/record scripts | Logs remain available at useful retention |

## Legacy, partial, and suspected future scope

| Status | Candidate feature | Why it is included | Review action |
|---|---|---|---|
| `UNKNOWN` | Old local ROS workspace procedure | `Useful Commands Tidy`, legacy paths | Decide whether it is supported or `ORPHANED` |
| `UNKNOWN` | Gazebo Classic deployment | `docker_files/px4_sim_gazebo_classic` in 5G Drone | Decide whether this repo still supports it |
| `UNKNOWN` | Native versus container QGC | `local/qgc/*`, setup docs | Define one supported operator path |
| `UNKNOWN` | Native versus container camera server | local/remote scripts | Define ownership and port contracts |
| `UNKNOWN` | Legacy multi-RTSP implementations | local and remote copies | Compare behavior and retire one if duplicated |
| `UNKNOWN` | Legacy perception nodes | copied package/source references | Identify replaced nodes and remove stale docs later |
| `PLANNED` | Fully declarative aircraft manifest | scattered env/config currently duplicate identity | Make deployment portable across airframes |
| `PLANNED` | Automated hardware inventory | board/camera/USB/network checks | Select correct configs without hand editing |
| `PLANNED` | Air-link degradation test harness | docs mention QoS but no clear harness | Establish reconnect and stale-data limits |
| `PLANNED` | Signed/versioned deployment bundle | current scripts copy mutable sources | Prevent mixed-version aircraft deployments |
| `PLANNED` | Factory reset/reprovision workflow | setup is install-focused | Recover an aircraft to a known baseline |
| `PLANNED` | Remote health dashboard | many services/logs but fragmented visibility | Expose deployment and runtime health in Foxglove |
| `PLANNED` | Backup/restore of calibration and logs | files are spread across local/remote paths | Preserve field evidence and calibration |
| `PLANNED` | CI deployment smoke test | test files exist but no complete deployment test | Catch stale package lists and units |

## Review rule

For every row, search the named paths and callers. If a feature is described
but unreachable, mark it `ORPHANED`; if the current deployment contract still
requires it, mark it `MISSING`; if it is deliberately future scope, mark it
`PLANNED`. Add `Tested: YES` only after a stage-specific assertion and record
the date, repository commits, Git status, and retained evidence.
