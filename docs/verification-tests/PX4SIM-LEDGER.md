# px4sim feature ledger

This ledger covers the shell front door, Compose stack, simulator helpers,
scene generation, portability, and operator diagnostics in the current
`px4-sim-stack` source. It is intentionally broad; classify each row after
tracing implementation and running the narrowest possible assertion.

`Status` records intended scope, not verification. Add `Tested: YES|NO|BLOCKED`
with date, commit, Git status, and evidence; this inventory defaults to
`Tested: NO`.

| Intent | Feature | Source evidence to inspect | Intended evidence |
|---|---|---|---|
| `CURRENT` | Host doctor/preflight checks | `scripts/preflight.sh`, `px4sim doctor` | Reports actionable host readiness |
| `CURRENT` | Portable bootstrap/setup | `scripts/bootstrap.sh`, `px4sim setup` | Fresh checkout can prepare required sources |
| `CURRENT` | Dependency/image preparation | `px4sim prepare`, Compose build targets | Online preparation creates all local images |
| `CURRENT` | Offline/repeatable restart | `px4sim restart`, Compose profiles | Restart reuses images and avoids network |
| `CURRENT` | Start/up/core lifecycle aliases | `px4sim` command dispatch | Aliases select intended profiles |
| `CURRENT` | Stop/down/clean/nuke lifecycle | `px4sim`, cleanup scripts | Teardown removes owned resources safely |
| `CURRENT` | Status and fleet facts | `px4sim status`, `scripts/fleet.sh` | Addresses, ports, models, streams are truthful |
| `TESTED` | Dynamic fleet selection, including non-monotonic active slots (for example 1,3,4), keyed from the single source in `.env` | `UAS_ACTIVE`, `px4sim fleet`, `scripts/fleet.sh`, simulator/offboard/router entrypoints | Code-tested and runtime-tested 2026-09-16: slots preserve `uas11`, `uas13`, `uas14` and model mapping; invalid/duplicate slots reject; selected services and telemetry run, QGC remains singleton. Evidence: `verify/evidence/sparse-fleet/`. Three-vehicle preview media is resource-limited on this host and remains a documented limitation. Compose retains its declared service ceiling. |
| `CURRENT` | Fleet numbering/sysid/domain allocation | `scripts/fleet.sh`, `compose.yaml` | UAS 11-19 remain collision-free |
| `CURRENT` | Scene/origin/scenario selection in `px4sim ui`; choose valid combinations, with scenarios constrained to parent scenes so the scenario determines scene, scenario, and origin | `.env`, `.workflow`, `scripts/origin-env.sh`, scenegen, `scripts/tui.py` | Source/code-tested 2026-09-16: scene and scenario actions exist, scenario choices come from runtime config, and origin is exposed; live selection/respawn remains unverified |
| `CURRENT` | World/scenario generation | `modules/scenegen/*.py` | World, models, casualties, terrain are reproducible |
| `CURRENT` | Scene editor/server | `editor_server.py`, `editor.html` | Editor loads and changes supported scene data |
| `CURRENT` | Terrain and building source ingestion | `sources.py`, `terrain_mesh.py`, `building_mesh.py` | Source assets convert with declared errors |
| `CURRENT` | Geographic coordinate conversion | `geo.py`, origin helpers | Coordinates and altitude datum are consistent |
| `CURRENT` | PX4 SITL startup | `modules/sim/px4-rcS`, sim entrypoint | PX4 boots with expected instance and params |
| `CURRENT` | Gazebo Sim world startup | `modules/sim/entrypoint.sh`, sim Dockerfile | World reaches ready state |
| `CURRENT` | Simulated gimbal bridge | PX4 `GZGimbal.*`, patches | Commands and state pass between PX4/Gazebo |
| `CURRENT` | SCF4 lens-controller emulation | `scf4_emulator.py`, `scf4_lens_check.py` | Zoom/limits/status emulate hardware contract |
| `CURRENT` | Gimbal rangefinder emulation | `gimbal_rangefinder.py` | Range data follows simulated geometry |
| `CURRENT` | Camera stream startup | `hold-stream-rates.sh`, sim entrypoint | RGB/low-rate/pilot/thermal streams appear |
| `CURRENT` | Stream listing/checking | `scripts/list-streams.py`, `px4sim streams` | Published paths and dimensions are discoverable |
| `CURRENT` | Media routing/RTSP/WebRTC | video-router config, Compose | Stream reaches declared RTSP/HTTP endpoints |
| `CURRENT` | MAVLink router configuration | router entrypoints/templates | PX4, GCS, ROS, TCP, and keepalive routes work |
| `CURRENT` | MAVLink keepalive | `keepalive.py` | Heartbeat policy is isolated and observable |
| `CURRENT` | Offboard viz/control companion path | `modules/offboard/*`, Compose | Offboard services launch and communicate |
| `CURRENT` | Onboard container launch | `modules/onboard/entrypoint.sh` | Runtime assets, env, launch, and teardown work |
| `CURRENT` | ROS environment/front door | `ros-env.sh`, `site-params.sh` | Commands source the intended workspace/config |
| `CURRENT` | DeepStream/Yolo parser installation | `install-parser.sh`, yolo modules | Parser ABI matches DeepStream and engine |
| `CURRENT` | TensorRT/Python runtime compatibility | ROS dependency Dockerfiles | TensorRT import and engine validation work |
| `CURRENT` | Foxglove layout rendering | `verify/stages/90-foxglove.sh`, layout tools | Layout settings, topics, services, and frames pass |
| `CURRENT` | Foxglove protocol probing | `foxglove_probe.py` | Advertised channels/services carry data |
| `TESTED (2026-09-16, commit 8074c4b)` | Capture front door | `px4sim capture` now retries transient ROS discovery; live full sim coverage passed mosaic, fiducial, and VLM capture paths, including ground delivery and overlay export | `px4sim`, `verify/stages/80-captures.sh` |
| `TESTED (live 2026-09-16)` | PX4 command front door | `px4sim px4` | `./px4sim px4 1 commander status` reached the selected simulated UAS and returned commander state (`Armed`, navigation mode, failsafe, and cycle statistics). Non-mutating command path is verified; mutating command and cleanup remain unverified. | `px4sim`, PX4 shell |
| `TESTED (live 2026-09-16)` | UAS topic/node inspection | `px4sim state`/UI reports link health, MAVLink message rate, system IDs, position, altitude, heading, gimbal attitude, GPS fix, satellites, explicit `mission_phase`, and a cached per-UAS detection summary (`state`, `boxes`, `error`). The ROS probe is bounded at `1.5s`, below the two-second watch period; live calls return either `live, boxes=N` or explicit `unavailable`. Dedicated `px4sim uas N detections` remains the detailed box/localization front door. | `scripts/state.py`, `scripts/mavlink.py`, `scripts/tui.py`, `px4sim uas` |
| `CURRENT` | Gimbal/zoom helper commands | `scripts/zoom.sh`, `sweep-gimbal.py` | Helpers exercise declared control paths |
| `CURRENT` | X11/QGC setup | `scripts/x11-allow.sh`, QGC modules | QGC starts with display/auth and video |
| `CURRENT` | QGC configuration/autoconnect | `modules/qgc/entrypoint.sh`, Compose | QGC receives intended MAVLink/video endpoints |
| `TESTED` | QGC singleton across repeated restarts | `verify/qgc-restart-test.sh`, QGC entrypoint lock | 2026-09-16, checkpoint `d6ba06d` and rerun evidence: two consecutive `./px4sim restart --no-build` cycles, one process each, no singleton error |
| `CURRENT` | Documentation/front-door consistency | `docs/front-doors.md`, `px4sim check` | Named commands and variables exist |
| `TESTED (live 2026-09-16)` | Cache/layer reuse | Dockerfiles, ccache mounts, build scripts, `verify/build-cache.py` | Cache matrix passed offline with the local Docker builder: unchanged build `1.9s`, isolated edits rebuilt only `umd_uas`, `mavinsight`, parser, or `cdcl_umd_msgs`, restored inputs rebuilt nothing, and assembled ROS type support passed. |
| `CURRENT` | Branch portability contract | `.env`, build contexts, docs | Changed repos are on `feature/ubuntu24-compat` |
| `CURRENT` | Docker logs and readiness markers | Compose logs, entrypoints | “Up” is distinguished from initialized |
| `CURRENT` | Evidence retention | `logs/`, verification test design | Test artifacts include date, commits, and status |
| `TESTED (live 2026-09-16)` | Portability manifest front door | `px4sim manifest` | Emits UTC checkpoint, world/config/layout/fleet, repository branches and commits with dirty counts, and local Compose image IDs/digests; live output confirmed all compatibility branches and canonical `chimera_sim.json`. |

## Future additions to consider

| Intent | Planned feature | Design concern |
|---|---|---|
| `CURRENT` | Hardware-in-the-loop mode | Keep simulator-only commands from reaching real vehicles |
| `CURRENT` | Real-drone transport profile | Reuse addresses/contracts while replacing SITL/Gazebo |
| `CURRENT` | Automated checkpoint snapshots | Capture image digests, branches, env, and logs together |
| `CURRENT` | Fast targeted rebuild command | Preserve caches while rebuilding one changed service |
| `CURRENT` | CI static front-door verification | Detect abandoned docs/config before deployment |

## `px4sim ui` front door inventory

This section is transcribed from `scripts/tui.py`'s `ACTIONS` table and the
`px4sim` dispatch paths. The action table is the implementation source for
keys, menus, prompts, confirmations, world restrictions, and the command each
action launches. Items mentioned in older documentation but absent from that
table should be reviewed as possible `ORPHANED` features.

| Intent | UI surface/action | Interaction and behavior to verify | Code path / likely issue |
|---|---|---|---|
| `CURRENT` | Stack pane | Shows services, vehicles, streams, and command output: add option when commanding over a running command to stop the currently running command. Also simplify the live reporting so you can fit more data like detections, locs status, bandwidth, etc for each drone, and simplify streams and make the reporting more reliable, sometimes a running stream reports down or vice versa | `tui.py` panes and `state --watch` |
| `CURRENT` | Live state feed | Refreshes about every 2 seconds and retries after feed failure: add the ability to get a fresh report when opening px4sim, sometimes it takes a long time to get one | `Feed`, `REPORT_PERIOD_S`, `FEED_RETRY_S` |
| `CURRENT` | Stale-feed indication | Reports when state has not refreshed within the stale threshold | `STALE_REPORT_S`, rendering code |
| `CURRENT` | Service selection | Tab moves to services; selected service exposes actions and configurable health signals beyond simple up/down where available | `PANES`, service rows |
| `CURRENT` | Vehicle selection | Tab moves to vehicles; selected vehicle substitutes UAS values: make sure it's using the same stuff as the foxglove front door too, since that is the single source although px4sim should be able to replicate that, but faithful replication is a condition for this feature passing | vehicle row/action formatting |
| `CURRENT` | Stream selection | Tab moves to streams; selected stream can be played, snapped, or recorded; reconcile this with recording improvements | stream pane/actions |
| `MISSING` | UAS_NUM-based `.env` template | Generate aircraft-specific values from the real drone's UAS number, including the fleet visible to that drone | `.env`, fleet, and deployment code; exact fields require source review |
| `CURRENT` | `r` restart | Runs `./px4sim restart` and refreshes state | `Action(STACK, "r")` |
| `CURRENT` | `s` start | Runs `./px4sim start`; should be non-disruptive if already running | `Action(STACK, "s")` |
| `CURRENT` | Bench enable menu | Prompts for exact phrase, confirms unsafe GPS-free mode, refreshes | `bench enable`, world ground/air only |
| `CURRENT` | Bench disable menu | Stops stack and disables bench mode after confirmation | `bench disable`, world ground/air only |
| `CURRENT` | `x` stop | Confirmation then stops every container | `Action(STACK, "x")` |
| `CURRENT` | `=` add vehicle | Prompts for airframe, changes fleet, reloads world | `fleet add`, simulator only |
| `CURRENT` | `P` place fleet | Confirms respawn at start position | `place`, simulator only |
| `CURRENT` | `A` place targets | Recreates scenario targets | `scenario` with no value |
| `CURRENT` | `N` switch scene | Prompts for scene, writes selection, refreshes; scenario-parent selection remains a separate live verification item | `scene`, simulator/ground |
| `CURRENT` | `T` switch targets: renamed to switch scenario | Prompts from available scenarios and reloads | `scenario`, simulator/ground |
| `CURRENT` | `F` move fiducial | Prompts east/north metres or lat/lon and changes marker | `fiducial`, simulator only |
| `TESTED (code/static)` | Verification stage selector | The stack menu now selects `code`, `bench`, `gps`, or `sim` and invokes the existing named stage through `px4sim verify`; live interactive selection remains to be exercised | `scripts/tui.py`, `verify/run.sh` |
| `CURRENT (menu-only)` | Code/front-door check | Runs compose/layout/doc consistency checks without consuming a hotkey | `check` |
| `CURRENT (menu-only)` | Host check | Runs host/GPU/Docker/X11/preflight checks without consuming a hotkey | `doctor` |
| `CURRENT (menu-only)` | Layout location | Prints selected Foxglove layout path and loading instructions without consuming a hotkey | `layout` |
| `CURRENT` | `K` PX4 console | Opens foreground PX4 shell and supports detach sequence | `console`, simulator only |
| `CURRENT` | Ground ROS graph action | Probes ground ROS nodes/topics | `probe ground`, only with ground |
| `CURRENT` | Ground Foxglove action | Probes ground bridge channels/services | `foxglove ground`, only with ground |
| `CURRENT` | Endpoint display | Prints MAVLink, video, Foxglove, and per-UAS addresses | `endpoints` |
| `CURRENT` | Fleet display | Prints full fleet facts | `fleet` |
| `CURRENT` | Stream display | Prints live stream paths and media facts | `streams` |
| `CURRENT` | Origin display | Prints scene coordinates and datum inputs | `origin`, scene worlds |
| `CURRENT` | `clean` | Confirmation then removes containers, networks, and volumes | `clean` |
| `CURRENT` | Service log follow | Follows selected service log | `logs service` |
| `CURRENT` | Service recent logs | Reads selected service's last 15 minutes | `logs --since 15m service` |
| `CURRENT` | Service shell | Opens a foreground shell in selected service | `shell service` |
| `CURRENT` | Vehicle status | Shows mode, arm, position, battery, GPS, and gimbal state | `uas N status` |
| `CURRENT` | Vehicle takeoff | Prompts altitude and sends takeoff | `uas N takeoff`, sim only |
| `CURRENT` | Vehicle land | Sends land command | `uas N land`, sim only |
| `CURRENT` | Vehicle fly | Confirms respawn then climb | `fly N height`, sim only |
| `CURRENT` | Vehicle arm | Sends arm command | `uas N arm`, sim only |
| `CURRENT` | Vehicle gimbal | Prompts pitch and sends gimbal command | `uas N gimbal degrees` |
| `CURRENT` | Vehicle zoom | Prompts from zoom presets | `zoom N preset` |
| `CURRENT` | Vehicle detection | Prompts on/off and changes detection state | `uas N detect on/off` |
| `CURRENT` | Vehicle capture | Prompts from configured capture kinds | `capture N kind` |
| `CURRENT` | Vehicle ROS probe | Probes selected vehicle graph | `probe N` |
| `CURRENT` | Vehicle camera view | Foreground stream viewer | `view N` |
| `CURRENT` | Vehicle frame snapshot | Saves one frame from selected gimbal stream | `snap stream`, sim only |
| `CURRENT` | Vehicle retirement | Confirms removal and optional renumbering | `fleet remove N --renumber`, sim only |
| `CURRENT` | Companion log | Follows selected vehicle companion log | `logs companion`, sim/air |
| `CURRENT` | Router log | Follows vehicle router log | `logs router`, simulator only |
| `CURRENT` | Detections report | Prints detections and projected locations | `uas N detections` |
| `CURRENT` | Heading report | Prints vehicle/camera/footprint direction | `uas N heading` |
| `CURRENT` | Scene report | Prints scene supplied to 3D panel | `uas N scene` |
| `CURRENT` | Vehicle goto | Prompts east/north/up and sends simulated goto | `uas N goto`, sim only |
| `CURRENT` | Vehicle topics | Prints ROS topics | `topics N` |
| `CURRENT` | Vehicle Foxglove probe | Probes vehicle bridge | `foxglove N` |
| `CURRENT` | Raw PX4 command | Prompts arbitrary PX4 command | `px4 N command`, sim only |
| `CURRENT` | Companion shell | Opens shell in onboard/companion service | `onboard N`, sim/air |
| `CURRENT` | Stream play | Foreground viewer for selected stream | `view stream` |
| `CURRENT` | Stream snapshot | Saves selected stream frame | `snap stream`, sim only |
| `CURRENT` | Tab navigation | Cycles service, vehicle, and stream panes | `handle_key`, pane state |
| `CURRENT` | Enter action menu | Opens actions for selected row | menu rendering/dispatch |
| `CURRENT` | `.` stack menu | Opens stack-level actions | documented in `tui.py` docstring |
| `CURRENT` | Page-up output history | Scrolls last command output | `OUTPUT_LINES`, output pane |
| `CURRENT` | `:` arbitrary command | Runs any px4sim command typed by operator | command prompt/parser |
| `CURRENT` | `?` action help | Lists available actions and exact command | action table renderer |
| `CURRENT` | Escape cancellation | Stops foreground/running command process group | `stop_process`, SIGTERM |
| `CURRENT` | `q` exit | Closes feed and leaves curses cleanly | `Feed.close`, curses teardown |
| `CURRENT` | World filtering | Hides simulator-only actions on ground/air worlds | `worlds`, `world_of()` |
| `CURRENT` | Prompt defaults | Uses scene, scenario, model, stream, and UAS values | `Ask`, substitutions |
| `CURRENT` | Tab autocomplete available options | Source/code-tested 2026-09-16: typed-command candidates include front-door commands plus configured scene, scenario, model, stream, zoom, and live UAS values | `Ask`, `command_candidates` |
| `CURRENT` | Confirmation handling | Requires explicit confirmation for destructive/flying actions | `Action.confirm` |
| `CURRENT` | Terminal width fallback: double check me on this, but the terminal seems fine as is but should work with arbitrary widths within reason | UI claims 100-column usability; narrow terminals need verification | module docstring, rendering widths |
| `CURRENT` | Terminal resize handling: double check me on this, but the terminal seems fine as is but should work with arbitrary widths within reason | Determine whether panes redraw safely after SIGWINCH | curses setup/render loop |
| `CURRENT` | Unicode/ANSI stripping | State/log output should not corrupt rows or widths | `ESCAPE_CODES`, `strip_codes` |
| `CURRENT` | Broken `state --watch` recovery | Feed retries while command actions continue safely | `Feed.run`, subprocess lifecycle |
| `CURRENT` | Long-running command output | Output cap, scrollback, and cancellation preserve useful evidence | `OUTPUT_LINES`, `Runner` |
| `CURRENT` | Foreground command cleanup | Viewer/console processes release sockets on Escape/q | `stop_process`, process groups |
| `CURRENT` | Empty fleet/stack rendering | UI remains usable when no services or UAS exist | state parsing/rendering |
| `CURRENT` | Real-aircraft world safety | Simulator-only keys are absent and typed commands are guarded | world filtering and `check_mode` |
| `CURRENT` | UI key for `fleet add` outside simulator menus | `ACTIONS` exposes `=` and routes to the fleet front door; source/code-tested 2026-09-16, live mutation remains unverified | `tui.py`, `fleet add` |
| `CURRENT` | UI key for `fleet remove` outside simulator menus | Vehicle actions expose `-`; source/code-tested 2026-09-16, live mutation remains unverified | `tui.py`, `fleet remove` |
| `CURRENT` | UI key for `scene`/`scenario` outside menus | Stack actions expose `N`/`T`; source/code-tested 2026-09-16, live selection remains unverified | `tui.py`, `scene`, `scenario` |
| `CURRENT` | UI key for `fiducial` | Stack action exposes `F`; source/code-tested 2026-09-16, live reposition and cleanup remain unverified | `tui.py`, `fiducial` |
| `CURRENT` | UI router-log visibility | Vehicle action exposes router log follow; source/code-tested 2026-09-16, live interactive follow remains unverified | `tui.py`, `logs` |
| `CURRENT` | UI `build`/`prepare` action | Front door supports build/prepare, but `ACTIONS` has no direct UI action; it should be usable from the UI | decide whether omission is intentional |
| `CURRENT` | UI `setup`/`bootstrap` action | Front door supports setup, but UI has no setup action; it should be usable from the UI | decide whether omission is intentional |
| `MISSING` | UI `fleet` edit prompt beyond add/remove | CLI supports more fleet controls than the action table; expose required controls from the UI | action table and fleet CLI |

For every row, keep the intent status above and append `Tested: YES|NO|BLOCKED`
with the checkpoint date and commit. A UI action can be `CURRENT` while its
underlying system is not yet tested; it becomes `Tested: YES` only after the
command, resulting state, and cleanup have all been observed.
| `CURRENT` | Variable fiducial lat/lon in scenegen | Source/code-tested 2026-09-16: the editor persists the surveyed fiducial coordinate into `fiducial_lat`/`fiducial_lon` while retaining independent `east_m`/`north_m` placement for simulated frame error; interactive editor round-trip remains unverified | `modules/scenegen/editor.html`, scene/scenario writers |
| `MISSING` | Offboard compute companion path | a setup that's easily chosen to essentially do the onboard stuff for a real drone on the ground (will need different mode of the rtsp server to encode livestream with network time for timesyncing on the ground with mavlink data) trades a bit of latency for unrestricted compute |
| `MISSING` | Recording front door | Should have a way to start, monitor, and inspect recordings on the drone, namely mcaps of configurable subsets of data synchronized with videos recording onboard or offboard (synced start times in name of file are good enough, but the more synced the better) |
| `CURRENT` | px4sim UI text wrapping | Source/code-tested 2026-09-16: `wrap_output` splits long tokens to the available pane width and `draw_output` applies it before rendering; narrow-terminal resize behavior remains unverified | `scripts/tui.py`, `verify/test_tui_helpers.py` |
