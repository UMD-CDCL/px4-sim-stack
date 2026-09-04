# px4-sim-stack

One container stack for three machines. It flies the Chimera flight code
against PX4 SITL and Gazebo Harmonic, it runs the real vehicle's companion
computer on the aircraft, and it runs the fielded ground station on the base
station laptop. The flight code from 5g_drone and MAVInsight is the same code
in all three, on the same ROS domains, ports and frame names.

`.env` says which of the three worlds this machine is. `COMPOSE_PROFILES` says
what runs and `UAS_BASE` says which world the numbers belong to.
[docs/front-doors.md](docs/front-doors.md) works one example for each world.

`docs/uas-contract.md` states what a simulated vehicle must present. The
simulator satisfies that contract. The flight code does not accommodate the
simulator. The diagram below is the simulator, which is the world with the
most parts.

```
                     ┌──────────────────┐
                     │       sim        │  Gazebo Harmonic, one world
                     │  PX4 SITL x N    │  cameras, gimbal, rangefinders
                     └──┬────────────┬──┘
              MAVLink   │            │   H.265
                        ▼            ▼
              ┌──────────────┐  ┌──────────────┐
              │ uas11..uas19 │  │ video-router │  RTSP, one path per stream
              │ mavlink-     │  └───┬──────┬───┘
              │ router       │      │      │
              └──┬────────┬──┘      │      │
                 │        │         │      │
                 ▼        └─────────┼──┐   │
          ┌────────────┐            │  │   │
          │ onboard11  │◄───────────┘  │   │
          │ ..onboard14│  ds_node, MAVROS, MAVInsight
          └─────┬──────┘               │   │
       domain   │ 99                   │   │
                ▼                      ▼   ▼
          ┌───────────────┐        ┌───────┐
          │   offboard    │        │  qgc  │
          │ ground-router │        └───────┘
          └───────────────┘
            the ground station, every vehicle at once
```

## What runs where

| Service | Contents | Profile |
|---|---|---|
| `sim` | Gazebo Harmonic, one PX4 SITL instance for each vehicle, the camera encoders | `sim` |
| `uas11` to `uas19` | mavlink-router, one container for each vehicle | from `UAS_FLEET` |
| `onboard11` to `onboard14` | The companion computer: MAVROS, `ds_node`, MAVInsight | from `UAS_FLEET` |
| `offboard` | The simulated ground station for the whole fleet, plus its `ground-router` | `offboard` |
| `video-router` | MediaMTX. Every camera enters here and leaves as RTSP | `sim`, `offboard` |
| `qgc` | QGroundControl v5.0.8, released build | `sim` |
| `ground` | The real ground station on t500: the offboard image on the host network, beside the native mavlink-router and lcam | `ground` |
| `onboard` | The aircraft: the companion image on the host network, beside the native mavlink-router and rcam | `aircraft` |
| `scenegen` | Builds a scene from map data. Runs on demand and exits | `scenegen` |
| `xrce-agent` | The uXRCE-DDS bridge, for a stack that needs `px4_msgs` | `xrce` |

A vehicle is one machine: the companion container shares its router's network
namespace, so the router reaches MAVROS on loopback, as it does on the Orin.
The ground station is one machine too, so `offboard` holds one MAVROS for each
vehicle and `ground-router` shares its namespace. On the real machines the
container shares the host's namespace instead, and the native router pushes to
`127.0.0.1:14402` as it does today.

`UAS_BASE` in `.env` says which world the numbers belong to: 10 numbers a
simulated fleet from uas11, 0 the real one from uas1. `COMPOSE_PROFILES` says
what runs. `./px4sim` refuses a `.env` where the two disagree.

## Start here

`.env` says which machine this is. `.env.example` holds the three pairs of
`COMPOSE_PROFILES` and `UAS_BASE`, one for each machine.
[docs/front-doors.md](docs/front-doors.md) gives the whole command line for
each world, and for every other door into this system.

The simulator, on one laptop, with `COMPOSE_PROFILES=sim,offboard` and
`UAS_BASE=10`:

```bash
./px4sim doctor    # check the driver, docker, GPU runtime and X11
./px4sim setup     # clone PX4 into ./src and build the scenes
./px4sim build     # build the images, about 20 minutes
./px4sim start     # start the stack
```

The real ground station on t500, with `COMPOSE_PROFILES=ground`, `UAS_BASE=0`
and `GROUND_DOMAIN=60`:

```bash
./px4sim doctor    # also lcam, mavlink-router, and the host ports 14402 and 8765
./px4sim build     # ros-base and the ground image
./px4sim start     # one container, ground, on the host network
```

The aircraft, with `COMPOSE_PROFILES=aircraft` and `UAS_BASE=0`, after
`chimera-deploy/remote/deploy_onboard.sh` wrote its `.env`:

```bash
./px4sim doctor    # also rcam, its sockets, the clock, the power mode and the lens
./px4sim build     # the arm64 images, on the Orin itself
./px4sim start     # one container, onboard, on the host network
```

The first build on the Orin compiles the whole ROS workspace at 15 W and takes
tens of minutes. A rebuild after a change in the flight code took 3 minutes on
the bench. Until `user` is in group `docker` there, every call needs sudo:
`sudo -E env HOME=/home/user UAS_NUM=1 ./px4sim start`. The boot unit does not
need that group.

Each machine builds its own architecture from its own checkout. No machine
copies an image from another.

To stop it:

```bash
./px4sim stop
```

The first `./px4sim start` builds PX4 inside the sim container. That takes 10 to
20 minutes and happens once, because the build output lands in
`./src/PX4-Autopilot` on the host. Watch it with `./px4sim logs sim`.

The onboard and offboard images build 5g_drone, cdcl_umd_msgs and MAVInsight
with colcon. `ROS2_WS_DIR` in `.env` says where those sources are checked out,
and a change there needs `./px4sim build onboard offboard`.

`./px4sim` with no arguments prints every command, and names which of the three
worlds this machine is. It is the front door: it reads `.env`, resolves the
DeepStream release for this GPU, and resolves the world origin from the scene
and the scenario. The `Makefile` keeps the same target names and forwards each
one to it.

A joystick plugged into the host works in QGroundControl. The container mounts
`/dev/input` and joins the host `input` group, whose id `./px4sim doctor` writes
into `.env` as `INPUT_GID`. Calibrate it once under Vehicle Setup, Joystick. If
the Joystick page does not appear, restart the `qgc` service with the controller
already plugged in.

## The addresses

`N` is the UAS number, 11 to 19 for a simulated vehicle.

| What | Address |
|---|---|
| MAVLink, to the ground station | `udp://<gcs>:14550 + N`, so 14561 for uas11 |
| MAVLink, to the companion | `udp://127.0.0.1:14402`, inside the vehicle |
| MAVLink, for scripts | `tcp://localhost:5761` upwards |
| Video | `rtsp://localhost:8554/rgb11`, `/pilot11`, `/thermal11`. `RTSP_HOST_PORT` moves the host side |
| Low-rate video | the same names with `l` appended: `rgbl11` |
| Foxglove, ground station | `ws://localhost:8765` |
| Foxglove, one vehicle | `ws://localhost:8771` for uas11 |

The real fleet, `N` 1 to 9, with `UAS_BASE=0`:

| What | Address |
|---|---|
| MAVLink, to the ground station | `udp://10.200.142.60:14550 + N`, so 14551 for uas1, into the native router |
| MAVLink, to MAVROS | `udp://127.0.0.1:14402`, from the native router, on the aircraft and on the ground alike |
| MAVLink, for scripts | `tcp://127.0.0.1:5760`, this machine's native router |
| Low-rate video, on the ground | `rtsp://127.0.0.1:8554/rgbl1` from lcam, `pilotl1`, `thermall1` |
| Video, on the aircraft | `rtsp://127.0.0.1:8554/rgb` from rcam, and `rgbl`, `pilot`, `pilotl`, `thermal`, `thermall` |
| Foxglove | `ws://localhost:8765` on each machine. From the ground, `ws://10.200.142.61:8765` is uas1's |

`./px4sim status` prints this list for the fleet that is running.

## Daily commands

`./px4sim help` is the one truthful list, and it names the world this machine
is. These work in all three worlds:

```bash
./px4sim ui                   # the console: what is running, and a key for every command
./px4sim start                # start what .env selects
./px4sim stop                 # stop everything
./px4sim status               # what is running, and the addresses
./px4sim logs onboard11       # follow one service. `logs --since 5m ground` reads back
./px4sim streams              # which video streams are live
./px4sim view 1               # play a vehicle's gimbal camera
./px4sim topics 1             # a vehicle's ROS topics
./px4sim layout               # render the Foxglove layout for this fleet
./px4sim shell ground         # a shell in any container
./px4sim check                # validate the compose file and lint the docs
./px4sim clean                # remove containers, networks and volumes
```

Ask one vehicle what its nodes make of the world. On a real fleet the ground
station answers for the vehicle, and each command reaches the machine that
holds the answer:

```bash
./px4sim uas 1 status         # mode, arming, position, battery, GPS, gimbal
./px4sim uas 1 gimbal -30     # degrees below the horizon
./px4sim uas 1 detect on      # continuous detection, off until asked
./px4sim uas 1 detections     # what it found, and where that landed
./px4sim uas 1 heading        # which way the vehicle, its camera and its footprint point
./px4sim capture 1 mosaic     # mosaic, vlm or fiducial
./px4sim zoom 1 wide          # a v3 lens: narrow, mid or wide
./px4sim probe 1              # what its ROS graph carries right now
./px4sim probe ground         # the same, on the ground station
./px4sim foxglove 1           # what a panel is offered, over Foxglove's own protocol
```

The simulator adds the commands that fly it and maintain it:

```bash
./px4sim uas 11 takeoff 40    # arm, takeoff, goto, land
./px4sim fly 11 20            # respawn, then climb
./px4sim console              # the pxh> prompt. Detach with Ctrl-P Ctrl-Q
./px4sim scene uroc           # change the world and restart the simulator
./px4sim scenario <name>      # change the targets and reload what scores them
./px4sim fleet add            # fly one more vehicle. `fleet remove 13` retires one
./px4sim snap rgb11           # one frame of a stream, to look at
```

With `UAS_BASE=0` a vehicle number is a real aircraft, so `./px4sim` refuses
every command that flies the simulator: `core`, `fly`, `place`, `scene`,
`scenario`, `fiducial`, `reset`, `px4`, `console`, `snap`, `genscene`, and
`uas N arm`, `takeoff`, `land` and `goto`. Nothing is sent. It also refuses
the commands that maintain the simulator, and each of those says what it did
not change: `setup`, `clean-src`, `nuke`, `fleet add` and `fleet remove`.
`./px4sim help` holds that list under "The simulator alone", and it names the
world this machine is. `verify` is not refused: it says for itself which of
its stages a real machine can answer. `router` is refused too, because the real
fleet's router is native: `systemctl status mavlink-router`.

## The console

`./px4sim ui` draws the stack in the terminal and runs these commands for you.
It shows the containers, every vehicle, the video paths and the Foxglove ports,
and it reads them again about every two seconds.

Move between the panes with tab, and pick a row with the arrow keys. Press
enter for what can be done to that row, or press the key beside an action.
Press `:` to type any px4sim command, and `?` to see every action with the
command it runs. Press esc to stop a command, and `q` to leave.

In the simulator, press `=` to fly one more vehicle and `-` to retire the
selected one. Both write `UAS_FLEET` in `.env` and bring the stack to the new
fleet.

The console offers the keys this world accepts, and no others. On a ground
station and on an aircraft the twenty actions that fly or maintain the
simulator are not in the menus, not on a key and not in `?`. A ground station
and an aircraft also get a NATIVE row, which is `systemctl is-active` for the
services this machine boots beside the containers: `lcam` and
`mavlink-router` on the ground, `rcam`, `mavlink-router` and `onboard` on the
aircraft. Every bridge of the real fleet holds port 8765, so the FOXGLOVE row
names each one by its machine.

The console starts nothing of its own. Every action runs `./px4sim ...`, so
what it does is what the prompt does. It is a way to reach this script, not a
second one.

Each reading comes from the thing itself, never from a log message:

| Reading | Where it comes from |
|---|---|
| A container | The container engine's own state |
| A video path | The video router API in the simulator, with the bytes that path carried since the report before. One probe by name on a real machine, where lcam and rcam serve no API |
| A vehicle | MAVLink on `tcp://localhost:5761` upwards, or on the native router's 5760 for the real fleet: mode, arming, battery, GPS, height and gimbal |
| A Foxglove bridge | A connection to the port, opened and closed |
| A native service | `systemctl is-active`, on a ground station and on an aircraft |
| The GPU | `nvidia-smi`: what the cameras, the encoders and the detector are using. A Jetson has no `nvidia-smi`, and the field is empty |

`./px4sim state` prints the same picture as JSON, and `./px4sim state --watch`
keeps it coming, one object for each line. The console reads that stream, and
so can a script.

### The fleet

`UAS_FLEET` is a list, and the place in the list is the vehicle number. The
first entry is uas11, the second uas12, and so on.

```bash
./px4sim fleet                # what flies now
./px4sim fleet add            # one more of the last airframe
./px4sim fleet add chimera_v2 # one more, of this airframe
./px4sim fleet remove         # the last vehicle
./px4sim fleet remove 12      # this vehicle
```

Each of these writes `.env`, removes the containers the fleet no longer holds,
and reloads the world, so every vehicle respawns. With nothing running it
writes the file and says so. `fleet` on its own reads the fleet in every world.
`fleet add` and `fleet remove` belong to the simulator, and a real machine
refuses both and says to edit `UAS_FLEET` by hand.

A vehicle taken from anywhere but the end moves every vehicle after it down
one, which gives each of them other frames, ports, ROS domains and stream
names. The command says so and stops. Add `--renumber` to say that this is
what you want.

The simulator flies nine vehicles at the most, uas11 to uas19. `compose.yaml`
holds a companion for uas11 to uas14 only, so a fifth vehicle flies with a
router and no ROS stack until you add an `onboard15` service.

## Verification

```bash
./px4sim verify               # every stage
./px4sim verify help          # what the stages are
./px4sim verify airframes     # one of them
```

`airframes`, `contract`, `units` and `localize` need nothing running.
`vehicle`, `flight`, `ground`, `fleet`, `captures` and `foxglove` read the
running stack and say so if it is not up. `fleet` and `captures` are skipped
without `VERIFY_FULL=1`. What each one measured last is in
[docs/development.md](docs/development.md).

A real machine runs `ground` and `foxglove` and says so. The other stages
expand the simulator's airframes or fly a vehicle, so `verify` refuses them by
name. The last simulator run passed 71 checks, and the last ground run passed
9.

Start a subset by naming the profiles:

```bash
./px4sim start ""             # the vehicles and QGC, with no ground station
./px4sim start offboard       # the default. ./px4sim adds sim itself in the simulator
```

The routers and the companions come from `UAS_FLEET`, so they need no profile
name here.

## State of the real-vehicle port (2026-09-03)

The aircraft world and the ground world ran on the real hardware for the first
time on 2026-09-03. Nothing armed and nothing flew. The evidence is component
tests on uas1 and on t500.

**Verified on the aircraft, uas1.** `onboard.service` starts the container at
boot, after docker, rcam, the native router and a clock step, and it reaches
`active (exited)` in 39 seconds. MAVROS connects to PX4 v1.18 and reads
`capabilities=321791`. The preview publishes at 27.4 Hz from the rcam NVMM
socket, and `ds_node` reports `pipelines PLAYING` 57 seconds after the
container starts. The detector runs on the GPU and publishes at 1.03 to 1.06
Hz, at 99% GR3D and 15.9 W. The SCF4 lens reaches a framing in about one
second. The gimbal publishes its state and its attitude at 4 Hz. The engines
deserialize from `perception_models/orin`, with no rebuild. 46 ROS nodes come
up and none restarts. The front door refuses all seven flight commands.

**Verified on the ground station, t500.** The `ground` profile runs one
container on the host network, beside the native lcam and mavlink-router. MAVROS
on domain 60 reads uas1 as connected. The air bridges carry 16 topics on domain
99 and 2 on domain 69. `./px4sim streams` lists all three lcam mounts.
`./px4sim layout` renders the Foxglove layout for uas1, and 0 of its 24 topics
are absent against the live graph. `./px4sim verify` passes 9 checks over the
`ground` and `foxglove` stages. The cross-wire run was 38 pass, 0 fail, 1
blocked. The simulator still passes `./px4sim verify`, 71 of 71.

**What a bench cannot prove.** A level camera means no ray meets the ground.
`tf_loc` needs a box at least 20 degrees below the horizon, and the bench
measured about 9. So `target_locations`, the mosaic map,
the fiducial survey and the full detection round trip stay unproved. The gimbal
rangefinder id 1 needs a target inside 50 metres. A reboot was out of scope, so
the boot path was proved with `systemctl start` and `systemctl restart`, not
with a cold start.

**Open items.**

| Item | What it needs |
|---|---|
| Docker on uas1 | `sudo usermod -aG docker user`, then a new login. Interactive `./px4sim` needs it. The boot unit does not |
| The drone password | Rotate it. It is out of the chimera-deploy tree and still in that repository's history on GitHub |
| The branches | `feature/real-drone-port` in px4-sim-stack, 5g_drone, MAVInsight and chimera-deploy is unpushed. The drones read it from the mirrors in `/srv/git` |
| The thermal mount | `thermall1` and the vehicle's own `thermal` answered 503 once. rcam was not restarted. A restart or a camera re-plug is the next step |
| `ROS_DOMAIN_ID` in a shell | The entry points export it into PID 1 alone, so a hand `docker exec` shell joins domain 0. Every front-door command passes the domain itself |
| Dead bridge files | 5g_drone still carries eight per-vehicle domain bridge files that no launch file reads. Delete them or keep them |

## Change something

| To change | Do this | Read |
|---|---|---|
| The world | `SCENE=` in `.env` | [docs/development.md](docs/development.md) |
| The targets | Edit a file in `modules/sim/scenes/scenarios/` | [docs/development.md](docs/development.md) |
| The airframe or its sensors | Edit `modules/sim/scenes/models/chimera_v2` or `chimera_v3` | [docs/development.md](docs/development.md) |
| The fleet | `./px4sim fleet add [model]` and `./px4sim fleet remove [N]`, or `UAS_FLEET=` in `.env` | [docs/uas-contract.md](docs/uas-contract.md) |
| Which cameras each vehicle serves | `UAS_STREAMS=` in `.env` | [docs/troubleshooting.md](docs/troubleshooting.md) |
| The zoom preset a v3 lens flies at | `UAS_ZOOM=` in `.env` | [docs/development.md](docs/development.md) |
| PX4 itself | Edit `src/PX4-Autopilot`, then restart `sim` | [docs/development.md](docs/development.md) |
| The flight code | Edit the tree at `ROS2_WS_DIR`, then rebuild | [docs/development.md](docs/development.md) |
| QGroundControl itself | `./scripts/bootstrap.sh qgc`, then the `qgc-dev` profile | [docs/development.md](docs/development.md) |
| A scene from map data | `./px4sim genscene --help` | [modules/scenegen/README.md](modules/scenegen/README.md) |

## Versions, and why

| Component | Version | Reason |
|---|---|---|
| PX4 | v1.17.0 | Current stable, May 2026. |
| ROS 2 | Humble | What the aircraft runs. `cdcl_umd_msgs` does not decode across distributions, so the whole fleet is Humble and nothing here may choose otherwise. |
| Gazebo | Harmonic | The Gazebo release that PX4 v1.17 installs. |
| QGroundControl | v5.0.8 | The mature v5.0 line. Set `QGC_REF` in `.env` to move. |
| DeepStream | 7.1 | The last release on Ubuntu 22.04, which is what Humble needs, and what the Orin runs. On a GPU newer than its TensorRT, `scripts/ds-select.sh` installs a TensorRT that fits and changes nothing else. On a Jetson it installs the host's own TensorRT build. `./px4sim doctor` explains. |

## Requirements

- Linux with an X11 session, where a window is wanted. Wayland works through
  XWayland, and is less tested. The aircraft runs with no display.
- An NVIDIA GPU, driver 535.183 or later, and `nvidia-container-toolkit`. A
  Blackwell card needs 570.133 or later, for the TensorRT that can build engines
  for it.
- Docker 24 or later with Compose v2.
- About 80 GB of free disk for a full build, and 16 GB of RAM. `./px4sim
  doctor` warns below either. Every vehicle costs GPU: a fleet of four renders
  ten cameras, encodes twenty streams and runs four detectors.
- `chimera-deploy` beside this checkout, with its `mavros` and `angles`
  submodules checked out. `ros-base` builds the PX4 v1.18 MAVROS patch from
  them. `MAVROS_PATCH=0` in `.env` removes that need.
- On the aircraft, `UAS_NUM` in `/etc/environment`, which
  `chimera-deploy/deploy.sh` writes. `user` in group `docker` for interactive
  use, which `chimera-deploy/remote/deploy_onboard.sh` sets up. The boot unit
  carries `SupplementaryGroups=docker` and works without it.
- On a real machine, `gst-discoverer-1.0` from `gstreamer1.0-plugins-base-apps`,
  which is how `./px4sim streams` probes a mount that serves no API.

`make preflight` checks all of it and says what is missing.

## Documentation

| File | Contents |
|---|---|
| [docs/front-doors.md](docs/front-doors.md) | Every door into this system, with the exact commands and one worked example for each world |
| [docs/uas-contract.md](docs/uas-contract.md) | What a simulated vehicle must present. The specification |
| [docs/architecture.md](docs/architecture.md) | Why the containers split where they do |
| [docs/interfaces.md](docs/interfaces.md) | What the simulator produces, and the frame conventions |
| [docs/development.md](docs/development.md) | How to change PX4, the airframe, the scenes and the targets |
| [docs/px4-simulated-gimbal.md](docs/px4-simulated-gimbal.md) | How the PX4 simulated gimbal behaves, and how to command it |
| [docs/localization-error.md](docs/localization-error.md) | Where a target position's error comes from, and which parts are floors |
| [docs/localization-report.md](docs/localization-report.md) | What the 2026-08-21 accuracy pass changed, and what it measured |
| [docs/altitude-datums.md](docs/altitude-datums.md) | Which altitude the ground is measured from, and the bias that came of it |
| [docs/troubleshooting.md](docs/troubleshooting.md) | What breaks, and what to do |
