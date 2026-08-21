# px4-sim-stack

A container stack that flies the Chimera flight code against PX4 SITL and
Gazebo Harmonic. The simulator presents MAVLink, RTSP video and a rangefinder.
The flight code from 5g_drone and MAVInsight runs unchanged, on the same ROS
domains, ports and frame names it uses on the aircraft.

`docs/uas-contract.md` states what a simulated vehicle must present. The
simulator satisfies that contract. The flight code does not accommodate the
simulator.

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
| `sim` | Gazebo Harmonic, one PX4 SITL instance for each vehicle, the camera encoders | always |
| `uas11` to `uas19` | mavlink-router, one container for each vehicle | from `UAS_FLEET` |
| `onboard11` to `onboard14` | The companion computer: MAVROS, `ds_node`, MAVInsight | from `UAS_FLEET` |
| `offboard` | The ground station for the whole fleet, plus its `ground-router` | `offboard` |
| `video-router` | MediaMTX. Every camera enters here and leaves as RTSP | always |
| `qgc` | QGroundControl v5.0.8, released build | always |
| `scenegen` | Builds a scene from map data. Runs on demand and exits | `scenegen` |
| `xrce-agent` | The uXRCE-DDS bridge, for a stack that needs `px4_msgs` | `xrce` |

A vehicle is one machine: the companion container shares its router's network
namespace, so the router reaches MAVROS on loopback, as it does on the Orin.
The ground station is one machine too, so `offboard` holds one MAVROS for each
vehicle and `ground-router` shares its namespace.

## Start here

```bash
./px4sim doctor    # check the driver, docker, GPU runtime and X11
./px4sim setup     # clone PX4 into ./src and build the scenes
./px4sim build     # build the images, about 20 minutes
./px4sim start     # start the stack
```

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

`./px4sim` with no arguments prints every command. It is the front door: it
reads `.env` and resolves the world origin from the scene and the scenario. The
`Makefile` keeps the same target names and forwards each one to it.

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
| Video | `rtsp://localhost:8554/rgb11`, `/pilot11`, `/thermal11` |
| Low-rate video | the same names with `l` appended: `rgbl11` |
| Foxglove, ground station | `ws://localhost:8765` |
| Foxglove, one vehicle | `ws://localhost:8771` for uas11 |

`./px4sim status` prints this list for the fleet that is running.

## Daily commands

```bash
./px4sim ui                   # the console: what is running, and a key for every command
./px4sim start                # start everything
./px4sim stop                 # stop everything
./px4sim status               # what is running, and the addresses
./px4sim logs onboard11       # follow one service
./px4sim console              # the pxh> prompt. Detach with Ctrl-P Ctrl-Q
./px4sim fleet add            # fly one more vehicle. `fleet remove 13` retires one
./px4sim scene baylands       # change the world and restart the sim
./px4sim scenario             # place the targets again, no restart
./px4sim streams              # which video streams are live
./px4sim view rgb11           # play one
./px4sim topics 11            # the ROS topics of uas11
./px4sim layout               # where the Foxglove layout is
./px4sim shell onboard11      # a shell in any container
./px4sim check                # validate the compose file and lint the docs
./px4sim clean                # remove containers, networks and volumes
```

Fly a vehicle and see what its nodes make of it:

```bash
./px4sim uas 11 takeoff 40    # status, arm, takeoff, goto, land
./px4sim uas 11 gimbal -30    # degrees below the horizon
./px4sim uas 11 detect on     # continuous detection, off until asked
./px4sim uas 11 detections    # what it found, and where that landed
./px4sim uas 11 capture mosaic   # mosaic, fiducial, vlm, snapshot
./px4sim probe 11             # what its ROS graph carries right now
./px4sim probe ground         # the same, on the ground station
./px4sim foxglove ground      # what a panel is offered, over Foxglove's own protocol
./px4sim uas 11 heading       # which way the vehicle, its camera and its footprint point
./px4sim uas 11 scene         # the ground, roofs and targets the 3D panel is given
./px4sim zoom 11 wide         # a v3 lens: narrow, mid or wide
./px4sim snap rgb11           # one frame of a stream, to look at
```

## The console

`./px4sim ui` draws the stack in the terminal and runs these commands for you.
It shows the containers, every vehicle, the video paths and the Foxglove ports,
and it reads them again about every two seconds.

Move between the panes with tab, and pick a row with the arrow keys. Press
enter for what can be done to that row, or press the key beside an action.
Press `:` to type any px4sim command, and `?` to see every action with the
command it runs. Press esc to stop a command, and `q` to leave.

Press `=` to fly one more vehicle, and `-` to retire the selected one. Both
write `UAS_FLEET` in `.env` and bring the stack to the new fleet.

The console starts nothing of its own. Every action runs `./px4sim ...`, so
what it does is what the prompt does. It is a way to reach this script, not a
second one.

Each reading comes from the thing itself, never from a log message:

| Reading | Where it comes from |
|---|---|
| A container | The container engine's own state |
| A video path | The video router API, and the bytes that path carried since the report before |
| A vehicle | MAVLink on `tcp://localhost:5761` upwards: mode, arming, battery, GPS, height and gimbal |
| A Foxglove bridge | A connection to the port, opened and closed |
| The GPU | `nvidia-smi`: what the cameras, the encoders and the detector are using |

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
writes the file and says so.

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

The first four stages need nothing running. `vehicle`, `ground`, `fleet`,
`captures` and `foxglove` fly the stack and say so if it is not up. What each one measured
last is in [docs/development.md](docs/development.md).

Start a subset by naming the profiles:

```bash
./px4sim start ""             # the vehicles and QGC, with no ground station
./px4sim start offboard       # the default, from COMPOSE_PROFILES in .env
```

The routers and the companions come from `UAS_FLEET`, so they need no profile
name here.

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
| ROS 2 | Humble | What the flight code runs on the aircraft. `cdcl_umd_msgs` does not decode across distributions, so every container that carries it is Humble. |
| Gazebo | Harmonic | The Gazebo release that PX4 v1.17 installs. |
| QGroundControl | v5.0.8 | The mature v5.0 line. Set `QGC_REF` in `.env` to move. |
| DeepStream | 7.1 | The last release on Ubuntu 22.04, which is what Humble needs. It needs driver 535.183 or later. |

## Requirements

- Linux with an X11 session. Wayland works through XWayland, and is less tested.
- An NVIDIA GPU, driver 535.183 or later, and `nvidia-container-toolkit`.
- Docker 24 or later with Compose v2.
- About 120 GB of disk and 32 GB of RAM. Every vehicle costs GPU: a fleet of
  four renders ten cameras, encodes twenty streams and runs four detectors.

`make preflight` checks all of it and says what is missing.

## Documentation

| File | Contents |
|---|---|
| [docs/uas-contract.md](docs/uas-contract.md) | What a simulated vehicle must present. The specification |
| [docs/architecture.md](docs/architecture.md) | Why the containers split where they do |
| [docs/interfaces.md](docs/interfaces.md) | What the simulator produces, and the frame conventions |
| [docs/development.md](docs/development.md) | How to change PX4, the airframe, the scenes and the targets |
| [docs/px4-simulated-gimbal.md](docs/px4-simulated-gimbal.md) | How the PX4 simulated gimbal behaves, and how to command it |
| [docs/localization-error.md](docs/localization-error.md) | Where a target position's error comes from, and which parts are floors |
| [docs/troubleshooting.md](docs/troubleshooting.md) | What breaks, and what to do |
