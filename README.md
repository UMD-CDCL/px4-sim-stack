# px4-sim-stack

A containerized PX4 and ROS 2 environment for drone autonomy work. It simulates
a gimbal camera, a nadir camera and a rangefinder, streams them as RTSP, runs
DeepStream inference on the video, and connects QGroundControl to all of it.

The stack splits along interface lines, not along tool lines. Each module talks
to the others through one address, so you can replace a module without touching
its neighbors.

```
                    ┌──────────────┐
                    │     sim      │  PX4 SITL + Gazebo Harmonic
                    │              │  gimbal cam, nadir cam, rangefinder
                    └───┬──────┬───┘
              MAVLink   │      │   H.264
                        ▼      ▼
              ┌─────────────┐ ┌──────────────┐
              │ mavlink-hub │ │ video-router │
              └──┬───┬───┬──┘ └───┬──────┬───┘
                 │   │   │        │      │
        ┌────────┘   │   └────────┼──┐   │
        ▼            ▼            ▼  │   ▼
   ┌────────┐   ┌─────────┐  ┌────────────┐
   │  qgc   │   │   ros   │◄─┤ perception │  DeepStream 8.0
   └────────┘   └─────────┘  └─────┬──────┘
                     ▲             │
                     └─────────────┘
                       message-bus (MQTT detections)
```

## What runs where

| Module | Contents | Replaceable with |
|---|---|---|
| `sim` | PX4 v1.17, Gazebo Harmonic, the camera encoders | A real airframe |
| `mavlink-hub` | mavlink-router | Any MAVLink router, or a telemetry radio |
| `video-router` | MediaMTX | Any RTSP server, or the camera's own server |
| `message-bus` | Mosquitto | Any MQTT broker |
| `qgc` | QGroundControl v5.0.8, released build | A source build, see the `qgc-dev` profile |
| `ros` | ROS 2 Jazzy, MAVROS, the stack overlay | Any stack under `modules/ros/stacks/` |
| `perception` | DeepStream 8.0, TrafficCamNet | Any detector, see `modules/perception/models/` |

## Start here

```bash
./px4sim doctor    # check the driver, docker, GPU runtime and X11
./px4sim setup     # clone PX4 into ./src, create the ROS workspace
./px4sim build     # build the images, about 20 minutes
./px4sim start     # start the stack
```

To stop it:

```bash
./px4sim stop
```

The first `./px4sim start` builds PX4 inside the sim container. That takes 10 to
20 minutes and only happens once, because the build output lands in
`./src/PX4-Autopilot` on the host.

Watch it:

```bash
./px4sim logs sim
```

`./px4sim` with no arguments prints every command. The `Makefile` still does the
same jobs if you prefer it.

When Gazebo shows the drone and QGroundControl shows a connected vehicle, the
stack is up. Take off from QGroundControl and the video appears in its window.

## The three addresses

Everything downstream of the vehicle uses these and nothing else.

| What | Address |
|---|---|
| MAVLink, for ROS | `udp://:14555@mavlink-hub:14551` |
| MAVLink, for scripts | `tcp://localhost:5760` |
| Video | `rtsp://localhost:8554/gimbal`, `/nadir`, `/gimbal_annotated` |
| Detections | `mqtt://localhost:1883`, topic `perception/detections` |
| Foxglove | `ws://localhost:8765`, layout in `modules/ros/stacks/baseline/foxglove/` |

`make endpoints` prints this list.

## Daily commands

```bash
./px4sim start                # start everything
./px4sim stop                 # stop everything
./px4sim status               # what is running, and the addresses
./px4sim logs perception      # follow one module
./px4sim console              # the pxh> prompt. Detach with Ctrl-P Ctrl-Q
./px4sim shell ros            # a ROS shell with the overlay sourced
./px4sim scene baylands       # change the world and restart the sim
./px4sim scenario             # place the targets again, no restart
./px4sim streams              # which video streams are live
./px4sim view nadir           # play one
./px4sim detections           # follow the MQTT detection topic
./px4sim sim                  # a shell inside a container: sim, ros, qgc, perception, hub, bus
./px4sim check                # validate the compose file and lint the docs
./px4sim clean                # remove containers, networks and volumes
```

Start a subset by naming the profiles:

```bash
./px4sim start ""                 # sim, transport and QGroundControl only
./px4sim start ros                # add the autonomy stack
./px4sim start ros,perception     # the default, from COMPOSE_PROFILES in .env
```

## Change something

| To change | Do this | Read |
|---|---|---|
| The world | `SCENE=` in `.env` | [docs/scenarios.md](docs/scenarios.md) |
| The targets | Edit a file in `modules/sim/scenes/scenarios/` | [docs/scenarios.md](docs/scenarios.md) |
| The airframe or its sensors | Edit `modules/sim/scenes/models/x500_recon/` | [docs/scenarios.md](docs/scenarios.md) |
| PX4 itself | Edit `src/PX4-Autopilot`, then restart `sim` | [docs/development.md](docs/development.md) |
| QGroundControl itself | `./scripts/bootstrap.sh qgc`, then the `qgc-dev` profile | [docs/development.md](docs/development.md) |
| The ROS stack | `ROS_STACK=` in `.env` | [modules/ros/stacks/README.md](modules/ros/stacks/README.md) |
| The detector | `modules/perception/configs/` | [modules/perception/models/README.md](modules/perception/models/README.md) |

## Versions, and why

| Component | Version | Reason |
|---|---|---|
| PX4 | v1.17.0 | Current stable, May 2026. |
| ROS 2 | Jazzy | The long term release for Ubuntu 24.04, supported to May 2029. Kilted and Lyrical are newer, and Jazzy has binary packages for everything this stack uses. |
| Gazebo | Harmonic | The Gazebo release that PX4 v1.17 installs and that pairs with Jazzy. |
| QGroundControl | v5.0.8 | The mature v5.0 line. v5.1.0 arrived on 30 July 2026, twelve days before this stack was written. Set `QGC_REF=v5.1.0` in `.env` to move. |
| DeepStream | 8.0 or 9.0 | Chosen from the driver, because the driver sets the ceiling: 9.0 needs 590.48 and 8.0 needs 570.133. `DS_VERSION=auto` reads it and takes the newest that runs. Pin it to reproduce a result elsewhere. See [docs/troubleshooting.md](docs/troubleshooting.md). |

Every version is one line in `.env`.

## What was tested

The stack was built and run on this machine on 11 August 2026. These were
exercised end to end:

- PX4 v1.17.0 builds in the container and reaches "Ready for takeoff".
- Both cameras encode with NVENC and publish. `rtsp://localhost:8554/gimbal`
  and `/nadir` both decode.
- MAVLink reaches a client on UDP 14552 and on TCP 5760. 31 message types,
  including `DISTANCE_SENSOR` from the simulated rangefinder.
- Restarting `mavlink-hub` alone recovers telemetry with no simulator restart.
- MAVROS reports `connected: true`. The rangefinder arrives on
  `/mavros/rangefinder_pub` at 9.8 Hz, reading 0.15 m on the ground.
- Both cameras arrive in ROS at `/camera/gimbal/image_raw` and
  `/camera/nadir/image_raw`, about 24 Hz.
- The scenario places all six targets from Gazebo Fuel.
- The vehicle arms, takes off, repositions over a target and lands, all over
  MAVLink.
- DeepStream runs at 28 fps and detects a person. 52 of 150 messages carried
  objects while the drone was overhead. The annotated stream decodes at
  `rtsp://localhost:8554/gimbal_annotated`.
- Those detections reach ROS as `vision_msgs/Detection2DArray` with
  `class_id: person` and a plausible box.

Not tested: the QGroundControl window itself, beyond confirming that the
container starts, seeds its settings and initializes Qt. The `qgc-dev` and
`xrce-agent` images are written but were never built. Only the `recon_field`
scene was flown.

## Requirements

- Linux with an X11 session. Wayland works through XWayland, and is less tested.
- An NVIDIA GPU, driver 570.133 or later, and `nvidia-container-toolkit`.
- Docker 24 or later with Compose v2.
- About 80 GB of disk and 16 GB of RAM.

`make preflight` checks all of it and says what is missing.

## Documentation

| File | Contents |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Why the modules split where they do |
| [docs/interfaces.md](docs/interfaces.md) | The exact contract at each boundary |
| [docs/development.md](docs/development.md) | How to change PX4, QGroundControl, ROS and DeepStream |
| [docs/scenarios.md](docs/scenarios.md) | Scenes, vehicles, sensors and target layouts |
| [docs/troubleshooting.md](docs/troubleshooting.md) | What breaks, and what to do |
