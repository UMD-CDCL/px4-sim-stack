# Development

Every component you are likely to change lives in a bind mount, in a config
file, or behind one variable in `.env`. This page says which, for each one.

## The general shape

| What you change | Where it lives | Cost of a change |
|---|---|---|
| PX4 firmware | `src/PX4-Autopilot`, on the host | Restart the sim container |
| The airframe or its sensors | `modules/sim/scenes/models/` | Restart the sim container |
| The world | `modules/sim/scenes/worlds/` | Restart the sim container |
| Target layout | `modules/sim/scenes/scenarios/` | `make scenario`, no restart |
| The camera encoders | `modules/sim/gz_video_streamer/` | Rebuild the sim image |
| A ROS stack | `modules/ros/stacks/` | Restart the ros container |
| The detector | `modules/perception/configs/` | Restart the perception container |
| QGroundControl | `src/qgroundcontrol`, on the host | Rebuild inside the qgc-dev container |
| An upstream version | `.env` | Rebuild the affected image |

## PX4

### The setup

`make bootstrap` clones PX4 at the tag in `.env` into `src/PX4-Autopilot`. That
tree is a normal git checkout. Your editor sees it, and so does the container,
at `/px4`.

The build happens inside the container and writes to
`src/PX4-Autopilot/build/`, which is on the host. Restarting the container does
not rebuild from scratch, and ccache keeps the incremental builds fast.

### The loop

```bash
vim src/PX4-Autopilot/src/modules/commander/Commander.cpp
docker compose restart sim
make logs S=sim
```

The entrypoint rebuilds only what changed. A one-file change takes about a
minute.

To build without a restart:

```bash
docker compose exec sim make -C /px4 px4_sitl_default -j16
```

To force a clean build, set `FORCE_BUILD=1` on the sim service, or:

```bash
docker compose exec sim make -C /px4 clean
```

### The PX4 console

PX4 runs as the container's main process, so its `pxh>` prompt is the container
terminal.

```bash
make px4-console
```

Detach with **Ctrl-P Ctrl-Q**. Ctrl-C stops PX4 and the container with it.

At the prompt:

```
pxh> commander status
pxh> listener distance_sensor
pxh> param show MPC_XY_VEL_MAX
pxh> mavlink status
pxh> gz_bridge status
```

### Changing a parameter

Prefer the environment. `rcS` applies every `PX4_PARAM_<NAME>` variable before
the modules start, so `src/PX4-Autopilot` stays clean:

```yaml
# compose.yaml, the sim service
environment:
  PX4_PARAM_MPC_XY_VEL_MAX: "12.0"
```

For an experiment, set it at the console with `param set`. Those values live in
the SITL parameter file and survive a restart, which is convenient and easy to
forget. `param reset_all` clears them.

### A different PX4 version

```bash
# .env
PX4_REF=v1.16.0
```

Then rebuild the image, because the dependency set comes from that release:

```bash
rm -rf src/PX4-Autopilot
make bootstrap
make build-sim
```

## The airframe and its sensors

`modules/sim/scenes/models/x500_recon/model.sdf` is the vehicle. It merges the
upstream `x500`, adds the upstream `gimbal`, and defines a nadir camera and a
rangefinder.

Two names in that file are fixed by PX4, not by choice. PX4's Gazebo bridge
subscribes to one topic for the rangefinder:

```
/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan
```

The link must be `lidar_sensor_link` and the sensor must be `lidar`. Rename
either and PX4 reports no rangefinder and logs nothing.

A custom model does **not** need a PX4 airframe file. `PX4_SYS_AUTOSTART` picks
the parameters and `PX4_SIM_MODEL` picks the model, and the two are
independent. The stack uses airframe 4019, which is the gimbal-enabled x500,
with the model `x500_recon`. That works because 4019 sets `PX4_SIM_MODEL` only
when the variable is unset.

To add a camera, see the "Adding a camera" section of
[interfaces.md](interfaces.md).

## The camera encoders

`modules/sim/gz_video_streamer/main.cc` reads Gazebo camera topics and encodes
them. It uses gz-transport and GStreamer, and it is not a Gazebo system plugin,
so it starts, stops and fails on its own.

Change it and rebuild the sim image:

```bash
make build-sim
docker compose up -d --force-recreate sim
```

Test it by hand:

```bash
docker compose exec sim gz_video_streamer --help
docker compose exec sim gz topic -l | grep image
```

To force the software encoder, add `--no-cuda`, or set an explicit fragment:

```bash
gz_video_streamer --encoder "x264enc tune=zerolatency bitrate=2000" ...
```

## ROS 2

See [modules/ros/stacks/README.md](../modules/ros/stacks/README.md) for the
stack contract. In short:

```bash
cp -r modules/ros/stacks/baseline modules/ros/stacks/my_stack
# edit modules/ros/stacks/my_stack/stack.launch.py
ROS_STACK=my_stack docker compose up -d --force-recreate ros
```

Interactive work:

```bash
make ros                       # a shell, overlay sourced
colcon build --symlink-install # rebuild in place
ros2 launch /stacks/my_stack/stack.launch.py
```

Set `ROS_AUTOLAUNCH=0` on the ros service to bring the container up without
starting the stack. It then idles and waits for you.

To add a ROS package to the image, edit `modules/ros/Dockerfile` and run
`make build-ros`. A package that a stack needs at run time belongs there, not
in the stack.

### rviz2 and rqt

Both are installed, and the container has the X11 socket:

```bash
docker compose exec ros bash -lc rviz2
docker compose exec ros bash -lc 'rqt_image_view /camera/gimbal/image_raw'
```

## QGroundControl

### The released build

The default `qgc` container runs the official AppImage. Change the version in
`.env` and rebuild:

```bash
# .env
QGC_REF=v5.1.0
```

```bash
make build-qgc && docker compose up -d --force-recreate qgc
```

The container seeds `QGroundControl.ini` on first start, so the vehicle
autoconnects and the video source is already the gimbal stream. It never
overwrites a file you already have. To start over:

```bash
docker volume rm px4simstack_qgc-config
```

### Building your own

```bash
./scripts/bootstrap.sh qgc                 # clone the source into ./src
COMPOSE_PROFILES=qgc-dev docker compose build qgc-dev   # Qt, about 3 GB
COMPOSE_PROFILES=qgc-dev docker compose up -d qgc-dev
docker compose exec qgc-dev qgc-build      # 15 to 40 minutes the first time
docker compose exec qgc-dev qgc-run
```

Then the loop is:

```bash
vim src/qgroundcontrol/src/...
docker compose exec qgc-dev qgc-build
docker compose exec qgc-dev qgc-run
```

Stop the released `qgc` container first, or two ground stations compete for
UDP 14550 and each gets half the telemetry.

`QGC_BUILD_TYPE=Release` gives a faster binary and a slower build. The Qt
version must match the one that release of QGroundControl expects. Look at
`.github/workflows/linux.yml` in the source tree, then set `QT_VERSION` in
`.env`.

## DeepStream

There is no application code. `deepstream-app` reads
`modules/perception/configs/camera_detector.txt`.

```bash
vim modules/perception/configs/camera_detector.txt
docker compose restart perception
make logs S=perception
```

The entrypoint expands `${RTSP_IN}`, `${MQTT_HOST}`, `${MQTT_PORT}` and
`${MQTT_TOPIC}` into a copy under `/tmp/perception`, so the file in the
repository keeps the variables.

A second config is a second file plus one variable:

```bash
DS_CONFIG=nadir_detector.txt docker compose up -d --force-recreate perception
```

For a different model, see
[modules/perception/models/README.md](../modules/perception/models/README.md).

Useful checks:

```bash
mosquitto_sub -h localhost -t 'perception/#' -v
ffplay rtsp://localhost:8554/gimbal_annotated
docker compose exec perception nvidia-smi
```

To see every payload when the topic is silent, uncomment `debug-payload-dir` in
the config. The files land in `logs/perception/`.

## Adding a module

Say you want a mission executor that speaks MAVLink and reads detections.

1. Make `modules/mission/` with a Dockerfile.
2. Add a service to `compose.yaml`, on `simnet`, with its own profile.
3. Give it the addresses it needs, and only those:

```yaml
mission:
  profiles: [mission]
  build: ./modules/mission
  environment:
    MAVLINK_URL: tcp://mavlink-hub:5760
    MQTT_HOST: message-bus
  networks: [simnet]
```

Do not give it a Gazebo path, a PX4 source mount or a ROS overlay. If it needs
one of those, the boundary is in the wrong place.

## Running against real hardware

The point of the layout. Three changes:

1. Stop the `sim` service.
2. Point `mavlink-hub` at the aircraft. Edit `PX4_HOST` and `PX4_PORT` on that
   service, or change the endpoint to a serial device in
   `modules/mavlink-hub/entrypoint.sh`.
3. Point `video-router` at the real camera. Add a path with an `rtsp://` or
   `udp+rtp://` source in `modules/video-router/mediamtx.yml`.

The `ros` and `perception` containers need no change at all. That is the test
of whether the boundaries are real.
