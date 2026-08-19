# Development

Every part you are likely to change lives in a bind mount, in a config file, or
behind one variable in `.env`. This page says which, for each one.

| What you change | Where it lives | Cost of a change |
|---|---|---|
| The world | `modules/sim/scenes/worlds/` | Restart the sim container |
| Target layout | `modules/sim/scenes/scenarios/` | `./px4sim scenario`, no restart |
| The airframe or its sensors | `modules/sim/scenes/models/` | Restart the sim container |
| The camera encoders | `modules/sim/gz_video_streamer/` | Rebuild the sim image |
| PX4 firmware | `src/PX4-Autopilot`, on the host | Restart the sim container |
| The flight code | the tree at `ROS2_WS_DIR` | Rebuild the onboard and offboard images |
| QGroundControl | `src/qgroundcontrol`, on the host | Rebuild inside the qgc-dev container |
| An upstream version | `.env` | Rebuild the affected image |

## Scenes, vehicles and scenarios

Three separate things, on purpose:

| Term | What it is | Where |
|---|---|---|
| **Scene** | The terrain, the buildings, the light | `modules/sim/scenes/worlds/<name>.sdf` |
| **Vehicle** | The airframe and its sensors | `modules/sim/scenes/models/<name>/` |
| **Scenario** | The targets placed in a scene | `modules/sim/scenes/scenarios/<name>.yaml` |

One scene carries many scenarios. One scenario runs in many scenes. A change of
scenario needs no simulator restart.

```bash
# In .env, or on the command line
SCENE=recon_field
UAS_FLEET=chimera_v3 chimera_v3 chimera_v2 chimera_v2
SCENARIO=urban_casualties
```

```bash
./px4sim scene forest        # new world, restarts the sim
./px4sim scenario            # place the targets again, no restart
./px4sim reset               # remove the targets
./px4sim origin              # the coordinates this pair flies at
```

The scene and the scenario carry the coordinates. `SCENE` and `SCENARIO` are the
only two lines that select them: the origin and the survey marker are read from
the scenario file (`home_*` and `fiducial_*`), and from the world's
`<spherical_coordinates>` when the scenario carries none. Keep no coordinate in
`.env`. A second copy can disagree with the world, and then the whole mission
flies somewhere else and looks correct while it does it.

## Scenes

The stack ships `recon_field`: flat ground, a dirt track and three blocks. It is
light, so the frame rate leaves room for the encoders and the detectors.

Every world from the PX4 model set is also available, because the entrypoint
merges the PX4 world directory with this one:

`aruco`, `baylands`, `default`, `forest`, `frictionless`, `kth_marinarium`,
`kthspacelab`, `lawn`, `moving_platform`, `ridge`, `rover`, `underwater`,
`walls`, `windy`.

`baylands` is large and detailed. Expect a lower frame rate and a warning from
Gazebo about the mesh count.

### A new scene

Copy `recon_field.sdf` and edit it.

```bash
cp modules/sim/scenes/worlds/recon_field.sdf \
   modules/sim/scenes/worlds/quarry.sdf
```

One rule, and the entrypoint checks it: the `<world name>` inside the file must
match the file name. PX4 addresses the world by its declared name, and a
mismatch hangs with no error.

```xml
<world name="quarry">
```

A file of yours that has the same name as a PX4 world replaces it, so you can
override `default.sdf` without a change to the PX4 tree.

### Models from Fuel

A world or a scenario can pull a model from the Gazebo Fuel library by URL. The
first use downloads it into the `sim-fuel` volume, and later runs come from
there.

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree</uri>
  <pose>12 -4 0 0 0 0</pose>
</include>
```

Browse the library at <https://app.gazebosim.org/fuel/models>.

### A scene from map data

The `scenegen` module builds a scene of a real place: terrain from elevation
tiles, the satellite image on the ground, OSM buildings, and detected cars and
buses. A browser editor fixes what the data got wrong, and a fiducial marker
ties the world to the map frame.

```bash
./px4sim genscene create --name campus --center 38.9869,-76.9426 --side 600
./px4sim genscene --help
```

`scenegen build` also writes `worlds/<name>_surface.json`, the localization
surface that `tf_loc` reads, and `worlds/<name>_buildings.json`, which
MAVInsight draws. A scene built before those exports needs one rebuild to
produce them.

See [modules/scenegen/README.md](../modules/scenegen/README.md).

## Vehicles

`UAS_FLEET` names one model for each vehicle, in UAS number order. The default
fleet follows `docs/uas-contract.md`: uas11 and uas12 are `chimera_v3`, uas13
and uas14 are `chimera_v2`.

Both marks build on `chimera_common`, which carries the x500 frame, the gimbal
mechanism, a 50 m laser along the gimbal boresight and a 200 m laser pointing
down. The two marks add the cameras they fly.

| Model | Payload | Streams |
|---|---|---|
| `chimera_v2` | gimbal RGB, gimbal thermal | `pilot<N>`, `pilotl<N>`, `thermal<N>`, `thermall<N>` |
| `chimera_v3` | gimbal RGB, gimbal thermal, down-facing RGB | `rgb<N>`, `rgbl<N>`, `pilot<N>`, `pilotl<N>`, `thermal<N>`, `thermall<N>` |

The gimbal is two axis, roll and pitch, with the pitch travel the aircraft has:
level to straight down and no further. Yaw is locked, because the aircraft has
no yaw actuator on the mount. A pointing command that needs azimuth must turn
the aircraft. See [px4-simulated-gimbal.md](px4-simulated-gimbal.md).

Both models are templates. The camera field of view belongs to the vehicle
rather than to the mark, so the entrypoint renders one airframe model for each
vehicle from `UAS_GIMBAL_HFOV_DEG`, `UAS_THERMAL_HFOV_DEG` and
`UAS_DOWN_HFOV_DEG`. `.env.example` says where each number came from. The
entrypoint writes the result to `/tmp/scenes/models/uas<N>/model.sdf` and points
that vehicle's PX4 at it.

Three names in `chimera_common` are fixed by PX4, not by choice. PX4's Gazebo
bridge subscribes to one topic for the rangefinder it reads:

```
/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan
```

The link must be `lidar_sensor_link` and the sensor must be `lidar`. Rename
either and PX4 reports no rangefinder and logs nothing. The gimbal laser uses
other names on purpose, so that PX4 leaves it alone. `gimbal_rangefinder.py`
sends it in as sensor id 1.

Three more names are fixed by PX4's gimbal driver. `GZGimbal.cpp` commands
`/model/<model>/command/gimbal_roll`, `gimbal_pitch` and `gimbal_yaw`, and reads
the mount attitude from the `camera_imu` sensor on `camera_link`.

A custom model does **not** need a PX4 airframe file. `PX4_SYS_AUTOSTART` picks
the parameters and `PX4_SIM_MODEL` picks the model, and the two are independent.
The stack uses airframe 4019, the gimbal-enabled x500, for both marks. That
works because 4019 sets `PX4_SIM_MODEL` only when the variable is unset.

### A new vehicle

```bash
cp -r modules/sim/scenes/models/chimera_v3 \
      modules/sim/scenes/models/chimera_v4
```

Edit `model.sdf`, change `<model name>` to match the directory, and edit
`streams.conf`. Then put the name in `UAS_FLEET` and run `./px4sim scene`.

A model with no `streams.conf` publishes no video. The upstream PX4 vehicles are
still available by name, and none of them carries one.

To add a camera, see "Adding a camera" in [interfaces.md](interfaces.md).

## Scenarios

A scenario is a list of things to place in the world.

```yaml
name: urban_casualties
description: Six people around the blocks, two of them out of sight from above.

entities:
  - name: person_track_01
    uri: https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person
    pose: [18, 1.5, 0, 0, 0, 2.1]
    static: true
```

| Field | Meaning |
|---|---|
| `name` | Unique in the world. It is also the handle for removal. |
| `uri` | `model://<name>` for a local model, or a Fuel URL |
| `pose` | `x y z roll pitch yaw`, in world ENU metres and radians |
| `static` | `true` keeps the model still. Default is `true`. |

```bash
./px4sim scenario                              # apply SCENARIO from .env
./px4sim reset                                 # remove what is placed
docker compose exec sim /scenes/spawn_scenario.py --list   # what is placed now
```

A new scenario removes the previous one first, so scenarios do not stack.

The same file is the ground truth. `sim_ground_truth` reads it and publishes
`/known_casualty_locations`, and the scoring nodes on both sides match estimates
against that. The simulator also writes `ground_truth_actual.yaml`, the poses
read back from Gazebo, and that file wins where it exists.

### Your own target models

Fuel has upright people. It does not have a good prone casualty. Put your own
mesh in the model directory and reference it locally:

```
modules/sim/scenes/models/casualty_prone/
├── model.config
├── model.sdf
└── meshes/
    └── casualty_prone.dae
```

```yaml
  - name: casualty_01
    uri: model://casualty_prone
    pose: [22, -8, 0, 0, 0, 1.4]
```

Nothing else changes. The model directory is merged into the Gazebo resource
path at start, so a local model resolves the same way an upstream one does.

### Edge cases worth building

The point of a scenario file is that a hard case is a file, not a rebuild.

| Case | How to build it |
|---|---|
| Occlusion | Put a target against a wall, so the gimbal sees it from one heading only |
| Shadow | Put a target on the north side of a block and change the sun direction |
| Density | Put six targets inside one camera frame |
| Scale | Put targets at 5 m and at 60 m from the flight path |
| A false positive | Place a `PatientWheelChair` with nobody in it |
| Nothing at all | An empty `entities` list. A detector that reports objects here is wrong. |

## PX4

`./px4sim setup` clones PX4 at the tag in `.env` into `src/PX4-Autopilot`. That
tree is a normal git checkout. Your editor sees it, and so does the container,
at `/px4`.

The build happens inside the container and writes to `src/PX4-Autopilot/build/`,
which is on the host. A container restart does not rebuild from scratch, and
ccache keeps the incremental builds fast.

```bash
vim src/PX4-Autopilot/src/modules/commander/Commander.cpp
docker compose restart sim
./px4sim logs sim
```

The entrypoint rebuilds only what changed. A one-file change takes about a
minute. To build without a restart:

```bash
docker compose exec sim make -C /px4 px4_sitl_default -j16
```

To force a clean build, set `FORCE_BUILD=1` on the sim service, or run
`docker compose exec sim make -C /px4 clean`.

### The PX4 console

PX4 runs as the container's main process, so its `pxh>` prompt is the container
terminal. Detach with **Ctrl-P Ctrl-Q**. Ctrl-C stops PX4 and the container with
it.

```bash
./px4sim console
```

```
pxh> commander status
pxh> listener distance_sensor
pxh> param show MPC_XY_VEL_MAX
pxh> mavlink status
pxh> gz_bridge status
```

Every other vehicle runs headless. `px4` sends one of them a single command. It
takes the UAS number and fills in the PX4 instance, which is that number minus
one:

```bash
./px4sim px4 12 mavlink status
docker compose exec sim /px4/build/px4_sitl_default/bin/px4-mavlink --instance 11 status
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
./px4sim setup
./px4sim build sim
```

## The camera encoders

`modules/sim/gz_video_streamer/main.cc` reads Gazebo camera topics and encodes
them. It uses gz-transport and GStreamer, and it is not a Gazebo system plugin,
so it starts, stops and fails on its own.

```bash
./px4sim build sim
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

## The flight code

5g_drone, cdcl_umd_msgs and MAVInsight are not in this repository. `ROS2_WS_DIR`
in `.env` names the checkout, and the onboard and offboard images build it with
colcon. So a change there is a rebuild, not a restart:

```bash
./px4sim build onboard offboard
docker compose up -d --force-recreate onboard11 offboard
```

Both images build the same workspace, so a change in a shared package needs
both.

The launch files take the identity as arguments, and the entry points fill them
in from `UAS_NUM` and `UAS_FLEET`:

```bash
ros2 launch umd_uas onboard.launch.py uas:=11 model:=v3 sim:=true
ros2 launch umd_uas offboard.launch.py uas:=11,12,13,14 models:=v3,v3,v2,v2
```

To look at a running graph, remember that `docker compose exec` does not run the
entry point, so the domain and the overlay are not set:

```bash
./px4sim topics 11        # does both for you
docker compose exec -e ROS_DOMAIN_ID=71 onboard11 bash -lc \
  '. /opt/ros/humble/setup.bash; . /home/user/ros2_ws/install/setup.bash; ros2 node list'
```

A vehicle holds domain `60 + N`. The ground station holds 70.

## QGroundControl

### The released build

The default `qgc` container runs the official AppImage. Change the version in
`.env` and rebuild:

```bash
# .env
QGC_REF=v5.1.0
```

```bash
./px4sim build qgc && docker compose up -d --force-recreate qgc
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

Stop the released `qgc` container first, or two ground stations compete for UDP
14550 and each gets half the telemetry.

`QGC_BUILD_TYPE=Release` gives a faster binary and a slower build. The Qt
version must match the one that release of QGroundControl expects. Look at
`.github/workflows/linux.yml` in the source tree, then set `QT_VERSION` in
`.env`.

## Speed and determinism

```bash
SIM_SPEED_FACTOR=2      # run the physics twice as fast
GZ_GUI=0                # no Gazebo window, for a batch run
```

Faster than real time stresses the pipeline in a way real time does not. The
cameras still render at their configured rate in simulation time, so the
encoders and the detectors receive frames faster than the real world can produce
them. Expect dropped frames above about 2x on this hardware.

A run repeats itself because the origin is scene data: the same `SCENE` and
`SCENARIO` always start from the same coordinates. To fly a scene from another
point, edit `home_lat`, `home_lon` and `home_alt` in its scenario file. Check
the result before the flight with `./px4sim origin`.

## Running against real hardware

The point of the layout. Three changes:

1. Stop the `sim` service.
2. Point the router for that vehicle at the aircraft. The aircraft carries its
   own router, so the usual answer is to stop the `uas<N>` service as well and
   let the companion computer's router do the work. To keep a router here,
   change its vehicle endpoint to a serial device in
   `modules/mavlink-router/main.conf.template`, which is where the aircraft
   template has its `[UartEndpoint alpha]`.
3. Point `video-router` at the real camera. Add a path with an `rtsp://` or
   `udp+rtp://` source in `modules/video-router/mediamtx.yml`.

The onboard and offboard containers need no change beyond `sim:=false`, which
takes the camera from the shared memory socket instead of from RTSP. That is the
test of whether the boundaries are real.
