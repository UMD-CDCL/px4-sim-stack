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

One scene carries many scenarios. One scenario runs in many scenes.

`./px4sim scenario` places the same targets again and needs no restart.
`./px4sim scenario <name>` changes to a different scenario, and that reloads
the world: the scenario carries the origin, and the scoring nodes read its
file at start up.

```bash
# In .env, or on the command line
SCENE=recon_field
UAS_FLEET=chimera_v3 chimera_v3 chimera_v2 chimera_v2
SCENARIO=urban_casualties
```

```bash
./px4sim scene forest        # new world, restarts the sim
./px4sim scenario <name>     # new targets, reloads the world
./px4sim scenario            # place the same targets again, no restart
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

The gimbal is three axis, yaw, roll and pitch, with the travel the aircraft has:
pitch from level to straight down and no further, yaw -180 to +180 degrees off
the nose. An azimuth past that travel must turn the aircraft. The model, the
device report and the flight code parameters state that travel in three places
and they change together. See [px4-simulated-gimbal.md](px4-simulated-gimbal.md).

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
./px4sim scenario umd_campus_casualties        # switch to another one
./px4sim reset                                 # remove what is placed
docker compose exec sim /scenes/spawn_scenario.py --list   # what is placed now
```

A new scenario removes the previous one first, so scenarios do not stack.

A name switches the scenario for this run. It does not write `.env`, so set
`SCENARIO` there to keep the new one.

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

## DeepStream, TensorRT and the GPU

The companion and the ground station are built from an NVIDIA DeepStream image,
and that choice settles the Ubuntu underneath and so the ROS 2 distribution the
flight code is compiled against.

It is not a choice this stack makes. The aircraft is a Jetson Orin on Ubuntu
22.04 with DeepStream 7.1, which is ROS 2 Humble, and everything here is built
to match it:

| | Orin (sm_87) | a Blackwell workstation (sm_120) |
|---|---|---|
| DeepStream | 7.1 | 7.1 |
| Ubuntu | 22.04 jammy | 22.04 jammy |
| ROS 2 | Humble | Humble |
| TensorRT | 10.3, as shipped | 10.9, installed over it |
| image tag | `7.1` | `7.1-trt10.9` |

The reason it cannot be a choice is `cdcl_umd_msgs`. Jazzy adds a field to
`sensor_msgs/Range`, which sits inside `TargetBoxArray` ahead of the box array.
A Humble reader of a Jazzy `TargetBoxArray` decodes `seq`, `system_id` and the
rangefinder correctly and then reports **zero boxes, with no error**, in both
directions. A vehicle and a ground station either side of that line silently
share no detections. See docs/uas-contract.md section 8.

### What does vary: TensorRT

TensorRT only emits kernels for the GPU architectures its release knows.
DeepStream 7.1 carries TensorRT 10.3, which stops at Hopper. On a Blackwell
card it parses the ONNX, reports `Unsupported SM: 0xc00`, builds no engine and
takes the process down -- while everything around it carries on. The streams
decode, the operator sees video, and no box is ever drawn. That failure is why
this section exists.

The fix is not a newer DeepStream, because 8.0 and 9.0 are Ubuntu 24.04 and so
Jazzy. It is a newer TensorRT inside the same 22.04 image. NVIDIA packages
TensorRT for jammy well past what DeepStream shipped with, the soname does not
change, and DeepStream's own `nvinfer` builds and loads engines against it.

`scripts/ds-select.sh` reads the compute capability and decides:

```bash
./px4sim doctor            # the release, the distribution, the TensorRT, and why
./scripts/ds-select.sh     # the same answer as shell assignments
```

Take the **oldest** TensorRT that covers the card, never the newest available.
Each release deletes more of the API that DeepStream 7.1-era sources call: at
10.16 the vendored DeepStream-Yolo parser stops compiling, because
`NetworkDefinitionCreationFlag::kEXPLICIT_BATCH`,
`IBuilder::platformHasFastFp16` and `BuilderFlag::kINT8` are all gone. 10.8 is
the first release that knows Blackwell and 10.9 is the one DeepStream 8.0
itself ships, so 10.9 is the shortest distance from 10.3. That reasoning is in
the header of `ds-select.sh`; read it before raising the version.

### Moving to another machine

Each combination builds to its own image tag, so a machine that moves between
them does not build one over the other and leave a container that will not
start. The derived images follow `DS_TAG`, not the release number.

The TensorRT engine beside each ONNX belongs to one GPU, one driver and one
TensorRT. Delete the `*.engine` files when any of those change and let the
first run rebuild them -- a stale engine that fails to deserialize is rebuilt
anyway, but one that loads and is wrong is worse. The bbox parser is stamped:
`.parser-deepstream` in the model directory records which release built the
`.so` beside it, and the companion's entry point replaces it when they differ.

### Pinning 8.0 or 9.0

```bash
# .env
DS_VERSION=9.0
```

This works -- the pyds bindings are compiled from source, because NVIDIA
publishes no wheel from 9.0 onward -- and it builds Jazzy, which breaks the
fleet as described above. `ds-select.sh` prints that warning every time. It is
there for a machine that has no Humble counterpart to talk to.

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

### The v3 zoom

A v3 carries a zoom lens. gz-sim cannot change a camera's field of view once
the world is loaded, so the airframe model carries one camera per framing. The
streamer reads the narrowest camera that still covers what the lens asks for on
`/uas<N>/camera/zoom`, and crops it by the ratio of the tangents of the half
angles. That ratio is what makes the result a real field of view rather than a
smaller picture. The stream keeps its size, so no consumer renegotiates when
the operator zooms.

`scripts/zoom.sh` is the one table of what the presets are, measured from the
calibration files in 5g_drone. The simulator reads it to crop the camera and
the companion reads it to write the calibration, so the two cannot disagree
about what `mid` means.

```bash
UAS_ZOOM="mid mid - -"        # the preset each vehicle boots at, - for a v2
./px4sim zoom 11 wide         # change the framing while it flies
```

`./px4sim zoom` asks on the topic the operator's Foxglove button publishes. It
then waits for the vehicle to report the framing it reached, prints that
framing and the calibration it read back, and fails if either one disagrees
with the request.

Nothing else has to change to keep the two together. The zoom node moves the
lens and publishes the calibration of the picture it delivers, so the
calibration travels with the lens.

A crop is soft, because it is fewer pixels stretched back. The geometry is
right and the detail is not. Render the camera larger in the airframe model to
buy that back.

#### Two floors the simulated lens has, and what they cost

These numbers come from this stack, with the vehicle 40 m over the ground and
the gimbal 45 degrees down. Neither floor is worth chasing.

**A settled framing rests about 0.42% wide of its nominal field of view.** The
emulated controller models coast, backlash and jitter (`COAST_JITTER_STEPS` and
`BACKLASH_STEPS` in `modules/sim/scf4_emulator.py`). After `preset/wide` it
reports the counter at 40000, but the mechanism rests about 21 steps short. The
picture is then 55.87 degrees where the calibration says 56.06.

That is a focal scale error: zero at the image centre, 0.04 m on the ground
80 px off centre, and about **0.15 m at the frame edge**. A real motorized lens
also rests a few steps off its counter, which is why you calibrate a lens at
its framings. The emulator is doing its job here. Read the resting value with:

```bash
docker compose exec sim gz topic -e -t /uas11/camera/zoom
```

**A widening move delivers the old field of view for about 25 ms.** Gazebo
renders nothing from a camera that nobody subscribes to. So when the lens opens
past the framing the streamer reads, the wider camera needs about 80 ms to give
its first frame, and the streamer clamps the crop until it arrives
(`zoom_fraction_ = min(1.0, ...)` in `modules/sim/gz_video_streamer/main.cc`).
The picture holds the narrower framing while the calibration has already
started to open.

Worst case: **0.75 m** of ground error 160 px off centre, for one frame of a
`mid` to `wide` recall. Narrowing has no such gap, because the camera already
in use covers the target. The aircraft has one sensor and no camera to switch
to, so this floor belongs to the simulator alone.

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

## Verification

`./px4sim verify` runs every stage. Four need nothing running, four fly the
stack.

| Stage | What it establishes |
|---|---|
| `airframes` | Every fleet airframe expands, with the links and sensors the flight code reads |
| `contract` | compose.yaml agrees with the fleet arithmetic: addresses, ports, domains, filters |
| `units` | The terrain rays, the roofs and the ground the fleet shares |
| `localize` | A hardcoded box lands where the geometry says, on the plane and on a roof |
| `vehicle` | Telemetry, the gimbal, the outline, the drape, and a localized target |
| `ground` | The ground station shows what the vehicle worked out, not its own version |
| `fleet` | Every vehicle over one target reports one position for it |
| `captures` | The mosaic is drawn, the fiducial surveys, the VLM frame crosses the link |

The stages that fly are written against `./px4sim uas`, which flies a vehicle
through the interfaces the aircraft uses: MAVROS for flight, the 5g_drone
topics for the gimbal, the detector's own services for a capture. A check that
passes there is a check of the flight code.

Two things a stage has to do or it measures nothing. It waits for the stack,
because Gazebo loads the world, the scenario places its entities, PX4 boots,
the streams come up and the detector builds its engines, which is minutes. And
it settles before it asks: a localization taken while the gimbal is still
slewing is computed against a pose the camera has already left, and one of
those in a sample makes two vehicles look fifteen metres apart.

A wait follows progress rather than the wall clock. Four vehicles with twelve
streams and four detectors run this machine at a fraction of real time, and a
fixed deadline then gives up on a vehicle that is flying perfectly well.

### What it measured

On an RTX 3070 with 16 cores, flying the campus scene.

| | One vehicle, two streams | Four, eight streams | Four, twelve streams |
|---|---|---|---|
| Real time factor | near 1 | about 0.4 | 0.07 to 0.24 |
| Closest box to its recorded target | 0.7 m | 0.2 m | 4.7 to 6.1 m |
| Two vehicles over one target | n/a | 0.27 m | 5.3 m |
| Localizations reaching the ground | every one, unchanged | every one, unchanged | every one, unchanged |
| Gimbal pointing | within half a degree | within half a degree | within half a degree |
| Terrain drawn against the surface file | 31.9 m of relief against 31.9 m | | |

The accuracy figures move with the load and not with the code. Everything that
rests on a timestamp loosens as the simulation falls behind real time: the pose
a capture is localized against, the settling of a gimbal, the age of a fix.
The tolerances in the stages are the line between working and broken, not a
measure of the best the flight code does. A wrong datum, a wrong frame or a
wrong anchor puts a box hundreds of metres out or off the planet, which is what
they catch; the distance inside that is reported rather than graded.

Four things do not move: what reaches the ground, whether it reaches it
unchanged, where the gimbal points, and whether a ray lands on terrain or on
the flat plane.

### Measuring localization error

```bash
./px4sim uas 11 record --seconds 60 > logs/hover.tsv
```

`record` writes one row for every localized box, with the motion that made it
beside the error it made. The rows go to standard output and the summary goes
to standard error, so the command above files the rows and shows the summary.

```
frames       242
rows         1693
unlocalized  0
unjudged     7
horizontal   n 1693  mean 0.26  p50 0.26  p95 0.45  max 0.50
vertical     n 1693  mean 0.01  p50 0.00  p95 0.02  max 0.02
```

That is uas11 at 40 m over the `lorton` casualties, gimbal at -45 degrees,
hovering. `unlocalized` counts boxes that carried no position at all, and
`unjudged` counts rows the scorer had not judged when the row went out.

Error is not one number. It grows with how fast the camera turns, so the
columns beside it carry the answer: `ground_speed`, `slew_rate`, `slant_range`
and `depression`. A run that records only the error cannot say whether a change
helped.

| Column | What it holds |
|---|---|
| `stamp` | the frame's own stamp, the instant the shutter opened |
| `casualty` `verdict` `cited` | which casualty this box is, the scorer's verdict, and how the scorer got there |
| `rep_*` `gt_*` | where the box was reported, and where the scenario really put that casualty |
| `err_east` `err_north` `err_up` `err_horiz` | the reported point measured from the truth |
| `veh_*` `ground_speed` | where the aircraft was and how fast it was flying |
| `gimbal_pitch` `gimbal_yaw` `slew_rate` | where the camera looked, and how fast it turned |
| `slant_range` `depression` | how far the camera was from this box, and how far under the horizon it looked to reach it |
| `px_*` `confidence` `class` | the box in the picture |
| `surface` | `terrain` if the ray met a tile, `plane` if it met the flat ground |
| `sigma` | the localizer's own horizontal uncertainty |

`verdict` is `TP`, `MISLOCALIZED` or `FP`. `cited` is `hit`, `crossed`,
`nearest` or `unjudged`.

`gimbal_pitch` is the angle an operator types: negative looks down.
`gimbal_yaw` is a compass bearing. `slew_rate` is how fast the boresight turns
in the world, so a yawing aircraft moves it as well as a moving mount.

The scorer names each box, on `<ns>/scoring/verdicts`. The command does not
match a box to the nearest truth. Casualties in a scenario stand a few metres
apart, so the nearest truth binds any error wider than half that spacing to the
wrong casualty and then reports the short distance to it. That hides exactly
the errors worth measuring.

A row the scorer did not judge reads `unjudged` in `cited`, and falls back to
the nearest casualty. A run with a large `unjudged` count measures less than it
says. `./px4sim uas <N> detections` and `./px4sim uas <N> published --named`
name their targets the same way.

The camera pose comes from the frame tree at the frame's own stamp. That is the
pose `tf_loc` cast the ray from, so a row measures the localizer and not the
delay in reading it.

Vertical error is `err_up`, and it carries a sign. A mean away from zero is a
bias in the surface the ray met, not a spread. Nothing else in the stack
measures it.

Read a run with `awk`. Take the columns by name off the header line. Mean
horizontal error against slew rate:

```bash
awk -F'\t' 'NR == 1 { for (i = 1; i <= NF; i++) column[$i] = i; next }
            { rate = int($column["slew_rate"])
              seen[rate]++; total[rate] += $column["err_horiz"] }
            END { for (rate in seen)
                    printf "%s deg/s\t%.2f m\t%d rows\n",
                           rate, total[rate] / seen[rate], seen[rate] }' \
  logs/hover.tsv | sort -n
```

### The scene in the 3D panel

Two nodes draw what the vehicle flies over, both from the files a ray is
localized against, so the panel and the localization cannot show different
ground.

| Node | Topic | Draws |
|---|---|---|
| `terrain_viz` | `/viz/scene/terrain` | the terrain, with the satellite image over it |
| `buildings_viz` | `/viz/scene/buildings` | the roofs, one surface for each floor, under the same image |

Both go out as a `foxglove_msgs/SceneUpdate` carrying a binary glTF. Foxglove
draws no texture on a triangle list, and a `ModelPrimitive` carries the model's
bytes in the message, so nothing has to serve a file and nothing has to reach a
URL. The detail is then the image's rather than the mesh's: a third of a metre
a pixel over a 600 m scene, against the 4.7 m a colour for each vertex gave.
`terrain_stride` takes one vertex in every few for a lighter message, at a
quarter of the triangles for each step.

A roof is one polygon whose corners are its only vertices, so a colour for each
corner would wash a whole building into one shade. The image goes over the
roofs too, and the map then reads as one picture draped over the ground and the
buildings on it rather than stopping at every wall.

The mesh is read rather than the surface file because it already carries the
texture coordinates. The imagery is a slippy tile mosaic, and placing a point
in it needs a georeference the built scene no longer ships. Those coordinates
are a straight map over the scene square to within half a pixel, which is how
the roofs take their own piece of the image without reading the mesh at all.

glTF is Y up and -Z forward, and Foxglove turns every glTF model a quarter turn
about X to stand it up in a Z up world. `models/gltf.py` writes the model in
that frame, so east stays east and north becomes -Z; written in ENU instead,
the ground arrives on its side. glTF also measures v down from the top of the
image where COLLADA measures it up from the bottom, and the same function flips
it: unflipped, the map draws mirrored north for south. `test/test_gltf.py`
holds both conventions, and `./px4sim uas ground scene` reads them back off the
wire.

Both place the scene against the vehicle's home fix, and both need
`geoid_height_m` to do it: the scene is anchored above mean sea level and a fix
is above the ellipsoid, so without it the whole scene is drawn a geoid
separation off the ground, 33 m in Maryland. The same separation reaches
`sim_ground_truth`, which publishes casualty positions as fixes and would
otherwise put every one of them 33 m over the ground it lies on.
`modules/ros-base/site-params.sh` works it out from the scene, and both entry
points source it: the ground station draws the same scene against the same
fixes as the vehicle, and a station without it draws its own version.

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
