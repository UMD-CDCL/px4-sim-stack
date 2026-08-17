# Scenes, vehicles and scenarios

Three separate things, on purpose:

| Term | What it is | Where |
|---|---|---|
| **Scene** | The terrain, the buildings, the light | `modules/sim/scenes/worlds/<name>.sdf` |
| **Vehicle** | The airframe and its sensors | `modules/sim/scenes/models/<name>/` |
| **Scenario** | The targets placed in a scene | `modules/sim/scenes/scenarios/<name>.yaml` |

One scene carries many scenarios. One scenario runs in many scenes. Changing a
scenario needs no simulator restart.

## Switching

```bash
# In .env, or on the command line
SCENE=recon_field
VEHICLE=x500_recon
SCENARIO=urban_casualties
```

```bash
./px4sim scene forest        # new world, restarts the sim
./px4sim scenario            # place the targets again, no restart
./px4sim reset               # remove the targets
./px4sim origin              # the coordinates this pair flies at
```

The scene and the scenario carry the coordinates. `SCENE` and `SCENARIO` are
the only two lines that select them: the origin and the survey marker are read
from the scenario file (`home_*` and `fiducial_*`), and from the world's
`<spherical_coordinates>` when the scenario carries none. Keep no coordinate in
`.env`. A second copy can disagree with the world, and then the whole mission
flies somewhere else and looks correct while it does it.

## Scenes

The stack ships `recon_field`: flat ground, a dirt track and three blocks. It
is light, so the frame rate leaves room for the encoders and for DeepStream.

Every world from the PX4 model set is also available, because the entrypoint
merges the PX4 world directory with this one:

`aruco`, `baylands`, `default`, `forest`, `frictionless`, `kth_marinarium`,
`kthspacelab`, `lawn`, `moving_platform`, `ridge`, `rover`, `underwater`,
`walls`, `windy`.

```bash
./px4sim scene baylands
```

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
override `default.sdf` without touching the PX4 tree.

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

The `scenegen` module builds a scene of a real place: terrain from
elevation tiles, the satellite image on the ground, OSM buildings, and
detected cars and buses. A browser editor fixes what the data got wrong,
and a fiducial marker ties the world to the map frame.

```bash
./px4sim genscene create --name campus --center 38.9869,-76.9426 --side 600
./px4sim genscene --help
```

A generated scenario carries `home_*` and `fiducial_*` lines, and every
world file carries `<spherical_coordinates>`. `./px4sim` reads the
scenario first and falls back to the world
(`scripts/origin-env.sh`), so `SCENE` and `SCENARIO` in `.env` select the
world, the targets, the origin and the survey marker together. No
coordinate is written in `.env`. Print what the pair resolves to:

```bash
./px4sim origin
```

See [modules/scenegen/README.md](../modules/scenegen/README.md).

## Vehicles

`x500_recon` is the default. It carries:

| Payload | Detail | Where it goes |
|---|---|---|
| Gimbal camera | 1920x1080, 15 fps, 3-axis mount | `rtsp://video-router:8554/gimbal` |
| Nadir camera | 1920x1080, 15 fps, fixed, points down | `rtsp://video-router:8554/nadir` |
| Rangefinder | LW20 laser, 100 m, points down | MAVLink `DISTANCE_SENSOR` |

The upstream PX4 vehicles are available too: `x500`, `x500_gimbal`,
`x500_mono_cam`, `x500_depth`, `x500_lidar_down`, `standard_vtol`,
`rover_ackermann`, `advanced_plane` and the rest.

```bash
VEHICLE=x500_depth AIRFRAME=4002 ./px4sim scene
```

`AIRFRAME` must match the vehicle. The number is the PX4 `SYS_AUTOSTART` id,
and the list is in
`src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/`.

An upstream vehicle other than `x500_recon` has no `streams.conf`, so it
publishes no video. Add one to give it a stream. See
[interfaces.md](interfaces.md).

### A new vehicle

```bash
cp -r modules/sim/scenes/models/x500_recon \
      modules/sim/scenes/models/x500_thermal
```

Edit `model.sdf`, change `<model name>` to match the directory, and edit
`streams.conf`. Then:

```bash
VEHICLE=x500_thermal ./px4sim scene
```

You do not need a PX4 airframe file. `AIRFRAME` supplies the parameters and
`VEHICLE` supplies the geometry, and they are independent.

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

Applying a scenario removes the previous one first, so scenarios do not stack.

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

## Speed and determinism

```bash
SIM_SPEED_FACTOR=2      # run the physics twice as fast
GZ_GUI=0                # no Gazebo window, for a batch run
```

Faster than real time stresses the pipeline in a way real time does not. The
cameras still render at their configured rate in simulation time, so the
encoders and DeepStream receive frames faster than they can be produced in the
real world. Expect dropped frames above about 2x on this hardware.

A run repeats itself because the origin is scene data: the same `SCENE` and
`SCENARIO` always start from the same coordinates. To fly a scene from
another point, edit `home_lat`, `home_lon` and `home_alt` in its scenario
file. Check the result before the flight:

```bash
./px4sim origin
```

## A full test run

```bash
# 1. Choose the scene and the targets
SCENE=recon_field SCENARIO=urban_casualties ./px4sim start

# 2. Wait for the vehicle in QGroundControl, then plan a survey in the app
#    and start the mission

# 3. Watch what the detector sees
mosquitto_sub -h localhost -t perception/detections -v
ffplay rtsp://localhost:8554/gimbal_annotated

# 4. Change the targets without restarting anything
vim modules/sim/scenes/scenarios/urban_casualties.yaml
./px4sim scenario

# 5. Fly the same mission again
```
