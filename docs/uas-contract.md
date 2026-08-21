# The UAS contract

What a simulated drone must present so that 5g_drone and MAVInsight cannot tell
it from a Chimera airframe. The simulator satisfies this contract. The flight
code does not accommodate the simulator.

`N` is the UAS number. A real vehicle is 1 to 9. Its simulated counterpart is
the same number plus ten, so `N` is 11 to 19 here. A simulator can then fly
beside the fleet and take no system id, MAVLink port, DDS domain or address from
it.

Models follow the real counterpart: uas11 and uas12 are Chimera v3, uas13 and
uas14 are v2. The calibration and the tuning come from the real vehicle's own
parameter file, so uas11 is a simulated uas1 and carries uas1's lens. Frames,
namespaces and stream names use the simulated number.

## 1. Identity

| Property | Value |
|---|---|
| MAVLink system id | `N` (11 to 19) |
| ROS namespace | `/uas${N}` |
| ROS domain, vehicle | `60 + N` (71 to 79) |
| ROS domain, ground | `70` |
| Air domain, telemetry and commands | `99` |
| Air domain, imagery | `69` |
| TF body frame prefix | `d${N}_` |
| TF world frame prefix | `uas${N}_` |

PX4 sets the system id from the instance, so `px4 -i $((N-1))` gives
`MAV_SYS_ID = N`. For uas11 that is instance 10, which leaves instances 0 to 8
to the real fleet.

Write the port and the domain as arithmetic, never by joining strings. `1455${N}`
and `6${N}` read correctly below ten and produce 145511 and 611 above it, and
both are out of range. `14550 + N` and `60 + N` give the fielded numbers for a
real vehicle and a clear block for a simulated one.

## 2. MAVLink

mavlink-router runs one instance for each vehicle, from
`chimera-deploy/remote/main.conf.template` with `UAS_NUM` substituted.

| Endpoint | Address | Filter |
|---|---|---|
| offboard | `14550 + N` | `AllowSrcSysIn = ${N},255` |
| onboard | `14402` | `AllowSrcSysIn = ${N},255` |
| tools | TCP `5760` | none |

Addresses share the fleet's subnet and sit in their own block. The simulated
ground station is `10.200.142.210` where the real one is `.60`, uas`${N}` is
`10.200.142.2${N}`, and simulator infrastructure sits at `.220` upwards. So a
simulated vehicle and a real one can share the network, and
`main.conf.template` stays the deployed file with the vehicle number filled in.

The addresses deconflict. The subnet does not. A bridge on this range shadows
the whole real range on a host that is itself on the radio network, `.60` to
`.64` included, so set `SIMNET_PREFIX` to `172.28.0` there.

One port for every role, on both sides: MAVROS reads 14402 and QGroundControl
reads 14401, whichever vehicle is in play.

A vehicle is ONE machine. The router and the ROS stack share a network
namespace, as they do on the Orin, so the router reaches MAVROS at
`127.0.0.1:14402` and needs no address of ours.

The ground station is ALSO one machine, and it must be modelled as one. It runs
one MAVROS for each vehicle, and a UDP port holds one listener per ADDRESS
rather than per host. All of `127.0.0.0/8` is local, so each vehicle's MAVROS
binds `127.0.0.${N}:14402` and the port stays the same. The ground router
pushes each vehicle to its own loopback address and filters that endpoint on the
vehicle's system id, so one vehicle's telemetry cannot reach another vehicle's
MAVROS.

Do not model the ground station as several machines. Giving each vehicle its own
ground container would make every `fcu_url` identical, but it would describe a
ground station that does not exist, and the fielded one is a single laptop.

The ground station runs one router from `chimera-deploy/local/main.conf`. It
listens on 14551 to 14554 in server mode and sends to loopback 14401 for
QGroundControl and 14402 for MAVROS.

## 3. Video

Streams are H.265, CBR, payload type 96. Each camera serves a full stream and a
low-rate stream. The low-rate stream crosses the radio link.

| Model | Gimbal RGB | Down-facing | Thermal |
|---|---|---|---|
| v2 | `pilot${N}` 1920x1080, `pilotl${N}` 640x360 | none | `thermal${N}`, `thermall${N}` 640x512 |
| v3 | `rgb${N}` 1920x1080, `rgbl${N}` 640x360 | `pilot${N}`, `pilotl${N}` | `thermal${N}`, `thermall${N}` |

The full stream is 1920x1080 because that is what the detector reads.
`onboard_common_params.yaml` declares `source.width` 1920 and `source.height`
1080, and `DS_WIDTH` and `DS_HEIGHT` match. The aircraft captures the pilot
camera at 3840x2160 and gives DeepStream a smaller surface. The simulator has no
reason to render the larger frame, so it does not.

`ds_node` reads the gimbal RGB camera: `rgb${N}` on v3, `pilot${N}` on v2. On
the aircraft that source is a shared-memory socket. In the simulator it is the
full RTSP mount, which is the only permitted difference.

## 4. MAVROS topics

The MAVROS plugin allowlist is `config/param_files/mavros/px4_pluginlists.yaml`.
Every topic below must carry data.

| Topic | Type | Reader |
|---|---|---|
| `state` | `mavros_msgs/State` | status, umd_uas_mission |
| `global_position/global` | `NavSatFix` | tf_loc, status |
| `global_position/local` | `nav_msgs/Odometry` | MAVInsight |
| `global_position/compass_hdg` | `Float64` | status |
| `local_position/pose` | `PoseStamped` | MAVInsight |
| `altitude` | `mavros_msgs/Altitude` | MAVInsight, umd_uas_mission |
| `home_position/home` | `mavros_msgs/HomePosition` | MAVInsight |
| `gimbal_control/device/attitude_status` | `GimbalDeviceAttitudeStatus` | gimbal, MAVInsight |
| `gimbal_lidar_50m` | `sensor_msgs/Range` | umd_uas_mission |
| `drone_lidar_200m` | `sensor_msgs/Range` | body rangefinder |
| `mission/waypoints` | `WaypointList` | umd_uas_mission |
| `mission/reached` | `WaypointReached` | umd_uas_mission |

The rangefinder names come from the `distance_sensor` plugin `config` string,
and the plugin keys them by MAVLink sensor id:

| id | Topic | Mounting |
|---|---|---|
| 0 | `drone_lidar_200m` | body fixed, down |
| 1 | `gimbal_lidar_50m` | gimbal boresight |
| 2 | `drone_lidar_6m` | body fixed, down |

Only id 1 feeds the flight code. `chimera_common_params.yaml` binds
`telemetry.topic.rangefinder` to `gimbal_lidar_50m`, and `umd_uas_mission`
reads it. The other two are recorded and nothing else.

The gimbal range may reach the topic by either route. Send it over MAVLink as
sensor id 1, or measure the gimbal ray against the scene and publish
`sensor_msgs/Range` directly. Take the second route if the first is difficult.
The topic name, the frame and the value are the contract. How the number is
produced is not.

MAVROS goes to the vehicle through `gimbal_control/manager/set_attitude`, the
`gimbal_control/manager/configure` and `set_roi` services, `cmd/command` and
`mission/set_current`.

The simulator uses the MAVROS package from apt. Its PX4 keeps the legacy
`MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES` shim, so MAVROS still learns the
autopilot capabilities and the waypoint plugin still selects `MISSION_ITEM_INT`.

The aircraft is different. PX4 v1.18 dropped that shim, so a real vehicle needs
the source build and the patch in `chimera-deploy/remote/mavros_patch/`. That
patch is out of scope here.

## 5. Frames

MAVInsight owns the frame tree. MAVROS publishes no transform: every
`tf.send` is false, and every `distance_sensor` entry sets `send_tf: false`.

```
d${N}_fiducial_offset
 └── uas${N}_home_position
      └── uas${N}_ekf_origin
           └── d${N}_base_link
                ├── d${N}_base_link_alt_plane
                └── d${N}_gimbal_frame_offset
                     └── d${N}_gimbal_frame_ref
                          └── d${N}_gimbal_frame
                               ├── d${N}_rangefinder_frame_offset
                               │    └── d${N}_rangefinder_frame
                               └── d${N}_rgb_offset
                                    └── d${N}_rgb_optical
```

A sensor builds `<frame_name>_offset` from its `offset` parameter, and that
parameter must hold six numbers, x y z roll pitch yaw. The rotation goes through
`euler_2_quat`, which reads **degrees**. A camera adds `<frame_name>_optical`, a
REP 103 optical leaf. The gimbal reference frame is `<frame_name>_ref`, and the
vehicle publishes it, not the gimbal.

`d${N}_gimbal_frame_ref` comes from the lock flags of
`GIMBAL_DEVICE_ATTITUDE_STATUS`, because those flags say which frame the gimbal
measures its attitude against:

| lock flags | the reference frame |
|---|---|
| YAW_LOCK | the earth frame, level, x axis North |
| ROLL_LOCK or PITCH_LOCK, no YAW_LOCK | the vehicle frame, level, x axis at the airframe heading |
| none | the body frame |

MAVLink gives the earth frame in NED, so its x axis is North, which is 90
degrees from the x axis of this ENU tree. The Chimera gimbal commands ROLL_LOCK
and PITCH_LOCK, so the middle row is the usual one. A gimbal that sends no lock
flag leaves the reference frame equal to the body frame, and the camera frame is
then wrong at every attitude except level and facing North.

The fiducial correction is the ROOT, and its edge points down the tree:
`fiducial -> home_position`. `tf_loc` looks up a non-fiducial detection through
`d${N}_fiducial_offset` and a fiducial frame through `uas${N}_home_position`.

Four rules that `tf_loc` depends on:

- `d${N}_rangefinder_frame` sits at the measured ground point. The translation
  x of `gimbal_frame -> rangefinder_frame` is the range.
- The x axis of `d${N}_gimbal_frame` points along the boresight.
- `d${N}_base_link` is ENU. Heading is `(90 - yaw_deg) mod 360`.
- `tf_loc` uses the body frame `d${N}_rgb_offset`, not the optical leaf. It maps
  an optical ray to `[z, -x, -y]` itself before it applies the camera rotation.
  A node that projects through `d${N}_rgb_optical` instead must use standard
  optical math, and the two must agree. If they disagree, a camera footprint
  will not contain the detections that fall inside it.

Every lookup happens at the message stamp, so the tree must stay continuous and
must cover `tf_lookup_timeout_duration_sec` past the newest stamp.

## 6. One ground, shared by the fleet

Every vehicle measures against the same physical ground. Four vehicles look at
one casualty, and their reported positions can only be compared if all four cast
their rays at the same surface.

So the ground is never defined by a vehicle's own frame. The z of a home frame
or a fiducial frame is a property of that vehicle, and a ground defined that way
gives four vehicles four different grounds. The same casualty then localizes to
four different places, and nothing can be compared.

The ground is geodetic instead. The terrain surface carries `origin_lla`, and
the flat-plane fallback takes a real altitude for the same reason. Ground truth
is WGS84, and every localization is reported in WGS84, so two vehicles that see
one target report one position.

The fiducial correction is the map from a vehicle's own GPS frame onto that
shared frame. Each vehicle needs its own because each has its own error. The
correction exists to bring the fleet into agreement, not to give a vehicle a
world of its own. It may carry a vertical component, so it applies to the camera
pose before the ray is cast, never to the answer afterwards: a vertical error
moves the ray origin, and a shift applied to the result cannot repair the
geometry that produced it.

## 7. What crosses the radio link

The ground station rebuilds what it can. It runs its own MAVROS off the same
MAVLink stream, its own MAVInsight frame tree, and its own `ds_node` in preview
mode against the low-rate RTSP stream, all on domain 70. Camera footprints, the live view
projection, the drone position and the verdicts are all computed again there.

One rule decides every entry below: a thing crosses only when the ground cannot
rebuild it, or when rebuilding it would cost more than sending it.

### Vehicle to ground, domain `60 + N` to 99 to 70

| Topic | Type | Why |
|---|---|---|
| `target_locations` | `TargetBoxArray`, `source_img` cleared | The localization must be identical, not recomputed. |
| `camera/camera_info` | `CameraInfo`, latched | v3 only. The zoom node owns the calibration on the vehicle, so the ground cannot build it locally. Crosses once. |
| `fiducial_update` | `TransformStamped`, latched | The standing fiducial correction. Without it the ground keeps an identity edge and its outlines sit off by the correction. Crosses when a survey lands. |

Plus what the fielded bridge already carries: `position`, `status`,
`heading_wrt_east`, `observation_no_id`, `hil_detection/undetected`,
`/observations` and `/observation_data_sources`. Imagery keeps its own domain,
69, and carries `mosaic/overlay` and `/casualty_image/compressed/vlm`.

`source_img` is about 40 kB of a 42 kB message, so removing it takes the
detection stream from roughly 680 to 38 kbit/s. A domain bridge cannot edit a
field, so the vehicle publishes a second topic. `image_strip` republishes
`target_locations` as `target_locations/for_air` with the image removed, the
bridge carries that, and `image_rehydrate` refills the field on the ground from
the local RTSP preview by nearest capture time. The refilled frame is a
neighbour rather than the same bytes, which is enough for display and for the
operator's context.

The per-frame telemetry inside the message STAYS: `uav_gps_location`,
`uav_local_pose`, `uav_compass_hdg`, `gimbal_attitude_quaternion` and
`rangefinder_dist`. It is about 900 bytes, and rebuilding it on the ground would
cost more than it saves. The vehicle sampled those values at the capture instant
from its own frame tree. A ground copy interpolates the same values across the
link delay, so it is less exact. The operator interface also reads that
telemetry out of the message rather than from topics of its own, which is why
six of its ten inputs need no bridge.

### Ground to vehicle, domain 70 to 99 to `60 + N`

`gimbal_point_cmd`, `roi_point_cmd`, `raw_roi_point_cmd`, `gimbal_raw_command`,
`gimbal_angle_cmd`, `reassert_gimbal_cmd`, `release_gimbal_cmd` and
`hil_detection/detected`. On a v3, `zoom/preset_cmd` as well.

### Ground to vehicle, domain 70 straight to `60 + N`

`start_survey_cmd`, `vlm_capture_cmd`, `mosaic_capture_cmd`,
`fiducial_capture_cmd`, `advance_mission_cmd` and `continuous_detection_cmd`,
each a `std_msgs/msg/Bool`. These six take no air hop. The ground half writes
them into the vehicle's own domain, so the vehicle half carries no entry for
them: two entries would deliver every command twice.

Five of the six are bare triggers and their contents are ignored.
`continuous_detection_cmd` is the exception: `data` is the switch. It reaches
`img_processing`, which sets its own `continuous` parameter, and that parameter
tells the detector to run and opens the publish types that turn a localized
detection into an observation. A mission sets the same parameter, so an
operator and a mission use one door and the last command wins. It is a topic
and not a parameter service because a service does not cross a domain.

The operator's console runs ON THE GROUND, and every gimbal command is issued
by the VEHICLE. A Foxglove click is a local message and the click mode services
are local services, so neither crosses; what crosses is one small message for
each operator action, between 8 and 750 bytes.

That split is not a preference. PX4 accepts `GIMBAL_MANAGER_SET_ATTITUDE` only
from the station that holds primary control and answers
`DO_GIMBAL_MANAGER_PITCHYAW` from anyone else with `DENIED`
(`src/modules/gimbal/input_mavlink.cpp`), and it implements no secondary
control. A ground station that commanded the gimbal over MAVLink would take it
from the vehicle's own node, and the onboard tracking would stop.

A region of interest is the exception and is instructive: `DO_SET_ROI_LOCATION`
reaches the navigator rather than the gimbal module and carries no sender, so
any station can point the gimbal with one, whoever holds control. The vehicle
holds the place itself and re-sends the attitude, because PX4 computes the
pointing once when the command arrives.

Losing primary control is reported and never contested. `gimbal/state` carries
the mode, the angles, the owner and the held place as one latched sentence, and
`reassert_gimbal_cmd` is the only way back in.

### What never crosses

`/tf`, every MAVROS telemetry topic, the live view point cloud, the camera
footprints, and the course ground truth.

The gimbal lock flags do not cross either. MAVROS publishes
`gimbal_control/manager/set_attitude` outbound, so a listening ground MAVROS
never sees it, but `GIMBAL_DEVICE_ATTITUDE_STATUS` carries the same
`RETRACT`, `NEUTRAL`, `ROLL_LOCK`, `PITCH_LOCK` and `YAW_LOCK` bits, it already
streams over MAVLink, and it reports what the gimbal did rather than what was
asked of it. MAVInsight takes its flags from there.

The course ground truth does not cross because it is static scene data, like the
localization surface. Both sides read the same file. A simulation node turns it
into `/known_casualty_locations` on whichever side wants verdicts, so the
vehicle and the ground can each score without sending anything.

## 8. Constraints that fail quietly

- Every container that carries `cdcl_umd_msgs` must run the same ROS 2
  distribution, and that distribution is Humble, because the aircraft is a
  Jetson Orin on Ubuntu 22.04. Jazzy adds a field to `sensor_msgs/Range`, which
  sits inside `TargetBoxArray` before the box array. A mixed pair decodes the
  scalars correctly and then reports zero boxes, with no error, either way
  round: `seq=7 sysid=11 boxes=3` is received as `seq=7 sysid=11 boxes=0`.
  The distribution comes with the DeepStream base: 7.1 is Humble; 8.0 and 9.0
  are Jazzy. So the fleet is DeepStream 7.1 and Humble throughout. A GPU too
  new for that release's TensorRT gets a newer TensorRT inside the same image,
  never a newer DeepStream. See scripts/ds-select.sh.
- A node specific parameter selector beats a wildcard selector whatever the file
  order. Keep `source.uri` out of the vehicle parameter file.
- Set the imagery flow controller with `FASTRTPS_DEFAULT_PROFILES_FILE` and
  `RMW_FASTRTPS_PUBLICATION_MODE=AUTO` on the image bridge process alone.
- Run the image bridge as its own process. `domain_bridge` republishes inside
  the subscription callback and can block.
- Declare the same QoS on both ends of a bridged topic. A best effort reader
  matches a reliable writer and then discards every repair.
