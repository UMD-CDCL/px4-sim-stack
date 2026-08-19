# Interfaces

What the simulator produces, and the conventions a consumer must obey to read
it. `docs/uas-contract.md` states WHAT must be present. This page says how the
simulator produces it, and which parts fail quietly.

## 1. MAVLink

### The endpoints

Every vehicle has its own mavlink-router in its own container. The config comes
from `modules/mavlink-router/main.conf.template`, which is
`chimera-deploy/remote/main.conf.template` with the UAS number filled in by
`envsubst`. `N` is the UAS number, 11 to 19.

| Endpoint | Address | Mode | Who uses it |
|---|---|---|---|
| vehicle | `0.0.0.0:14545/udp` | server | PX4 SITL, or a real autopilot |
| offboard | `<GCS_ADDRESS>:14550 + N/udp` | client | the ground station router |
| onboard | `127.0.0.1:14402/udp` | client | MAVROS in the companion container |
| tools | `0.0.0.0:5760/tcp` | server | pymavlink, MAVSDK, mission scripts |
| ground station | `<QGC_ADDRESS>:14550/udp` | client | the QGroundControl container |

Both filtered endpoints carry `AllowSrcSysIn = <N>,255`, so a router passes its
own vehicle and a ground station and nothing else.

The onboard endpoint is loopback because the companion container shares this
container's network namespace. That is one machine with one address, as the
aircraft is.

Every router binds TCP 5760 and sends to port 14402, and they do not collide
because every container has its own network namespace. Only the host ports must
differ, so uas`N` publishes TCP `5750 + N`: 5761 for uas11.

The first and the last endpoint are not on the aircraft. The vehicle endpoint
replaces the serial port PX4 does not have here. The ground station endpoint
exists because QGroundControl autoconnects on 14550, and one QGC then shows the
whole fleet.

### Why the addresses are numbers

mavlink-router parses `Address` as an IP literal. Give it a name and it stops
with `Invalid IP address qgc`. Compose therefore gives every service a fixed
address on the `simnet` network. The prefix is `SIMNET_PREFIX`, and it defaults
to `10.200.142`.

| Service | Address | | Service | Address |
|---|---|---|---|---|
| `offboard` | `.210` | | `video-router` | `.222` |
| `uas11` to `uas19` | `.211` to `.219` | | `qgc` | `.224` |
| `sim` | `.220` | | `qgc-dev` | `.225` |
| | | | `xrce-agent` | `.228` |
| | | | `scenegen` | `.229` |

Two blocks carry meaning. A router for uas`N` is at `.2<N>`, which is where PX4
pushes, and the ground station is at `.210` where the fielded one is `.60`. The
companion container has no address of its own: it shares its router's.

The ADDRESSES deconflict with the fleet. The SUBNET does not. A bridge on this
range shadows the whole real range on a host that is itself on the radio
network, so set `SIMNET_PREFIX=172.28.0` there.

### Connecting

MAVROS, already set on each companion container:

```
FCU_URL=udp://:14402@:14550
```

That binds 14402 and waits. The router speaks first, which is the aircraft's own
configuration.

pymavlink or MAVSDK, from the host, uas11:

```python
from pymavlink import mavutil
link = mavutil.mavlink_connection("tcp:localhost:5761")
link.wait_heartbeat()
print(link.target_system, link.target_component)
```

A ground station outside this stack: run the router from
`chimera-deploy/local/main.conf` on the host. It listens on 14551 upwards and
feeds QGroundControl and MAVROS on the loopback. `GCS_ADDRESS` in `.env` decides
where the fleet sends. The default is the simnet gateway, which is this host.

### The startup handshake

This is the part that surprises people.

PX4 SITL starts a link like this, one for each vehicle:

```
mavlink start -x -u $((14569 + instance)) -o 14545 -t <router address> -r 4000000 -f
```

The `-t` flag sets the partner address up front. PX4 therefore speaks first and
no handshake is needed.

That matters because of how PX4 records a peer. A link started without `-t` is a
UDP server that learns its peer from the first packet it receives, and PX4 then
keeps that address forever: `mavlink_receiver.cpp` records the peer only while
`get_client_source_initialized()` is false, and nothing ever clears it. A router
dials out from an ephemeral source port, so a router restart would change that
port, PX4 would keep sending to the old one, and telemetry would stop with no
error on either side.

With the push link, a router can restart as often as you like, and the simulator
can restart, in any order.

`keepalive.py` runs inside each router. It sends a ground station HEARTBEAT once
a second, as system 255, component 190, so the PX4 data link loss failsafe stays
quiet with no ground station attached.

| `KEEPALIVE` | Behavior |
|---|---|
| `1` (default) | Send forever. The vehicle always sees a ground station. |
| `bootstrap` | Stop once vehicle traffic appears. |
| `0` | Do not run. |

Use `bootstrap` or `0` to test the data link loss failsafe.

### What the simulator sends

`modules/sim/px4-rcS` sets the stream rates. Every message below reaches the
router, and through it MAVROS. The contract's MAVROS topics section names the
readers.

| Message | Rate | What it feeds |
|---|---|---|
| `HEARTBEAT`, `SYS_STATUS`, `EXTENDED_SYS_STATE` | PX4 default | `state` |
| `GLOBAL_POSITION_INT` | 50 Hz | `global_position/global`, `global_position/local` |
| `LOCAL_POSITION_NED` | 50 Hz | `local_position/pose` |
| `ATTITUDE`, `ATTITUDE_QUATERNION` | 50 Hz | `imu/data`, the pose orientation |
| `VFR_HUD` | 10 Hz | `global_position/compass_hdg` |
| `ALTITUDE` | 10 Hz | `altitude` |
| `HOME_POSITION` | 1 Hz | `home_position/home` |
| `GIMBAL_DEVICE_ATTITUDE_STATUS` | 50 Hz | `gimbal_control/device/attitude_status` |
| `GIMBAL_DEVICE_SET_ATTITUDE`, `MOUNT_ORIENTATION` | 20 Hz | the gimbal frame, as a second opinion |
| `DISTANCE_SENSOR` | 10 Hz | `drone_lidar_200m` and `gimbal_lidar_50m` |
| `MISSION_ITEM_INT`, `MISSION_ITEM_REACHED` | on demand | `mission/waypoints`, `mission/reached` |

PX4 drops a link's stream rates back to the profile default when a ground
station appears on it. Measured here, `GIMBAL_DEVICE_ATTITUDE_STATUS` fell from
50 Hz to 0.8 Hz, which left the gimbal frame over a second stale while the
gimbal slewed. `modules/sim/hold-stream-rates.sh` reads the rates back out of
`px4-rcS` and reissues them every ten seconds.

### The two rangefinders

MAVROS names a rangefinder topic after the MAVLink sensor id, and the flight
code reads id 1. The whole map is in the contract's MAVROS topics section.

| id | MAVROS topic | Source |
|---|---|---|
| 0 | `drone_lidar_200m` | the Gazebo lidar PX4 reads through its own bridge |
| 1 | `gimbal_lidar_50m` | `modules/sim/gimbal_rangefinder.py` |

PX4 numbers the id from the uORB instance, and its Gazebo bridge maps exactly
one lidar, on instance 0. `gimbal_rangefinder.py` reads the second Gazebo lidar
and sends it into PX4 as `DISTANCE_SENSOR`, which lands on instance 1 and comes
back out as id 1.

Whoever advertises first owns instance 0, and the wrong order is silent: the
gimbal range would arrive as `drone_lidar_200m` and `gimbal_lidar_50m` would
never publish. So the bridge waits until PX4 reports an id 0 of its own before
it sends anything, and it says so in the log while it waits.

### PX4 parameters, without patching PX4

PX4's `rcS` turns every `PX4_PARAM_<NAME>` environment variable into a
`param set` before the modules start. Add them to the `sim` service in
`compose.yaml`:

```yaml
environment:
  PX4_PARAM_MAV_0_FORWARD: "1"
  PX4_PARAM_COM_RCL_EXCEPT: "4"
  PX4_PARAM_EKF2_GPS_CTRL: "7"
```

This is the supported way to change a parameter. It keeps `src/PX4-Autopilot`
clean, so `git status` there shows only your own work. It applies to every
vehicle, because every vehicle reads the same container environment.

## 2. Video

### The streams

Streams are H.265, CBR, payload type 96. Every camera serves a full stream and a
low-rate stream, and the low-rate stream is the one that crosses a radio link.

| Model | Gimbal RGB | Down-facing | Thermal |
|---|---|---|---|
| `chimera_v2` | `pilot<N>` 1920x1080 | none | `thermal<N>` 640x512 |
| `chimera_v3` | `rgb<N>` 1920x1080 | `pilot<N>` 1920x1080 | `thermal<N>` 640x512 |

The low-rate stream carries the same name with `l` appended: `rgbl<N>`,
`pilotl<N>`, `thermall<N>`. The RGB ones are 640x360 at 1 Mbit/s. The thermal
one keeps 640x512 at 400 kbit/s, as on the aircraft.

`ds_node` on the vehicle reads the full gimbal RGB stream. The ground station
reads the low-rate one. The contract's video section says why 1920x1080 is the
full size.

### Reading

| Protocol | URL | Notes |
|---|---|---|
| RTSP | `rtsp://localhost:8554/rgb11` | The default. Use TCP transport. |
| WebRTC | `http://localhost:8889/rgb11` | Open it in a browser. Lowest latency. |
| HLS | `http://localhost:8888/rgb11` | Widest compatibility, about 2 s behind. |

Inside the compose network, use `video-router` in place of `localhost`.

Check a stream from the host:

```bash
ffplay -fflags nobuffer -flags low_delay rtsp://localhost:8554/rgb11
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/rgb11 protocols=tcp \
  ! rtph265depay ! h265parse ! avdec_h265 ! autovideosink
```

`gz_video_streamer` probes the encoders and takes the first that works, H.265
before H.264. A machine with no H.265 encoder therefore publishes H.264, and a
reader that picks its depayloader from the discovered caps does not care. Set
`VIDEO_ENCODER` and `VIDEO_PARSER` to skip the probe.

### Publishing

A producer publishes with RTSP ANNOUNCE, with RTMP, or as plain RTP.

```bash
# RTSP, which is what gz_video_streamer uses
... ! h265parse ! rtspclientsink protocols=tcp location=rtsp://video-router:8554/rgb11

# RTMP
... ! flvmux ! rtmpsink location=rtmp://video-router:1935/rgb11
```

For plain RTP from a real camera, add a path to
`modules/video-router/mediamtx.yml`:

```yaml
paths:
  pilot11_rtp:
    source: udp+rtp://0.0.0.0:8100
```

### Adding a camera

Two files change, both under `modules/sim/scenes/models/<model>/`.

1. Add the sensor to `model.sdf`, on the airframe that carries it. Do not give
   it a `<topic>`: the default topic holds the model instance name, which is
   the only name that is unique for each vehicle.
2. Add a line to `streams.conf`:

```
# name  topic regex  bitrate  fps  width  height
thermal${UAS_NUM}  ^/world/[^/]+/model/${GZ_MODEL}/link/thermal_camera_link/sensor/thermal_camera/image$  2000  15  -  -
```

The entrypoint fills in `${UAS_NUM}` and `${GZ_MODEL}` for each vehicle, so one
line serves the whole fleet. `width` and `height` rescale the encoder input, and
`-` keeps the size the camera renders. A second line with the same regex costs
one encode and no extra render, which is how the low-rate streams are made.

Restart `sim`. MediaMTX accepts an unlisted path through its `all_others` rule,
so it needs no edit.

## 3. Frames

MAVInsight owns the frame tree, and the contract's frames section holds the tree
itself and the four rules `tf_loc` depends on. What follows is the reasoning
behind those rules, because each one has produced a plausible wrong answer.

### Body frames and optical frames

A camera *body* frame uses x forward along the view axis. A camera *optical*
frame uses REP 103: x right, y down, z forward. The fixed rotation between them
is rpy (-90, 0, -90).

`tf_loc` localizes through the BODY frame `d<N>_rgb_offset`. It maps an optical
ray to `[z, -x, -y]` itself, then applies the camera rotation. A node that
projects through `d<N>_rgb_optical` instead must use standard optical math, and
the two must agree. If they disagree, a camera footprint does not contain the
detections that fall inside it, and both pictures look reasonable on their own.

`CameraInfo` values come from the field of view, not from a calibration. For a
simulated camera that is exact, because a Gazebo camera is an ideal pinhole. For
a real camera, calibrate and replace it.

### Aerospace and ROS conventions

MAVROS reports the gimbal attitude in aerospace convention: NED reference with
FRD body axes. Every frame in the tree is ENU and FLU. Converting between them
needs a rotation on each side, `NED_TO_ENU * q * FRD_TO_FLU`, because both ends
change. Converting a rotation that is already relative to the body needs only
the axis swap, which reduces to negating y and z. Either conversion applied to
the wrong quantity produces a frame that looks reasonable and tracks the
aircraft incorrectly.

`d<N>_base_link` is ENU, so heading is `(90 - yaw_deg) mod 360`.

The gimbal report needs more care than that, and
[px4-simulated-gimbal.md](px4-simulated-gimbal.md) holds the whole argument. Two
sections of it decide whether a camera frame is right: PX4 labels an absolute
attitude as vehicle relative, and the vehicle attitude divides off the LEFT.

### Timestamps

Every lookup happens at the message stamp, so the tree must stay continuous and
must cover `tf_lookup_timeout_duration_sec` past the newest stamp.

The simulator sends no capture time of its own. `ds_node` prefers the sender's
in-band NTP stamp and falls back to its own ingest clock, and both sides of the
link stamp a frame the same way, which is what lets `image_rehydrate` match a
ground frame to a vehicle frame by stamp.

### The ground a ray meets

`tf_loc` casts a ray through each detection box. The ray meets the cached
terrain surface where the vehicle holds a tile for it, and the flat plane
through the measurement frame's origin everywhere else.

One ground serves the fleet, and the contract states that rule. A surface is
anchored on the Earth rather than in a vehicle's own frame, so four vehicles
that look at one casualty cast their rays at one ground.

The surface is `worlds/<SCENE>_surface.json`, which `scenegen build` writes next
to the world. The onboard and offboard entry points link the one surface for
`SCENE` into `TERRAIN_DIR`, because the cache takes the first tile whose square
holds the ray origin and two scenes in one directory would make that choice
arbitrary.

A roof is the building's own footprint polygon with its courtyard holes cut out,
so a ray into a courtyard lands on the ground inside it. Walls are not in the
surface on purpose: a detection on a wall lands on the terrain behind it. Over
sloped ground or a roof, the surface removes an offset that a flat plane bakes
into every estimate.

### Correcting a frame offset with a fiducial

Some of the error is a frame difference rather than a mistake, and on real
hardware most of it is. A position lands in the vehicle's own EKF frame, built
from the vehicle's own receiver. Any other frame, a surveyed map or a second
aircraft, is built from a different receiver and sits metres away. The shapes
agree and the origins do not.

That offset is measured once against a point whose position is known, and then
subtracted. The tree carries the result: `d<N>_fiducial_offset` is the ROOT, and
its edge points down the tree to `uas<N>_home_position`. `tf_loc` looks up a
non-fiducial detection through the fiducial frame and a fiducial detection
through the home frame.

In simulation the true position is written down rather than surveyed. A
`scenegen` scene carries a fiducial marker in the world, and its scenario file
carries the same point as `fiducial_*`. `./px4sim origin` prints it.

**Do not survey a target you also score against.** Fitting the correction to a
scored target and then reporting the error against that same target measures the
arithmetic and nothing else. Use a fiducial no detector is graded on.

## 4. The optional interface: uXRCE-DDS

The `xrce` profile puts PX4 uORB topics straight on the ROS 2 graph. It gives
`px4_msgs` and the low latency that `px4_ros2_interface_lib` needs.

It also breaks the rule this document is about. A stack that uses it depends on
PX4 directly, and it will not run against an aircraft that offers only MAVLink.

```bash
COMPOSE_PROFILES=xrce docker compose up -d
```

Set `ROS_DOMAIN_ID` on the agent to the domain that reads it, which is `60 + N`
for a vehicle.

PX4 also needs the agent address, and `UXRCE_DDS_AG_IP` is a signed 32-bit
integer, not a string. The agent sits at `10.200.142.228`, so the value is
**180915940**. Add it to the `sim` service environment:

```yaml
environment:
  PX4_PARAM_UXRCE_DDS_AG_IP: "180915940"
```

For a different address:

```bash
python3 -c 'import ipaddress, struct
addr = "172.28.0.228"
print(struct.unpack("<i", struct.pack("<I", int(ipaddress.IPv4Address(addr))))[0])'
```
