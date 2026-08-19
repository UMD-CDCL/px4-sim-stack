# Architecture

## The rule

A module may depend on an address. A module may not depend on another module's
internals.

The simulator presents two addresses: a MAVLink endpoint and an RTSP URL. Put a
real aircraft behind those two and nothing downstream changes. Everything above
them, the ROS domains and the topic names, belongs to the flight code, and
`docs/uas-contract.md` states it.

## Why these boundaries

### Video leaves the simulator as H.265, not as a topic

The obvious design puts Gazebo and the flight code in one container and bridges
images with `ros_gz`. It is less code on day one and it costs you later:

- The flight code then knows that Gazebo exists. On the aircraft it does not, so
  the code paths diverge exactly where you stop testing them.
- `ros_gz` pulls in ROS-vendored Gazebo packages. The simulator needs upstream
  Gazebo. One image for both is a version fight with no winner.
- A raw 1920x1080 image topic is 6.2 MB per frame. H.265 at 8 Mbit/s is 33 kB.

So the simulator encodes and `ds_node` decodes. That is what happens on the
aircraft, where the camera is a shared memory socket instead. The RTSP source is
the one difference the contract allows.

### MAVLink goes through a router, not point to point

PX4 SITL opens four MAVLink links, and a consumer can attach to any of them.
That works, and it makes every consumer learn which port does what.

A router gives one endpoint for each role and hides the vehicle. It is also what
runs on the companion computer, so the shape of the wiring does not change when
the vehicle does. The config is `chimera-deploy/remote/main.conf.template` with
the UAS number filled in.

There is one router for each vehicle, in its own container. Separate containers
are not decoration: every router binds TCP 5760 and pushes to port 14402, and
only a private network namespace lets nine of them do that. The aircraft gets
the same property from being nine aircraft.

### A vehicle is one machine, and so is the ground station

The companion container shares its router's network namespace, so both hold one
address and the router reaches MAVROS at `127.0.0.1:14402`. The Orin does the
same, and `main.conf.template` then needs no address of ours.

The ground station is one laptop, so it is one container. It runs one MAVROS for
each vehicle, and a UDP port holds one listener for each ADDRESS rather than for
each host. Each vehicle's MAVROS therefore binds `127.0.0.<N>:14402` and the
port stays the same. `ground-router` shares that namespace and pushes each
vehicle to its own loopback address.

A container for each vehicle on the ground would make every `fcu_url` identical,
but it would describe a ground station that does not exist.

### The simulator holds PX4, Gazebo and the whole fleet together

These could be separate containers. They are not, because PX4 talks to Gazebo
over gz-transport, which finds its peers with UDP multicast. Multicast across a
docker bridge works until it does not.

The same argument keeps every vehicle in one container. One Gazebo server holds
the world, and each vehicle is a PX4 SITL instance started as
`px4 -i <instance>`, which gives it `MAV_SYS_ID = instance + 1` and the Gazebo
model name `<model>_<instance>`. `PX4_GZ_STANDALONE=1` makes each instance
attach to the world that already exists.

### The flight code is not vendored

5g_drone, cdcl_umd_msgs and MAVInsight stay in their own checkout, and the
onboard and offboard images build that directory with colcon through a named
build context. `ROS2_WS_DIR` says where it is. A copy in this repository would
be a second version of the flight code, and the point of the stack is that
there is only one.

## What is inside each container

### sim

| Piece | Job |
|---|---|
| Gazebo Harmonic server | Physics, rendering, sensors. One, for the fleet |
| PX4 SITL | Firmware, one instance for each vehicle |
| `gz_video_streamer` | Gazebo camera topics to H.265. One process for each vehicle |
| `gimbal_rangefinder.py` | The gimbal laser into PX4 as `DISTANCE_SENSOR` id 1 |
| `hold-stream-rates.sh` | Reissues the MAVLink stream rates PX4 drops |
| `spawn_scenario.py` | Puts the targets in the world at run time |

PX4 runs in standalone mode. The entrypoint starts Gazebo first, then PX4
attaches to the world that already exists. That order lets the entrypoint choose
the world and the model directory, which PX4 would otherwise overwrite from its
own generated `gz_env.sh`.

`gz_video_streamer` is a program in this repository. PX4 ships a similar plugin,
and that one binds to the first camera it finds. This stack has several cameras
on several vehicles, so it needs one that handles a list. The stack's
`server.config` leaves the PX4 plugin out, so the two never fight over a camera.

The airframe models are templates. `UAS_FLEET` names one model for each vehicle,
and the entrypoint renders a model and a `streams.conf` for each one. Both need
rendering for the same reason: a camera's field of view and a stream's name
belong to the vehicle, not to the mark. uas13 and uas14 are both `chimera_v2`
and carry different lenses.

### mavlink-router

mavlink-router, plus one small program. One container for each vehicle, all from
one image and one config template.

`keepalive.py` sends a ground station heartbeat into a private endpoint on the
router. PX4 then counts a ground station as present, and the data link loss
failsafe stays quiet on a desk where no ground station is attached. Set
`KEEPALIVE=0` to test that failsafe.

### video-router

MediaMTX. It accepts RTSP, RTMP and RTP, and it serves RTSP, WebRTC and HLS.

It holds no list of vehicle streams. The names carry the UAS number, so the list
would have to change with the fleet, and an unlisted publisher is accepted
anyway.

### onboard

The companion computer for one vehicle: MAVROS, `ds_node`, `tf_loc`, the mission
and survey nodes, the MAVInsight frame tree, and the two domain bridges. `sim`
is the only launch argument that differs from the aircraft.

`UAS_NUM` is the whole identity. The entry point derives the system id, the
`/uas<N>` namespace, ROS domain `60 + N` and the frame prefixes from it.

Two nodes run here only in simulation. `sim_ground_truth` stands in for the
course data that a real exercise sends over the UGV bridge. A second Foxglove
bridge lets an operator attach to one vehicle, which on the aircraft would
collide with the fielded read-only bridge.

### offboard

The ground station, one container for the whole fleet. It runs a listening
MAVROS for each vehicle, its own MAVInsight frame tree, its own `ds_node`
against the low-rate stream, and the viz nodes. Domain 70 throughout.

It rebuilds what the radio link would otherwise have to carry. The contract's
section on what crosses the radio link says what it does not rebuild, and why.

### qgc

QGroundControl, extracted from the released AppImage. The container seeds its
settings on first start, so the vehicle and the video appear without a visit to
the settings pages. Use the `qgc-dev` profile for a Qt toolchain and a source
tree.

## The data paths

### From the camera to a detection on the ground station

```
gz camera sensor
  → gz-transport image topic, named after the model instance
  → gz_video_streamer: NVENC H.265, one full stream and one low-rate stream
  → RTSP publish to video-router
  → ds_node in onboard<N>: decode, infer, track
  → /uas<N>/target_detections, boxes in image space
  → tf_loc: a ray through each box, against the terrain surface
  → /uas<N>/target_locations
  → image_strip drops source_img, the domain bridge carries the rest
  → image_rehydrate on the ground refills the image from its own preview
```

Every arrow is a hop that the aircraft also has.

### From a stick input to a motor

```
QGroundControl
  → UDP 14550 to the router for that vehicle
  → UDP 14545 in, and out to the PX4 link for that instance
  → PX4 mixer
  → gz_bridge: actuator commands on gz-transport
  → Gazebo multicopter motor model
```

### From the rangefinder to a ROS topic

```
gz gpu_lidar on lidar_sensor_link       gz gpu_lidar on gimbal_lidar_link
  → PX4 gz_bridge, uORB instance 0        → gimbal_rangefinder.py
                                          → PX4 MAVLink in, uORB instance 1
  → PX4 MAVLink DISTANCE_SENSOR id 0    → PX4 MAVLink DISTANCE_SENSOR id 1
  → the router                          → the router
  → MAVROS: drone_lidar_200m            → MAVROS: gimbal_lidar_50m
```

PX4 subscribes to a fixed Gazebo topic name for the sensor it reads. The link
must be called `lidar_sensor_link` and the sensor must be called `lidar`. Rename
either one and PX4 reports no rangefinder, with no error message. The comment at
the top of `chimera_common/model.sdf` says so, next to the code it applies to.

The gimbal laser takes the second path because MAVROS names a rangefinder topic
after the sensor id, and the flight code reads id 1. `gimbal_rangefinder.py`
carries the whole argument.

## What this design costs

- **More containers.** Four, plus two for each vehicle. Start time is longer
  than one big image.
- **H.265 loses detail.** The detector sees a compressed frame, as it would on
  the aircraft. For pixel-exact frames, read the Gazebo topic directly and
  accept that the path is then simulation only.
- **Every vehicle costs GPU.** A fleet of four renders ten cameras, encodes
  twenty streams and runs four detectors. Shorten `UAS_FLEET` first when the
  machine cannot keep up.
- **Encode and decode cost latency.** Roughly 80 to 150 ms from the sensor to
  the detector. That is honest for an aircraft with a video downlink. It is
  worse than a shared memory image topic.
- **Two builds of the flight code.** The onboard and offboard images each
  compile the workspace, so a change there is two rebuilds.

Each cost buys the same thing: the simulator and the flight code do not know
about each other.
