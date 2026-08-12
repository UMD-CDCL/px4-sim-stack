# Architecture

## The rule

A module may depend on an address. A module may not depend on another module's
internals.

There are three addresses in this system: a MAVLink endpoint, an RTSP URL and an
MQTT topic. Everything else is private to a module.

That rule is the reason the layout looks the way it does, and it is the reason
you can put a real aircraft where the simulator is and change nothing else.

## Why these boundaries

### Video leaves the simulator as H.264, not as a topic

The obvious design puts Gazebo and ROS in one container and bridges images with
`ros_gz`. It is less code on day one and it costs you later:

- The autonomy stack then knows that Gazebo exists. On the real aircraft it does
  not, so the code paths diverge exactly where you stop testing them.
- `ros_gz` on Jazzy pulls in ROS-vendored Gazebo packages. The simulator needs
  upstream Gazebo. Putting both in one image is a version fight with no winner.
- A raw 1280x720 image topic is 2.6 MB per frame. H.264 at 4 Mbit/s is 17 kB.

So the simulator encodes, and the autonomy stack decodes. That is what happens
on the aircraft, and now it is what happens on the desk.

### MAVLink goes through a router, not point to point

PX4 SITL opens four MAVLink links, and a consumer can attach to any of them
directly. That works, and it means every consumer needs to know which port does
what.

A router gives one endpoint per role and hides the vehicle. It is also what runs
on a real companion computer, so the shape of the wiring does not change when
the vehicle does.

### Detections use MQTT, not ROS

DeepStream is a C and GStreamer application on an NVIDIA base image.
ROS 2 Jazzy is a different base image with a different toolchain. Making one
depend on the other means one container, one release schedule and one upgrade
that breaks both.

MQTT costs one small broker. In exchange, perception and autonomy upgrade
separately, and a stack that does not want detections simply does not subscribe.

### The simulator holds PX4 and Gazebo together

These two could be separate containers. They are not, because PX4 talks to
Gazebo over gz-transport, which finds its peers with UDP multicast. Multicast
across a docker bridge works until it does not, and debugging it is a bad use of
an afternoon.

They belong together anyway. Both are the vehicle.

## What is inside each module

### sim

| Piece | Job |
|---|---|
| Gazebo Harmonic server | Physics, rendering, sensors |
| PX4 SITL | Firmware, in standalone mode against the running world |
| `gz_video_streamer` | Gazebo camera topics to H.264 |
| `spawn_scenario.py` | Puts the targets in the world at run time |

PX4 runs in standalone mode. The entrypoint starts Gazebo first, then PX4
attaches to the world that already exists. That order matters: it lets the
entrypoint choose the world and the model directory, which PX4 would otherwise
overwrite from its own generated `gz_env.sh`.

`gz_video_streamer` is a program in this repository. PX4 ships a similar
plugin, and that one binds to the first camera it finds. This stack has more
than one camera, so it needs one that handles a list. The stack's
`server.config` leaves the PX4 plugin out, so the two never fight over the same
camera.

### mavlink-hub

mavlink-router, plus one small program.

The small program exists because of a deadlock. PX4's ground station link is a
UDP server that learns its peer from the first packet it receives. It never
speaks first. mavlink-router only forwards, so it never speaks first either.
With no ground station attached, both wait.

`keepalive.py` sends a ground station heartbeat into the router's own TCP
server. The router forwards it, PX4 learns the peer, and telemetry starts.
Set `KEEPALIVE=bootstrap` to stop once traffic flows, which is what you want
when you test the data link loss failsafe.

### video-router

MediaMTX. It accepts RTSP, RTMP and RTP, and it serves RTSP, WebRTC and HLS.

It also pulls the annotated stream out of the perception container and
republishes it. That keeps one address space for video: a consumer does not
need to know that one stream comes from Gazebo and another from DeepStream.

### ros

ROS 2 Jazzy, MAVROS, and one directory of stacks.

The image has no Gazebo, no `ros_gz`, no `px4_msgs` and no uXRCE-DDS client.
That is deliberate. The stack sees the three addresses and nothing else.

A stack is a directory with a `stack.launch.py` at its root. The entrypoint
builds the directory with colcon and launches that file. Switching stacks is one
environment variable.

### perception

DeepStream 8.0, driven by a config file. There is no application code.

`deepstream-app` reads RTSP, runs the detector, draws boxes, serves the result
as RTSP, and publishes the detections over MQTT. Every one of those is a config
section. Python bindings would add a dependency on a DeepStream release and buy
nothing.

One line in that config is easy to get wrong and silent when it is:
`msg-conv-msg2p-new-api=1`. Without it, `nvmsgconv` waits for per-object
metadata that `deepstream-app` never produces, and the MQTT topic stays empty
with no error.

### qgc

QGroundControl, extracted from the released AppImage. The container seeds its
settings on first start so that the vehicle and the video both appear without
a visit to the settings pages.

Use the `qgc-dev` profile for a Qt toolchain and a source tree.

## The data paths

### From the camera to a detection

```
gz camera sensor
  → gz-transport image topic
  → gz_video_streamer: NVENC H.264
  → RTSP publish to video-router
  → DeepStream: decode, infer, track
  → MQTT payload, and an annotated RTSP stream
  → detections_bridge in ROS: vision_msgs/Detection2DArray
```

Every arrow is a network hop that a real system also has. Nothing here is a
simulation shortcut.

### From a stick input to a motor

```
QGroundControl
  → UDP 14550 to mavlink-hub
  → UDP 18570 to PX4
  → PX4 mixer
  → gz_bridge: actuator commands on gz-transport
  → Gazebo multicopter motor model
```

### From the rangefinder to a ROS topic

```
gz gpu_lidar sensor on lidar_sensor_link
  → PX4 gz_bridge: distance_sensor uORB message
  → PX4 MAVLink DISTANCE_SENSOR
  → mavlink-hub
  → MAVROS: /mavros/distance_sensor/...
```

PX4 subscribes to a fixed Gazebo topic name for that sensor. The link must be
called `lidar_sensor_link` and the sensor must be called `lidar`. Rename either
one and PX4 reports no rangefinder, with no error message. The comment at the
top of `x500_recon/model.sdf` says so, next to the code it applies to.

## What this design costs

Be clear about the trade:

- **More containers.** Six by default. Start time is longer than one big image.
- **H.264 loses detail.** The detector sees a compressed frame, as it would on a
  real aircraft. If you need pixel-exact frames for an experiment, read the
  Gazebo topic directly and accept that the path is now simulation only.
- **Encode and decode cost latency.** Roughly 80 to 150 ms from the sensor to
  DeepStream. That is honest for an aircraft with a video downlink. It is worse
  than a shared memory image topic.
- **MQTT has no types.** A schema change breaks the consumer at run time, not at
  compile time. One file, `detections_bridge.py`, holds that risk.

Each cost buys the same thing: the modules do not know about each other.
