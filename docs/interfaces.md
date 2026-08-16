# Interfaces

The exact contract at each module boundary. If you replace a module, this is
the page it must satisfy.

## 1. MAVLink

### The endpoints

`mavlink-hub` runs mavlink-router. It holds one link to the vehicle and four
links to consumers.

| Endpoint | Address | Mode | Who uses it |
|---|---|---|---|
| vehicle | `172.28.0.10:18570/udp` | client | PX4 SITL, or a real autopilot |
| ground station | `172.28.0.14:14550/udp` | client | The QGroundControl container |
| autonomy | `0.0.0.0:14551/udp` | server | MAVROS |
| tools | `0.0.0.0:14552/udp` | server | pymavlink, MAVSDK, mission scripts |
| any | `0.0.0.0:5760/tcp` | server | Anything that prefers TCP |

Host ports: `14550/udp`, `14552/udp` and `5760/tcp`.

Client mode means the router sends to that address. Server mode means the
router waits and then replies to whoever arrives.

### Why the addresses are numbers

mavlink-router parses `Address` as an IP literal. Give it a name and it stops
with `Invalid IP address qgc`. Compose therefore gives every service a fixed
address on the `simnet` network:

| Service | Address | | Service | Address |
|---|---|---|---|---|
| `sim` | 172.28.0.10 | | `qgc-dev` | 172.28.0.15 |
| `mavlink-hub` | 172.28.0.11 | | `ros` | 172.28.0.16 |
| `video-router` | 172.28.0.12 | | `perception` | 172.28.0.17 |
| `message-bus` | 172.28.0.13 | | `xrce-agent` | 172.28.0.18 |
| `qgc` | 172.28.0.14 | | | |

Every other module uses service names, which docker resolves. Only the router,
and the uXRCE-DDS parameter below, need the numbers.

The hub entrypoint also resolves a name if you give it one. That path waits for
the other container to join the network, so the fixed address is the better
default.

### Connecting

MAVROS, already set in the ros container:

```
FCU_URL=udp://:14555@mavlink-hub:14551
```

pymavlink or MAVSDK, from the host:

```python
from pymavlink import mavutil
link = mavutil.mavlink_connection("tcp:localhost:5760")
link.wait_heartbeat()
print(link.target_system, link.target_component)
```

QGroundControl on the host, not in a container: set `HOST_GCS_IP` in `.env` to
your host address. The router then pushes to that address on port 14550 as well.

### The startup handshake

This is the part that surprises people.

PX4 SITL starts its ground station link like this:

```
mavlink start -x -u 18570 -r 4000000 -f
```

That is a UDP server with no configured peer. PX4 records the peer address from
the first packet it receives. It never sends first.

mavlink-router forwards packets. It does not create them. So with no ground
station attached, PX4 waits for a packet and the router has none to send.

`keepalive.py` breaks the deadlock. It connects to the router's own TCP server
on port 5760 and sends a MAVLink 2 HEARTBEAT once a second, as system 255,
component 190. The router forwards it to PX4, PX4 records the router as its
peer, and telemetry flows to every endpoint.

| `KEEPALIVE` | Behavior |
|---|---|
| `1` (default) | Send forever. The vehicle always sees a ground station. |
| `bootstrap` | Stop once vehicle traffic appears. |
| `0` | Do not run. |

Use `bootstrap` to test the data link loss failsafe. With the default, PX4
always sees a ground station, so that failsafe never fires.

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
clean, so `git status` there shows only your own work.

## 2. Video

### The streams

| Name | Source | Resolution | Rate | Bitrate |
|---|---|---|---|---|
| `gimbal` | 3-axis gimbal camera | 1920x1080 | 15 | 2.5 Mbit/s |
| `nadir` | Fixed downward camera | 1920x1080 | 15 | 2.5 Mbit/s |
| `<camera>_annotated` | DeepStream, with boxes | 1920x1080 | 15 | 4 Mbit/s |

The router serves an annotated stream only while the `perception` profile
runs with that camera in `ANNOTATED_STREAMS`, a comma separated list of
camera names. The default is `gimbal`, the stream QGC displays. `1` enables
every camera, `0` disables all of them.

### Reading

| Protocol | URL | Notes |
|---|---|---|
| RTSP | `rtsp://localhost:8554/gimbal` | The default. Use TCP transport. |
| WebRTC | `http://localhost:8889/gimbal` | Open it in a browser. Lowest latency. |
| HLS | `http://localhost:8888/gimbal` | Widest compatibility, about 2 s behind. |

Inside the compose network, use `video-router` in place of `localhost`.

Check a stream from the host:

```bash
ffplay -fflags nobuffer -flags low_delay rtsp://localhost:8554/gimbal
gst-launch-1.0 rtspsrc location=rtsp://localhost:8554/gimbal protocols=tcp \
  ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink
```

### Publishing

A producer publishes with RTSP ANNOUNCE, with RTMP, or as plain RTP.

```bash
# RTSP, which is what gz_video_streamer uses
... ! h264parse ! rtspclientsink protocols=tcp location=rtsp://video-router:8554/gimbal

# RTMP
... ! flvmux ! rtmpsink location=rtmp://video-router:1935/gimbal
```

For plain RTP from a real camera, add a path to
`modules/video-router/mediamtx.yml`:

```yaml
paths:
  nadir_rtp:
    source: udp+rtp://0.0.0.0:8100
```

### Adding a camera

Two files change. Nothing else does.

1. Add the sensor to `modules/sim/scenes/models/x500_recon/model.sdf`. Give it
   an explicit `<topic>`, so the pattern that finds it stays simple.
2. Add a line to `modules/sim/scenes/models/x500_recon/streams.conf`:

```
# name  topic regex  bitrate  fps
thermal  ^/recon/thermal/image$  2000  9
```

Restart `sim`. The stream appears at `rtsp://video-router:8554/thermal`.
MediaMTX accepts an unlisted path through its `all_others` rule, so it needs no
edit. Add a named path there when you want the list to be the record.

## 3. Detections

### The topic

MQTT on `message-bus:1883`, topic `perception/detections`. Port 1883 and the
websocket port 9001 are on the host as well.

### How the payload gets there

Not through DeepStream's MQTT adapter. That adapter is broken against
Mosquitto 2: it creates its client as MQTT 3.1.1 and then publishes with
`mosquitto_publish_v5`, the broker answers with a protocol error, and the whole
pipeline stops. `new-api=1` does not change it.

Instead, `nvmsgconv` writes each payload to a directory, and
`modules/perception/app/payload_forwarder.py` publishes the files and deletes
them. The topic and the payload are identical to what the adapter would have
sent, so nothing downstream can tell. That file records the evidence and what
to do when a DeepStream release fixes the adapter.

### The payload

DeepStream's minimal schema, one message for each published frame:

```json
{
  "version": "4.0",
  "id": "1234",
  "@timestamp": "2026-08-11T20:41:07.123Z",
  "sensorId": "gimbal",
  "objects": [
    "17|412.5|233.1|498.0|401.7|person",
    "23|880.2|190.4|944.6|352.9|person"
  ]
}
```

Each object string is pipe-separated: tracker id, then four box numbers, then
the class name. The four numbers are left, top, right and bottom.

The field order inside that string has changed between DeepStream releases. The
ROS bridge exposes a `bbox_format` parameter with values `ltrb` and `ltwh` so
that a change costs one parameter and no code.

The minimal schema carries no confidence value. `detections_bridge` reports
`1.0` and says so in its source. Switch to `msg-conv-payload-type=0` for the
full schema, which carries more, at one message for each object.

Watch the topic:

```bash
mosquitto_sub -h localhost -t 'perception/#' -v
```

### In ROS

`detections_bridge` republishes the payload as
`vision_msgs/Detection2DArray` on `/perception/<camera>/detections`, routed
by the payload's `sensorId`.

```bash
make ros
ros2 topic echo /perception/nadir/detections
```

## 4. What the ROS stack sees

Three environment variables, set on the `ros` service:

| Variable | Default |
|---|---|
| `FCU_URL` | `udp://:14555@mavlink-hub:14551` |
| `RTSP_BASE` | `rtsp://video-router:8554` |
| `MQTT_HOST` | `message-bus` |

Point those three at a real aircraft and the same stack flies it.

The baseline stack publishes:

| Topic | Type |
|---|---|
| `/mavros/state` | `mavros_msgs/State` |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` |
| `/mavros/rangefinder_pub` | `sensor_msgs/Range` |
| `/camera/gimbal/image_raw/compressed` | `sensor_msgs/CompressedImage`, jpeg |
| `/camera/gimbal/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/nadir/image_raw/compressed` | `sensor_msgs/CompressedImage`, jpeg |
| `/camera/nadir/camera_info` | `sensor_msgs/CameraInfo` |
| `/perception/<camera>/detections` | `vision_msgs/Detection2DArray` |
| `/gimbal/click_mode` | `std_msgs/String`, latched, the current click mode |
| `/gimbal/roi_geojson` | `foxglove_msgs/GeoJSON`, latched, the held region of interest as one point feature, an empty collection when unset |
| `/gimbal/roi_local` | `geometry_msgs/PointStamped`, latched, the same point in the reference frame |
| `/drone/position` | `foxglove_msgs/LocationFix`, the vehicle fix with its compass heading |

One topic goes the other way. `click_to_gimbal` subscribes to
`/foxglove/cursor/click` (`geometry_msgs/PointStamped`, x and y in gimbal
image pixels). Its `click_mode` parameter decides what one click does, and
one parameter means the modes cannot overlap. `roi`, the default, projects
the pixel onto the ground plane and holds the camera on that world point:
the point is published latched on `/gimbal/roi_geojson` and `/gimbal/roi_local`,
and the hold survives vehicle motion. `point` turns the camera onto the
pixel, holds pitch and roll against the horizon, and lets yaw follow the
vehicle heading, the gimbal protocol's default lock flags. `off` ignores
clicks, and a standing hold continues until a new click or the
`/gimbal/center` service. The mode switches at runtime from the layout's
mode buttons, the Foxglove Parameters panel, or the `/gimbal/click_mode/*`
services, and the current mode is published latched on `/gimbal/click_mode`. An honest
MAVLink gimbal stabilizes both behaviors because the flags say so, and
`sim_bridge/roi_tracker.py` emulates them for the simulated gimbal, which
ignores the flags. The `gimbal_convention` parameter picks between them,
and `sim_bridge/click_to_gimbal.py` documents both.

Each camera publishes one encoding: JPEG, at the full stream rate,
published only while something subscribes to it. The Foxglove layout points
its image panels at it, and every in-container consumer that wants pixels,
the ground projector included, decodes it at its own rate. There is no raw
image topic: a raw 1080p stream is about 93 MB/s that nothing needs whole.

The rangefinder topic is `/mavros/rangefinder_pub`, not
`/mavros/distance_sensor/rangefinder_pub`. MAVROS names the topic after the
entry in its sensor config and puts it directly under `/mavros`, not under the
plugin's node. Two settings in
`modules/ros/stacks/baseline/sim_bridge/config/mavros_overrides.yaml` make it
appear at all, and the file says why.

The `CameraInfo` values come from the field of view, not from a calibration.
For the simulated cameras that is exact, because a Gazebo camera is an ideal
pinhole. For a real camera, calibrate and replace it.

## 5. The optional interface: uXRCE-DDS

The `xrce` profile puts PX4 uORB topics straight on the ROS 2 graph. It gives
you `px4_msgs` and the low latency that `px4_ros2_interface_lib` needs.

It also breaks the rule this document is about. A stack that uses it depends on
PX4 directly, and it will not run against an aircraft that offers only MAVLink.

```bash
COMPOSE_PROFILES=ros,perception,xrce docker compose up -d
```

PX4 also needs the agent address, and `UXRCE_DDS_AG_IP` is a signed 32-bit
integer, not a string. The agent sits at 172.28.0.18, so the value is
**-1407451118**. Add it to the `sim` service environment:

```yaml
environment:
  PX4_PARAM_UXRCE_DDS_AG_IP: "-1407451118"
```

For a different address:

```bash
python3 -c 'import ipaddress, struct
addr = "172.28.0.18"
print(struct.unpack("<i", struct.pack("<I", int(ipaddress.IPv4Address(addr))))[0])'
```


## 6. The perception frame tree and localization

The ROS stack turns image-space detections into ground positions. Everything
the 3D view needs is a stock ROS message.

### Frames

```
map                         local ENU, origin where the EKF initialized
 └── base_link              FLU body, published by MAVROS
      ├── gimbal_mount      static
      │    └── gimbal_camera_link          turns with the gimbal
      │         └── gimbal_camera_optical_frame
      └── nadir_camera_link static, looks straight down
           └── nadir_camera_optical_frame
```

A camera *link* uses the body convention, x forward along the view axis. A
camera *optical* frame uses REP 103, x right, y down, z forward. The fixed
rotation between them is rpy (-90, 0, -90). CameraInfo and every projection
assume the optical frame.

MAVROS reports the gimbal attitude in aerospace convention, NED reference
with FRD body axes, while every frame here is ENU and FLU. `scene_tf`
converts it. The conversion is the subject of the gimbal section below,
because getting it wrong produces plausible wrong answers.

### Which camera

`PERCEPTION_CAMERA` in `.env` picks the camera that DeepStream reads and that
detections are localized from. It sets `RTSP_IN` on the perception service and
the optical frame in the ROS stack together, because a mismatch would project
detections through the wrong lens from the wrong frame and produce plausible
wrong answers.

The default is `nadir`. It looks straight down, which suits an overhead search,
and it needs no gimbal pointing.

The anchor follows from that. Looking obliquely, the bottom edge of a box is
where the subject meets the ground. Looking straight down, the box centre is.
Using the wrong one shifts every estimate by a constant.

### Localization mode

`LOCALIZATION_MODE` selects what the detection ray intersects. `plane`, the
default, uses a flat plane latched at the takeoff altitude. `scene` uses the
terrain grid and the building roofs from `worlds/<SCENE>_surface.json`, which
`scenegen build` writes next to the world. Walls are not in the surface on
purpose: a detection on a wall lands on the terrain behind it. Over sloped
ground or a roof, `scene` removes the offset that a flat plane bakes into
every estimate. A missing surface file falls back to the plane, with an
error in the log. Scenes that were built before the surface export need one
`scenegen build` re-run to produce the file.

### Timestamps

`detections_bridge` stamps each `Detection2DArray` with DeepStream's frame
time, taken as the frame enters the pipeline and before inference. The
localizer looks up the transform **at that stamp**, so the pose is the one the
camera actually had.

Measured on this machine, that timestamp trails true capture by about 16 ms
with 6 ms of jitter, and the whole pipeline is quick enough that the detection
stamp is often a few tens of milliseconds *newer* than the newest transform.
The localizer waits, and then clamps to the newest transform when the gap is
under `future_tolerance`, counting how often it does. Past that bound it drops
the detection rather than answer with the wrong pose.

The simulator also publishes a true capture time for every frame on
`video/frames/<stream>`, which is how `scripts/measure-latency.py` measures the
offset. Nothing in the flight path depends on it.

### Ground truth and scoring

Simulation only. `ground_truth` reads the scenario file and publishes where the
targets really are; `detection_scorer` matches estimates against them with two
radii. Within the gate the verdict is TP. Between the gate and the detection
radius it is MISLOCALIZED: the detector saw the target, but the reported
position is not good enough to act on. Beyond the detection radius it is FP,
with one exception: an estimate whose viewing ray from the camera passes
within the gate of an unclaimed target detected that target through the
ground-plane assumption, a roof target being the usual case, and it scores
MISLOCALIZED at any ground distance.
Association is pure geometry over every target, in view or not, so a real
detection overrides what the view alone would say about a target. The view
decides only what counts as a miss: a target is in view when its exact
point projects into the image within the same 100 m limit that truncates
the footprint, and an unclaimed target in view is an FN. So flying away
from the scene does not read as a collapse in recall, and a rooftop target
is a miss only when it is truly in view.

Every camera is localized and scored on its own, under `/perception/<camera>/`
and `/scoring/<camera>/`. Nothing merges them. A camera looking straight down
and a camera looking at the horizon have different error, and one combined
recall figure would describe neither.

| Topic | Type |
|---|---|
| `/ground_truth/markers` | `visualization_msgs/MarkerArray`, the truth bubbles and their name labels |
| `/ground_truth/truth_3d` | `vision_msgs/Detection3DArray` |
| `/ground_truth/geojson` | `foxglove_msgs/GeoJSON`, every target in one message, for the Map panel. Each target is a gate-radius circle in its status color, plus a pin whose tooltip shows the name and altitude |
| `/perception/<camera>/detections_3d` | `vision_msgs/Detection3DArray` with covariance |
| `/camera/<camera>/footprint` | `geometry_msgs/PolygonStamped`, truncated at 100 m |
| `/camera/<camera>/footprint_geojson` | `foxglove_msgs/GeoJSON`, the same outline for the Map panel. The Foxglove layout colors it to match the 3D panel line |
| `/scoring/<camera>/verdicts` | `vision_msgs/Detection3DArray`, each labelled TP, MISLOCALIZED, FP or FN |
| `/scoring/<camera>/markers` | `visualization_msgs/MarkerArray`, a TP as a green dot, a MISLOCALIZED estimate as a yellow cross, an FP as a red cross |
| `/scoring/<camera>/true_positives`, `missed_localizations`, `false_positives` | `sensor_msgs/NavSatFix`, one per verdict, for the Map panel and recordings |
| `/scoring/<camera>/position_error` | `std_msgs/Float64`, meters, one per matched estimate, mislocalized ones included |
| `/scoring/<camera>/recall`, `/scoring/<camera>/precision` | `std_msgs/Float64`, targets placed within the gate |
| `/scoring/<camera>/detection_recall`, `/scoring/<camera>/detection_precision` | `std_msgs/Float64`, the same ratios with MISLOCALIZED counted as found, so they measure the detector alone |

In the 3D view each truth target is a bubble with the scoring gate's radius,
colored by its status across the cameras GROUND_TRUTH_CAMERAS selects, the
gimbal alone by default: green when some camera placed an
estimate within the gate of it, yellow when some camera detected it but
every estimate failed the gate, red when some camera's view covers it but
nothing detected it, grey when no camera sees it and nothing matched it.
Green and yellow need no view: a detection overrides what the view alone
would say about a target. Each camera's estimates
are marks: a green dot within the gate of a target, a yellow cross for a
matched estimate outside it, a red cross for an estimate that matched
nothing. A dot inside a bubble is a hit by construction, because the bubble
radius equals the gate. A missed target gets no mark, only its red bubble.
The image overlay colors its boxes by the same verdicts. Scoring runs on a
clock, so misses appear even while the detector is silent. The colors,
shapes and status rules live in `sim_bridge/verdicts.py`, so the views
cannot drift apart.

The footprint is truncated at 100 m from the camera. A camera near the
horizon reports the near ground it sees, closed by an arc at the limit, and
only a camera that sees no ground publishes nothing. The scorers treat a
stale footprint as no coverage, so a camera pointed at the sky stops
counting its targets as visible. The projected imagery stops at the same
limit, and its cost tracks what is displayed: pixels outside the limit are
masked before their colors are read.

Covariance comes from constants in `detection_localizer.py`, not from a
derivation. `COVARIANCE_DIAGONAL` is a two metre standard deviation in x and
y, and `RANGE_VARIANCE_SCALE` grows that with slant range.

### The gimbal frame, and how it was got wrong twice

The camera frame is built from the gimbal attitude PX4 reports. Three things
about that report are worth knowing, because each one caused a wrong answer
that looked plausible. `docs/px4-simulated-gimbal.md` states the whole
behavior set and the accommodations on one page, for use in other projects.

**PX4 mislabels the frame.** `GZGimbal.cpp` builds the attitude from the gimbal
IMU, which Gazebo reports against the world, then publishes it with
`DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME`. The value is absolute and the label says
it is relative. `scene_tf` therefore ignores the flag, and
`patches/px4-gzgimbal-frame.patch` corrects it at the source.

**MAVROS passes it through untouched.** `gimbal_control.cpp` calls
`mavlink_to_quaternion` and nothing else, so the quaternion arrives in
aerospace convention, NED reference with FRD body axes, and its `base_link_frd`
frame_id is honest. Converting to ENU and FLU needs a rotation on each side,
`NED_TO_ENU * q * FRD_TO_FLU`, because both ends change. Converting a rotation
that is already relative to the body needs only the axis swap, which reduces to
negating y and z. Using either conversion on the wrong quantity produces a
frame that looks reasonable and tracks the aircraft incorrectly.

**The vehicle attitude comes off the left.** An absolute attitude is the vehicle
attitude followed by the gimbal's own rotation, `q_abs = q_vehicle * q_rel`, so

```
q_rel = conj(q_vehicle) * q_abs
```

Dividing on the right instead leaves `q_vehicle * q_rel * conj(q_vehicle)`, a
conjugation: the same rotation through the same angle, about an axis turned by
the vehicle heading. That has a distinctive signature. Conjugation maps identity
to identity, so a **centred gimbal looks perfect at every heading** and the
error appears only once the gimbal moves off centre, where it reads as swapped
axes rather than as a rotation error:

| aircraft at 90 deg yaw, gimbal pitched 30 deg down | roll | pitch | yaw |
|---|---|---|---|
| truth | 0.0 | -30.0 | 0.0 |
| `conj(qv) * qabs` | 0.0 | -30.0 | 0.0 |
| `qabs * conj(qv)` | +30.0 | 0.0 | 0.0 |

**Correct at zero, wrong off zero** means conjugation. It does not mean a bad
axis, and no constant offset can fix it.

One more trap when judging any of this from the aircraft: a gimbal holding an
ROI is earth locked, so its angle relative to the airframe genuinely changes as
the aircraft yaws. That is correct behaviour and it reads exactly like the
fault. Centre the gimbal, or command it in vehicle relative mode, before
deciding.

With `GIMBAL_DIAGNOSTICS=1`, `scene_tf` prints the vehicle heading, the
gimbal heading and their difference once a second. With the gimbal centred,
"gimbal rel body" reads near zero at every aircraft heading. A constant there
is the `GIMBAL_OFFSET_*` mounting value; a value that moves with the aircraft
means a frame handling bug.

### Correcting a frame offset with a fiducial

Some of that error is a frame difference rather than a mistake, and on real
hardware most of it is. Positions land in `map`, the vehicle's own EKF frame,
built from the vehicle's own receiver. Any other frame, a surveyed map or a
second aircraft, is built from a different receiver and sits metres away. The
shapes agree and the origins do not.

That offset is measured once, against a point whose position is known, and then
subtracted. `fiducial_alignment` does it:

| Parameter | Meaning |
|---|---|
| `FIDUCIAL_SURVEYED_LAT/LON/ALT` | Where the fiducial truly is |
| `FIDUCIAL_MEASURED_LAT/LON/ALT` | Where this pipeline put it |
| `FIDUCIAL_ENABLED` | `1` turns the correction on |

The node converts both to local metres, takes the difference, and publishes a
transform from `map` to `fiducial`. With it enabled, `detection_localizer`
publishes in `fiducial`, so every position carries the correction and nothing
downstream does arithmetic of its own.

In simulation neither value has to be flown for. The true position is in the
scenario file, and the measured one can be read off a localization, so both are
simply written down.

**Do not survey a target you also score against.** Fitting the correction to a
scored target and then reporting the error against that same target measures
the arithmetic and nothing else. Use a fiducial no detector is graded on.

Verified by injecting a known offset: a fiducial surveyed 3 m east and 5 m
north of its measured position produced `east +3.00 m, north +5.00 m`, a
transform of `[-3.000, -5.000, 0.000]`, and detections published in
`frame_id: fiducial`.
