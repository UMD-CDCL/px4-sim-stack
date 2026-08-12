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
| `gimbal` | 3-axis gimbal camera | 1280x720 | 30 | 4 Mbit/s |
| `nadir` | Fixed downward camera | 1280x720 | 15 | 2.5 Mbit/s |
| `gimbal_annotated` | DeepStream, with boxes | 1280x720 | 30 | 4 Mbit/s |

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
`vision_msgs/Detection2DArray` on `/perception/detections`.

```bash
make ros
ros2 topic echo /perception/detections
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
| `/camera/gimbal/image_raw` | `sensor_msgs/Image`, rgb8 |
| `/camera/gimbal/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/nadir/image_raw` | `sensor_msgs/Image`, rgb8 |
| `/camera/nadir/camera_info` | `sensor_msgs/CameraInfo` |
| `/perception/detections` | `vision_msgs/Detection2DArray` |

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
