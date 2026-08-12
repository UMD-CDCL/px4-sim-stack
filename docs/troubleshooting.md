# Troubleshooting

Start here:

```bash
make preflight     # the host
docker compose ps  # which containers are up
make logs S=sim    # the module you suspect
```

## The GPU

### DeepStream exits at once with a driver error

DeepStream refuses to start when the driver is older than the release needs.

| DeepStream | Needs driver |
|---|---|
| 8.0 | 570.133 |
| 9.0 | 590.48 |

```bash
nvidia-smi --query-gpu=driver_version --format=csv,noheader
```

If your driver is below the number for the image in `DS_IMAGE`, you have two
choices: use the older DeepStream, or update the driver. There is no third
option, and no container flag works around it. A DeepStream 9.0 image on disk
does not mean the driver can run it.

### Gazebo renders on the CPU, and the frame rate is 5

The container did not get the GPU. Check:

```bash
docker compose exec sim nvidia-smi
docker compose exec sim glxinfo -B | grep -i "OpenGL renderer"
```

The renderer must name your GPU. `llvmpipe` means software rendering.

Causes, in order of likelihood:

1. The nvidia container runtime is not registered.
   ```bash
   sudo nvidia-ctk runtime configure --runtime=docker
   sudo systemctl restart docker
   ```
2. `NVIDIA_DRIVER_CAPABILITIES` does not include `graphics`. The compose file
   sets `all`, which covers it. A local override can drop it.
3. You are on Wayland without XWayland GLX.

### The encoder falls back to software

The log line from the streamer says which encoder it chose:

```bash
docker compose logs sim | grep "encoder:"
```

`x264enc (software)` means the `nvcodec` GStreamer plugin did not load. NVENC
needs the `video` driver capability, which `NVIDIA_DRIVER_CAPABILITIES=all`
gives. Check inside the container:

```bash
docker compose exec sim gst-inspect-1.0 nvh264enc
```

Software encoding works. It costs about one core for each 720p stream.

## The display

### "rm: cannot remove '/tmp/.docker.xauth': Is a directory"

Docker creates a missing bind-mount source as a **directory**. Start a
container before the cookie file exists and Docker makes a directory in its
place. Under `/tmp` that directory belongs to root, so removing it needs sudo.

This stack keeps the cookie at `./.xauth` inside the project for that reason.
It belongs to you, and `make x11` clears it whatever shape it is in.

If you still have the old root-owned directory from an earlier version:

```bash
sudo rm -rf /tmp/.docker.xauth
```

Then check that `XAUTH_FILE=./.xauth` in your `.env`.

### No window appears

```bash
make x11
echo $DISPLAY          # must not be empty
ls -l /tmp/.docker.xauth
docker compose exec sim bash -lc 'xdpyinfo | head -3'
```

`make x11` writes the cookie file that the containers mount. Run it again after
you log out and back in, because the cookie changes.

Under Wayland, X11 applications go through XWayland. It works, and GPU
acceleration is less reliable. An X11 session is the tested path.

### QGroundControl logs "Error loading text-to-speech plug-in speechd"

Cosmetic. It means the spoken alerts are off, because the image has no speech
dispatcher daemon. Everything else works. Add `speech-dispatcher` to
`modules/qgc/Dockerfile` and run a daemon in the container if you want the
voice alerts.

### The window opens and stays black

Usually shared memory. `QT_X11_NO_MITSHM=1` is already set, and the containers
run with `ipc: host`. If you removed `ipc: host` in an override, put it back.

## MAVLink

### QGroundControl shows no vehicle

Work along the chain, in order.

1. **Is PX4 running?**
   ```bash
   docker compose logs sim | tail -30
   ```
   Look for `INFO [commander] Ready for takeoff`.

2. **Is the hub talking to PX4?**
   ```bash
   docker compose logs mavlink-hub | grep -i keepalive
   ```
   `vehicle traffic seen, the link is up` means the handshake worked.

3. **Is anything on the wire?**
   ```bash
   docker compose exec mavlink-hub timeout 5 python3 - <<'EOF'
   import socket
   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   s.connect(("127.0.0.1", 5760))
   print("bytes in 3 s:", len(s.recv(4096)))
   EOF
   ```

4. **Is QGroundControl listening?** It binds UDP 14550 and the hub pushes to it.
   Two ground stations cannot both bind 14550. Stop one.

If step 2 never reports vehicle traffic, the keepalive is the thing to look at.
PX4's ground station link is a UDP server that waits for a first packet.
`KEEPALIVE=0` removes the only thing that sends one.

### The vehicle connects, then drops after 10 seconds

Data link loss. PX4's `NAV_DLL_ACT` is 2 in the x500 airframe, which means
return to launch.

With `KEEPALIVE=1`, the default, this should not happen. With
`KEEPALIVE=bootstrap` it happens as soon as you close QGroundControl, and that
is the point of that mode.

### MAVROS reports no connection

```bash
docker compose exec ros bash -lc 'ros2 topic echo /mavros/state --once'
```

`connected: false` means MAVROS cannot reach the hub. Check `FCU_URL`. The
format matters:

```
udp://:14555@mavlink-hub:14551
     ^bind      ^send to
```

The bind port must be free, and the target must be the hub, not PX4.

## Video

### VLC cannot open an RTSP URL, but the browser can

Use `--rtsp-tcp`:

```bash
vlc --rtsp-tcp rtsp://localhost:8554/gimbal
```

Without it, VLC hands `rtsp://` to its SAT>IP module, which speaks a dialect
the server rejects. The server says so plainly in its own log:

```
invalid SETUP path. This typically happens when VLC fails a request,
and then switches to an unsupported RTSP dialect
```

ffmpeg, ffplay, GStreamer and Foxglove all work with no flags. `./px4sim view`
uses ffplay and already passes the right ones.

### First, ask what is actually live

```bash
./px4sim streams
```

```
  PATH                 STATE     READERS  SOURCE
  gimbal               online    0        rtspSession
  gimbal_annotated     offline   0        rtspSource
  nadir                online    0        rtspSession
```

`offline` means nobody is publishing, and a player will get a 404 or a timeout.
`gimbal` and `nadir` come from the simulator, so they stay offline until Gazebo
is up and the vehicle has spawned. `gimbal_annotated` stays offline until the
perception profile runs.

### No stream at rtsp://localhost:8554/gimbal

1. **Is the streamer bound to a camera?**
   ```bash
   docker compose logs sim | grep -E "bound to|waiting for"
   ```
   `waiting for a topic that matches ...` means the pattern found nothing.

2. **Does the topic exist?**
   ```bash
   docker compose exec sim gz topic -l | grep image
   ```
   No image topics means the vehicle has not spawned, or the model has no
   camera. The stock PX4 vehicles other than `x500_recon` have no
   `streams.conf`, so they publish no video by design.

3. **Does the pattern match?** Compare the topic from step 2 against
   `modules/sim/scenes/models/<vehicle>/streams.conf`. The gimbal pattern
   assumes the link is `camera_link` and the sensor is `camera`, which is what
   the upstream gimbal model uses.

4. **Is the router reachable?**
   ```bash
   docker compose logs video-router | tail -20
   ```

### The stream stutters or lags

- Raise `latency` in the DeepStream source config, or lower it for less delay
  and more dropped frames.
- Lower the bitrate in `streams.conf`.
- Drop the frame rate. The nadir camera runs at 15 fps for this reason.
- Check the GPU: `nvidia-smi dmon -s u`. An RTX 3070 with 8 GB runs two 720p
  encoders and TensorRT inference at once, and it has no headroom to spare.

### gimbal_annotated is empty

That path proxies the DeepStream RTSP server. It appears only when
`perception` is up and has a source.

```bash
docker compose ps perception
docker compose logs perception | grep -i rtsp
```

DeepStream prints `Launched RTSP Streaming at rtsp://localhost:8554/ds-test`
when its server is ready.

## Detections

### The MQTT topic stays silent

```bash
mosquitto_sub -h localhost -t 'perception/#' -v
```

In order of likelihood:

1. **`msg-conv-msg2p-new-api` is not 1.** This is the common one. Without it,
   `nvmsgconv` waits for per-object metadata that `deepstream-app` never
   creates, and the payload directory stays empty with no error at all.
2. **The detector found nothing.** Watch `rtsp://localhost:8554/gimbal_annotated`.
   No boxes means no messages, which is correct behavior.
3. **The class filter removes everything.** `config_infer_person.txt` has
   `filter-out-class-ids=0;1;3`, which keeps only people. Comment it out to see
   everything.
4. **The forwarder is not running.** `docker compose logs perception | grep forwarder`.

To watch the raw payloads before they are published:

```bash
docker compose exec perception sh -c 'ls -t /tmp/ds-payloads | head; cat /tmp/ds-payloads/$(ls -t /tmp/ds-payloads | head -1)'
```

### DeepStream logs "The client is not currently connected" and exits

This is the DeepStream 8.0 MQTT adapter, and this stack does not use it. The
adapter creates its client with `mosquitto_new()`, which selects MQTT 3.1.1,
and then publishes with `mosquitto_publish_v5()`. Mosquitto answers with a
protocol error and closes the connection, and the pipeline stops. `new-api=1`
does not change the code path.

`[sink1]` in `gimbal_detector.txt` is therefore disabled, and
`payload_forwarder.py` does the publishing. If you turned that sink back on,
turn it off again.

### The boxes in ROS are in the wrong place

The object string format changed between DeepStream releases. Switch the
format:

```python
# in stack.launch.py
"bbox_format": "ltwh",   # instead of ltrb
```

Compare a raw payload against the annotated video to see which is right.

## The simulator

### PX4 rebuilds on every start

The build output goes to `src/PX4-Autopilot/build/`, on the host. If it
disappears, the bind mount is wrong or the ownership is wrong.

```bash
ls -la src/PX4-Autopilot/build/px4_sitl_default/bin/px4
docker compose exec sim id
```

The user id inside the container must match yours. `make preflight` sets
`HOST_UID` and `HOST_GID` in `.env`, and the image bakes them in, so rebuild the
sim image after a change:

```bash
make build-sim
```

### "World file X declares world name Y"

The `<world name>` inside the SDF must match the file name. PX4 addresses the
world by its declared name. The entrypoint checks this and stops, because the
alternative is a silent hang.

### The vehicle spawns with no rangefinder

PX4 subscribes to one fixed topic:

```
/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan
```

The link must be `lidar_sensor_link` and the sensor must be `lidar`. Check:

```bash
docker compose exec sim gz topic -l | grep scan
docker compose exec sim bash -lc 'gz topic -e -t <the topic> -n 1'
```

Also check `SIM_GZ_EN_LIDAR`, which is 1 by default:

```
pxh> param show SIM_GZ_EN_LIDAR
pxh> listener distance_sensor
```

### A Fuel model does not download

```bash
docker compose exec sim bash -lc 'gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"'
```

The cache is the `sim-fuel` volume. A first download needs network access from
the container. `spawn_scenario.py` reports how many entities it placed and how
many it did not.

## Builds

### mavlink-hub restarts, and logs "Invalid IP address"

mavlink-router parses `Address` as an IP literal and rejects a name. Compose
gives every service a fixed address on `simnet` for that reason. The list is in
[interfaces.md](interfaces.md).

If you changed `PX4_HOST` or `QGC_HOST` to a name, the entrypoint resolves it,
and it waits up to 30 seconds for the other container to join the network. An
address avoids the wait and the failure mode.

After you edit the addresses, recreate the containers. A running container keeps
the address it was created with:

```bash
docker compose up -d --force-recreate
```

### The mavlink-hub build fails on "Dependency systemd not found"

Fixed in this repository by passing `-Dsystemdsystemunitdir` to meson. If you
edited that Dockerfile, put the flag back. The `auto` default makes meson read
the unit directory out of `systemd.pc`, which Debian bookworm does not ship in
any package this image installs.

### One build failure cancels the others

`docker compose build a b c` stops everything when one target fails. Build them
one at a time when you are debugging:

```bash
make build-sim
make build-ros
```

### The image is enormous

The DeepStream image is 33 GB, and that is upstream. The `-samples` tag carries
sample models and videos. `nvcr.io/nvidia/deepstream:8.0-triton-multiarch` is
smaller, and it does not ship the TrafficCamNet model this stack uses as its
default detector. Change `DS_IMAGE` in `.env` after you supply your own model.

## Starting over

```bash
make down                 # containers and network
make clean                # and the named volumes
make clean-src            # and the cloned sources
docker builder prune -f   # and the build cache
```
