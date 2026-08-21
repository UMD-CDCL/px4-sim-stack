# Troubleshooting

Start here:

```bash
./px4sim ui        # the console: containers, vehicles, streams and the GPU at once
./px4sim doctor    # the host
./px4sim status    # which containers are up
./px4sim logs sim  # the service you suspect
```

## The GPU

### A container exits at once with a driver error

DeepStream refuses to start when the driver is older than the release needs.
DeepStream 7.1 needs driver 535.183.

```bash
nvidia-smi --query-gpu=driver_version --format=csv,noheader
```

There are two answers: use an older DeepStream, or update the driver. No
container flag works around it, and an image on disk does not mean the driver
can run it. `./px4sim doctor` reports the driver and says whether it is enough.

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

Docker creates a missing bind-mount source as a **directory**. Start a container
before the cookie file exists and Docker makes a directory in its place. Under
`/tmp` that directory belongs to root, so its removal needs sudo.

This stack keeps the cookie at `./.xauth` inside the project for that reason. It
belongs to you, and `./px4sim x11` clears it whatever shape it is in. To remove
the old root-owned directory from an earlier version, run
`sudo rm -rf /tmp/.docker.xauth`, then check that `XAUTH_FILE=./.xauth` in your
`.env`.

### No window appears

```bash
./px4sim x11
echo $DISPLAY          # must not be empty
docker compose exec sim bash -lc 'xdpyinfo | head -3'
```

`./px4sim x11` writes the cookie file that the containers mount. Run it again
after you log out and back in, because the cookie changes.

Under Wayland, X11 applications go through XWayland. It works, and GPU
acceleration is less reliable. An X11 session is the tested path.

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

2. **Is that vehicle's router talking to PX4?**
   ```bash
   docker compose logs uas11 | grep -i keepalive
   ```
   `vehicle traffic seen, the link is up` means telemetry reaches the router.

3. **Is anything on the wire?**
   ```bash
   timeout 5 python3 - <<'PY'
   import socket
   s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
   s.connect(("127.0.0.1", 5761))   # 5750 + N is uas<N>
   print("bytes in 3 s:", len(s.recv(4096)))
   PY
   ```

4. **Is QGroundControl listening?** It binds UDP 14550 and every router pushes to
   it. Two ground stations cannot both bind 14550. Stop one.

If step 2 never reports vehicle traffic, PX4 is not pushing. Check that `sim`
resolved a router address: its log prints `router 10.200.142.2<N>:14545` as each
vehicle starts.

### The vehicle connects, then drops after 10 seconds

Data link loss. PX4's `NAV_DLL_ACT` is 2 in the x500 airframe, which means
return to launch.

With `KEEPALIVE=1`, the default, this does not happen. With
`KEEPALIVE=bootstrap` it happens as soon as you close QGroundControl, and that
is the point of that mode.

### MAVROS reports no connection

```bash
./px4sim topics 11 | grep state
docker compose exec -e ROS_DOMAIN_ID=71 onboard11 bash -lc \
  '. /opt/ros/humble/setup.bash; . /home/user/ros2_ws/install/setup.bash; \
   ros2 topic echo /uas11/state --once'
```

`connected: false` means the router never reached MAVROS. The two share a
network namespace, so the router sends to `127.0.0.1:14402` and MAVROS binds it.
Check both ends:

```bash
docker compose logs uas11 | head -40      # the router prints its whole config
docker compose logs onboard11 | grep -i fcu
```

A missing companion is the other half of this. Without the `onboard<N>` profile
the routers start, PX4 pushes into them, and nothing ever listens. `./px4sim`
adds that profile from `UAS_FLEET`, so this only happens when compose is called
directly.

## Video

### First, ask what is actually live

```bash
./px4sim streams
```

```
  PATH                 STATE     READERS  SOURCE
  rgb11                online    1        rtspSession
  rgbl11               online    1        rtspSession
  thermal11            online    0        rtspSession
```

`offline` means nobody is publishing, and a player gets a 404 or a timeout.
Every path comes from the simulator, so they stay offline until Gazebo is up and
the vehicle has spawned.

### Some streams are online and others never appear

The log says `pipeline error: Could not encode stream` for the ones that are
missing. A GeForce card allows a small number of encoding sessions at once,
eight on the cards measured here, and every camera costs one session for its
full stream and one for its scaled stream. A fleet of four asks for more than
that, and the streams that ask last are the ones that fail.

Three ways out, cheapest first.

Serve fewer cameras. `UAS_STREAMS` in `.env` says which cameras each vehicle
serves, and `gimbal` is the one the detector reads. Turn one vehicle up without
touching the others:

```bash
UAS_STREAMS="all gimbal gimbal gimbal" ./px4sim start
```

Encode the scaled streams in software, which is the default. They are 640x360
and cost little CPU, and it is the same codec either way, so only the full size
streams take a GPU session. `VIDEO_SCALED_ENCODER=gpu` puts them back.

Lift the limit in the driver. `keylase/nvidia-patch` on GitHub patches
`libnvidia-encode` on the host to remove the session cap. It is not part of this
stack: it changes a file the NVIDIA driver installs, it has to be applied again
after every driver update, and it is the host's decision rather than the
simulator's. It is the only one of the three that keeps every stream on the GPU.

### No stream at rtsp://localhost:8554/rgb11

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
   camera. The stock PX4 vehicles have no `streams.conf`, so they publish no
   video by design.

3. **Does the pattern match?** Compare the topic from step 2 against
   `modules/sim/scenes/models/<model>/streams.conf`. The regex holds the Gazebo
   model instance, which is `uas<N>_<N-1>`.

4. **Is the router reachable?**
   ```bash
   docker compose logs video-router | tail -20
   ```

### QGroundControl shows no video

Use the address that works inside its container. Everything this stack prints
uses host addresses, and inside QGroundControl `localhost` is QGroundControl.

The container forwards its own loopback to the video router, so both of these
work:

```
rtsp://localhost:8554/rgb11
rtsp://video-router:8554/rgb11
```

The settings are seeded with the second form on first start, so a fresh
container needs no configuration at all.

```bash
docker compose logs qgc | grep -iE "video|gst|rtsp"
```

`Decoding started` and `resized. New resolution: 1280 x 720` mean the video is
running. `Streaming did not start` means it never reached the server.

### VLC cannot open an RTSP URL, but the browser can

Use `--rtsp-tcp`:

```bash
vlc --rtsp-tcp rtsp://localhost:8554/rgb11
```

Without it, VLC hands `rtsp://` to its SAT>IP module, which speaks a dialect the
server rejects. The server says so plainly in its own log:

```
invalid SETUP path. This typically happens when VLC fails a request,
and then switches to an unsupported RTSP dialect
```

ffmpeg, ffplay, GStreamer and Foxglove all work with no flags. `./px4sim view`
uses ffplay and already passes the right ones.

### The stream stutters or lags

- Lower the bitrate in `streams.conf`.
- Drop the frame rate, or shorten `UAS_FLEET`. A fleet of four is twenty
  encodes and four detectors.
- Check the GPU: `nvidia-smi dmon -s u`.

## Detections and localization

### The detector produces nothing

```bash
docker compose logs onboard11 | grep -i ds_pipeline
```

The first run builds a TensorRT engine next to the ONNX file, which takes 1 to 3
minutes and says so. It stays in `ONBOARD_MODEL_DIR` afterwards.

After that, check the source. `ds_node` reads the full gimbal RGB stream:
`rgb<N>` on a v3 and `pilot<N>` on a v2. `UAS_FLEET` and `UAS1<N>_MODEL` decide
which, and a disagreement with the simulator is silent: the model publishes one
name while the detector opens the other, so no frame ever arrives.

### The boxes are drawn, but nothing is localized

`tf_loc` looks the frame tree up at the message stamp. The tree must be
continuous and must cover `tf_lookup_timeout_duration_sec` past the newest
stamp, so a gap in MAVROS telemetry stops localization while the picture keeps
moving.

```bash
docker compose logs onboard11 | grep -i tf_loc
```

The frames section of `uas-contract.md` holds the four rules `tf_loc` depends
on. Frame
handling is where a plausible wrong answer comes from, and
[px4-simulated-gimbal.md](px4-simulated-gimbal.md) says how the gimbal frame has
been got wrong before.

### A footprint does not hold the detections inside it

The footprint node and `tf_loc` must meet the same ground. Both read the terrain
surface that the entry point links into `TERRAIN_DIR`:

```bash
docker compose logs onboard11 | grep terrain
```

`no surface for scene` means every ray meets the flat plane instead. Run
`./px4sim genscene build --name <scene>` to write `worlds/<scene>_surface.json`.
A hand-written world has no surface, and the flat plane is the answer for it.

The other cause is the frame convention. `tf_loc` localizes through the camera
BODY frame, and a node that projects through the optical leaf must use standard
optical math. See "Body frames and optical frames" in
[interfaces.md](interfaces.md).

### The ground station shows no detections

The vehicle publishes `target_locations/for_air` with the image removed, the
domain bridge carries it, and `image_rehydrate` refills the image on the ground.
Check the bridge first:

```bash
docker compose logs onboard11 | grep -i domain_bridge
docker compose logs offboard | grep -i domain_bridge
```

Both ends must declare the same QoS. A best effort reader matches a reliable
writer and then discards every repair, which reads as a link that carries
nothing.

Every container that carries `cdcl_umd_msgs` must run ROS 2 Humble. Jazzy adds a
field to `sensor_msgs/Range`, which sits inside `TargetBoxArray` before the box
array, so a mixed pair decodes an empty box list and reports no error.

## The simulator

### The vehicle is on its side, or will not climb

A test that ends badly leaves the vehicle tipped over. Nothing reports it: every
ROS topic still carries data, the graph is complete, and the next takeoff simply
never climbs.

```bash
./px4sim fly 11              # into the air from whatever state it is in
./px4sim fly 11 25 clean     # respawn the world first, whatever the state
./px4sim place               # put the fleet back upright, and stop there
```

`fly` measures how far the vehicle leans, respawns it if that is more than 20
degrees, takes the gimbal back, then climbs. It retries once with a respawn if
it will not.

`place` restarts the simulator, because nothing smaller is honest: PX4 owns the
vehicle's dynamics and drives the Gazebo model every step, so moving the entity
underneath it is overwritten within a step.

### The gimbal ignores every command, and the state says it is in control

The autopilot forgets who holds the gimbal when it reboots, and the manager
status still names the old owner. `gimbal/state` reads `primary=<vehicle>` while
nothing moves.

```bash
ros2 topic pub --once /uas11/reassert_gimbal_cmd std_msgs/msg/Float32 "{data: -361.0}"
```

`./px4sim fly` sends that on every takeoff, so it does not arise there.

### A click moves the camera in pitch, and never turns it

The mount yaws, so a pixel left or right of the boresight is a yaw command like
any other. A camera that answers such a click with a small pitch move is a yaw
setpoint that something clamped on the way to the joint. Ask for an azimuth and
read it back:

```bash
./px4sim uas 11 gimbal -45 --yaw 30
```

If the reported azimuth stays at zero, one of the three places that state the
travel of that axis disagrees with the others: the yaw joint limits in
`modules/sim/scenes/models/gimbal/model.sdf`, the yaw range and capability
flags in `patches/px4-gzgimbal-lock.patch`, and `yaw_angle_min` and
`yaw_angle_max` in the flight code parameters. `./px4sim check` says whether
the patch reached the source at all, and the gimbal node logs its own limits
when it starts:

```bash
./px4sim logs --since 1h onboard11 | grep yaw_limits
```

### PX4 rebuilds on every start

The build output goes to `src/PX4-Autopilot/build/`, on the host. If it
disappears, the bind mount is wrong or the ownership is wrong.

```bash
ls -la src/PX4-Autopilot/build/px4_sitl_default/bin/px4
docker compose exec sim id
```

The user id inside the container must match yours. `./px4sim doctor` sets
`HOST_UID` and `HOST_GID` in `.env`, and the image bakes them in, so rebuild the
sim image after a change:

```bash
./px4sim build sim
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

The gimbal rangefinder is the other half. It arrives as sensor id 1 only when
PX4 already owns id 0, so `gimbal_rangefinder.py` waits for that and logs while
it waits.

### A Fuel model does not download

```bash
docker compose exec sim bash -lc 'gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person"'
```

The cache is the `sim-fuel` volume. A first download needs network access from
the container. `spawn_scenario.py` reports how many entities it placed and how
many it did not.

## Builds

### A router restarts, and logs "Invalid IP address"

mavlink-router parses `Address` as an IP literal and rejects a name. Compose
gives every service a fixed address on `simnet` for that reason. The list is in
[interfaces.md](interfaces.md).

After you change the addresses, recreate the containers. A running container
keeps the address it was created with:

```bash
docker compose up -d --force-recreate
```

### The mavlink-router build fails on "Dependency systemd not found"

Fixed in this repository by passing `-Dsystemdsystemunitdir` to meson. If you
edited that Dockerfile, put the flag back. The `auto` default makes meson read
the unit directory out of `systemd.pc`, which Debian bookworm does not ship in
any package this image installs.

### The onboard or offboard build fails on a missing package

Both images build the workspace at `ROS2_WS_DIR` through a named build context.
A missing checkout, or a checkout without `src/5g_drone`, fails there. `./px4sim
doctor` reports it before the build does.

### One build failure cancels the others

`docker compose build a b c` stops everything when one target fails. Build them
one at a time when you are debugging:

```bash
./px4sim build sim
./px4sim build onboard
```

### The image is enormous

The DeepStream image is upstream, and the `-samples` tag carries sample models
and videos. The onboard and offboard Dockerfiles take `DS_IMAGE_FLAVOR`, so
`triton-multiarch` builds a smaller image. Pass it in `compose.yaml`, under the
build `args` of those two services.

### The ground station hears nothing, and a node logs open_and_lock_file failed

```
[RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port25031:
open_and_lock_file failed
```

Fast DDS keeps its shared memory in `/dev/shm`, and these containers share the
host's, so a segment outlives the container that made it. A few hundred pile up
over a day of restarts until a new participant cannot lock a port, and the
ground station stops hearing anything while every container still looks healthy.

```bash
ls /dev/shm | grep -c fastrtps
```

`./px4sim stop` releases the ones nobody has open. To clear them by hand:

```bash
./px4sim stop
```

## Starting over

```bash
./px4sim stop             # containers and network
./px4sim clean            # and the named volumes
./px4sim clean-src        # and the cloned sources
docker builder prune -f   # and the build cache
```
