# The front doors

Every task in this stack has one door. This page names each door, says where it
runs, gives the exact command line, names what it reads and what it refuses,
and works one example.

| Door | Machine | For |
|---|---|---|
| [`./px4sim`](#px4sim) | t500 and uas1 | build, start, read and stop the stack |
| [`./px4sim ui` and `state`](#the-console-and-the-state-report) | t500 and uas1 | the same readings in a console, or as JSON |
| [`make`](#the-makefile) | t500 and uas1 | the old target names, forwarded to `./px4sim` |
| [`deploy_onboard.sh`](#deploy_onboardsh) | uas1 | put this stack on an aircraft |
| [`onboard.service`](#onboardservice) | uas1 | start the aircraft stack at boot |
| [`.bash_aliases`](#the-aircraft-aliases) | uas1 | the hand versions of the aircraft commands |
| [`setup_git_server.sh`](#setup_git_serversh) | t500 and uas1 | move code to a machine with no internet |
| [`fetch_models.py`](#fetch_modelspy) | t500 and uas1 | the detector and classifier artifacts |
| [`ds-select.sh`](#ds-selectsh) | t500 and uas1 | the DeepStream release and the TensorRT |

The stack has three worlds. `.env` says which one this machine is.

| World | `COMPOSE_PROFILES` | `UAS_BASE` | Services | Vehicles |
|---|---|---|---|---|
| simulator | `sim,offboard` | `10` | `sim`, `uas1<N>`, `onboard1<N>`, `offboard`, `ground-router`, `video-router`, `qgc` | uas11 to uas19 |
| ground station | `ground` | `0` | `ground` | uas1 to uas9, on their own aircraft |
| aircraft | `aircraft` | `0` | `onboard` | this machine alone |

---

## `./px4sim`

**For.** Every day-to-day action on the stack: the host check, the image build,
the start and the stop, the readings, and one vehicle's nodes.

**Where.** `/home/user/px4-sim-stack` on t500, and `/home/user/px4-sim-stack`
on uas1. The same script, the same commands, three worlds.

**What it reads.**

| Source | What it takes |
|---|---|
| `./.env` | every key. `COMPOSE_PROFILES` and `UAS_BASE` name the world |
| the environment | `SCENE`, `SCENARIO`, `UAS_FLEET`, `UAS_STREAMS`, `AIRFRAME`, `COMPOSE_PROFILES`, `GZ_GUI`, `SIM_SPEED_FACTOR` and `VIDEO_SCALED_ENCODER` beat `.env` for one run |
| `/etc/environment` | `UAS_NUM` on the aircraft, through compose. `.env` never carries it |
| `scripts/ds-select.sh` | the DeepStream release, the image, the tag and the TensorRT, resolved on every run |
| `.origin.env` | the coordinates of the scene and the scenario, in the simulator |

`UAS_BASE` is not in the override list, so a world is changed in `.env` and not
on the command line.

**What it refuses.**

1. A `.env` where `COMPOSE_PROFILES` and `UAS_BASE` describe different worlds.
   Every command stops, except `help`, `doctor`, `check`, `x11`, `setup`,
   `stop`, `clean`, `clean-src` and `nuke`.
2. On a real machine, the commands that fly the simulator: `core`, `fly`,
   `place`, `fiducial`, `reset`, `px4`, `console`, `snap`, and `uas <N> arm`,
   `takeoff`, `land` and `goto`. Each one exits 1 and prints `Nothing was
   sent.`
2b. On an aircraft, the commands that work on a scene: `scene`, `scenario` and
   `genscene`. A scene is map data, so the ground station builds and selects
   one the way the simulator does. The aircraft reads the scene it is given.
3. On a real machine, the commands that maintain the simulator: `setup`,
   `clean-src`, `nuke`, `fleet add` and `fleet remove`. Each one exits 1 and
   prints `Nothing was changed.`
4. `router` on a real machine. The real router is native:
   `systemctl status mavlink-router`.

`verify` is not refused. It runs the stages this machine's world can answer and
says which ones those are: `ground` and `foxglove` on a ground station, which
holds no vehicle of its own, and `units`, `vehicle` and `foxglove` on an
aircraft, which is the vehicle and serves its own bridge.

`./px4sim help` prints the whole list and names the world this machine is.
`./px4sim check` reads that text back and fails on a command the help does not
name, so the list cannot drift.

### The simulator world, on t500

`.env` carries `COMPOSE_PROFILES=sim,offboard`, `UAS_BASE=10`, a `SCENE` and a
`SCENARIO`.

```bash
cd /home/user/px4-sim-stack
./px4sim doctor            # driver, docker, GPU runtime, X11, disk, ports
./px4sim setup             # clone PX4 and QGroundControl into ./src
./px4sim build             # ros-base, then every image
./px4sim start             # sim, uas11, onboard11, offboard, ground-router,
                           # video-router and qgc
./px4sim status            # what is up, and the addresses
./px4sim fly 11 20         # respawn uas11 and take it to 20 m
./px4sim view 11           # play rgb11
./px4sim verify            # every stage. The last bench run was 71 checks
./px4sim stop
```

The first `start` builds PX4 inside the `sim` container. That takes 10 to 20
minutes, once, because the output lands in `./src/PX4-Autopilot` on the host.

### The ground world, on t500

`.env` carries `COMPOSE_PROFILES=ground`, `UAS_BASE=0`, `GROUND_DOMAIN=60`,
`RTSP_BASE=rtsp://127.0.0.1:8554`, and the `SCENE` and `SCENARIO` of the site
it stands on: `SCENE=uroc` and `SCENARIO=uroc_casualties` at the UMD test site.
Those draw the terrain, the buildings and the satellite map in the Foxglove 3D
panel, cast the camera footprint and the live view at that surface, and score
against the targets the scenario names. Both empty is a bench outside any
surveyed site. The native `lcam.service`, `mavlink-router.service` and
`git-daemon.service` keep running. No `ground-router`, `video-router` or `qgc` container starts, and
the fielded QGroundControl stays native.

```bash
cd /home/user/px4-sim-stack
./px4sim doctor            # also lcam, mavlink-router, 14402/udp and 8765/tcp
./px4sim build             # ros-base and the ground image
./px4sim start             # one container, `ground`, on the host network
./px4sim uas 1 status      # the vehicle, through the ground station
./px4sim streams           # the lcam mounts, asked for by name
./px4sim view 1            # rgbl1
./px4sim layout            # render the Foxglove layout for uas1
./px4sim verify            # the ground and foxglove stages
./px4sim stop
```

`./px4sim uas`, `probe` and `topics` run inside the `ground` container, on
domain 60. `./px4sim foxglove 1` reaches the vehicle's own bridge at
`ws://10.200.142.61:8765`, because a bridge belongs to a machine.

**To try this world on a simulator laptop**, copy `.env` first, flip the six
keys, and put the file back at the end:

```bash
cp .env .env.backup
# COMPOSE_PROFILES=ground UAS_BASE=0 GROUND_DOMAIN=60
# RTSP_BASE=rtsp://127.0.0.1:8554 SCENE= SCENARIO=
./px4sim start && ./px4sim state && ./px4sim stop
mv .env.backup .env
```

### The aircraft world, on uas1

`.env` carries `COMPOSE_PROFILES=aircraft`, `UAS_BASE=0`, the whole
`UAS_FLEET`, the `SCENE` and `SCENARIO` the ground station carries,
`SIMNET_PREFIX=172.28.0` and
`ONBOARD_LENS_DEVICE`. `UAS_NUM` comes from `/etc/environment`. The native
`rcam.service` and `mavlink-router.service` keep running.

```bash
cd /home/user/px4-sim-stack
./px4sim doctor            # also rcam, its sockets, the clock, the power mode,
                           # the lens and the perception_models group
./px4sim build ros-base onboard
./px4sim start             # one container, `onboard`, on the host network
./px4sim logs onboard
./px4sim uas 1 status
./px4sim zoom 1 wide
./px4sim verify            # units, vehicle and foxglove: the aircraft's own
./px4sim stop
```

**Docker needs sudo here until the operator runs `sudo usermod -aG docker
user` and logs in again.** Until then every call takes this shape, which keeps
`HOME` and `UAS_NUM`:

```bash
echo <the drone password> | sudo -S -p x env HOME=/home/user UAS_NUM=1 ./px4sim start
```

`./px4sim doctor` and `scripts/fleet.sh` hand `.env` and the log directories
back to the operator after a run under sudo, so the uid 1000 container can
still write them.

The boot unit does not need that group. `onboard.service` carries
`SupplementaryGroups=docker` and works before the operator's next login.

---

## The console and the state report

**For.** Every reading at once, and a key for every command.

**Where.** Any of the three worlds.

```bash
./px4sim ui                # the console
./px4sim state             # one JSON object
./px4sim state --watch     # one object for each line, about every two seconds
```

The console starts nothing of its own. Every action runs `./px4sim ...`, so it
can do what the prompt can do and nothing more.

**Keys.** Tab moves between the panes and the arrow keys pick a row. Enter
opens what can be done to that row. `:` types any px4sim command, `?` lists
every action with the command it runs, esc stops a command and `q` leaves.

**What each world offers.** The console offers the keys this world accepts, and
no others. On a ground station and on an aircraft the eighteen actions that fly
or maintain the simulator are not in the menus, not on a key and not in `?`:
`fleet add`, `fleet remove`, `place`, `scenario` twice, `scene`, `fiducial`,
`console`, `origin`, `takeoff`, `land`, `fly`, `arm`, `snap` twice, `goto`,
`px4` and the router log. A ground station also loses the companion log and the
companion shell, because its companion flies on the aircraft.

**What the readings come from.**

| Reading | Source |
|---|---|
| A container | the container engine's own state |
| A video path | the MediaMTX API in the simulator, one probe by name on a real machine |
| A vehicle | MAVLink on `tcp://localhost:5761` upwards, or on the native router's `127.0.0.1:5760` |
| A Foxglove bridge | a connection to the port, opened and closed |
| A native service | `systemctl is-active`, on a ground station and on an aircraft |
| The GPU | `nvidia-smi`. A Jetson has none, and the field is empty |

`state` reports `config.world` as `simulator`, `ground` or `aircraft`, and
`config.units` as the native services this machine boots. The simulator holds
every service in a container, so its unit list is empty.

---

## The Makefile

**For.** The target names people already type.

**Where.** Beside `./px4sim`, on either machine.

Every target forwards to the front door and does no work of its own.

| Target | Runs |
|---|---|
| `make preflight` | `./px4sim doctor` |
| `make bootstrap` | `./px4sim setup` |
| `make build`, `make build-onboard` | `./px4sim build [service]` |
| `make up`, `make up-core`, `make down` | `./px4sim start`, `core`, `stop` |
| `make restart S=sim` | `./px4sim restart sim` |
| `make ps`, `make ui`, `make state` | `./px4sim status`, `ui`, `state` |
| `make logs S=onboard` | `./px4sim logs onboard` |
| `make onboard N=13`, `make router N=13` | `./px4sim onboard 13`, `router 13` |
| `make topics N=11` | `./px4sim topics 11` |
| `make scene SCENE=uroc` | `./px4sim scene uroc` |
| `make genscene ARGS="..."` | `./px4sim genscene ...` |
| `make check`, `make lint-docs` | `./px4sim check`, `./scripts/lint-docs.sh` |

`make help` prints `./px4sim help`, so one list serves both.

---

## `deploy_onboard.sh`

**For.** Putting this stack on an aircraft for the first time.

**Where.** `~/chimera-deploy` on the Orin, as `user`, after `deploy.sh`.

```bash
cd ~/chimera-deploy
./remote/deploy_onboard.sh                    # install the unit, do not enable it
ENABLE_BOOT_UNIT=1 ./remote/deploy_onboard.sh # and enable it
```

**What it reads.** `UAS_NUM` from `/etc/environment`, which `deploy.sh` writes.
`SERVER_IP` (10.200.142.60), `GIT_PORT` (9418), `STACK_BRANCH` and `WS` come
from the environment and have defaults.

**What it does, in order.** Each step examines the machine first, so a second
run changes nothing.

1. Adds `user` to group `docker`.
2. Renames `~/ros2_ws/src/umd_uas` to `~/ros2_ws/src/5g_drone`, which is the
   name the container build and `fetch_models.py` use.
3. Clones `git://10.200.142.60:9418/px4-sim-stack.git` into `~/px4-sim-stack`,
   and makes the four log directories.
4. Writes `.env` from `.env.example` with the aircraft keys, and finds the SCF4
   lens under `/dev/serial/by-id/usb-Kurokesu_*`.
5. Runs `fetch_models.py resolve --link` and `check --role onboard`.
6. Installs `remote/onboard.service` into `/etc/systemd/system`.

**What it refuses.** A machine with no `UAS_NUM`, a `UAS_NUM` outside 1 to 9,
a missing `~/ros2_ws/src/5g_drone`, and a machine `fetch_models.py` knows no
engine group for. It leaves an existing `.env` alone.

---

## `onboard.service`

**For.** The aircraft stack at boot.

**Where.** `/etc/systemd/system/onboard.service` on the Orin, from
`chimera-deploy/remote/onboard.service`.

```bash
sudo systemctl enable --now onboard    # start it and keep it at the next boot
sudo systemctl restart onboard         # after a rebuild
sudo systemctl status onboard
journalctl -u onboard -n 40
sudo systemctl disable --now onboard   # give the stack back to the prompt
```

**What it reads.** `/etc/environment` as its `EnvironmentFile`, so `UAS_NUM`
and `ROS_DOMAIN_ID` reach it. `HOME=/home/user` is set in the unit.
`WorkingDirectory` is `/home/user/px4-sim-stack`.

**How it runs.** `Type=oneshot` with `RemainAfterExit=yes`. It is ordered after
`docker.service`, `rcam.service`, `mavlink-router.service` and
`time-sync.target`. `ExecStartPre` removes any container a power cut left, then
waits up to three minutes for a clock step with `chronyc waitsync`. Both
`ExecStartPre` lines carry a leading dash, so neither can hold the boot.
`ExecStart` is `./px4sim start` and `ExecStop` is `./px4sim stop`.

`SupplementaryGroups=docker` gives the unit the docker socket whether or not
the login user is in that group.

`After=` names `multi-user.target` first, and that line is load bearing. The
aircraft's own `mavlink-router.service` is ordered after `multi-user.target`.
Without the line, systemd adds the opposite order for this unit, the three make
a cycle, and systemd breaks a cycle by deleting a start job: this one. The unit
then reads `enabled` and `inactive` after every boot, with nothing in its
journal to say why.

**What it costs.** A `systemctl start` always cycles the container, because
`ExecStartPre` stops it first. Detection is off on a fresh container. Turn it
on again with `/ds/mode/toggle_detection` and `continuous_detection_cmd`.

A hand `./px4sim stop` leaves the unit active with no container. `./px4sim`
says so, and `sudo systemctl restart onboard` brings it back.

---

## The aircraft aliases

**For.** The hand versions of the aircraft commands.

**Where.** `chimera-deploy/remote/.bash_aliases`, copied to `~/.bash_aliases`
on the Orin.

| Alias | Runs |
|---|---|
| `onboard` | `(cd ~/px4-sim-stack && ./px4sim start)` |
| `onboard-logs` | `(cd ~/px4-sim-stack && ./px4sim logs onboard)` |
| `onboard-native` | `ccb && ros2 launch umd_uas onboard.launch.py uas:=${UAS_NUM:?}` |
| `rs`, `ws`, `cdr`, `ccb` | source ROS, source the workspace, go to it, build it |
| `rdom [N]` | read or set `ROS_DOMAIN_ID` |
| `bag` | `ros2 bag record` of the topics in `resource/rosbag_topics.txt` |

**Never run `onboard-native` and `onboard` at once.** One MAVROS can bind
14402, and one node can hold the SCF4 lens.

`onboard-native` is for a machine that has no image yet. Everything else on a
provisioned aircraft goes through `./px4sim`.

---

## `setup_git_server.sh`

**For.** Moving code to a drone that cannot reach GitHub. The laptop keeps bare
mirrors in `/srv/git` and serves them read-only over `git://10.200.142.60:9418`.

**Where.** `~/chimera-deploy` on the laptop for `local`, `sync`, `push`,
`deploy` and `status`. On an Orin for `remote`.

```bash
./setup_git_server.sh local             # build the mirrors and the daemon
./setup_git_server.sh deploy            # point every Orin at the laptop
./setup_git_server.sh sync              # GitHub and the laptop and every Orin
./setup_git_server.sh sync --no-push    # stop at the mirrors
./setup_git_server.sh push              # mirrors into each Orin's working copy
./setup_git_server.sh remote            # on an Orin: point its repos here
./setup_git_server.sh remote --restore  # and back to GitHub
./setup_git_server.sh status
```

`sync` also takes `--submodules`, `--no-upstream`, `--no-local` and
`--push-new`. Nothing is force-pushed. A branch that cannot fast-forward is
reported and left for a person.

**Which repositories.** `cdcl_umd_msgs`, `MAVInsight`, `5g_drone`, `px4_msgs`,
`px4-sim-stack` and `chimera-deploy`, plus the chimera-deploy submodules.
`umd_uas.git` is a symlink to `5g_drone.git`, for a clone made before the
rename.

### A branch that GitHub has never seen

This is the path the real-vehicle port used. The work is on a feature branch,
nothing goes to GitHub, and the drone still gets it.

On the laptop, push the worktree straight into the mirror:

```bash
git -C /home/user/px4-sim-stack push /srv/git/px4-sim-stack.git \
    refs/heads/feature/real-drone-port:refs/heads/feature/real-drone-port
git -C /home/user/ros2_ws/src/5g_drone push /srv/git/5g_drone.git \
    refs/heads/feature/real-drone-port:refs/heads/feature/real-drone-port
git --git-dir=/srv/git/px4-sim-stack.git branch -v      # read it back
systemctl is-active git-daemon
```

On the drone, fetch it. `origin` is already the mirror, because
`setup_git_server.sh remote` set it:

```bash
cd ~/px4-sim-stack
git fetch origin
git reset --hard origin/feature/real-drone-port
git status --porcelain          # empty, and .env is gitignored
```

Then rebuild and restart on the drone:

```bash
./px4sim build ros-base onboard
sudo systemctl restart onboard
```

---

## `fetch_models.py`

**For.** The detector and classifier artifacts, and the link that points every
machine at its own TensorRT engines. An engine belongs to one GPU, one driver
and one TensorRT build, so it is filed under the machine that built it.

**Where.** `~/ros2_ws/src/5g_drone/scripts/fetch_models.py`, on any machine.

```bash
./scripts/fetch_models.py check --role onboard   # is everything here
./scripts/fetch_models.py resolve                # which group is this machine
./scripts/fetch_models.py resolve --link         # and point `local` at it
./scripts/fetch_models.py fetch --role onboard   # download what is missing
./scripts/fetch_models.py check --group orin     # one group, whatever machine
```

`--role` is `onboard`, `offboard` or `convert`, and it defaults to `onboard`.
`--group` acts on one group and skips the machine test. `--all` covers every
non-empty group. `--tag` and `--repo` name another GitHub release.

**What it reads.** `perception_models/manifest.json`. `machines` maps a board
model or a GPU name to an engine group. `roles` says whether a role needs
models at all. The groups today are `checkpoints`, `shared`, `orin`, `t500`
and `blackwell`.

**What it refuses.** A machine the manifest has no rule for. `resolve` stops
rather than guessing, and the answer is to add the machine to `machines`.
`fetch` verifies every download against the manifest sha256 and refuses a file
that does not match.

**What it gives the stack.** `.env` names
`ONBOARD_MODEL_DIR=${ROS2_WS_DIR}/src/5g_drone/perception_models` and
`ONBOARD_PARAMS_FILE=/models/local/params.yaml`. Mount the whole tree: the
group directories link into `shared/` with relative symlinks, which dangle if
only one directory is mounted. `local` is the symlink `resolve --link` writes,
so every path is the same on the Orin and on either laptop.

The bbox parser `libnvdsinfer_custom_impl_Yolo.so` is not here.
`modules/onboard/entrypoint.sh` copies it out of `/opt/ds-yolo/` in the image
into the model volume, and stamps `.parser-deepstream` with the release.

---

## `ds-select.sh`

**For.** One question: what does this machine need to run DeepStream. It reads
the GPU and prints shell assignments. `./px4sim` evaluates them on every run
and hands them to compose.

**Where.** `scripts/ds-select.sh` in this repository.

```bash
./scripts/ds-select.sh              # every assignment
./scripts/ds-select.sh --explain    # one sentence, with the reason
./scripts/ds-select.sh --tag        # the built image tag
./scripts/ds-select.sh --version    # the DeepStream release
./scripts/ds-select.sh --image      # and --distro, --codename, --trt
```

**What it decides.** The release is DeepStream 7.1 with ROS 2 Humble on Ubuntu
22.04, always, because that is what the aircraft is and because `cdcl_umd_msgs`
does not decode across a distribution change. Only TensorRT varies:

| Machine | TensorRT | Tag |
|---|---|---|
| An x86 GPU up to Hopper | the release's own 10.3 | `7.1` |
| A Blackwell card | 10.9 for jammy, in the same image | `7.1-trt10.9` |
| A Jetson | the host's own build, from the JetPack apt source | `7.1-trt10.3` |

A Jetson takes the host's build because its engines load only under the exact
TensorRT that made them.

**How to override.** Two keys in `.env`, highest first:

| Key | Effect |
|---|---|
| `DS_IMAGE` | an image outright, for a rebuild or a mirror. The release is read from its tag |
| `DS_VERSION` | pins the release. `7.1`, `8.0` or `9.0` |

**What it refuses.** A `DS_IMAGE` tag that names no release it knows, and a
`DS_VERSION` outside the three. It warns, and carries on, where the driver is
below the release minimum, where the choice brings ROS 2 Jazzy, and where the
GPU is past the TensorRT it can offer.

`./px4sim doctor` prints what this machine chose and why.
