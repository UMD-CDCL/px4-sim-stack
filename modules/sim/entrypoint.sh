#!/usr/bin/env bash
# Start order: build PX4 if necessary, start the Gazebo server, start the
# camera encoders, place the scenario targets, then hand the terminal to the
# PX4 shell.
set -euo pipefail

PX4_DIR=${PX4_DIR:-/px4}
SCENES_DIR=${SCENES_DIR:-/scenes}
SCENE=${SCENE:-recon_field}
VEHICLE=${VEHICLE:-x500_recon}
SCENARIO=${SCENARIO:-}
GZ_GUI=${GZ_GUI:-1}
BUILD_JOBS=${BUILD_JOBS:-$(nproc)}
VIDEO_SINK_BASE=${VIDEO_SINK_BASE:-rtsp://video-router:8554}
BUILD_DIR=$PX4_DIR/build/px4_sitl_default
MERGED=/tmp/scenes

log()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }
die()  { printf '\033[31m!!! %s\033[0m\n' "$*" >&2; exit 1; }

children=()
cleanup() {
	for pid in "${children[@]:-}"; do
		[ -n "${pid:-}" ] && kill "$pid" 2>/dev/null || true
	done
}
trap cleanup EXIT INT TERM

# ---------------------------------------------------------------- 1. the source
[ -f "$PX4_DIR/Makefile" ] || die "No PX4 source at $PX4_DIR. Run: make bootstrap"

# ---------------------------------------------------------------- 2. the build
if [ ! -x "$BUILD_DIR/bin/px4" ] || [ "${FORCE_BUILD:-0}" = "1" ]; then
	log "Building PX4 SITL with $BUILD_JOBS jobs. The first build takes 10 to 20 minutes."
	if ! make -C "$PX4_DIR" px4_sitl_default -j"$BUILD_JOBS"; then
		# GCC 13 sometimes hits an internal compiler error on one of the heavy
		# EKF2 translation units when many compiles run at once. It is not a
		# source problem, and it does not repeat in the same place. ninja
		# resumes from where it stopped, so a retry costs only the failed file.
		retry_jobs=$(( BUILD_JOBS > 4 ? BUILD_JOBS / 2 : 1 ))
		warn "The build stopped. Retrying once with $retry_jobs jobs."
		make -C "$PX4_DIR" px4_sitl_default -j"$retry_jobs" \
			|| die "PX4 failed to build twice. Read the error above."
	fi
else
	log "PX4 binary present. Set FORCE_BUILD=1 to rebuild, or run 'make -C /px4 px4_sitl_default'."
fi
"$BUILD_DIR/bin/px4" -h 2>/dev/null | head -1 || true

# ------------------------------------------------- 3. the Gazebo resource paths
# PX4 generates gz_env.sh at build time. It points at the PX4 model set and at
# the PX4 gz plugins. Source it first, then put our own scenes on top.
#
# That generated script appends to GZ_SIM_RESOURCE_PATH and
# GZ_SIM_SYSTEM_PLUGIN_PATH without checking whether they exist. Under `set -u`
# an unset variable stops the script, so define both as empty first.
: "${GZ_SIM_RESOURCE_PATH:=}"
: "${GZ_SIM_SYSTEM_PLUGIN_PATH:=}"
export GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH

# shellcheck disable=SC1091
[ -f "$BUILD_DIR/rootfs/gz_env.sh" ] && . "$BUILD_DIR/rootfs/gz_env.sh"

PX4_MODEL_DIR=${PX4_GZ_MODELS:-$PX4_DIR/Tools/simulation/gz/models}
PX4_WORLD_DIR=${PX4_GZ_WORLDS:-$PX4_DIR/Tools/simulation/gz/worlds}

# A merged view: every PX4 model and world, with ours layered over the top. A
# file of ours that has the same name as a PX4 file replaces it.
rm -rf "$MERGED"
mkdir -p "$MERGED/models" "$MERGED/worlds"
[ -d "$PX4_MODEL_DIR" ] && find "$PX4_MODEL_DIR" -mindepth 1 -maxdepth 1 -type d \
	-exec ln -sfn -t "$MERGED/models" {} +
[ -d "$PX4_WORLD_DIR" ] && find "$PX4_WORLD_DIR" -mindepth 1 -maxdepth 1 -name '*.sdf' \
	-exec ln -sfn -t "$MERGED/worlds" {} +
[ -d "$SCENES_DIR/models" ] && find "$SCENES_DIR/models" -mindepth 1 -maxdepth 1 -type d \
	-exec ln -sfn -t "$MERGED/models" {} +
[ -d "$SCENES_DIR/worlds" ] && find "$SCENES_DIR/worlds" -mindepth 1 -maxdepth 1 -name '*.sdf' \
	-exec ln -sfn -t "$MERGED/worlds" {} +

export PX4_GZ_MODELS="$MERGED/models"
export PX4_GZ_WORLDS="$MERGED/worlds"
export GZ_SIM_RESOURCE_PATH="$MERGED/models:$MERGED/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_GZ_WORLD="$SCENE"

# Our server plugin list. It matches PX4's, minus the single-camera streamer
# that PX4 loads by default. See scenes/server.config for the reason.
if [ -f "$SCENES_DIR/server.config" ]; then
	export GZ_SIM_SERVER_CONFIG_PATH="$SCENES_DIR/server.config"
fi

WORLD_FILE="$MERGED/worlds/$SCENE.sdf"
[ -f "$WORLD_FILE" ] || die "No world named '$SCENE'. Available: $(cd "$MERGED/worlds" && ls *.sdf | sed 's/\.sdf//' | tr '\n' ' ')"
[ -d "$MERGED/models/$VEHICLE" ] || die "No vehicle model named '$VEHICLE' in $MERGED/models"

# PX4 addresses the world by the name inside the file, not by the file name.
# A mismatch produces a silent hang, so check it here.
world_name=$(grep -oP "<world[^>]*name=['\"]\K[^'\"]+" "$WORLD_FILE" | head -1)
if [ "$world_name" != "$SCENE" ]; then
	die "World file $SCENE.sdf declares <world name='$world_name'>. The two must match."
fi

# ------------------------------------------------------- 4. the Gazebo server
log "Starting Gazebo Harmonic, world '$SCENE'"
gz sim -r -s -v "${GZ_VERBOSE:-1}" "$WORLD_FILE" &
children+=($!)

for _ in $(seq 1 90); do
	if gz topic -l 2>/dev/null | grep -qx "/world/$SCENE/clock"; then break; fi
	sleep 1
done
gz topic -l 2>/dev/null | grep -qx "/world/$SCENE/clock" \
	|| die "Gazebo did not bring up world '$SCENE'."
log "World '$SCENE' is up"

if [ "$GZ_GUI" = "1" ]; then
	if [ -n "${DISPLAY:-}" ]; then
		log "Starting the Gazebo GUI on $DISPLAY"
		gz sim -g >/tmp/gz-gui.log 2>&1 &
		children+=($!)
	else
		warn "GZ_GUI=1 but DISPLAY is empty. Running headless."
	fi
fi

# ------------------------------------------------------ 5. the camera encoders
STREAMS_FILE="$SCENES_DIR/models/$VEHICLE/streams.conf"
if [ -f "$STREAMS_FILE" ]; then
	args=(--sink-base "$VIDEO_SINK_BASE")
	while read -r name regex bitrate fps; do
		case "${name:-}" in ''|\#*) continue ;; esac
		args+=(--stream "name=$name,regex=$regex,bitrate=${bitrate:-4000},fps=${fps:-30}")
	done < "$STREAMS_FILE"
	# The streamer probes the encoders and takes the first that works. Set
	# VIDEO_ENCODER to a GStreamer fragment to skip that and name your own:
	#   VIDEO_ENCODER='x264enc tune=zerolatency speed-preset=ultrafast bitrate=4000'
	if [ -n "${VIDEO_ENCODER:-}" ]; then
		args+=(--encoder "$VIDEO_ENCODER")
	fi

	# The capture-time side channel. RTSP carries no usable capture time, so
	# the streamer reports one datagram for each frame and this forwards them
	# to the message bus. See modules/sim/frame_clock.py.
	if [ "${FRAME_CLOCK:-1}" = "1" ]; then
		log "Starting the frame clock on udp/${FRAME_CLOCK_PORT:-5599}"
		python3 /opt/sim/frame_clock.py --port "${FRAME_CLOCK_PORT:-5599}" &
		children+=($!)
		args+=(--frame-clock "127.0.0.1:${FRAME_CLOCK_PORT:-5599}")
	fi

	log "Starting the camera encoders from $(basename "$STREAMS_FILE")"
	gz_video_streamer "${args[@]}" &
	children+=($!)
else
	warn "No streams.conf for '$VEHICLE'. This vehicle publishes no video."
fi

# -------------------------------------------------------- 6. the scenario props
if [ -n "$SCENARIO" ] && [ -f "$SCENES_DIR/scenarios/$SCENARIO.yaml" ]; then
	log "Placing scenario '$SCENARIO'"
	export RESOLVED_TRUTH_FILE="$SCENES_DIR/ground_truth_actual.yaml"
	"$SCENES_DIR/spawn_scenario.py" --world "$SCENE" \
		--scenario "$SCENES_DIR/scenarios/$SCENARIO.yaml" || warn "Scenario spawn failed."
elif [ -n "$SCENARIO" ]; then
	warn "No scenario file at $SCENES_DIR/scenarios/$SCENARIO.yaml"
fi

# --------------------------------------------------------------- 7. PX4 itself
# PX4 runs in standalone mode: Gazebo already exists, so PX4 spawns the vehicle
# into the running world and attaches its bridge.
mkdir -p /px4-logs
ln -sfn /px4-logs "$BUILD_DIR/rootfs/log" 2>/dev/null || true

log "Starting PX4 SITL: model $PX4_SIM_MODEL, airframe $PX4_SYS_AUTOSTART"
echo "    Uplink to the hub: ${MAVLINK_HUB_IP:-unset}:${MAVLINK_HUB_PORT:-unset}"
echo "    The pxh> console is this container's terminal."
echo "    Attach with: make px4-console      Detach with: Ctrl-P Ctrl-Q"
cd "$BUILD_DIR/rootfs"

# -s runs our startup script instead of the stock one. Ours sources the stock
# script first, then adds the link that pushes to the hub. See px4-rcS for why.
#
# PX4 needs the trailing rootfs path when you give it -s. PX4 links etc/ into
# the working directory from that path. It falls back to its own build
# directory only without -s. With -s and no path, px4-rcS cannot find the
# stock rcS.
PX4_RCS=${PX4_RCS:-/opt/sim/px4-rcS}
if [ -f "$PX4_RCS" ] && [ -n "${MAVLINK_HUB_IP:-}" ]; then
	exec "$BUILD_DIR/bin/px4" -s "$PX4_RCS" "$BUILD_DIR/etc"
fi
warn "No PX4_RCS or no MAVLINK_HUB_IP. Using the stock PX4 startup."
warn "Telemetry then depends on the hub keepalive, and a hub restart needs a sim restart."
exec "$BUILD_DIR/bin/px4"
