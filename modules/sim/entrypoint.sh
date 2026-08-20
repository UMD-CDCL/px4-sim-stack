#!/usr/bin/env bash
# Start order: build PX4 if necessary, start the Gazebo server, start the
# camera encoders, place the scenario targets, then start one PX4 SITL instance
# for each vehicle in the fleet.
#
# Every vehicle shares this container, because PX4 finds Gazebo over
# gz-transport, which discovers its peers with UDP multicast. One container
# keeps that on the loopback where it always works.
set -euo pipefail

PX4_DIR=${PX4_DIR:-/px4}
SCENES_DIR=${SCENES_DIR:-/scenes}
SCENE=${SCENE:-recon_field}
# One entry for each vehicle, in UAS number order. uas11 is the first.
# The camera fields of view are degrees, and they belong to the vehicle rather
# than to the mark: uas13 and uas14 are both v2 and carry different lenses.
# .env.example says where each number came from.
UAS_FLEET=${UAS_FLEET:-"chimera_v3 chimera_v3 chimera_v2 chimera_v2"}
# Simulated vehicles are numbered from 11, so they never take a system id, a
# port, a DDS domain or an address from a real one. Do not set this to 0: PX4
# instance 0 puts our rangefinder link on 14590, which its own offboard link
# already holds, and that link then fails to start.
UAS_BASE=${UAS_BASE:-10}
UAS_GIMBAL_HFOV_DEG=${UAS_GIMBAL_HFOV_DEG:-"27.45 27.45 85.25 25.98"}
UAS_THERMAL_HFOV_DEG=${UAS_THERMAL_HFOV_DEG:-"29.75 29.75 31.03 29.75"}
UAS_DOWN_HFOV_DEG=${UAS_DOWN_HFOV_DEG:-"25.98 25.98 25.98 25.98"}
UAS_SPACING_M=${UAS_SPACING_M:-1}
SCENARIO=${SCENARIO:-}
GZ_GUI=${GZ_GUI:-0}
BUILD_JOBS=${BUILD_JOBS:-$(nproc)}
VIDEO_SINK_BASE=${VIDEO_SINK_BASE:-rtsp://video-router:8554}
MAVLINK_ROUTER_IP_BASE=${MAVLINK_ROUTER_IP_BASE:-10.200.142.2}
MAVLINK_ROUTER_PORT=${MAVLINK_ROUTER_PORT:-14545}
LOG_DIR=/px4-logs
BUILD_DIR=$PX4_DIR/build/px4_sitl_default
MERGED=/tmp/scenes
STREAM_CONF_DIR=/tmp/streams

log()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }
die()  { printf '\033[31m!!! %s\033[0m\n' "$*" >&2; exit 1; }

read -r -a FLEET <<< "$UAS_FLEET"
read -r -a GIMBAL_HFOV <<< "$UAS_GIMBAL_HFOV_DEG"
read -r -a THERMAL_HFOV <<< "$UAS_THERMAL_HFOV_DEG"
read -r -a DOWN_HFOV <<< "$UAS_DOWN_HFOV_DEG"
read -r -a STREAM_CHOICE <<< "${UAS_STREAMS:-gimbal}"
# shellcheck disable=SC1091
. /opt/sim/zoom.sh
[ ${#FLEET[@]} -ge 1 ] || die "UAS_FLEET is empty. Give one model name for each vehicle."
[ ${#FLEET[@]} -le 9 ] || die "UAS_FLEET has ${#FLEET[@]} vehicles. The simulator numbers them 11 to 19."

radians() { awk -v deg="$1" 'BEGIN { printf "%.6f", deg * atan2(0, -1) / 180 }'; }

# The mark of an airframe, which decides what it carries.
mark_of_model() {
	case "$1" in
	*_v3) echo v3 ;;
	*_v2) echo v2 ;;
	esac
}

# The fields of view a v3's gimbal renders, widest first, in degrees. One
# camera for each framing the zoom lens reaches, all on the same mount: gz-sim
# cannot change a camera's field of view once the world is loaded.
#
# They come from the framing table rather than from UAS_GIMBAL_HFOV_DEG, and
# that is deliberate. A vehicle rendered narrower than its widest framing could
# never reach that framing, and nothing downstream would be able to tell -- the
# picture would simply stop at the widest the camera has while the calibration
# went on saying otherwise. Deriving it removes the setting that can be wrong.
gimbal_framings_deg() {
	[ "$(mark_of_model "$1")" = v3 ] || return 1
	local preset
	zoom_presets_widest_first | while read -r preset; do zoom_hfov_deg "$preset"; done
}

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
# Always, not only when the binary is missing. patches/ changes the gimbal in
# this tree, so a binary that predates an edit runs the old behaviour and says
# nothing. ninja finds nothing to do in a few seconds when the tree is built.
if [ "${FORCE_BUILD:-0}" = "1" ]; then
	rm -rf "$BUILD_DIR"
fi
if [ ! -x "$BUILD_DIR/bin/px4" ]; then
	log "Building PX4 SITL with $BUILD_JOBS jobs. The first build takes 10 to 20 minutes."
fi
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
# gz-sim resolves model:// through a callback it installs itself. The `gz sdf`
# command line has no callback and reads SDF_PATH instead, so without this the
# check below reports every airframe as unspawnable while the vehicle spawns.
export SDF_PATH="$MERGED/models"
export PX4_GZ_WORLD="$SCENE"

# Our server plugin list. It matches PX4's, minus the single-camera streamer
# that PX4 loads by default. See scenes/server.config for the reason.
if [ -f "$SCENES_DIR/server.config" ]; then
	export GZ_SIM_SERVER_CONFIG_PATH="$SCENES_DIR/server.config"
fi

WORLD_FILE="$MERGED/worlds/$SCENE.sdf"
[ -f "$WORLD_FILE" ] || die "No world named '$SCENE'. Available: $(cd "$MERGED/worlds" && ls *.sdf | sed 's/\.sdf//' | tr '\n' ' ')"

# One airframe model for each vehicle, rendered from the mark's template. The
# camera field of view is the reason: it belongs to the vehicle, and SDF has no
# way to override a sensor from outside the file that declares it. PX4 spawns
# ${PX4_GZ_MODELS}/<model>/model.sdf and names the entity <model>_<instance>,
# so uas11 becomes the Gazebo model uas11_10.
for index in "${!FLEET[@]}"; do
	model=${FLEET[$index]}
	template="$SCENES_DIR/models/$model/model.sdf"
	[ -f "$template" ] || die "No vehicle model named '$model' in $SCENES_DIR/models"

	uas_num=$((UAS_BASE + index + 1))
	rendered="$MERGED/models/uas$uas_num"
	rm -rf "$rendered"
	mkdir -p "$rendered"

	# shellcheck disable=SC2207
	framings=($(gimbal_framings_deg "$model" || true))
	configured=${GIMBAL_HFOV[$index]:-${GIMBAL_HFOV[0]}}
	if [ ${#framings[@]} -gt 0 ]; then
		if [ "$configured" != "${framings[0]}" ]; then
			warn "uas$uas_num renders its gimbal camera at ${framings[0]} degrees, the widest framing, not the $configured degrees UAS_GIMBAL_HFOV_DEG names. A v3 takes its gimbal field of view from UAS_ZOOM_PRESETS."
		fi
		GIMBAL_HFOV_RAD=$(radians "${framings[0]}")
		# A framing the table does not name renders at the widest, which makes
		# it a duplicate. The streamer keeps only the framings narrower than
		# the one it bound to, so a duplicate is simply never chosen.
		GIMBAL_HFOV_2_RAD=$(radians "${framings[1]:-${framings[0]}}")
		GIMBAL_HFOV_3_RAD=$(radians "${framings[2]:-${framings[0]}}")
	else
		GIMBAL_HFOV_RAD=$(radians "$configured")
		GIMBAL_HFOV_2_RAD=$GIMBAL_HFOV_RAD
		GIMBAL_HFOV_3_RAD=$GIMBAL_HFOV_RAD
	fi
	THERMAL_HFOV_RAD=$(radians "${THERMAL_HFOV[$index]:-${THERMAL_HFOV[0]}}")
	DOWN_HFOV_RAD=$(radians "${DOWN_HFOV[$index]:-${DOWN_HFOV[0]}}")
	UAS_NUM=$uas_num
	export GIMBAL_HFOV_RAD GIMBAL_HFOV_2_RAD GIMBAL_HFOV_3_RAD
	export THERMAL_HFOV_RAD DOWN_HFOV_RAD UAS_NUM
	envsubst '${GIMBAL_HFOV_RAD} ${GIMBAL_HFOV_2_RAD} ${GIMBAL_HFOV_3_RAD} ${THERMAL_HFOV_RAD} ${DOWN_HFOV_RAD} ${UAS_NUM}' \
		< "$template" > "$rendered/model.sdf"
	printf '<?xml version="1.0"?>\n<model><name>uas%s</name><version>1.0</version>\n<sdf version="1.9">model.sdf</sdf>\n<description>%s for uas%s, rendered by the sim entrypoint.</description>\n</model>\n' \
		"$uas_num" "$model" "$uas_num" > "$rendered/model.config"

	# The template itself carries ${...} where a number belongs, so take it out
	# of the model path. Only the rendered copies are spawnable.
	rm -f "$MERGED/models/$model"

	# An airframe merges chimera_common, which merges x500, the gimbal and the
	# LW20. A model that libsdformat cannot expand does not fail loudly: PX4
	# asks Gazebo to spawn it, Gazebo declines, and the vehicle simply never
	# appears. Expanding it here turns that into one message.
	if ! sdf_errors=$(gz sdf -p "$rendered/model.sdf" 2>&1 >/dev/null); then
		warn "uas$uas_num: Gazebo cannot expand $model. It will not spawn."
		warn "$sdf_errors"
	fi
done
unset GIMBAL_HFOV_RAD GIMBAL_HFOV_2_RAD GIMBAL_HFOV_3_RAD
unset THERMAL_HFOV_RAD DOWN_HFOV_RAD UAS_NUM

# PX4 addresses the world by the name inside the file, not by the file name.
# A mismatch produces a silent hang, so check it here.
world_name=$(grep -oP "<world[^>]*name=['\"]\K[^'\"]+" "$WORLD_FILE" | head -1)
if [ "$world_name" != "$SCENE" ]; then
	die "World file $SCENE.sdf declares <world name='$world_name'>. The two must match."
fi

# ------------------------------------------------------- 4. the Gazebo server
log "Starting Gazebo Harmonic, world '$SCENE', ${#FLEET[@]} vehicles"
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

# Whether a vehicle serves one camera, given what it was asked for.
serves() {
	local choice=$1 camera=$2 gimbal=$3
	case "$choice" in
	all)    return 0 ;;
	gimbal) [ "$camera" = "$gimbal" ]; return ;;
	esac
	case ",$choice," in
	*",$camera,"*) return 0 ;;
	esac
	return 1
}

# ------------------------------------------------------ 5. the camera encoders
mkdir -p "$STREAM_CONF_DIR"
for index in "${!FLEET[@]}"; do
	model=${FLEET[$index]}
	# Simulated vehicles are 11 upwards, so a simulator can fly beside the real
	# fleet without taking a system id from it. PX4 gives MAV_SYS_ID =
	# instance + 1, so the instance is UAS_NUM - 1.
	export UAS_NUM=$((UAS_BASE + index + 1))
	export GZ_MODEL="uas${UAS_NUM}_$((UAS_NUM - 1))"

	template="$SCENES_DIR/models/$model/streams.conf"
	if [ ! -f "$template" ]; then
		warn "No streams.conf for '$model'. uas$UAS_NUM publishes no video."
		continue
	fi

	# streams.conf names the stream after the vehicle and matches the Gazebo
	# model instance, so one file serves every vehicle of that model.
	conf="$STREAM_CONF_DIR/uas$UAS_NUM.conf"
	envsubst '${UAS_NUM} ${GZ_MODEL}' < "$template" > "$conf"

	# Which cameras this vehicle serves. A GPU allows a handful of encoding
	# sessions at once, and a fleet of four asks for more than that, so a
	# vehicle serves its gimbal camera alone unless it is told otherwise.
	#   gimbal   what the detector reads: rgb on a v3, pilot on a v2
	#   all      every camera the model declares
	#   a list   rgb, pilot or thermal, comma separated
	choice=${STREAM_CHOICE[$index]:-${STREAM_CHOICE[0]:-gimbal}}
	# The same three fields of view the model was rendered with, so the
	# streamer, the cameras and the calibration all describe one lens.
	# shellcheck disable=SC2207
	framings=($(gimbal_framings_deg "$model" || true))
	if [ ${#framings[@]} -gt 0 ]; then
		gimbal_hfov_rad=$(radians "${framings[0]}")
	else
		gimbal_hfov_rad=$(radians "${GIMBAL_HFOV[$index]:-${GIMBAL_HFOV[0]}}")
	fi
	case "$model" in
	*_v3) gimbal_camera=rgb ;;
	*_v2) gimbal_camera=pilot ;;
	*)    gimbal_camera= ;;
	esac

	args=(--sink-base "$VIDEO_SINK_BASE")
	served=()
	while read -r name regex bitrate fps width height; do
		case "${name:-}" in ''|\#*) continue ;; esac
		# rgb and rgbl are one camera, and so are the other pairs. The vehicle
		# number is on the end of the name and the l is what marks the scaled
		# stream, so the camera is what is left.
		camera=${name%%[0-9]*}
		camera=${camera%l}
		serves "$choice" "$camera" "$gimbal_camera" || continue

		spec="name=$name,regex=$regex,bitrate=${bitrate:-4000},fps=${fps:-30}"
		# A zoom lens, on the camera that has one. gz-sim cannot change a
		# camera's field of view once the world is loaded, so the camera
		# renders its widest and the streamer crops the middle out for a
		# narrower one. The crop is a real field of view, which is what the
		# calibration and every ray cast through it depend on.
		if [ "$camera" = "$gimbal_camera" ] && [ "$(mark_of_model "$model")" = v3 ]; then
			spec="$spec,hfov=${gimbal_hfov_rad},zoom_topic=/uas${UAS_NUM}/camera/zoom"
			# The narrower framings, in the order the model declares them.
			# The streamer reads whichever one covers what the lens is asking
			# for, so a framing has a whole rendering at its own field of view
			# rather than a fraction of the widest stretched back up.
			for framing in 2 3; do
				[ -n "${framings[$((framing - 1))]:-}" ] || continue
				spec="$spec,framing=$(radians "${framings[$((framing - 1))]}"):/uas${UAS_NUM}/camera/framing/${framing}"
			done
		fi
		case "${width:--}" in
		-|'') ;;
		*)  spec="$spec,width=$width,height=$height"
		    # A scaled stream is small, so it encodes in software without
		    # troubling the physics loop and leaves a GPU session for a full
		    # one. Same codec either way.
		    [ "${VIDEO_SCALED_ENCODER:-software}" = software ] && spec="$spec,encoder=software"
		    ;;
		esac
		args+=(--stream "$spec")
		served+=("$name")
	done < "$conf"

	if [ ${#served[@]} -eq 0 ]; then
		warn "uas$UAS_NUM serves no camera: UAS_STREAMS entry '$choice' matches nothing in $model/streams.conf"
		continue
	fi

	# The streamer probes the encoders and takes the first that works. Set
	# VIDEO_ENCODER to a GStreamer fragment to skip that and name your own, and
	# VIDEO_PARSER to the parser that fragment's output needs:
	#   VIDEO_ENCODER='x264enc tune=zerolatency bitrate=4000' VIDEO_PARSER=h264parse
	if [ -n "${VIDEO_ENCODER:-}" ]; then
		args+=(--encoder "$VIDEO_ENCODER" --parser "${VIDEO_PARSER:-h264parse}")
	fi

	log "Starting the uas$UAS_NUM camera encoders: ${served[*]}"
	gz_video_streamer "${args[@]}" &
	children+=($!)
done
unset UAS_NUM GZ_MODEL

# ----------------------------------------------------------- 5b. the v3 lenses
# On the aircraft the lens is a Kurokesu SCF4 on a USB serial port. Here it is
# the same controller answering the same G-code on a TCP port, and the
# companion opens socket://sim:<port> instead of /dev/tty*. The companion runs
# the real umd_uas/zoom.py against it either way: the same homing, the same
# travel limits, the same preset recalls, the same CameraInfo.
#
# The lens writes the field of view it is at and gz_zoom_publisher puts it on
# the topic the streamer follows, so the picture goes where the lens goes --
# including through a preset recall, which takes the mechanism a real second or
# two, exactly as it does on the aircraft.
#
# Its own loop rather than part of the encoders': a vehicle that serves no
# camera still has a lens, and zoom.py takes itself down when the controller
# never turns up.
for index in "${!FLEET[@]}"; do
	[ "$(mark_of_model "${FLEET[$index]}")" = v3 ] || continue
	uas_num=$((UAS_BASE + index + 1))
	port=$(zoom_port "$uas_num")
	mkdir -p "$LOG_DIR/uas$uas_num"
	log "Starting the uas$uas_num lens: an SCF4 on port $port"
	echo "    Log: logs/px4/uas$uas_num/lens.log"
	( python3 /opt/sim/scf4_emulator.py \
		--uas "$uas_num" --port "$port" \
		--presets "$UAS_ZOOM_PRESETS" --steps "$UAS_ZOOM_STEPS" \
		--travel "$UAS_ZOOM_TRAVEL" --datum "$UAS_ZOOM_DATUM" \
		| gz_zoom_publisher --topic "/uas${uas_num}/camera/zoom" \
		) >> "$LOG_DIR/uas$uas_num/lens.log" 2>&1 &
	children+=($!)
done

# -------------------------------------------------------- 6. the scenario props
if [ -n "$SCENARIO" ] && [ -f "$SCENES_DIR/scenarios/$SCENARIO.yaml" ]; then
	log "Placing scenario '$SCENARIO'"
	export RESOLVED_TRUTH_FILE="$SCENES_DIR/ground_truth_actual.yaml"
	"$SCENES_DIR/spawn_scenario.py" --world "$SCENE" \
		--scenario "$SCENES_DIR/scenarios/$SCENARIO.yaml" || warn "Scenario spawn failed."
elif [ -n "$SCENARIO" ]; then
	warn "No scenario file at $SCENES_DIR/scenarios/$SCENARIO.yaml"
fi

# --------------------------------------------------------------- 7. the fleet
# PX4 runs in standalone mode: Gazebo already exists, so each instance spawns
# its own vehicle into the running world and attaches its bridge.
#
# `px4 -i <instance>` gives MAV_SYS_ID = instance + 1 and names the Gazebo model
# <model>_<instance>, which is the whole of the identity contract.
#
# uas11 keeps the container's terminal, so `./px4sim console` still reaches a
# pxh> prompt. Every other vehicle runs with -d and logs to a file.
start_vehicle() {
	local index=$1 uas_num=$2 model=$3
	# PX4 sets MAV_SYS_ID to the instance plus one, so the instance is the
	# vehicle number minus one. With UAS_BASE at 10 that is 10 upwards, and the
	# fleet's own instances 0 to 8 stay free.
	local instance=$((uas_num - 1))
	local work="$BUILD_DIR/rootfs/$instance"

	mkdir -p "$work" "$LOG_DIR/uas$uas_num"
	ln -sfn "$LOG_DIR/uas$uas_num" "$work/log" 2>/dev/null || true

	export PX4_SIM_MODEL="gz_uas$uas_num"
	export PX4_GZ_STANDALONE=1
	export PX4_GZ_MODEL_POSE="0,$(echo "$index * $UAS_SPACING_M" | bc),0,0,0,0"
	export MAVLINK_ROUTER_IP="${MAVLINK_ROUTER_IP_BASE}${uas_num}"
	export MAVLINK_ROUTER_PORT

	# PX4 needs the trailing rootfs path when you give it -s. PX4 links etc/
	# into the working directory from that path. It falls back to its own build
	# directory only without -s, and with -s and no path px4-rcS cannot find the
	# stock rcS.
	local px4=("$BUILD_DIR/bin/px4" -i "$instance" -w "$work")
	[ "$index" = 0 ] || px4+=(-d)
	px4+=(-s "${PX4_RCS:-/opt/sim/px4-rcS}" "$BUILD_DIR/etc")

	if [ "${HOLD_STREAM_RATES:-1}" = "1" ] && [ -x /opt/sim/hold-stream-rates.sh ]; then
		# PX4 drops the boot-time stream rates when a ground station joins the
		# link, which starves the camera frame. See hold-stream-rates.sh.
		PX4_RCS="${PX4_RCS:-/opt/sim/px4-rcS}" PX4_BUILD_DIR="$BUILD_DIR" \
			PX4_INSTANCE="$instance" /opt/sim/hold-stream-rates.sh &
		children+=($!)
	fi

	if [ "${GIMBAL_RANGEFINDER:-1}" = "1" ]; then
		# The gimbal laser reaches MAVROS as gimbal_lidar_50m only if it arrives
		# as DISTANCE_SENSOR id 1. See modules/sim/gimbal_rangefinder.py.
		python3 /opt/sim/gimbal_rangefinder.py \
			--world "$SCENE" --model "uas${uas_num}_${instance}" --sysid "$uas_num" \
			--px4-port $((14590 + instance)) \
			>> "$LOG_DIR/uas$uas_num/gimbal-rangefinder.log" 2>&1 &
		children+=($!)
	fi

	if [ "$index" = 0 ]; then
		log "Starting uas$uas_num: $model, system id $uas_num, router ${MAVLINK_ROUTER_IP}:${MAVLINK_ROUTER_PORT}"
		echo "    The pxh> console for uas$uas_num is this container's terminal."
		echo "    Attach with: ./px4sim console      Detach with: Ctrl-P Ctrl-Q"
		cd "$work"
		exec "${px4[@]}"
	fi

	log "Starting uas$uas_num: $model, system id $uas_num, router ${MAVLINK_ROUTER_IP}:${MAVLINK_ROUTER_PORT}"
	echo "    Log: logs/px4/uas$uas_num/px4.log"
	( cd "$work" && "${px4[@]}" >> "$LOG_DIR/uas$uas_num/px4.log" 2>&1 ) &
	children+=($!)
	# Each instance spawns its model through the same Gazebo service. Let one
	# finish before the next asks.
	sleep "${UAS_START_DELAY_S:-4}"
}

for index in $(seq $(( ${#FLEET[@]} - 1 )) -1 0); do
	start_vehicle "$index" $((UAS_BASE + index + 1)) "${FLEET[$index]}"
done
