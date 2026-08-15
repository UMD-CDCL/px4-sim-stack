#!/usr/bin/env bash
# Wait for the video streams, expand a config for each camera, then run one
# deepstream-app for each.
#
# One pipeline per camera, not one batched pipeline for both. Batching and then
# splitting with nvstreamdemux produced two payload streams in different
# coordinate spaces, which read from the outside as boxes that were sometimes
# correct and sometimes at two thirds of their true position, and the demuxed
# annotated streams never served a frame. See configs/camera_detector.txt.
#
# Each camera gets its own run directory, its own payload directory, its own
# RTSP port and its own sensor id. Nothing is shared but the model and the
# TensorRT engine file, which is the point: two pipelines cannot disagree about
# a coordinate space they do not share.
set -euo pipefail

DS_CONFIG=${DS_CONFIG:-camera_detector.txt}
SRC_DIR=/opt/perception/configs
MQTT_HOST=${MQTT_HOST:-message-bus}
MQTT_PORT=${MQTT_PORT:-1883}
MQTT_TOPIC=${MQTT_TOPIC:-perception/detections}

# The cameras, in order. The first gets RTSP port 8554, the second 8555, and
# the video router maps those to <camera>_annotated by that order.
PERCEPTION_CAMERA=${PERCEPTION_CAMERA:-nadir}
PERCEPTION_CAMERA_2=${PERCEPTION_CAMERA_2:-gimbal}
RTSP_BASE=${RTSP_BASE:-rtsp://video-router:8554}
RTSP_IN=${RTSP_IN:-$RTSP_BASE/$PERCEPTION_CAMERA}
RTSP_IN_2=${RTSP_IN_2:-$RTSP_BASE/$PERCEPTION_CAMERA_2}

# The resolution each pipeline works in, and the coordinate space it reports
# boxes in. Keep it equal to the camera so nvstreammux neither scales down nor
# up, and the ROS side finds that the boxes already match the image.
DS_WIDTH=${DS_WIDTH:-1920}
DS_HEIGHT=${DS_HEIGHT:-1080}

# Which cameras get a boxes-burned-in RTSP stream, as a comma separated list
# of camera names. "1" or "all" turns every camera on, "0" or "off" turns
# every camera off. The default is the gimbal alone: QGC displays it, and the
# nadir boxes already reach Foxglove as annotations over the plain stream.
ANNOTATED_STREAMS=${ANNOTATED_STREAMS:-gimbal}

annotated_enabled() {
	# Spaces after the commas are tolerated: "nadir, gimbal" works.
	case ",${ANNOTATED_STREAMS// /}," in
	*,1,* | *,all,* | *,true,* | *,yes,* | *,on,*) return 0 ;;
	*,"$1",*) return 0 ;;
	*) return 1 ;;
	esac
}

export MQTT_HOST MQTT_PORT MQTT_TOPIC DS_WIDTH DS_HEIGHT

log()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }

[ -f "$SRC_DIR/$DS_CONFIG" ] || {
	echo "No config at $SRC_DIR/$DS_CONFIG. Available:"
	ls "$SRC_DIR"
	exit 1
}

# Wait for the video router to accept connections. Do not try to pull a frame
# here: deepstream-app has rtsp-reconnect-attempts=-1 and retries the stream
# itself, and a probe that gets the pipeline slightly wrong blocks startup
# forever for no gain.
host=$(echo "$RTSP_IN" | sed -E 's|^rtsp://([^:/]+).*|\1|')
port=$(echo "$RTSP_IN" | sed -nE 's|^rtsp://[^:/]+:([0-9]+).*|\1|p'); port=${port:-8554}
log "Waiting for the video router at $host:$port"
deadline=$(( $(date +%s) + ${RTSP_WAIT_SECONDS:-900} ))
until timeout 3 bash -c "exec 3<>/dev/tcp/$host/$port" 2>/dev/null; do
	if [ "$(date +%s)" -ge "$deadline" ]; then
		warn "No video router after ${RTSP_WAIT_SECONDS:-900} s. Starting anyway."
		break
	fi
	sleep 5
done

# nvinfer writes the TensorRT engine next to the ONNX file and ignores
# model-engine-file when it saves. That path is inside the image, so the engine
# is lost on every container recreate and the first frame waits 45 s again.
# Copying the model into the mounted volume puts the engine there too.
DS_MODELS=/opt/nvidia/deepstream/deepstream/samples/models
if [ ! -d /opt/perception/models/Primary_Detector ]; then
	log "Seeding the default model into modules/perception/models/"
	cp -r "$DS_MODELS/Primary_Detector" /opt/perception/models/
fi

# The first value of a key in a deepstream-app config. The anchor keeps
# config-file= from also matching the tracker's ll-config-file=.
config_value() {
	[ -f "$1" ] || return 0
	sed -n "s/^[[:space:]]*$2=[[:space:]]*//p" "$1" | head -1
}

engine_loads() {
	[ -f "$1" ] || return 1
	python3 /opt/perception/app/engine_check.py "$1" >/dev/null 2>&1
}

# engine_check.py deserializes the whole engine, which costs seconds on every
# start. So each engine that passes it gets a sentinel file next to it holding
# the engine's size, mtime and the driver version. A later start whose values
# still match trusts the sentinel and skips the check; any mismatch falls back
# to the full check.
engine_fingerprint() {
	echo "$(stat -c '%s %Y' "$1" 2>/dev/null) $(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1)"
}

mark_engine_ok() {
	engine_fingerprint "$1" > "$1.ok"
}

# Hold until the engine on disk loads, so the second camera never joins a build
# that is still running.
wait_for_engine() {
	local engine=$1
	local deadline=$(( $(date +%s) + ${ENGINE_WAIT_SECONDS:-900} ))
	while [ "$(date +%s)" -lt "$deadline" ]; do
		if engine_loads "$engine"; then
			mark_engine_ok "$engine"
			log "The engine is built. Starting the second camera."
			return 0
		fi
		sleep 1
	done
	warn "No usable engine after ${ENGINE_WAIT_SECONDS:-900} s. Starting the second camera anyway."
	return 1
}

# Both cameras run the same model, and nvinfer saves the plan next to the ONNX
# under a name it generates rather than the model-engine-file path, so the two
# processes write one file whatever the config says.
#
# Building it in both at once leaves a plan that loads in neither. The next
# start then fails to deserialize, rebuilds in both again, and corrupts it
# again: it never settles, and every start pays the build.
#
# So a plan that loads means both cameras start together, and anything else
# means the first builds it alone while the second waits.
ENGINE=""
infer_config=$(config_value "$SRC_DIR/$DS_CONFIG" config-file)
if [ -n "$infer_config" ]; then
	case "$infer_config" in
	/*) infer_path=$infer_config ;;
	*) infer_path=$SRC_DIR/$infer_config ;;
	esac
	ENGINE=$(config_value "$infer_path" model-engine-file)
fi

ENGINE_READY=0
if [ -z "$ENGINE" ]; then
	warn "No model-engine-file in the config. Both cameras start together and may each build an engine."
elif [ -f "$ENGINE" ] && [ -f "$ENGINE.ok" ] &&
	[ "$(cat "$ENGINE.ok")" = "$(engine_fingerprint "$ENGINE")" ]; then
	# The engine passed the load check before and has not changed since.
	ENGINE_READY=1
elif engine_loads "$ENGINE"; then
	ENGINE_READY=1
	mark_engine_ok "$ENGINE"
elif [ -f "$ENGINE" ]; then
	# Left by a build that raced, by another GPU, or by another TensorRT.
	# nvinfer would overwrite it anyway; removing it keeps the state readable.
	warn "The cached engine does not load. Removing it and building once."
	rm -f "$ENGINE" "$ENGINE.ok"
fi

pids=""

# Stop both pipelines together. Without this, killing the container leaves one
# deepstream-app holding the GPU while the other has already gone.
shutdown() {
	trap - TERM INT
	[ -n "$pids" ] && kill $pids 2>/dev/null || true
	wait 2>/dev/null || true
	exit 0
}
trap shutdown TERM INT

# start_camera <name> <uri> <rtsp-port> <udp-port>
start_camera() {
	local name=$1 uri=$2 rtsp_port=$3 udp_port=$4
	local run_dir=/tmp/perception-$name
	local payload_dir=/tmp/ds-payloads-$name

	rm -rf "$run_dir"
	mkdir -p "$run_dir" "$payload_dir"

	# envsubst reads the environment, so these have to be exported rather than
	# passed as a command prefix.
	export DS_SENSOR_ID=$name RTSP_IN=$uri DS_RTSP_PORT=$rtsp_port
	export DS_UDP_PORT=$udp_port DS_PAYLOAD_DIR=$payload_dir
	if annotated_enabled "$name"; then ANNOTATED_ENABLE=1; else ANNOTATED_ENABLE=0; fi
	export ANNOTATED_ENABLE

	# deepstream-app resolves relative paths against the config file's
	# directory, so the whole directory is copied and expanded together.
	for f in "$SRC_DIR"/*; do
		[ -f "$f" ] || continue
		envsubst < "$f" > "$run_dir/$(basename "$f")"
	done

	# One forwarder for each payload directory. They publish to the same MQTT
	# topic, and the sensorId in each payload says which camera it came from.
	if [ "${PAYLOAD_FORWARDER:-1}" = "1" ]; then
		python3 /opt/perception/app/payload_forwarder.py \
			--dir "$payload_dir" --name "$name" \
			--host "$MQTT_HOST" --port "$MQTT_PORT" \
			--topic "$MQTT_TOPIC" &
		pids="$pids $!"
	fi

	( cd "$run_dir" && exec deepstream-app -c "$run_dir/$DS_CONFIG" ) &
	pids="$pids $!"
	printf '    %-8s %s\n' "$name" "$uri"
	if [ "$ANNOTATED_ENABLE" = 1 ]; then
		printf '    %-8s annotated rtsp://perception:%s/ds-test -> %s_annotated\n' \
			"" "$rtsp_port" "$name"
	else
		printf '    %-8s annotated off (ANNOTATED_STREAMS=%s)\n' "" "$ANNOTATED_STREAMS"
	fi
}

log "Starting deepstream-app for each camera with $DS_CONFIG"
cat <<EOF
    detector    ${DS_WIDTH}x${DS_HEIGHT} for each camera, which is also the box
                coordinate space the ROS side reads
    detections  mqtt://$MQTT_HOST:$MQTT_PORT topic $MQTT_TOPIC
    annotated   $ANNOTATED_STREAMS

    The first run builds a TensorRT engine, which takes one to three minutes.
    The first camera builds it and the second waits, so one plan is written
    once. The engine is cached next to the model in
    modules/perception/models/Primary_Detector/ and reused on later starts.

EOF

start_camera "$PERCEPTION_CAMERA" "$RTSP_IN" 8554 5400
if [ "$ENGINE_READY" = 0 ] && [ -n "$ENGINE" ]; then
	log "Building the TensorRT engine in the first camera, so it is written once"
	wait_for_engine "$ENGINE"
fi
start_camera "$PERCEPTION_CAMERA_2" "$RTSP_IN_2" 8555 5401

# Exit when the first pipeline exits, so a crash is visible to compose rather
# than leaving the container up with half a perception stack.
wait -n $pids
warn "A pipeline exited. Stopping the rest."
shutdown
