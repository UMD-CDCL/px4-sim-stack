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

case "${ANNOTATED_STREAMS:-1}" in
0 | false | no | off) ANNOTATED_ENABLE=0 ;;
*) ANNOTATED_ENABLE=1 ;;
esac

export MQTT_HOST MQTT_PORT MQTT_TOPIC DS_WIDTH DS_HEIGHT ANNOTATED_ENABLE

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
until python3 -c "import socket,sys; socket.create_connection((sys.argv[1], int(sys.argv[2])), 3).close()" \
        "$host" "$port" 2>/dev/null; do
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
			--dir "$payload_dir" --host "$MQTT_HOST" --port "$MQTT_PORT" \
			--topic "$MQTT_TOPIC" --poll 0.02 &
		pids="$pids $!"
	fi

	( cd "$run_dir" && exec deepstream-app -c "$run_dir/$DS_CONFIG" ) &
	pids="$pids $!"
	printf '    %-8s %s\n' "$name" "$uri"
	printf '    %-8s annotated rtsp://perception:%s/ds-test -> %s_annotated\n' \
		"" "$rtsp_port" "$name"
}

log "Starting deepstream-app for each camera with $DS_CONFIG"
cat <<EOF
    detector    ${DS_WIDTH}x${DS_HEIGHT} for each camera, which is also the box
                coordinate space the ROS side reads
    detections  mqtt://$MQTT_HOST:$MQTT_PORT topic $MQTT_TOPIC
    annotated   $([ "$ANNOTATED_ENABLE" = 1 ] && echo "on" || echo "off (ANNOTATED_STREAMS=0)")

    The first run builds a TensorRT engine, which takes one to three minutes.
    The engine is cached in modules/perception/models/cache/.

EOF

start_camera "$PERCEPTION_CAMERA" "$RTSP_IN" 8554 5400
start_camera "$PERCEPTION_CAMERA_2" "$RTSP_IN_2" 8555 5401

# Exit when the first pipeline exits, so a crash is visible to compose rather
# than leaving the container up with half a perception stack.
wait -n $pids
warn "A pipeline exited. Stopping the rest."
shutdown
