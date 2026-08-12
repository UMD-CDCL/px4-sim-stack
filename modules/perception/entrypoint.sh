#!/usr/bin/env bash
# Wait for the video stream, expand the config, then run deepstream-app.
set -euo pipefail

DS_CONFIG=${DS_CONFIG:-gimbal_detector.txt}
SRC=/opt/perception/configs/$DS_CONFIG
RUN_DIR=/tmp/perception
RTSP_IN=${RTSP_IN:-rtsp://video-router:8554/gimbal}
MQTT_HOST=${MQTT_HOST:-message-bus}
MQTT_PORT=${MQTT_PORT:-1883}
MQTT_TOPIC=${MQTT_TOPIC:-perception/detections}

export RTSP_IN MQTT_HOST MQTT_PORT MQTT_TOPIC

log()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }

[ -f "$SRC" ] || {
	echo "No config at $SRC. Available:"
	ls /opt/perception/configs/
	exit 1
}

# deepstream-app resolves relative paths against the config file's directory,
# so the whole directory is copied and expanded together.
rm -rf "$RUN_DIR"
mkdir -p "$RUN_DIR"
for f in /opt/perception/configs/*; do
	[ -f "$f" ] || continue
	envsubst < "$f" > "$RUN_DIR/$(basename "$f")"
done

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

# nvmsgconv writes payloads here, the forwarder publishes and deletes them.
mkdir -p /tmp/ds-payloads
if [ "${PAYLOAD_FORWARDER:-1}" = "1" ]; then
	python3 /opt/perception/app/payload_forwarder.py \
		--dir /tmp/ds-payloads --host "$MQTT_HOST" --port "$MQTT_PORT" \
		--topic "$MQTT_TOPIC" --poll 0.02 &
fi

log "Starting deepstream-app with $DS_CONFIG"
cat <<EOF
    source      $RTSP_IN
    detections  mqtt://$MQTT_HOST:$MQTT_PORT topic $MQTT_TOPIC
    annotated   rtsp://perception:8554/ds-test
                also served as rtsp://localhost:8554/gimbal_annotated

    The first run builds a TensorRT engine, which takes one to three minutes.
    The engine is cached in modules/perception/models/cache/.
EOF

cd "$RUN_DIR"
exec deepstream-app -c "$RUN_DIR/$DS_CONFIG"
