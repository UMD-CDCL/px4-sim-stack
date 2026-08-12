#!/usr/bin/env bash
# Seed the settings on first start, then run QGroundControl.
set -euo pipefail

VIDEO_URL=${QGC_VIDEO_URL:-rtsp://video-router:8554/gimbal}
LISTEN_PORT=${QGC_UDP_PORT:-14550}

# QGroundControl 5 stores settings under the organization name "QGroundControl".
# Version 4 used "QGroundControl.org". Seed both, so a change of QGC_REF does not
# silently produce an unconfigured window.
seed_settings() {
	local ini=$1
	[ -f "$ini" ] && return 0   # Never overwrite what the user changed.
	mkdir -p "$(dirname "$ini")"
	cat > "$ini" <<EOF
[AutoConnect]
autoConnectUDP=true
udpListenPort=$LISTEN_PORT
autoConnectPixhawk=false
autoConnectSiKRadio=false
autoConnectRTKGPS=false
autoConnectLibrePilot=false

[Video]
videoSource=RTSP Video Stream
rtspUrl=$VIDEO_URL
lowLatencyMode=true
rtspTimeout=10
streamEnabled=true
disableWhenDisarmed=false
EOF
	echo "seeded $ini"
}

seed_settings "$HOME/.config/QGroundControl/QGroundControl.ini"
seed_settings "$HOME/.config/QGroundControl.org/QGroundControl.ini"

echo "QGroundControl listens for MAVLink on UDP $LISTEN_PORT."
echo "The MAVLink hub pushes telemetry there, so the vehicle appears on its own."
echo "Video source: $VIDEO_URL"

exec /opt/qgc/AppRun "$@"
