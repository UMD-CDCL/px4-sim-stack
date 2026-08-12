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

# Make the advertised URLs work verbatim inside this container.
#
# Everything this stack prints uses host addresses: rtsp://localhost:8554/gimbal
# is what `px4sim endpoints`, the README and the video player all show. Typed
# into QGroundControl's video settings that address fails, because in here
# localhost is QGroundControl itself and the video router is a different
# container. The symptom is a video panel that stays black with no error worth
# reading.
#
# Forwarding the loopback port to the router removes the trap, so both
# rtsp://localhost:8554/... and rtsp://video-router:8554/... work. RTSP here is
# TCP only, so a plain TCP forward carries the whole session, control and
# interleaved media together.
# Do not use `set --` here: this script ends with `exec AppRun "$@"`, and
# rewriting the positional parameters would hand QGroundControl the loop
# variables as command line arguments.
router=${VIDEO_ROUTER_HOST:-video-router}
for port in 8554 1935; do
	socat "TCP4-LISTEN:${port},fork,reuseaddr,bind=127.0.0.1" \
	      "TCP4:${router}:${port}" >/dev/null 2>&1 &
done
echo "Loopback forwards up: localhost:8554 and localhost:1935 reach the video router."

echo "QGroundControl listens for MAVLink on UDP $LISTEN_PORT."
echo "The MAVLink hub pushes telemetry there, so the vehicle appears on its own."
echo "Video source: $VIDEO_URL"

exec /opt/qgc/AppRun "$@"
