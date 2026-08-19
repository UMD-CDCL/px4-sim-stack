#!/bin/sh
# Save one frame of a live stream, so a picture can be looked at rather than
# guessed about. Runs inside the sim container, which is on the video network
# and already carries gstreamer.
#
#   snap.sh <stream> <output>
set -eu
stream=$1
output=$2
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

# The encoder picks H.265 where the GPU offers it and H.264 otherwise, so
# decode whatever arrives rather than naming one.
timeout "${SNAP_SECONDS:-20}" gst-launch-1.0 -q \
	rtspsrc "location=rtsp://video-router:8554/$stream" latency=100 \
	! decodebin ! videoconvert ! jpegenc \
	! multifilesink "location=$work/frame_%04d.jpg" >/dev/null 2>&1 || true

newest=$(ls -1 "$work"/frame_*.jpg 2>/dev/null | tail -1)
[ -n "$newest" ] || { echo "no frame arrived on $stream" >&2; exit 1; }
cp "$newest" "$output"
echo "$output"
