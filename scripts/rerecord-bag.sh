#!/usr/bin/env bash
# Replay one raw MAVLink bag and its independently recorded RGB video through
# the same companion container used by the selected aircraft.
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
cd "$ROOT"

usage() {
	cat >&2 <<'EOF'
Usage: ./px4sim rerecord <merged-directory> [uas-number]

The directory must contain one *mavlink.mcap and one *video*.mp4.  The video
offset is read from its t+<seconds>s_<milliseconds>ms filename prefix.
EOF
	exit 2
}

DIR=${1:-}; UAS=${2:-4}
[ -d "$DIR" ] || usage
MCAP=$(find "$DIR" -maxdepth 1 -type f -name '*mavlink.mcap' -print -quit)
VIDEO=$(find "$DIR" -maxdepth 1 -type f \( -name '*video*.mp4' -o -name '*video*.MP4' \) -print -quit)
[ -n "$MCAP" ] && [ -n "$VIDEO" ] || usage

offset=$(basename "$VIDEO" | sed -nE 's/^t\+([0-9]+)s_([0-9]+)ms-.*/\1.\2/p')
[ -n "$offset" ] || { echo "video filename has no t+<s>s_<ms>ms prefix: $VIDEO" >&2; exit 2; }

case "$UAS" in (1|2|3|4|11|12|13|14) ;; (*) echo "unsupported UAS number: $UAS" >&2; exit 2 ;; esac

OUT=${RERECORD_OUT:-"$DIR/rerecorded-uas${UAS}-$(date -u +%Y%m%dT%H%M%SZ)"}
mkdir -p "$OUT"
echo "mcap:   $MCAP"
echo "video:  $VIDEO"
echo "offset: ${offset}s"
echo "output: $OUT"

container="onboard${UAS}"
docker compose ps --status running --services | grep -qx "$container" || {
	echo "$container is not running; start the laptop stack first with ./px4sim start" >&2
	exit 1
}

# Use the container's already-built UAS4 image and its native ROS graph. The
# video source is started after the filename-derived offset while the bag is
# replayed at maximum throughput. rosbag2 preserves each MCAP timestamp and
# --clock makes all downstream nodes use that same timeline.
docker exec "$container" mkdir -p /rerecord
docker cp "$VIDEO" "$container:/rerecord/input.mp4"
docker cp "$MCAP" "$container:/rerecord/input.mcap"

docker exec "$container" bash -lc "
  set -euo pipefail
  . /usr/local/bin/ros-env.sh
  rm -rf /rerecord/result
  mkdir -p /rerecord/result
  ros2 bag record -s mcap -a -o /rerecord/result > /rerecord/record.log 2>&1 &
  recorder=\$!
  ros2 bag play /rerecord/input.mcap --clock --read-ahead-queue-size 10000 > /rerecord/play.log 2>&1 &
  player=\$!
  sleep '$offset'
  ros2 run umd_uas ds_node --ros-args \\
    -p source.uri:=file:///rerecord/input.mp4 \\
    -p source.loop:=false \\
    -p model.detector:=yolo26l-zach-960 \\
    -p continuous.stride:=1 \\
    > /rerecord/video.log 2>&1 &
  video=\$!
  wait \$player || true
  wait \$video || true
  kill -INT \$recorder 2>/dev/null || true
  wait \$recorder || true
" 

docker cp "$container:/rerecord/result" "$OUT/"
docker cp "$container:/rerecord/record.log" "$OUT/" || true
docker cp "$container:/rerecord/play.log" "$OUT/" || true
docker cp "$container:/rerecord/video.log" "$OUT/" || true
echo "replay complete: $OUT/result"
