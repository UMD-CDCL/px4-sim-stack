#!/usr/bin/env bash
# Build the selected stack, then launch it.
#
# A stack is a directory under modules/ros/stacks/. It holds ROS packages and
# one stack.launch.py at its root. Switching stacks is one variable:
#   ROS_STACK=my_stack docker compose up -d ros
# No `set -u` here. The ROS setup files read AMENT_TRACE_SETUP_FILES and several
# other variables without a default, so `set -u` stops the script on the first
# `source`. colcon sources them again at build time, so the problem would come
# back later even if this script worked around it once.
set -o pipefail

ROS_DISTRO=${ROS_DISTRO:-jazzy}
ROS_STACK=${ROS_STACK:-baseline}
STACK_DIR=/stacks/$ROS_STACK
AUTOLAUNCH=${ROS_AUTOLAUNCH:-1}

log()  { printf '\n\033[1m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[33m    %s\033[0m\n' "$*"; }

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"

idle() {
	warn "The container stays up so you can debug. Get a shell with: make ros"
	exec sleep infinity
}

if [ ! -d "$STACK_DIR" ]; then
	warn "No stack named '$ROS_STACK'. Available: $(ls /stacks 2>/dev/null | tr '\n' ' ')"
	idle
fi

# ---------------------------------------------------------------- build
# The workspace lives on the host at ./src/ros2_ws, so build output survives a
# container restart and your editor sees the same tree.
mkdir -p /ws/src
ln -sfn "$STACK_DIR" "/ws/src/$ROS_STACK"

log "Building stack '$ROS_STACK'"
cd /ws
if ! colcon build --symlink-install --event-handlers console_direct+ \
       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo; then
	warn "colcon build failed."
	idle
fi

# shellcheck disable=SC1091
source /ws/install/setup.bash

# ---------------------------------------------------------------- report
log "Stack '$ROS_STACK' is built"
cat <<EOF
    Drone interface, and nothing else:
      MAVLink     ${FCU_URL:-unset}
      Video       ${RTSP_BASE:-unset}/gimbal , ${RTSP_BASE:-unset}/nadir
      Detections  mqtt://${MQTT_HOST:-unset}:1883  topic perception/detections
    ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
EOF

if [ "$AUTOLAUNCH" != "1" ]; then
	log "ROS_AUTOLAUNCH=0, not starting the stack"
	idle
fi

LAUNCH_FILE=$STACK_DIR/stack.launch.py
if [ ! -f "$LAUNCH_FILE" ]; then
	warn "Stack '$ROS_STACK' has no stack.launch.py at its root."
	idle
fi

log "Launching $LAUNCH_FILE"
ros2 launch "$LAUNCH_FILE" || warn "The launch exited with status $?."
idle
