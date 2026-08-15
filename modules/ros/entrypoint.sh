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
FORCE_BUILD=${ROS_FORCE_BUILD:-0}
STAMP=/ws/install/.stamp

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
# container restart and your editor sees the same tree. A restart with an
# unchanged tree therefore needs no build at all. The stamp records which
# stack built last, and find looks for any source newer than it. -L follows
# the stack symlink and covers extra packages you put in /ws/src yourself.
mkdir -p /ws/src
# Re-link only when the link is wrong: ln -sfn always recreates the link,
# which touches /ws/src and would read as a change on every boot.
[ "$(readlink "/ws/src/$ROS_STACK" 2>/dev/null)" = "$STACK_DIR" ] \
	|| ln -sfn "$STACK_DIR" "/ws/src/$ROS_STACK"

build_is_current() {
	[ "$FORCE_BUILD" != "1" ] || return 1
	[ -f /ws/install/setup.bash ] || return 1
	[ -f "$STAMP" ] || return 1
	[ "$(cat "$STAMP")" = "$ROS_STACK" ] || return 1
	# Bytecode caches are written at run time, so they do not count.
	[ -z "$(find -L /ws/src -name __pycache__ -prune -o -newer "$STAMP" \
		-print -quit)" ]
}

cd /ws
if build_is_current; then
	log "Stack '$ROS_STACK' is unchanged, not building. ROS_FORCE_BUILD=1 builds anyway."
else
	log "Building stack '$ROS_STACK'"
	# The stamp carries the pre-build time, so a file edited while colcon
	# runs still reads as newer on the next boot.
	build_started=$(mktemp)
	if ! colcon build --symlink-install --event-handlers console_direct+ \
	       --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo; then
		rm -f "$build_started"
		warn "colcon build failed."
		idle
	fi
	echo "$ROS_STACK" > "$STAMP"
	touch -r "$build_started" "$STAMP"
	rm -f "$build_started"
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
