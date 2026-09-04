# shellcheck shell=bash
# The fleet, derived from UAS_FLEET. ./px4sim sources this, and so does
# verify/run.sh, so the two agree on every number by construction.
# docs/uas-contract.md is the specification.

# shellcheck disable=SC2206
FLEET=(${UAS_FLEET:-chimera_v3 chimera_v3 chimera_v2 chimera_v2})
UAS_COUNT=${#FLEET[@]}
# A simulated vehicle is its real counterpart plus ten, so vehicle 1 of a
# simulated fleet is uas11 and vehicle 1 of the real fleet is uas1. This one
# number says which world .env describes. Keep it equal to UAS_BASE in
# compose.yaml and the entry points.
UAS_BASE=${UAS_BASE:-10}
FIRST_UAS=$((UAS_BASE + 1))
LAST_UAS=$((UAS_BASE + UAS_COUNT))
if [ "$UAS_BASE" -ge 10 ]; then FLEET_IS_SIMULATED=true; else FLEET_IS_SIMULATED=false; fi
# Whether .env selects a real machine. px4sim and scripts/preflight.sh both
# ask, so the profile names are written once.
real_profiles_selected() { case ",${COMPOSE_PROFILES:-}," in *,ground,* | *,aircraft,*) return 0 ;; esac; return 1; }
# Whether this machine is the vehicle itself, which serves its own cameras and
# runs its own companion. px4sim and scripts/preflight.sh both ask.
aircraft_selected() { case ",${COMPOSE_PROFILES:-}," in *,aircraft,*) return 0 ;; esac; return 1; }
# The directories compose binds a volume to, made before compose reaches them.
# Compose makes a missing one root-owned, and the uid 1000 container then
# writes nothing into it. Under `sudo ./px4sim start` the mkdir here runs as
# root as well, so hand each directory back to the operator, the way
# scripts/preflight.sh hands back .env.
make_bind_dirs() { # dir...
	mkdir -p "$@" || return 1
	[ -n "${SUDO_UID:-}" ] || return 0
	chown "${SUDO_UID}:${SUDO_GID:-$SUDO_UID}" "$@"
}
# Where the vehicles are. The real fleet flies 10.200.142.6<N> with the
# ground station at .60. The simulated one sits in its own block of the same
# range, or of SIMNET_PREFIX where the host is itself on the radio network.
FLEET_PREFIX=${FLEET_PREFIX:-10.200.142}
SIMNET_PREFIX=${SIMNET_PREFIX:-10.200.142}
# The ground station's domain follows the fleet: 70 beside a simulated fleet,
# 60 beside the real one, so a simulator and the fleet never discover each
# other. modules/offboard/entrypoint.sh derives the same number.
GROUND_DOMAIN=${GROUND_DOMAIN:-$((60 + UAS_BASE))}

fleet_numbers() { seq "$FIRST_UAS" "$LAST_UAS"; }

# The airframe of one vehicle, and what it serves. The mark decides the stream
# names: a v3 carries a gimbal camera and a down camera, a v2 carries the gimbal
# alone and serves it under the down camera's name. See section 3 of
# docs/uas-contract.md and modules/sim/scenes/models/<model>/streams.conf.
model_of() {
	local slot=$(( ${1:-0} - UAS_BASE - 1 ))
	[ "$slot" -ge 0 ] && echo "${FLEET[$slot]:-}"
	return 0
}
mark_of() {
	case "$(model_of "$1")" in
	*_v3) echo v3 ;;
	*_v2) echo v2 ;;
	esac
}
# The full rate gimbal stream, which the detector reads on the vehicle.
gimbal_stream() {
	case "$(mark_of "$1")" in
	v3) echo "rgb$1" ;;
	v2) echo "pilot$1" ;;
	esac
}
# The low rate one, which crosses the radio link and the ground station reads.
# lcam serves the real fleet under exactly these names.
ground_stream() {
	case "$(mark_of "$1")" in
	v3) echo "rgbl$1" ;;
	v2) echo "pilotl$1" ;;
	esac
}
# Every camera an airframe carries, the gimbal camera first.
cameras_of_mark() {
	case "$(mark_of "$1")" in
	v3) echo "rgb pilot thermal" ;;
	v2) echo "pilot thermal" ;;
	esac
}
# Which cameras a vehicle serves, from UAS_STREAMS. A GPU allows a handful of
# encoding sessions at once, so a fleet serves its gimbal cameras alone unless
# it is told otherwise. See .env.example and modules/sim/entrypoint.sh.
# shellcheck disable=SC2206
STREAM_CHOICE=(${UAS_STREAMS:-gimbal})
cameras_of() {
	local slot=$(( $1 - UAS_BASE - 1 ))
	local choice=${STREAM_CHOICE[$slot]:-${STREAM_CHOICE[0]:-gimbal}}
	local all gimbal
	all=$(cameras_of_mark "$1")
	[ -n "$all" ] || return 0
	gimbal=${all%% *}
	case "$choice" in
	all)    echo "$all" ;;
	gimbal) echo "$gimbal" ;;
	*)      echo "$choice" | tr ',' ' ' ;;
	esac
}
# Every stream a vehicle offers this machine. The simulator serves both rates
# from one router, and UAS_STREAMS says how many cameras it encodes. The real
# ground reads lcam, which pulls the low rate of every camera the airframe
# carries, whatever this stack encodes.
streams_of() {
	local camera cameras
	if [ "$FLEET_IS_SIMULATED" = false ] && ! aircraft_selected; then
		cameras=$(cameras_of_mark "$1")
	else
		cameras=$(cameras_of "$1")
	fi
	for camera in $cameras; do
		if [ "$FLEET_IS_SIMULATED" = true ]; then
			printf '%s%s %sl%s ' "$camera" "$1" "$camera" "$1"
		else
			printf '%sl%s ' "$camera" "$1"
		fi
	done
	echo
}
# What rcam serves on the aircraft itself: every camera of this airframe at
# both rates, with no number, because one machine is one vehicle.
aircraft_streams() {
	local camera
	for camera in $(cameras_of_mark "$1"); do
		printf '%s %sl ' "$camera" "$camera"
	done
	echo
}

# Everything else a UAS number decides, as arithmetic. Joining strings reads
# correctly below ten and gives 145511 and 611 above it.
#
# A simulated vehicle publishes its own TCP and Foxglove ports on the host,
# one pair for each. A real vehicle is one machine: its native router serves
# TCP 5760 and its companion serves Foxglove 8765, and so does the ground's.
uas_domain()   { echo "$((60 + $1))"; }
uas_gcs_port() { echo "$((14550 + $1))"; }
uas_address() {
	if [ "$FLEET_IS_SIMULATED" = true ]; then echo "$SIMNET_PREFIX.$((200 + $1))"; else echo "$FLEET_PREFIX.$((60 + $1))"; fi
}
# The ground station of the real fleet. It holds slot zero of the same rule,
# so the real fleet answers on .60 and the vehicles above it.
ground_address() { uas_address 0; }
uas_tcp_port() {
	if [ "$FLEET_IS_SIMULATED" = true ]; then echo "$((5750 + $1))"; else echo 5760; fi
}
uas_foxglove_port() {
	if [ "$FLEET_IS_SIMULATED" = true ]; then echo "$((8760 + $1))"; else echo 8765; fi
}
# Where an operator opens a vehicle's own bridge from this machine.
uas_foxglove_url() {
	if [ "$FLEET_IS_SIMULATED" = true ]; then echo "ws://localhost:$(uas_foxglove_port "$1")"; else echo "ws://$(uas_address "$1"):$(uas_foxglove_port "$1")"; fi
}
# Where this host plays a stream from: the video router's published port in
# the simulator, lcam on the real ground, rcam on the aircraft.
host_rtsp_base() {
	if [ "$FLEET_IS_SIMULATED" = true ]; then
		echo "rtsp://127.0.0.1:${RTSP_HOST_PORT:-8554}"
	else
		echo "${RTSP_BASE:-rtsp://127.0.0.1:8554}"
	fi
}

# The operator's Foxglove console. The shipped file is the simulator's: every
# panel, topic and TF frame in it carries uas11 and d11, so a real fleet
# imports a window of empty panels and reads no error. Render it for this
# fleet's first vehicle instead, and give the operator that copy. A simulated
# fleet renders the bytes it ships, so one path serves both worlds.
LAYOUT_TEMPLATE=${FOXGLOVE_LAYOUT:-${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone/config/foxglove/chimera_sim.json}
LAYOUT_RENDERED=${LAYOUT_RENDERED:-logs/foxglove/chimera_uas$FIRST_UAS.json}
# Prints the file it wrote. Prints the template, and fails, where there is none.
render_layout() {
	[ -f "$LAYOUT_TEMPLATE" ] || { printf '%s\n' "$LAYOUT_TEMPLATE"; return 1; }
	make_bind_dirs "$(dirname "$LAYOUT_RENDERED")" >/dev/null || return 1
	sed -e "s/uas11/uas$FIRST_UAS/g" -e "s/d11_/d${FIRST_UAS}_/g" \
		"$LAYOUT_TEMPLATE" > "$LAYOUT_RENDERED" || return 1
	printf '%s\n' "$LAYOUT_RENDERED"
}
