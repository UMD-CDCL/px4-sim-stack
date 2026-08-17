#!/usr/bin/env bash
# Reapply the MAVLink stream rates px4-rcS asks for.
#
# PX4 reconfigures a link's stream set when a ground station appears on it, and
# the rates set at boot fall back to the profile default. Measured on this
# stack: GIMBAL_DEVICE_ATTITUDE_STATUS dropped from the 50 Hz px4-rcS asks for
# to 0.8 Hz, one report every 1.2 s. The camera frame was then over a second
# stale whenever the gimbal slewed, and every detection localized off that
# frame carried the error.
#
# px4-rcS stays the one place the rates are written. This reads them back out
# of it and reissues them, so a reset lasts one interval rather than the rest
# of the session.
set -uo pipefail

RCS=${PX4_RCS:-/opt/sim/px4-rcS}
MAVLINK=${PX4_BUILD_DIR:-/px4/build/px4_sitl_default}/bin/px4-mavlink
INTERVAL_S=${STREAM_RATE_INTERVAL:-10}

log() { printf '[stream-rates] %s\n' "$*"; }

[ -r "$RCS" ] || { log "no $RCS, rates not held"; exit 0; }
mapfile -t COMMANDS < <(grep -E '^[[:space:]]*mavlink stream ' "$RCS")
[ ${#COMMANDS[@]} -gt 0 ] || { log "no stream lines in $RCS, nothing to hold"; exit 0; }

log "holding ${#COMMANDS[@]} stream rates from $RCS every ${INTERVAL_S}s"
while :; do
	sleep "$INTERVAL_S"
	[ -x "$MAVLINK" ] || continue
	for line in "${COMMANDS[@]}"; do
		# Word splitting is the point: the line is already an argument list.
		# shellcheck disable=SC2086
		"$MAVLINK" ${line#*mavlink } >/dev/null 2>&1 || true
	done
done
