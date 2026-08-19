#!/usr/bin/env bash
# Reapply the MAVLink stream rates px4-rcS asks for, for one PX4 instance.
#
# PX4 reconfigures a link's stream set when a ground station appears on it, and
# the rates set at boot fall back to the profile default. Measured on this
# stack: GIMBAL_DEVICE_ATTITUDE_STATUS dropped from the 50 Hz px4-rcS asks for
# to 0.8 Hz, one report every 1.2 s. The camera frame was then over a second
# stale whenever the gimbal slewed, and every detection localized off that
# frame carried the error.
#
# px4-rcS stays the one place the rates are written. This reads the rate and the
# message name back out of it and reissues them against the router link, so a
# reset lasts one interval rather than the rest of the session.
set -uo pipefail

RCS=${PX4_RCS:-/opt/sim/px4-rcS}
INSTANCE=${PX4_INSTANCE:-0}
MAVLINK=${PX4_BUILD_DIR:-/px4/build/px4_sitl_default}/bin/px4-mavlink
INTERVAL_S=${STREAM_RATE_INTERVAL:-10}
ROUTER_PORT=$((14569 + INSTANCE))

log() { printf '[stream-rates uas%s] %s\n' "$((INSTANCE + 1))" "$*"; }

[ -r "$RCS" ] || { log "no $RCS, rates not held"; exit 0; }

# Only the router link. px4-rcS also sets a rate on the rangefinder link, and
# replaying that one here would overwrite the router's DISTANCE_SENSOR rate.
mapfile -t RATES < <(grep -E '^[[:space:]]*mavlink stream .* -u \$router_port$' "$RCS" \
	| sed -E 's/^.*-r[[:space:]]+([0-9]+)[[:space:]]+-s[[:space:]]+([A-Z0-9_]+).*$/\1 \2/')
[ ${#RATES[@]} -gt 0 ] || { log "no router stream lines in $RCS, nothing to hold"; exit 0; }

log "holding ${#RATES[@]} stream rates on port $ROUTER_PORT every ${INTERVAL_S}s"
while :; do
	sleep "$INTERVAL_S"
	[ -x "$MAVLINK" ] || continue
	for rate_and_name in "${RATES[@]}"; do
		read -r rate name <<<"$rate_and_name"
		"$MAVLINK" --instance "$INSTANCE" stream -r "$rate" -s "$name" -u "$ROUTER_PORT" \
			>/dev/null 2>&1 || true
	done
done
