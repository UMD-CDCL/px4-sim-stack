#!/bin/sh
# Render main.conf.template for one vehicle, then run the router.
set -eu

UAS_NUM=${UAS_NUM:?set UAS_NUM to the vehicle number}
# A real vehicle is 1 to 9. A simulated one is 11 to 19, so a simulator can fly
# beside the fleet without taking a system id, a port or an address from it.
case "$UAS_NUM" in
[1-9]|1[1-9]) ;;
*) echo "UAS_NUM must be 1 to 9 for a real vehicle or 11 to 19 for a simulated one, not '$UAS_NUM'." >&2; exit 1 ;;
esac
# 14551 to 14559 for the fleet, 14561 to 14569 for the simulator. The string
# form 1455${UAS_NUM} only works below ten.
GCS_PORT=$((14550 + UAS_NUM))
PX4_UPLINK_PORT=${PX4_UPLINK_PORT:-14545}
GCS_ADDRESS=${GCS_ADDRESS:-10.200.142.160}
ONBOARD_ADDRESS=${ONBOARD_ADDRESS:-127.0.0.1}
QGC_ADDRESS=${QGC_ADDRESS:-}
TEMPLATE=/opt/mavlink-router/main.conf.template
CONF=/tmp/mavlink-router.conf

# mavlink-router parses Address as an IP literal and rejects a name with
# "Invalid IP address". Compose gives every service a fixed address for that
# reason, so the usual path needs no lookup. A name still works here, and costs
# a wait for the other container to join the network.
resolve() {
	case "$1" in
		*[!0-9.]*) ;;              # holds a letter, so it is a name
		*) echo "$1"; return 0 ;;  # already an address
	esac
	i=0
	while [ "$i" -lt 30 ]; do
		addr=$(getent hosts "$1" 2>/dev/null | awk '{print $1; exit}')
		if [ -n "$addr" ]; then echo "$addr"; return 0; fi
		i=$((i + 1))
		sleep 1
	done
	return 1
}

GCS_ADDRESS=$(resolve "$GCS_ADDRESS") || {
	echo "Cannot resolve GCS_ADDRESS. The ground station endpoint would be invalid." >&2
	exit 1
}
ONBOARD_ADDRESS=$(resolve "$ONBOARD_ADDRESS") || {
	echo "Cannot resolve ONBOARD_ADDRESS. The companion endpoint would be invalid." >&2
	exit 1
}

export UAS_NUM PX4_UPLINK_PORT GCS_ADDRESS ONBOARD_ADDRESS GCS_PORT
envsubst '${UAS_NUM} ${PX4_UPLINK_PORT} ${GCS_ADDRESS} ${ONBOARD_ADDRESS} ${GCS_PORT}' \
	< "$TEMPLATE" > "$CONF"

if [ -n "$QGC_ADDRESS" ]; then
	QGC_ADDRESS=$(resolve "$QGC_ADDRESS") || QGC_ADDRESS=""
fi
if [ -n "$QGC_ADDRESS" ]; then
	cat >> "$CONF" <<-EOF

		# QGroundControl autoconnects by listening on 14550, so push to it. Every
		# vehicle pushes to the same instance, which is how one ground station
		# shows a fleet. Not on the aircraft, and not part of the contract.
		[UdpEndpoint qgc]
		Mode = Normal
		Address = $QGC_ADDRESS
		Port = 14550
	EOF
fi

echo "=== mavlink-router config, uas$UAS_NUM ==="
cat "$CONF"
echo "=========================================="

case "${KEEPALIVE:-1}" in
0|false|no|off) ;;
*) python3 /usr/local/bin/keepalive.py --host 127.0.0.1 --port 14599 & ;;
esac

exec mavlink-routerd -c "$CONF"
