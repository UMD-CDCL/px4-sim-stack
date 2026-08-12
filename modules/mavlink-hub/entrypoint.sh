#!/bin/sh
# Write the router config from the environment, then run the router.
set -eu

PX4_UPLINK_PORT=${PX4_UPLINK_PORT:-14545}
QGC_HOST=${QGC_HOST:-172.28.0.14}
QGC_PORT=${QGC_PORT:-14550}
EXTRA_GCS_IP=${EXTRA_GCS_IP:-}
CONF=/tmp/mavlink-router.conf

# mavlink-router parses Address as an IP literal and rejects a name with
# "Invalid IP address". Compose gives every service a fixed address on simnet
# for that reason, so the usual path needs no lookup at all. A name still works
# here, and costs a wait for the other container to join the network.
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

QGC_ADDR=$(resolve "$QGC_HOST") || {
	echo "Cannot resolve QGC_HOST '$QGC_HOST'. Skipping the ground station endpoint." >&2
	QGC_ADDR=""
}

cat > "$CONF" <<EOF
[General]
TCPServerPort=5760
ReportStats=false
MavlinkDialect=auto
DebugLogLevel=info

# ---------------------------------------------------------------- the vehicle
# Server mode. PX4 pushes to this port from a link that modules/sim/px4-rcS
# starts with a fixed partner address, so the router never has to dial out and
# never has to guess. Either side can restart in any order.
[UdpEndpoint px4]
Mode=Server
Address=0.0.0.0
Port=$PX4_UPLINK_PORT

# ---------------------------------------------------------------- the autonomy
# Server mode: MAVROS connects and the router replies to wherever it came from.
[UdpEndpoint ros]
Mode=Server
Address=0.0.0.0
Port=14551

# ------------------------------------------------------------------- free slot
# For pymavlink, MAVSDK or a mission script.
[UdpEndpoint tools]
Mode=Server
Address=0.0.0.0
Port=14552

# ------------------------------------------------------------------- keepalive
# A private endpoint for keepalive.py. It gets its own port so that the
# heartbeat never competes with a real client for the tools endpoint.
[UdpEndpoint keepalive]
Mode=Server
Address=127.0.0.1
Port=14553
EOF

if [ -n "$QGC_ADDR" ]; then
	cat >> "$CONF" <<EOF

# ------------------------------------------------------------------- the GCS
# QGroundControl autoconnects by listening on 14550, so the router pushes to it.
[UdpEndpoint qgc]
Mode=Normal
Address=$QGC_ADDR
Port=$QGC_PORT
EOF
fi

if [ -n "$EXTRA_GCS_IP" ]; then
	cat >> "$CONF" <<EOF

# A GCS outside the compose network, from HOST_GCS_IP.
[UdpEndpoint external_gcs]
Mode=Normal
Address=$EXTRA_GCS_IP
Port=14550
EOF
fi

echo "=== mavlink-router config ==="
cat "$CONF"
echo "============================="

if [ "${KEEPALIVE:-1}" = "1" ]; then
	# PX4 now pushes telemetry on its own, so this is no longer needed to
	# start the link. It keeps a ground station heartbeat present, which stops
	# the PX4 data link loss failsafe from firing when no ground station is
	# attached. Set KEEPALIVE=0 to test that failsafe.
	python3 /usr/local/bin/keepalive.py --host 127.0.0.1 --port 14553 &
fi

exec mavlink-routerd -c "$CONF"
