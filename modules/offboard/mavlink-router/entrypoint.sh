#!/bin/sh
# The ground station router: every vehicle in, QGroundControl and one MAVROS
# for each vehicle out.
#
# Taken from chimera-deploy/local/main.conf, which is what runs on the fielded
# ground station. The fleet decides how many endpoints there are, so the
# configuration is written here rather than kept as a file with four vehicles
# in it. UAS_FLEET names one airframe for each vehicle, and only the count is
# read.
#
# One port for every role, on the ground exactly as on the aircraft: MAVROS
# reads 14402 and QGroundControl reads 14401, whichever vehicle is in play. A
# UDP port holds one listener per address, not per host, and the whole of
# 127.0.0.0/8 is local, so each vehicle gets its own loopback address and they
# all keep the same port. uas<N> answers on 127.0.0.<N>.
#
# Each vehicle endpoint also filters on its own system id. The vehicle router
# already filtered on the way out, so this cannot add traffic. It does keep one
# vehicle's telemetry out of another vehicle's MAVROS, which matters here
# because every vehicle now shares a port number.
#
# Two differences from the fielded file, and one is an absence:
#
#   - No [UdpEndpoint f11]. Nothing in this stack produces that payload.
#   - The fielded file gives each vehicle its own port, 14402 upwards, because
#     it was written for one address. The loopback address does that job here.
#
# The MAVROS endpoints are loopback, so this router must share a network
# namespace with the ground station container. compose does that with
# network_mode.
set -eu

UAS_FLEET=${UAS_FLEET:-chimera_v3 chimera_v3 chimera_v2 chimera_v2}
QGC_PORT=${QGC_PORT:-14401}
# The same port the MAVROS on the aircraft binds. The launch file binds the
# other end of this: 5g_drone/launch/offboard.launch.py.
MAVROS_PORT=${MAVROS_PORT:-14402}
CONF=/tmp/ground-router.conf

COUNT=0
for airframe in ${UAS_FLEET}; do
	COUNT=$((COUNT + 1))
done

if [ "$COUNT" -lt 1 ] || [ "$COUNT" -gt 9 ]; then
	echo "UAS_FLEET has $COUNT vehicles. The simulator numbers them 11 to 19." >&2
	exit 1
fi

cat > "$CONF" <<-EOF
	[General]
	TcpServerPort = 5760
	ReportStats = false
	DebugLogLevel = info

	# QGroundControl. Loopback, so it reads the fleet through this router when
	# it runs beside it.
	[UdpEndpoint qgc]
	Mode = Normal
	Address = 127.0.0.1
	Port = ${QGC_PORT}
EOF

N=${UAS_BASE:-10}
N=$((N + 1))
while [ "$N" -le "$((${UAS_BASE:-10} + COUNT))" ]; do
	cat >> "$CONF" <<-EOF

		# uas${N}: in from the vehicle router, out to the MAVROS for it.
		[UdpEndpoint uas${N}]
		Mode = Server
		Address = 0.0.0.0
		Port = $((14550 + N))
		AllowSrcSysIn = ${N},255

		[UdpEndpoint mavros${N}]
		Mode = Normal
		Address = 127.0.0.${N}
		Port = ${MAVROS_PORT}
		AllowSrcSysIn = ${N},255
	EOF
	N=$((N + 1))
done

echo "=== mavlink-router config, ground station ==="
cat "$CONF"
echo "============================================="

exec mavlink-routerd -c "$CONF"
