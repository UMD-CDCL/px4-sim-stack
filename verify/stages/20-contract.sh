# shellcheck shell=bash
# compose.yaml agrees with the fleet arithmetic: addresses, ports, domains
#
# Every one of these follows from the UAS number. A hand-written copy that
# drifts starts a healthy looking container that talks to nobody.

cfg=$(mktemp)
profiles=""
for n in $(fleet_numbers); do profiles="$profiles,uas$n,onboard$n"; done
COMPOSE_PROFILES="${profiles#,},offboard" docker compose config --format json >"$cfg" 2>/dev/null ||
	{ fail "docker compose config resolves"; return 0; }

fact() { python3 verify/compose_fact.py "$cfg" "$@"; }
endpoint_port() {
	awk -v want="[UdpEndpoint $1]" '$0 == want { inside = 1; next }
	     /^\[/ { inside = 0 }
	     inside && $1 == "Port" { print $3 }'
}

for n in $(fleet_numbers); do
	expect_eq "uas$n address"          "$(uas_address "$n")"       "$(fact address uas"$n")"
	expect_eq "uas$n MAVLink TCP port" "$(uas_tcp_port "$n")"      "$(fact port uas"$n" 5760)"
	expect_eq "uas$n Foxglove port"    "$(uas_foxglove_port "$n")" "$(fact port uas"$n" 8765)"
	expect_eq "uas$n system id"        "$n"                        "$(fact env uas"$n" UAS_NUM)"
	expect_eq "onboard$n is uas$n"     "service:uas$n"             "$(fact netmode onboard"$n")"
	expect_eq "onboard$n system id"    "$n"                        "$(fact env onboard"$n" UAS_NUM)"
	expect_eq "ground station takes uas$n on $(uas_gcs_port "$n")" \
		yes "$(fact publishes offboard "$(uas_gcs_port "$n")")"
done

expect_eq "ground station address" "$SIMNET_PREFIX.210" "$(fact address offboard)"
expect_eq "ground router is the ground station" "service:offboard" "$(fact netmode ground-router)"

# The router config is rendered from one template for every vehicle. A vehicle
# that filters on the wrong system id passes another vehicle's telemetry
# through, which reads as one aircraft in two places.
router_image=$(python3 -c "
import json; print(json.load(open('$cfg'))['services']['uas$FIRST_UAS']['image'])")
if docker image inspect "$router_image" >/dev/null 2>&1; then
	for n in $(fleet_numbers); do
		conf=$(docker run --rm --entrypoint sh \
			-e UAS_NUM="$n" -e GCS_ADDRESS="$SIMNET_PREFIX.210" \
			-e ONBOARD_ADDRESS=127.0.0.1 -e KEEPALIVE=0 "$router_image" -c '
				GCS_PORT=$((14550 + UAS_NUM)) PX4_UPLINK_PORT=14545 \
				envsubst "\${UAS_NUM} \${PX4_UPLINK_PORT} \${GCS_ADDRESS} \${ONBOARD_ADDRESS} \${GCS_PORT}" \
					< /opt/mavlink-router/main.conf.template')
		expect_eq "uas$n router sends to the ground station on $(uas_gcs_port "$n")" \
			"$(uas_gcs_port "$n")" "$(printf '%s' "$conf" | endpoint_port alpha)"
		expect_eq "uas$n router passes only system $n and 255" \
			2 "$(printf '%s' "$conf" | grep -c "^AllowSrcSysIn = $n,255$")"
		expect_eq "uas$n router serves the companion on 14402" \
			14402 "$(printf '%s' "$conf" | endpoint_port ros)"
	done
else
	fail "$router_image is not built. Run ./px4sim restart"
fi

if output=$(python3 -m unittest verify.test_tui_helpers -q 2>&1); then
	pass "px4sim UI helper behavior"
else
	fail "px4sim UI helper behavior"
	note "$(printf '%s' "$output" | tail -5 | tr '\n' ' ')"
fi

# QGroundControl owns a process-wide singleton lock. A second GUI instance is
# not an independent service; it is a launch error that can leave a container
# looking Up while the operator's original window owns the session.
qgc_id=$(docker compose ps -q qgc 2>/dev/null)
if [ -n "$qgc_id" ] && [ "$(docker inspect -f '{{.State.Status}}' "$qgc_id" 2>/dev/null)" = running ]; then
	qgc_processes=$(docker exec "$qgc_id" sh -lc \
		"pgrep -c -f '^/opt/qgc/usr/bin/QGroundControl$'" 2>/dev/null || printf 0)
	expect_eq "QGroundControl has one active process" 1 "$qgc_processes"
else
	skip "QGroundControl is not running, so singleton process count is unavailable"
fi

# Recording is a front-door lifecycle, not an undocumented docker exec. Keep
# its safety and persistence contract visible to the code stage.
if grep -q 'record)' px4sim \
	&& grep -q 'recording name contains unsafe characters' px4sim \
	&& grep -q 'RECORD_STORAGE' px4sim \
	&& grep -q 'stop \[name\]' px4sim \
	&& grep -q './logs/recordings:/recordings' compose.yaml; then
	pass "px4sim recording front door and persistent mount"
else
	fail "px4sim recording front door and persistent mount"
fi

rm -f "$cfg"
