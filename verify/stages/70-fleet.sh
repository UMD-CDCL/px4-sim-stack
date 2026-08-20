# shellcheck shell=bash
# Every vehicle over one target reports one position for it
#
# The fleet shares a ground: each vehicle casts its rays at the same surface,
# anchored the same way, so two aircraft over one casualty agree about where it
# is. Anchored on each vehicle's own frame instead, they disagree by the
# difference between their homes, which is unbounded.
#
# How closely they agree depends on what else the machine is doing. Four
# vehicles serving two cameras each run this one at a tenth of real time, and
# everything that depends on a timestamp loosens with it: sub-metre with eight
# streams, a few metres with twelve. FLEET_AGREEMENT_M is the line between
# "agrees" and "does not share a ground", not a measure of the best it does.

if [ "$UAS_COUNT" -lt 2 ]; then
	skip "this fleet has one vehicle, and this asks what two make of one target"
	note "fly more: UAS_FLEET='chimera_v3 chimera_v3 chimera_v2 chimera_v2' ./px4sim start"
	return 0
fi
for n in $(fleet_numbers); do
	if [ -z "$(COMPOSE_PROFILES="uas$n,onboard$n" docker compose ps -q "onboard$n" 2>/dev/null)" ]; then
		fail "uas$n is not running. Start it: ./px4sim start"
		return 0
	fi
done

# Every vehicle aimed at one target, from a different place. The campus
# scenario puts casualty_m14 111 m north of the origin. Each takes its own
# offset and height, and its heading and gimbal angle are worked out from
# those, because a gimbal's yaw is locked to the airframe: pointing the camera
# at something means pointing the vehicle at it first.
#
# Flown at the same time, not one after another: four vehicles in turn is a
# quarter of an hour of waiting, and they do not interfere with each other.
# Getting there is setup, so a vehicle that arrives late still answers the
# question this stage asks.
for n in $(fleet_numbers); do
	vehicle_ready "$n" || return 0
done

TARGET_EAST=${TARGET_EAST:-0.69}
TARGET_NORTH=${TARGET_NORTH:-111.23}
fly() { ./px4sim uas "$@" >/dev/null 2>&1 || true; }
station() {
	local n=$1 east=$2 north=$3 up=$4 heading=$5 depression=$6
	flying "$n" 30 || true
	fly "$n" goto "$east" "$north" "$up" --heading "$heading"
	fly "$n" gimbal "-$depression"
	fly "$n" detect on
}
work=$(mktemp -d)
slot=0
for n in $(fleet_numbers); do
	read -r east north up heading depression < <(python3 -c "
import math
slot = $slot
east = $TARGET_EAST + (slot - 1.5) * 4
north = $TARGET_NORTH - (22 + slot * 4)
up = 12 + slot * 4
away = math.hypot($TARGET_EAST - east, $TARGET_NORTH - north)
print(east, north, up,
      math.degrees(math.atan2($TARGET_EAST - east, $TARGET_NORTH - north)) % 360,
      math.degrees(math.atan2(up, away)))")
	station "$n" "$east" "$north" "$up" "$heading" "$depression" &
	slot=$((slot + 1))
done
wait

# Settle before asking. A localization taken while the gimbal is still slewing
# is computed against a pose the camera has already left, and one of those in
# the sample is enough to make two vehicles look like they disagree.
sleep "${FLEET_SETTLE_S:-15}"

for n in $(fleet_numbers); do
	# What the vehicle publishes, not what it answers when asked: the ground
	# reads the topic, and a service call can miss a stream that is flowing.
	./px4sim uas "$n" published --named --seconds 12 >"$work/$n" 2>/dev/null || true
	if [ -s "$work/$n" ]; then
		pass "uas$n localizes $(wc -l < "$work/$n") targets"
	else
		fail "uas$n localizes something"
	fi
done

verdict=$(python3 verify/component/compare_fleet.py "${FLEET_AGREEMENT_M:-8.0}" "$work"/* 2>&1) || true
if python3 verify/component/compare_fleet.py "${FLEET_AGREEMENT_M:-8.0}" "$work"/* >/dev/null 2>&1; then
	pass "$verdict"
else
	fail "the fleet agrees about where a target is"
	note "$verdict"
fi
rm -rf "$work"
