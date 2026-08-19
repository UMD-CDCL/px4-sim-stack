# shellcheck shell=bash
# Two vehicles over one target report one position for it
#
# The fleet shares a ground: every vehicle casts its rays at the same surface,
# anchored the same way, so two aircraft over one casualty agree about where it
# is. Anchored on each vehicle's own frame instead, they disagree by the
# difference between their homes.

if [ "$UAS_COUNT" -lt 2 ]; then
	fail "this fleet has one vehicle. Fly two: UAS_FLEET='chimera_v3 chimera_v2' ./px4sim start"
	return 0
fi

first=$FIRST_UAS
second=$((FIRST_UAS + 1))
for n in "$first" "$second"; do
	if [ -z "$(COMPOSE_PROFILES="uas$n,onboard$n" docker compose ps -q "onboard$n" 2>/dev/null)" ]; then
		fail "uas$n is not running. Start it: ./px4sim start"
		return 0
	fi
done

# Both south of one casualty, at different offsets and heights, so the two see
# it down different rays. casualty_m14 is 111 m north of the origin.
# Getting there is setup, not the check: a vehicle that arrives late still
# answers the question, and a failure here would otherwise end the stage with
# nothing said.
fly() { ./px4sim uas "$@" >/dev/null 2>&1 || true; }
fly "$first" takeoff 30
fly "$second" takeoff 30
fly "$first" goto 0 91 12
fly "$second" goto -18 96 20
fly "$first" gimbal -30
fly "$second" gimbal -42
fly "$first" detect on
fly "$second" detect on

work=$(mktemp -d)
./px4sim uas "$first" detections --tsv >"$work/$first" 2>/dev/null || true
./px4sim uas "$second" detections --tsv >"$work/$second" 2>/dev/null || true

for n in "$first" "$second"; do
	if [ -s "$work/$n" ]; then
		pass "uas$n localizes $(wc -l < "$work/$n") targets"
	else
		fail "uas$n localizes something"
	fi
done

verdict=$(python3 verify/component/compare_fleet.py "$work/$first" "$work/$second" \
	"${FLEET_AGREEMENT_M:-3.0}")
if python3 verify/component/compare_fleet.py "$work/$first" "$work/$second" \
	"${FLEET_AGREEMENT_M:-3.0}" >/dev/null 2>&1; then
	pass "$verdict"
else
	fail "the two vehicles agree about where a target is"
	note "$verdict"
fi
rm -rf "$work"
