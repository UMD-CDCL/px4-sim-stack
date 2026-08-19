# shellcheck shell=bash
# The ground station shows what the vehicle worked out, not its own version
#
# The radio link carries the localizations and little else. The ground rebuilds
# the picture from those plus MAVLink and the compressed streams, so the numbers
# on the two sides have to be the same numbers.

lead=$FIRST_UAS
if [ -z "$(COMPOSE_PROFILES=offboard docker compose ps -q offboard 2>/dev/null)" ]; then
	fail "the ground station is not running. Start it: ./px4sim start offboard"
	return 0
fi

for topic in camera/camera_info position status; do
	verdict=$(./px4sim probe ground "/uas$lead/$topic" 2>/dev/null | cut -f3)
	expect_eq "the ground receives $topic" data "$verdict"
done
expect_eq "the ground holds the scene's casualty locations" data \
	"$(./px4sim probe ground /known_casualty_locations 2>/dev/null | cut -f3)"

# Both at once: the two are watching one live stream, so sampling each in turn
# compares different frames.
work=$(mktemp -d)
./px4sim uas "$lead" published --seconds "${AIR_LINK_SECONDS:-15}" >"$work/vehicle" 2>/dev/null &
./px4sim uas ground published --seconds "${AIR_LINK_SECONDS:-15}" >"$work/ground" 2>/dev/null &
wait
if [ ! -s "$work/vehicle" ]; then
	fail "the vehicle publishes localizations"
	note "point the gimbal at the ground and turn detection on:"
	note "  ./px4sim uas $lead gimbal -60 && ./px4sim uas $lead detect on"
else
	verdict=$(python3 verify/component/compare_localizations.py "$work/vehicle" "$work/ground")
	if python3 verify/component/compare_localizations.py "$work/vehicle" "$work/ground" >/dev/null; then
		pass "every localization reaches the ground unchanged: $verdict"
	else
		fail "every localization reaches the ground unchanged"
		note "$verdict"
	fi
fi
rm -rf "$work"

# The verdicts are recomputed on the ground rather than sent, so both sides
# have to reach them. The numbers are running averages over whatever each side
# has seen, so they are close rather than equal: the localizations behind them
# are what must match exactly, and they are checked above.
for side in "$lead" ground; do
	scored=$(./px4sim uas "$side" score 2>/dev/null | wc -l)
	name=$([ "$side" = ground ] && echo "the ground station" || echo "uas$lead")
	expect_eq "$name scores the detections against the known targets" 3 "$scored"
done
