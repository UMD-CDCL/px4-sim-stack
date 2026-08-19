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

# The bridges take a moment to forward what the vehicle latched, and a stage
# that asks first reads an empty graph.
waited=0
until [ "$(./px4sim probe ground "/uas$lead/camera/camera_info" 2>/dev/null | cut -f3)" = data ]; do
	if [ "$waited" -ge "${GROUND_READY_S:-180}" ]; then
		fail "the ground receives uas$lead after ${waited}s"
		return 0
	fi
	sleep 10
	waited=$((waited + 10))
done

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
	scored=$(./px4sim uas "$side" score --deadline 45 2>/dev/null | wc -l) || true
	name=$([ "$side" = ground ] && echo "the ground station" || echo "uas$lead")
	# Three numbers are published, and under load one can be slower than the
	# window this waits. Any of them means the scoring ran.
	if [ "${scored:-0}" -gt 0 ]; then
		pass "$name scores the detections against the known targets"
	else
		fail "$name scores the detections against the known targets"
	fi
done

# The operator points the gimbal by clicking the preview image on the ground.
# A pixel low in the image is nearer the vehicle, so the camera looks steeper.
# Where the camera ends up, not how far it moved. The simulated gimbal rides on
# the airframe, so a loitering multirotor's own pitch wanders the reported angle
# by degrees, which swamps the difference a single click makes. Where a low
# click leaves it against where a high click leaves it does not care about that.
aimed() { sed -n 's/.*depression [-0-9.]* -> \([-0-9.]*\).*/\1/p'; }
low=$(./px4sim uas ground click point 320 320 2>/dev/null | aimed) || true
high=$(./px4sim uas ground click point 320 40 2>/dev/null | aimed) || true
if [ -z "$low" ] || [ -z "$high" ]; then
	fail "a click on the ground points the gimbal on the vehicle"
else
	expect_eq "a click low in the image aims nearer than one high in it" True \
		"$(python3 -c "print($low > $high + 2)")"
fi

# Off means off. A click that changes nothing leaves the camera where the last
# one put it, so this compares against where the low click aimed rather than
# against how far it moved.
./px4sim uas ground click off >/dev/null 2>&1
./px4sim uas ground click point 320 320 >/dev/null 2>&1 || true
aimed_before=$(./px4sim uas ground click point 320 320 --keep-mode 2>/dev/null | aimed) || true
./px4sim uas ground click off >/dev/null 2>&1
ignored=$(./px4sim uas ground click point 320 40 --keep-mode 2>/dev/null | aimed) || true
if [ -z "$ignored" ] || [ -z "$aimed_before" ]; then
	fail "a station whose clicks are off ignores them"
else
	# A click high in the image would open the view by degrees if it landed.
	expect_eq "a station whose clicks are off ignores them" True \
		"$(python3 -c "print(abs($ignored - $aimed_before) < 3)")"
fi
