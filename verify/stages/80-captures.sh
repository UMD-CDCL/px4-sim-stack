# shellcheck shell=bash
# The captures an operator asks for: mosaic, fiducial and the VLM frame
#
# Each produces something different. A mosaic is a picture of the ground built
# from several frames and written to disk. A fiducial capture is a survey that
# moves the whole fleet's frame. A VLM frame is a full size picture for a model
# that does not run on the vehicle, and it is the one image the radio carries.


# The long one. It flies several legs, so it is not in the default run: the
# flight stage covers what a change to this stack usually breaks, from one
# takeoff. Set VERIFY_FULL=1 to include this.
if [ "${VERIFY_FULL:-0}" != 1 ]; then
	note "skipped. VERIFY_FULL=1 includes it"
	return 0
fi

lead=$FIRST_UAS
if [ -z "$(COMPOSE_PROFILES="uas$lead,onboard$lead" docker compose ps -q "onboard$lead" 2>/dev/null)" ]; then
	fail "uas$lead is not running. Start it: ./px4sim start"
	return 0
fi
uas() { ./px4sim uas "$lead" "$@" 2>&1 || true; }

vehicle_ready "$lead" || return 0

# A mosaic frame is only added when its corner rays all reach the ground and it
# covers ground the mosaic does not already hold, so this flies a short line
# looking straight down rather than capturing from one place.
# Where the targets are in THIS scene, not in the one this check was written
# for. A written-in leg is either a long flight to empty ground or a view of
# nothing: campus puts its casualties about 100 m north of the origin and
# Lorton puts them within 10 m of it.
viewpoint=$(python3 verify/component/viewpoint.py \
	"modules/sim/scenes/scenarios/${SCENARIO:-${SCENE}_casualties}.yaml" \
	--height "${VERIFY_HEIGHT_M:-20}" --depression "${VERIFY_DEPRESSION_DEG:-45}" \
	2>/dev/null)
if [ -z "$viewpoint" ]; then
	fail "the scenario says where its targets are"
	return 0
fi

# A mosaic frame is only added when its corner rays all reach the ground and it
# covers ground the mosaic does not already hold, so this flies a short line
# rather than capturing from one place. The line is around the targets of
# whichever scene is loaded, and it is as short as a mosaic will accept.
read -r east north up _ <<< "$viewpoint"
flying "$lead" 40 || fail "uas$lead reaches the air"
uas detect on >/dev/null
for step in 0 25 50; do
	uas goto "$east" "$(python3 -c "print($north + $step)")" 40 --heading 0 >/dev/null
	uas gimbal -85 >/dev/null
	uas capture mosaic >/dev/null
done

# The node's own tally, not a count of lines in a window: a run that takes
# longer than the window ages the lines out and reads as a mosaic that took
# nothing while it was drawing one.
added=$(./px4sim logs --since 30m "onboard$lead" 2>/dev/null |
	sed -n 's/.*added_to_mosaic=\([0-9]*\).*/\1/p' | tail -1)
if [ "${added:-0}" -gt 0 ]; then
	pass "the mosaic took $added frames from the flight"
else
	fail "the mosaic took a frame from the flight"
	note "$(./px4sim logs --since 5m "onboard$lead" 2>/dev/null | grep 'SKIP.*summary' | tail -1)"
fi

for file in overlay.png map.html; do
	path=/home/user/d${lead}_rgb_mosaic_${file%%.*}.${file##*.}
	size=$(./px4sim shell "onboard$lead" "stat -c %s '$path' 2>/dev/null || echo 0" 2>/dev/null | tr -d '\r')
	if [ "${size:-0}" -gt 1000 ]; then
		pass "the mosaic wrote $(basename "$path") (${size} bytes)"
	else
		fail "the mosaic wrote $(basename "$path")"
		note "${size:-0} bytes"
	fi
done

# Back over the casualties, where there is something to mark and something to
# hand a vision model.
read -r view_east view_north view_up view_heading <<< "$viewpoint"
uas goto "$view_east" "$view_north" "$view_up" --heading "$view_heading" >/dev/null
uas gimbal "-${VERIFY_DEPRESSION_DEG:-45}" >/dev/null

for what in fiducial vlm; do
	answer=$(uas capture "$what")
	if printf '%s' "$answer" | grep -qiE 'fail|refus|error|no .* service'; then
		fail "the $what capture is accepted"; note "$answer"
	else
		pass "the $what capture is accepted"
	fi
done

# A real fiducial capture goes to a human to mark. `uas fiducial` stands in for
# that and reads back the survey, which is what moves the fleet's frame. A
# marker that is not configured surveys against latitude zero, so the size of
# the correction is the check that matters.
survey=$(uas fiducial | tail -1)
moved=$(printf '%s' "$survey" | awk -F'\t' '{ print ($2 < 0 ? -$2 : $2) + ($3 < 0 ? -$3 : $3) }')
if [ -z "$moved" ]; then
	fail "a marked capture produces a survey"
	note "$survey"
elif [ "$(python3 -c "print(${moved:-0} < 1000)")" = True ]; then
	pass "a marked capture produces a survey the site could hold: ${survey}"
else
	fail "a marked capture produces a survey the site could hold"
	note "moved ${moved} m, which is not a survey. Is fiducial_lla set?"
fi

# The VLM frame is the one image the radio carries, so watch the ground for it
# while the vehicle is asked for one.
if [ -n "$(COMPOSE_PROFILES=offboard docker compose ps -q offboard 2>/dev/null)" ]; then
	# Watch the ground before asking the vehicle, and watch long enough: a
	# saturated machine takes its time between the capture and the image
	# arriving on the other side of the bridge.
	arrival=$(mktemp)
	./px4sim probe ground --deadline 60 /casualty_image/compressed/vlm >"$arrival" 2>/dev/null &
	watcher=$!
	sleep 10
	uas capture vlm >/dev/null
	wait "$watcher" 2>/dev/null || true
	expect_eq "the VLM image reaches the ground" data "$(cut -f3 "$arrival" | tail -1)"
	rm -f "$arrival"
fi
