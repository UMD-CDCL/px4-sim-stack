# shellcheck shell=bash
# One flight: the gimbal, the lens, and where a casualty lands
#
# Everything that needs the vehicle in the air, from ONE takeoff and ONE leg.
# The old suite flew a leg for each question, in coordinates written for the
# campus scene, which at Lorton is a long trip to empty ground. This flies to
# where the loaded scene keeps its targets and asks every question from there.
#
# The only extra movement is one short reposition, because a region of interest
# that is held cannot be told from one that is merely pointed at unless the
# aircraft moves under it.

lead=$FIRST_UAS
uas() { ./px4sim uas "$lead" "$@" 2>&1 || true; }
lens() { ./px4sim zoom "$lead" "$1" >/dev/null 2>&1 || true; }

vehicle_ready "$lead" || return 0

viewpoint=$(python3 verify/component/viewpoint.py \
	"modules/sim/scenes/scenarios/${SCENARIO:-${SCENE}_casualties}.yaml" \
	--height "${VERIFY_HEIGHT_M:-20}" --depression "${VERIFY_DEPRESSION_DEG:-45}" 2>/dev/null)
if [ -z "$viewpoint" ]; then
	fail "the scenario says where its targets are"
	return 0
fi
depression=${VERIFY_DEPRESSION_DEG:-45}

flying "$lead" "${VERIFY_HEIGHT_M:-20}" || { fail "uas$lead reaches the air"; return 0; }
read -r view_east view_north view_up view_heading <<< "$viewpoint"
uas goto "$view_east" "$view_north" "$view_up" --heading "$view_heading" >/dev/null

# ---------------------------------------------------------------- the gimbal
pointing=$(uas gimbal "-$depression")
reported=$(printf '%s' "$pointing" | sed -n 's/^reported depression \([-0-9.]*\).*/\1/p')
if [ -z "$reported" ]; then
	fail "the gimbal reports where it points"
	note "$pointing"
else
	expect_eq "the gimbal points where it is told" True \
		"$(python3 -c "print(abs($reported - $depression) <= 2.0)")"
fi

# A click on the boresight asks for nothing and must move nothing. It is the
# cheapest check there is and it caught a defect that walked the gimbal a
# degree per click.
before=$(printf '%s' "$(uas click point 320 180)" | sed -n 's/.*depression \([-0-9.]*\) -> \([-0-9.]*\).*/\1 \2/p')
if [ -z "$before" ]; then
	fail "a click on the boresight moves nothing"
else
	read -r was now <<< "$before"
	expect_eq "a click on the boresight moves nothing" True \
		"$(python3 -c "print(abs($now - $was) <= 0.3)")"
fi

# Off the boresight the slew is the angle the intrinsics say it is.
uas gimbal "-$depression" >/dev/null
moved=$(printf '%s' "$(uas click point 320 90)" | sed -n 's/.*depression \([-0-9.]*\) -> \([-0-9.]*\).*/\1 \2/p')
read -r was now <<< "${moved:-0 0}"
want=$(./px4sim uas "$lead" published --topic camera/camera_info >/dev/null 2>&1; \
	python3 -c "
import math
# 90 px above the centre of a 360 row image, through the focal length in hand.
print(round(math.degrees(math.atan(90 / ${VERIFY_FX:-1802.85})), 2))")
if [ -z "$moved" ]; then
	fail "an off-boresight click slews by the angle the lens says"
else
	expect_eq "an off-boresight click slews by the angle the lens says" True \
		"$(python3 -c "print(abs(abs($now - $was) - $want) <= 0.6)")"
fi

# ------------------------------------------------------------------ the lens
uas gimbal "-$depression" >/dev/null
for preset in wide mid narrow; do
	lens "$preset"
	sleep 6
	seen=$(./px4sim probe "$lead" "/uas$lead/camera/camera_info" 2>/dev/null | cut -f3)
	expect_eq "the $preset framing publishes a calibration" data "${seen:-absent}"
done

# Every framing has to put the same casualty in the same place. A stale
# calibration passes every centred check and fails this one.
lens mid
sleep 6
uas detect on >/dev/null
sleep "${VERIFY_SETTLE_S:-12}"
placed=$(uas detections | grep -c "on the terrain" || true)
if [ "${placed:-0}" -gt 0 ]; then
	pass "casualties localize onto the terrain: $placed of them"
else
	fail "casualties localize onto the terrain"
	note "$(uas detections | tail -3 | tr '\n' ' ')"
fi

# ----------------------------------------------------------- a held place
held=$(uas click roi 320 180)
# The state is latched and it speaks only when the mode changes, so it is read
# with the latched reader. `published` speaks TargetBoxArray and can never
# answer a String topic, which is what made this look like a broken gimbal.
state=$(./px4sim uas "$lead" topic gimbal/state 2>/dev/null | tail -1)
if printf '%s' "$state" | grep -q "mode=roi"; then
	pass "a click in region of interest mode holds a place"
else
	fail "a click in region of interest mode holds a place"
	note "${state:-$held}"
fi

# The one extra move. A held place is only proved by the aircraft leaving it.
first=$(uas status | sed -n 's/^gimbal *\([-0-9.]*\).*/\1/p')
uas goto "$view_east" "$(python3 -c "print($view_north - 20)")" "$view_up" \
	--heading "$view_heading" >/dev/null
second=$(uas status | sed -n 's/^gimbal *\([-0-9.]*\).*/\1/p')
if [ -n "$first" ] && [ -n "$second" ]; then
	expect_eq "the gimbal repoints to hold that place as the aircraft moves" True \
		"$(python3 -c "print(abs($second - $first) >= 3.0)")"
else
	fail "the gimbal repoints to hold that place as the aircraft moves"
fi

# ------------------------------------------- what the operator is shown
# The overlay is what Foxglove projects into the image panel, so its rate is a
# product feature rather than waste. It fell to the scoring rate once and the
# marks visibly trailed the picture.
uas gimbal "-$depression" >/dev/null
counts=$(./px4sim probe "$lead" --count 8 \
	"/viz/uas$lead/scoring/targets" \
	"/uas$lead/scoring/verdicts" \
	"/viz/uas$lead/scoring/annotations" 2>/dev/null)
marks=$(printf '%s' "$counts" | awk -F'\t' '/viz.*scoring\/targets/ { print $3 }')
judged=$(printf '%s' "$counts" | awk -F'\t' '/^\/uas[0-9]*\/scoring\/verdicts/{ print $3 }')
boxes=$(printf '%s' "$counts" | awk -F'\t' '/scoring\/annotations/  { print $3 }')
expect_eq "the overlay is drawn about ten times a second" True \
	"$(python3 -c "print(${marks:-0} >= 60)")"

# One message decides both views, so a box goes on the picture when the verdict
# that judges it does. They were built from different messages once, and a box
# wore the verdict of whatever shared its place in the frame before.
#
# This counts the SCORER's verdicts, not the marker arrays: the marks are
# restamped on their own timer at ten a second so that the overlay tracks the
# camera, and comparing the boxes against them measures the timer instead.
if [ "${judged:-0}" -gt 0 ] && [ "${boxes:-0}" -gt 0 ]; then
	expect_eq "the boxes are published with the verdicts that judge them" True \
		"$(python3 -c "print(abs($boxes - $judged) <= 0.2 * $judged)")"
else
	fail "the boxes are published with the verdicts that judge them"
fi
note "verdicts ${judged:-0}, boxes ${boxes:-0}, marks ${marks:-0} in 8s"

# The one that used to fail. Nothing localizes above the horizon gate, and the
# localizer publishes nothing at all on a frame that held nothing, so the marks
# and the boxes both used to hang until a timeout swept them.
uas gimbal -5 >/dev/null
empty=$(./px4sim probe "$lead" --count 4 \
	"/viz/uas$lead/scoring/verdicts" 2>/dev/null | awk -F'\t' '{ print $3 }')
still=$(./px4sim uas "$lead" published --topic scoring/verdicts 2>/dev/null | tail -1)
uas gimbal "-$depression" >/dev/null
back=$(./px4sim probe "$lead" --count 6 \
	"/viz/uas$lead/scoring/annotations" 2>/dev/null | awk -F'\t' '{ print $3 }')
expect_eq "the marks come back when the camera does" True \
	"$(python3 -c "print(${back:-0} > 0)")"

# ------------------------------------------------------------- the mosaic
# A capture asked for from the ground has to reach the vehicle, and the map it
# builds has to land on the ground the vehicle flew over.
#
# A frame is not always added. A vehicle holding station sees the ground it has
# already mapped, and the node refuses a frame that is nearly all old ground.
# That is the node working, so this asks whether the capture ARRIVED, and lets
# the node decide what to do with it.
seen_before=$(mosaic_summary 30m "$lead" | sed -n 's/received=\([0-9]*\).*/\1/p')
drew_before=$(terrain_map_draws 30m)
added_before=$(mosaic_added 30m "$lead")
./px4sim capture "$lead" mosaic >/dev/null 2>&1
seen_after=$(mosaic_summary 30m "$lead" | sed -n 's/received=\([0-9]*\).*/\1/p')
if [ "${seen_after:-0}" -gt "${seen_before:-0}" ]; then
	pass "a capture asked for from the ground reaches the vehicle"
	note "$(mosaic_summary 30m "$lead")"
else
	fail "a capture asked for from the ground reaches the vehicle"
	note "the mosaic node saw ${seen_after:-no} captures, was ${seen_before:-none}"
fi

# Only a frame the mosaic accepted produces a new map to draw. Asking the
# terrain to redraw for a frame that was refused as old ground would fail on
# the node doing the right thing.
if [ "$(mosaic_added 30m "$lead")" = "${added_before:-0}" ]; then
	skip "the map is drawn into the terrain: the frame was old ground"
elif await 40 "the map is drawn into the terrain, one surface" \
		terrain_drew_since "${drew_before:-0}" 30m; then
	note "$(terrain_drew_map 30m)"
else
	note "the mosaic took the frame and the terrain never drew it"
fi

# ------------------------------------------------------------- the survey
# A survey measures the error between the frame the vehicle holds and the ground
# it flies over. This simulator holds that error by standing the marker away
# from the coordinate the vehicles were given, so the correction must come back
# as minus the offset.
#
# Displace it AFTER the vehicle is airborne. `fly` reloads the world and the
# marker goes back to its surveyed coordinate, so a displacement set earlier is
# silently undone and the survey measures about zero against an expected ten
# metres -- which reads as a broken loop rather than as a reset marker.
survey_east=6
survey_north=-9
stood=$(./px4sim fiducial "$survey_east" "$survey_north" 2>&1)
if printf '%s' "$stood" | grep -q "(+6.00, -9.00)"; then
	# Stand off far enough that the camera, at this depression, looks at it.
	uas goto "$survey_east" \
		"$(python3 -c "print($survey_north - ${VERIFY_HEIGHT_M:-20})")" \
		"${VERIFY_HEIGHT_M:-20}" --heading 0 >/dev/null
	uas gimbal "-$depression" >/dev/null
	surveyed=$(uas fiducial --placed "$survey_east" "$survey_north")
	# The line reads "<from> -> <to>\teast\tnorth\tup", so the numbers are the
	# last three fields. Counting from the left picks up the arrow.
	read -r east north <<< "$(printf '%s' "$surveyed" | grep fiducial_offset \
		| awk '{ print $(NF - 2), $(NF - 1) }')"
	if [ -n "${north:-}" ]; then
		expect_eq "a survey of the marker recovers where it stands" True \
			"$(python3 -c "
import math
print(math.hypot($east - -$survey_east, $north - -($survey_north)) <= 1.5)")"
		note "$(printf '%s' "$surveyed" | tail -2 | tr '\n' ' ')"
	else
		fail "a survey of the marker recovers where it stands"
		note "$(printf '%s' "$surveyed" | tail -1)"
	fi
else
	fail "the marker stands where it is put"
	note "$(printf '%s' "$stood" | tail -1)"
fi
# Put it back, so a scoring run after this one is not measured against a moved
# marker. A world reload would do it too, and this does not wait for one.
./px4sim fiducial 0 0 >/dev/null 2>&1

# ------------------------------------------------------- giving it up
./px4sim uas "$lead" click off >/dev/null 2>&1 || true
