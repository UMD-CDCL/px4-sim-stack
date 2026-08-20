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
state=$(./px4sim uas "$lead" published --topic gimbal/state 2>/dev/null | tail -1)
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

# ------------------------------------------------------- giving it up
./px4sim uas "$lead" click off >/dev/null 2>&1 || true
