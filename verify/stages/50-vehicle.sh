# shellcheck shell=bash
# A flying vehicle: telemetry, the gimbal, the outline, and a localized target
#
# Needs the stack running. Flies the lead vehicle over a target the scenario
# recorded, points the gimbal at it and checks what the nodes make of it.

lead=$FIRST_UAS
container=$(COMPOSE_PROFILES="uas$lead,onboard$lead" docker compose ps -q "onboard$lead" 2>/dev/null)
if [ -z "$container" ]; then
	fail "uas$lead is not running. Start it: ./px4sim start"
	return 0
fi

# Not an assertion: a step that fails still lets the checks below speak.
uas() { ./px4sim uas "$lead" "$@" 2>&1 || true; }

telemetry=$(./px4sim probe "$lead" 2>/dev/null)
while IFS=$'\t' read -r topic publishers verdict; do
	[ -n "$topic" ] || continue
	case "$topic" in
	*/target_detections | */target_locations | */camera_fov | */ground_projection) continue ;;
	esac
	expect_eq "${topic##*/uas$lead/} carries data" data "$verdict"
done <<< "$telemetry"

# The outline and the drape need the camera to see ground, so put it there.
uas takeoff 40 >/dev/null
pointing=$(uas gimbal -60)
depression=$(printf '%s' "$pointing" | sed -n 's/^reported depression \([-0-9.]*\).*/\1/p')
if [ -z "$depression" ]; then
	fail "the gimbal reports where it points"
	note "$pointing"
else
	# A degree is well inside the slew the report settles to.
	within=$(python3 -c "print(abs($depression - 60) <= 2.0)")
	expect_eq "the gimbal points where it is told" True "$within"
fi

for topic in camera_fov ground_projection; do
	verdict=$(./px4sim probe "$lead" "/uas$lead/$topic" 2>/dev/null | cut -f3)
	expect_eq "$topic is published with ground in view" data "$verdict"
done

uas detect on >/dev/null
found=$(uas detections)
boxes=$(printf '%s' "$found" | sed -n 's/^\([0-9]*\) boxes.*/\1/p')
if [ "${boxes:-0}" -gt 0 ]; then
	pass "the detector finds $boxes boxes in the live stream"
else
	fail "the detector finds something in the live stream"
	note "$(printf '%s' "$found" | tail -2)"
fi

# Every localized box should land on the target the scenario recorded. The
# scenario places people, the detector finds people, so the nearest recorded
# target is the one it found.
errors=$(printf '%s' "$found" | sed -n 's/.* -> \([0-9.]*\) m from .*/\1/p')
if [ -z "$errors" ]; then
	fail "the boxes localize against the recorded targets"
	note "$(printf '%s' "$found" | tail -3)"
else
	worst=$(printf '%s' "$errors" | sort -g | tail -1)
	on_terrain=$(printf '%s' "$found" | grep -c 'on the terrain' || true)
	if [ "$(python3 -c "print($worst <= ${LOCALIZATION_TOLERANCE_M:-5.0})")" = True ]; then
		pass "every box localizes within $worst m of a recorded target"
	else
		fail "every box localizes within ${LOCALIZATION_TOLERANCE_M:-5.0} m of a recorded target"
		note "worst was $worst m"
	fi
	expect_eq "the boxes land on the terrain, not the flat plane" \
		"$(printf '%s\n' "$errors" | wc -l)" "$on_terrain"
fi
