# shellcheck shell=bash
# A vehicle that is up: every topic the contract names carries data
#
# Needs the stack running. Nothing here flies: the flight checks are one
# takeoff in the flight stage, so a fault in the graph is found before the
# vehicle leaves the ground.

lead=$FIRST_UAS
container=$(COMPOSE_PROFILES="uas$lead,onboard$lead" docker compose ps -q "onboard$lead" 2>/dev/null)
if [ -z "$container" ]; then
	fail "uas$lead is not running. Start it: ./px4sim start"
	return 0
fi

# Not an assertion: a step that fails still lets the checks below speak.
uas() { ./px4sim uas "$lead" "$@" 2>&1 || true; }

vehicle_ready "$lead" || return 0

telemetry=$(./px4sim probe "$lead" 2>/dev/null)
while IFS=$'\t' read -r topic publishers verdict; do
	[ -n "$topic" ] || continue
	case "$topic" in
	*/target_detections | */target_locations | */camera_fov | */ground_projection) continue ;;
	esac
	expect_eq "${topic##*/uas$lead/} carries data" data "$verdict"
done <<< "$telemetry"
