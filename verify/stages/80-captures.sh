# shellcheck shell=bash
# The captures an operator asks for: mosaic, fiducial and the VLM frame
#
# Each is a service on the detector and each produces something different: a
# mosaic is an image of the ground built from many frames, a fiducial capture
# is a survey that moves the whole fleet's frame, and a VLM frame is a full
# resolution picture for a model that does not run on the vehicle.

lead=$FIRST_UAS
container=$(COMPOSE_PROFILES="uas$lead,onboard$lead" docker compose ps -q "onboard$lead" 2>/dev/null)
if [ -z "$container" ]; then
	fail "uas$lead is not running. Start it: ./px4sim start"
	return 0
fi

# A capture needs ground in view, the same as a detection does.
./px4sim uas "$lead" takeoff 40 >/dev/null 2>&1 || true
./px4sim uas "$lead" gimbal -60 >/dev/null 2>&1 || true
./px4sim uas "$lead" detect on >/dev/null 2>&1 || true

for what in mosaic fiducial vlm; do
	answer=$(./px4sim uas "$lead" capture "$what" 2>&1) || true
	if printf '%s' "$answer" | grep -qiE 'fail|refus|error|no .* service'; then
		fail "the $what capture is accepted"
		note "$answer"
	else
		pass "the $what capture is accepted: $(printf '%s' "$answer" | tail -1)"
	fi
done

# A mosaic that is accepted and never drawn is the failure worth catching, so
# look for the file the node writes rather than for the service answer.
overlay=/home/user/d${lead}_rgb_mosaic_overlay.png
map=/home/user/d${lead}_rgb_mosaic_map.html
for file in "$overlay" "$map"; do
	size=$(./px4sim shell "onboard$lead" "stat -c %s '$file' 2>/dev/null || echo 0" 2>/dev/null | tr -d '\r')
	if [ "${size:-0}" -gt 1000 ]; then
		pass "the mosaic wrote $(basename "$file") (${size} bytes)"
	else
		fail "the mosaic wrote $(basename "$file")"
		note "${size:-0} bytes"
	fi
done

for topic in mosaic/overlay target_detections/vlm fiducial_update; do
	verdict=$(./px4sim probe "$lead" "/uas$lead/$topic" 2>/dev/null | cut -f3)
	expect_eq "$topic is published" data "$verdict"
done

# The VLM frame is full resolution and the reason the air link carries images at
# all, so it has to reach the ground.
if [ -n "$(COMPOSE_PROFILES=offboard docker compose ps -q offboard 2>/dev/null)" ]; then
	verdict=$(./px4sim probe ground "/uas$lead/target_detections/vlm" 2>/dev/null | cut -f3)
	expect_eq "the VLM frame reaches the ground" data "$verdict"
fi
