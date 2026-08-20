# Result keeping for the verification stages.

VERIFY_PASS=0
VERIFY_FAIL=0
VERIFY_CURRENT=

BOLD=$'\033[1m'; DIM=$'\033[2m'; GRN=$'\033[32m'; RED=$'\033[31m'; YEL=$'\033[33m'; OFF=$'\033[0m'

stage() { VERIFY_CURRENT=$1; printf '\n%s== %s ==%s\n' "$BOLD" "$1" "$OFF"; }
pass()  { VERIFY_PASS=$((VERIFY_PASS + 1)); printf '  %sok  %s%s\n' "$GRN" "$OFF" "$*"; }
fail()  { VERIFY_FAIL=$((VERIFY_FAIL + 1)); printf '  %sFAIL%s %s\n' "$RED" "$OFF" "$*"; }
note()  { printf '       %s%s%s\n' "$DIM" "$*" "$OFF"; }
# A stage that does not apply to this stack is not a failure. A fleet of one
# has nothing to say about two vehicles agreeing, and saying so as a failure
# would make the default run always report one.
skip()  { printf '  %s--  %s%s\n' "$DIM" "$OFF" "$*"; }

expect() {
	local what=$1; shift
	if "$@" >/dev/null 2>&1; then pass "$what"; else fail "$what"; fi
}

expect_eq() {
	local what=$1 want=$2 got=$3
	if [ "$want" = "$got" ]; then pass "$what"; else fail "$what"; note "want $want, got $got"; fi
}

# Poll rather than sleep. A stage that waits out a fixed timeout hides which
# step was slow and stalls the whole run behind code that already died.
POLL_PERIOD_S=0.25
await() {
	local deadline=$1 what=$2; shift 2
	local waited=0
	while ! "$@" >/dev/null 2>&1; do
		if [ "$(echo "$waited >= $deadline" | bc)" = 1 ]; then
			fail "$what"; note "gave up after ${deadline}s"; return 1
		fi
		sleep "$POLL_PERIOD_S"
		waited=$(echo "$waited + $POLL_PERIOD_S" | bc)
	done
	pass "$what"
	[ "$waited" != 0 ] && note "after ${waited}s"
	return 0
}

# A stack takes minutes to be ready: Gazebo loads the world, the scenario places
# its entities, PX4 boots, the streams come up and the detector builds or loads
# its engines. A stage that starts before that fails on everything and says
# nothing useful, so every runtime stage waits here first.
vehicle_ready() {
	local n=$1 deadline=${2:-600} waited=0
	# altitude, not state: MAVROS publishes state on a timer whether or not it
	# is hearing an autopilot, so a stack that has just restarted its simulator
	# looks ready while nothing is flying.
	while [ "$(./px4sim probe "$n" "/uas$n/altitude" 2>/dev/null | cut -f3)" != data ]; do
		if [ "$waited" -ge "$deadline" ]; then
			fail "uas$n answers after ${deadline}s"
			note "is it started? ./px4sim status, ./px4sim logs onboard$n"
			return 1
		fi
		sleep 10
		waited=$((waited + 10))
	done
	[ "$waited" -gt 0 ] && note "uas$n ready after ${waited}s"
	return 0
}

# Get a vehicle into the air, and get the stack out of the way if it will not.
#
# PX4's land detector can settle into believing a grounded vehicle is still
# flying. It then refuses to disarm ("Disarming denied: not landed") and
# ignores a takeoff, and no command reaches that: only a new PX4 does. A stage
# that measures a vehicle which never left the ground reports nothing useful,
# so it restarts the simulator once and tries again.
flying() {
	# One way into the air, shared with the front door. `./px4sim fly` handles
	# a vehicle on its side, an autopilot that rebooted and stopped honouring
	# the gimbal, and a takeoff that will not climb, and it says which of those
	# it found. A second copy of that logic here would drift from it.
	# Keep what it said. `fly` names the condition it gave up on -- the world,
	# the telemetry or the camera -- and throwing that away leaves a stage
	# reporting only that a vehicle did not reach the air, which is the one
	# thing the reader already knows.
	local n=$1 height=${2:-30} said
	said=$(./px4sim fly "$n" "$height" 2>&1)
	local flew=$?
	printf '%s' "$said" | grep -q "is flying at" || {
		printf '%s\n' "$said" | grep -E "never|not ready|will not climb" \
			| sed 's/^/       /' >&2
		return 1
	}
	return $flew
}

# What a vehicle's mosaic node did with the captures it was asked for. Its own
# words say more than the canvas does: they name the frame that was refused and
# why, as well as the one that was added.
mosaic_summary() {
	./px4sim logs --since "${1:-20m}" "onboard$2" 2>/dev/null \
		| grep -oE "received=[0-9]+, added_to_mosaic=[0-9]+, dropped=[0-9]+" | tail -1
}

# terrain_viz reports the share of the terrain image the vehicle's map covers,
# which is the one line that says the map reached the ground rather than merely
# being published.
terrain_drew_map() {
	./px4sim logs --since "${1:-20m}" offboard 2>/dev/null \
		| grep -oE "the vehicle.s map over [^,]*" | tail -1
}

# How many frames the mosaic has ACCEPTED. A vehicle holding station sees ground
# it has already mapped, and the node refuses a frame that is nearly all old
# ground. Nothing downstream redraws for a refused frame, and that is the node
# working, so a check on the drawing has to know which happened.
mosaic_added() {
	mosaic_summary "${1:-20m}" "$2" | sed -n 's/.*added_to_mosaic=\([0-9]*\).*/\1/p'
}

# Whether it has drawn one since the count taken before the capture. A capture
# command returns as soon as the Bool is published, and the map has to be built,
# published and composited after that, so a reader that looks straight away sees
# the state before the work.
terrain_drew_since() {
	[ "$(terrain_map_draws "${2:-20m}")" -gt "${1:-0}" ]
}

# How many times it has said it. A check that only asks whether the words
# appear anywhere in the window passes on a map drawn before the run started,
# which is the same answer a stack that drew nothing today would give.
terrain_map_draws() {
	./px4sim logs --since "${1:-20m}" offboard 2>/dev/null \
		| grep -c "vehicle.s map over" | tr -d '\r'
}

report() {
	printf '\n%s%s passed, %s failed%s\n' "$BOLD" "$VERIFY_PASS" "$VERIFY_FAIL" "$OFF"
	[ "$VERIFY_FAIL" -eq 0 ]
}
