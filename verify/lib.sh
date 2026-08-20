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
	local n=$1 height=${2:-30}
	if ./px4sim uas "$n" takeoff "$height" >/dev/null 2>&1; then
		return 0
	fi
	note "uas$n would not climb, so the stack is restarted for a clean slate"
	# The companions go with the simulator. A detector opens its camera once,
	# and the stream it was reading does not survive a new simulator, so a
	# companion left running afterwards holds a pipeline that will never see
	# another frame. Its entry point waits for the camera on the way back up.
	local number companions=""
	for number in $(fleet_numbers); do companions="$companions onboard$number"; done
	./px4sim restart sim >/dev/null 2>&1
	# shellcheck disable=SC2086
	./px4sim restart $companions >/dev/null 2>&1
	for number in $(fleet_numbers); do
		vehicle_ready "$number" || return 1
	done
	./px4sim uas "$n" takeoff "$height" >/dev/null 2>&1
}

report() {
	printf '\n%s%s passed, %s failed%s\n' "$BOLD" "$VERIFY_PASS" "$VERIFY_FAIL" "$OFF"
	[ "$VERIFY_FAIL" -eq 0 ]
}
