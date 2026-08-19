# Result keeping for the verification stages.

VERIFY_PASS=0
VERIFY_FAIL=0
VERIFY_CURRENT=

BOLD=$'\033[1m'; DIM=$'\033[2m'; GRN=$'\033[32m'; RED=$'\033[31m'; YEL=$'\033[33m'; OFF=$'\033[0m'

stage() { VERIFY_CURRENT=$1; printf '\n%s== %s ==%s\n' "$BOLD" "$1" "$OFF"; }
pass()  { VERIFY_PASS=$((VERIFY_PASS + 1)); printf '  %sok  %s%s\n' "$GRN" "$OFF" "$*"; }
fail()  { VERIFY_FAIL=$((VERIFY_FAIL + 1)); printf '  %sFAIL%s %s\n' "$RED" "$OFF" "$*"; }
note()  { printf '       %s%s%s\n' "$DIM" "$*" "$OFF"; }

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

report() {
	printf '\n%s%s passed, %s failed%s\n' "$BOLD" "$VERIFY_PASS" "$VERIFY_FAIL" "$OFF"
	[ "$VERIFY_FAIL" -eq 0 ]
}
