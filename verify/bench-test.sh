#!/usr/bin/env bash
set -euo pipefail
base=${UAS_BASE:-10}
if [ -z "${UAS_BASE+x}" ] && [ -f .env ]; then
	base=$(sed -n 's/^UAS_BASE=//p' .env | tail -1)
fi
if [ "${1:-}" = --live ] && [ "${base:-10}" != 0 ]; then
	# Bench evidence starts on the ground even when the preceding sim packet
	# left the vehicle airborne or pointed at a target. `place` is the same
	# simulator front door used by operators and does not rebuild or alter the
	# selected scene/configuration.
	./px4sim place >/dev/null
fi
VERIFY_STAGE_NAME=bench VERIFY_STAGE_STAGES='ground foxglove' exec "$(dirname "$(readlink -f "$0")")/stage-entry.sh" "$@"
