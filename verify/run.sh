#!/usr/bin/env bash
# Verification stages, in the order that finds a fault soonest. ./px4sim verify
# runs this. It reads the same fleet definition the front door does, so a
# vehicle number, a port and a domain mean here exactly what they mean there.
# Not -e, and not pipefail. A stage measures things, and a tool that answers
# "no" is an answer rather than a fault: under -e a probe that found nothing
# ended the whole run silently, part way through, with no report. A stage says
# what it found through pass and fail, and nothing else decides.
set -u

cd "$(dirname "$(readlink -f "$0")")/.."

# shellcheck disable=SC1091
. ./verify/lib.sh

# The same configuration the front door reads, and in the same order: the file
# says what the stack is, and the environment overrides it for one run. Without
# this a stage measures the built-in defaults and reports on a fleet that is
# not flying.
if [ -f .env ]; then
	for name in SCENE SCENARIO UAS_FLEET UAS_STREAMS UAS_ZOOM; do
		if [ -n "${!name+set}" ]; then eval "override_$name=\$$name"; fi
	done
	# shellcheck disable=SC1091
	set -a; . ./.env; set +a
	for name in SCENE SCENARIO UAS_FLEET UAS_STREAMS UAS_ZOOM; do
		holder="override_$name"
		if [ -n "${!holder+set}" ]; then export "$name=${!holder}"; fi
	done
fi
# shellcheck disable=SC1091
. ./scripts/fleet.sh
# shellcheck disable=SC1091
. ./scripts/zoom.sh

# What is FLYING wins over what is configured. A stage that reads .env while a
# different world is loaded measures the wrong scene: it flies to coordinates
# the running world has nothing at, which reads as a broken vehicle rather than
# as a stale setting. The simulator holds the scene it was started with, and
# Gazebo's own world name confirms it.
running_scene=$(docker exec "$(docker compose ps -q sim 2>/dev/null)" \
	sh -c 'echo "$SCENE"' 2>/dev/null | tr -d '\r')
if [ -n "$running_scene" ] && [ "$running_scene" != "${SCENE:-}" ]; then
	printf '%b\n' "${BOLD}The simulator is flying '${running_scene}', not '${SCENE:-unset}'.${OFF}"
	printf '    Checking what is flying. To change it: ./px4sim scene %s\n' "${SCENE:-}"
	SCENE=$running_scene
	SCENARIO=${running_scene}_casualties
	export SCENE SCENARIO
fi

# The number in each file name is the order, and nothing else reads it.
stage_files() { ls verify/stages/*.sh; }
# Which stages a real machine can answer. The others expand the simulator's
# airframes, read its compose services, or fly a vehicle to a viewpoint, and
# on the real fleet that number is an aircraft on the bench. Each stage below
# skips what needs a scene.
#
# A real machine is one of two worlds, and they answer different questions. A
# ground station holds no vehicle of its own: it reads the picture it rebuilt
# from the radio, so it answers for the ground station and its bridge. An
# aircraft IS the vehicle: it runs the companion and serves its own bridge, so
# it answers for the graph it flies. Giving both worlds the ground station's
# list told the operator of a healthy aircraft that the stack was broken.
real_stages() {
	case "$(world_of)" in
	aircraft) echo "units vehicle foxglove" ;;
	*)        echo "ground foxglove" ;;
	esac
}
REAL_STAGES=$(real_stages)
real_stage() { case " $REAL_STAGES " in *" $1 "*) return 0 ;; esac; return 1; }
stage_name()  { basename "$1" .sh | cut -d- -f2-; }
stage_file()  {
	local match
	match=$(stage_files | grep -E "/[0-9]+-$1\.sh$" || true)
	[ -n "$match" ] || return 1
	echo "$match"
}

usage() {
	cat <<EOF
${BOLD}px4sim verify${OFF} [stage ...]   default: every stage, in this order

$(for f in $(stage_files); do printf '    %-12s %s\n' "$(stage_name "$f")" "$(sed -n '2s/^# //p' "$f")"; done)

  A stage that needs the stack running says so and stops. Start it with
  ./px4sim restart.

  A real machine answers these stages: $REAL_STAGES. The rest fly the
  simulator, and this file refuses them by name.
EOF
}

case "${1:-}" in
-h | --help | help) usage; exit 0 ;;
esac

if [ $# -gt 0 ]; then
	files=""
	for name in "$@"; do
		found=$(stage_file "$name") ||
			{ printf 'No verify stage named %s.\n\n' "$name" >&2; usage >&2; exit 2; }
		if [ "$FLEET_IS_SIMULATED" = false ] && ! real_stage "$name"; then
			printf "'%s' verifies the simulator, and this stack is not one (UAS_BASE=%s,\n" \
				"$name" "$UAS_BASE" >&2
			printf "    profiles '%s'). This machine answers: %s\n" \
				"${COMPOSE_PROFILES:-none}" "$REAL_STAGES" >&2
			exit 2
		fi
		files="$files $found"
	done
else
	files=$(stage_files)
	if [ "$FLEET_IS_SIMULATED" = false ]; then
		kept=""
		for f in $files; do real_stage "$(stage_name "$f")" && kept="$kept $f"; done
		files=$kept
		printf '%bThis stack is not a simulator, so it runs: %s%b\n' \
			"$BOLD" "$REAL_STAGES" "$OFF"
	fi
fi

for f in $files; do
	stage "$(stage_name "$f")"
	# shellcheck disable=SC1090
	. "$f"
done

report
