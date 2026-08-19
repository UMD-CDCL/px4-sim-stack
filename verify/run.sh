#!/usr/bin/env bash
# Verification stages, in the order that finds a fault soonest. ./px4sim verify
# runs this. It reads the same fleet definition the front door does, so a
# vehicle number, a port and a domain mean here exactly what they mean there.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")/.."

# shellcheck disable=SC1091
. ./verify/lib.sh
# shellcheck disable=SC1091
. ./scripts/fleet.sh

# The number in each file name is the order, and nothing else reads it.
stage_files() { ls verify/stages/*.sh; }
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
  ./px4sim start.
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
		files="$files $found"
	done
else
	files=$(stage_files)
fi

for f in $files; do
	stage "$(stage_name "$f")"
	# shellcheck disable=SC1090
	. "$f"
done

report
