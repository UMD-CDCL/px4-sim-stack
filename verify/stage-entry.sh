#!/usr/bin/env bash
# Thin, sequestered entry point for one verification packet.
set -u

repo=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
name=${VERIFY_STAGE_NAME:?VERIFY_STAGE_NAME is required}
stages=${VERIFY_STAGE_STAGES:?VERIFY_STAGE_STAGES is required}
evidence_root=${VERIFY_EVIDENCE_DIR:-$repo/verify/evidence}
evidence_dir=$evidence_root/$name
mkdir -p "$evidence_dir"

usage() {
	cat <<EOF
Usage: verify/$name-test.sh [--dry-run] [--live]

Default: report the selected stages and write metadata without touching Docker.
--live: run the selected existing stages against the current stack and retain
        the raw report. This command does not start, stop, or reset services.
EOF
}

mode=dry-run
case "${1:-}" in
	--dry-run|'') ;;
	--live) mode=live ;;
	-h|--help|help) usage; exit 0 ;;
	*) printf 'unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
esac
[ "$#" -le 1 ] || {
	printf 'too many arguments\n' >&2
	usage >&2
	exit 2
}

metadata=$evidence_dir/metadata.txt
{
	printf 'stage=%s\nmode=%s\nstarted_utc=%s\n' "$name" "$mode" "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
	printf 'repo=%s\ncommit=%s\nbranch=%s\n' "$repo" \
		"$(git -C "$repo" rev-parse HEAD 2>/dev/null || printf unknown)" \
		"$(git -C "$repo" branch --show-current 2>/dev/null || printf unknown)"
	status=$(git -C "$repo" status --short 2>/dev/null || printf unknown)
	if [ -z "$status" ]; then status=clean; fi
	case "$status" in clean) printf 'git_state=clean\n' ;; *) printf 'git_state=dirty\n' ;; esac
	printf 'git_status=%s\n' "$status" | sed '2,$s/^/git_status_continued=/'
	printf 'selected_stages=%s\n' "$stages"
} >"$metadata"

printf 'stage=%s mode=%s\nselected: %s\nevidence: %s\n' "$name" "$mode" "$stages" "$evidence_dir"

if [ "$mode" = dry-run ]; then
	printf 'result=PENDING (dry-run produced no test evidence)\nstatus=PENDING\nfinished_utc=%s\n' \
		"$(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee "$evidence_dir/result.txt"
	exit 0
fi

report=$evidence_dir/report.txt
set +e
	read -r -a selected_stages <<< "$stages"
	(cd "$repo" && ./verify/run.sh "${selected_stages[@]}") 2>&1 | tee "$report"
status=${PIPESTATUS[0]}
set -e
printf 'result=%s\nexit_status=%s\nfinished_utc=%s\n' \
	"$( [ "$status" -eq 0 ] && printf PASS || printf FAIL )" "$status" \
	"$(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee "$evidence_dir/result.txt"
exit "$status"
