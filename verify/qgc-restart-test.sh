#!/usr/bin/env bash
# Verify the QGroundControl front door across two consecutive restarts.
set -u

repo=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
evidence=${VERIFY_EVIDENCE_DIR:-$repo/verify/evidence/qgc-restart}
mkdir -p "$evidence"
metadata=$evidence/metadata.txt
report=$evidence/report.txt
result=$evidence/result.txt

printf 'stage=qgc-restart\nmode=live\nstarted_utc=%s\nrepo=%s\ncommit=%s\nbranch=%s\n' \
  "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$repo" \
  "$(git -C "$repo" rev-parse HEAD)" "$(git -C "$repo" branch --show-current)" >"$metadata"
git_status=$(git -C "$repo" status --short)
if [ -z "$git_status" ]; then git_status=clean; fi
printf 'git_state=%s\ngit_status=%s\n' \
  "$([ "$git_status" = clean ] && printf clean || printf dirty)" "$git_status" >>"$metadata"

status=0
{
  for cycle in 1 2; do
    printf '\n== restart %s ==\n' "$cycle"
    if ! (cd "$repo" && ./px4sim restart --no-build); then
      printf 'FAIL restart %s did not complete\n' "$cycle"
      status=1
      break
    fi
    qgc_id=$(cd "$repo" && docker compose ps -q qgc 2>/dev/null)
    count=0
    [ -n "$qgc_id" ] && count=$(docker exec "$qgc_id" sh -lc \
      "pgrep -c -f '^/opt/qgc/usr/bin/QGroundControl$'" 2>/dev/null || printf 0)
    printf 'QGroundControl process count: %s\n' "$count"
    if [ "$count" != 1 ]; then
      printf 'FAIL expected exactly one QGroundControl process\n'
      status=1
    else
      printf 'PASS exactly one QGroundControl process\n'
    fi
    errors=$(cd "$repo" && docker compose logs --since 3m qgc 2>&1 | \
      rg -i 'second instance|already running|single instance' || true)
    if [ -n "$errors" ]; then
      printf 'FAIL QGC singleton error appeared:\n%s\n' "$errors"
      status=1
    else
      printf 'PASS no QGC singleton error in recent logs\n'
    fi
  done
  exit "$status"
} 2>&1 | tee "$report"
status=${PIPESTATUS[0]}

printf 'result=%s\nexit_status=%s\nfinished_utc=%s\n' \
  "$([ "$status" -eq 0 ] && printf PASS || printf FAIL)" "$status" \
  "$(date -u +%Y-%m-%dT%H:%M:%SZ)" | tee "$result"
exit "$status"
