#!/usr/bin/env bash
set -euo pipefail
VERIFY_STAGE_NAME=code VERIFY_STAGE_STAGES='airframes contract units' exec "$(dirname "$(readlink -f "$0")")/stage-entry.sh" "$@"
