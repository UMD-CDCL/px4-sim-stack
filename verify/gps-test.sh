#!/usr/bin/env bash
set -euo pipefail
VERIFY_STAGE_NAME=gps VERIFY_STAGE_STAGES='localize ground' exec "$(dirname "$(readlink -f "$0")")/stage-entry.sh" "$@"
