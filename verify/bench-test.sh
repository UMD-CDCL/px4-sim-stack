#!/usr/bin/env bash
set -euo pipefail
VERIFY_STAGE_NAME=bench VERIFY_STAGE_STAGES='ground foxglove' exec "$(dirname "$(readlink -f "$0")")/stage-entry.sh" "$@"
