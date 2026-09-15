#!/usr/bin/env bash
set -euo pipefail
VERIFY_STAGE_NAME=sim VERIFY_STAGE_STAGES='vehicle flight captures foxglove' exec "$(dirname "$(readlink -f "$0")")/stage-entry.sh" "$@"
