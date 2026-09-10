#!/usr/bin/env bash
# Lint the prose with the ASD-STE100 heuristic linter.
# Score is violations per 100 words. Lower is cleaner.
# Target for general prose is under 2.5.
set -uo pipefail

cd "$(dirname "$0")/.." || exit 1

LINT=${STE_LINT:-$(dirname "$0")/../tools/asd-ste100/ste-lint.py}

if [ ! -f "$LINT" ]; then
	echo "Linter not found at $LINT. The docs are not linted."
	echo "Set STE_LINT to the path of ste-lint.py, or install the asd-ste100 skill."
	# An explicit STE_LINT that is missing is an error. The default is optional:
	# the aircraft has no linter, and ./px4sim check must pass there.
	[ -n "${STE_LINT:-}" ] && exit 127
	exit 0
fi

files=$(git ls-files '*.md' 2>/dev/null || find . -name '*.md' -not -path './src/*')
# shellcheck disable=SC2086
python3 "$LINT" --fail-over "${STE_MAX:-2.5}" $files
