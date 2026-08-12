#!/usr/bin/env bash
# Lint the prose with the ASD-STE100 heuristic linter.
# Score is violations per 100 words. Lower is cleaner.
# Target for general prose is under 2.5.
set -uo pipefail

cd "$(dirname "$0")/.."

LINT=${STE_LINT:-$HOME/.claude/skills/ste-writing/ste-lint.py}

if [ ! -f "$LINT" ]; then
	echo "Linter not found at $LINT."
	echo "Set STE_LINT to the path of ste-lint.py, or install the ste-writing skill."
	exit 127
fi

files=$(git ls-files '*.md' 2>/dev/null || find . -name '*.md' -not -path './src/*')
# shellcheck disable=SC2086
python3 "$LINT" --fail-over "${STE_MAX:-2.5}" $files
