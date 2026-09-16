#!/usr/bin/env bash
# Resolve scenario metadata from the scene sources used by px4sim.
set -euo pipefail
SCENES_DIR=${SCENES_DIR:-modules/sim/scenes}
scenario_file() { local name=$1; [ -f "$name" ] && { printf '%s\n' "$name"; return; }; [ -f "$SCENES_DIR/scenarios/$name.yaml" ] && printf '%s\n' "$SCENES_DIR/scenarios/$name.yaml"; }
scenario_parent_scene() { local file; file=$(scenario_file "$1") || return 1; sed -n 's/^parent_scene:[[:space:]]*//p' "$file" | head -1 | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//'; }
scene_exists() { [ -f "$SCENES_DIR/worlds/$1.sdf" ] || [ -f "$SCENES_DIR/worlds/$1_surface.json" ]; }
validate_scenario_selection() { local scenario=$1 parent; [ -n "$(scenario_file "$scenario")" ] || return 2; parent=$(scenario_parent_scene "$scenario"); [ -n "$parent" ] || return 3; scene_exists "$parent" || return 4; printf '%s\n' "$parent"; }
