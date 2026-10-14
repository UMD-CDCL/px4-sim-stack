#!/usr/bin/env bash
# Keep the host GPU scorer available before starting an enabled ground gate.
set -euo pipefail

case "${VLM_GATE:-off}" in on|shadow) ;; *) exit 0 ;; esac

runner="${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone/scripts/vlm_gate/run_server.sh"
gate_dir=${VLM_GATE_DIR:-$HOME/vlm_gate}
model=${VLM_GATE_MODEL:-$gate_dir/models/Qwen3-VL-2B-Instruct}

healthy() {
  python3 - <<'PY' >/dev/null 2>&1
import json
import urllib.request
http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
with http.open('http://127.0.0.1:8770/health', timeout=1) as response:
    health = json.load(response)
assert health.get('model') and health.get('device') and health.get('prompts') == 4
PY
}

healthy && exit 0
command -v tmux >/dev/null || { echo 'VLM scorer needs tmux on the host.' >&2; exit 1; }
[[ -x "$runner" && -x "$gate_dir/venv/bin/python" && -d "$model" ]] || {
  echo "VLM scorer installation missing. See $runner's sibling README.md." >&2
  exit 1
}
if ! tmux has-session -t '=vlm_gate' 2>/dev/null; then
  echo 'Starting the host VLM scorer in tmux session vlm_gate'
  # Pass arguments separately so paths and model overrides survive spaces.
  tmux new-session -d -s vlm_gate env "VLM_GATE_DIR=$gate_dir" \
    "VLM_GATE_MODEL=$model" "$(realpath "$runner")"
fi
echo 'Waiting for the VLM scorer (up to 60 seconds)'
deadline=$((SECONDS + 60))
while (( SECONDS < deadline )); do
  if healthy; then
    echo 'VLM scorer ready at http://127.0.0.1:8770'
    exit 0
  fi
  sleep 1
done
echo "VLM scorer not ready; inspect $gate_dir/logs/scorer_$(date +%F).log or tmux attach -t vlm_gate." >&2
exit 1
