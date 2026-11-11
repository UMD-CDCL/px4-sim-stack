#!/usr/bin/env bash
# Start the host scorer supervisor without waiting for model readiness.
# The caller treats setup failures as warnings so ground startup can continue.
set -euo pipefail

case "${VLM_GATE:-off}" in on|shadow) ;; *) exit 0 ;; esac

runner="${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone/scripts/vlm_gate/run_server.sh"
gate_dir=${VLM_GATE_DIR:-$HOME/vlm_gate}
model=${VLM_GATE_MODEL:-$gate_dir/models/Qwen3-VL-2B-Instruct}

# A new process cannot repair a driver that requires a host reboot.
# Older drivers can lack this query, so retain the HTTP check in that case.
if command -v nvidia-smi >/dev/null; then
  recovery=$(nvidia-smi --query-gpu=gpu_recovery_action --format=csv,noheader 2>/dev/null) || recovery=
  case "$recovery" in
    *Reboot*)
      echo 'NVIDIA GPU recovery requires a laptop reboot. Restart the laptop before starting the VLM scorer.' >&2
      exit 20
      ;;
  esac
fi

healthy() {
  python3 - <<'PY' >/dev/null 2>&1
import json
import urllib.request
http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
with http.open('http://127.0.0.1:8770/health', timeout=1) as response:
    health = json.load(response)
assert health.get('model') and health.get('device') and health.get('prompts') == 4
assert health.get('ready') is True
PY
}

healthy && exit 0
command -v tmux >/dev/null || { echo 'VLM scorer needs tmux on the host.' >&2; exit 21; }
[[ -x "$runner" && -x "$gate_dir/venv/bin/python" && -d "$model" ]] || {
  echo "VLM scorer installation missing. See $runner's sibling README.md." >&2
  exit 22
}
if ! tmux has-session -t '=vlm_gate' 2>/dev/null; then
  echo 'Starting the host VLM scorer in tmux session vlm_gate'
  # Pass arguments separately so paths and model overrides survive spaces.
  if ! tmux new-session -d -s vlm_gate env "VLM_GATE_DIR=$gate_dir" \
      "VLM_GATE_MODEL=$model" "$(realpath "$runner")"; then
    echo 'VLM scorer supervisor failed to start; check host tmux/session permissions.' >&2
    exit 23
  fi
fi
echo 'VLM scorer supervisor is running in tmux session vlm_gate; continuing without waiting for readiness.'
echo 'The supervisor retries exited scorer processes after 5 seconds. The gate reports degraded health until a successful health check or image query.'
