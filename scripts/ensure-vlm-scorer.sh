#!/usr/bin/env bash
# Start the host scorer supervisor if none runs, without waiting for model readiness.
# --restart (px4sim restart) first stops a running supervisor, so the scorer loads the
# code now on disk, then waits for the new scorer: the gate checks it once at startup.
# The caller treats setup failures as warnings so ground startup can continue.
set -euo pipefail

case "${VLM_GATE:-off}" in on|shadow) ;; *) exit 0 ;; esac
case "${1:-}" in --restart) restart=true ;; *) restart=false ;; esac

runner="${ROS2_WS_DIR:-../ros2_ws}/src/5g_drone/scripts/vlm_gate/run_server.sh"
gate_dir=${VLM_GATE_DIR:-$HOME/vlm_gate}
model=${VLM_GATE_MODEL:-$gate_dir/models/Qwen3-VL-2B-Instruct}
# seconds a restart waits for the old scorer to free the port, and for the new one to
# answer ready. after a crash the ground station's scorer was ready again in 9 s,
# 5 s of it the supervisor's pause, so a fresh one needs about 4 s.
stop_wait=10 ready_wait=60

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

port_held() {
  python3 - <<'PY' >/dev/null 2>&1
import socket
host, port = '127.0.0.1:8770'.split(':')
socket.create_connection((host, int(port)), timeout=1).close()
PY
}

if $restart; then
  if command -v tmux >/dev/null && tmux has-session -t '=vlm_gate' 2>/dev/null; then
    echo 'Stopping the VLM scorer in tmux session vlm_gate, so the new one loads the current code'
    tmux kill-session -t '=vlm_gate' || true
  fi
  deadline=$((SECONDS + stop_wait))
  while port_held && (( SECONDS < deadline )); do sleep 0.2; done
  if port_held; then
    echo "VLM scorer port 8770 is still held after $stop_wait s, by a scorer outside tmux session vlm_gate" >&2
    echo "or one that did not stop. It keeps the code it loaded. Stop it (ss -ltnp 'sport = :8770')," >&2
    echo 'then restart again.' >&2
  fi
fi

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
if $restart; then
  echo "Waiting up to $ready_wait s for the VLM scorer"
  deadline=$((SECONDS + ready_wait))
  while (( SECONDS < deadline )); do
    if healthy; then
      echo 'VLM scorer ready at http://127.0.0.1:8770'
      exit 0
    fi
    sleep 1
  done
  echo "VLM scorer not ready after $ready_wait s; continuing. Inspect" \
    "$gate_dir/logs/scorer_$(date +%F).log or tmux attach -t vlm_gate." >&2
  exit 0
fi
echo 'VLM scorer supervisor is running in tmux session vlm_gate; continuing without waiting for readiness.'
echo 'The supervisor retries exited scorer processes after 5 seconds. The gate reports degraded health until a successful health check or image query.'
