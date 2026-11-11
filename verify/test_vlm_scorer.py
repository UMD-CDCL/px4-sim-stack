import json
import os
from pathlib import Path
import shlex
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'ensure-vlm-scorer.sh'


@pytest.fixture
def scorer():
    body = {'model': 'test', 'device': 'cuda:0', 'prompts': 4, 'ready': True}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(body).encode())

        def log_message(self, *args):
            pass

    with HTTPServer(('127.0.0.1', 0), Handler) as http:
        thread = threading.Thread(target=http.serve_forever, daemon=True)
        thread.start()
        yield body, http.server_port
        http.shutdown()
        thread.join()


def run(tmp_path, scorer, recovery='None', mode='on', query_status=0,
        installed=False, existing_supervisor=False, tmux_failure=False):
    binary = tmp_path / 'nvidia-smi'
    binary.write_text(f"#!/bin/sh\nprintf '%s\\n' '{recovery}'\nexit {query_status}\n")
    binary.chmod(0o755)
    if installed:
        runner = tmp_path / 'src/5g_drone/scripts/vlm_gate/run_server.sh'
        runner.parent.mkdir(parents=True)
        runner.write_text('#!/bin/sh\nexit 0\n')
        runner.chmod(0o755)
        python = tmp_path / 'absent/venv/bin/python'
        python.parent.mkdir(parents=True)
        python.write_text('#!/bin/sh\nexit 0\n')
        python.chmod(0o755)
        (tmp_path / 'absent/models/Qwen3-VL-2B-Instruct').mkdir(parents=True)
    tmux = tmp_path / 'tmux'
    tmux.write_text(
        '#!/bin/sh\n'
        f'printf "%s\\n" "$*" >> {shlex.quote(str(tmp_path / "tmux.calls"))}\n'
        f'if [ "$1" = has-session ]; then exit {0 if existing_supervisor else 1}; fi\n'
        f'exit {1 if tmux_failure else 0}\n')
    tmux.chmod(0o755)
    # The script uses a private HTTP port so this test cannot touch the real scorer.
    script = SCRIPT.read_text().replace('127.0.0.1:8770', f'127.0.0.1:{scorer[1]}')
    env = dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'],
               VLM_GATE=mode, VLM_GATE_DIR=str(tmp_path / 'absent'), ROS2_WS_DIR=str(tmp_path))
    return subprocess.run(['bash', '-c', script], env=env, capture_output=True, text=True, timeout=5)


def test_reboot_requirement_rejects_even_a_live_http_scorer(tmp_path, scorer):
    result = run(tmp_path, scorer, recovery='Reboot')
    assert result.returncode == 1
    assert 'requires a laptop reboot' in result.stderr


def test_healthy_device_allows_startup(tmp_path, scorer):
    assert run(tmp_path, scorer).returncode == 0


def test_metadata_without_a_device_check_is_not_ready(tmp_path, scorer):
    del scorer[0]['ready']
    assert run(tmp_path, scorer).returncode == 1


def test_older_driver_uses_the_device_health_check(tmp_path, scorer):
    assert run(tmp_path, scorer, recovery='', query_status=1).returncode == 0


def test_disabled_gate_does_not_require_gpu_recovery(tmp_path, scorer):
    assert run(tmp_path, scorer, recovery='Reboot', mode='off').returncode == 0


@pytest.mark.parametrize('mode', ['on', 'shadow'])
@pytest.mark.parametrize('existing', [False, True])
def test_unready_scorer_starts_or_reuses_supervisor_without_waiting(tmp_path, scorer, mode, existing):
    scorer[0]['ready'] = False
    result = run(tmp_path, scorer, installed=True, existing_supervisor=existing, mode=mode)
    assert result.returncode == 0, result.stderr
    assert 'without waiting for readiness' in result.stdout
    calls = (tmp_path / 'tmux.calls').read_text()
    assert ('new-session' in calls) is not existing


def test_supervisor_launch_failure_is_reported(tmp_path, scorer):
    scorer[0]['ready'] = False
    assert run(tmp_path, scorer, installed=True, tmux_failure=True).returncode != 0


@pytest.mark.parametrize('services', ['ground', 'offboard'])
@pytest.mark.parametrize('failure', [1, 23])
def test_scorer_setup_failure_cannot_abort_ground_startup(tmp_path, services, failure):
    # Run the actual helper under the front door's errexit setting, without
    # executing the front door's Docker/configuration startup side effects.
    frontdoor = (SCRIPT.parents[1] / 'px4sim').read_text()
    function = 'ensure_vlm_scorer() {' + frontdoor.split('ensure_vlm_scorer() {', 1)[1].split(
        'push_env_selection_to_drones() {', 1)[0]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    (scripts / 'ensure-vlm-scorer.sh').write_text(f'exit {failure}\n')
    script = ('set -euo pipefail\n'
              'warn() { printf "%s\\n" "$*"; }\n'
              f'compose() {{ printf "%s\\n" {shlex.quote(services)}; }}\n'
              'DC=compose\n' + function +
              '\nensure_vlm_scorer\nprintf "START_CONTAINERS\\n"\n')
    result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'VLM scorer unavailable; continuing ground startup.' in result.stdout
    assert 'START_CONTAINERS' in result.stdout


def test_aircraft_only_startup_does_not_attempt_scorer(tmp_path):
    frontdoor = (SCRIPT.parents[1] / 'px4sim').read_text()
    function = 'ensure_vlm_scorer() {' + frontdoor.split('ensure_vlm_scorer() {', 1)[1].split(
        'push_env_selection_to_drones() {', 1)[0]
    script = ('set -euo pipefail\n'
              'warn() { exit 99; }\n'
              'compose() { printf "onboard\\n"; }\n'
              'DC=compose\n' + function + '\nensure_vlm_scorer\n')
    result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
