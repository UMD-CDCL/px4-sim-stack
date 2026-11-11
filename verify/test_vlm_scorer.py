import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading
import time
import urllib.request
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
        installed=False, existing_supervisor=False, tmux_failure=False, missing_tmux=False,
        args=(), tmux_body=None, edits=()):
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
        + (tmux_body if tmux_body is not None else
           f'if [ "$1" = has-session ]; then exit {0 if existing_supervisor else 1}; fi\n'
           f'exit {1 if tmux_failure else 0}\n'))
    tmux.chmod(0o755)
    if missing_tmux:
        tmux.unlink()
        (tmp_path / 'python3').symlink_to('/usr/bin/python3')
    # The script uses a private HTTP port so this test cannot touch the real scorer.
    script = SCRIPT.read_text().replace('127.0.0.1:8770', f'127.0.0.1:{scorer[1]}')
    for old, new in edits:
        script = script.replace(old, new)
    env = dict(os.environ, PATH=(str(tmp_path) if missing_tmux else
                               str(tmp_path) + os.pathsep + os.environ['PATH']),
               VLM_GATE=mode, VLM_GATE_DIR=str(tmp_path / 'absent'), ROS2_WS_DIR=str(tmp_path))
    return subprocess.run(['/bin/bash', '-c', script, 'ensure-vlm-scorer.sh', *args],
                          env=env, capture_output=True, text=True, timeout=5)


def test_reboot_requirement_rejects_even_a_live_http_scorer(tmp_path, scorer):
    result = run(tmp_path, scorer, recovery='Reboot')
    assert result.returncode == 20
    assert 'requires a laptop reboot' in result.stderr


def test_healthy_device_allows_startup(tmp_path, scorer):
    assert run(tmp_path, scorer).returncode == 0


def test_metadata_without_a_device_check_is_not_ready(tmp_path, scorer):
    del scorer[0]['ready']
    assert run(tmp_path, scorer).returncode == 22


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
    assert run(tmp_path, scorer, installed=True, tmux_failure=True).returncode == 23


def test_missing_tmux_has_a_specific_setup_code(tmp_path, scorer):
    scorer[0]['ready'] = False
    result = run(tmp_path, scorer, installed=True, missing_tmux=True)
    assert result.returncode == 21
    assert 'needs tmux' in result.stderr


@pytest.mark.parametrize('services', ['ground', 'offboard'])
@pytest.mark.parametrize('failure,issue', [
    (1, 'setup_failed'), (20, 'setup_gpu_reboot'), (21, 'setup_tmux_missing'),
    (22, 'setup_installation_missing'), (23, 'setup_supervisor_failed')])
def test_scorer_setup_failure_cannot_abort_ground_startup(tmp_path, services, failure, issue):
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
              '\nensure_vlm_scorer\nprintf "START_CONTAINERS:%s\\n" "$VLM_SCORER_STARTUP_ISSUE"\n')
    result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    assert 'VLM scorer unavailable; continuing ground startup.' in result.stdout
    assert f'START_CONTAINERS:{issue}' in result.stdout


def test_aircraft_only_startup_does_not_attempt_scorer(tmp_path):
    frontdoor = (SCRIPT.parents[1] / 'px4sim').read_text()
    function = 'ensure_vlm_scorer() {' + frontdoor.split('ensure_vlm_scorer() {', 1)[1].split(
        'push_env_selection_to_drones() {', 1)[0]
    script = ('set -euo pipefail\n'
              'warn() { exit 99; }\n'
              'compose() { printf "onboard\\n"; }\n'
              'DC=compose\nexport VLM_SCORER_STARTUP_ISSUE=setup_gpu_reboot\n' + function +
              '\nensure_vlm_scorer\ntest -z "$VLM_SCORER_STARTUP_ISSUE"\n')
    result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr


def test_compose_passes_setup_hint_to_both_ground_services():
    import yaml
    config = yaml.safe_load((SCRIPT.parents[1] / 'compose.yaml').read_text())
    for service in ['ground', 'offboard']:
        assert config['services'][service]['environment']['VLM_SCORER_STARTUP_ISSUE'] == '${VLM_SCORER_STARTUP_ISSUE:-}'


# a scorer process that a fake tmux starts and kills, so a restart can be watched
# from outside: the pid on /health changes only if the old scorer was replaced
LIVE_SCORER = '''
import json, os, sys
from http.server import BaseHTTPRequestHandler, HTTPServer
ready = os.environ['FAKE_READY'] == '1'
class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        body = json.dumps(dict(model='test', device='cuda:0', prompts=4, ready=ready,
                               pid=os.getpid())).encode()
        self.send_response(200)
        self.end_headers()
        self.wfile.write(body)
    def log_message(self, *args):
        pass
HTTPServer(('127.0.0.1', int(sys.argv[1])), Handler).serve_forever()
'''


def health_pid(port):
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open(f'http://127.0.0.1:{port}/health', timeout=1) as response:
        return json.load(response)['pid']


@pytest.fixture
def live(tmp_path):
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    (tmp_path / 'scorer.py').write_text(LIVE_SCORER)
    q = lambda name: shlex.quote(str(tmp_path / name))

    def tmux_body(new_ready=True):
        return (
            'case "$1" in\n'
            f'has-session) test -f {q("scorer.pid") } ;;\n'
            f'kill-session) kill "$(cat {q("scorer.pid")})"; rm -f {q("scorer.pid")} ;;\n'
            f'new-session) FAKE_READY={int(new_ready)} python3 {q("scorer.py")} {port} '
            '>/dev/null 2>&1 </dev/null &\n'
            f'  echo $! > {q("scorer.pid")}; echo $! >> {q("all.pids")} ;;\n'
            'esac\n')

    # the scorer that runs before the front door is called
    (tmp_path / 'tmux').write_text('#!/bin/sh\n' + tmux_body())
    (tmp_path / 'tmux').chmod(0o755)
    subprocess.run([str(tmp_path / 'tmux'), 'new-session'], check=True)
    deadline = time.monotonic() + 5
    while True:
        try:
            first = health_pid(port)
            break
        except OSError:
            assert time.monotonic() < deadline, 'the fake scorer did not start'
            time.sleep(0.05)
    yield dict(port=port, first=first, tmux_body=tmux_body)
    for pid in (tmp_path / 'all.pids').read_text().split():
        try:
            os.kill(int(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass


def calls(tmp_path):
    path = tmp_path / 'tmux.calls'
    return path.read_text() if path.exists() else ''


def test_restart_replaces_a_running_scorer_so_it_loads_new_code(tmp_path, live):
    result = run(tmp_path, (None, live['port']), installed=True, args=['--restart'],
                 tmux_body=live['tmux_body']())
    assert result.returncode == 0, result.stderr
    assert 'kill-session' in calls(tmp_path)
    assert health_pid(live['port']) != live['first']
    assert 'VLM scorer ready' in result.stdout


def test_start_reuses_a_running_scorer(tmp_path, live):
    result = run(tmp_path, (None, live['port']), installed=True, tmux_body=live['tmux_body']())
    assert result.returncode == 0, result.stderr
    assert calls(tmp_path) == ''
    assert health_pid(live['port']) == live['first']


def test_restart_with_the_gate_off_leaves_the_scorer_alone(tmp_path, live):
    result = run(tmp_path, (None, live['port']), installed=True, args=['--restart'], mode='off',
                 tmux_body=live['tmux_body']())
    assert result.returncode == 0, result.stderr
    assert calls(tmp_path) == ''
    assert health_pid(live['port']) == live['first']


def test_restart_continues_when_the_new_scorer_is_not_ready(tmp_path, live):
    result = run(tmp_path, (None, live['port']), installed=True, args=['--restart'],
                 tmux_body=live['tmux_body'](new_ready=False),
                 edits=[('ready_wait=60', 'ready_wait=1')])
    assert result.returncode == 0, result.stderr
    assert 'new-session' in calls(tmp_path)
    assert 'not ready after 1 s' in result.stderr


def test_restart_names_a_scorer_it_cannot_stop_and_reuses_it(tmp_path, scorer):
    # the session is killed, but the scorer on the port lives outside it
    result = run(tmp_path, scorer, installed=True, existing_supervisor=True, args=['--restart'],
                 edits=[('stop_wait=10', 'stop_wait=1')])
    assert result.returncode == 0, result.stderr
    assert 'still held' in result.stderr
    assert 'new-session' not in calls(tmp_path)


def test_front_door_asks_for_a_scorer_restart_only_on_restart(tmp_path):
    frontdoor = (SCRIPT.parents[1] / 'px4sim').read_text()
    assert 'ensure_vlm_scorer "$cmd"\n\t$DC up -d --no-build --pull never' in frontdoor
    function = 'ensure_vlm_scorer() {' + frontdoor.split('ensure_vlm_scorer() {', 1)[1].split(
        'push_env_selection_to_drones() {', 1)[0]
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    (scripts / 'ensure-vlm-scorer.sh').write_text('printf "[%s]\\n" "$*"\n')
    for cmd, expected in [('restart', '[--restart]'), ('start', '[]'), ('up', '[]'), ('', '[]')]:
        script = ('set -euo pipefail\nwarn() { :; }\ncompose() { printf "ground\\n"; }\n'
                  'DC=compose\n' + function + f'\nensure_vlm_scorer {cmd}\n')
        result = subprocess.run(['bash', '-c', script], cwd=tmp_path,
                                capture_output=True, text=True, timeout=5)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == expected, cmd
