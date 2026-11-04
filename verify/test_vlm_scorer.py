import json
import os
from pathlib import Path
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


def run(tmp_path, scorer, recovery='None', mode='on', query_status=0):
    binary = tmp_path / 'nvidia-smi'
    binary.write_text(f"#!/bin/sh\nprintf '%s\\n' '{recovery}'\nexit {query_status}\n")
    binary.chmod(0o755)
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
