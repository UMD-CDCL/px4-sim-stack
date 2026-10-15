"""Native rosbag/video processes must not outlive their ROS container."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]


def wait_for(predicate, timeout=15):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError("recording session did not reach the expected state")


@pytest.fixture
def session(tmp_path):
    docker = tmp_path / "docker"
    docker.write_text("#!/usr/bin/env python3\n"
                      "import os, pathlib, time\n"
                      "while not pathlib.Path(os.environ['CONTAINER_STOPPED']).exists(): time.sleep(.05)\n")
    docker.chmod(0o755)
    recorder = tmp_path / "recorder"
    recorder.write_text('''#!/usr/bin/env python3
import os, pathlib, signal, subprocess, sys, time
root = pathlib.Path(os.environ['SESSION_TEST_DIR'])
video = len(sys.argv) > 1
stopped = False
def stop(sig, frame):
    global stopped
    if video and (root / 'ignore-video-signals').exists():
        return
    stopped = True
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
if not video:
    child = subprocess.Popen([sys.executable, __file__, 'video'])
else:
    (root / 'video-ready').touch()
(root / ('video-pid' if video else 'bag-pid')).write_text(str(os.getpid()))
while not stopped:
    if not video and (root / 'bag-exit').exists():
        break
    time.sleep(.05)
(root / ('video-finalized' if video else 'bag-finalized')).touch()
if not video and stopped:
    child.wait()
''')
    recorder.chmod(0o755)
    env = dict(os.environ, PATH=f"{tmp_path}:{os.environ['PATH']}",
               SESSION_TEST_DIR=str(tmp_path), CONTAINER_STOPPED=str(tmp_path / "container-stopped"))
    process = subprocess.Popen(
        [sys.executable, str(ROOT / "scripts/record-session.py"), "original-container-id", str(recorder)],
        env=env, start_new_session=True)
    wait_for(lambda: (tmp_path / "video-ready").exists())
    try:
        yield process, tmp_path
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=15)


@pytest.mark.parametrize("trigger", ["container-stopped", "manual-stop", "bag-exit"])
def test_session_finalizes_bag_and_video(session, trigger):
    process, directory = session
    if trigger == "manual-stop":
        process.send_signal(signal.SIGINT)
    else:
        (directory / trigger).touch()
    assert process.wait(timeout=15) == 0
    assert (directory / "bag-finalized").exists()
    assert (directory / "video-finalized").exists()


def test_unresponsive_video_is_force_stopped(session):
    process, directory = session
    (directory / "ignore-video-signals").touch()
    video_pid = int((directory / "video-pid").read_text())
    (directory / "container-stopped").touch()
    assert process.wait(timeout=15) == 0
    status = Path(f"/proc/{video_pid}/stat")
    assert not status.exists() or status.read_text().split(") ", 1)[1].startswith("Z ")


def test_stack_stop_finalizes_recording_before_compose_down(session):
    process, directory = session
    source = (ROOT / "px4sim").read_text()

    def function(name):
        start = source.index(f"{name}() {{")
        return source[start:source.index("\n}", start) + 2]

    (directory / "pid").write_text(f"{process.pid}\n")
    # Reap the supervisor while the shell's kill -0 checks its lifetime.
    import threading
    reaper = threading.Thread(target=process.wait)
    reaper.start()
    script = "\n".join(function(n) for n in
                       ("recording_status", "stop_native_recording", "stop_stack"))
    script += '''
set -euo pipefail
RECORD_PID_FILE=$1/pid
RECORD_MODE_FILE=$1/mode
RECORD_BAG_FILE=$1/bag
RECORD_SIM_FILE=$1/sim
ALL_PROFILES=aircraft
DC=compose_down
say() { :; }
die() { exit 1; }
unit_owns_stack() { return 1; }
stop_container_log() { :; }
release_dds_shm() { :; }
compose_down() {
    test -f "${RECORD_PID_FILE%/*}/bag-finalized"
    test -f "${RECORD_PID_FILE%/*}/video-finalized"
    test ! -f "$RECORD_PID_FILE"
}
stop_stack
'''
    result = subprocess.run(["bash", "-c", script, "_", str(directory)],
                            capture_output=True, text=True, timeout=20)
    reaper.join(timeout=15)
    assert result.returncode == 0, result.stderr
