#!/usr/bin/env python3
"""Keep a native rosbag/video session within its container's lifetime."""

import os
import signal
import subprocess
import sys
import time


def group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_recorder(recorder):
    # The video scripts can have grandchildren. Signal the entire dedicated
    # session, even if the top-level recorder has already exited.
    for sig, grace in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
        recorder.poll()  # Reap the leader before checking its group.
        if not group_alive(recorder.pid):
            return
        try:
            os.killpg(recorder.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            recorder.poll()
            if not group_alive(recorder.pid):
                return
            time.sleep(0.1)


def main():
    container, recorder_path = sys.argv[1:]
    stopping = False

    def request_stop(_sig, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    # A fixed container ID prevents a replacement container from extending an
    # old recording. docker wait also returns if it was stopped before we ran.
    watcher = subprocess.Popen(
        ["docker", "wait", container], stdout=subprocess.DEVNULL)
    recorder = None
    try:
        recorder = subprocess.Popen([recorder_path], start_new_session=True)
        while not stopping and watcher.poll() is None and recorder.poll() is None:
            time.sleep(0.1)
    finally:
        if recorder is not None:
            stop_recorder(recorder)
        if watcher.poll() is None:
            watcher.terminate()
            try:
                watcher.wait(timeout=2)
            except subprocess.TimeoutExpired:
                watcher.kill()
                watcher.wait()
    return 0


if __name__ == "__main__":
    sys.exit(main())
