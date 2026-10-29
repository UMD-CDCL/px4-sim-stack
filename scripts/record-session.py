#!/usr/bin/env python3
"""Keep a native rosbag/video session within its container's lifetime."""

import os
import json
from pathlib import Path
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
    grace = float(os.environ.get("PX4SIM_RECORD_STOP_GRACE", "30"))
    for sig, grace in ((signal.SIGINT, grace), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
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


def recording_health(directory, video=True, final=False):
    directory = Path(directory)
    bags = list(directory.glob("*.mcap"))
    videos = [p for p in directory.glob("video-*.ts") if p.stat().st_size > 0]
    streams = []
    for path in sorted(directory.glob("video-*.status.json")):
        try:
            streams.append(json.loads(path.read_text()))
        except (OSError, ValueError):
            continue
    bag_bytes = sum(p.stat().st_size for p in bags)
    finalized = bool(bags) and (directory / "metadata.yaml").is_file()
    for path in bags:
        with path.open("rb") as stream:
            if path.stat().st_size < 16:
                finalized = False
            else:
                stream.seek(-8, 2)
                finalized = finalized and stream.read() == b"\x89MCAP0\r\n"
    receiving = sum(s.get("state") == "recording" and time.time() - s.get("updated", 0) < 5 for s in streams)
    complete = finalized and (not video or bool(videos))
    if final:
        summary = "saved" if complete else "FAILED: incomplete bag or no video"
    else:
        summary = f"bag {bag_bytes // 1024} KiB"
        if video:
            summary += f", video {receiving}/{len(streams)} receiving" if streams else f", {len(videos)} video files"
    return dict(summary=summary, finalized=finalized, complete=complete,
                bag_bytes=bag_bytes, video_files=len(videos), streams=streams,
                updated=time.time())


def save_health(directory, video=True, final=False):
    if directory is None or not directory.is_dir():
        return None
    state = recording_health(directory, video, final)
    path = directory / "recording-status.json"
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2) + "\n")
    temporary.replace(path)
    return state


def main():
    if sys.argv[1:2] == ["--health"]:
        path = Path(sys.argv[2]) / "recording-status.json"
        try:
            state = json.loads(path.read_text())
            print(state["summary"] if time.time() - state["updated"] < 10 else "status is stale")
        except (OSError, ValueError, KeyError):
            print("starting recorders")
        return 0
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
    directory = (Path(os.environ["BAG_ROOT"]) / os.environ["BAG_NAME"]
                 if "BAG_ROOT" in os.environ and "BAG_NAME" in os.environ else None)
    video = os.environ.get("RECORD_VIDEO", "1") == "1"
    state = None
    try:
        recorder = subprocess.Popen([recorder_path], start_new_session=True)
        last_health = 0
        while not stopping and watcher.poll() is None and recorder.poll() is None:
            if time.monotonic() - last_health >= 1:
                save_health(directory, video)
                last_health = time.monotonic()
            time.sleep(0.1)
    finally:
        if recorder is not None:
            stop_recorder(recorder)
            state = save_health(directory, video, final=True)
        if watcher.poll() is None:
            watcher.terminate()
            try:
                watcher.wait(timeout=2)
            except subprocess.TimeoutExpired:
                watcher.kill()
                watcher.wait()
    if state:
        print(f"[INFO] Recording {state['summary']}: {directory}", flush=True)
        return 0 if state["complete"] else 1
    if directory is not None:
        print(f"[ERROR] Recording did not create its output directory: {directory}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
