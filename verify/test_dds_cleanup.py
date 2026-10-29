"""Check restart cleanup with isolated files and a real live lock."""

import fcntl
from pathlib import Path
import subprocess


SOURCE = (Path(__file__).resolve().parents[1] / "px4sim").read_text()
START = SOURCE.index("release_dds_shm() {")
FUNCTION = SOURCE[START:SOURCE.index("\n}", START) + 2]


def cleanup(directory):
    function = FUNCTION.replace("/dev/shm", str(directory))
    return subprocess.run(["bash", "-c", "warn() { echo \"$*\" >&2; }\n" + function + "\nrelease_dds_shm"],
                          capture_output=True, text=True, timeout=5)


def test_fresh_orphans_and_unlocked_objects_are_removed(tmp_path):
    for name in ("fastrtps_recent", "sem.fastrtps_port22463_mutex",
                 "fastrtps_unused", "fastrtps_unused_el"):
        (tmp_path / name).touch()
    assert cleanup(tmp_path).returncode == 0
    assert list(tmp_path.iterdir()) == []


def test_live_lock_prevents_restart_success_and_preserves_its_data(tmp_path):
    data = tmp_path / "fastrtps_live"
    data.write_text("live data")
    lock = tmp_path / "fastrtps_live_el"
    with lock.open("w") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        result = cleanup(tmp_path)
        assert result.returncode != 0
        assert "live locks" in result.stderr
        assert data.read_text() == "live data"
        assert lock.exists()
    assert cleanup(tmp_path).returncode == 0
    assert not data.exists()
