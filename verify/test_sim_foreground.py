"""Exercise simulator fleet startup with a stub PX4 binary; no Docker/Gazebo."""
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "modules/sim/entrypoint.sh").read_text()


class SimForeground(unittest.TestCase):
    def run_fleet(self, active):
        with tempfile.TemporaryDirectory(prefix="px4sim-foreground-") as directory:
            root = Path(directory)
            binary = root / "build/px4_sitl_default/bin/px4"
            binary.parent.mkdir(parents=True)
            binary.write_text('''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
entry = {"instance": int(args[args.index("-i") + 1]), "daemon": "-d" in args}
fd = os.open(os.environ["TASK_TRACE"], os.O_WRONLY | os.O_APPEND | os.O_CREAT, 0o600)
os.write(fd, (json.dumps(entry) + "\\n").encode())
os.close(fd)
# A distinctive foreground exit proves the entrypoint execs this PX4.
sys.exit(0 if entry["daemon"] else 17)
''')
            binary.chmod(0o755)
            # Run the actual initialization, selection and startup function.
            # Skip Gazebo, rendering and encoding, and disable optional helpers.
            setup = SOURCE[:SOURCE.index("\nradians()")].replace(". /opt/sim/zoom.sh", ":")
            startup = SOURCE[SOURCE.index("\nstart_vehicle() {"):]
            script = setup + '\nLOG_DIR="$TASK_LOG_DIR"\nchildren=()\n' + startup
            env = dict(os.environ, PX4_DIR=str(root), SCENES_DIR=str(root / "scenes"),
                       UAS_FLEET="chimera_v3 chimera_v3 chimera_v2 chimera_v2",
                       UAS_ACTIVE=active, UAS_BASE="10", HOLD_STREAM_RATES="0",
                       GIMBAL_RANGEFINDER="0", UAS_START_DELAY_S="0.1",
                       TASK_TRACE=str(root / "trace"), TASK_LOG_DIR=str(root / "logs"))
            result = subprocess.run(["bash"], input=script, env=env, text=True,
                                    capture_output=True, timeout=10)
            trace = root / "trace"
            entries = [json.loads(line) for line in trace.read_text().splitlines()] if trace.exists() else []
            return result, entries

    def test_uas13_alone_is_foreground(self):
        result, entries = self.run_fleet("13")
        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertEqual(entries, [{"instance": 12, "daemon": False}])
        self.assertIn("console for uas13", result.stdout)

    def test_partial_fleet_uses_lowest_active_vehicle(self):
        result, entries = self.run_fleet("14,12")
        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertEqual(entries, [{"instance": 13, "daemon": True}, {"instance": 11, "daemon": False}])

    def test_full_fleet_retains_uas11_foreground(self):
        result, entries = self.run_fleet("")
        self.assertEqual(result.returncode, 17, result.stderr)
        self.assertEqual(entries, [{"instance": 13, "daemon": True}, {"instance": 12, "daemon": True},
                                   {"instance": 11, "daemon": True}, {"instance": 10, "daemon": False}])

    def test_empty_selection_fails_before_starting_px4(self):
        result, entries = self.run_fleet("19")
        self.assertEqual(result.returncode, 1)
        self.assertIn("No active vehicles selected", result.stderr)
        self.assertEqual(entries, [])


if __name__ == "__main__":
    unittest.main()
