"""Check dependency preparation order without Docker or network access."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "px4sim").read_text()
PREPARE = SOURCE[SOURCE.index("prepare_stack() {"):SOURCE.index("\nstop_stack() {")]


class PrepareCache(unittest.TestCase):
    def prepare(self, *, pinned=False, fail=False):
        with tempfile.TemporaryDirectory(prefix="px4sim-prepare-") as directory:
            root = Path(directory)
            (root / "scripts").mkdir()
            (root / "scripts/build-contexts.sh").write_text("echo 12.6\n")
            script = r'''
set -euo pipefail
record() { printf '%s\n' "$*" >> "$TRACE"; }
add_fleet_profiles() { :; }
say() { :; }
aircraft_selected() { return 0; }
build_stack() { record application-build; }
python3() {
    record dependency-build
    [ "$FAIL_BUILD" = 0 ]
}
docker() {
    record "docker $*"
    if [ "$1 $2" = 'image inspect' ]; then
        [[ "$3" = px4simstack/* ]] && [ "$PX4SIM_PINNED_DEPS" = 1 ]
    fi
}
DC='docker compose'
DS_IMAGE='example/deepstream:7.1'
DS_TAG=7.1
''' + PREPARE + "\nprepare_stack aircraft\n"
            trace = root / "trace"
            result = subprocess.run(
                ["bash"], input=script, text=True, cwd=root, capture_output=True,
                env=dict(os.environ, TRACE=str(trace),
                         PX4SIM_PINNED_DEPS=str(int(pinned)), FAIL_BUILD=str(int(fail))),
            )
            return result, trace.read_text().splitlines()

    def test_buildkit_extracts_before_engine_pull(self):
        result, calls = self.prepare()
        self.assertEqual(result.returncode, 0, result.stderr)
        build = calls.index("dependency-build")
        for image in ("example/deepstream:7.1", "ubuntu:24.04", "debian:bookworm-slim"):
            self.assertGreater(calls.index("docker pull " + image), build)
        self.assertLess(calls.index("docker pull example/deepstream:7.1"),
                        calls.index("application-build"))

    def test_pinned_dependencies_still_retain_offline_parents(self):
        result, calls = self.prepare(pinned=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("dependency-build", calls)
        self.assertIn("docker pull example/deepstream:7.1", calls)
        self.assertIn("application-build", calls)

    def test_failed_dependency_build_does_not_pre_pull(self):
        result, calls = self.prepare(fail=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(calls, ["dependency-build"])


if __name__ == "__main__":
    unittest.main()
