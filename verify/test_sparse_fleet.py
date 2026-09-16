"""Contract tests for sparse active fleets.

UAS_ACTIVE is a list of one-based UAS_FLEET slots.  The configured fleet is
still the model table; selecting slots must never compact that table.
"""
from pathlib import Path
import os
import subprocess
import unittest


ROOT = Path(__file__).parents[1]


def fleet_shell(*, active="1,3,4", command="fleet_numbers"):
    script = f'. ./scripts/fleet.sh; {command}'
    env = dict(os.environ, UAS_BASE="10",
               UAS_FLEET="chimera_v3 chimera_v3 chimera_v2 chimera_v2",
               UAS_ACTIVE=active)
    return subprocess.run(["bash", "-c", script], cwd=ROOT, env=env,
                          text=True, capture_output=True)


class SparseFleet(unittest.TestCase):
    def test_active_slots_preserve_numbers_and_models(self):
        result = fleet_shell(command='printf "numbers=%s\\n" "$(fleet_numbers | paste -sd, -)"; for n in $(fleet_numbers); do printf "%s=%s " "$n" "$(model_of "$n")"; done')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("numbers=11,13,14", result.stdout)
        self.assertIn("11=chimera_v3", result.stdout)
        self.assertIn("13=chimera_v2", result.stdout)
        self.assertIn("14=chimera_v2", result.stdout)

    def test_empty_active_selects_all_slots(self):
        result = fleet_shell(active="", command='fleet_numbers | paste -sd, -')
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "11,12,13,14")

    def test_invalid_or_duplicate_slots_are_rejected(self):
        for active in ("0", "5", "3,3", "three"):
            with self.subTest(active=active):
                result = fleet_shell(active=active)
                self.assertNotEqual(result.returncode, 0)

    def test_px4sim_profile_generation_consumes_active_numbers(self):
        text = (ROOT / "px4sim").read_text()
        start = text.index("fleet_profiles()")
        end = text.index("add_fleet_profiles()", start)
        profile_function = text[start:end]
        self.assertIn("for n in $(fleet_numbers)", profile_function)
        self.assertNotIn("FIRST_UAS", profile_function)


if __name__ == "__main__":
    unittest.main()
