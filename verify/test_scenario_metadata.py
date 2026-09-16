"""Scenario selections resolve to an existing parent scene."""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ROOT / "modules/sim/scenes/scenarios"
WORLDS = ROOT / "modules/sim/scenes/worlds"

class ScenarioMetadata(unittest.TestCase):
    def test_every_scenario_declares_existing_parent_scene(self):
        for path in sorted(SCENARIOS.glob("*.yaml")):
            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            parent = data.get("parent_scene")
            self.assertIsInstance(parent, str, path.name)
            self.assertTrue((WORLDS / f"{parent}.sdf").is_file() or (WORLDS / f"{parent}_surface.json").is_file(), path.name)

    def test_resolver_rejects_missing_parent_scene(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); (root / "scenarios").mkdir(); (root / "worlds").mkdir()
            (root / "scenarios/bad.yaml").write_text("parent_scene: absent\n")
            result = subprocess.run(["bash", "-c", "source scripts/scenario-metadata.sh; validate_scenario_selection bad"], cwd=ROOT, env=dict(os.environ, SCENES_DIR=str(root)), text=True, capture_output=True)
            self.assertNotEqual(result.returncode, 0)

    def test_cli_uses_metadata_resolver(self):
        source = (ROOT / "px4sim").read_text(encoding="utf-8")
        self.assertIn("validate_scenario_selection", source)
        self.assertIn('export SCENE="$parent_scene"', source)

if __name__ == "__main__":
    unittest.main()
