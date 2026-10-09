"""Bake permission and Compose build-plan regressions; no Docker builds."""
import importlib.util
import json
from pathlib import Path
import subprocess
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("compose_build", ROOT / "scripts/compose-build.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ComposeBuild(unittest.TestCase):
    def plan(self, network="host"):
        return {"group": {"default": {"targets": ["ros-core"]}}, "target": {
            "ros-core": {"network": network, "context": str(ROOT / "modules/ros-deps"),
                         "contexts": {"base": "target:ros-deps", "source": str(ROOT / ".build-contexts/cdcl_umd_msgs")},
                         "tags": ["px4simstack/ros-core:7.1"], "output": ["type=docker"]}}}

    def test_host_network_grant_and_contexts_are_explicit(self):
        command = module.bake_command(self.plan())
        self.assertIn("--allow=network.host", command)
        self.assertIn(f"--allow=fs.read={ROOT / 'modules/ros-deps'}", command)
        self.assertIn(f"--allow=fs.read={ROOT / '.build-contexts/cdcl_umd_msgs'}", command)
        self.assertFalse(any("target:" in arg or "security.insecure" in arg or "fs.write" in arg for arg in command))

    def test_offline_plan_does_not_grant_host_network(self):
        self.assertNotIn("--allow=network.host", module.bake_command(self.plan("none")))

    def test_compose_plan_and_service_selection_are_preserved(self):
        plan = json.dumps(self.plan())
        with patch.object(module.subprocess, "run", side_effect=[
            subprocess.CompletedProcess([], 0, "--print", ""),
            subprocess.CompletedProcess([], 0, plan, ""),
            subprocess.CompletedProcess([], 0),
        ]) as run:
            self.assertEqual(module.main(["ros-core"]), 0)
        self.assertEqual(run.call_args_list[1].args[0], ["docker", "compose", "build", "--print", "ros-core"])
        self.assertEqual(run.call_args_list[2].kwargs["input"], plan)
        self.assertIn("--allow=network.host", run.call_args_list[2].args[0])

    def test_failed_plan_does_not_start_build(self):
        with patch.object(module.subprocess, "run", side_effect=[
            subprocess.CompletedProcess([], 0, "--print", ""),
            subprocess.CompletedProcess([], 2, "", ""),
        ]) as run:
            self.assertEqual(module.main([]), 2)
            self.assertEqual(run.call_count, 2)

    def test_older_compose_uses_existing_build(self):
        with patch.object(module.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "build help", "")), \
             patch.object(module.os, "execvp", side_effect=SystemExit(0)) as execvp:
            with self.assertRaises(SystemExit):
                module.main(["sim"])
            execvp.assert_called_once_with("docker", ["docker", "compose", "build", "sim"])


if __name__ == "__main__":
    unittest.main()
