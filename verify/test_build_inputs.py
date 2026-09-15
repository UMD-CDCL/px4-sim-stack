"""Build-input and mounted-parser regressions; no Docker or source edits."""
import concurrent.futures
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


REPO = Path(__file__).resolve().parents[1]


class BuildInputs(unittest.TestCase):
    def test_staging_ignores_models_and_tracks_source_deletions(self):
        with tempfile.TemporaryDirectory(prefix="px4sim-inputs-") as tmp:
            root = Path(tmp)
            scripts = root / "repo/scripts"
            scripts.mkdir(parents=True)
            script = scripts / "build-contexts.sh"
            shutil.copy2(REPO / "scripts/build-contexts.sh", script)
            ws, deploy = root / "ws", root / "deploy"
            sources = {
                "5g_drone/umd_uas/__init__.py": "",
                "5g_drone/umd_uas/ds_ros_pipeline/infer_configs.py":
                    '_DS_CUDA_VERSIONS = {"7.1": "12.6"}\n',
                "5g_drone/config/params.yaml": "enabled: true\n",
                "5g_drone/perception_models/a.onnx": "weights",
                "5g_drone/models/mesh.stl": "runtime mesh",
                "tracking_test_5g/tracking_test/__init__.py": "",
                "5g_drone/build/stale.py": "generated",
                "5g_drone/.git/index": "git state",
                "5g_drone/config/deepstream/nvdsinfer_custom_impl_Yolo/Makefile": "all:\n",
                "MAVInsight/models/vehicle.py": "# actual Python code\n",
                "MAVInsight/resource/mesh.stl": "installed mesh",
                "px4_msgs/msg/Example.msg": "float32 x\n",
                "px4_msgs/README.md": "message documentation",
                "cdcl_umd_msgs/msg/Example.msg": "float32 y\n",
            }
            for name, content in sources.items():
                path = ws / "src" / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
            for name in ("submodules/mavros", "submodules/angles", "remote/mavros_patch",
                         "submodules/geographic_info/geographic_msgs"):
                (deploy / name).mkdir(parents=True)
            env = dict(os.environ, ROS2_WS_DIR=str(ws), CHIMERA_DEPLOY_DIR=str(deploy), DS_VERSION="7.1")
            staged = root / "repo/.build-contexts"

            def stage():
                self.assertEqual(subprocess.check_output(["bash", str(script)], env=env, text=True).strip(), "12.6")
                return {str(p.relative_to(staged)): p.read_bytes()
                        for p in staged.rglob("*") if p.is_file()}

            before = stage()
            self.assertIn("mavinsight/models/vehicle.py", before)
            self.assertIn("mavinsight/resource/mesh.stl", before)
            self.assertNotIn("umd_uas/models/mesh.stl", before)
            self.assertFalse(any("perception_models" in p or ".git" in p or "/build/" in p for p in before))
            (ws / "src/5g_drone/perception_models/a.onnx").write_text("new weights")
            (ws / "src/5g_drone/.git/index").write_text("new git state")
            (ws / "src/px4_msgs/README.md").write_text("updated message documentation")
            self.assertEqual(stage(), before)
            # General inference edits cannot change the extracted CUDA input.
            config = ws / "src/5g_drone/umd_uas/ds_ros_pipeline/infer_configs.py"
            config.write_text(config.read_text() + "# unrelated change\n")
            after = stage()
            changed = {p for p in before if before[p] != after[p]}
            self.assertEqual(changed, {"umd_uas/umd_uas/ds_ros_pipeline/infer_configs.py"})
            (ws / "src/5g_drone/config/params.yaml").unlink()
            self.assertNotIn("umd_uas/config/params.yaml", stage())

    def test_parser_changes_within_same_release_and_concurrent_starts(self):
        with tempfile.TemporaryDirectory(prefix="px4sim-parser-") as tmp:
            root = Path(tmp)
            source, models = root / "parser.so", root / "models"
            models.mkdir()
            command = ["bash", str(REPO / "modules/onboard/install-parser.sh"), str(source), str(models), "7.1"]

            def install():
                subprocess.run(command, check=True, stdout=subprocess.DEVNULL)

            source.write_bytes(b"first parser")
            install()
            destination = models / "libnvdsinfer_custom_impl_Yolo.so"
            first_mtime = destination.stat().st_mtime_ns
            install()
            self.assertEqual(destination.stat().st_mtime_ns, first_mtime)
            source.write_bytes(b"new parser, same DeepStream release" * 10000)
            with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
                list(executor.map(lambda _: install(), range(4)))
            self.assertEqual(destination.read_bytes(), source.read_bytes())
            stamp = "7.1:" + hashlib.sha256(source.read_bytes()).hexdigest()
            self.assertEqual((models / ".parser-deepstream").read_text().strip(), stamp)
            self.assertFalse(list(models.glob(".parser.*")))
            self.assertFalse(list(models.glob(".parser-stamp.*")))


if __name__ == "__main__":
    unittest.main()
