#!/usr/bin/env python3
"""Exercise real package cache boundaries without editing source checkouts.

Run after ./px4sim prepare. Builds a temporary image tag from temporary copies
of the staged contexts. Does not start the stack or change its image tags.
"""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]
STAGES = {"px4_msgs", "cdcl_umd_msgs", "mavinsight", "umd_uas", "mavros", "parser"}


def main():
    env = os.environ.copy()
    env["PX4SIM_BUILD_NETWORK"] = "none"
    # The wrapper supplies hardware settings and current filtered inputs.
    subprocess.run([str(ROOT / "px4sim"), "build", "ros-base"], cwd=ROOT, check=True)
    # Match the front door's local dependency tag, including hardware overrides.
    env["DS_TAG"] = subprocess.check_output(
        ["bash", "-c", "set -a; . ./.env; ./scripts/ds-select.sh --tag"],
        cwd=ROOT, text=True,
    ).strip()

    with tempfile.TemporaryDirectory(prefix="px4sim-cache-test-") as temporary:
        root = Path(temporary)
        contexts = root / "contexts"
        shutil.copytree(ROOT / ".build-contexts", contexts)
        image = "px4simstack/cache-test:" + root.name
        override = root / "compose.json"
        override.write_text(json.dumps({"services": {"ros-base": {
            "image": image,
            "build": {"additional_contexts": {
                f"{name}_src": str(contexts / name)
                for name in ("px4_msgs", "cdcl_umd_msgs", "mavinsight", "umd_uas",
                             "mavros", "angles", "mavros_patch", "yolo")
            }},
        }}}))
        command = ["docker", "compose", "--progress", "plain", "-f",
                   str(ROOT / "compose.yaml"), "-f", str(override), "build", "ros-base"]

        def build(label, changed):
            started = time.monotonic()
            result = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            log = result.stdout
            if result.returncode:
                raise RuntimeError(log)
            steps = {}
            for match in re.finditer(r"^#(\d+) \[(?:ros-base )?(\w+) [^\]]+\] RUN ", log, re.M):
                number, stage = match.groups()
                if stage in STAGES:
                    steps[stage] = bool(re.search(rf"^#{number} CACHED$", log, re.M))
            assert set(steps) == STAGES, (label, "Missing build steps", steps, log)
            rebuilt = {stage for stage, cached in steps.items() if not cached}
            assert rebuilt == changed, (label, rebuilt, changed, log)
            print(f"PASS {label}: {time.monotonic() - started:.1f}s; rebuilt {sorted(rebuilt)}",
                  flush=True)

        def change(relative, addition, expected):
            path = contexts / relative
            before = path.read_bytes()
            try:
                path.write_bytes(before + addition)
                build(relative, expected)
            finally:
                path.write_bytes(before)

        try:
            build("unchanged", set())
            change("umd_uas/umd_uas/__init__.py", b"\n# cache regression probe\n", {"umd_uas"})
            change("mavinsight/mavinsight/__init__.py", b"\n# cache regression probe\n", {"mavinsight"})
            change("umd_uas/umd_uas/ds_ros_pipeline/infer_configs.py",
                   b"\n# unrelated inference config edit\n", {"umd_uas"})
            change("yolo/Makefile", b"\n# cache regression probe\n", {"parser"})
            # A message input change must not rebuild the other message package.
            message = next((contexts / "cdcl_umd_msgs" / "msg").glob("*.msg"))
            change(str(message.relative_to(contexts)), b"\n# cache regression probe\n",
                   {"cdcl_umd_msgs"})
            build("restored inputs", set())
            subprocess.run([
                "docker", "run", "--rm", "--network", "none", "--entrypoint", "bash", image,
                "-ec", ". /usr/local/bin/ros-env.sh; "
                "for p in px4_msgs cdcl_umd_msgs mavinsight umd_uas mavros; do ros2 pkg prefix $p; done; "
                "python3 -c 'import umd_uas, mavinsight, models; "
                "from rosidl_generator_py import import_type_support; "
                "import_type_support(\"px4_msgs\"); import_type_support(\"cdcl_umd_msgs\")'"
            ], check=True)
            print("PASS assembled ROS environment and native message type support offline", flush=True)
        finally:
            # Remove only the uniquely named image created by this test. Build
            # cache stays available for the real application images.
            subprocess.run(["docker", "image", "rm", image], check=False,
                           stdout=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
