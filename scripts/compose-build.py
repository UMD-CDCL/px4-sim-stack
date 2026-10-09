#!/usr/bin/env python3
"""Build Compose's selected targets with explicit Bake permissions."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys


def bake_command(config: dict) -> list[str]:
    command = ["docker", "buildx", "bake", "--file", "-"]
    paths = set()
    host_network = False
    for target in config.get("target", {}).values():
        host_network |= target.get("network") == "host" or "network.host" in target.get("entitlements", [])
        for context in [target.get("context", "."), *target.get("contexts", {}).values()]:
            # Service/image/URL contexts are resolved by Bake, not read from disk.
            if context.startswith(("target:", "service:", "docker-image:", "git@")) or "://" in context:
                continue
            paths.add(str(Path(context).resolve()))
    if host_network:
        command.append("--allow=network.host")
    # Match Compose's grants for its declared local contexts, without granting
    # filesystem writes or unrestricted filesystem access.
    command.extend(f"--allow=fs.read={path}" for path in sorted(paths))
    return command


def main(args: list[str]) -> int:
    compose = ["docker", "compose", "build"]
    help_result = subprocess.run([*compose, "--help"], capture_output=True, text=True)
    if help_result.returncode:
        sys.stderr.write(help_result.stderr)
        return help_result.returncode
    # Older Compose releases build directly and do not expose a Bake plan.
    if "--print" not in help_result.stdout:
        os.execvp("docker", [*compose, *args])
    plan = subprocess.run([*compose, "--print", *args], capture_output=True, text=True)
    sys.stderr.write(plan.stderr)
    if plan.returncode:
        return plan.returncode
    try:
        config = json.loads(plan.stdout)
    except ValueError as exc:
        print(f"Cannot read Compose build plan: {exc}", file=sys.stderr)
        return 1
    if not config.get("group", {}).get("default", {}).get("targets"):
        return 0
    # Compose's plan retains profiles, target selection, build args, service
    # contexts, tags and Docker image output. BUILDX_BUILDER is inherited.
    return subprocess.run(bake_command(config), input=plan.stdout, text=True).returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
