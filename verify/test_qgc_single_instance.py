#!/usr/bin/env python3
"""Static checks for the single-QGroundControl front-door contract."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_qgc_has_shutdown_grace_period():
    compose = (ROOT / "compose.yaml").read_text()
    block = compose[compose.index("  qgc:"):compose.index("  # -------------------------------------------------------------------------\n  # qgc-dev")]
    assert "stop_grace_period: 30s" in block


def test_qgc_entrypoint_serializes_instances():
    script = (ROOT / "modules/qgc/entrypoint.sh").read_text()
    assert 'flock -n 9' in script
    assert "px4sim-instance.lock" in script


def test_sim_gimbal_does_not_shadow_mavros_front_door():
    script = (ROOT / "scripts/sim_gimbal_attitude.py").read_text()
    assert "GimbalManagerConfigure" not in script
    assert "gimbal_control/manager/configure" not in script
    assert "GimbalDeviceAttitudeStatus" not in script
