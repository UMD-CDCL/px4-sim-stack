"""Static regression checks for the portable bench launch boundary."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_build_context_applies_bench_frame_patch():
    script = (ROOT / "scripts/build-contexts.sh").read_text()
    assert "patches/umd-uas-bench-sim-frame.patch" in script


def test_bench_patch_selects_simulated_mavinsight_frames():
    patch = (ROOT / "patches/umd-uas-bench-sim-frame.patch").read_text()
    assert "+                              'sim': 'true' if bench else 'false'," in patch


def test_verifier_keeps_home_frame_contract_strict():
    verifier = (ROOT / "verify/component/uas.py").read_text()
    assert 'origin = args.origin or f"uas{args.number}_home_position"' in verifier
    assert '"camera": f"d{args.number}_gimbal_frame"' in verifier
