#!/usr/bin/env python3
"""Focused static checks for the video startup contract."""

from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_media_router_is_ready_before_publishers_and_consumers():
    compose = yaml.safe_load((ROOT / "compose.yaml").read_text())
    router = compose["services"]["video-router"]
    assert router["healthcheck"]["test"] == ["CMD", "/mediamtx", "--version"]
    for service in ("sim", "offboard"):
        assert compose["services"][service]["depends_on"]["video-router"]["condition"] == "service_healthy"


def test_sim_entrypoint_has_bounded_direct_launch_readiness_gate():
    text = (ROOT / "modules/sim/entrypoint.sh").read_text()
    assert "wait_for_video_router" in text
    assert "VIDEO_ROUTER_WAIT_S" in text
    assert "video-router did not become ready" in text


def test_sim_layout_remains_the_single_simulation_layout_authority():
    text = (ROOT / "scripts/fleet.sh").read_text()
    assert "DEFAULT_LAYOUT=chimera_sim.json" in text
    assert text.count("chimera_sim.json") == 1


def test_scenegen_persists_fiducial_coordinate_and_placement_separately():
    editor = (ROOT / "modules/scenegen/editor.html").read_text()
    assert "fiducial_lat" in editor
    assert "fiducial_lon" in editor
    assert "placed_east_m" in editor
    assert "placed_north_m" in editor


def test_manifest_is_a_documented_read_only_front_door():
    text = (ROOT / "px4sim").read_text()
    assert "manifest)" in text
    assert "manifest            Record branches, commits, config, and image digests" in text
    assert "dirty_files" in text
