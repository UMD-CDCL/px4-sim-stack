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


def test_onboard_rtsp_preflight_uses_one_elapsed_deadline():
    text = (ROOT / "modules/onboard/entrypoint.sh").read_text()
    assert "stream_deadline=$((stream_started + STREAM_WAIT_S))" in text
    assert 'remaining=$((stream_deadline - now))' in text
    assert 'probe_timeout=$((remaining < 15 ? remaining : 15))' in text
    assert 'sleep_for=$((remaining < 5 ? remaining : 5))' in text
    assert 'timeout "${probe_timeout}" gst-launch-1.0' in text
    assert 'sleep "${sleep_for}"' in text


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
    assert "model_manifest_configured=" in text
    assert "model_manifest_source=" in text
    assert "sha256sum" in text


def test_canonical_layout_raw_roi_matches_gimbal_contract():
    layout = yaml.safe_load(
        (ROOT.parent / "ros2_ws/src/5g_drone/config/foxglove/chimera_sim.json").read_text()
    )
    panel = layout["configById"]["Publish!uas11_raw_roi"]
    assert panel["topicName"] == "/uas11/raw_roi_point_cmd"
    assert panel["datatype"] == "sensor_msgs/msg/NavSatFix"
    gimbal = (ROOT.parent / "ros2_ws/src/5g_drone/umd_uas/gimbal.py").read_text()
    assert '"gimbal.topic.raw_roi_point_cmd": "raw_roi_point_cmd"' in gimbal
    assert "NavSatFix, self._raw_roi_topic" in gimbal


def test_topic_front_door_retries_eventual_dds_discovery():
    text = (ROOT / "verify/component/uas.py").read_text()
    assert "Discovery is eventually consistent" in text
    assert "name in dict(uas.get_topic_names_and_types())" in text
    assert 'f"discover {name}"' in text
