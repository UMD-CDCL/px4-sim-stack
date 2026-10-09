"""The simulator console follows fleet selection without losing controls."""
import importlib.util
import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sim_layout', ROOT / 'scripts/render-sim-layout.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
TEMPLATE = ROOT / '../ros_workspaces/cdcl_ws/src/5g_drone/config/foxglove/chimera_sim.json'


@pytest.mark.parametrize('vehicles', [[11], [11, 13], [12, 15]])
def test_console_contains_only_selected_vehicles_and_preserves_controls(vehicles):
    template = json.loads(TEMPLATE.read_text())
    result = module.render(template, vehicles)
    configs = result['configById']
    tabs = next(c['tabs'] for key, c in configs.items() if key.startswith('Tab!'))
    assert [tab['title'] for tab in tabs] == [f'uas{v}' for v in vehicles]
    for v in vehicles:
        assert any(c.get('topicName') == f'/uas{v}/vlm_capture_cmd' for c in configs.values())
        assert any(c.get('serviceName') == f'/uas{v}/gimbal/click_mode/roi' for c in configs.values())
    scene = next(c for key, c in configs.items() if key.startswith('3D!'))
    assert all(f'/viz/uas{v}/footprint' in scene['topics'] for v in vehicles)
    text = json.dumps(result)
    assert '"/global_position/global"' not in text
    assert '/target_locations/topo' not in text
    assert '/static_targets' not in text
    for number in range(11, 20):
        if number not in vehicles:
            assert f'/uas{number}/' not in text
            assert f'/viz/uas{number}/' not in text
