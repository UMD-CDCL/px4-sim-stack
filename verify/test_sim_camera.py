"""The renderer and ROS must use the same camera geometry."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sim_camera', ROOT / 'scripts/sim-camera.py')
camera = importlib.util.module_from_spec(spec)
spec.loader.exec_module(camera)


@pytest.mark.parametrize('number,model', [(11, 'v3'), (12, 'v3'), (13, 'v2'), (14, 'v2')])
def test_real_intrinsics_and_fallback_match_rendering(tmp_path, number, model):
    real = tmp_path / 'real'
    real.mkdir()
    preset = 'mid' if model == 'v3' else 'pilot'
    source = camera.pinhole(640, 360, 27.45, 'real')
    source['camera_matrix']['data'] = [1310., 0., 318.4, 0., 1312., 173.5, 0., 0., 1.]
    source['distortion_coefficients'] = dict(rows=1, cols=5, data=[-.14, .66, -.003, .0003, 0.])
    (real / f'uas{number-10}-{preset}-640x360.yaml').write_text(yaml.safe_dump(source))
    out = tmp_path / 'generated'
    camera.generate(ROOT / f'modules/sim/scenes/models/chimera_{model}/model.sdf',
                    number, real, out, 85.25, 31.03, 25.98,
                    'narrow=6.09 mid=20.13 wide=56.06')
    tree = ET.parse(out / 'model.sdf')
    sensor = 'gimbal_camera_2' if model == 'v3' else 'gimbal_camera'
    optic = tree.find(f'.//sensor[@name="{sensor}"]/camera')
    calibration = yaml.safe_load((out / ('gimbal-mid.yaml' if model == 'v3' else 'gimbal.yaml')).read_text())
    k = calibration['camera_matrix']['data']
    assert k == source['camera_matrix']['data']
    assert calibration['distortion_coefficients']['data'] == [0.] * 5
    assert float(optic.findtext('lens/intrinsics/fx')) == pytest.approx(3*k[0])
    assert float(optic.findtext('lens/intrinsics/fy')) == pytest.approx(3*k[4])
    assert float(optic.findtext('lens/intrinsics/cx')) == pytest.approx(3*k[2])
    assert float(optic.findtext('lens/intrinsics/cy')) == pytest.approx(3*k[5])
    assert float(optic.findtext('horizontal_fov')) == pytest.approx(2*math.atan(320/k[0]))
    fallback = yaml.safe_load((out / 'thermal.yaml').read_text())
    assert fallback['camera_matrix']['data'][0] == pytest.approx(320/math.tan(math.radians(31.03)/2))
    if model == 'v3':
        zoom = dict(entry.split('=') for entry in (out / 'zoom-presets').read_text().split())
        assert float(zoom['mid']) == pytest.approx(math.degrees(2*math.atan(320/k[0])))
