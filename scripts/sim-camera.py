#!/usr/bin/env python3
"""Generate matching Gazebo cameras and ROS calibration from the real camera K."""
import argparse
import copy
import math
import os
from pathlib import Path
from string import Template
import xml.etree.ElementTree as ET

import yaml


def pinhole(width, height, hfov, name):
    f = width / (2 * math.tan(math.radians(hfov) / 2))
    return dict(image_width=width, image_height=height, camera_name=name,
                camera_matrix=dict(rows=3, cols=3, data=[f, 0., width/2, 0., f, height/2, 0., 0., 1.]))


def simulation_calibration(source):
    result = copy.deepcopy(source)
    # Harmonic's Ogre2 backend has no Brown distortion render pass.
    # Retain the measured K, with the same ideal optics in ROS and Gazebo.
    result['distortion_model'] = 'plumb_bob'
    result['distortion_coefficients'] = dict(rows=1, cols=5, data=[0.] * 5)
    result['rectification_matrix'] = dict(rows=3, cols=3, data=[1., 0., 0., 0., 1., 0., 0., 0., 1.])
    k = [float(v) for v in result['camera_matrix']['data']]
    if len(k) != 9 or k[0] <= 0 or k[4] <= 0 or not all(map(math.isfinite, k)):
        raise ValueError('Invalid camera matrix')
    result['camera_matrix']['data'] = k
    result['projection_matrix'] = dict(rows=3, cols=4, data=[*k[:3], 0., *k[3:6], 0., *k[6:], 0.])
    return result


def configure_camera(camera, calibration):
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    k = calibration['camera_matrix']['data']
    sx, sy = width / calibration['image_width'], height / calibration['image_height']
    hfov = 2 * math.atan(calibration['image_width'] / (2 * k[0]))
    camera.find('horizontal_fov').text = str(hfov)
    lens = ET.SubElement(camera, 'lens')
    ET.SubElement(lens, 'type').text = 'gnomonical'
    intrinsics = ET.SubElement(lens, 'intrinsics')
    for name, value in dict(fx=k[0]*sx, fy=k[4]*sy, cx=k[2]*sx, cy=k[5]*sy, s=k[1]*sx).items():
        ET.SubElement(intrinsics, name).text = str(value)
    return math.degrees(hfov)


def generate(model, uas, calibrations, output, gimbal, thermal, down, presets):
    number = uas - 10 if uas > 10 else uas
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    presets = dict(entry.split('=', 1) for entry in presets.split())
    names = sorted(presets, key=lambda name: float(presets[name]), reverse=True)
    template = Path(model).read_text()
    zoom_lens = ET.fromstring(template).find('model').get('name').endswith('_v3')
    env = dict(os.environ, UAS_NUM=str(uas), GIMBAL_HFOV_RAD=str(math.radians(gimbal)),
               THERMAL_HFOV_RAD=str(math.radians(thermal)), DOWN_HFOV_RAD=str(math.radians(down)))
    for i in range(3):
        key = 'GIMBAL_HFOV' + (f'_{i+1}' if i else '') + '_RAD'
        hfov = float(presets[names[min(i, len(names)-1)]]) if zoom_lens else gimbal
        env[key] = str(math.radians(hfov))
    root = ET.fromstring(Template(template).substitute(env))
    zoom = {}
    for sensor in root.findall('.//sensor[@type="camera"]'):
        name = sensor.attrib['name']
        camera = sensor.find('camera')
        if name.startswith('gimbal_camera'):
            if zoom_lens:
                index = int(name.rsplit('_', 1)[1]) - 1 if name[-1].isdigit() else 0
                preset = names[min(index, len(names)-1)]
                real = f'uas{number}-{preset}-640x360.yaml'
                dest = f'gimbal-{preset}.yaml'
            else:
                preset = None
                real, dest = f'uas{number}-pilot-640x360.yaml', 'gimbal.yaml'
            width, height = 640, 360
        elif name == 'thermal_camera':
            preset = None
            real, dest = f'uas{number}-thermal-640x512.yaml', 'thermal.yaml'
            width, height = 640, 512
        else:
            preset = None
            real, dest = f'uas{number}-down-640x360.yaml', 'down.yaml'
            width, height = 640, 360
        path = Path(calibrations) / real
        source = yaml.safe_load(path.read_text()) if path.exists() else pinhole(
            width, height, math.degrees(float(camera.findtext('horizontal_fov'))), name)
        calibration = simulation_calibration(source)
        hfov = configure_camera(camera, calibration)
        (output / dest).write_text(yaml.safe_dump(calibration, sort_keys=False))
        if preset:
            zoom[preset] = hfov
        print(f'camera: uas{number} {name}: {path if path.exists() else "sim fallback"}, zero distortion', flush=True)
    ET.indent(root)
    ET.ElementTree(root).write(output / 'model.sdf', encoding='unicode', xml_declaration=True)
    (output / 'zoom-presets').write_text(' '.join(f'{n}={zoom[n]:.12g}' for n in names if n in zoom))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--uas', type=int, required=True)
    parser.add_argument('--calibrations', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--gimbal', type=float, default=56.06)
    parser.add_argument('--thermal', type=float, default=29.75)
    parser.add_argument('--down', type=float, default=25.98)
    parser.add_argument('--presets', default='narrow=6.09 mid=20.13 wide=56.06')
    generate(**vars(parser.parse_args()))
