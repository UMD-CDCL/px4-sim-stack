#!/usr/bin/env python3
"""Render the simulator console for the selected fleet, retaining its controls."""
import argparse
import copy
import json
import re

VEHICLE = re.compile(r'(?<![a-zA-Z0-9])(?:uas|d)(\d+)(?=\b|_)')
OBSOLETE = re.compile(r'^/(?:uas\d+/(?:static_targets|target_locations/topo|target_detections)|global_position/global|target_detections)$')


def renumber(value, vehicle):
    if isinstance(value, str):
        return re.sub(r'uas11(?=\b|_)', f'uas{vehicle}',
                      re.sub(r'd11_', f'uas{vehicle}_', value))
    if isinstance(value, list):
        return [renumber(v, vehicle) for v in value]
    if isinstance(value, dict):
        return {renumber(k, vehicle): renumber(v, vehicle) for k, v in value.items()}
    return value


def shared(value, vehicles):
    """Clone the primary vehicle's shared map settings for each active vehicle."""
    if isinstance(value, dict):
        result = {}
        for key, entry in value.items():
            if OBSOLETE.fullmatch(key):
                continue
            match = VEHICLE.search(key)
            if match:
                if int(match.group(1)) == 11:
                    for vehicle in vehicles:
                        result[renumber(key, vehicle)] = shared(renumber(entry, vehicle), vehicles)
                # Secondary template entries are replaced by the primary settings.
                continue
            result[key] = shared(entry, vehicles)
        return result
    if isinstance(value, list):
        result = []
        for entry in value:
            if isinstance(entry, str):
                if OBSOLETE.fullmatch(entry):
                    continue
                match = VEHICLE.search(entry)
                if match:
                    if int(match.group(1)) == 11:
                        result.extend(renumber(entry, v) for v in vehicles)
                    continue
            result.append(shared(entry, vehicles))
        return result
    if isinstance(value, str):
        return renumber(value, vehicles[0])
    return value


def render(template, vehicles):
    if not vehicles:
        raise ValueError('select at least one vehicle')
    result = copy.deepcopy(template)
    configurations = {}
    for panel, config in template['configById'].items():
        if re.search(r'__uas\d+$', panel):
            if panel.endswith('__uas11'):
                for vehicle in vehicles:
                    configurations[renumber(panel, vehicle)] = renumber(config, vehicle)
        elif panel.startswith('Tab!') and 'tabs' in config:
            tabs = config['tabs']
            if tabs and tabs[0].get('title') == 'uas11':
                config = copy.deepcopy(config)
                config['tabs'] = [renumber(tabs[0], v) for v in vehicles]
                config['activeTabIdx'] = 0
            configurations[panel] = config
        else:
            configurations[renumber(panel, vehicles[0])] = shared(config, vehicles)
    result['configById'] = configurations
    result['layout'] = renumber(template['layout'], vehicles[0])
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('template')
    parser.add_argument('vehicles', nargs='+', type=int)
    args = parser.parse_args()
    with open(args.template, encoding='utf-8') as source:
        print(json.dumps(render(json.load(source), args.vehicles), indent=2))
