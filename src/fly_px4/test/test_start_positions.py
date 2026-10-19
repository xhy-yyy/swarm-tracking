#!/usr/bin/env python3
"""Check six independent start edits and preserve all seven road presets."""

import copy
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest

import roslaunch.config
import roslaunch.xmlloader
import yaml

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / 'scripts'))
from start_positions import load_start_config, load_start_positions, validate_start_positions


def launch_config(filename, args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(PACKAGE / 'launch' / filename), config, argv=args, verbose=False)
    # Restarting after deleting a YAML entry must not reuse a stale ROS value.
    cleared = {name.rstrip('/') for name in config.clear_params}
    for node in config.nodes:
        if node.name.startswith(('fly_px4_1_', 'odom_relay_')):
            assert '/' + node.name in cleared, node.name
    return {key: param.value for key, param in config.params.items()}


class StartPositionTests(unittest.TestCase):
    def test_default_preserves_existing_starts(self):
        self.assertEqual(tuple(load_start_positions().values()),
                         ((-14., -3.), (-14., 0.), (-14., 3.),
                          (-17., -3.), (-17., 0.), (-17., 3.)))

    def test_optional_maps_directory_and_cli_keep_six_coordinates(self):
        positions = load_start_positions()
        original = {key: {'x': xy[0], 'y': xy[1]} for key, xy in positions.items()}
        with tempfile.TemporaryDirectory(prefix='start-maps-directory-') as tmp:
            path = Path(tmp) / 'starts.yaml'
            for selected in (None, 'patrol_maps', 'patrol_maps_2'):
                data = copy.deepcopy(original)
                if selected is not None:
                    data['patrol_maps_dir'] = selected
                path.write_text(yaml.safe_dump(data))
                with self.subTest(selected=selected):
                    self.assertEqual(load_start_positions(path), positions)
                    self.assertEqual(load_start_config(path)[1], selected or 'patrol_maps')
                    result = subprocess.run(
                        [sys.executable, str(PACKAGE / 'scripts/start_positions.py'),
                         str(path), '--print-maps-dir'], capture_output=True, text=True)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    self.assertEqual(result.stdout, (selected or 'patrol_maps') + '\n')

    def test_invalid_maps_directory_or_unknown_metadata_is_rejected(self):
        original = {key: {'x': xy[0], 'y': xy[1]}
                    for key, xy in load_start_positions().items()}
        for selected in ('../patrol_maps', '/tmp/patrol_maps', 'patrol_maps_3',
                         None, True, ['patrol_maps'], {'name': 'patrol_maps'}):
            data = copy.deepcopy(original)
            data['patrol_maps_dir'] = selected
            with self.subTest(selected=selected), self.assertRaises(ValueError):
                validate_start_positions(data)
        original['unrecognized_field'] = 'patrol_maps'
        with self.assertRaises(ValueError):
            validate_start_positions(original)

    def test_independent_edits_reach_both_launches_and_keep_road_waypoints(self):
        data = {'drone_%d' % i: {'x': 7.25 * i, 'y': -2.75 * i}
                for i in range(1, 7)}
        with tempfile.TemporaryDirectory(prefix='start-position-config-') as tmp:
            path = Path(tmp) / 'starts.yaml'
            path.write_text(yaml.safe_dump(data))
            override = ['start_positions_file:=' + str(path)]
            relay = launch_config('swarm_relay_ego.launch', override)
            for map_name in ('base0', 'base3', 'base4', 'base5', 'base6', 'base15', 'base16'):
                controller = launch_config('swarm_fly_px4_1.launch', override + ['map:=' + map_name])
                baseline = launch_config('swarm_fly_px4_1.launch', ['map:=' + map_name])
                roads = lambda values: {k: v for k, v in values.items() if '/patrol/' in k}
                self.assertEqual(roads(controller), roads(baseline), map_name)
                for i in range(1, 7):
                    selected = 'drone_%d' % i
                    for prefix, params in (('/fly_px4_1_%d/' % i, controller),
                                           ('/odom_relay_%d/' % i, relay)):
                        self.assertEqual(params[prefix + 'start_position_key'], selected)
                        for key, xy in data.items():
                            for axis in ('x', 'y'):
                                self.assertEqual(params[prefix + 'start_positions/' + key + '/' + axis],
                                                 xy[axis])

    def test_incomplete_or_invalid_coordinates_are_rejected(self):
        valid = {key: {'x': xy[0], 'y': xy[1]} for key, xy in load_start_positions().items()}
        bad = copy.deepcopy(valid)
        del bad['drone_6']
        with self.assertRaises(ValueError):
            validate_start_positions(bad)
        for value in (True, '14', None, float('nan'), float('inf')):
            bad = copy.deepcopy(valid)
            bad['drone_4']['x'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_start_positions(bad)
        for entry in ({'x': 0}, {'x': 0, 'y': 0, 'z': 0}):
            bad = copy.deepcopy(valid)
            bad['drone_2'] = entry
            with self.assertRaises(ValueError):
                validate_start_positions(bad)


if __name__ == '__main__':
    unittest.main()
