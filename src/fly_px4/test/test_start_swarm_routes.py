#!/usr/bin/env python3
"""Check directory selection and startup wiring without launching ROS or aircraft."""

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PACKAGE / 'scripts'))
from check_patrol_maps import read_config


class StartSwarmRoutesTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='swarm-route-selection-')
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.package = root / 'src/fly_px4'
        scripts = self.package / 'scripts'
        scripts.mkdir(parents=True)
        for name in ('start_swarm.sh', 'start_positions.py'):
            shutil.copy2(PACKAGE / 'scripts' / name, scripts / name)
        scripts.joinpath('check_patrol_maps.py').write_text(
            'import json, os, sys\n'
            'with open(os.environ["SWARM_TEST_CHECK_LOG"], "w") as stream:\n'
            '    json.dump(sys.argv[1:], stream)\n')
        self.config = self.package / 'config'
        self.config.mkdir()
        maps = ('base0', 'base3', 'base4', 'base5', 'base6', 'base15', 'base16',
                'competition_base1', 'competition_base2', 'competition_base3')
        for directory in ('patrol_maps', 'patrol_maps_2'):
            target = self.config / directory
            target.mkdir()
            for name in maps:
                target.joinpath(name + '.yaml').write_text('patrol: {}\n')
        self.starts = self.config / 'start_positions.yaml'
        self.data = {'drone_%d' % i: {'x': -14. if i <= 3 else -17.,
                                    'y': float(((i - 1) % 3 - 1) * 3)}
                     for i in range(1, 7)}
        mock_bin = root / 'mock_bin'
        mock_bin.mkdir()
        launcher = mock_bin / 'roslaunch'
        launcher.write_text(
            '#!' + sys.executable + '\n'
            'import json, os, signal, sys, time\n'
            'signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n'
            'log = os.environ["SWARM_TEST_LOG"]\n'
            'with open(log, "a") as stream:\n'
            '    stream.write(json.dumps(sys.argv[1:]) + "\\n")\n'
            'if "swarm_fly_px4_1.launch" in sys.argv:\n'
            '    deadline = time.monotonic() + 3\n'
            '    while time.monotonic() < deadline:\n'
            '        with open(log) as stream:\n'
            '            if len(stream.readlines()) == 3:\n'
            '                sys.exit(0)\n'
            '        time.sleep(.01)\n'
            '    sys.exit(1)\n'
            'time.sleep(30)\n')
        launcher.chmod(0o755)
        setup = root / 'devel/setup.bash'
        setup.parent.mkdir()
        setup.write_text('export PATH="$SWARM_TEST_BIN:$PATH"\n')
        self.log = root / 'launches.jsonl'
        self.check_log = root / 'check.json'
        self.env = dict(os.environ, SWARM_TEST_BIN=str(mock_bin),
                        SWARM_TEST_LOG=str(self.log), SWARM_TEST_CHECK_LOG=str(self.check_log))

    def run_start(self, selected, *args):
        data = dict(self.data)
        if selected is not None:
            data['patrol_maps_dir'] = selected
        self.starts.write_text(yaml.safe_dump(data))
        self.log.unlink(missing_ok=True)
        self.check_log.unlink(missing_ok=True)
        result = subprocess.run(
            ['bash', str(self.package / 'scripts/start_swarm.sh'), *args],
            env=self.env, text=True, capture_output=True, timeout=10)
        calls = [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []
        return result, calls

    def test_original_commands_select_both_directories_and_keep_map_aliases(self):
        aliases = (('route1', 'base0'), ('route2', 'base3'), ('route3', 'base4'),
                   ('route4', 'base5'), ('route5', 'base6'), ('route6', 'base15'),
                   ('route7', 'base16'), ('base1', 'competition_base1'),
                   ('base2', 'competition_base2'), ('base3', 'competition_base3'))
        for selected in ('patrol_maps', 'patrol_maps_2'):
            for alias, map_name in aliases:
                with self.subTest(directory=selected, alias=alias):
                    result, calls = self.run_start(selected, alias)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                    self.assertEqual(len(calls), 3)
                    launches = {call[1]: call for call in calls}
                    controller = launches['swarm_fly_px4_1.launch']
                    self.assertIn('map:=' + map_name, controller)
                    route_arg = next(arg for arg in controller if arg.startswith('routes_file:='))
                    self.assertEqual(Path(route_arg.split(':=', 1)[1]).resolve(),
                                     self.config / selected / (map_name + '.yaml'))
                    for launch in ('swarm_relay_ego.launch', 'swarm_fly_px4_1.launch'):
                        start_arg = next(arg for arg in launches[launch]
                                         if arg.startswith('start_positions_file:='))
                        self.assertEqual(Path(start_arg.split(':=', 1)[1]).resolve(), self.starts)

    def test_missing_directory_setting_preserves_original_default(self):
        result, calls = self.run_start(None)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        controller = next(call for call in calls if call[1] == 'swarm_fly_px4_1.launch')
        self.assertIn('map:=base0', controller)
        self.assertTrue(any('/patrol_maps/base0.yaml' in arg for arg in controller))

    def test_invalid_directory_or_missing_route_stops_before_ros(self):
        for selected in ('../patrol_maps', 'patrol_maps_3'):
            result, calls = self.run_start(selected, 'base3')
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(calls, [])
        (self.config / 'patrol_maps_2/competition_base3.yaml').unlink()
        result, calls = self.run_start('patrol_maps_2', 'base3')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(calls, [])

    def test_check_only_receives_selected_routes_directory(self):
        for selected in ('patrol_maps', 'patrol_maps_2'):
            result, calls = self.run_start(selected, 'route2', '--check-only')
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(calls, [])
            args = json.loads(self.check_log.read_text())
            self.assertEqual(args[args.index('--map') + 1], 'base3')
            self.assertEqual(Path(args[args.index('--routes-dir') + 1]).resolve(),
                             self.config / selected)

    def test_checker_loads_explicit_directory_and_keeps_public_parameters(self):
        original = read_config('base0')
        override = yaml.safe_load((PACKAGE / 'config/patrol_maps/base0.yaml').read_text())
        changed_entry = [{'x': -10., 'y': 1.5}, {'x': -5., 'y': 1.5}]
        override['patrol']['routes']['drone_3']['entry_waypoints'] = changed_entry
        directory = self.config / 'patrol_maps_2'
        directory.joinpath('base0.yaml').write_text(yaml.safe_dump(override))
        selected = read_config('base0', directory)
        self.assertEqual(selected['routes']['drone_3']['entry_waypoints'], changed_entry)
        self.assertEqual(selected['defaults'], original['defaults'])
        self.assertEqual(selected['routes']['drone_3']['waypoints'],
                         original['routes']['drone_3']['waypoints'])


if __name__ == '__main__':
    unittest.main()
