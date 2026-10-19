#!/usr/bin/env python3
"""Read and validate the six manually supplied horizontal start positions."""

import argparse
import math
from pathlib import Path
import sys

import yaml

DEFAULT_START_POSITIONS = Path(__file__).resolve().parent.parent / 'config/start_positions.yaml'
PATROL_MAPS_DIRS = ('patrol_maps', 'patrol_maps_2')


def validate_patrol_maps_dir(data):
    selected = data.get('patrol_maps_dir', 'patrol_maps')
    if selected not in PATROL_MAPS_DIRS:
        raise ValueError('patrol_maps_dir 只能是 patrol_maps 或 patrol_maps_2')
    return selected


def validate_start_positions(data):
    expected = {'drone_%d' % i for i in range(1, 7)}
    if not isinstance(data, dict) or set(data) - {'patrol_maps_dir'} != expected:
        raise ValueError('起点配置必须完整包含 drone_1..drone_6')
    validate_patrol_maps_dir(data)
    positions = {}
    for i in range(1, 7):
        key = 'drone_%d' % i
        entry = data[key]
        if not isinstance(entry, dict) or set(entry) != {'x', 'y'}:
            raise ValueError(key + ': 需要且只接受 x/y 两个坐标')
        xy = []
        for axis in ('x', 'y'):
            value = entry[axis]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('%s.%s 必须是有限数值' % (key, axis))
            xy.append(float(value))
        positions[key] = tuple(xy)
    return positions


def load_start_config(path=DEFAULT_START_POSITIONS):
    with Path(path).open() as stream:
        data = yaml.safe_load(stream)
    return validate_start_positions(data), validate_patrol_maps_dir(data)


def load_start_positions(path=DEFAULT_START_POSITIONS):
    return load_start_config(path)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', nargs='?', type=Path, default=DEFAULT_START_POSITIONS)
    parser.add_argument('--print-maps-dir', action='store_true',
                        help='检查完整起点配置，只输出选择的航线目录')
    args = parser.parse_args()
    try:
        positions, maps_dir = load_start_config(args.file)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print('起点配置错误: ' + str(exc), file=sys.stderr)
        return 2
    if args.print_maps_dir:
        print(maps_dir)
        return 0
    print('六机起点配置: ' + str(args.file.resolve()))
    print('巡航航线目录: ' + maps_dir)
    for key, xy in positions.items():
        print('  %s: x=%g, y=%g' % (key, *xy))
    return 0


if __name__ == '__main__':
    sys.exit(main())
