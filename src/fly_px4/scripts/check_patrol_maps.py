#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline validation of patrol loops and one-time approaches using reference worlds.
Not run by the race launcher. Example: --all --plot-dir docs/patrol_maps."""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

# Catkin devel wrappers are executable shims, not importable helper modules.
# __file__ inside the wrapper points to this source file; import its neighbours.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import yaml
from shapely.geometry import LineString, Point
from shapely.ops import unary_union

from patrol_world_geometry import DEFAULT_WORLD_DIR, MAPS, SPAWNS as ORIGINAL_BASE0_STARTS, load_world
from start_positions import DEFAULT_START_POSITIONS, load_start_config, load_start_positions

PACKAGE = Path(__file__).resolve().parent.parent
DEFAULT_ROUTES_DIR = PACKAGE / 'config/patrol_maps'
CLEARANCE = 1.5   # flight-centre distance to conservative collision footprint
MIN_Z, MAX_Z = 4.1, 4.9
PATROL_Z = 4.2
PATROL_HEIGHT_BAND = (3.85, 5.15)
ENTRY_HEIGHT_BAND = (1.35, 5.15)


def merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = copy.deepcopy(value)
    return base


def read_config(map_name, routes_dir=None):
    with (PACKAGE / 'config/patrol_routes.yaml').open() as stream:
        config = yaml.safe_load(stream)
    with (Path(routes_dir or DEFAULT_ROUTES_DIR) / (map_name + '.yaml')).open() as stream:
        override = yaml.safe_load(stream)
    if override['patrol']['map'] != map_name:
        raise ValueError('航线文件地图标识不匹配: ' + map_name)
    if set(override['patrol']['routes']) != {'drone_%d' % i for i in range(1, 7)}:
        raise ValueError('地图必须明确配置六机航线: ' + map_name)
    merge(config, override)
    return config['patrol']


def world_signature(path):
    """Static definition + saved state; ignore injected vehicles and actor runtime state."""
    world = ET.parse(path).getroot().find('world')
    if world is None:
        raise ValueError('缺少 <world>: ' + str(path))

    def canonical(e):
        return [e.tag, sorted(e.attrib.items()), ' '.join((e.text or '').split()),
                [canonical(c) for c in e]]

    models = {m.get('name'): m for m in world.findall('model')
              if not m.get('name').startswith('typhoon_h480')}
    data = [canonical(r) for r in world.findall('road')]
    for name in sorted(models):
        data.append(canonical(models[name]))
    for m in world.findall('state/model'):
        if m.get('name') in models:
            data.append(canonical(m))
    return hashlib.sha256(json.dumps(data, ensure_ascii=True).encode()).hexdigest()


def check_spawn_poses(path, spawns=None):
    if spawns is None:
        spawns = tuple(load_start_positions().values())
    world = ET.parse(path).getroot().find('world')
    for i, expected in enumerate(spawns):
        model = world.find("model[@name='typhoon_h480_%d']" % i)
        if model is None:
            raise ValueError('内联 world 缺少飞机 typhoon_h480_%d' % i)
        xy = tuple(map(float, model.findtext('pose').split()[:2]))
        if math.dist(xy, expected) > .001:
            raise ValueError('typhoon_h480_%d 起点 %s 与 YAML 起点 %s 不一致' % (i, xy, expected))


def points(items):
    result = []
    for wp in items:
        xy = tuple(float(wp[key]) for key in ('x', 'y'))
        if not all(math.isfinite(v) for v in xy):
            raise ValueError('航点坐标必须是有限数值')
        result.append(xy)
    return result


def corner_chords(pts, radius):
    # Early waypoint switching can cut across the inside of a right-angle turn.
    # Inspect the chord joining points radius metres before/after each corner.
    for i, corner in enumerate(pts):
        before, after = pts[i-1], pts[(i+1) % len(pts)]
        def approach(neighbor):
            d = math.dist(corner, neighbor)
            t = min(radius / d, .5) if d else 0
            return tuple(corner[j] + t*(neighbor[j]-corner[j]) for j in (0, 1))
        yield LineString([approach(before), approach(after)])


def nearest(geometry, obstacles):
    return min((geometry.distance(poly), name) for name, poly in obstacles)


def validate(map_name, world_path, compact=False, spawns=None, routes_dir=None):
    if spawns is None:
        spawns = tuple(load_start_positions().values())
    patrol = read_config(map_name, routes_dir)
    defaults, routes = patrol['defaults'], patrol['routes']
    roads, road_area, obstacles = load_world(world_path, *PATROL_HEIGHT_BAND)
    _, _, entry_obstacles = load_world(world_path, *ENTRY_HEIGHT_BAND)
    errors, warnings, rows, loops, approaches, initial_goals = [], [], [], {}, {}, {}
    if set(routes) != {'drone_%d' % i for i in range(1, 7)}:
        raise ValueError('需要 drone_1..drone_6')

    for i in range(1, 7):
        name = 'drone_%d' % i
        route = routes[name]
        settings = dict(defaults, **{k: v for k, v in route.items()
                                    if k not in ('waypoints', 'entry_waypoints')})
        pts = points(route['waypoints'])
        entries = points(route.get('entry_waypoints', []))
        if len(pts) < 2:
            raise ValueError(name + ': 巡航至少需要两个点')
        initial_goals[name] = Point((entries or pts)[0])
        if not settings.get('loop', True):
            errors.append(name + ': 比赛巡航需要 loop=true')
        for wp in route['waypoints'] + route.get('entry_waypoints', []):
            z = float(wp.get('z', settings['z']))
            if not math.isfinite(z) or not MIN_Z <= z <= MAX_Z:
                errors.append(name + ': 航点高度不在 [4.1,4.9]')
        if float(settings['z']) != PATROL_Z:
            errors.append(name + ': 巡航目标高度必须为 %.1fm（飞控本地 ENU）' % PATROL_Z)
        track_z = float(settings['track_z'])
        if not math.isfinite(track_z) or track_z <= 0:
            errors.append(name + ': track_z 必须为有限正数，以 YAML 配置为准')
        speed, accel = float(settings['cruise_speed']), float(settings['cruise_accel'])
        delay = float(settings.get('start_delay', 0))
        if not all(math.isfinite(v) and v >= 0 for v in (speed, accel, delay)) or min(speed, accel) == 0:
            errors.append(name + ': 速度/加速度/延迟不合法')
        radius = float(settings.get('arrive_radius', 1.5))
        loop = LineString(pts + pts[:1])
        corners = unary_union(list(corner_chords(pts, radius)))
        approach = LineString([spawns[i-1]] + entries + pts[:1])
        loops[name], approaches[name] = loop, approach
        for label, geometry in (('闭环', loop), ('到点切角', corners), ('进场', approach)):
            if not road_area.buffer(.001).covers(geometry):
                errors.append(name + ': ' + label + '离开实际道路范围')
        clear, obstacle = nearest(unary_union([loop, corners]), obstacles)
        entry_clear, entry_obstacle = nearest(approach, entry_obstacles)
        spawn_clear, spawn_obstacle = nearest(Point(spawns[i-1]), entry_obstacles)
        if clear + 1e-6 < CLEARANCE:
            errors.append('%s: 闭环距 %s 只有 %.2fm' % (name, obstacle, clear))
        if entry_clear + 1e-6 < CLEARANCE:
            text = '%s: 进场距 %s 只有 %.2fm' % (name, entry_obstacle, entry_clear)
            # The exception covers only the previously flown base0 approach.
            # A changed start creates a new first leg and must satisfy clearance.
            original_approach = map_name == 'base0' and spawns[i-1] == ORIGINAL_BASE0_STARTS[i-1]
            (warnings if original_approach else errors).append(text)
        if spawn_clear + 1e-6 < CLEARANCE:
            errors.append('%s: 起点距 %s 只有 %.2fm' % (name, spawn_obstacle, spawn_clear))
        # Both climb modes are allowed. The approach, including its first straight
        # leg under PX4 control, is checked against the low-altitude obstacles above.
        perim = loop.length
        rows.append(dict(drone=name, start_position=spawns[i-1], waypoints=pts, entry_waypoints=entries,
                         climb_in_place=bool(settings.get('climb_in_place', False)),
                         start_delay=delay, perimeter=round(perim, 2),
                         lap_time_estimate=round(perim/speed+len(pts)*speed/accel, 2),
                         patrol_clearance=round(clear, 3), nearest_obstacle=obstacle,
                         entry_clearance=round(entry_clear, 3)))
    pairs, initial_pairs = [], []
    for a in range(1, 7):
        for b in range(a+1, 7):
            ka, kb = 'drone_%d' % a, 'drone_%d' % b
            distance = loops[ka].distance(loops[kb])
            pairs.append((distance, ka, kb))
            initial_distance = initial_goals[ka].distance(initial_goals[kb])
            initial_pairs.append(initial_distance)
            if initial_distance < 2.99:
                errors.append('%s/%s: 起飞首目标间距只有 %.2fm' % (ka, kb, initial_distance))
            if distance < 2.99:
                errors.append('%s/%s: 闭环间距只有 %.2fm' % (ka, kb, distance))
    if not compact:
        print('\n%s: world=%s' % (map_name, world_path))
        print('机号   周长(m)  圈时估计(s)  闭环净空(m)  进场净空(m)  延迟(s)')
        for row in rows:
            print('%s  %7.1f  %11.1f  %11.2f  %11.2f  %7.1f' %
                  (row['drone'], row['perimeter'], row['lap_time_estimate'],
                   row['patrol_clearance'], row['entry_clearance'], row['start_delay']))
    for warning in warnings:
        print('  提醒（原 base0 航线保留）: ' + warning)
    for error in errors:
        print('  失败: ' + error)
    print('%s: %s；闭环净空最小 %.2fm；闭环间距最小 %.2fm' %
          (map_name, '失败' if errors else '通过', min(r['patrol_clearance'] for r in rows),
           min(p[0] for p in pairs)))
    result = dict(map=map_name, world=str(world_path), passed=not errors,
                  flight_height_band=list(PATROL_HEIGHT_BAND), entry_height_band=list(ENTRY_HEIGHT_BAND),
                  clearance_required=CLEARANCE, min_loop_separation=round(min(p[0] for p in pairs), 3),
                  min_initial_goal_separation=round(min(initial_pairs), 3),
                  drones=rows, errors=errors, warnings=warnings)
    return result, (roads, obstacles, loops, approaches)


def draw(ax, name, data):
    from matplotlib.patches import Polygon
    roads, obstacles, loops, approaches = data
    for _, line, half in roads:
        poly = line.buffer(half)
        ax.add_patch(Polygon(list(poly.exterior.coords), facecolor='#e8edf1', edgecolor='none'))
    for _, poly in obstacles:
        if poly.geom_type == 'Polygon':
            ax.add_patch(Polygon(list(poly.exterior.coords), facecolor='#646c76', alpha=.55))
    for name, poly in obstacles:
        if name.startswith('lamp_post'):
            ax.scatter(poly.centroid.x, poly.centroid.y, color='#333b44', s=7, zorder=3)
    colors = ('#e45756', '#4c78a8', '#f2a541', '#8d5ab5', '#249c83', '#bd6590')
    for (key, loop), color in zip(loops.items(), colors):
        spawn = approaches[key].coords[0]
        xs, ys = loop.xy
        ax.plot(xs, ys, color=color, linewidth=1.5, marker='.', label=key.replace('drone_', '#'))
        x, y = approaches[key].xy
        ax.plot(x, y, color=color, linestyle=':', linewidth=1.1, alpha=.85)
        ax.scatter(*spawn, color=color, s=12, marker='x')
        a, b = list(loop.coords)[:2]
        ax.annotate('', xy=(a[0]+.62*(b[0]-a[0]), a[1]+.62*(b[1]-a[1])),
                    xytext=(a[0]+.43*(b[0]-a[0]), a[1]+.43*(b[1]-a[1])),
                    arrowprops=dict(arrowstyle='->', color=color, lw=1.5))
    ax.set(title=name + ' | local ENU z=%.1fm' % PATROL_Z, xlim=(-50, 125), ylim=(-50, 50), xlabel='world x (m)', ylabel='world y (m)')
    ax.set_aspect('equal'); ax.grid(alpha=.18)
    ax.legend(loc='upper left', ncol=3, fontsize=7)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    group = ap.add_mutually_exclusive_group()
    group.add_argument('--map', choices=MAPS, default='base0')
    group.add_argument('--all', action='store_true')
    ap.add_argument('--world-dir', type=Path, default=DEFAULT_WORLD_DIR)
    ap.add_argument('--world', type=Path, help='核对已启动的内联 world 与所选地图一致')
    ap.add_argument('--plot-dir', type=Path)
    ap.add_argument('--report', type=Path)
    ap.add_argument('--start-positions', type=Path, default=DEFAULT_START_POSITIONS,
                    help='巡航起点 YAML；用于进场、起点净空及图中的起点检查')
    ap.add_argument('--routes-dir', type=Path,
                    help='航线 YAML 目录；缺省读取起点配置的 patrol_maps_dir')
    ap.add_argument('--compact', action='store_true')
    args = ap.parse_args(argv)
    if args.all and args.world:
        ap.error('--world 只能与单个 --map 一起用')
    maps = MAPS if args.all else (args.map,)
    reports, plots = [], []
    try:
        positions, maps_dir = load_start_config(args.start_positions)
        spawns = tuple(positions.values())
        routes_dir = args.routes_dir or PACKAGE / 'config' / maps_dir
        for name in maps:
            source = args.world_dir / (name + '.world')
            if args.world and world_signature(args.world) != world_signature(source):
                raise ValueError('离线 world 与预设 %s 的参考场景不一致: %s'
                                 % (name, source))
            if args.world:
                check_spawn_poses(args.world, spawns)
            report, data = validate(name, args.world or source, args.compact, spawns, routes_dir)
            reports.append(report); plots.append((name, data))
        if args.plot_dir:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            args.plot_dir.mkdir(parents=True, exist_ok=True)
            for name, data in plots:
                fig, ax = plt.subplots(figsize=(12, 7)); draw(ax, name, data)
                fig.tight_layout(); fig.savefig(args.plot_dir / (name + '.png'), dpi=150); plt.close(fig)
            if len(plots) > 1:
                fig, axes = plt.subplots(4, 2, figsize=(16, 19))
                for ax, (name, data) in zip(axes.flat, plots): draw(ax, name, data)
                for ax in list(axes.flat)[len(plots):]: ax.axis('off')
                fig.suptitle('Six-drone patrols | solid: loop | dotted: one-time approach | x: fixed spawn')
                fig.tight_layout(rect=(0, 0, 1, .975))
                fig.savefig(args.plot_dir / 'overview.png', dpi=130); plt.close(fig)
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(reports, ensure_ascii=False, indent=2)+'\n')
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print('航线校验失败: ' + str(exc), file=sys.stderr)
        return 2
    return 1 if any(not r['passed'] for r in reports) else 0


if __name__ == '__main__':
    sys.exit(main())
