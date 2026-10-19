#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线校验巡逻航线，默认读取 config/patrol_maps/base0.yaml。
显式传入其他 YAML 时，按旧版 base.world 固定几何检查房区、灯杆、道路和高度，
并打印圈长、圈时及机间最小距离；其他地图使用 check_patrol_maps.py。"""

import math
import os
import sys

try:
    import yaml
except ImportError:
    sys.stderr.write("需要 pyyaml: pip3 install pyyaml\n")
    sys.exit(2)

# 地图静态先验
# 六片房区 (2 行 x 3 列), 由 3m 宽的人行道环围出来
BLOCKS = {
    'NW': (-41.3, -18.7,   3.7,  41.3),
    'NC': (-11.3,  41.3,   3.7,  41.3),
    'NE': ( 48.7, 106.3,   3.7,  41.3),
    'SW': (-41.3, -18.7, -41.3,  -3.7),
    'SC': (-11.3,  41.3, -41.3,  -3.7),
    'SE': ( 48.7, 106.3, -41.3,  -3.7),
}

# 马路中心线, 路宽 7.4m => 半宽 3.7
ROADS_NS = [-45.0, -15.0, 45.0, 110.0, 120.0]   # 南北向, 固定 x
ROADS_EW = [-45.0, 0.0, 45.0]                   # 东西向, 固定 y
ROAD_HALF = 3.7

# 10 根路灯, 9.3m 高、杆径 0.5m, 全在主街 y=±3.7 的路缘上
LAMPS = [
    (-35.1, -3.7), (-3.0, -3.7), (21.0, -3.7), (58.0, -3.7), (84.0, -3.7),
    (-25.2,  3.7), ( 9.0,  3.7), (27.0,  3.7), (71.0,  3.7), (90.5,  3.7),
]

# 判据余量
CLEAR_BLOCK = 1.5     # 距房区矩形
CLEAR_LAMP  = 1.5     # 距路灯杆中心
SAMPLE_STEP = 0.5     # 采样间距 m

# 与 swarm_fly_px4_1.launch 的默认值一致
MIN_Z, MAX_Z = 3.6, 4.4


def dist_to_rect(px, py, rect):
    """点到轴对齐矩形的距离; 点在矩形内返回负数(-到最近边的距离)"""
    x0, x1, y0, y1 = rect
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(px - x0, x1 - px, py - y0, y1 - py)
    return math.hypot(dx, dy)


def in_road(px, py):
    """是否落在任意一条马路走廊内"""
    for cx in ROADS_NS:
        if abs(px - cx) <= ROAD_HALF:
            return True
    for cy in ROADS_EW:
        if abs(py - cy) <= ROAD_HALF:
            return True
    return False


def sample_leg(a, b, step=SAMPLE_STEP):
    ax, ay = a
    bx, by = b
    n = max(1, int(math.hypot(bx - ax, by - ay) / step))
    for i in range(n + 1):
        t = float(i) / n
        yield ax + t * (bx - ax), ay + t * (by - ay)


def seg_seg_dist(p1, p2, p3, p4):
    """两条线段之间的最小距离(采样近似, 够用)"""
    best = 1e18
    for a in sample_leg(p1, p2, 1.0):
        for b in sample_leg(p3, p4, 1.0):
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            if d < best:
                best = d
    return best


def lap_time(perim, n_wp, cruise, accel):
    """粗估圈时: 直线跑满 cruise, 每个航点减速到 0 再加速 (ego 就是这个行为)"""
    # 每个航点一次 减速到0 + 加速回 cruise, 相比匀速多花的时间
    # 匀速走完 d 需 d/v; 走 d 且中间停一次需 2*(v/a) + (d - v^2/a)/v  (d 足够长时)
    extra_per_stop = cruise / accel          # 一次停+起相比匀速多花的时间
    return perim / cruise + n_wp * extra_per_stop


def main():
    # 默认及多地图校验统一读取 patrol_maps；[yaml路径] 历史用法保留。
    if len(sys.argv) == 1 or any(arg.startswith('--') for arg in sys.argv[1:]):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from check_patrol_maps import main as maps_main
        return maps_main()
    here = os.path.dirname(os.path.abspath(__file__))
    default_yaml = os.path.join(here, '..', 'config', 'patrol_routes.yaml')
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.normpath(default_yaml)

    if not os.path.isfile(path):
        sys.stderr.write("找不到航线文件: %s\n" % path)
        return 2

    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)

    try:
        patrol = cfg['patrol']
        defaults = patrol.get('defaults', {})
        routes = patrol['routes']
    except (KeyError, TypeError):
        sys.stderr.write("yaml 结构不对: 需要 patrol/{defaults,routes}\n")
        return 2

    print("=" * 74)
    print(" 巡逻航线校验: %s" % path)
    print("=" * 74)

    def_z = float(defaults.get('z', 5.0))
    def_cruise = float(defaults.get('cruise_speed', 12.0))
    def_accel = float(defaults.get('cruise_accel', 8.0))
    def_arrive = float(defaults.get('arrive_radius', 1.5))

    errors = []
    warns = []
    summary = []
    legs_by_drone = {}

    for key in sorted(routes.keys()):
        rt = routes[key] or {}
        wps = rt.get('waypoints') or []
        z = float(rt.get('z', def_z))
        cruise = float(rt.get('cruise_speed', def_cruise))
        accel = float(rt.get('cruise_accel', def_accel))
        loop = bool(rt.get('loop', defaults.get('loop', True)))
        delay = float(rt.get('start_delay', defaults.get('start_delay', 0.0)))

        if len(wps) < 2:
            errors.append("%s: 航点数 %d < 2, fly_px4_1 会改用内置兜底航线" % (key, len(wps)))
            continue

        pts = [(float(w['x']), float(w['y'])) for w in wps]

        # z 钳位
        if not (MIN_Z <= z <= MAX_Z):
            errors.append("%s: z=%.2f 不在钳位区间 [%.1f, %.1f] 内, 会被 fly_px4_1 改掉"
                          % (key, z, MIN_Z, MAX_Z))

        # 逐段采样
        legs = []
        n_legs = len(pts) if loop else len(pts) - 1
        for i in range(n_legs):
            legs.append((pts[i], pts[(i + 1) % len(pts)]))
        legs_by_drone[key] = legs

        perim = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in legs)

        worst_block = (1e18, None, None)
        worst_lamp = (1e18, None)
        off_road = []
        for a, b in legs:
            for (px, py) in sample_leg(a, b):
                for name, rect in BLOCKS.items():
                    d = dist_to_rect(px, py, rect)
                    if d < worst_block[0]:
                        worst_block = (d, name, (px, py))
                for (lx, ly) in LAMPS:
                    d = math.hypot(px - lx, py - ly)
                    if d < worst_lamp[0]:
                        worst_lamp = (d, (px, py))
                if not in_road(px, py):
                    off_road.append((px, py))

        if worst_block[0] < CLEAR_BLOCK:
            errors.append("%s: 距房区 %s 只有 %.2fm (要求 >=%.1f), 最近点 (%.1f, %.1f)"
                          % (key, worst_block[1], worst_block[0], CLEAR_BLOCK,
                             worst_block[2][0], worst_block[2][1]))
        if worst_lamp[0] < CLEAR_LAMP:
            errors.append("%s: 距路灯杆只有 %.2fm (要求 >=%.1f), 最近点 (%.1f, %.1f)"
                          % (key, worst_lamp[0], CLEAR_LAMP,
                             worst_lamp[1][0], worst_lamp[1][1]))
        if off_road:
            p = off_road[0]
            errors.append("%s: 有 %d 个采样点不在马路走廊内, 例如 (%.1f, %.1f)"
                          % (key, len(off_road), p[0], p[1]))

        # 到点半径把直角向内切掉多少: r*(1 - 1/sqrt2)
        corner_cut = def_arrive * (1.0 - 1.0 / math.sqrt(2.0))
        if worst_block[0] - corner_cut < CLEAR_BLOCK:
            warns.append("%s: 算上 arrive_radius=%.1f 的切角(%.2fm), 距房区余量降到 %.2fm"
                         % (key, def_arrive, corner_cut, worst_block[0] - corner_cut))

        t = lap_time(perim, len(pts), cruise, accel)
        summary.append((key, len(pts), z, perim, t, delay,
                        worst_block[0], worst_lamp[0], loop))

    # 机间: 两条航线之间的最小间距
    pair_min = []
    keys = sorted(legs_by_drone.keys())
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            best = 1e18
            for la in legs_by_drone[keys[i]]:
                for lb in legs_by_drone[keys[j]]:
                    d = seg_seg_dist(la[0], la[1], lb[0], lb[1])
                    if d < best:
                        best = d
            pair_min.append((best, keys[i], keys[j]))
    pair_min.sort()

    # 报告
    print("\n%-9s %4s %6s %9s %8s %7s %9s %8s" %
          ("航线", "点数", "z", "周长m", "圈时s", "延迟s", "距房区m", "距路灯m"))
    print("-" * 74)
    for (key, n, z, perim, t, delay, db, dl, loop) in summary:
        print("%-9s %4d %6.2f %9.1f %8.1f %7.1f %9.2f %8.2f%s" %
              (key, n, z, perim, t, delay, db, dl, "" if loop else "  (不循环)"))

    if summary:
        total = sum(s[3] for s in summary)
        worst_t = max(s[4] for s in summary)
        print("-" * 74)
        print("六机航线总长 %.0f m, 最慢一圈 %.0f s => 600s 内最少 %.0f 圈/机"
              % (total, worst_t, 600.0 / worst_t))

    print("\n两机航线最小间距 (前 5 对):")
    for (d, a, b) in pair_min[:5]:
        flag = ""
        if d < 2.0:
            flag = "  <-- 太近!"
        elif d < 3.0:
            flag = "  <-- 偏近(靠高度棋盘错开)"
        print("  %-9s <-> %-9s  %.2f m%s" % (a, b, d, flag))

    if warns:
        print("\n提醒:")
        for w in warns:
            print("  ! %s" % w)

    if errors:
        print("\n校验失败:")
        for e in errors:
            print("  x %s" % e)
        print("\n结论: 不合格, 请修 config/patrol_routes.yaml")
        return 1

    print("\n结论: 全部通过 (房区余量 >=%.1fm, 路灯余量 >=%.1fm, 全在马路走廊内)"
          % (CLEAR_BLOCK, CLEAR_LAMP))
    return 0


if __name__ == '__main__':
    sys.exit(main())
