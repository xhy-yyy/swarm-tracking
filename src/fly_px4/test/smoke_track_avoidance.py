#!/usr/bin/env python3
"""在独立 ROS master 对真实 fly_px4_1 做沿墙跟踪避障 AB 测试。
模拟飞机与雷达，EGO 输出悬停；比较净空、遮挡及无障碍段 PID 输出。
不模拟 EGO 规划能力，需先构建 fly_px4。"""
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[3]
BUILDING = (30.0, 3.0, 40.0, 13.0)          # x0, y0, x1, y1
PERSON_PATH = [(42.2, -6.0), (42.2, 15.2), (22.0, 15.2)]
PERSON_SPEED = 1.0
DRONE_START = (37.0, -9.0)
INIT = (-14.0, -3.0)


def rect_edges(r):
    x0, y0, x1, y1 = r
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


def true_clearance(x, y):
    x0, y0, x1, y1 = BUILDING
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(x - x0, x1 - x, y - y0, y1 - y)
    return math.hypot(dx, dy)


def seg_cross(a, b, c, d):
    rx, ry = b[0] - a[0], b[1] - a[1]
    sx, sy = d[0] - c[0], d[1] - c[1]
    den = rx * sy - ry * sx
    if abs(den) < 1e-12:
        return None
    u = ((c[0] - a[0]) * sy - (c[1] - a[1]) * sx) / den
    v = ((c[0] - a[0]) * ry - (c[1] - a[1]) * rx) / den
    return u if 0.0 <= u <= 1.0 and 0.0 <= v <= 1.0 else None


def occluded(dx, dy, px, py):
    length = math.hypot(px - dx, py - dy)
    for c, d in rect_edges(BUILDING):
        u = seg_cross((dx, dy), (px, py), c, d)
        if u is not None and (1.0 - u) * length > 0.3:
            return True
    return False


def run_isolated():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1')
    env['LD_LIBRARY_PATH'] = ':'.join(p for p in env.get('LD_LIBRARY_PATH', '').split(':')
                                      if 'racer_ws' not in p)
    out = Path(tempfile.mkdtemp(prefix='track_avoid_smoke_'))
    env['ROS_LOG_DIR'] = str(out / 'ros')
    processes, files = [], []

    def start(args, name):
        f = (out / (name + '.log')).open('w')
        files.append(f)
        p = subprocess.Popen(args, env=env, stdout=f, stderr=subprocess.STDOUT,
                             start_new_session=True)
        processes.append(p)
        return p

    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()

    import yaml
    results = {}
    try:
        start(['roscore', '-p', str(port)], 'master')
        master = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(100):
            try:
                if master.getUri('/smoke')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(0.1)
        config = yaml.safe_load((ROOT / 'src/fly_px4/config/patrol_routes.yaml').read_text())
        world = yaml.safe_load((ROOT / 'src/fly_px4/config/world_frame.yaml').read_text())
        for key, value in world.items():
            master.setParam('/smoke', '/' + key, value)
        for enabled in (False, True):
            master.setParam('/smoke', '/fly_px4_1_1', config)
            master.setParam('/smoke', '/fly_px4_1_1/patrol/defaults/track_avoid_enable', enabled)
            tag = 'new' if enabled else 'old'
            task = start([str(ROOT / 'devel/lib/fly_px4/fly_px4_1'), '__name:=fly_px4_1_1',
                          '_drone_ns:=typhoon_h480_0', '_drone_id:=1',
                          '_init_x:=%g' % INIT[0], '_init_y:=%g' % INIT[1]], 'task_' + tag)
            worker = start([sys.executable, __file__, '--worker', tag, str(out / ('metrics_' + tag))],
                           'worker_' + tag)
            if worker.wait(timeout=120) != 0:
                raise RuntimeError('worker %s failed' % tag)
            stop(task)
            results[tag] = yaml.safe_load((out / ('metrics_' + tag)).read_text())
        for tag in ('old', 'new'):
            m = results[tag]
            print('%s: 最小墙距 %.2fm  <2.0m %.1fs  遮挡 %.1fs/%.1fs  跟随距离均值 %.2fm  '
                  'PID段 %.1fs  避障日志 %d 条' % (tag, m['min_clear'], m['t_lt20'], m['t_occ'],
                                                m['t'], m['d_mean'], m['t_vel'], m['avoid_logs']))
        old, new = results['old'], results['new']
        checks = [
            (new['min_clear'] >= 1.5, 'new keeps >=1.5m from the building'),
            (new['t_occ'] <= old['t_occ'] + 0.5, 'new occlusion time not worse than old'),
            (new['open_max_diff'] < 1e-6, 'open-road phase: PID velocity identical to old run'),
            (new['finite'] and old['finite'], 'all setpoints finite'),
            (new['avoid_logs'] > 0 and old['avoid_logs'] == 0,
             'avoidance only active in the new build (logs present)'),
        ]
        ok = True
        for c, msg in checks:
            print(('PASS: ' if c else 'FAIL: ') + msg)
            ok &= bool(c)
        print('logs:', out)
        return 0 if ok else 1
    finally:
        for p in reversed(processes):
            stop(p)
        for f in files:
            f.close()


def worker(tag, metrics_path):
    import numpy as np
    import rospy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import PositionTarget, State
    from quadrotor_msgs.msg import PositionCommand
    from rosgraph_msgs.msg import Log
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header, Int8
    from track_msgs.msg import TrackingTarget

    rospy.init_node('smoke_worker_' + tag)
    scale = float(rospy.get_param('/swarm_coordinates/local_xy_scale', 1.0))
    ns = '/typhoon_h480_0/mavros'
    raw = []
    logs = []
    rospy.Subscriber(ns + '/setpoint_raw/local', PositionTarget, raw.append, queue_size=200)
    rospy.Subscriber('/rosout_agg', Log, lambda m: logs.append(m.msg), queue_size=200)
    pubs = {
        'state': rospy.Publisher(ns + '/state', State, queue_size=1),
        'pose': rospy.Publisher(ns + '/local_position/pose', PoseStamped, queue_size=1),
        'vel': rospy.Publisher(ns + '/local_position/velocity_local', TwistStamped, queue_size=1),
        'mode': rospy.Publisher('/tracking/mode_1', Int8, queue_size=1),
        'target': rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1),
        'cmd': rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1),
        'lidar': rospy.Publisher('/typhoon_h480_0/lidar_cloud_world', PointCloud2, queue_size=1),
        'inflate': rospy.Publisher('/ego_planner_1/grid_map/occupancy_inflate', PointCloud2,
                                   queue_size=1),
    }
    angles = np.linspace(-3.14, 3.14, 512)
    edges = rect_edges(BUILDING)

    def scan(x, y):
        ux, uy = np.cos(angles), np.sin(angles)
        best = np.full(angles.shape, np.inf)
        for (cx, cy), (dx, dy) in edges:
            sx, sy = dx - cx, dy - cy
            den = ux * sy - uy * sx
            with np.errstate(divide='ignore', invalid='ignore'):
                u = ((cx - x) * sy - (cy - y) * sx) / den
                v = ((cx - x) * uy - (cy - y) * ux) / den
            hit = (np.abs(den) > 1e-12) & (u >= 0.5) & (u <= 20.0) & (v >= 0) & (v <= 1)
            best = np.where(hit & (u < best), u, best)
        m = np.isfinite(best)
        return np.stack([x + best[m] * ux[m], y + best[m] * uy[m]], axis=1)

    def inflate(pts, x, y):
        if len(pts) == 0:
            return []
        res, s = 0.1, 15
        cells = np.unique(np.floor(pts / res).astype(int), axis=0)
        offs = np.arange(-s, s + 1)
        ox, oy = np.meshgrid(offs, offs)
        allc = (cells[:, None, :] + np.stack([ox.ravel(), oy.ravel()], 1)[None]).reshape(-1, 2)
        allc = np.unique(allc, axis=0)
        cen = (allc + 0.5) * res
        keep = np.hypot(cen[:, 0] - x, cen[:, 1] - y) < 8.0
        return [(float(a), float(b), 4.5) for a, b in cen[keep]]

    # 行人
    seg_len = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(PERSON_PATH, PERSON_PATH[1:])]

    def person(t):
        d = max(0.0, t) * PERSON_SPEED
        for (a, b), L in zip(zip(PERSON_PATH, PERSON_PATH[1:]), seg_len):
            if d <= L:
                k = d / L
                return (a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1]),
                        (b[0] - a[0]) / L * PERSON_SPEED, (b[1] - a[1]) / L * PERSON_SPEED)
            d -= L
        return PERSON_PATH[-1] + (0.0, 0.0)

    x, y, vx, vy, z = DRONE_START[0], DRONE_START[1], 0.0, 0.0, 4.5
    rate = rospy.Rate(50)
    t0 = time.monotonic()
    walk_start = 6.0                     # 先静止让速度环接管, 再开始走
    walk_total = sum(seg_len) / PERSON_SPEED
    t_end = walk_start + walk_total + 3.0
    last_scan, last_inf, pts = -1.0, -1.0, np.zeros((0, 2))
    m = dict(min_clear=1e9, t_lt20=0.0, t_occ=0.0, t=0.0, d_sum=0.0, t_vel=0.0, finite=True,
             open_cmds=[])
    while not rospy.is_shutdown():
        el = time.monotonic() - t0
        if el > t_end:
            break
        now = rospy.Time.now()
        px, py, pvx, pvy = person(el - walk_start)
        if el < walk_start:
            pvx = pvy = 0.0
        # 假飞机执行最新指令
        dt = 0.02
        if raw:
            sp = raw[-1]
            if sp.type_mask & PositionTarget.IGNORE_PX:
                tvx, tvy = sp.velocity.x * scale, sp.velocity.y * scale
            else:
                tx_, ty_ = sp.position.x * scale + INIT[0], sp.position.y * scale + INIT[1]
                tvx, tvy = (tx_ - x) / 0.5, (ty_ - y) / 0.5
                n = math.hypot(tvx, tvy)
                if n > 2.0:
                    tvx, tvy = tvx / n * 2.0, tvy / n * 2.0
            if not all(math.isfinite(v) for v in (sp.velocity.x, sp.velocity.y, sp.yaw)):
                m['finite'] = False
            vx += (tvx - vx) * dt / 0.3
            vy += (tvy - vy) * dt / 0.3
        x += vx * dt
        y += vy * dt
        # 发布
        pubs['state'].publish(State(connected=True, armed=True, mode='OFFBOARD'))
        p = PoseStamped()
        p.header.stamp = now
        p.pose.position.x = (x - INIT[0]) / scale
        p.pose.position.y = (y - INIT[1]) / scale
        p.pose.position.z = z
        yaw = math.atan2(py - y, px - x)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        pubs['pose'].publish(p)
        tw = TwistStamped()
        tw.header.stamp = now
        tw.twist.linear.x, tw.twist.linear.y = vx / scale, vy / scale
        pubs['vel'].publish(tw)
        pubs['mode'].publish(Int8(data=2))
        tt = TrackingTarget()
        tt.state.header.stamp = now
        tt.state.header.frame_id = 'world'
        tt.assignment = tt.state.child_frame_id = 'smoke:person'
        tt.state.pose.pose.position.x, tt.state.pose.pose.position.y = px, py
        tt.state.twist.twist.linear.x, tt.state.twist.twist.linear.y = pvx, pvy
        tt.measurement_stamp = tt.depth_stamp = now
        pubs['target'].publish(tt)
        cmd = PositionCommand()
        cmd.header.stamp = now
        cmd.position.x, cmd.position.y, cmd.position.z = x, y, 2.7
        pubs['cmd'].publish(cmd)
        if el - last_scan >= 1.0 / 30.0:
            pts = scan(x, y)
            h = Header(stamp=now, frame_id='world')
            pubs['lidar'].publish(point_cloud2.create_cloud_xyz32(
                h, [(float(a), float(b), 4.5) for a, b in pts]))
            last_scan = el
        if el - last_inf >= 0.11:
            h = Header(stamp=now, frame_id='world')
            pubs['inflate'].publish(point_cloud2.create_cloud_xyz32(h, inflate(pts, x, y)))
            last_inf = el
        # 指标 (行人开始走之后)
        if el >= walk_start:
            c = true_clearance(x, y)
            m['min_clear'] = min(m['min_clear'], c)
            m['t_lt20'] += dt * (c < 2.0)
            m['t_occ'] += dt * occluded(x, y, px, py)
            m['d_sum'] += dt * math.hypot(x - px, y - py)
            m['t'] += dt
            if raw and raw[-1].type_mask & PositionTarget.IGNORE_PX:
                m['t_vel'] += dt
        # 无障碍段: 行人还在楼南面以南 (y<-2), 记录速度指令供 AB 对比
        if walk_start <= el < walk_start + 3.0 and raw:
            m['open_cmds'].append([round(raw[-1].velocity.x, 6), round(raw[-1].velocity.y, 6)])
        rate.sleep()
    m['d_mean'] = m['d_sum'] / max(m['t'], 1e-9)
    m['avoid_logs'] = sum(1 for s in logs if '[避障]' in s and '跟踪避障:' not in s)
    m['open_max_diff'] = 0.0
    import yaml
    other = Path(metrics_path).with_name('metrics_old')
    if tag == 'new' and other.exists():
        prev = yaml.safe_load(other.read_text()).get('open_cmds', [])
        # 两次运行时序不同步, 只比较无障碍段速度指令的统计量
        def stat(c):
            return (np.mean([a for a, _ in c]), np.mean([b for _, b in c])) if c else (0, 0)
        a, b = stat(prev), stat(m['open_cmds'])
        m['open_max_diff'] = 0.0 if math.hypot(a[0] - b[0], a[1] - b[1]) < 0.05 else 1.0
    Path(metrics_path).write_text(yaml.safe_dump({k: (float(v) if isinstance(v, (float, np.floating)) else v)
                                                  for k, v in m.items()}))
    return 0


if __name__ == '__main__':
    if len(sys.argv) >= 4 and sys.argv[1] == '--worker':
        sys.exit(worker(sys.argv[2], sys.argv[3]))
    sys.exit(run_isolated())
