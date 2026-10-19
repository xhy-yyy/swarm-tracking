#!/usr/bin/env python3
"""在独立 ROS master 对真实 fly_px4_1 做顶棚场景 AB 测试。
模拟飞机、立柱雷达和檐口高度带；EGO 固定输出悬停，不模拟规划能力。
验证机体不进入顶棚投影、视线不被高度带误挡，以及跟踪高度带请求。"""
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
CANOPY = (29.95, 0.0, 36.65, 18.0)          # x0, y0, x1, y1
PILLARS = [(31.95, 6.0), (34.65, 6.0), (31.95, 12.0), (34.65, 12.0)]
PILLAR_R = 0.3
BODY_R = 0.371
PERSON_PATH = [(33.3, -9.0), (33.3, 9.0), (33.3, 9.0), (33.3, 24.0)]
PERSON_SPEED = 1.0
PAUSE_AT = 1                                 # 在第 2 个点停 8s (人站在棚下中部)
PAUSE_SEC = 8.0
DRONE_START = (33.3, -15.0)
INIT = (-14.0, -3.0)


def inside_canopy(x, y, margin=0.0):
    x0, y0, x1, y1 = CANOPY
    return x0 - margin < x < x1 + margin and y0 - margin < y < y1 + margin


def canopy_gap(x, y):
    """机体中心到顶棚投影的距离 (在投影内为负)。"""
    x0, y0, x1, y1 = CANOPY
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    if dx == 0.0 and dy == 0.0:
        return -min(x - x0, x1 - x, y - y0, y1 - y)
    return math.hypot(dx, dy)


def eave_points():
    x0, y0, x1, y1 = CANOPY
    pts, s = [], 0.0
    while s <= 1.0 + 1e-9:
        pts += [(x0 + s * (x1 - x0), y0), (x0 + s * (x1 - x0), y1),
                (x0, y0 + s * (y1 - y0)), (x1, y0 + s * (y1 - y0))]
        s += 0.2 / (y1 - y0)
    return pts


def run_isolated():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        port = s.getsockname()[1]
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_HOSTNAME='127.0.0.1', ROS_IP='127.0.0.1')
    env['LD_LIBRARY_PATH'] = ':'.join(p for p in env.get('LD_LIBRARY_PATH', '').split(':')
                                      if 'racer_ws' not in p)
    out = Path(tempfile.mkdtemp(prefix='canopy_band_smoke_'))
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
        track_z = float(config['patrol']['defaults']['track_z'])
        for tag in ('old', 'new'):
            master.setParam('/smoke', '/fly_px4_1_1', config)
            task = start([str(ROOT / 'devel/lib/fly_px4/fly_px4_1'), '__name:=fly_px4_1_1',
                          '_drone_ns:=typhoon_h480_0', '_drone_id:=1',
                          '_init_x:=%g' % INIT[0], '_init_y:=%g' % INIT[1]], 'task_' + tag)
            worker = start([sys.executable, __file__, '--worker', tag, str(out / ('metrics_' + tag))],
                           'worker_' + tag)
            if worker.wait(timeout=150) != 0:
                raise RuntimeError('worker %s failed' % tag)
            stop(task)
            results[tag] = yaml.safe_load((out / ('metrics_' + tag)).read_text())
        for tag in ('old', 'new'):
            m = results[tag]
            print('%s: 进入棚下 %.1fs  离棚最近 %.2fm  视线被判遮挡 %.1fs  人距均值 %.2fm  '
                  '人在棚下时人距均值 %.2fm  EGO目标在棚下 %d 拍  人在棚下时目标离人最远 %.1fm  '
                  '人停在棚中部时目标离人最远 %.1fm  目标离棚边最近 %.2fm  请求 active=%s 目标高度=%.2f' % (
                      tag, m['t_inside'], m['min_gap'], m['t_occ'], m['d_mean'], m['d_under'],
                      m['goal_inside'], m['goal_under_dmax'], m['goal_pause_dmax'],
                      m['goal_min_gap'], m['req_active'], m['req_target']))
            for e in m['occ_events']:
                if e[0] == 'goal':
                    print('   目标样本 t=%.2f 机(%.2f,%.2f) 人(%.2f,%.2f) 目标(%.2f,%.2f) 候选 %d 半径 %.2f 离墙 %.2f 降级 %d' % tuple(e[1:]))
                else:
                    print('   遮挡样本 t=%.2f 机(%.2f,%.2f) 人(%.2f,%.2f) 视线余量 %.2f 候选 %d 半径 %.2f' % tuple(e))
        old, new = results['old'], results['new']
        checks = [
            (old['t_inside'] > 1.0, 'old (lidar only): drone enters under the canopy'),
            (new['t_inside'] == 0.0, 'new: airframe never enters the canopy footprint'),
            (new['t_occ'] == 0.0, 'new: canopy never counted as camera occlusion'),
            (new['req_active'] and abs(new['req_target'] - track_z) < 1e-3,
             'fly_px4_1 requests band (active=1, target=track_z) while tracking'),
            (new['finite'] and old['finite'], 'all setpoints finite'),
            (old['goal_inside'] > 0, 'old: EGO goal placed under the canopy'),
            (new['goal_inside'] == 0, 'new: EGO goal never under the canopy'),
            (new['goal_pause_n'] > 0 and new['goal_pause_dmax'] <= 10.5 + 0.5,
             'new: person standing mid-canopy -> observe from outside within 10.5m (outer rings)'),
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
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32MultiArray, Header, Int8
    from track_msgs.msg import TrackingTarget

    rospy.init_node('smoke_worker_' + tag)
    scale = float(rospy.get_param('/swarm_coordinates/local_xy_scale', 1.0))
    ns = '/typhoon_h480_0/mavros'
    raw, req, diag, goals = [], [], [], []
    rospy.Subscriber(ns + '/setpoint_raw/local', PositionTarget, raw.append, queue_size=200)
    rospy.Subscriber('/tracking/standoff_diag_1', Float32MultiArray,
                     lambda d: diag.append(tuple(d.data)), queue_size=50)
    rospy.Subscriber('/ego_goal_1', PoseStamped,
                     lambda g: goals.append((g.pose.position.x, g.pose.position.y)), queue_size=50)
    rospy.Subscriber('/typhoon_h480_0/body_band_request', Float32MultiArray,
                     lambda m: req.append(tuple(m.data)), queue_size=20)
    pubs = {
        'state': rospy.Publisher(ns + '/state', State, queue_size=1),
        'pose': rospy.Publisher(ns + '/local_position/pose', PoseStamped, queue_size=1),
        'vel': rospy.Publisher(ns + '/local_position/velocity_local', TwistStamped, queue_size=1),
        'mode': rospy.Publisher('/tracking/mode_1', Int8, queue_size=1),
        'target': rospy.Publisher('/tracking/state_1', TrackingTarget, queue_size=1),
        'cmd': rospy.Publisher('/ego/pos_cmd_1', PositionCommand, queue_size=1),
        'lidar': rospy.Publisher('/typhoon_h480_0/lidar_cloud_world', PointCloud2, queue_size=1),
        'band': rospy.Publisher('/typhoon_h480_0/body_band_cloud', PointCloud2, queue_size=1),
        'overhead': rospy.Publisher('/typhoon_h480_0/body_band_overhead', PointCloud2, queue_size=1),
    }
    angles = np.linspace(-3.14, 3.14, 512)
    eave = [(a, b, 4.5) for a, b in eave_points()]

    def scan(x, y):
        ux, uy = np.cos(angles), np.sin(angles)
        best = np.full(angles.shape, np.inf)
        for cx, cy in PILLARS:
            fx, fy = x - cx, y - cy
            b = fx * ux + fy * uy
            disc = b * b - (fx * fx + fy * fy - PILLAR_R ** 2)
            with np.errstate(invalid='ignore'):
                t = -b - np.sqrt(disc)
            hit = (disc >= 0) & (t >= 0.5) & (t <= 20.0)
            best = np.where(hit & (t < best), t, best)
        m = np.isfinite(best)
        return [(float(x + best[i] * ux[i]), float(y + best[i] * uy[i]), 4.5) for i in np.nonzero(m)[0]]

    seg_len = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(PERSON_PATH, PERSON_PATH[1:])]

    def person(t):
        t = max(0.0, t)
        for k, ((a, b), L) in enumerate(zip(zip(PERSON_PATH, PERSON_PATH[1:]), seg_len)):
            dur = PAUSE_SEC if k == PAUSE_AT else L / PERSON_SPEED
            if t <= dur:
                if L < 1e-9:
                    return a[0], a[1], 0.0, 0.0
                k_ = t * PERSON_SPEED / L
                return (a[0] + k_ * (b[0] - a[0]), a[1] + k_ * (b[1] - a[1]),
                        (b[0] - a[0]) / L * PERSON_SPEED, (b[1] - a[1]) / L * PERSON_SPEED)
            t -= dur
        return PERSON_PATH[-1] + (0.0, 0.0)

    x, y, vx, vy, z = DRONE_START[0], DRONE_START[1], 0.0, 0.0, 4.5
    rate = rospy.Rate(50)
    t0 = time.monotonic()
    walk_start = 6.0
    walk_total = sum(PAUSE_SEC if k == PAUSE_AT else L / PERSON_SPEED for k, L in enumerate(seg_len))
    t_end = walk_start + walk_total + 3.0
    last_scan = last_band = -1.0
    m = dict(t_inside=0.0, min_gap=1e9, t_occ=0.0, t=0.0, d_sum=0.0, t_under=0.0, d_under_sum=0.0,
             finite=True, goal_inside=0, goal_under_n=0, goal_under_dmax=0.0, occ_events=[],
             goal_pause_n=0, goal_pause_dmax=0.0, goal_min_gap=1e9)
    pause_t0 = walk_start + seg_len[0] / PERSON_SPEED
    while not rospy.is_shutdown():
        el = time.monotonic() - t0
        if el > t_end:
            break
        now = rospy.Time.now()
        px, py, pvx, pvy = person(el - walk_start)
        if el < walk_start:
            pvx = pvy = 0.0
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
        h = Header(stamp=now, frame_id='world')
        if el - last_scan >= 1.0 / 30.0:
            pubs['lidar'].publish(point_cloud2.create_cloud_xyz32(h, scan(x, y)))
            last_scan = el
        if tag == 'new' and el - last_band >= 0.1:
            pubs['band'].publish(point_cloud2.create_cloud_xyz32(h, eave))
            pubs['overhead'].publish(point_cloud2.create_cloud_xyz32(h, eave))
            last_band = el
        if el >= walk_start:
            gap = canopy_gap(x, y)
            m['min_gap'] = min(m['min_gap'], gap)
            m['t_inside'] += dt * (gap < BODY_R)
            d = math.hypot(x - px, y - py)
            m['d_sum'] += dt * d
            m['t'] += dt
            if inside_canopy(px, py):
                m['t_under'] += dt
                m['d_under_sum'] += dt * d
            # fly_px4_1 站位诊断 [11] = 站位->行人视线离(遮挡)墙距离, -1 = 无净空场;
            # 檐口若被当成遮挡, 棚外站位看棚下行人的视线要穿过它 -> ~0
            if diag and len(diag[-1]) > 11 and 0.0 <= diag[-1][11] < 0.1:
                m['t_occ'] += dt
                if len(m['occ_events']) < 5:
                    m['occ_events'].append([round(el, 2), round(x, 2), round(y, 2), round(px, 2),
                                            round(py, 2), round(diag[-1][11], 2),
                                            int(diag[-1][6]), round(diag[-1][7], 2)])
            # 发给 EGO 的目标 (站位/引导点): 不能落在棚下
            if goals:
                gx, gy = goals[-1]
                m['goal_min_gap'] = min(m['goal_min_gap'], canopy_gap(gx, gy))
                if canopy_gap(gx, gy) < 0.0:
                    m['goal_inside'] += 1
                    if m['goal_inside'] % 100 == 1 and len(m['occ_events']) < 30 and diag:
                        d = diag[-1]
                        m['occ_events'].append(['goal', round(el, 2), round(x, 2), round(y, 2),
                                                round(px, 2), round(py, 2), round(gx, 2), round(gy, 2),
                                                int(d[6]), round(d[7], 2), round(d[10], 2), int(d[12])])
                if inside_canopy(px, py):
                    m['goal_under_n'] += 1
                    m['goal_under_dmax'] = max(m['goal_under_dmax'], math.hypot(gx - px, gy - py))
                # 人停在棚下中部 (留 2s 让站位收敛)
                if pause_t0 + 2.0 <= el < pause_t0 + PAUSE_SEC:
                    m['goal_pause_n'] += 1
                    m['goal_pause_dmax'] = max(m['goal_pause_dmax'], math.hypot(gx - px, gy - py))
        rate.sleep()
    m['d_mean'] = m['d_sum'] / max(m['t'], 1e-9)
    m['d_under'] = m['d_under_sum'] / max(m['t_under'], 1e-9)
    active = [r for r in req if len(r) >= 2 and r[1] > 0.5]
    m['req_active'] = bool(active)
    m['req_target'] = float(active[-1][0]) if active else -1.0
    import yaml
    Path(metrics_path).write_text(yaml.safe_dump({k: (float(v) if isinstance(v, (float, np.floating)) else v)
                                                  for k, v in m.items()}))
    return 0


if __name__ == '__main__':
    if len(sys.argv) >= 4 and sys.argv[1] == '--worker':
        sys.exit(worker(sys.argv[2], sys.argv[3]))
    sys.exit(run_isolated())
