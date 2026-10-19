#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""监视六机 RACER 网格进度、距离、速度、FSM 和机间距，不参与控制。
进度分母为观测到的活跃网格并集峰值，不代表地图体积覆盖率。
可选发布 /expl_monitor/drones 标记及有限长度航迹。"""
import math
import itertools
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

try:
    from exploration_manager.msg import DroneState
    HAVE_DRONE_STATE = True
except ImportError:      # racer_ws 没 source 时退化: 只是没有网格进度
    HAVE_DRONE_STATE = False

# RACER 的 FSM 状态枚举 (fast_exploration_fsm.h 的 EXPL_STATE)
FSM_NAMES = ['INIT', 'WAIT_TRIG', 'PLAN_TRAJ', 'PUB_TRAJ', 'EXEC_TRAJ', 'FINISH', 'IDLE']

C = {'r': '\033[31m', 'y': '\033[33m', 'g': '\033[32m', 'b': '\033[1m', '0': '\033[0m'}

# 6 机航迹配色 (rviz 里区分各机)
TRAIL_RGB = [(1.0, 0.30, 0.30), (1.0, 0.65, 0.10), (1.0, 1.0, 0.25),
             (0.35, 1.0, 0.35), (0.35, 0.75, 1.0), (0.80, 0.45, 1.0)]


class ExplMonitor:
    def __init__(self):
        self.n = rospy.get_param('~drone_num', 6)
        self.period = rospy.get_param('~period', 2.0)
        self.time_limit = rospy.get_param('~time_limit', 600.0)
        self.warn_dist = rospy.get_param('~warn_dist', 2.0)
        self.crit_dist = rospy.get_param('~crit_dist', 1.2)
        self.stuck_sec = rospy.get_param('~stuck_sec', 20.0)
        self.stuck_dist = rospy.get_param('~stuck_dist', 1.0)
        self.pub_markers = rospy.get_param('~publish_markers', True)
        # 航迹折线(飞机飞过的路线)默认不显示 —— 只留飞机球/机号/HUD, rviz 干净得多。
        # 想看历史路线: show_trail:=true
        self.show_trail = rospy.get_param('~show_trail', False)
        self.trail_max = rospy.get_param('~trail_max', 600)
        self.trail_cleared = False   # latch 话题里的旧折线只需删一次
        self.frame_id = rospy.get_param('~frame_id', 'world')

        idx = range(1, self.n + 1)
        self.pos = {i: None for i in idx}
        self.vel = {i: 0.0 for i in idx}
        self.state = {i: -1 for i in idx}
        self.travel = {i: 0.0 for i in idx}       # 累计已飞距离
        self.last_pos = {i: None for i in idx}
        self.ref_pos = {i: None for i in idx}     # 卡死判定的参考点
        self.ref_t = {i: None for i in idx}
        self.last_rx = {i: None for i in idx}     # 最后一次收到里程计的时刻
        self.grids = {i: 0 for i in idx}          # 该机剩余网格数
        self.grid_set = {i: set() for i in idx}
        self.trail = {i: [] for i in idx}         # rviz 航迹点
        self.min_dist_ever = 9e9
        self.min_dist_pair = None
        self.crit_events = 0
        self.t0 = None

        for i in idx:
            ns = 'typhoon_h480_%d' % (i - 1)
            rospy.Subscriber('/%s/odom_world_relay' % ns, Odometry,
                             self.odom_cb, callback_args=i, queue_size=1)
            rospy.Subscriber('/exploration/state_%d' % i, Int8,
                             self.state_cb, callback_args=i, queue_size=1)
        if HAVE_DRONE_STATE:
            rospy.Subscriber('/swarm_expl/drone_state', DroneState,
                             self.drone_state_cb, queue_size=10)

        self.marker_pub = (rospy.Publisher('/expl_monitor/drones', MarkerArray,
                                           queue_size=1, latch=True)
                           if self.pub_markers else None)

        rospy.Timer(rospy.Duration(self.period), self.report)
        rospy.loginfo('[expl_monitor] %d 机, 每 %.1fs 打印一次', self.n, self.period)

    # 回调
    def odom_cb(self, msg, i):
        p = msg.pose.pose.position
        cur = (p.x, p.y, p.z)
        v = msg.twist.twist.linear
        self.vel[i] = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
        if self.last_pos[i] is not None:
            d = math.dist(cur, self.last_pos[i])
            if d < 2.0:            # 跳变(EKF 发散)不计入里程
                self.travel[i] += d
        self.last_pos[i] = cur
        self.pos[i] = cur
        self.last_rx[i] = rospy.Time.now()
        # 航迹按 0.5m 抽稀: 10 分钟 x 3m/s 全存下来是 36000 点/机, rviz 会卡
        # 不显示航迹时连累积都省掉
        if self.show_trail:
            tr = self.trail[i]
            if not tr or math.dist(cur, tr[-1]) > 0.5:
                tr.append(cur)
                if len(tr) > self.trail_max:
                    del tr[0]
        if self.t0 is None:
            self.t0 = rospy.Time.now()
        if self.ref_pos[i] is None:
            self.ref_pos[i], self.ref_t[i] = cur, rospy.Time.now()

    def state_cb(self, msg, i):
        self.state[i] = int(msg.data)

    def drone_state_cb(self, msg):
        i = int(msg.drone_id)
        if i in self.grid_set:
            self.grid_set[i] = set(msg.grid_ids)
            self.grids[i] = len(msg.grid_ids)

    # 卡死判定
    def stuck_secs(self, i):
        """返回该机"已静止秒数", 未卡死返回 0"""
        if self.pos[i] is None or self.ref_pos[i] is None:
            return 0.0
        now = rospy.Time.now()
        if math.dist(self.pos[i], self.ref_pos[i]) > self.stuck_dist:
            self.ref_pos[i], self.ref_t[i] = self.pos[i], now
            return 0.0
        return (now - self.ref_t[i]).to_sec()

    # rviz 标记
    def _mk(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = rospy.Time.now()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        return m

    def publish_markers(self, near, pct, el):
        """near: {机号: 与最近邻的距离}, 用来给球体上色"""
        arr = MarkerArray()
        for i in range(1, self.n + 1):
            p = self.pos[i]
            if p is None:
                continue
            r, g, b = TRAIL_RGB[(i - 1) % len(TRAIL_RGB)]
            sk = self.stuck_secs(i)
            d = near.get(i, 9e9)

            ball = self._mk('drone', i, Marker.SPHERE)
            ball.pose.position.x, ball.pose.position.y, ball.pose.position.z = p
            ball.scale.x = ball.scale.y = ball.scale.z = 1.3   # 机身对角尺寸, 一眼看出会不会碰
            if d < self.crit_dist or sk >= self.stuck_sec:
                ball.color.r, ball.color.g, ball.color.b = 1.0, 0.0, 0.0
            elif d < self.warn_dist:
                ball.color.r, ball.color.g, ball.color.b = 1.0, 0.9, 0.0
            else:
                ball.color.r, ball.color.g, ball.color.b = r, g, b
            ball.color.a = 0.85
            arr.markers.append(ball)

            txt = self._mk('label', i, Marker.TEXT_VIEW_FACING)
            txt.pose.position.x, txt.pose.position.y = p[0], p[1]
            txt.pose.position.z = p[2] + 1.4
            txt.scale.z = 1.6
            st = FSM_NAMES[self.state[i]] if 0 <= self.state[i] < len(FSM_NAMES) else '-'
            txt.text = '%d %s %.1fm/s%s' % (i, st, self.vel[i],
                                            ' 卡死' if sk >= self.stuck_sec else '')
            txt.color.r = txt.color.g = txt.color.b = 1.0
            arr.markers.append(txt)

            if self.show_trail:
                if len(self.trail[i]) >= 2:
                    ln = self._mk('trail', i, Marker.LINE_STRIP)
                    ln.scale.x = 0.35
                    ln.color.r, ln.color.g, ln.color.b, ln.color.a = r, g, b, 0.75
                    ln.points = [Point(x=q[0], y=q[1], z=q[2]) for q in self.trail[i]]
                    arr.markers.append(ln)
            elif not self.trail_cleared:
                # 话题是 latch 的, 上一次运行残留的折线不会自己消失, 发一次 DELETE 清掉
                ln = self._mk('trail', i, Marker.LINE_STRIP)
                ln.action = Marker.DELETE
                arr.markers.append(ln)

        hud = self._mk('hud', 0, Marker.TEXT_VIEW_FACING)
        hud.pose.position.x, hud.pose.position.y, hud.pose.position.z = -45.0, 45.0, 12.0
        hud.scale.z = 5.0
        hud.color.r = hud.color.g = hud.color.b = 1.0
        hud.text = 't %.0f/%.0fs   完成 %.0f%%' % (el, self.time_limit, pct)
        arr.markers.append(hud)

        self.marker_pub.publish(arr)
        self.trail_cleared = True

    # 打印
    def report(self, _):
        if self.t0 is None:
            rospy.loginfo_throttle(5.0, '[expl_monitor] 等待里程计...')
            return
        el = (rospy.Time.now() - self.t0).to_sec()

        # 网格进度
        union = set()
        for s in self.grid_set.values():
            union |= s
        remain = len(union)
        self.peak = max(getattr(self, 'peak', 0), remain)
        pct = (1.0 - remain / self.peak) * 100.0 if self.peak else 0.0

        # 机间距
        pairs = []
        for a, b in itertools.combinations(range(1, self.n + 1), 2):
            if self.pos[a] and self.pos[b]:
                pairs.append((math.dist(self.pos[a], self.pos[b]), a, b))
        pairs.sort()
        near = {}                       # 每机与最近邻的距离, 给 rviz 球体上色用
        for d, a, b in pairs:
            near[a] = min(near.get(a, 9e9), d)
            near[b] = min(near.get(b, 9e9), d)
        if pairs and pairs[0][0] < self.min_dist_ever:
            self.min_dist_ever, self.min_dist_pair = pairs[0][0], (pairs[0][1], pairs[0][2])
        if pairs and pairs[0][0] < self.crit_dist:
            self.crit_events += 1

        lines = []
        over = ' %s[超时]%s' % (C['r'], C['0']) if el > self.time_limit else ''
        lines.append('%s══ 探索监视  t=%.0fs / %.0fs%s  网格 %d/%d 剩余 (%.0f%% 完成)%s' %
                     (C['b'], el, self.time_limit, over, remain, self.peak, pct, C['0']))
        lines.append(' 机  状态       位置(x, y, z)              速度   已飞    格  静止')
        for i in range(1, self.n + 1):
            p = self.pos[i]
            st = FSM_NAMES[self.state[i]] if 0 <= self.state[i] < len(FSM_NAMES) else '-'
            sk = self.stuck_secs(i)
            if p is None:
                lines.append(' %d  %-10s (无里程计)' % (i, st))
                continue
            mark = ''
            # 里程计断流(mavros 掉了/ 节点死了)和真卡死要区分开, 否则会误判
            stale = (rospy.Time.now() - self.last_rx[i]).to_sec() if self.last_rx[i] else 0.0
            if stale > 3.0:
                mark = '%s里程计断流 %.0fs%s' % (C['r'], stale, C['0'])
            elif sk >= self.stuck_sec:
                mark = '%s%3.0fs 卡死%s' % (C['r'], sk, C['0'])
            elif sk >= self.stuck_sec / 2:
                mark = '%s%3.0fs%s' % (C['y'], sk, C['0'])
            lines.append(' %d  %-10s (%7.1f,%7.1f,%5.2f)  %4.1fm/s %5.0fm %4d  %s' %
                         (i, st, p[0], p[1], p[2], self.vel[i], self.travel[i],
                          self.grids[i], mark))

        if pairs:
            d, a, b = pairs[0]
            col = C['r'] if d < self.crit_dist else (C['y'] if d < self.warn_dist else C['g'])
            lines.append(' 最近机对 %s%.2fm (%d-%d)%s   历史最小 %.2fm%s   危险采样 %d 次' %
                         (col, d, a, b, C['0'], self.min_dist_ever,
                          (' (%d-%d)' % self.min_dist_pair) if self.min_dist_pair else '',
                          self.crit_events))
            close = ['%d-%d:%.1f' % (a, b, d) for d, a, b in pairs[1:4] if d < self.warn_dist]
            if close:
                lines.append(' 其他贴近: ' + '  '.join(close))
        print('\n'.join(lines) + '\n', flush=True)

        if self.marker_pub is not None:
            self.publish_markers(near, pct, el)


if __name__ == '__main__':
    rospy.init_node('expl_monitor')
    try:
        ExplMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
