#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""融合六路视觉并排他分配跟踪任务，独占发布六个裁判目标话题。

单目仅作导航，纯深度航迹通过质量与身份门控后发布。两条红色航迹独立关联、
发布和确认消除；影子计数、消失确认及复活仅使用本系统观测，不读取裁判或仿真真值。
连续发布期间保护认领，失联或占用超时后释放；内部消除状态由巡逻继续复核。"""

import copy
import json
import math
import os
import sys
import threading
import uuid

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header, Int8, String

from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from track_msgs.msg import (Detection, DetectionArray, TargetState, TargetArray,
                            TrackingTarget, TrackingView)

# catkin's devel wrapper executes this source without adding its directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from publication_prediction import PublicationPrediction

# 裁判接口 (score_cal.py 的镜像, 不可改)
# actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}
JUDGE_TOPICS = {
    'green': '/actor_green_info',
    'blue': '/actor_blue_info',
    'brown': '/actor_brown_info',
    'white': '/actor_white_info',
    'red1': '/actor_red1_info',
    'red2': '/actor_red2_info',
}
CLS_NAMES = ['green', 'blue', 'brown', 'white', 'red']
# 每个颜色对应几个 actor: 红色有两个(actor_4/actor_5), 其余各一个
SLOTS_OF_CLS = {
    'green': ['green'], 'blue': ['blue'], 'brown': ['brown'],
    'white': ['white'], 'red': ['red1', 'red2'],
}
ALL_SLOTS = ['green', 'blue', 'brown', 'white', 'red1', 'red2']

ACTOR_Z_ORIGIN = 1.0191      # 裁判比对的就是这个高度的 (x,y)

# 深度法在 Detection.method 里的位 (与 swarm_vision 的 M_DEPTH 一致)
M_DEPTH = 1
M_JOINT = 8

# 目标状态
ST_UNSEEN, ST_TRACKED, ST_COASTING, ST_ELIMINATED = 0, 1, 2, 3
# 飞机模式 (发给 fly_px4; 速度分层由 fly_px4 按距离自己决定, mode 只报状态)
MODE_OFF, MODE_APPROACH, MODE_LOCKED, MODE_REACQUIRE = 0, 1, 2, 3


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def hungarian(cost):
    """纯 numpy 匈牙利分配，返回 (row_idx, col_idx)。
    任意形状代价矩阵补为方阵，不可行配对用 np.inf。"""
    cost = np.asarray(cost, dtype=float)
    if cost.size == 0:
        return np.array([], dtype=int), np.array([], dtype=int)
    n, m = cost.shape
    k = max(n, m)
    # 补成方阵; 用一个比任何有限代价都大的数填充, 而不是 inf (inf 会让内部减法出 nan)
    finite = cost[np.isfinite(cost)]
    big = (finite.max() + 1.0) * (k + 1) if finite.size else 1.0
    C = np.full((k, k), big, dtype=float)
    C[:n, :m] = np.where(np.isfinite(cost), cost, big)

    INF = float('inf')
    u = np.zeros(k + 1)
    v = np.zeros(k + 1)
    p = np.zeros(k + 1, dtype=int)     # p[j] = 匹配到列 j 的行
    way = np.zeros(k + 1, dtype=int)
    for i in range(1, k + 1):
        p[0] = i
        j0 = 0
        minv = np.full(k + 1, INF)
        used = np.zeros(k + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta = INF
            j1 = -1
            for j in range(1, k + 1):
                if used[j]:
                    continue
                cur = C[i0 - 1, j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            if j1 < 0:
                break
            for j in range(k + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    rows, cols = [], []
    for j in range(1, k + 1):
        i = p[j]
        if 1 <= i <= n and 1 <= j <= m:
            rows.append(i - 1)
            cols.append(j - 1)
    order = np.argsort(rows)
    return np.array(rows)[order], np.array(cols)[order]


class Track:
    """一条目标航迹: 常速卡尔曼 + 影子计数 + 状态机"""

    def __init__(self, tid, cls_name, sigma_a, pub_gap_reset=0.9):
        self.tid = tid
        self.cls = cls_name
        self.slot = ''
        self.x = np.zeros(4)             # [x, y, vx, vy]
        self.P = np.eye(4) * 1e4
        self.sigma_a = sigma_a
        # 停发超过这么久就认为这一轮连续发布结束 (与裁判的 interval<1s 判据一致)
        self.pub_gap_reset = pub_gap_reset
        self.inited = False
        self.status = ST_UNSEEN
        self.last_meas = None            # 最后一次有效量测的时刻
        self.last_pred = None            # KF 上次推进到的时刻
        self.assigned = 0                # 负责的机号, 0=无
        self.assigned_since = None
        self.assignment = ''             # 每次认领的新凭证，旧飞控回执不能授权新任务
        self.owner_observation_assignment = ''  # 本任务真正采用过负责机观测的凭证
        self.last_tracking_ack = None    # 本次任务最后一次有效 TRACK 回执，不是认领时刻
        self.shared_source = 0           # 接手机尚未看到目标时，持续提供导航坐标的发现机
        # 本任务负责机还没亲眼看到过目标（共享坐标接近 / 排队重捕）。只看这个
        # 标志决定 25s 获取窗口；负责机跟丢后改用旁观者引导不算“获取期”。
        self.blind_acquire = False
        self.reacq_arrived_since = None  # 盲接近到达冻结保存点附近的时刻
        self.queued_meas = None          # 已为这次观测排队派过/空飞过，不再重复派
        self.seen_by = {}                # drone_id -> 最近关联到候选检测的成像时刻
        # accepted_seen_by 只记录被采用的观测；拒绝量测及其他机观测不能续租当前负责机。
        self.accepted_seen_by = {}        # drone_id -> 最近被航迹真正采用的量测时刻
        # 旁观机的几何关联观测单独缓存，只用于发现、存活证明和退出握手后的
        # 接管。接手机获取目标前只允许 shared_source 更新，获取后隔离旁观机。
        self.observer_detections = {}     # drone_id -> (Detection, 成像时刻)
        self.identity_det = None         # 红色关联用原始观测及其不确定度
        self.identity_t = None
        self.identity_epoch = 0          # 重建后不能沿用旧的同帧双人证据
        self.identity_blocked_until = 0.0
        self.report_blocked_until = 0.0  # 长期未消除后的短暂上报冷却
        # 防"争抢行人": 释放时保留上一任 assigned 机号, 重新认领时优先还给它,
        # 而不是被恰好路过、也看到同一人的另一架机抢走。
        self.last_assigned = 0           # 上一任 assigned 机号 (释放后不清空)
        self.last_assigned_seen = None   # 上一任 assigned 机最近检测到本目标的时刻
        # 两帧确认: 第一次检测先挂起, 第二次(近距离/短间隔)才真正建航迹
        self.pending_det = None
        self.pending_t = None
        # 门控外的新位置不能凭一两帧就把整条航迹搬过去。候选必须由同一架机
        # 连续、空间一致地观察一段时间，才认为是行人瞬移/航迹确实丢失。
        self.reinit_det = None
        self.reinit_t0 = None
        self.reinit_last_t = None
        self.reinit_n = 0
        # 远距纯单目的距离可能从 12m 跳到 50m，但方位仍可用于接近。该模式下
        # x/y 只是飞机前方的滚动导航点，绝不能发给裁判。
        self.bearing_only_nav = False
        self.bearing_nav_angle = None
        self.bearing_nav_drone = 0
        # 分层定位的下游状态
        self.last_depth_t = None         # 最近一次带深度量测的时刻 (发布门限用)
        self.last_slant = None           # 最近一次量测的斜距
        self.last_meas_drone = 0         # 最近一次量测来自哪架机
        # 影子计数: 我们自己连续发布了多久
        self.pub_run_start = None
        self.last_pub = None
        self.gate_fail = 0
        self.last_gate_fail = None
        self.last_residual = 0.0          # 最近一次 update 时"检测 vs 预测"的水平残差(m)
        self.last_innovation = np.zeros(2)
        self.last_det = None              # 真正采用的观测，不能由最近同色框事后猜测
        self.last_published_det = None
        self.last_vision_receive_stamp = None
        self.last_measurement_update_stamp = None
        self.pending_view = None         # accepted owner observation in this batch
        self.view_published_assignment = ''
        self.view_published_stamp = None
        self.publication_predictor = PublicationPrediction()
        self.cls_seen = {}               # cls_id -> 最近一次看到的时刻
        self.eliminated_at = None
        # 裁判响应检查: 影子计数满 lock_need 后的时刻 + judge_grace,
        # 那时人还在 = 裁判没删(有超差/断流消息) -> 影子计数清零重试
        self.judge_check_at = None
        self.judge_rejects = 0           # 裁判否决次数 (定位质量的直接证据)
        self.det_gap_since = None

    def clear_reinit_candidate(self):
        self.reinit_det = None
        self.reinit_t0 = None
        self.reinit_last_t = None
        self.reinit_n = 0

    def reinit_candidate_ready(self, det, t, min_updates, min_sec,
                               max_dist, max_gap):
        """累计门控外的新位置；返回是否已达到重初始化确认条件。"""
        if (not math.isfinite(t) or (self.reinit_last_t is not None
                                    and t <= self.reinit_last_t)):
            # 重复/迟到帧不能凑确认次数，也不能抹掉较新的连续候选。
            return False
        candidate_ok = (
            self.reinit_det is not None
            and self.reinit_last_t is not None
            and 0.0 <= t - self.reinit_last_t <= max_gap
            and det.drone_id == self.reinit_det.drone_id
            and math.hypot(det.x - self.reinit_det.x,
                           det.y - self.reinit_det.y) <= max_dist)
        if candidate_ok:
            self.reinit_n += 1
            self.reinit_det = det
            self.reinit_last_t = t
        else:
            self.reinit_det = det
            self.reinit_t0 = t
            self.reinit_last_t = t
            self.reinit_n = 1
        span = t - self.reinit_t0
        # ROS 浮点时间的 0.1+0.1 可能得到 0.199999999；微小容差只消除数值误差，
        # 不放宽实际确认时长。
        return self.reinit_n >= min_updates and span + 1e-6 >= min_sec

    # KF
    def accepts_time(self, t):
        return (math.isfinite(t)
                and (self.last_pred is None or t >= self.last_pred))

    def predict(self, t):
        if not self.inited:
            return
        if self.last_pred is None:
            self.last_pred = t
            return
        dt = t - self.last_pred
        if dt <= 0.0:
            return
        # 连续白噪声加速度的 F/Q 对完整 dt 是精确解。不能只传播 1s
        # 却把状态标成 t；长失联是否重建航迹由上层重捕门限决定。
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
                     dtype=float)
        # 白噪声加速度模型
        s = self.sigma_a ** 2
        d3, d2 = dt ** 3 / 3.0, dt ** 2 / 2.0
        Q = np.array([[d3, 0, d2, 0], [0, d3, 0, d2],
                      [d2, 0, dt, 0], [0, d2, 0, dt]], dtype=float) * s
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_pred = t

    def prediction(self, t):
        """返回外推到 t 的状态副本，不改变仅由有序量测推进的航迹时间轴。"""
        x = self.x.copy()
        P = self.P.copy()
        if not self.inited or self.last_pred is None:
            return x, P
        dt = t - self.last_pred
        if dt <= 0.0:
            return x, P
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt],
                      [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        s = self.sigma_a ** 2
        d3, d2 = dt ** 3 / 3.0, dt ** 2 / 2.0
        Q = np.array([[d3, 0, d2, 0], [0, d3, 0, d2],
                      [d2, 0, dt, 0], [0, d2, 0, dt]], dtype=float) * s
        return F @ x, F @ P @ F.T + Q

    def R_of(self, det):
        """由 bearing + 各向异性 sigma 构造 2x2 量测噪声。

        测距误差是横向的 2~3 倍, 用各向同性 R 会把 KF 往视线方向拉偏。
        """
        c, s = math.cos(det.bearing), math.sin(det.bearing)
        Rot = np.array([[c, -s], [s, c]])
        D = np.diag([max(det.sigma_along, 0.03) ** 2,
                     max(det.sigma_cross, 0.03) ** 2])
        return Rot @ D @ Rot.T

    def mahalanobis2(self, det, t=None):
        if not self.inited:
            return 0.0
        x, P = (self.x, self.P) if t is None else self.prediction(t)
        S = P[:2, :2] + self.R_of(det)
        dz = np.array([det.x - x[0], det.y - x[1]])
        try:
            return float(dz @ np.linalg.solve(S, dz))
        except np.linalg.LinAlgError:
            return float('inf')

    def init_from(self, det, t):
        if not self.accepts_time(t):
            return False
        self.identity_epoch += 1
        # 新位置/导航模式建立了新的关联基准。旧旁观缓存即使时间仍新鲜，
        # 也不能在下一次接管时把已确认的新位置拉回去。
        self.observer_detections.clear()
        self.x = np.array([det.x, det.y, 0.0, 0.0])
        self.P = np.eye(4)
        self.P[:2, :2] = self.R_of(det) + np.eye(2) * 0.1
        self.inited = True
        self.last_pred = t
        self.last_meas = t
        self.status = ST_TRACKED
        self.last_depth_t = t if (det.method & M_DEPTH) else None
        self.last_slant = float(det.slant)
        self.last_meas_drone = int(det.drone_id)
        self.accepted_seen_by[int(det.drone_id)] = max(
            t, self.accepted_seen_by.get(int(det.drone_id), -1e9))
        self.last_residual = 0.0
        self.last_innovation = np.zeros(2)
        self.last_det = copy.copy(det)
        self.owner_observation_assignment = ''
        self.bearing_only_nav = False
        self.bearing_nav_angle = None
        self.bearing_nav_drone = 0
        self.clear_reinit_candidate()
        self.publication_predictor.reset()
        if det.method & M_DEPTH:
            self.publication_predictor.observe(t, int(det.drone_id), self.x[2:])
        # 重新初始化等于航迹断了 -> 影子计数必须清零; 上一任 assigned 机也随之失效
        self.pub_run_start = None
        self.last_assigned = 0
        self.last_assigned_seen = None
        return True

    def update(self, det, t):
        # 真实坐标 KF 是裁判坐标源，只允许深度量测。单目由 fusion 的方位引导
        # 分支直接维护滚动导航点；在这里拒绝可防以后新增调用路径时意外混入。
        if not (det.method & M_DEPTH):
            return False
        # 跨 DetectionArray 的回调也可能乱序。旧量测不能更新到较新的状态上，
        # 否则 last_meas 会倒退、速度被反向拉偏，随后发布年龄和外推全错。
        if not self.accepts_time(t):
            return False
        self.predict(t)
        H = np.array([[1., 0, 0, 0], [0, 1., 0, 0]])
        R = self.R_of(det)
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        dz = np.array([det.x - self.x[0], det.y - self.x[1]])
        self.last_residual = float(math.hypot(dz[0], dz[1]))
        self.last_innovation = dz.copy()
        self.last_det = copy.copy(det)
        self.x = self.x + K @ dz
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_meas = t
        self.status = ST_TRACKED
        self.cls_seen[int(det.cls_id)] = t
        if det.method & M_DEPTH:
            self.last_depth_t = t
        self.last_slant = float(det.slant)
        self.last_meas_drone = int(det.drone_id)
        self.accepted_seen_by[int(det.drone_id)] = max(
            t, self.accepted_seen_by.get(int(det.drone_id), -1e9))
        self.publication_predictor.observe(t, int(det.drone_id), self.x[2:])
        return True

    def publication_prediction(self, t):
        """Change only judge displacement; association/navigation keep CV KF.

        Keep the original full-age covariance: damping a stale mean does not
        make the target's present location known or relax quality gates.
        """
        x, P = self.prediction(t)
        xy, metadata = self.publication_predictor.predict(
            self.x[:2], self.x[2:], self.last_pred, t)
        x[:2] = xy
        return x, P, metadata

    @property
    def pos_std(self):
        return float(math.sqrt(max(self.P[0, 0] + self.P[1, 1], 0.0)))

    def predicted_pos_std(self, t):
        _, P = self.prediction(t)
        return float(math.sqrt(max(P[0, 0] + P[1, 1], 0.0)))

    def lock_seconds(self, now):
        """返回连续发布时长；last_pub 超过 pub_gap_reset 即视为本轮中断。"""
        if self.pub_run_start is None or self.last_pub is None:
            return 0.0
        if now - self.last_pub > self.pub_gap_reset:
            return 0.0
        return self.last_pub - self.pub_run_start


class SwarmFusion:
    def __init__(self):
        self.drone_num = int(rospy.get_param('~drone_num', 6))
        self.ns_prefix = rospy.get_param('~ns_prefix', 'typhoon_h480')

        # 过程噪声覆盖静止、行走和转向状态。
        self.sigma_a = float(rospy.get_param('~sigma_a', 1.2))
        self.turn_prediction_guard = bool(rospy.get_param('~turn_prediction_guard', True))
        self.turn_prediction_window = float(rospy.get_param('~turn_prediction_window', 0.5))
        self.turn_prediction_deg = float(rospy.get_param('~turn_prediction_deg', 45.0))
        self.turn_prediction_decay_sec = float(rospy.get_param('~turn_prediction_decay_sec', 0.15))
        self.gate_chi2 = float(rospy.get_param('~gate_chi2', 9.21))   # chi2_2 @99%
        self.gate_abs = float(rospy.get_param('~gate_abs', 2.5))

        # init_* 用于红色复活；普通颜色通过连续且空间一致的观测复活。
        self.init_gap = float(rospy.get_param('~init_gap', 2.0))
        self.init_dist = float(rospy.get_param('~init_dist', 4.0))
        self.normal_revive_updates = int(
            rospy.get_param('~normal_revive_updates', 3))
        self.normal_revive_sec = float(
            rospy.get_param('~normal_revive_sec', 0.2))
        self.normal_revive_gap = float(
            rospy.get_param('~normal_revive_gap', 1.0))
        self.normal_revive_dist = float(
            rospy.get_param('~normal_revive_dist', 4.0))

        # 发布门限
        self.pub_rate = float(rospy.get_param('~pub_rate', 10.0))
        # New accepted depth is published immediately; the timer still keeps
        # the track alive between measurements using the same quality gates.
        self.publish_on_measurement = bool(rospy.get_param('~publish_on_measurement', True))
        self.max_meas_age = float(rospy.get_param('~max_meas_age', 0.4))
        self.max_pos_std = float(rospy.get_param('~max_pos_std', 0.35))
        # 分别限制量测后验和发布外推的不确定度；标准差门限不代表实际误差保证。
        self.pub_max_age = float(rospy.get_param('~pub_max_age', 0.9))
        self.pub_max_pos_std = float(rospy.get_param('~pub_max_pos_std', 0.9))
        # 用近期有效量测与 KF 的创新量检查发布质量。
        self.max_pub_residual = float(rospy.get_param('~max_pub_residual', 1.0))
        # 裁判的 interval 判据是 <1s, 留 0.1s 余量
        self.pub_gap_reset = float(rospy.get_param('~pub_gap_reset', 0.9))
        # 15s + 裁判采样余量: find_time[i] 是**裁判自己**看到的第一条好消息,
        # 而 actors_pos[i] 最多有 0.1s 陈旧(10Hz 轮询 get_model_state)
        self.lock_need = float(rospy.get_param('~lock_need', 16.5))
        # 连续发布期间保护认领，发布中断后才允许拒绝释放。
        self.count_lock_enable = bool(rospy.get_param('~count_lock_enable', True))
        # 计数和宽限结束后仍新鲜可见则重置影子计数，由当前负责机重试。
        self.judge_grace = float(rospy.get_param('~judge_grace', 2.0))
        # 裁判只发深度航迹。depth_hold 内允许用“最后一条深度航迹”的 KF 外推
        # 保持 10Hz 连续发送，但这期间到达的单目检测不会更新真实坐标 KF。
        self.depth_hold = float(rospy.get_param('~depth_hold', 0.8))
        # 深度消失多久后，从真实坐标 KF 转入单目方位接近。应不小于
        # pub_max_age：先安全外推完最后一段深度航迹，再改成只导航不发布。
        self.mono_nav_switch_age = float(
            rospy.get_param('~mono_nav_switch_age', max(self.pub_max_age, 0.9)))
        # 单目标占用上限: 认领后这么久仍没消除(裁判反复否决 / 不可得分的静态
        # 诱饵) -> 释放飞机 + 拉黑一阵。**不**标消除 —— 人还在, 过阵子再试。
        self.track_max_hold = float(rospy.get_param('~track_max_hold', 75.0))
        self.track_blacklist_sec = float(rospy.get_param('~track_blacklist_sec', 120.0))
        # 误判消除的复活判据 (红衣用): 红检测落在已消除红槽的消除点这附近
        self.revive_dist = float(rospy.get_param('~revive_dist', 3.0))
        # 飞行认领仍一人一机；同帧双框的可靠深度可证明 3m 内是两个红人。
        self.red_distinct_dist = float(rospy.get_param('~red_distinct_dist', 3.0))
        self.red_confirm_sec = float(rospy.get_param('~red_confirm_sec', 0.3))
        self.tracking_ack_timeout = float(rospy.get_param('~tracking_ack_timeout', 0.6))
        self.tracking_start_timeout = float(rospy.get_param('~tracking_start_timeout', 5.0))
        # 仅用于裁判发布/新认领的观测新鲜度；接近时俯仰会造成短时丢框，
        # 不能把 2s 的发布门限直接当作跟踪失败。
        self.owner_seen_timeout = float(rospy.get_param('~owner_seen_timeout', 2.0))
        # 负责机释放窗口与全局航迹记忆分别计时。
        self.owner_release_timeout = float(
            rospy.get_param('~owner_release_timeout', 8.0))
        # 跨机接近可能超过通常的 8s 丢失窗口。获取期有独立的总时限，发现机
        # 的观测不会无限延长它；接手机获得本机观测后恢复正常的丢失窗口。
        self.shared_acquire_timeout = max(self.owner_release_timeout, float(
            rospy.get_param('~shared_acquire_timeout', 25.0)))
        # 共享点过期且到达后仍未获取目标时提前释放，总等待另有上限。
        self.reacq_frozen_age = float(rospy.get_param('~reacq_frozen_age', 1.5))
        self.reacq_arrive_radius = float(rospy.get_param('~reacq_arrive_radius', 9.0))
        self.reacq_arrive_hold = float(rospy.get_param('~reacq_arrive_hold', 3.0))
        # 负责机失联时优先交给更近的空闲观察机，消除确认窗口受保护。
        self.handover_owner_age = float(rospy.get_param('~handover_owner_age', 2.0))
        self.handover_seen = float(rospy.get_param('~handover_seen', 0.6))
        self.handover_margin = float(rospy.get_param('~handover_margin', 3.0))
        # 发现机即将空闲且仍持续观察时，可等待它接手，减少远机无效接近。
        self.observer_wait_enable = bool(rospy.get_param('~observer_wait_enable', True))
        self.observer_wait_max = float(rospy.get_param('~observer_wait_max', 8.0))
        self.observer_wait_margin = float(rospy.get_param('~observer_wait_margin', 1.0))
        self.observer_wait_seen = float(rospy.get_param('~observer_wait_seen', 0.5))
        self.observer_wait_speed = float(rospy.get_param('~observer_wait_speed', 6.0))
        self.observer_wait_near = float(rospy.get_param('~observer_wait_near', 10.0))
        self.observer_wait_free_overhead = float(
            rospy.get_param('~observer_wait_free_overhead', 1.5))
        self.observer_wait_retire = float(rospy.get_param('~observer_wait_retire', 0.5))
        self.observer_waiting = {}   # slot -> 正在等待的发现机 (只用于日志去重)
        # 只剩一个红人时, 久未观测的已认领红航迹可被任何飞机的持续一致观测重建 (见 _sole_stale_red)
        self.red_sole_takeover = bool(rospy.get_param('~red_sole_takeover', True))
        # 有效量测过期后才允许重建航迹。
        self.gate_reinit_stale = float(rospy.get_param('~gate_reinit_stale', 1.5))
        # 重建需要连续且空间一致的新观测。
        self.reinit_confirm_sec = float(rospy.get_param('~reinit_confirm_sec', 1.0))
        self.reinit_confirm_updates = int(
            rospy.get_param('~reinit_confirm_updates', 5))
        self.reinit_confirm_dist = float(
            rospy.get_param('~reinit_confirm_dist', 0.8))
        self.reinit_confirm_gap = float(
            rospy.get_param('~reinit_confirm_gap', 0.5))
        # 消除点周边拉黑只保留给红色的身份关联。普通颜色改用上面的持续视觉
        # 确认，不能在复活状态机之前把检测直接丢掉。
        self.elim_radius = float(rospy.get_param('~elim_radius', 2.0))
        self.elim_blacklist_sec = float(rospy.get_param('~elim_blacklist_sec', 3.0))
        # 消失需持续确认，避免把短暂遮挡当作消除。
        self.elim_confirm_gap = float(rospy.get_param('~elim_confirm_gap', 2.0))
        # 已完成连续发布的一轮使用较短消失确认时间。
        self.elim_full_run = float(rospy.get_param('~elim_full_run', 15.0))
        self.elim_confirm_gap_full = min(self.elim_confirm_gap, float(
            rospy.get_param('~elim_confirm_gap_full', 2.0)))

        # 分配
        self.assign_rate = float(rospy.get_param('~assign_rate', 10.0))
        self.lost_timeout = float(rospy.get_param('~lost_timeout', 8.0))
        self.reacquire_lo = float(rospy.get_param('~reacquire_lo', 0.4))
        self.reacquire_hi = float(rospy.get_param('~reacquire_hi', 2.0))
        # 六个“内部已消除”不能立刻等同于裁判已全部删除。继续巡航复核一段时间，
        # 期间任何复活候选都会撤销计时，避免 mission_complete 让飞机过早降落。
        self.completion_confirm_sec = float(
            rospy.get_param('~completion_confirm_sec', 30.0))

        # 数据合规: 本节点**不订阅任何裁判话题、任何 Gazebo 真值话题/服务**。
        # (详见文件头) warmup 只是纯保险: 有检测本身就蕴含裁判早就在跑了。
        self.warmup_sec = float(rospy.get_param('~warmup_sec', 3.0))
        self.node_start = None
        # 标定用: 给发布的坐标加一个已知偏移, 用裁判**终端上的 find/OK 打印**反推
        # 真实误差(人眼读打印, 不订阅任何话题)。只在专门的调试轮里用。
        self.probe_dx = float(rospy.get_param('~probe_offset_x', 0.0))
        self.probe_dy = float(rospy.get_param('~probe_offset_y', 0.0))

        # 状态
        self.lock = threading.Lock()
        self.tracks = {}          # 红槽是视觉航迹编号，不是官方 actor 身份
        for i, s in enumerate(ALL_SLOTS):
            cls = 'red' if s.startswith('red') else s
            t = Track(i, cls, self.sigma_a, self.pub_gap_reset)
            t.publication_predictor = PublicationPrediction(
                self.turn_prediction_guard, self.turn_prediction_window,
                self.turn_prediction_deg, self.turn_prediction_decay_sec)
            t.slot = s
            self.tracks[s] = t
        self.drone_pos = [None] * self.drone_num
        self.drone_yaw = [0.0] * self.drone_num
        self.drone_mode = [MODE_OFF] * self.drone_num
        self.reject = {}          # (drone_id, slot) -> 拉黑到什么时候
        self.blacklist = []       # (x, y, 半径, 到期时刻)
        self.last_seen_by = {}    # slot -> drone_id, 谁最近看到了哪个航迹(谁看到谁跟)
        self.active_tracking = {}  # drone_id -> (认领凭证, 飞控回执时刻)
        self.red_pair_seen = {}    # drone_id -> (成像时刻, red1/red2 身份代次)
        self.retiring = {}         # drone_id -> (slot, 撤销时刻)，等旧机退出再改派
        self.all_eliminated_since = None
        self.t0 = None

        # 发布器
        self.actor_pubs = {s: rospy.Publisher(JUDGE_TOPICS[s], ActorInfo, queue_size=1)
                           for s in ALL_SLOTS}
        self.target_pubs = [rospy.Publisher('/tracking/target_%d' % (i + 1),
                                            Odometry, queue_size=2)
                            for i in range(self.drone_num)]
        self.state_pubs = [rospy.Publisher('/tracking/state_%d' % (i + 1),
                                           TrackingTarget, queue_size=2)
                           for i in range(self.drone_num)]
        self.view_pubs = [rospy.Publisher('/tracking/view_%d' % (i + 1),
                                         TrackingView, queue_size=1)
                          for i in range(self.drone_num)]
        self.diagnostics_pub = rospy.Publisher('/tracking/diagnostics', String,
                                               queue_size=100)
        self.mode_pubs = [rospy.Publisher('/tracking/mode_%d' % (i + 1),
                                          Int8, queue_size=2)
                          for i in range(self.drone_num)]
        self.complete_pub = rospy.Publisher('/tracking/mission_complete', Bool,
                                            queue_size=1, latch=True)
        self.targets_pub = rospy.Publisher('/tracking/targets', TargetArray,
                                           queue_size=2)
        self.complete_pub.publish(Bool(data=False))

        # 订阅
        rospy.Subscriber('/vision/detections', DetectionArray, self.det_cb,
                         queue_size=10)
        for i in range(self.drone_num):
            rospy.Subscriber('/%s_%d/odom_world_relay' % (self.ns_prefix, i),
                             Odometry, self.odom_cb, callback_args=i,
                             queue_size=2, tcp_nodelay=True)
            rospy.Subscriber('/tracking/reject_%d' % (i + 1), Int8,
                             self.reject_cb, callback_args=i, queue_size=4)
            rospy.Subscriber('/tracking/active_%d' % (i + 1), Header,
                             self.tracking_cb, callback_args=i, queue_size=2,
                             tcp_nodelay=True)
        # 只订阅本系统视觉、里程计桥接和跟踪反馈，不使用裁判或仿真真值。
        self.node_start = rospy.get_time()

        rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.publish_cb)
        rospy.Timer(rospy.Duration(1.0 / self.assign_rate), self.assign_cb)
        rospy.Timer(rospy.Duration(1.0), self.viz_cb)
        rospy.Timer(rospy.Duration(5.0), self.log_cb)

        self.check_sole_publisher()
        rospy.loginfo('[fusion] 启动: %d 机, 发布 %.0fHz, 锁定门限 %.1fs, '
                      '裁判仅用深度(保持 %.1fs)；单目仅作方位接近',
                      self.drone_num, self.pub_rate, self.lock_need,
                      self.depth_hold)
        rospy.loginfo('[fusion] 新有效深度立即发布=%s；定时发布继续保活',
                      self.publish_on_measurement)
        rospy.loginfo('[fusion] 转向外推保护=%s；窗口%.2fs 转角%.0fdeg 速度衰减%.2fs',
                      self.turn_prediction_guard, self.turn_prediction_window,
                      self.turn_prediction_deg, self.turn_prediction_decay_sec)

    # 自检
    def check_sole_publisher(self):
        """裁判话题上出现第二个发布者就是整轮报废 -> 启动时查一次并打醒目错误"""
        try:
            import rosgraph
            master = rosgraph.Master('/swarm_fusion')
            pubs = dict((t, n) for t, n in master.getSystemState()[0])
            for s, topic in JUDGE_TOPICS.items():
                others = [n for n in pubs.get(topic, []) if n != rospy.get_name()]
                if others:
                    rospy.logerr('[fusion] !! %s 上还有别的发布者: %s !! '
                                 '两个发布者会互相打断裁判的 15 秒计数, 必须关掉',
                                 topic, others)
        except Exception as e:
            rospy.logwarn('[fusion] 发布者自检跳过: %s', e)

    # 回调
    def odom_cb(self, msg, idx):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        with self.lock:
            self.drone_pos[idx] = np.array([p.x, p.y, p.z])
            self.drone_yaw[idx] = math.atan2(siny, cosy)

    def reject_cb(self, msg, idx):
        """暂时拉黑拒绝任务的 (机, 目标) 配对；连续发布期间保留认领保护。"""
        now = rospy.get_time()
        with self.lock:
            for s, t in self.tracks.items():
                if t.assigned == idx + 1:
                    if self.count_lock_enable and t.lock_seconds(now) > 0.0:
                        rospy.logwarn_throttle(
                            2.0, '[fusion] %d 号机报告接不了 %s, 但影子计数进行中'
                                 '(%.1fs) —— 计数保护锁: 先不撒手, 等发布断了再改派',
                            idx + 1, s, t.lock_seconds(now))
                        continue
                    self.reject[(idx + 1, s)] = now + 2.0
                    rospy.logwarn('[fusion] %d 号机报告无法接近 %s, 拉黑 2s 后重分配',
                                  idx + 1, s)
                    self._release(t, now)

    def tracking_cb(self, msg, idx):
        """飞控真正进入 TRACK 后回显本次凭证；frame_id 为空表示已退出。"""
        now = rospy.get_time()
        stamp = msg.stamp.to_sec()
        with self.lock:
            old = self.active_tracking.get(idx + 1, ('', -1.0))
            if old[1] <= stamp <= now + 0.05:
                self.active_tracking[idx + 1] = (msg.frame_id, stamp)
                if msg.frame_id and now - stamp <= self.tracking_ack_timeout:
                    for tr in self.tracks.values():
                        if tr.assigned == idx + 1 and tr.assignment == msg.frame_id:
                            tr.last_tracking_ack = stamp

    def _tracking_ready(self, tr, now):
        if not tr.assigned or not tr.assignment:
            return False
        token, stamp = self.active_tracking.get(tr.assigned, ('', -1e9))
        return (token == tr.assignment
                and 0.0 <= now - stamp <= self.tracking_ack_timeout
                and tr.owner_observation_assignment == tr.assignment
                and now - tr.accepted_seen_by.get(
                    tr.assigned, -1e9) <= self.owner_seen_timeout
                and now >= tr.identity_blocked_until)

    def _release(self, tr, now):
        if tr.assigned:
            self.retiring[tr.assigned] = (tr.slot, now)
            self.mode_pubs[tr.assigned - 1].publish(Int8(data=MODE_OFF))
        tr.assigned = 0
        tr.assigned_since = None
        tr.assignment = ''
        tr.owner_observation_assignment = ''
        tr.last_tracking_ack = None
        tr.shared_source = 0
        tr.blind_acquire = False
        tr.reacq_arrived_since = None
        if tr.cls != 'red':
            tr.pub_run_start = None
            tr.judge_check_at = None

    def _blind_at_saved_point(self, tr, now):
        """盲接近的保存点已冻结, 且接手机已在点附近停留 reacq_arrive_hold 秒仍未看到。"""
        p = self.drone_pos[tr.assigned - 1]
        if (p is None or tr.last_meas is None
                or now - self._last_observation_time(tr) <= self.reacq_frozen_age):
            tr.reacq_arrived_since = None
            return False
        # 与 emit_modes 下发给飞控的保存点一致。
        x_cmd, _ = tr.prediction(min(now, tr.last_meas + self.pub_max_age))
        if math.hypot(p[0] - x_cmd[0], p[1] - x_cmd[1]) > self.reacq_arrive_radius:
            tr.reacq_arrived_since = None
            return False
        if tr.reacq_arrived_since is None:
            tr.reacq_arrived_since = now
        return now - tr.reacq_arrived_since >= self.reacq_arrive_hold

    def _help_blind_owner(self, tr, reds, owner_age, now):
        """负责机看不到、别的飞机正看到同一航迹: 交给更近的空闲观察者, 或改用其观测引导。

        观测只取 observer_detections (已通过身份/几何关联)。红人身份有歧义时不动。
        返回 True 表示负责机已释放。
        """
        if tr.cls == 'red' and any(other is not tr and not self._reds_distinct(tr, other, now)
                                   for other in reds):
            return False
        owner = tr.assigned
        if now - tr.seen_by.get(owner, -1e9) <= self.handover_owner_age:
            # 负责机仍看到同色候选(门控外的新位置确认中), 交给原有重建逻辑。
            return False
        busy = {t.assigned for t in self.tracks.values() if t.assigned}
        busy.update(self.retiring)
        fresh = []
        for did, (det, stamp) in tr.observer_detections.items():
            p = self.drone_pos[did - 1] if 1 <= did <= self.drone_num else None
            if (did != owner and did not in self.retiring and p is not None and p[2] >= 1.5
                    and 0.0 <= now - stamp <= self.handover_seen):
                fresh.append((did, det, stamp, math.hypot(p[0] - det.x, p[1] - det.y)))
        if not fresh:
            return False
        p_owner = self.drone_pos[owner - 1]
        latest = max(fresh, key=lambda f: f[2])[1]
        d_owner = (math.hypot(p_owner[0] - latest.x, p_owner[1] - latest.y)
                   if p_owner is not None else float('inf'))
        closer = [f for f in fresh if f[0] not in busy
                  and self.reject.get((f[0], tr.slot), 0.0) <= now
                  and f[3] + self.handover_margin < d_owner]
        if closer:
            did, _, _, d_obs = min(closer, key=lambda f: f[3])
            rospy.logwarn('[fusion] %s 的%d号负责机 %.1fs 看不到, %d号机正看到且更近 '
                          '(%.1fm vs %.1fm), 交给它', tr.slot, owner, owner_age,
                          did, d_obs, d_owner)
            self.reject[(owner, tr.slot)] = now + 2.0
            self._release(tr, now)
            return True
        if tr.last_meas is not None and now - tr.last_meas <= self.handover_owner_age:
            return False   # 当前导航来源仍在更新
        newer = [f for f in fresh if tr.last_meas is None or f[2] > tr.last_meas]
        if not newer:
            return False
        did = max(newer, key=lambda f: f[2])[0]
        self._adopt_observer(tr, did)
        tr.shared_source = did
        tr.reacq_arrived_since = None
        rospy.loginfo('[fusion] %s 的%d号负责机 %.1fs 看不到, 改用%d号机的新鲜观测引导重捕',
                      tr.slot, owner, owner_age, did)
        return False

    def _observer_free_in(self, did, now):
        """忙碌机还要多久空闲。只有正在发布(锁定)的任务可预估, 否则 None。"""
        if did in self.retiring:
            return self.observer_wait_retire
        cur = next((t for t in self.tracks.values()
                    if t.assigned == did and t.status != ST_ELIMINATED), None)
        if cur is None:
            return None
        lock = cur.lock_seconds(now)
        if lock > 0.0:
            # 裁判约在我们发布满 elim_full_run(+外推尾巴) 时删人, 再等短确认窗口和退出回执。
            return (max(0.0, self.elim_full_run + self.pub_max_age - lock)
                    + self.elim_confirm_gap_full + self.observer_wait_retire)
        if cur.last_pub is not None and cur.pub_run_start is not None \
                and cur.last_pub - cur.pub_run_start >= self.elim_full_run:
            # 发满一整轮后人已消失、停发: 正在等消除确认 (lock_seconds 已归零)。
            observation_age = now - self._last_observation_time(cur)
            return (max(0.0, self.elim_confirm_gap_full - observation_age)
                    + self.observer_wait_retire)
        return None

    def _approach_eta(self, did, tr, overhead):
        p = self.drone_pos[did - 1]
        d = math.hypot(p[0] - tr.x[0], p[1] - tr.x[1])
        return overhead + max(0.0, d - self.observer_wait_near) / self.observer_wait_speed

    def _wait_for_busy_observer(self, tr, free, busy, now):
        """返回值得等待的忙碌发现机编号; 0 = 不等, 按原逻辑共享派给空闲机。"""
        if not self.observer_wait_enable or not free:
            return 0
        best_free = min(self._approach_eta(d, tr, self.observer_wait_free_overhead)
                        for d in free)
        best, best_eta = 0, float('inf')
        for did, (_, stamp) in tr.observer_detections.items():
            if (did not in busy or not 1 <= did <= self.drone_num
                    or self.drone_pos[did - 1] is None
                    or not 0.0 <= now - stamp <= self.observer_wait_seen
                    or self.reject.get((did, tr.slot), 0.0) > now):
                continue
            free_in = self._observer_free_in(did, now)
            if free_in is None or free_in > self.observer_wait_max:
                continue
            eta = self._approach_eta(did, tr, free_in)
            if eta < best_eta:
                best, best_eta = did, eta
        if best and best_eta + self.observer_wait_margin < best_free:
            if self.observer_waiting.get(tr.slot) != best:
                rospy.loginfo('[fusion] %s 等 %d 号机忙完接手 (预计 %.1fs 到位, '
                              '最近空闲机 %.1fs)', tr.slot, best, best_eta, best_free)
            self.observer_waiting[tr.slot] = best
            return best
        self.observer_waiting.pop(tr.slot, None)
        return 0

    def _drain_releases(self, now):
        for did, (_, released) in list(self.retiring.items()):
            token, stamp = self.active_tracking.get(did, ('', -1e9))
            # 回执证明旧机已退出；若节点断线，等待其 1s 指令租约过期后再改派。
            # 三秒留出飞控释放滞回与消息排队余量，仍有活动回执则继续等。
            if ((stamp + 1e-6 >= released and not token)
                    or (now - released > 3.0 and now - stamp > 3.0)):
                del self.retiring[did]

    # 检测融合
    def det_cb(self, msg):
        now = rospy.get_time()
        if self.t0 is None:
            self.t0 = now
        with self.lock:
            before = {slot: tr.last_det for slot, tr in self.tracks.items()}
            for tr in self.tracks.values():
                tr.pending_view = None
            # 按颜色分组：红色先做跨机去重/双目标关联，其余喂对应单色航迹。
            reds = []
            # DetectionArray 由多机批次组成，数组顺序不是严格时间顺序。KF 只能
            # 沿量测时间向前推进，否则后到的旧图像会被更新到未来状态。
            for d in sorted(msg.dets, key=lambda dd: dd.header.stamp.to_sec()):
                if d.cls_id < 0 or d.cls_id >= len(CLS_NAMES):
                    continue
                cls = CLS_NAMES[d.cls_id]
                t = d.header.stamp.to_sec()
                if (not 1 <= int(d.drone_id) <= self.drone_num
                        or not 0.0 <= now - t <= self.owner_seen_timeout
                        or not all(math.isfinite(v) for v in (
                            d.x, d.y, d.sigma_along, d.sigma_cross, d.bearing, d.slant))):
                    continue
                # 普通颜色若已被内部标成 ELIMINATED，视觉证据优先，必须送进
                # 持续复活确认；否则消除点黑名单会让复活代码永远收不到输入。
                normal_revival = (cls != 'red'
                                   and self.tracks[cls].status == ST_ELIMINATED)
                known_red = (cls == 'red' and any(
                    tr.inited and tr.status != ST_ELIMINATED
                    and math.isfinite(self._red_cost(tr, d, t))
                    for tr in (self.tracks['red1'], self.tracks['red2'])))
                if (self.in_blacklist(d.x, d.y, now, cls)
                        and not normal_revival and not known_red):
                    continue
                if cls == 'red':
                    reds.append((d, t))
                else:
                    self._feed_with_view(self.tracks[cls], d, t, now)
            if reds:
                self.feed_reds(reds, now)
            # Complete dual-red association first: a later ambiguous box in
            # this batch must suppress feedback instead of steering to a guess.
            self._publish_tracking_views(now)
            updated = set()
            update_stamp = rospy.get_time()
            for slot, tr in self.tracks.items():
                if tr.last_det is not before[slot] and tr.last_det is not None:
                    tr.last_vision_receive_stamp = now
                    tr.last_measurement_update_stamp = update_stamp
                    if tr.last_det.method & M_DEPTH:
                        updated.add(slot)
        # Finish the complete batch (especially dual-red association) before
        # publishing. Do not re-enter the non-reentrant track lock here.
        if self.publish_on_measurement and updated:
            self.publish_cb(None, publish_slots=updated)

    def _feed_with_view(self, tr, det, t_meas, now):
        """Attach feedback only to the observation actually accepted by feed()."""
        previous_det = tr.last_det
        self.feed(tr, det, t_meas, now)
        accepted = tr.last_det
        if (accepted is not previous_det and accepted is not None
                and tr.last_meas == t_meas
                and tr.last_meas_drone == int(det.drone_id)):
            self._remember_observer(tr, accepted, t_meas)
            if tr.assigned == int(det.drone_id):
                tr.owner_observation_assignment = tr.assignment
                tr.blind_acquire = False
                tr.reacq_arrived_since = None
        if (accepted is not previous_det and accepted is not None
                and tr.last_meas == t_meas
                and tr.last_meas_drone == int(det.drone_id)
                and tr.assigned == int(det.drone_id) and tr.assignment
                and getattr(accepted, 'image_bearing_valid', False)):
            tr.pending_view = (accepted, tr.assignment)

    def _publish_tracking_views(self, now):
        """Send fresh, owner-associated image feedback without the assign timer."""
        for tr in self.tracks.values():
            pending, tr.pending_view = tr.pending_view, None
            if pending is None:
                continue
            det, assignment = pending
            stamp = det.header.stamp.to_sec()
            did = int(det.drone_id)
            if (not tr.inited or tr.status == ST_ELIMINATED
                    or assignment != tr.assignment or did != tr.assigned
                    or not 1 <= did <= self.drone_num
                    or det.header.frame_id != 'world'
                    or not 0.0 <= now - stamp <= self.owner_seen_timeout
                    or tr.identity_blocked_until > now
                    or not all(math.isfinite(v) for v in (
                        det.image_bearing, det.image_error_x, det.conf))
                    or not 0.0 <= det.conf <= 1.0
                    or (tr.view_published_assignment == assignment
                        and tr.view_published_stamp is not None
                        and stamp <= tr.view_published_stamp)):
                continue
            view = TrackingView()
            view.header.stamp = det.header.stamp
            view.header.frame_id = 'world'
            view.assignment = assignment
            view.drone_id = did
            view.bearing = float(det.image_bearing)
            view.horizontal_error_px = float(det.image_error_x)
            view.confidence = float(det.conf)
            self.view_pubs[did - 1].publish(view)
            tr.view_published_assignment = assignment
            tr.view_published_stamp = stamp

    def in_blacklist(self, x, y, now, cls=None):
        """按颜色隔离禁区；红色的尾帧禁区不能误伤附近的普通行人。"""
        for item in self.blacklist:
            bx, by, r, exp = item[:4]
            scope = item[4] if len(item) > 4 else None
            if scope is not None and cls is not None and scope != cls:
                continue
            if now < exp and (x - bx) ** 2 + (y - by) ** 2 < r * r:
                return True
        return False

    def _mark_seen(self, tr, drone_id, now):
        """记录观察者；仅上一任负责机自身的观测刷新 last_assigned_seen。"""
        self.last_seen_by[tr.slot] = drone_id
        tr.seen_by[int(drone_id)] = max(now, tr.seen_by.get(int(drone_id), -1e9))
        if drone_id == tr.last_assigned:
            tr.last_assigned_seen = max(
                now, tr.last_assigned_seen if tr.last_assigned_seen is not None else -1e9)

    def _remember_observer(self, tr, det, stamp):
        """保存每机已关联观测，不借此更新负责机的有效量测或控制状态。"""
        did = int(det.drone_id)
        previous = tr.observer_detections.get(did)
        if previous is None or stamp > previous[1]:
            tr.observer_detections[did] = (copy.copy(det), stamp)

    def _observer_matches(self, tr, det, stamp):
        """旁观者须解释当前航迹，原始同色框不能直接成为接管候选。"""
        if not tr.inited or not math.isfinite(self._red_cost(tr, det, stamp)):
            return False
        if not tr.bearing_only_nav and det.method & M_DEPTH:
            xy, _ = self._red_state(tr, stamp)
            if math.hypot(det.x - xy[0], det.y - xy[1]) > self.gate_abs:
                return False
        return True

    @staticmethod
    def _last_observation_time(tr):
        """主轨停更不代表人消失；旁观者可证明存活，但不能替负责机续租。"""
        stamps = [stamp for _, stamp in tr.observer_detections.values()]
        if tr.last_meas is not None:
            stamps.append(tr.last_meas)
        return max(stamps) if stamps else -1e9

    def _adopt_observer(self, tr, did):
        """认领新来源时采用其真实观测；滤波时间轴按新任务重新建立。"""
        det, stamp = tr.observer_detections[did]
        if tr.last_meas_drone == did and tr.last_meas == stamp:
            return
        # 新任务的相机可比上一来源更慢，不能把合法缓存当成旧任务的乱序帧。
        tr.last_pred = None
        if det.method & M_DEPTH:
            tr.init_from(det, stamp)
        else:
            self._enter_bearing_nav(tr, det, stamp)
        self._remember_observer(tr, det, stamp)
        tr.gate_fail = 0
        tr.last_gate_fail = None
        if tr.cls == 'red':
            tr.identity_det, tr.identity_t = copy.copy(det), stamp

    def _bearing_nav_det(self, det):
        """复制单目检测供导航；bearing_only_nav 阻止其进入裁判发布。
        恢复深度后重建可信坐标航迹。"""
        return copy.copy(det)

    def _enter_bearing_nav(self, tr, det, t):
        nav_det = self._bearing_nav_det(det)
        if nav_det is None:
            return False
        if not tr.init_from(nav_det, t):
            return False
        # 单目导航目标不走普通 KF，避免把它的距离噪声积累进裁判坐标航迹。
        tr.x[2:] = 0.0
        tr.P = np.diag([0.04, 0.04, 1.0, 1.0])
        tr.bearing_only_nav = True
        tr.bearing_nav_angle = float(det.bearing)
        tr.bearing_nav_drone = int(det.drone_id)
        return True

    def _update_bearing_nav(self, tr, det, t):
        """只用负责机的最新单目导航目标直接刷新，不用常速 KF。"""
        if not tr.accepts_time(t):
            return False
        did = int(det.drone_id)
        source = tr.shared_source or int(tr.assigned)
        if source and did != source:
            return False
        if did != tr.bearing_nav_drone:
            # 目标重新分配给另一架机后，以新负责机的第一帧建立方位基准。
            tr.bearing_nav_drone = did
            tr.bearing_nav_angle = None
        nav_det = self._bearing_nav_det(det)
        if nav_det is None:
            return False
        tr.x = np.array([nav_det.x, nav_det.y, 0.0, 0.0])
        tr.P = np.diag([0.04, 0.04, 1.0, 1.0])
        tr.inited = True
        tr.last_pred = t
        tr.last_meas = t
        tr.status = ST_TRACKED
        tr.last_slant = float(det.slant)
        tr.last_meas_drone = did
        tr.accepted_seen_by[did] = max(
            t, tr.accepted_seen_by.get(did, -1e9))
        tr.last_residual = 0.0
        tr.last_innovation = np.zeros(2)
        tr.last_det = copy.copy(det)
        tr.cls_seen[int(det.cls_id)] = t
        tr.bearing_nav_angle = float(det.bearing)
        tr.clear_reinit_candidate()
        return True

    def feed(self, tr, det, t_meas, now):
        # 每机的发现时间独立记录，不能让另一台快相机的主轨时间饿死慢相机。
        self._mark_seen(tr, det.drone_id, t_meas)
        if tr.shared_source and int(det.drone_id) == tr.assigned:
            # 接手机的第一份有效观测建立自己的时间轴。相机时延可能更大，
            # 不得被发现机的较新时间戳饿死；仍须先通过目标身份/几何关联。
            if not self._observer_matches(tr, det, t_meas):
                return
            identity_epoch = tr.identity_epoch
            old_xy, _ = self._red_state(tr, t_meas)
            same_position = math.hypot(det.x - old_xy[0], det.y - old_xy[1]) <= self.gate_abs
            tr.last_pred = None
            if det.method & M_DEPTH:
                tr.init_from(det, t_meas)
            else:
                self._enter_bearing_nav(tr, det, t_meas)
            tr.shared_source = 0
            self._remember_observer(tr, det, t_meas)
            if tr.cls == 'red':
                tr.identity_det, tr.identity_t = copy.copy(det), t_meas
                # 合法来源交接没有搬动目标，保留仍新鲜的同帧双红证据。
                # 真正重定位仍使用 init_from 的新 epoch，旧身份依据不能续用。
                if same_position:
                    tr.identity_epoch = identity_epoch
            # 飞控以凭证区分时间轴，允许切换到较慢相机的真实观测时间。
            tr.assignment = '%s/%s' % (tr.slot, uuid.uuid4().hex)
            tr.assigned_since = now
            tr.last_tracking_ack = None
            rospy.loginfo('[fusion] %s 的%d号接手机已获得本机观测，结束共享坐标接近',
                          tr.slot, tr.assigned)
            return
        if (tr.assigned and int(det.drone_id) != tr.assigned
                and int(det.drone_id) != tr.shared_source):
            if self._observer_matches(tr, det, t_meas):
                self._remember_observer(tr, det, t_meas)
            return
        # 在任何门控、导航切换和重初始化之前检查时间。只在 update()
        # 检查太晚：旧帧仍可能清候选、累计 gate_fail 或用深度重置导航航迹。
        if not tr.accepts_time(t_meas):
            # 未认领时也可能由一台忙碌快相机刷新主轨。空闲慢相机只要
            # 几何关联成立，就应保留自己的候选；它不能回拨主滤波时间。
            if not tr.assigned and self._observer_matches(tr, det, t_meas):
                self._remember_observer(tr, det, t_meas)
            return
        has_depth = bool(det.method & M_DEPTH)
        if tr.status == ST_ELIMINATED:
            # 复活期间仍更新观察者，避免按旧任务继续分配。
            confirm_det = det if has_depth else self._bearing_nav_det(det)
            if tr.cls != 'red':
                # 普通色一类只有一个真实目标/一个槽。由同一架机连续、空间一致
                # 的观测证明它仍存在；红色不走这里，继续使用下面原有双目标逻辑。
                ready = (confirm_det is not None
                         and tr.reinit_candidate_ready(
                             confirm_det, t_meas, self.normal_revive_updates,
                             self.normal_revive_sec, self.normal_revive_dist,
                             self.normal_revive_gap))
                if not ready:
                    rospy.logwarn_throttle(
                        1.0, '[fusion] %s 已标消除但视觉重新发现：复活确认中 %d/%d，'
                             '尚不分配/不发裁判', tr.slot, tr.reinit_n,
                        self.normal_revive_updates)
                    return
                n, span = tr.reinit_n, t_meas - tr.reinit_t0
                tr.judge_rejects = 0
                tr.judge_check_at = None
                tr.eliminated_at = None
                tr.pending_det = None
                if has_depth:
                    tr.init_from(det, t_meas)
                else:
                    self._enter_bearing_nav(tr, det, t_meas)
                rospy.logwarn('[fusion] %s 在内部消除后被视觉持续确认 %.1fs/%d帧 —— '
                              '撤销 ELIMINATED，重新参与一人一机分配',
                              tr.slot, span, n)
                return

            # 红色复活保持原逻辑：落在已消除红槽附近的深度观测须两帧确认，
            # 不改变双红关联及每条航迹独立跟踪、发布的约定。
            if (confirm_det is not None and tr.pending_det is not None
                    and (t_meas - tr.pending_t) <= self.init_gap
                    and int(det.drone_id) == int(tr.pending_det.drone_id)
                    and math.hypot(confirm_det.x - tr.pending_det.x,
                                   confirm_det.y - tr.pending_det.y) < self.init_dist):
                rospy.logwarn('[fusion] %s 消除后又出现确认的检测 —— 当初是误判, '
                              '复活重试', tr.slot)
                tr.status = ST_TRACKED
                tr.judge_rejects = 0
                tr.judge_check_at = None
                if has_depth:
                    tr.init_from(det, t_meas)
                else:
                    self._enter_bearing_nav(tr, det, t_meas)
                tr.pending_det = None
            else:
                tr.pending_det, tr.pending_t = confirm_det, t_meas
            return
        tr.cls_seen[int(det.cls_id)] = now
        # 方位导航仅由负责机更新；只有深度量测才能更新真实坐标 KF。
        if tr.inited and tr.bearing_only_nav:
            if has_depth:
                rospy.loginfo('[fusion] %s 方位引导结束：深度恢复，切回纯深度航迹',
                              tr.slot)
                tr.init_from(det, t_meas)
                tr.last_gate_fail = now
                return
            self._update_bearing_nav(tr, det, t_meas)
            return

        if not tr.inited:
            # 单帧即建航迹(用户要求"一看到人就飞过去"): 不再等两帧确认。
            # 误检兜底靠门控关联(gate_chi2/gate_abs) + 谁看到谁跟 + 连续门控失败重初始化。
            if not has_depth:
                if self._enter_bearing_nav(tr, det, t_meas):
                    rospy.loginfo('[fusion] %s 首帧为纯单目(估计斜距 %.1fm)，'
                                  '进入方位引导；只导航、不发裁判',
                                  tr.slot, det.slant)
                # 本机 odom 尚未到时不能构造滚动点；等待下一帧，绝不能退回
                # 到“直接拿单目 x/y 建真实航迹”。
                return
            tr.init_from(det, t_meas)
            return

        # 所有纯单目距离都无条件禁止进入真实坐标 KF。最后一条深度航迹仍在安全
        # 外推窗内时保持它；过窗后立即转入滚动方位引导，避免等待确认期间悬停。
        if not has_depth:
            age = 1e9 if tr.last_meas is None else now - tr.last_meas
            if age <= self.mono_nav_switch_age:
                rospy.logwarn_throttle(
                    2.0, '[fusion] %s 忽略单目距离 %.1fm；保持最后的纯深度 KF',
                    tr.slot, det.slant)
                return
            if self._enter_bearing_nav(tr, det, t_meas):
                tr.gate_fail = 0
                tr.last_gate_fail = now
                rospy.logwarn('[fusion] %s 深度已中断 %.1fs，转入单目方位引导；'
                              '只导航、不发裁判', tr.slot, age)
            return

        # 门控仅查询预测副本。拒绝量测不应推进主时间轴，否则其他相机
        # 随后抵达的有效观测会被误当成旧帧丢掉。
        x_gate, _ = tr.prediction(t_meas)
        m2 = tr.mahalanobis2(det, t_meas)
        dist = math.hypot(det.x - x_gate[0], det.y - x_gate[1])
        if m2 > self.gate_chi2 or dist > self.gate_abs:
            tr.gate_fail += 1
            # 拒绝单次量测不阻止已有 KF 外推；确认重建后才短暂抑制发布。
            age = 1e9 if tr.last_meas is None else now - tr.last_meas
            if tr.gate_fail >= 2 and age > self.gate_reinit_stale:
                # 只把当前坏点记成“新位置候选”，不立刻搬航迹。候选需要同机、
                # 相邻帧间隔足够短且位置相互接近；任何不一致都会从头累计。
                ready = tr.reinit_candidate_ready(
                    det, t_meas, self.reinit_confirm_updates,
                    self.reinit_confirm_sec, self.reinit_confirm_dist,
                    self.reinit_confirm_gap)
                span = t_meas - tr.reinit_t0
                if not ready:
                    rospy.logwarn_throttle(
                        1.0, '[fusion] %s 新位置候选 %.2fm: 确认中 %d/%d, %.2f/%.2fs; '
                             '暂不重置/暂不把跳点发给裁判',
                        tr.slot, dist, tr.reinit_n, self.reinit_confirm_updates,
                        span, self.reinit_confirm_sec)
                    return
                rospy.logwarn('[fusion] %s 连续门控失败且 %.1fs 无有效更新, '
                              '新位置已连续确认 %.1fs/%d 帧, 重新初始化航迹 '
                              '(跳变 %.2fm)', tr.slot, age, span, tr.reinit_n, dist)
                tr.init_from(det, t_meas)
                tr.gate_fail = 0
                tr.last_gate_fail = now
            return
        tr.gate_fail = 0
        tr.clear_reinit_candidate()
        tr.update(det, t_meas)

    def _red_det_cov(self, det):
        """关联的不确定度独立于导航用 P；远距单目不能冒充精确坐标。"""
        along, cross = max(det.sigma_along, 0.1), max(det.sigma_cross, 0.1)
        if not (det.method & M_DEPTH):
            along = max(along, 0.25 * det.slant)
            cross = max(cross, 0.5 + 0.02 * det.slant)
        c, s = math.cos(det.bearing), math.sin(det.bearing)
        rot = np.array([[c, -s], [s, c]])
        return rot @ np.diag([along ** 2, cross ** 2]) @ rot.T

    def _red_state(self, tr, t):
        if tr.bearing_only_nav and tr.last_det is not None:
            d = tr.identity_det if tr.identity_det is not None else tr.last_det
            stamp = tr.identity_t if tr.identity_det is not None else tr.last_meas
            dt = min(max(t - stamp, 0.0), 2.0)
            return np.array([d.x, d.y]), self._red_det_cov(d) + np.eye(2) * dt ** 2
        # 身份记忆有时限，与 KF 协方差传播分别处理。
        identity_t = min(t, tr.last_meas + self.gate_reinit_stale)
        x, p = tr.prediction(identity_t)
        return x[:2], p[:2, :2]

    def _red_cost(self, tr, det, t):
        # 关联只读，不推进主轨。不同相机的图像延迟可以不同；真正更新的
        # 时间门在 feed()，旁观者记录则按自己的成像时刻保持单调。
        x, p = self._red_state(tr, t)
        dz = np.array([det.x, det.y]) - x
        cov = p + self._red_det_cov(det) + np.eye(2) * 0.04
        m2 = float(dz @ np.linalg.solve(cov, dz))
        return m2 if m2 <= self.gate_chi2 else float('inf')

    def _reds_distinct(self, a, b, now):
        xa, pa = self._red_state(a, now)
        xb, pb = self._red_state(b, now)
        dz = xa - xb
        epochs = (self.tracks['red1'].identity_epoch,
                  self.tracks['red2'].identity_epoch)
        coobserved = any(0.0 <= now - stamp <= self.owner_seen_timeout
                         and pair_epochs == epochs
                         for stamp, pair_epochs in self.red_pair_seen.values())
        return ((np.linalg.norm(dz) >= self.red_distinct_dist or coobserved)
                and float(dz @ np.linalg.solve(
                    pa + pb + np.eye(2) * 0.04, dz)) > self.gate_chi2)

    def _red_boxes_distinct(self, a, b):
        """同帧两份可靠深度可证明两个人，重叠的重复框不能提供这个证据。"""
        if not (a.method & M_DEPTH and b.method & M_DEPTH):
            return False
        dz = np.array([a.x - b.x, a.y - b.y])
        cov = self._red_det_cov(a) + self._red_det_cov(b) + np.eye(2) * 0.04
        if float(dz @ np.linalg.solve(cov, dz)) <= self.gate_chi2:
            return False
        boxes = []
        for d in (a, b):
            values = (d.u, d.box_w, d.v_top, d.v_bot)
            if (not all(math.isfinite(v) for v in values)
                    or d.box_w <= 0 or d.v_bot <= d.v_top):
                # 旧检测/离线输入没有框几何时仍采用原有的远距去重门槛。
                return np.linalg.norm(dz) >= self.red_distinct_dist
            boxes.append((d.u - d.box_w / 2., d.v_top,
                          d.u + d.box_w / 2., d.v_bot))
        aa, bb = boxes
        overlap = (max(0., min(aa[2], bb[2]) - max(aa[0], bb[0]))
                   * max(0., min(aa[3], bb[3]) - max(aa[1], bb[1])))
        area_a = (aa[2] - aa[0]) * (aa[3] - aa[1])
        area_b = (bb[2] - bb[0]) * (bb[3] - bb[1])
        return overlap / (area_a + area_b - overlap) < 0.5

    def _feed_red_match(self, tr, det, tm, now):
        # feed() 只接收已经完成红人关联的观测，不能把另一个红人当作跳点重置。
        previous_det = tr.last_det
        previous_observer = tr.observer_detections.get(int(det.drone_id))
        self._feed_with_view(tr, det, tm, now)
        if (tr.last_det is not previous_det and tr.last_meas == tm
                and tr.last_meas_drone == int(det.drone_id)):
            tr.identity_det, tr.identity_t = det, tm
        observer = tr.observer_detections.get(int(det.drone_id))
        return (observer is not None and observer[1] == tm
                and (observer is not previous_observer or tr.last_det is not previous_det))

    def feed_reds(self, reds, now):
        """视觉关联两条红航迹。跨机重复观测合并，身份不确定时不新建第二份。"""
        red_tracks = [self.tracks['red1'], self.tracks['red2']]
        # 一幅图中一条航迹最多匹配一个框；不同相机可以观测同一条航迹。
        frames = {}
        for d, tm in reds:
            if 0.0 <= now - tm <= self.owner_seen_timeout:
                frames.setdefault((tm, int(d.drone_id)), []).append(d)
        for (tm, _), dets in sorted(frames.items()):
            live = [tr for tr in red_tracks if tr.inited and tr.status != ST_ELIMINATED]
            used = set()
            matched = {}
            if live:
                costs = np.array([[self._red_cost(tr, d, tm) for d in dets]
                                  for tr in live])
                # 两条航迹都能解释同一个框时，无视觉证据可确定身份；不硬猜。
                ambiguous = {j for j in range(len(dets))
                             if np.count_nonzero(np.isfinite(costs[:, j])) > 1}
                for j in ambiguous:
                    used.add(j)
                    for i, tr in enumerate(live):
                        if np.isfinite(costs[i, j]):
                            tr.identity_blocked_until = now + 0.5
                    costs[:, j] = np.inf
                rows, cols = hungarian(costs)
                for i, j in zip(rows, cols):
                    if np.isfinite(costs[i, j]):
                        if self._feed_red_match(live[i], dets[j], tm, now):
                            matched[live[i].slot] = dets[j]
                        used.add(int(j))

            for j, d in enumerate(dets):
                if j in used:
                    continue
                live = [tr for tr in red_tracks
                        if tr.inited and tr.status != ST_ELIMINATED]
                # 包括同拍刚初始化的航迹：不同相机看到同一个人不能再造第二条。
                if any(math.isfinite(self._red_cost(tr, d, tm)) for tr in live):
                    continue
                # 同一帧已接受的另一框 + 独立深度位置可以在 3m 内建立第二人。
                # 跨相机/单目/重复框仍须走原来的空间去重和连续确认。
                if any(math.hypot(d.x - self._red_state(tr, tm)[0][0],
                                  d.y - self._red_state(tr, tm)[0][1])
                       < self.red_distinct_dist
                       and not (tr.slot in matched
                                and self._red_boxes_distinct(d, matched[tr.slot]))
                       for tr in live):
                    continue

                # 已消除位置持续再现才复活；另一个正在跟踪的红人先在上面关联，
                # 因而不会把另一条活航迹误认成已消除的人。
                dead = [tr for tr in red_tracks if tr.status == ST_ELIMINATED
                        and math.hypot(d.x - tr.x[0], d.y - tr.x[1]) < self.revive_dist]
                if dead and d.method & M_DEPTH:
                    self._feed_red_match(dead[0], d, tm, now)
                    continue
                # 重建失联红航迹不能抢占另一条活跃或正在确认消除的红航迹。
                free = next((tr for tr in live
                             if not tr.assigned and tr.last_meas is not None
                             and now - tr.last_meas > self.gate_reinit_stale
                             and (tr.pub_run_start is None or (
                                 tr.last_pub is not None
                                 and now - tr.last_pub > self.elim_confirm_gap))), None)
                if free is None:
                    free = next((tr for tr in red_tracks
                                 if not tr.inited and tr.status != ST_ELIMINATED), None)
                if free is None:
                    free = self._sole_stale_red(red_tracks, now)
                if free is None:
                    continue
                if live and not free.reinit_candidate_ready(
                        d, tm, self.reinit_confirm_updates if free.inited else 3,
                        self.reinit_confirm_sec if free.inited else self.red_confirm_sec,
                        max(self.reinit_confirm_dist,
                            min(3.0, 0.1 * d.slant)), self.init_gap):
                    continue
                if free.inited:
                    if free.assigned and free.assigned != int(d.drone_id):
                        # 负责机正飞向旧保存点却看不到人; 交还分配, 由真正看到的
                        # 飞机按"谁看到谁跟"接手 (非负责机的观测不能更新已认领航迹)。
                        rospy.logwarn('[fusion] %s 已是唯一红人, %d 号机在离旧保存点 '
                                      '%.1fm 处持续看到, 撤下久未观测的 %d 号机重建航迹',
                                      free.slot, int(d.drone_id),
                                      math.hypot(d.x - free.x[0], d.y - free.x[1]),
                                      free.assigned)
                        self._release(free, now)
                    free.inited = False
                    free.seen_by.clear()
                if self._feed_red_match(free, d, tm, now):
                    matched[free.slot] = d

            if len(matched) == 2:
                if self._red_boxes_distinct(matched['red1'], matched['red2']):
                    self.red_pair_seen[int(dets[0].drone_id)] = (
                        tm, tuple(tr.identity_epoch for tr in red_tracks))
                elif all(d.method & M_DEPTH for d in matched.values()):
                    # 重复框/无法区分的深度不能借上一帧的双人证据继续上报。
                    for tr in red_tracks:
                        tr.identity_blocked_until = now + 0.5

    def _sole_stale_red(self, red_tracks, now):
        """另一红人已确认消除时，允许连续且位置一致的新观测重建失联红航迹。
        仍须通过 reinit_candidate_ready 多帧确认。"""
        if not self.red_sole_takeover:
            return None
        live = [tr for tr in red_tracks if tr.inited and tr.status != ST_ELIMINATED]
        if len(live) != 1 or not any(tr.status == ST_ELIMINATED for tr in red_tracks):
            return None
        tr = live[0]
        if tr.last_meas is None or now - tr.last_meas <= self.gate_reinit_stale:
            return None
        if tr.pub_run_start is not None and (
                tr.last_pub is None or now - tr.last_pub <= self.elim_confirm_gap):
            return None
        return tr

    def _judge_authorized(self, tr, now):
        """红色按可靠定位上报；飞行认领/回执只决定飞机追哪个人。"""
        if tr.cls == 'red':
            other = self.tracks['red2' if tr.slot == 'red1' else 'red1']
            return (tr.inited and tr.status != ST_ELIMINATED
                    and now >= tr.identity_blocked_until
                    and now >= tr.report_blocked_until
                    and tr.last_det is not None and tr.last_det.method & M_DEPTH
                    and 0.0 <= now - tr.accepted_seen_by.get(
                        tr.last_meas_drone, -1e9) <= self.owner_seen_timeout
                    and (not other.inited or other.status == ST_ELIMINATED
                         or self._reds_distinct(tr, other, now)))
        return self._tracking_ready(tr, now)

    @staticmethod
    def _judge_pub_slots(slot):
        """视觉航迹固定发布自己的入口；红槽不绑定官方 actor 编号。"""
        return (slot,)

    def _publish_diagnostics(self, tr, pub_slots, x_pub, P_pub, now, trigger='timer',
                             prediction=None):
        """记录真正采用的观测和本次发布，不再猜最近的同色检测来源。"""
        det = tr.last_det
        if det is None:
            return
        lag = float(getattr(det, 'pipeline_lag', 0.0))
        data = dict(
            slot=tr.slot, pub_slots=list(pub_slots), state_stamp=float(now),
            measurement_stamp=float(tr.last_meas), depth_stamp=tr.last_depth_t,
            source_drone=int(tr.last_meas_drone), assignment=tr.assignment,
            meas_xy=[float(det.x), float(det.y)],
            x=float(x_pub[0] + self.probe_dx), y=float(x_pub[1] + self.probe_dy),
            vx=float(x_pub[2]), vy=float(x_pub[3]), cov=P_pub[:2, :2].tolist(),
            innovation=tr.last_innovation.tolist(), bearing=float(det.bearing),
            method=int(det.method), pipeline_lag=lag if math.isfinite(lag) else None,
            vision_receive_stamp=tr.last_vision_receive_stamp,
            measurement_update_stamp=tr.last_measurement_update_stamp,
            publication_trigger=trigger, publication_prediction=prediction)
        self.diagnostics_pub.publish(String(data=json.dumps(
            data, allow_nan=False, separators=(',', ':'))))

    # 发布 (核心)
    def publish_cb(self, _, publish_slots=None):
        now = rospy.get_time()
        with self.lock:
            n_elim = 0
            # 红色独立检查身份与定位；其他颜色还检查实际跟踪回执。
            for slot in ALL_SLOTS:
                tr = self.tracks[slot]
                if tr.status == ST_ELIMINATED:
                    n_elim += 1
                    continue
                if not tr.inited:
                    continue
                meas_age = (now - tr.last_meas) if tr.last_meas else 1e9
                observation_age = now - self._last_observation_time(tr)
                # 每条航迹独立生成无副作用的发布快照。
                x_pub, P_pub = tr.prediction(now)
                pub_pos_std = float(math.sqrt(max(P_pub[0, 0] + P_pub[1, 1], 0.0)))

                # 足够连续发布后再确认消失，作为内部消除推断；第二红人允许无单独认领。
                if meas_age > 1.5:
                    run_len = 0.0
                    if tr.last_pub is not None and tr.pub_run_start is not None:
                        run_len = tr.last_pub - tr.pub_run_start
                    confirm_gap = (self.elim_confirm_gap_full
                                   if run_len >= self.elim_full_run
                                   else self.elim_confirm_gap)
                    if (tr.last_pub is not None
                            and run_len >= 13.0
                            and (tr.assigned or tr.cls == 'red')
                            and abs(tr.last_pub - tr.last_meas) <= self.pub_max_age + 0.2
                            and observation_age > confirm_gap):
                        tr.status = ST_ELIMINATED
                        tr.eliminated_at = now
                        tr.clear_reinit_candidate()
                        tr.pending_det = None
                        tr.pending_t = None
                        self._release(tr, now)
                        # 消除位置黑名单仅用于红色；普通颜色可通过连续观测复活。
                        if tr.cls == 'red':
                            self.blacklist.append((tr.x[0], tr.x[1],
                                                   self.elim_radius,
                                                   now + self.elim_blacklist_sec,
                                                   tr.cls))
                        n_elim += 1
                        rospy.logwarn('[fusion] %s 连续发布 %.1fs 后消失, 确认消除 '
                                      '(t=%.1fs, 裁判曾否决 %d 次)',
                                      slot, run_len, now - (self.t0 or now),
                                      tr.judge_rejects)
                        continue
                    if tr.status == ST_TRACKED:
                        tr.status = ST_COASTING
                    continue

                # 发布轮次和宽限结束后仍新鲜可见，重置影子计数并重试。
                if (tr.judge_check_at is not None
                        and now >= tr.judge_check_at
                        and meas_age <= self.max_meas_age):
                    tr.judge_check_at = None
                    tr.pub_run_start = None
                    tr.judge_rejects += 1
                    rospy.logwarn('[fusion] %s 已连续发布 %.1fs，目标仍被视觉检测到 '
                                  '(第 %d 次未通过) —— 推断本轮未消除；可能存在超差'
                                  '(>1m)或断流(>=1s)；本航迹影子计数清零重试',
                                  slot, self.lock_need, tr.judge_rejects)
                    if tr.judge_rejects == 2 and tr.x[0] ** 2 + tr.x[1] ** 2 < 1.0:
                        rospy.logerr(
                            '[fusion] %s 的估计位置就在世界原点附近 (%.2f, %.2f) —— '
                            'score_cal.py 会把"真值 (0,0)"当成查询失败'
                            '("actors_pos_tmp.x**2 + actors_pos_tmp.y**2 != 0"), '
                            '该目标在本局无法由发送端消除；本程序不会修改官方程序，'
                            '请保留日志并重开本局', slot, tr.x[0], tr.x[1])

                # 仅发布纯深度 KF，分别检查新鲜度、协方差与身份；允许帧间外推。
                # 单目观测只作导航，不更新发布坐标。
                depth_ok = (
                    not tr.bearing_only_nav
                    and tr.last_depth_t is not None
                    and now - tr.last_depth_t <= self.depth_hold)
                ok = (now - self.node_start >= self.warmup_sec
                      and 0.0 <= meas_age <= self.pub_max_age
                      and tr.pos_std <= self.max_pos_std
                      and pub_pos_std <= self.pub_max_pos_std
                      and depth_ok
                      and tr.last_residual <= self.max_pub_residual
                      # 仅重建后短暂静默，单次门控拒绝不使航迹停发。
                      and not (tr.last_gate_fail
                               and now - tr.last_gate_fail < 0.5))
                if not ok:
                    if tr.status == ST_TRACKED and meas_age > self.max_meas_age:
                        tr.status = ST_COASTING
                    continue

                # An observation event only sends tracks updated by that
                # batch. The timer continues to send every eligible track.
                if publish_slots is not None and slot not in publish_slots:
                    continue
                if tr.last_pub == now and tr.last_published_det is tr.last_det:
                    continue

                # 红色只按独立身份和可靠深度上报，未认领的第二人也能计时。
                # 其他颜色仍要求负责机已实际进入 TRACK 并回显本次凭证。
                if not self._judge_authorized(tr, now):
                    continue

                pub_slots = self._judge_pub_slots(slot)
                # 转弯均值守卫不反馈 KF，协方差、身份和深度门控仍独立生效。
                x_pub, P_pub, prediction = tr.publication_prediction(now)

                # 发布间隔断了 -> 裁判那边计数已清零, 我们的影子计数也要清零
                if (tr.last_pub is None
                        or now - tr.last_pub > self.pub_gap_reset
                        or tr.pub_run_start is None):
                    # 发布间隔诊断区分量测丢失与质量门控拒绝。
                    if tr.last_pub is not None:
                        gap = now - tr.last_pub
                        if gap > self.pub_gap_reset:
                            rospy.logwarn(
                                '[fusion] %s 发布断流 %.2fs -> 影子计数清零 '
                                '(量测年龄%.2fs 后验std%.2f 外推std%.2f 残差%.2f 深度%s)',
                                slot, gap, meas_age, tr.pos_std, pub_pos_std,
                                tr.last_residual, '有' if depth_ok else '无')
                    tr.pub_run_start = now
                m = ActorInfo()
                # cls 必须是裁判 actor_id_dict 的 key: red1/red2 都填 'red'
                m.cls = 'red' if slot.startswith('red') else slot
                m.x = float(x_pub[0] + self.probe_dx)
                m.y = float(x_pub[1] + self.probe_dy)
                # 每条红航迹只发自己的固定入口；两个回调均能按坐标匹配
                # 任一官方红人，因此无需猜 actor 编号，也不混发两个人的位置。
                for pub_slot in pub_slots:
                    self.actor_pubs[pub_slot].publish(m)
                self._publish_diagnostics(tr, pub_slots, x_pub, P_pub, now,
                                          'measurement' if publish_slots is not None else 'timer',
                                          prediction=prediction)
                tr.last_pub = now
                tr.last_published_det = tr.last_det

                # 影子计数满一轮 -> 设裁判响应检查点
                lock_s = tr.lock_seconds(now)
                if (lock_s >= self.lock_need
                        and (tr.judge_check_at is None
                             or now > tr.judge_check_at + 5.0)):
                    tr.judge_check_at = now + self.judge_grace

            all_eliminated = (n_elim >= len(ALL_SLOTS))
            # 第一帧复活候选也足以说明“内部已消除”仍存疑，立刻撤销完成计时；
            # 不必等候选累计完成才阻止飞机降落。
            revive_pending = any(
                tr.status == ST_ELIMINATED and (
                    (tr.cls != 'red' and tr.reinit_last_t is not None
                     and now - tr.reinit_last_t <= self.normal_revive_gap)
                    or (tr.cls == 'red' and tr.pending_t is not None
                        and now - tr.pending_t <= self.init_gap))
                for tr in self.tracks.values())
            if all_eliminated and not revive_pending:
                if self.all_eliminated_since is None:
                    self.all_eliminated_since = now
                    rospy.logwarn('[fusion] 6 个目标仅在内部状态上全部消除；继续巡航复核 '
                                  '%.1fs，期间视觉重现会自动复活，不立即通知降落',
                                  self.completion_confirm_sec)
                done = now - self.all_eliminated_since >= self.completion_confirm_sec
            else:
                self.all_eliminated_since = None
                done = False
            self.complete_pub.publish(Bool(data=done))

    # 分配
    def assign_cb(self, _):
        now = rospy.get_time()
        with self.lock:
            # 过期的拉黑
            self.reject = {k: v for k, v in self.reject.items() if v > now}
            self.blacklist = [b for b in self.blacklist if b[3] > now]
            self._drain_releases(now)
            # fusion 单独重启时，先让仍执行旧凭证的飞机退出，避免带着旧任务认新活。
            owners = {tr.assigned for tr in self.tracks.values() if tr.assigned}
            for did, (token, stamp) in self.active_tracking.items():
                if (token and now - stamp <= self.tracking_ack_timeout
                        and did not in owners and did not in self.retiring):
                    self.retiring[did] = ('', now)
                    self.mode_pubs[did - 1].publish(Int8(data=MODE_OFF))

            reds = [self.tracks[s] for s in ('red1', 'red2')
                    if self.tracks[s].inited and self.tracks[s].status != ST_ELIMINATED]
            if len(reds) == 2 and not self._reds_distinct(reds[0], reds[1], now):
                # 红色重叠时不新增第二份认领，保留已有负责机；认领先后不证明身份。
                keep = min(reds, key=lambda t: (
                    not bool(t.assigned),
                    t.assigned_since if t.assigned_since is not None else float('inf'),
                    t.slot))
                for tr in reds:
                    tr.identity_blocked_until = now + 0.5
                    if tr is not keep and tr.assigned:
                        self._release(tr, now)

            # 释放丢失/消除的目标
            for slot in ALL_SLOTS:
                tr = self.tracks[slot]
                if tr.status == ST_ELIMINATED or not tr.inited:
                    continue
                if now - self._last_observation_time(tr) > self.lost_timeout:
                    if tr.assigned:
                        rospy.loginfo('[fusion] %s 丢失 %.1fs, 释放 %d 号机',
                                      slot, now - tr.last_meas, tr.assigned)
                        self._release(tr, now)
                    continue
                if tr.assigned:
                    # 负责机新鲜度只由其有效量测续租；其他机或被拒量测不能续租。
                    # 新认领从本次任务开始计获取窗口，发布仍须真实新观测。
                    owner_epoch = max(tr.assigned_since, tr.accepted_seen_by.get(
                        tr.assigned, tr.assigned_since))
                    owner_age = now - owner_epoch
                    token, stamp = self.active_tracking.get(tr.assigned, ('', -1e9))
                    ack_lost = (token != tr.assignment
                                or now - stamp > self.tracking_ack_timeout)
                    # 发布回执超过 0.6s 即停发，但连续收不到有效回执满 5s 才释放。
                    # 不能拿“认领已超过 5s”作判断，否则跟踪越久越容易被单次延迟弹走。
                    ack_age = now - (tr.last_tracking_ack
                                     if tr.last_tracking_ack is not None
                                     else tr.assigned_since)
                    # 计满后视觉消失的短确认窗不能被“负责机丢失”先抢走。
                    run = (tr.last_pub - tr.pub_run_start
                           if tr.last_pub is not None and tr.pub_run_start is not None
                           else 0.0)
                    # 只有原负责机自身的有效量测可以维持消除确认保护。
                    confirming = (run >= 13.0
                                  and owner_age <= self.elim_confirm_gap + 1.0)
                    # 短时丢框维持原负责机做 REACQUIRE；持续没有本机有效更新就
                    # 释放。全局 lost_timeout 只保留航迹记忆，不能再永久续租旧机。
                    release_timeout = (self.shared_acquire_timeout if tr.blind_acquire
                                       else self.owner_release_timeout)
                    if (not confirming and owner_age > self.handover_owner_age
                            and self._help_blind_owner(tr, reds, owner_age, now)):
                        continue
                    if (not confirming and tr.blind_acquire
                            and self._blind_at_saved_point(tr, now)):
                        rospy.logwarn('[fusion] %s 的%d号机已到冻结保存点附近 %.1fs 仍未看到 '
                                      '(全场 %.1fs 无观测), 放回巡逻',
                                      slot, tr.assigned, now - tr.reacq_arrived_since,
                                      now - self._last_observation_time(tr))
                        self.reject[(tr.assigned, slot)] = now + 2.0
                        tr.queued_meas = tr.last_meas   # 空点不再排队派别的飞机
                        self._release(tr, now)
                        continue
                    if (not confirming and not tr.blind_acquire
                            and owner_age > self.reacquire_lo
                            and owner_age <= release_timeout):
                        rospy.logwarn_throttle(
                            2.0, '[fusion] %s 的%d号负责机 %.1fs 无有效目标更新：'
                                 '短暂重捕中，超过 %.1fs 将释放并允许其他飞机接管',
                            slot, tr.assigned, owner_age,
                            self.owner_release_timeout)
                    if not confirming and (owner_age > release_timeout or (
                            ack_lost and ack_age > self.tracking_start_timeout)):
                        rospy.logwarn('[fusion] %s 释放%d号机: 本机有效量测年龄%.1fs, '
                                      'TRACK回执%s(%.1fs无有效回执)；等退出回执后重新分配',
                                      slot, tr.assigned, owner_age,
                                      '失效' if ack_lost else '正常', ack_age)
                        self.reject[(tr.assigned, slot)] = now + 2.0
                        self._release(tr, now)
                        continue
                # 占用超时则释放并暂时拉黑，不标消除；黑名单过期后可重新认领。
                if (tr.assigned and tr.assigned_since
                        and now - tr.assigned_since > self.track_max_hold):
                    rospy.logwarn('[fusion] %s 占用 %d 号机 %.0fs 仍未消除 '
                                  '(裁判否决 %d 次), 释放并拉黑 %.0fs 后重试',
                                  slot, tr.assigned,
                                  now - tr.assigned_since, tr.judge_rejects,
                                  self.track_blacklist_sec)
                    self._release(tr, now)
                    tr.pub_run_start = None
                    tr.judge_check_at = None
                    tr.report_blocked_until = now + self.track_blacklist_sec
                    self.blacklist.append((tr.x[0], tr.x[1], self.elim_radius,
                                           now + self.track_blacklist_sec, tr.cls))

            # 优先由看到目标的空闲机认领，否则用新鲜共享坐标引导。
            # 另一红人确认消除后允许从有时限的旧点重捕，旧深度不能直接发布；分配遵守拒绝黑名单。
            busy = set(tr.assigned for tr in self.tracks.values() if tr.assigned)
            busy.update(self.retiring)
            retiring_slots = {slot for slot, _ in self.retiring.values()}
            for slot in ALL_SLOTS:
                tr = self.tracks[slot]
                if tr.assigned != 0 or tr.status == ST_ELIMINATED or not tr.inited:
                    continue
                if (slot in retiring_slots
                        or self.in_blacklist(tr.x[0], tr.x[1], now, tr.cls)):
                    continue
                if tr.cls == 'red' and any(
                        other is not tr and not self._reds_distinct(tr, other, now)
                        for other in reds):
                    continue
                # 目标已丢失(超 lost_timeout 没再检测到) -> 不认领。
                # 否则会"丢失->释放->又认领"死循环(last_seen_by 里还是旧记录)
                if now - self._last_observation_time(tr) > self.lost_timeout:
                    continue
                # 优先用所有新鲜观察者；最后看到的机可能已在跟别人。
                candidates = [did for did, (_, stamp) in tr.observer_detections.items()
                              if 1 <= did <= self.drone_num and did not in busy
                              and now - stamp <= self.owner_seen_timeout
                              and self.reject.get((did, slot), 0.0) <= now
                              and self.drone_pos[did - 1] is not None
                              and self.drone_pos[did - 1][2] >= 1.5]
                shared_source = 0
                if not candidates:
                    # 只共享真正关联/采用的观测，原始同色框或门控拒绝不能派任务。
                    sources = [(did, stamp) for did, (_, stamp)
                               in tr.observer_detections.items()
                               if did not in self.retiring
                               and 0.0 <= now - stamp <= self.owner_seen_timeout]
                    if sources:
                        candidates = [did for did in range(1, self.drone_num + 1)
                                      if did not in busy
                                      and self.reject.get((did, slot), 0.0) <= now
                                      and self.drone_pos[did - 1] is not None
                                      and self.drone_pos[did - 1][2] >= 1.5]
                        if self._wait_for_busy_observer(tr, candidates, busy, now):
                            # 发现机空闲后仍新鲜看见目标, 由上面的"谁看到谁跟"接手。
                            continue
                        if candidates:
                            shared_source = max(sources, key=lambda item: item[1])[0]
                queued_red = False
                if not candidates and tr.cls == 'red':
                    other_red = self.tracks['red2' if slot == 'red1' else 'red1']
                    meas_age = now - tr.last_meas if tr.last_meas is not None else 1e9
                    # 另一红人已确认消除时才允许旧点兜底，并受 lost_timeout 和排他条件限制。
                    # 同一次旧观测只安排一次重捕。
                    if (other_red.status == ST_ELIMINATED
                            and meas_age <= self.lost_timeout
                            and tr.queued_meas != tr.last_meas):
                        candidates = [did for did in range(1, self.drone_num + 1)
                                      if did not in busy
                                      and self.reject.get((did, slot), 0.0) <= now
                                      and self.drone_pos[did - 1] is not None
                                      and self.drone_pos[did - 1][2] >= 1.5]
                        queued_red = bool(candidates)
                if not candidates:
                    continue
                if shared_source:
                    self._adopt_observer(tr, shared_source)
                    did = min(candidates, key=lambda d: (
                        math.hypot(self.drone_pos[d - 1][0] - tr.x[0],
                                   self.drone_pos[d - 1][1] - tr.x[1]), d))
                elif queued_red:
                    # 上一任/曾看到它的机优先；都没有时选离保存导航点最近的空闲机。
                    did = min(candidates, key=lambda d: (
                        d != tr.last_assigned,
                        d not in tr.seen_by,
                        math.hypot(self.drone_pos[d - 1][0] - tr.x[0],
                                   self.drone_pos[d - 1][1] - tr.x[1])))
                else:
                    did = min(candidates,
                              key=lambda d: (d != tr.last_assigned,
                                             -tr.observer_detections[d][1]))
                if not queued_red and not shared_source:
                    self._adopt_observer(tr, did)
                tr.assigned = did
                tr.assigned_since = now
                tr.assignment = '%s/%s' % (slot, uuid.uuid4().hex)
                tr.owner_observation_assignment = ''
                tr.last_tracking_ack = None
                tr.shared_source = shared_source
                tr.blind_acquire = bool(shared_source or queued_red)
                tr.reacq_arrived_since = None
                if queued_red:
                    tr.queued_meas = tr.last_meas
                tr.last_assigned = did
                busy.add(did)
                self.observer_waiting.pop(slot, None)
                if shared_source:
                    rospy.loginfo('[fusion] %s 由%d号机发现，共享坐标派给空闲%d号机接近；'
                                  '等待接手机本机观测', slot, shared_source, did)
                elif queued_red:
                    rospy.logwarn('[fusion] %s 排队期间观测已过 %.1fs；另一个红人已消除，'
                                  '派空闲 %d 号机飞向保存方位重捕。重捕到本机深度前不发裁判',
                                  slot, now - tr.last_meas, did)
                else:
                    rospy.loginfo('[fusion] %s 由 %d 号机认领(谁看到谁跟)', slot, did)

            self.emit_modes(now)

    def emit_modes(self, now):
        """下发目标和状态：0 OFF、1 APPROACH、2 LOCKED、3 REACQUIRE。
        速度策略由飞控按目标距离决定。"""
        busy = {}
        for slot in ALL_SLOTS:
            tr = self.tracks[slot]
            if tr.assigned and tr.status != ST_ELIMINATED:
                busy[tr.assigned] = tr
        for i in range(self.drone_num):
            did = i + 1
            tr = busy.get(did)
            msg = Odometry()
            msg.header.frame_id = 'world'
            if tr is not None:
                meas_age = (now - tr.last_meas) if tr.last_meas else 1e9
                # 排队红人可能是按保存点派给从未亲眼量测过它的空闲机；这种任务
                # 从第一拍就应是 REACQUIRE，而不是伪装成有新鲜量测的 APPROACH。
                owner_age = now - tr.accepted_seen_by.get(did, -1e9)
                if tr.shared_source:
                    mode = MODE_REACQUIRE
                elif tr.lock_seconds(now) > 0.0:
                    mode = MODE_LOCKED
                elif owner_age > self.reacquire_lo:
                    mode = MODE_REACQUIRE
                else:
                    mode = MODE_APPROACH
                # 目标时间戳保留真实量测时间；只在 pub_max_age 内外推，超时冻结。
                # 飞控另检查调度接收租约，通过 child_frame_id 回显认领。
                target_t = min(now, tr.last_meas + self.pub_max_age)
                x_cmd, _ = tr.prediction(target_t)
                msg.header.stamp = rospy.Time.from_sec(tr.last_meas)
                # child_frame_id 随认领变化；不能用 header.seq，ROS 发布器会重写它。
                msg.child_frame_id = tr.assignment
                msg.pose.pose.position.x = float(x_cmd[0])
                msg.pose.pose.position.y = float(x_cmd[1])
                msg.pose.pose.position.z = ACTOR_Z_ORIGIN
                msg.twist.twist.linear.x = float(x_cmd[2])
                msg.twist.twist.linear.y = float(x_cmd[3])
                # 新主控使用原子消息：状态时刻与真实观测/深度时刻分别表达。
                # legacy Odometry 保留旧 stamp 含义，兼容其他现有主控。
                state_t = tr.last_meas if tr.bearing_only_nav else target_t
                x_state, P_state = tr.prediction(state_t)
                target = TrackingTarget()
                target.state = copy.deepcopy(msg)
                target.state.header.stamp = rospy.Time.from_sec(state_t)
                target.state.pose.pose.position.x = float(x_state[0])
                target.state.pose.pose.position.y = float(x_state[1])
                target.state.pose.pose.orientation.w = 1.0
                target.state.twist.twist.linear.x = float(x_state[2])
                target.state.twist.twist.linear.y = float(x_state[3])
                pose_cov = np.eye(6) * 1e6
                pose_cov[:2, :2] = P_state[:2, :2]
                twist_cov = np.eye(6) * 1e6
                twist_cov[:2, :2] = P_state[2:, 2:]
                target.state.pose.covariance = pose_cov.ravel().tolist()
                target.state.twist.covariance = twist_cov.ravel().tolist()
                target.measurement_stamp = rospy.Time.from_sec(tr.last_meas)
                target.depth_stamp = (rospy.Time.from_sec(tr.last_depth_t)
                                      if tr.last_depth_t is not None else rospy.Time())
                target.assignment = tr.assignment
                target.bearing_only = tr.bearing_only_nav
            else:
                # 空闲机: 发 OFF 让 fly_px4 回巡逻
                mode = MODE_OFF
            self.drone_mode[i] = mode
            self.mode_pubs[i].publish(Int8(data=mode))
            if mode != MODE_OFF:
                self.target_pubs[i].publish(msg)
                self.state_pubs[i].publish(target)

    # 可视化 / 日志
    def viz_cb(self, _):
        now = rospy.get_time()
        with self.lock:
            arr = TargetArray()
            arr.header.stamp = rospy.Time.now()
            arr.header.frame_id = 'world'
            n_elim = n_trk = 0
            for slot in ALL_SLOTS:
                tr = self.tracks[slot]
                ts = TargetState()
                ts.tid = tr.tid
                ts.cls = tr.cls
                ts.slot = slot
                ts.x, ts.y = float(tr.x[0]), float(tr.x[1])
                ts.vx, ts.vy = float(tr.x[2]), float(tr.x[3])
                ts.pos_std = tr.pos_std
                ts.last_meas_age = float(now - tr.last_meas) if tr.last_meas else -1.0
                ts.lock_seconds = float(tr.lock_seconds(now))
                ts.status = tr.status
                ts.assigned_drone = tr.assigned
                arr.targets.append(ts)
                if tr.status == ST_ELIMINATED:
                    n_elim += 1
                elif tr.status == ST_TRACKED:
                    n_trk += 1
            arr.n_eliminated = n_elim
            arr.n_tracked = n_trk
            arr.mission_seconds = float(now - self.t0) if self.t0 else 0.0
            self.targets_pub.publish(arr)

    def log_cb(self, _):
        now = rospy.get_time()
        with self.lock:
            names = {ST_UNSEEN: '未见', ST_TRACKED: '跟踪', ST_COASTING: '预测',
                     ST_ELIMINATED: '已消除'}
            rows = []
            n_elim_log = 0
            for slot in ALL_SLOTS:
                tr = self.tracks[slot]
                if tr.status == ST_ELIMINATED:
                    n_elim_log += 1
                if tr.status == ST_UNSEEN and not tr.inited:
                    rows.append('%s:未见' % slot)
                else:
                    depth = ('方位' if tr.bearing_only_nav else
                             ('深' if (tr.last_depth_t is not None
                                     and now - tr.last_depth_t <= self.depth_hold)
                              else '单'))
                    route = ''
                    if slot.startswith('red'):
                        route = ' ->%s(独立发布)' % slot
                    if (tr.cls == 'red' and tr.status != ST_ELIMINATED
                            and not self._judge_authorized(tr, now)):
                        route += ' [红人身份/上报门控未通过]'
                    elif tr.cls != 'red' and tr.assigned and not self._tracking_ready(tr, now):
                        route += ' [等待TRACK回执/本机观测]'
                    if tr.inited and tr.status != ST_ELIMINATED:
                        age = now - tr.last_meas
                        predicted_std = tr.predicted_pos_std(now)
                        blocks = []
                        if not 0.0 <= age <= self.pub_max_age:
                            blocks.append('量测过期')
                        if tr.pos_std > self.max_pos_std:
                            blocks.append('后验协方差')
                        if predicted_std > self.pub_max_pos_std:
                            blocks.append('外推协方差')
                        if depth != '深':
                            blocks.append('无同期深度航迹')
                        if tr.last_residual > self.max_pub_residual:
                            blocks.append('创新残差')
                        if tr.last_gate_fail and now - tr.last_gate_fail < 0.5:
                            blocks.append('重建隔离')
                        route += ' [年龄%.2fs 外推std%.2f%s]' % (
                            age, predicted_std,
                            ' 停发:' + ','.join(blocks) if blocks else '')
                    rows.append('%s:%s(%.1f,%.1f) 后验std%.2f 锁%.1fs %s 拒%d 机%d%s' % (
                        slot, names.get(tr.status, '?'), tr.x[0], tr.x[1],
                        tr.pos_std, tr.lock_seconds(now), depth,
                        tr.judge_rejects, tr.assigned, route))
            rospy.loginfo('[fusion] t=%.0fs 已消除%d/6 | %s',
                          (now - self.t0) if self.t0 else 0.0,
                          n_elim_log,
                          ' | '.join(rows))


def main():
    rospy.init_node('swarm_fusion')
    SwarmFusion()
    rospy.spin()


if __name__ == '__main__':
    main()
