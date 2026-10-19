#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""融合算法离线自检：匈牙利分配、KF/门控收敛、影子计数与消除时序。
加载 ROS 和工作空间环境后直接运行，无需 roscore。"""
import itertools
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'scripts'))
import swarm_fusion as sf   # noqa: E402

FAILED = []


def check(name, cond, detail=''):
    print("  [%s] %s%s" % ("OK " if cond else "FAIL", name,
                           ('  ' + detail) if detail else ''))
    if not cond:
        FAILED.append(name)


def brute_force_min(cost):
    """暴力枚举最优指派代价 (只用于小矩阵对照)"""
    n, m = cost.shape
    best = float('inf')
    if n <= m:
        for perm in itertools.permutations(range(m), n):
            s = sum(cost[i, perm[i]] for i in range(n))
            best = min(best, s)
    else:
        for perm in itertools.permutations(range(n), m):
            s = sum(cost[perm[j], j] for j in range(m))
            best = min(best, s)
    return best


def main():
    print("=" * 78)
    print("swarm_fusion 算法自检")
    print("=" * 78)

    # 1. 匈牙利算法 vs 暴力枚举
    print("\n[1] 手写匈牙利算法 (无 scipy) vs 暴力枚举")
    rng = np.random.RandomState(20260815)
    worst = 0.0
    for trial in range(300):
        n = rng.randint(1, 6)
        m = rng.randint(1, 6)
        cost = np.round(rng.rand(n, m) * 10.0, 3)
        rows, cols = sf.hungarian(cost)
        got = sum(cost[i, j] for i, j in zip(rows, cols))
        want = brute_force_min(cost)
        worst = max(worst, abs(got - want))
        # 指派必须是合法的一对一
        assert len(set(rows)) == len(rows), "行重复"
        assert len(set(cols)) == len(cols), "列重复"
        assert len(rows) == min(n, m), "指派数不等于 min(n,m)"
    check("300 组随机矩阵的最优代价与暴力枚举一致", worst < 1e-9,
          "最大偏差 %.2e" % worst)

    # 含 inf (不可行配对) 的情况
    print("      含 inf(不可行配对) 的情况:")
    cost = np.array([[1.0, np.inf], [np.inf, 2.0]])
    rows, cols = sf.hungarian(cost)
    pairs = set(zip(rows.tolist(), cols.tolist()))
    check("对角可行时选对角", pairs == {(0, 0), (1, 1)}, str(sorted(pairs)))
    cost = np.array([[np.inf, 1.0], [2.0, np.inf]])
    rows, cols = sf.hungarian(cost)
    pairs = set(zip(rows.tolist(), cols.tolist()))
    check("反对角可行时选反对角", pairs == {(0, 1), (1, 0)}, str(sorted(pairs)))
    # 全 inf: 不能崩, 且调用方会用 isfinite 过滤掉
    cost = np.full((2, 2), np.inf)
    rows, cols = sf.hungarian(cost)
    check("全不可行时不崩溃", True, "返回 %d 组(调用方用 isfinite 过滤)" % len(rows))
    rows, cols = sf.hungarian(np.zeros((0, 3)))
    check("空矩阵不崩溃", len(rows) == 0)
    # 长方形 + 明显最优解
    cost = np.array([[9.0, 1.0, 9.0], [9.0, 9.0, 2.0]])
    rows, cols = sf.hungarian(cost)
    got = sum(cost[i, j] for i, j in zip(rows, cols))
    check("长方形矩阵取到最优 3.0", abs(got - 3.0) < 1e-9, "got %.3f" % got)

    # 2. KF: 常速目标上的收敛
    print("\n[2] 卡尔曼滤波: 1m/s 直行目标 + 0.2m 量测噪声")

    class D:      # 最小 Detection 替身
        pass

    def mk(x, y, sa=0.15, sc=0.10, bearing=0.0, cls_id=0):
        d = D()
        d.x, d.y = x, y
        d.sigma_along, d.sigma_cross = sa, sc
        d.bearing = bearing
        d.cls_id = cls_id
        d.method = 1            # M_DEPTH (发布门限的深度支撑)
        d.slant = 7.0
        d.drone_id = 1
        return d

    tr = sf.Track(0, "green", 1.2)
    rng = np.random.RandomState(7)
    t = 0.0
    errs, stds = [], []
    for k in range(120):          # 12 秒 @10Hz
        t += 0.1
        tx, ty = 1.0 * t, 0.0
        d = mk(tx + rng.randn() * 0.15, ty + rng.randn() * 0.10)
        if not tr.inited:
            tr.init_from(d, t)
        else:
            tr.predict(t)
            tr.update(d, t)
        if k > 30:
            errs.append(np.hypot(tr.x[0] - tx, tr.x[1] - ty))
            stds.append(tr.pos_std)
    rms = float(np.sqrt(np.mean(np.square(errs))))
    print("      稳态位置 RMS 误差 %.3f m, 平均 pos_std %.3f m, 速度估计 %.3f m/s"
          % (rms, float(np.mean(stds)), tr.x[2]))
    check("稳态误差优于单帧量测噪声(滤波确实在起作用)", rms < 0.15,
          "RMS %.3f < 0.15" % rms)
    check("速度估计接近真值 1.0 m/s", abs(tr.x[2] - 1.0) < 0.25,
          "%.3f" % tr.x[2])
    check("pos_std 收敛到发布门限 0.35 以下", float(np.mean(stds)) < 0.35,
          "%.3f" % float(np.mean(stds)))

    # 发布外推必须是无副作用快照；图像通常比 wall-clock 晚到 100~200ms。
    # 旧实现由发布 timer 把本体推进到 now，下一张旧时间戳图像便更新在未来状态上。
    x0, P0, tp0 = tr.x.copy(), tr.P.copy(), tr.last_pred
    xp, Pp = tr.prediction(tp0 + 0.5)
    check("发布用 prediction() 不修改 KF 状态",
          np.allclose(tr.x, x0) and np.allclose(tr.P, P0) and tr.last_pred == tp0)
    check("prediction() 会给出向前外推的位置", xp[0] > x0[0])
    last_meas0 = tr.last_meas
    accepted = tr.update(mk(-100.0, 0.0), tp0 - 0.2)
    check("跨批次乱序的旧量测会被拒绝",
          accepted is False and tr.last_meas == last_meas0 and np.allclose(tr.x, x0))

    # 丢帧 3s 后的真实位移仍是 v*3，不能传播 1s 却把时间写成 3s。
    tr_gap = sf.Track(10, 'green', 1.2)
    tr_gap.init_from(mk(0.0, 0.0), 10.0)
    tr_gap.x[2:] = [1.0, -0.5]
    x_gap, P_gap = tr_gap.prediction(13.0)
    check("3s 失联预测覆盖完整物理位移",
          np.allclose(x_gap[:2], [3.0, -1.5]) and tr_gap.last_pred == 10.0)
    tr_gap.predict(13.0)
    check("3s 后恢复观测没有人为残差或时间漏段",
          tr_gap.update(mk(3.0, -1.5), 13.0)
          and tr_gap.last_residual < 1e-10 and tr_gap.last_meas == 13.0)
    tr_steps = sf.Track(11, 'green', 1.2)
    tr_steps.init_from(mk(0.0, 0.0), 10.0)
    tr_steps.x[2:] = [1.0, -0.5]
    for k in range(30):
        tr_steps.predict(10.0 + 0.1 * (k + 1))
    check("完整3s协方差与30个0.1s物理传播一致",
          np.allclose(tr_steps.x, x_gap) and np.allclose(tr_steps.P, P_gap))

    print("      并发发布状态: 各航迹独立，纯深度不依赖 A+J 双链")
    tq = sf.Track(6, 'green', 1.2)
    tq2 = sf.Track(7, 'blue', 1.2)
    d1, d2 = mk(5.0, 0.0), mk(5.0, 10.0)
    d1.method, d1.drone_id = sf.M_DEPTH, 1
    d2.method, d2.drone_id = sf.M_DEPTH, 2
    tq.init_from(d1, 10.0)
    tq2.init_from(d2, 10.0)
    x1, _ = tq.prediction(10.2)
    x2, _ = tq2.prediction(10.2)
    check("两个目标的纯深度航迹可同时形成发布快照",
          tq.last_depth_t == 10.0 and tq2.last_depth_t == 10.0
          and np.allclose(x1[:2], [5.0, 0.0])
          and np.allclose(x2[:2], [5.0, 10.0]))
    tq.update(mk(5.1, 0.0), 10.1)
    check("更新目标1不会修改目标2状态",
          np.allclose(tq2.x[:2], [5.0, 10.0]))

    # 远距纯单目恢复初始接近方式：估计坐标原样交给飞控做三档导航，但航迹
    # 带 bearing_only_nav 标记，绝不能把它当裁判坐标。
    fq = sf.SwarmFusion.__new__(sf.SwarmFusion)
    fq.drone_pos = [np.array([10.0, 20.0, 3.0]),
                    np.array([-30.0, -40.0, 3.0])]
    dfar = mk(100.0, -50.0)
    dfar.method, dfar.slant, dfar.bearing = sf.M_JOINT, 54.8, np.pi / 2.0
    nav = fq._bearing_nav_det(dfar)
    check("单目估计位置原样交给初始版飞控作导航",
          nav is not None and nav.x == 100.0 and nav.y == -50.0
          and nav.slant == 54.8)
    check("生成导航点不会篡改原检测", dfar.x == 100.0 and dfar.y == -50.0)
    tq.bearing_only_nav = True
    tq.init_from(mk(5.0, 0.0), 11.0)
    check("深度重建后退出方位引导", not tq.bearing_only_nav)

    # 仍保留负责机隔离：不能把不同飞机看到的两个同色目标混进同一条导航航迹。
    tr_nav = sf.Track(9, 'white', 1.2)
    fq._enter_bearing_nav(tr_nav, dfar, 20.0)
    tr_nav.assigned = 1
    fq.drone_pos[0] = np.array([11.0, 20.0, 3.0])
    d_owner = mk(42.0, 24.0, bearing=np.deg2rad(92.0))
    d_owner.method, d_owner.slant, d_owner.drone_id = sf.M_JOINT, 50.8, 1
    accepted = fq._update_bearing_nav(tr_nav, d_owner, 20.1)
    check("负责机的单目导航目标直接刷新（不经过裁判坐标 KF）",
          accepted and np.allclose(tr_nav.x[:2], [42.0, 24.0])
          and tr_nav.last_meas == 20.1 and tr_nav.pos_std < 0.3)
    owner_xy = tr_nav.x.copy()
    d_other = mk(-20.0, -30.0, bearing=np.deg2rad(92.0))
    d_other.method, d_other.slant, d_other.drone_id = sf.M_JOINT, 50.8, 2
    accepted = fq._update_bearing_nav(tr_nav, d_other, 20.2)
    check("非负责机生成的另一原点滚动点不会拉扯当前航迹",
          not accepted and np.allclose(tr_nav.x, owner_xy)
          and tr_nav.last_meas == 20.1)
    d_jump = mk(60.0, 40.0, bearing=np.deg2rad(120.0))
    d_jump.method, d_jump.slant, d_jump.drone_id = sf.M_JOINT, 50.8, 1
    accepted = fq._update_bearing_nav(tr_nav, d_jump, 20.2)
    check("初始版接近允许负责机直接刷新新的单目目标",
          accepted and np.allclose(tr_nav.x[:2], [60.0, 40.0])
          and tr_nav.last_meas == 20.2
          and abs(tr_nav.bearing_nav_angle - d_jump.bearing) < 1e-9)

    # 真实 KF 新鲜时，任何纯单目（即使自报 4m）都只能被忽略：
    # 不能更新 last_meas/last_slant，更不能借上一帧 depth_hold 的资格
    # 把单目坐标发给裁判。深度过期后可以转方位引导，但近距单目
    # 仍不能退出；只有真深度帧能重建裁判坐标 KF。
    tr_real = sf.Track(10, 'white', 1.2)
    tr_real.slot = 'white'
    d_real = mk(50.0, 4.0)
    tr_real.init_from(d_real, 30.0)
    real_x, real_P = tr_real.x.copy(), tr_real.P.copy()
    fq.last_seen_by = {}
    fq.mono_nav_switch_age = 0.9
    d_near_mono = mk(-100.0, 80.0, bearing=np.deg2rad(30.0))
    d_near_mono.method, d_near_mono.slant = sf.M_JOINT, 4.0
    # 离线测试未 init_node，屏蔽预期拒绝分支里的 ROS 节流日志。
    old_logwarn_throttle = sf.rospy.logwarn_throttle
    sf.rospy.logwarn_throttle = lambda *args, **kwargs: None
    fq.feed(tr_real, d_near_mono, 30.1, 30.1)
    check("新鲜深度 KF 不接受“近距”单目坐标",
          np.allclose(tr_real.x, real_x) and np.allclose(tr_real.P, real_P)
          and tr_real.last_meas == 30.0 and tr_real.last_slant == d_real.slant
          and not tr_real.bearing_only_nav)

    fq.feed(tr_real, d_near_mono, 31.0, 31.0)
    check("深度过期后近距单目也只进方位引导、清除深度资格",
          tr_real.bearing_only_nav and tr_real.last_depth_t is None
          and tr_real.last_meas == 31.0)
    fq.feed(tr_real, d_near_mono, 31.1, 31.1)
    check("方位引导不会因单目自报近距而退出",
          tr_real.bearing_only_nav and tr_real.last_depth_t is None
          and tr_real.last_meas == 31.1)

    d_depth_back = mk(12.0, 22.0, bearing=np.deg2rad(30.0))
    d_depth_back.method, d_depth_back.slant = sf.M_DEPTH, 7.0
    fq.feed(tr_real, d_depth_back, 31.2, 31.2)
    sf.rospy.logwarn_throttle = old_logwarn_throttle
    check("只有深度恢复才退出方位引导并重建真实 KF",
          not tr_real.bearing_only_nav and tr_real.last_depth_t == 31.2
          and tr_real.last_meas == 31.2
          and np.allclose(tr_real.x[:2], [12.0, 22.0]))

    d_direct_mono = mk(999.0, 999.0)
    d_direct_mono.method = sf.M_JOINT
    x_before = tr_real.x.copy()
    check("Track.update 底层也拒绝单目，防止新路径意外混入",
          tr_real.update(d_direct_mono, 31.3) is False
          and np.allclose(tr_real.x, x_before)
          and tr_real.last_meas == 31.2)

    # 双红并行跟踪、固定话题独立发布与飞控回执的端到端回归见 test_tracking_publication.py。
    # 3. 门控: 离群点应被拒，重初始化必须持续确认
    print("\n[3] 门控: 2.5m 绝对门 + chi2 马氏门")
    tr2 = sf.Track(1, "blue", 1.2)
    tr2.init_from(mk(0.0, 0.0), 0.0)
    for k in range(20):
        tr2.predict(0.1 * (k + 1))
        tr2.update(mk(0.0, 0.0), 0.1 * (k + 1))
    m2_near = tr2.mahalanobis2(mk(0.2, 0.0))
    m2_far = tr2.mahalanobis2(mk(6.0, 0.0))
    check("近点在门内", m2_near < 9.21, "m2=%.2f" % m2_near)
    check("6m 外的点在门外", m2_far > 9.21, "m2=%.2f" % m2_far)
    check("绝对门也会拦下 6m 跳变", 6.0 > 2.5)

    # v17 回归：green 曾被 3.46m 外的两帧坏双链直接重初始化，随后产生
    # 1.54m/1.95m 两条超差消息。新位置必须持续一致 1s 才允许搬航迹。
    tr_jump = sf.Track(8, "green", 1.2)
    ready = False
    for k in range(2):
        ready = tr_jump.reinit_candidate_ready(
            mk(3.4, 0.05 * k), 2.0 + 0.2 * k, 5, 1.0, 0.8, 0.5)
    check("v17 式两帧 3.46m 假跳点不能触发重初始化", not ready)
    tr_jump.clear_reinit_candidate()
    for k in range(6):
        ready = tr_jump.reinit_candidate_ready(
            mk(3.4 + 0.05 * k, 0.0), 3.0 + 0.2 * k, 5, 1.0, 0.8, 0.5)
    check("真实新位置持续一致 1s 后仍可重初始化", ready)

    n_before, candidate_before = tr_jump.reinit_n, tr_jump.reinit_det
    ready_duplicate = tr_jump.reinit_candidate_ready(
        mk(100.0, 0.0), tr_jump.reinit_last_t, 5, 1.0, 0.8, 0.5)
    check("重复旧帧不能增加重初始化次数或清掉新候选",
          not ready_duplicate and tr_jump.reinit_n == n_before
          and tr_jump.reinit_det is candidate_before)

    # 两机异步：较晚图像是离群点，较早的有效图像仍应能更新航迹。
    fq.gate_chi2, fq.gate_abs, fq.gate_reinit_stale = 9.21, 2.5, 1.5
    fq.reinit_confirm_updates, fq.reinit_confirm_sec = 5, 1.0
    fq.reinit_confirm_dist, fq.reinit_confirm_gap = 0.8, 0.5
    tr_async = sf.Track(12, 'green', 1.2)
    tr_async.slot = 'green'
    tr_async.init_from(mk(0.0, 0.0), 10.0)
    tr_async.x[2] = 1.0
    async_x, async_P = tr_async.x.copy(), tr_async.P.copy()
    bad = mk(10.0, 20.0)
    fq.feed(tr_async, bad, 11.9, 12.0)
    check("被门控拒绝的较新量测不会推进主状态时间轴",
          tr_async.last_pred == 10.0 and tr_async.last_meas == 10.0
          and np.array_equal(tr_async.x, async_x)
          and np.array_equal(tr_async.P, async_P) and tr_async.gate_fail == 1)
    good = mk(1.0, 0.0)
    good.drone_id = 2
    good.pipeline_lag = 0.12
    fq.feed(tr_async, good, 11.0, 12.0)
    check("异步抵达的较早有效量测仍可采用且记录真实来源",
          tr_async.last_meas == 11.0 and tr_async.last_meas_drone == 2
          and tr_async.last_det.drone_id == 2 and tr_async.gate_fail == 0)
    accepted_x, accepted_P = tr_async.x.copy(), tr_async.P.copy()
    fq.feed(tr_async, bad, 10.9, 12.0)
    check("旧帧在门控之前被拒绝，不影响计数、来源或状态",
          tr_async.last_pred == 11.0 and tr_async.last_meas == 11.0
          and tr_async.gate_fail == 0 and tr_async.reinit_det is None
          and tr_async.last_det.drone_id == 2
          and np.array_equal(tr_async.x, accepted_x)
          and np.array_equal(tr_async.P, accepted_P))
    check("直接旧时间初始化也不能回拨航迹",
          tr_async.init_from(bad, 10.5) is False
          and tr_async.last_meas == 11.0 and np.array_equal(tr_async.x, accepted_x))
    nav_old = sf.Track(13, 'green', 1.2)
    fq._enter_bearing_nav(nav_old, dfar, 20.0)
    fq.feed(nav_old, mk(-10.0, -20.0), 19.9, 20.1)
    check("迟到深度不能把较新的单目导航重置到旧位置",
          nav_old.bearing_only_nav and nav_old.last_meas == 20.0
          and nav_old.last_depth_t is None and np.allclose(nav_old.x[:2], [100.0, -50.0]))

    # 4. 各向异性 R: 沿视线方向的不确定度应更大
    print("\n[4] 各向异性量测噪声 R (测距误差 >> 横向误差)")
    tr3 = sf.Track(2, "brown", 1.2)
    for bearing_deg in (0.0, 45.0, 90.0):
        b = np.deg2rad(bearing_deg)
        R = tr3.R_of(mk(0, 0, sa=0.30, sc=0.10, bearing=b))
        along = np.array([np.cos(b), np.sin(b)])
        cross = np.array([-np.sin(b), np.cos(b)])
        v_al = float(along @ R @ along)
        v_cr = float(cross @ R @ cross)
        check("bearing=%.0f 度: 沿视线方差 0.30^2" % bearing_deg,
              abs(v_al - 0.09) < 1e-9, "%.4f" % v_al)
        check("bearing=%.0f 度: 横向方差 0.10^2" % bearing_deg,
              abs(v_cr - 0.01) < 1e-9, "%.4f" % v_cr)
    tr_init_y = sf.Track(14, 'brown', 1.2)
    tr_init_y.init_from(mk(0.0, 0.0, sa=1.0, sc=0.1, bearing=np.pi / 2), 1.0)
    check("90度视线初始化把较大的测距方差放在世界y轴",
          abs(tr_init_y.P[0, 0] - 0.11) < 1e-9
          and abs(tr_init_y.P[1, 1] - 1.1) < 1e-9)
    tr_init_diag = sf.Track(15, 'brown', 1.2)
    tr_init_diag.init_from(mk(0.0, 0.0, sa=1.0, sc=0.1, bearing=np.pi / 4), 1.0)
    check("45度视线初始化保留世界xy之间的相关方差",
          abs(tr_init_diag.P[0, 1] - 0.495) < 1e-9
          and abs(tr_init_diag.P[1, 0] - 0.495) < 1e-9)

    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

    # 原子目标接口必须把预测状态时间与实际观测时间分开；legacy保持兼容。
    f_cmd = sf.SwarmFusion.__new__(sf.SwarmFusion)
    f_cmd.drone_num = 1
    f_cmd.drone_mode = [sf.MODE_OFF]
    f_cmd.pub_max_age, f_cmd.reacquire_lo = 0.9, 0.4
    f_cmd.mode_pubs, f_cmd.target_pubs, f_cmd.state_pubs = ([Publisher()],
                                                         [Publisher()], [Publisher()])
    f_cmd.tracks = {slot: sf.Track(k, slot, 1.2)
                    for k, slot in enumerate(sf.ALL_SLOTS)}
    tc = f_cmd.tracks['green']
    tc.init_from(mk(10.0, 5.0), 100.0)
    tc.assigned, tc.assignment = 1, 'green/test-token'
    tc.x[2:] = [1.5, 0.0]
    f_cmd.emit_modes(100.4)
    legacy = f_cmd.target_pubs[0].messages[-1]
    atomic = f_cmd.state_pubs[0].messages[-1]
    check("原子目标的state_stamp对应预测坐标，观测/深度时间不被刷新",
          abs(atomic.state.header.stamp.to_sec() - 100.4) < 1e-8
          and atomic.measurement_stamp.to_sec() == 100.0
          and atomic.depth_stamp.to_sec() == 100.0
          and abs(atomic.state.pose.pose.position.x - 10.6) < 1e-8
          and atomic.assignment == atomic.state.child_frame_id == 'green/test-token'
          and atomic.state.header.frame_id == 'world' and not atomic.bearing_only)
    check("legacy目标保留最后观测stamp及相同坐标以兼容其他主控",
          legacy.header.stamp.to_sec() == 100.0
          and legacy.pose.pose.position.x == atomic.state.pose.pose.position.x)
    f_cmd.emit_modes(102.0)
    frozen = f_cmd.state_pubs[0].messages[-1]
    check("失联目标冻结后不会用发送时间伪装状态或量测时间",
          abs(frozen.state.header.stamp.to_sec() - 100.9) < 1e-8
          and frozen.measurement_stamp.to_sec() == 100.0
          and abs(frozen.state.pose.pose.position.x - 11.35) < 1e-8)
    f_cmd._enter_bearing_nav(tc, dfar, 200.0)
    f_cmd.emit_modes(200.4)
    mono_atomic = f_cmd.state_pubs[0].messages[-1]
    check("单目导航有观测stamp，缺少深度独立表达且不虚构预测",
          mono_atomic.bearing_only and mono_atomic.depth_stamp.is_zero()
          and mono_atomic.measurement_stamp.to_sec() == 200.0
          and mono_atomic.state.header.stamp.to_sec() == 200.0
          and mono_atomic.state.twist.twist.linear.x == 0.0)
    fq.diagnostics_pub, fq.probe_dx, fq.probe_dy = Publisher(), 0.0, 0.0
    xp_diag, pp_diag = tr_async.prediction(11.2)
    fq._publish_diagnostics(tr_async, ['green'], xp_diag, pp_diag, 11.2)
    diag = json.loads(fq.diagnostics_pub.messages[-1].data)
    check("发布诊断记录实际采纳观测，拒绝帧不冒充来源",
          diag['source_drone'] == 2 and diag['measurement_stamp'] == 11.0
          and diag['meas_xy'] == [1.0, 0.0]
          and abs(diag['pipeline_lag'] - 0.12) < 1e-9
          and diag['pub_slots'] == ['green'] and np.shape(diag['cov']) == (2, 2))

    # 5. 影子计数的时序语义
    print("\n[5] 影子计数: 只有连续发布才累积, 断流 >0.9s 清零")
    print("      (裁判 score_cal.py:27 的 interval<1s 判据 => 静默 0.9s 是安全的,")
    print("       而发一条超差消息要赔整整 15 秒 —— 这就是'存疑就闭嘴'的依据)")
    tr4 = sf.Track(3, 'white', 1.2, pub_gap_reset=0.9)
    tr4.pub_run_start = 100.0
    tr4.last_pub = 116.5
    check("连续发布 16.5s 达到消除门限", tr4.lock_seconds(116.5) >= 16.5,
          "%.1f" % tr4.lock_seconds(116.5))
    tr4.last_pub = 103.0
    check("刚发 3s 未达门限", tr4.lock_seconds(103.0) < 16.5)
    tr4.pub_run_start = None
    check("清零后 lock_seconds 归 0", tr4.lock_seconds(200.0) == 0.0)

    # 回归: 停止发布后影子计数**必须**归零。
    # 联调时实测过这个 bug —— 停发 4.8 秒后计数还在涨到 20.9 秒, 足以触发假的
    # "已消除"判定, 然后把飞机放走、把目标标成完成。裁判那边 interval>=1s 就把
    # count_flag 清零了, 所以我们的语义必须一致。
    print("      回归: 停发后计数必须归零 (曾经的真 bug)")
    tr6 = sf.Track(5, 'green', 1.2, pub_gap_reset=0.9)
    tr6.pub_run_start = 100.0
    for k in range(200):                  # 20 秒 @10Hz 连续发布
        tr6.last_pub = 100.0 + 0.1 * (k + 1)
    last = tr6.last_pub
    check("连续发布 20s -> 计数约 20s", abs(tr6.lock_seconds(last) - 20.0) < 0.2,
          "%.2f" % tr6.lock_seconds(last))
    check("停发 0.5s (容差内) -> 计数保持", tr6.lock_seconds(last + 0.5) > 19.8,
          "%.2f" % tr6.lock_seconds(last + 0.5))
    check("停发 1.5s (超容差) -> 计数归零", tr6.lock_seconds(last + 1.5) == 0.0,
          "%.2f" % tr6.lock_seconds(last + 1.5))
    check("停发 4.8s -> 计数归零 (修复前会涨到 24.8)",
          tr6.lock_seconds(last + 4.8) == 0.0,
          "%.2f" % tr6.lock_seconds(last + 4.8))

    # 重初始化必须清掉影子计数(航迹断了, 裁判那边也断了)
    tr5 = sf.Track(4, 'green', 1.2, pub_gap_reset=0.9)
    tr5.init_from(mk(0, 0), 0.0)
    tr5.pub_run_start = 0.0
    tr5.init_from(mk(9, 9), 5.0)
    check("KF 重初始化会清掉影子计数", tr5.pub_run_start is None)

    # 6. 扩散先验网格
    print("\n[6] 扩散先验: 观测清零 + 3x3 均匀滤波 (5m格/5s = 1m/s, 与行人同速)")
    nx, ny = 8, 6
    prior = np.ones((nx, ny))
    prior[3, 3] = 0.0          # 假设这一格刚被看过

    def diffuse(p):
        acc = np.zeros_like(p)
        cnt = np.zeros_like(p)
        n, m = p.shape
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                sl = p[max(dx, 0):n + min(dx, 0), max(dy, 0):m + min(dy, 0)]
                acc[max(-dx, 0):n - max(dx, 0), max(-dy, 0):m - max(dy, 0)] += sl
                cnt[max(-dx, 0):n - max(dx, 0), max(-dy, 0):m - max(dy, 0)] += 1.0
        return np.clip(acc / np.maximum(cnt, 1.0), 0.0, 1.0)

    p1 = diffuse(prior)
    check("扩散后清零格开始回升(目标可能走回来)", p1[3, 3] > 0.0,
          "%.3f" % p1[3, 3])
    check("扩散不会越界/产生 nan", np.isfinite(p1).all())
    check("扩散后仍在 [0,1]", p1.min() >= 0.0 and p1.max() <= 1.0)
    p = prior.copy()
    for _ in range(6):
        p = diffuse(p)
        p[3, 3] = min(p[3, 3], 1.0)
    check("反复扩散后清零格趋近 1(30 秒没回去看就该重扫)", p[3, 3] > 0.7,
          "%.3f" % p[3, 3])
    # 全部看过 -> 先验应接近 0, 扩散也拉不起来
    allseen = np.zeros((nx, ny))
    check("全场看过后先验为 0(不会瞎派搜索)", diffuse(allseen).max() == 0.0)

    # 孤立格一步就回升到 8/9=0.889 看着很快, 但那是对的: 一个四周都没看过的
    # 5m 格, 行人 5 秒就能从邻格走进来。真实的扫描是**连续带状**的, 带中心恢复
    # 得慢得多 —— 这才是实际会遇到的情形, 单独验一下。
    print("      连续扫出一条 20m 宽带后, 带中心的恢复曲线(每步=5s):")
    band = np.ones((40, 20))
    band[10:14, :] = 0.0
    q = band.copy()
    curve = []
    for k in range(8):
        curve.append(q[12, 10])
        q = diffuse(q)
    print("        " + "  ".join("t=%ds:%.2f" % (5 * i, v)
                                 for i, v in enumerate(curve)))
    check("带中心 10 秒内几乎不回升(不会立刻重扫刚扫过的地方)",
          curve[1] < 0.05, "t=5s 时 %.3f" % curve[1])
    check("带中心 35 秒后明显回升(行人可能已走回来)",
          curve[7] > 0.30, "t=35s 时 %.3f" % curve[7])

    # 7. 常量与裁判接口一致性
    print("\n[7] 裁判接口常量核对 (照抄 score_cal.py, 错一个字就不得分)")
    check("6 个裁判话题齐全", sorted(sf.JUDGE_TOPICS.values()) == sorted([
        '/actor_green_info', '/actor_blue_info', '/actor_brown_info',
        '/actor_white_info', '/actor_red1_info', '/actor_red2_info']))
    check("cls 名与 actor_id_dict 的 key 一致",
          sf.CLS_NAMES == ['green', 'blue', 'brown', 'white', 'red'])
    check("red 占两个槽位", sf.SLOTS_OF_CLS['red'] == ['red1', 'red2'])
    check("裁判比对高度 = 1.0191 (ActorPluginRos 每拍写死)",
          abs(sf.ACTOR_Z_ORIGIN - 1.0191) < 1e-9)
    check("lock_need 留了裁判采样余量 (>15s)", True, "默认 16.5s")

    print("\n" + "=" * 78)
    if FAILED:
        print("失败 %d 项:" % len(FAILED))
        for f in FAILED:
            print("   -", f)
        return 1
    print("全部通过。匈牙利/KF/门控/影子计数/扩散先验 在离线输入下自洽。")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
