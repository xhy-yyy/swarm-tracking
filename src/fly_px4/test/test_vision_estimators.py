#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线合成框和深度，验证定位几何、姿态敏感度、联合解算及质量护栏。
仅需 numpy，不启动 ROS/Gazebo/YOLO。深度夹具验证同像素反投影；
轴对齐框共用中心 u 导致毫米级脚/头残差，任意方位测试采用 5mm 容差。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'scripts'))

# swarm_vision 在 import 时不碰 ROS/torch (YOLO 是延迟 import 的), 可以直接引入
import swarm_vision as sv   # noqa: E402

FAILED = []
CAM_PITCH_DEG = 25.0
FLIGHT_H = 5.0


def check(name, got, want, tol):
    ok = abs(got - want) <= tol
    print("  [%s] %-52s got %+9.4f want %+9.4f (tol %g)"
          % ("OK " if ok else "FAIL", name, got, want, tol))
    if not ok:
        FAILED.append(name)


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


R_OPT2CAM = np.array([[0, 0, 1.], [-1, 0, 0], [0, -1, 0]])


def cam_pose(drone_xy, yaw_deg, pitch_deg=CAM_PITCH_DEG, height=FLIGHT_H):
    """返回 (p_cam 世界系相机光心, R_cw 光学->世界)。与 odom_world_relay 的约定一致。"""
    yaw = np.deg2rad(yaw_deg)
    R_cw = Rz(yaw) @ Ry(np.deg2rad(pitch_deg)) @ R_OPT2CAM
    p_body = np.array([drone_xy[0], drone_xy[1], height])
    p_cam = p_body + Rz(yaw) @ sv.CAM_OFFSET_BODY
    return p_cam, R_cw


def project(p_cam, R_cw, pt_world):
    """世界点 -> 像素 (u,v) 和光轴分量 z_c。点在相机后方返回 None。"""
    v = np.asarray(pt_world, dtype=float) - p_cam
    v_opt = R_cw.T @ v            # 世界 -> 光学系
    if v_opt[2] <= 1e-6:
        return None
    u = sv.CAMERA_FX * v_opt[0] / v_opt[2] + sv.CAMERA_CX
    vv = sv.CAMERA_FY * v_opt[1] / v_opt[2] + sv.CAMERA_CY
    return u, vv, v_opt[2]


def synth(actor_xy, drone_xy, yaw_deg, pitch_deg=CAM_PITCH_DEG, height=FLIGHT_H,
          with_depth=True):
    """合成 (box, depth_img 或 None, p_cam, R_cw)，不在画面内返回 None。
    p_cam 已含杆臂，距离真值以光心为原点；深度为躯干前表面，body_half_depth 修正回中心。"""
    p_cam, R_cw = cam_pose(drone_xy, yaw_deg, pitch_deg, height)
    a = np.array([actor_xy[0], actor_xy[1], 0.0])
    top = project(p_cam, R_cw, a + [0, 0, sv.ACTOR_Z_HEAD])
    bot = project(p_cam, R_cw, a + [0, 0, sv.ACTOR_Z_FEET])
    ctr = project(p_cam, R_cw, a + [0, 0, sv.ACTOR_Z_ORIGIN])
    if top is None or bot is None or ctr is None:
        return None
    if not (0 <= ctr[0] < sv.IMG_W and 0 <= top[1] and bot[1] < sv.IMG_H):
        return None            # 人没完整落在画面里
    # 人的横向宽度约 0.337m, 用中心距离折算成像素宽
    half_w = 0.5 * 0.337 * sv.CAMERA_FX / ctr[2]
    box = np.array([ctr[0] - half_w, top[1], ctr[0] + half_w, bot[1]])
    depth = None
    if with_depth:
        # 只需要躯干带有效: _estimate 取 v∈[top+0.25h, top+0.55h], u∈uc±0.15w
        depth = np.full((sv.IMG_H, sv.IMG_W), np.nan, dtype=np.float32)
        bh = box[3] - box[1]
        r0 = int(max(0, np.floor(box[1] + 0.20 * bh)))
        r1 = int(min(sv.IMG_H, np.ceil(box[1] + 0.60 * bh)))
        c0 = int(max(0, np.floor(ctr[0] - half_w)))
        c1 = int(min(sv.IMG_W, np.ceil(ctr[0] + half_w)))
        if r1 <= r0 or c1 <= c0:
            return None
        # 躯干前表面近似成一个竖直平面: 过人体中心沿"指向相机"方向前移 0.15m,
        # 法向指向相机水平方向。每个像素的光轴分量取"射线与该平面交点"的 z_c。
        n = np.array([p_cam[0] - actor_xy[0], p_cam[1] - actor_xy[1], 0.0])
        n /= np.linalg.norm(n)
        p0 = np.array([actor_xy[0], actor_xy[1], sv.ACTOR_Z_ORIGIN]) \
            + 0.15 * n          # 0.15 = body_half_depth (前表面)
        for r in range(r0, r1):
            for c in range(c0, c1):
                rw = R_cw @ np.array([(c - sv.CAMERA_CX) / sv.CAMERA_FX,
                                      (r - sv.CAMERA_CY) / sv.CAMERA_FY, 1.0])
                den = float(np.dot(n, rw))
                if abs(den) < 1e-9:
                    continue
                t = float(np.dot(n, p0 - p_cam)) / den
                if t <= 0:
                    continue
                v_opt = R_cw.T @ (t * rw)
                depth[r, c] = v_opt[2]
    return box, depth, p_cam, R_cw


class FakeStream:
    def __init__(self, drone_id=1):
        self.drone_id = drone_id


class Estimator:
    """只借用 SwarmVision 的几何方法, 不启动 ROS/YOLO"""

    def __init__(self, min_range=3.0, max_range=12.0, rate=10.0):
        self.min_range = min_range
        self.max_range = max_range
        self.rate_hz = rate
        self.body_half_depth = 0.15
        self.depth_scale = 1.0
        self.depth_trust_range = 12.0
        self.mono_scale = 1.0
        self.mono_sigma_scale = 2.0
        # 合成框上下沿就是精确脚/头，测试夹具不需要生产环境针对 YOLO 框的修正。
        self.foot_margin = 0.0
        self.cls_depth_corr = [1.0] * 6
        self.cls_mono_corr = [1.0] * 6
        self.bearing_corr_deg = 0.0
        self.veto_publish = True
        self.depth_min_pixels = 20
        self.depth_min_valid_ratio = 0.10
        self.depth_max_mad = 0.35
        self.depth_joint_max_diff = 0.70
        self.depth_size_max_ratio = 2.5
        self.n_drop_range = 0
        self.n_drop_consist = 0
        self.n_veto_downgrade = 0

    _ray_world = sv.SwarmVision._ray_world
    _corr = staticmethod(sv.SwarmVision._corr)
    _plane_hit = staticmethod(sv.SwarmVision._plane_hit)
    _estimate = sv.SwarmVision._estimate


def run_case(est, actor_xy, drone_xy, yaw_deg, **kw):
    s = synth(actor_xy, drone_xy, yaw_deg, **kw)
    if s is None:
        return None
    box, depth, p_cam, R_cw = s
    return est._estimate(FakeStream(), box, depth, p_cam, R_cw)


def main():
    print("=" * 78)
    print("swarm_vision 定位估计量自检   pitch=%.1f deg  高度=%.1f m" %
          (CAM_PITCH_DEG, FLIGHT_H))
    print("=" * 78)

    # 1. 理想输入: 沿环带扫距离, 定位误差应精确为 0
    # 这一节是整个方案的地基: 估计量在理想输入下必须**无偏**。任何常量偏差都会
    # 直接吃掉 1m 误差预算, 而且不像随机噪声那样能被 15 秒平均掉。
    print("\n[1] 理想输入(姿态/高度都准), 正对目标, 扫水平距离")
    print("      %-7s %-11s %-9s %-9s %-7s"
          % ("距离", "定位误差", "sig_along", "sig_cross", "method"))
    # 4.0m 故意放在环带外: 22 度下倾时全身约 4.4m 才入画(脚会掉出画面下沿),
    # 所以这个距离**应该**没有解 —— 这同时验证了环带下界的算法。
    est = Estimator()
    p_cam, _ = cam_pose((0.0, 0.0), 0.0)
    check("4.0m (全身入画下界约 4.4m 之内) 应无解",
          0.0 if run_case(est, (p_cam[0] + 4.0, 0.0), (0.0, 0.0), 0.0,
                          with_depth=False) is None else 1.0, 0.0, 0.5)
    for d in (6.0, 6.5, 7.5, 9.0):
        p_cam, _ = cam_pose((0.0, 0.0), 0.0)
        ax = p_cam[0] + d          # 相对**相机光心**摆放, 免得杆臂混进真值
        # 纯几何链路(不给深度): 必须精确为 0
        r = run_case(est, (ax, 0.0), (0.0, 0.0), 0.0, with_depth=False)
        if r is None:
            print("      %-7.1f (无解/被护栏拦下)" % d)
            FAILED.append("d=%.1f 理想输入无解" % d)
            continue
        err = np.hypot(r['x'] - ax, r['y'] - 0.0)
        # 带合成深度时坐标必须只取 A；J 只做一致性互查。
        # 这个平面深度夹具自带约 6~7cm 偏差，容差只看量级。
        depth_est = Estimator()
        rd = run_case(depth_est, (ax, 0.0), (0.0, 0.0), 0.0, with_depth=True)
        err_d = np.hypot(rd['x'] - ax, rd['y']) if rd else float('nan')
        print("      %-7.1f %-11.5f %-9.3f %-9.3f %-7d  (含合成深度 %.4f)"
              % (d, err, r['sigma_along'], r['sigma_cross'], r['method'], err_d))
        check("d=%.1f 纯几何链路定位误差(应精确为0)" % d, err, 0.0, 1e-6)
        # 无深度时只有 J 进均值 (C/H 已降级为否决专用): DEPTH|JOINT = 8
        check("d=%.1f 单目链路 method=8(仅 JOINT)" % d, float(r['method']), 8.0, 0.5)
        if rd is not None:
            check("d=%.1f 纯深度输出误差(同像素反投影)" % d,
                  err_d, 0.0, 0.012)
            check("d=%.1f 含深度时保留 DEPTH 来源位" % d,
                  float(rd['method'] & sv.M_DEPTH), float(sv.M_DEPTH), 0.5)
            # 把单目 sigma 放大 50 倍也不应改变坐标；若还在 A/J 加权，
            # 这个参数会改变加权结果。
            depth_est2 = Estimator()
            depth_est2.mono_sigma_scale = 100.0
            rd2 = run_case(depth_est2, (ax, 0.0), (0.0, 0.0), 0.0,
                           with_depth=True)
            check("d=%.1f 深度坐标不受单目权重影响" % d,
                  np.hypot(rd2['x'] - rd['x'], rd2['y'] - rd['y']),
                  0.0, 1e-9)

    # 任意方位测试容差 5mm：轴对齐框共用中心 u，无法保留脚/头投影的微小横向差。
    print("\n[2] 任意机体朝向 + 目标不在光轴上 (查方位依赖/符号错误)")
    est = Estimator()
    for yaw in (0.0, 41.0, 127.0, -95.0, 180.0):
        for lateral in (-1.5, 0.0, 1.5):
            d = 6.5
            yr = np.deg2rad(yaw)
            # 相对**相机光心**摆放: 机头方向 d 米、侧向 lateral 米
            p_cam, _ = cam_pose((10.0, -4.0), yaw)
            ax = p_cam[0] + d * np.cos(yr) - lateral * np.sin(yr)
            ay = p_cam[1] + d * np.sin(yr) + lateral * np.cos(yr)
            r = run_case(est, (ax, ay), (10.0, -4.0), yaw, with_depth=False)
            if r is None:
                FAILED.append("yaw=%.0f lat=%.1f 无解" % (yaw, lateral))
                print("      yaw=%+6.1f lat=%+4.1f -> 无解 (不该发生)" % (yaw, lateral))
                continue
            err = np.hypot(r['x'] - ax, r['y'] - ay)
            check("yaw=%+6.1f lat=%+4.1f 误差" % (yaw, lateral), err, 0.0, 0.005)

    # 3. 姿态误差: 单目 J 应显著小于 C/H 的理论敏感度
    print("\n[3] 注入 1 度俯仰误差 (相机实际下倾 %.0f 度, 而系统以为是 %.0f 度)"
          % (CAM_PITCH_DEG + 1., CAM_PITCH_DEG))
    print("      C/H 单链路的理论敏感度 d*(1/tan a + tan a): 6.5m 处约 0.24~0.26 m/度,")
    print("      9m 处 0.38~0.53 m/度。J 不依赖输入高度，但仍有俯仰敏感度,")
    print("      J 的误差应远小于 C/H 理论值 —— 它可用于远距方位引导。")
    est = Estimator()
    for d in (5.0, 6.5, 9.0):
        s = synth((d, 0.0), (0.0, 0.0), 0.0, pitch_deg=CAM_PITCH_DEG + 1.0)
        if s is None:
            continue
        box, depth, p_cam, _ = s
        # 系统按配置角解算(用错的 R_cw), 真实成像多下倾 1 度
        _, R_cw_believed = cam_pose((0.0, 0.0), 0.0, CAM_PITCH_DEG)
        r = est._estimate(FakeStream(), box, None, p_cam, R_cw_believed)  # 只看单目链路
        if r is None:
            print("      d=%.1f 被一致性/环带护栏拦下" % d)
            continue
        err = np.hypot(r['x'] - d, r['y'])
        print("      d=%-5.1f 1度俯仰误差 -> 定位误差 %.3f m" % (d, err))
        check("d=%.1f 1度俯仰误差 < 0.5m (J俯仰敏感度受检验, 远小于 C/H 理论)" % d,
              1.0 if err <= 0.50 else 0.0, 1.0, 0.5)

    # 4. 联合解算器 J: 从两条俯角同时解出距离和相机高度
    print("\n[4] 联合解算器 J: 脚/头两条俯角 -> (距离, 相机高度)")
    print("      行人脚 z=%.4f 头 z=%.4f 都是精确常量(插件每拍写死),"
          % (sv.ACTOR_Z_FEET, sv.ACTOR_Z_HEAD))
    print("      于是 d=(h-z_f)cot(a_f)=(h-z_h)cot(a_h) 是 2 方程 2 未知量, 有闭式解。")

    def joint_solve(box, R_cw):
        """复算 _estimate 里的 J: 返回 (d, h) 或 None"""
        uc = 0.5 * (box[0] + box[2])
        e = Estimator()
        r_b = sv.SwarmVision._ray_world(e, R_cw, uc, box[3])
        r_t = sv.SwarmVision._ray_world(e, R_cw, uc, box[1])

        def depression(r):
            horiz = np.hypot(r[0], r[1])
            if horiz < 1e-9 or r[2] >= -1e-9:
                return None
            return float(np.arctan2(-r[2], horiz))
        a_f, a_h = depression(r_b), depression(r_t)
        if a_f is None or a_h is None:
            return None
        cf, ch = 1.0 / np.tan(a_f), 1.0 / np.tan(a_h)
        den = cf - ch
        if abs(den) < 1e-9:
            return None
        h = (sv.ACTOR_Z_FEET * cf - sv.ACTOR_Z_HEAD * ch) / den
        return (h - sv.ACTOR_Z_FEET) * cf, h

    print("      %-7s %-14s %-14s %-12s" % ("距离", "d 残差", "h 残差", "条件数|cf-ch|"))
    for d_true in (5.0, 6.5, 9.0, 11.0):
        p_cam, R_cw = cam_pose((0.0, 0.0), 0.0)
        ax = p_cam[0] + d_true
        s = synth((ax, 0.0), (0.0, 0.0), 0.0, with_depth=False)
        if s is None:
            continue
        box, _, pc, Rc = s
        js = joint_solve(box, Rc)
        if js is None:
            FAILED.append("d=%.1f 联合解算失败" % d_true)
            continue
        dj, hj = js
        uc = 0.5 * (box[0] + box[2])
        e = Estimator()
        rb = sv.SwarmVision._ray_world(e, Rc, uc, box[3])
        rt = sv.SwarmVision._ray_world(e, Rc, uc, box[1])
        cf = 1.0 / np.tan(np.arctan2(-rb[2], np.hypot(rb[0], rb[1])))
        ch = 1.0 / np.tan(np.arctan2(-rt[2], np.hypot(rt[0], rt[1])))
        print("      %-7.1f %-14.6f %-14.6f %-12.4f"
              % (d_true, dj - d_true, hj - pc[2], abs(cf - ch)))
        check("d=%.1f J 解出的距离" % d_true, dj, d_true, 1e-4)
        check("d=%.1f J 解出的相机高度" % d_true, hj, pc[2], 1e-4)

    # 条件数: 脚行抖 1 px 会给相机高度带来多大误差
    print("\n      条件数检查 (脚行抖 1 px -> 相机高度误差):")
    for d_true in (5.0, 6.5, 9.0):
        p_cam, R_cw = cam_pose((0.0, 0.0), 0.0)
        ax = p_cam[0] + d_true
        s = synth((ax, 0.0), (0.0, 0.0), 0.0, with_depth=False)
        if s is None:
            continue
        box, _, pc, Rc = s
        h0 = joint_solve(box, Rc)[1]
        box2 = box.copy()
        box2[3] += 1.0
        h1 = joint_solve(box2, Rc)[1]
        print("        d=%-5.1f 框高 %3.0f px, 1px -> h 误差 %+.4f m"
              % (d_true, box[3] - box[1], h1 - h0))
        check("d=%.1f 条件数可接受(1px 的 h 误差 <0.05m)" % d_true,
              abs(h1 - h0), 0.0, 0.05)

    # 4b. 高度误差免疫: A/J 两条链路都不依赖相机高度
    print("\n[4b] 注入 0.3m 相机高度误差 (EKF z 漂移的典型量级)")
    d_true = 6.5
    p_cam, R_cw = cam_pose((0.0, 0.0), 0.0)
    ax = p_cam[0] + d_true
    box, _, pc, Rc = synth((ax, 0.0), (0.0, 0.0), 0.0, with_depth=False)
    p_wrong = pc.copy()
    p_wrong[2] += 0.3       # 系统以为飞得更高
    r_bad = Estimator()._estimate(FakeStream(), box, None, p_wrong, Rc)
    # C/H 单独的误差理论值 (FOOT 较小 / HEAD 较大) —— 融合已不包含它们
    theo_foot = 0.3 / np.tan(np.arctan2(pc[2] - sv.ACTOR_Z_FEET, d_true))
    theo_head = 0.3 / np.tan(np.arctan2(pc[2] - sv.ACTOR_Z_HEAD, d_true))
    if r_bad is None:
        print("      被一致性护栏拦下 —— 也可接受 (存疑就闭嘴)")
        check("0.3m 高度误差被护栏拦下", 1.0, 1.0, 0.5)
    else:
        err = np.hypot(r_bad['x'] - ax, r_bad['y'])
        print("      未被拦下, 定位误差 %.4f m (C/H 若进融合: FOOT %.3f ~ HEAD %.3f)"
              % (err, theo_foot, theo_head))
        # J 自己把高度解出来 (压根不用 EKF 的 z), A 的水平投影也与高度无关 ——
        # 所以 0.3m 的高度误差对融合结果**零影响**。这正是 A/J 架构的核心优点,
        # 也是删掉 dh 高度自标定的底气。
        check("0.3m 高度误差零影响 (err < 5mm)", err, 0.0, 0.005)
    # J 本身用错的相机高度也照样解对 —— 这正是它能当自标定基准的原因
    dj, hj = joint_solve(box, Rc)
    print("      而 J 解出的高度 %.4f m 与真值 %.4f m 只差 %.6f m —— J 压根不用 EKF 的 z,"
          % (hj, pc[2], hj - pc[2]))
    check("J 不受注入的高度误差影响", hj, pc[2], 1e-4)

    # 5. 护栏
    print("\n[5] 护栏: 环带外 / 框截断 / 深度缺失")
    est = Estimator(min_range=3.0, max_range=12.0)
    r = run_case(est, (25.0, 0.0), (0.0, 0.0), 0.0)
    check("25m 远(超 max_range=12)应被拒", 0.0 if r is None else 1.0, 0.0, 0.5)
    r = run_case(est, (1.0, 0.0), (0.0, 0.0), 0.0)
    check("1m 近(人在画面外/超近)应被拒", 0.0 if r is None else 1.0, 0.0, 0.5)
    # 深度缺失时仍应能靠单目出解
    est2 = Estimator()
    r = run_case(est2, (6.5, 0.0), (0.0, 0.0), 0.0, with_depth=False)
    if r is not None:
        check("无深度时仍出解", 1.0, 1.0, 0.5)
        check("无深度时不含 DEPTH 位", float(r['method'] & sv.M_DEPTH), 0.0, 0.5)
        check("无深度时的融合误差", np.hypot(r['x'] - 6.5, r['y']), 0.0, 0.03)
    else:
        FAILED.append("无深度时出不了解")
        print("  [FAIL] 无深度时应该仍能靠单目出解")

    # 5b. 远距纯图像几何 (10~32m, 无深度): J 主力
    # 用户要求的核心能力: 深度只有 10m, 更远的人靠"画面中人的大小"估算位置。
    # J 用框上下沿的俯角差解距离, 对俯仰系统偏差免疫, 理想输入下仍应精确。
    print("\n[5b] 远距(10~32m)纯图像几何: J 联合解为主 (max_range=32)")
    est_far = Estimator(min_range=3.0, max_range=32.0)
    print("      %-7s %-11s %-9s %-9s %-7s %-7s"
          % ("距离", "定位误差", "sig_along", "sig_cross", "method", "框高px"))
    for d in (10.0, 14.0, 20.0, 26.0, 31.0):
        p_cam, _ = cam_pose((0.0, 0.0), 0.0)
        ax = p_cam[0] + d
        r = run_case(est_far, (ax, 0.0), (0.0, 0.0), 0.0, with_depth=False)
        if r is None:
            print("      %-7.1f (无解/被护栏拦下)" % d)
            FAILED.append("远距 d=%.1f 无解" % d)
            continue
        err = np.hypot(r['x'] - ax, r['y'])
        bh = r['v_bot'] - r['v_top']
        print("      %-7.1f %-11.5f %-9.3f %-9.3f %-7d %-7.0f"
              % (d, err, r['sigma_along'], r['sigma_cross'], r['method'], bh))
        check("远距 d=%.1f 纯图像定位误差(理想输入应精确为0)" % d, err, 0.0, 1e-6)
        check("远距 d=%.1f 单目链路 method=8(仅 JOINT)" % d, float(r['method']), 8.0, 0.5)
    # 远距 + 1 度俯仰误差: J俯仰敏感度受检验, 误差应远小于 C/H 的理论敏感度
    print("      远距注入 1 度俯仰误差 (C/H 的理论误差 20m 处约 1.5~2.3m):")
    for d in (14.0, 20.0):
        s = synth((d, 0.0), (0.0, 0.0), 0.0, pitch_deg=CAM_PITCH_DEG + 1.0,
                  with_depth=False)
        if s is None:
            continue
        box, _, p_cam, _ = s
        _, R_believed = cam_pose((0.0, 0.0), 0.0, CAM_PITCH_DEG)
        r = est_far._estimate(FakeStream(), box, None, p_cam, R_believed)
        if r is None:
            print("        d=%.1f 被护栏拦下 (可接受)" % d)
            continue
        err = np.hypot(r['x'] - d, r['y'])
        print("        d=%-5.1f 1度俯仰误差 -> 定位误差 %.3f m" % (d, err))
        check("远距 d=%.1f 俯仰1度误差 < 0.5m (J俯仰敏感度受检验)" % d,
              1.0 if err <= 0.5 else 0.0, 1.0, 0.5)

    # 5c. A-J 一致性否决: 深度块被遮挡物污染时必须降成单链
    # 半遮挡时深度中位数可能落在遮挡物上 (A 错), J 仍由框决定 (对) ——
    # 两条链路互查，分歧超阈值时降成单链；深度被否决就不再具备裁判资格。
    print("\n[5c] A-J 一致性否决 (深度块被遮挡物污染)")
    est_v = Estimator(min_range=3.0, max_range=32.0)
    p_cam, R_cw = cam_pose((0.0, 0.0), 0.0)
    ax = p_cam[0] + 7.0
    s = synth((ax, 0.0), (0.0, 0.0), 0.0, with_depth=True)
    if s is not None:
        box, depth, pc, Rc = s
        r_ok = est_v._estimate(FakeStream(), box, depth, pc, Rc)
        check("无污染时出解 (A 与 J 一致)", 0.0 if r_ok is None else 1.0, 1.0, 0.5)
        depth_sparse = np.full_like(depth, np.nan)
        uc, vc = int(0.5 * (box[0] + box[2])), int(0.5 * (box[1] + box[3]))
        depth_sparse[vc:vc + 2, uc:uc + 2] = 5.0
        r_sparse = est_v._estimate(FakeStream(), box, depth_sparse, pc, Rc)
        check("只有4个有效深度像素时不得形成 DEPTH 候选",
              1.0 if (r_sparse is not None
                      and not (r_sparse['method'] & sv.M_DEPTH)) else 0.0,
              1.0, 0.5)
        # 把整个躯干带的深度改成一个近处遮挡物的距离 (人前面 3m 处一堵墙)
        depth_bad = depth.copy()
        bh = box[3] - box[1]
        r0 = int(max(0, np.floor(box[1] + 0.25 * bh)))
        r1 = int(min(sv.IMG_H, np.ceil(box[1] + 0.55 * bh)))
        c0 = int(max(0, np.floor(0.5 * (box[0] + box[2]) - 0.15 * (box[2] - box[0]))))
        c1 = int(min(sv.IMG_W, np.ceil(0.5 * (box[0] + box[2]) + 0.15 * (box[2] - box[0]))))
        z_wall = 4.0     # 光轴分量 4m 的近墙
        depth_bad[r0:r1, c0:c1] = z_wall
        r_bad = est_v._estimate(FakeStream(), box, depth_bad, pc, Rc)
        check("深度块被墙污染时不得保留 A+J 双链",
              0.0 if (r_bad is None or
                      (r_bad['method'] & (sv.M_DEPTH | sv.M_JOINT))
                      != (sv.M_DEPTH | sv.M_JOINT)) else 1.0,
              0.0, 0.5)

        # v14 的核心回归：远处目标有一块 5m 假深度时，不能再由被污染后的
        # d_fused 判断“目标在近处”并留下 M_DEPTH；必须保留 J 的远距解。
        p_far, R_far = cam_pose((0.0, 0.0), 0.0)
        ax_far = p_far[0] + 30.0
        sf = synth((ax_far, 0.0), (0.0, 0.0), 0.0, with_depth=True)
        if sf is not None:
            bf, df, pcf, Rcf = sf
            bhf = bf[3] - bf[1]
            rr0 = int(max(0, np.floor(bf[1] + 0.25 * bhf)))
            rr1 = int(min(sv.IMG_H, np.ceil(bf[1] + 0.55 * bhf)))
            cc0 = int(max(0, np.floor(0.5 * (bf[0] + bf[2]) - 0.15 * (bf[2] - bf[0]))))
            cc1 = int(min(sv.IMG_W, np.ceil(0.5 * (bf[0] + bf[2]) + 0.15 * (bf[2] - bf[0]))))
            df[rr0:rr1, cc0:cc1] = 5.0
            rf = Estimator(min_range=3.0, max_range=40.0)._estimate(
                FakeStream(), bf, df, pcf, Rcf)
            check("30m 目标+5m 假深度应保留 J 而非 DEPTH",
                  float(rf['method']) if rf is not None else -1.0,
                  float(sv.M_JOINT), 0.5)
            # 框贴边时 J 不可用，不能因此让 5m 假深度单独通过；框高仍明确
            # 表明这是约 30m 的小目标。
            bt = bf.copy()
            bt[0], bt[2] = 0.0, float(sv.IMG_W)
            dt = np.full((sv.IMG_H, sv.IMG_W), np.nan, dtype=np.float32)
            bht = bt[3] - bt[1]
            tr0 = int(max(0, np.floor(bt[1] + 0.25 * bht)))
            tr1 = int(min(sv.IMG_H, np.ceil(bt[1] + 0.55 * bht)))
            tc0 = int(max(0, np.floor(0.5 * (bt[0] + bt[2]) - 0.15 * (bt[2] - bt[0]))))
            tc1 = int(min(sv.IMG_W, np.ceil(0.5 * (bt[0] + bt[2]) + 0.15 * (bt[2] - bt[0]))))
            dt[tr0:tr1, tc0:tc1] = 5.0
            rt = Estimator(min_range=3.0, max_range=40.0)._estimate(
                FakeStream(), bt, dt, pcf, Rcf)
            check("30m贴边框+5m假深度不得带DEPTH授权（可导航兜底）",
                  1.0 if rt is None or not (rt['method'] & sv.M_DEPTH) else 0.0, 1.0, 0.5)

    print("\n[5d] 图像/位姿时间同步: 缓存端点外不得把旧 pose 当作零跨度插值")
    pb = sv.PoseBuffer()
    q_ident = np.array([0.0, 0.0, 0.0, 1.0])
    pb.push(10.00, np.zeros(3), q_ident)
    pb.push(10.05, np.ones(3), q_ident)
    check("缓存区间内可插值", 1.0 if pb.query(10.025, 0.06) is not None else 0.0,
          1.0, 0.5)
    check("即使只晚30ms也不得使用端点 pose", 1.0 if pb.query(10.08, 0.06) is None else 0.0,
          1.0, 0.5)
    check("晚200ms必须拒绝旧 pose", 1.0 if pb.query(10.25, 0.06) is None else 0.0,
          1.0, 0.5)
    check("早200ms必须拒绝旧 pose", 1.0 if pb.query(9.80, 0.06) is None else 0.0,
          1.0, 0.5)

    # 6. 误差预算总结
    print("\n[6] 误差预算复核 (判据: 连续 15s 每帧 <1m => 需要 sigma <= 0.30m)")
    est = Estimator()
    for d in (5.0, 6.5, 8.0, 9.0):
        r = run_case(est, (d, 0.0), (0.0, 0.0), 0.0)
        if r is None:
            continue
        tot = np.hypot(r['sigma_along'], r['sigma_cross'])
        verdict = "OK" if tot <= 0.30 else "超预算"
        print("      d=%-5.1f sigma_total=%.3f m  3sigma=%.3f m  %s"
              % (d, tot, 3 * tot, verdict))

    print("\n" + "=" * 78)
    if FAILED:
        print("失败 %d 项:" % len(FAILED))
        for f in FAILED:
            print("   -", f)
        return 1
    print("全部通过。深度优先、单目互查与各项护栏在合成输入下自洽。")
    print("=" * 78)
    return 0


if __name__ == "__main__":
    sys.exit(main())
