#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""仿真就绪、放飞前执行只读检查：裁判话题发布者唯一，且活跃节点不订阅裁判或真值。
用法：rosrun fly_px4 preflight_check.py _strict:=true；严格模式遇 FAIL 非零退出。"""
import os
import re
import sys

import rospy

# 违规订阅的黑名单 (话题名的正则)
FORBIDDEN_TOPICS = [
    r'^/actor_\w+_info$',      # 裁判的输入 —— 只能发布, 不能订阅
    r'^/left_actors$',         # 裁判用 get_model_state 拿真值算出来的
    r'^/score$',
    r'^/time_usage$',
    r'^/gazebo/model_states$',
    r'^/gazebo/link_states$',
    r'.*ground_truth.*',
]
# 违规服务 (真值查询)
FORBIDDEN_SERVICES = [
    r'^/gazebo/get_model_state$',
    r'^/gazebo/get_link_state$',
]
# 这些节点允许出现在"订阅裁判话题"的名单里 —— 它们**就是**裁判/行人, 不是我们的程序
JUDGE_NODES = [r'score_cal', r'/actor_\d+$', r'control_actor']
# 开发期真值工具: 只要它在跑就报错(它绝不该出现在比赛轮次里)
DEV_ONLY_NODES = [r'dev_gt_error_logger', r'coordinate_accuracy_logger', r'inject_detections']

RESULTS = []


def rec(ok, name, detail=''):
    RESULTS.append((ok, name, detail))
    print('  [%s] %s%s' % ('OK  ' if ok else 'FAIL', name,
                           ('\n         ' + detail) if detail else ''))


def is_judge_node(n):
    return any(re.search(p, n) for p in JUDGE_NODES)


def alive_nodes():
    """ping master 注册的节点，仅检查活跃节点，将失效注册单独报告。"""
    import rosnode
    live = set()
    stale = set()
    try:
        names = rosnode.get_node_names()
    except Exception:
        return live, stale
    for n in names:
        try:
            if rosnode.rosnode_ping(n, max_count=1, verbose=False):
                live.add(n)
            else:
                stale.add(n)
        except Exception:
            stale.add(n)
    return live, stale


def main():
    rospy.init_node('preflight_check', anonymous=True)
    strict = bool(rospy.get_param('~strict', False))
    drone_num = int(rospy.get_param('~drone_num', 6))
    ns_prefix = rospy.get_param('~ns_prefix', 'typhoon_h480')

    import rosgraph
    import rosnode
    master = rosgraph.Master('/preflight_check')
    live, stale = alive_nodes()
    pubs, subs, srvs = master.getSystemState()
    pub_map = dict(pubs)
    sub_map = dict(subs)
    srv_map = dict(srvs)
    topic_types = dict(master.getTopicTypes())

    print('=' * 74)
    print('起飞前自检')
    print('=' * 74)

    # 1. 裁判话题的发布者数量
    print('\n[1] 裁判话题必须**恰好 1 个**发布者 (两个会互相打断 15 秒计数)')
    for c in ('green', 'blue', 'brown', 'white', 'red1', 'red2'):
        t = '/actor_%s_info' % c
        ps = [n for n in pub_map.get(t, [])]
        if len(ps) == 1:
            rec(True, '%-22s 1 个发布者 (%s)' % (t, ps[0]))
        elif not ps:
            rec(False, '%-22s **没有发布者**' % t,
                'swarm_fusion 没起来? 那一个目标都消不掉。')
        else:
            rec(False, '%-22s **%d 个发布者**: %s' % (t, len(ps), ps),
                '必须只留 swarm_fusion 一个。旧的 vision_V1.py 是不是还在跑?')

    # 2. 合规: 不得订阅裁判话题/真值
    print('\n[2] 合规: 我们的节点不得订阅裁判话题或 Gazebo 真值')
    print('    (规则: 裁判会实时监控所有节点的话题订阅以防违规订阅)')
    violations, zombie_hits = [], []
    for topic, subscribers in sub_map.items():
        for pat in FORBIDDEN_TOPICS:
            if re.search(pat, topic):
                for n in subscribers:
                    if is_judge_node(n) or n == rospy.get_name():
                        continue
                    if n in live:
                        violations.append((n, topic))
                    else:
                        zombie_hits.append((n, topic))
    if violations:
        for n, t in sorted(set(violations)):
            rec(False, '节点 %s 订阅了 %s' % (n, t), '必须去掉这个订阅')
    else:
        rec(True, '没有任何**活着的**节点订阅裁判话题或真值话题')
    if zombie_hits:
        # ROS master 不清理死节点的注册, 这些是僵尸残留 —— 不算违规, 但要提示,
        # 否则现场会因为一个早已退出的调试节点白白排查半天。
        print('         (以下是已死但仍留在 master 注册表里的僵尸, 不算违规;')
        print('          跑 `rosnode cleanup` 可清掉: %s)'
              % sorted(set(n for n, _ in zombie_hits)))
    if stale:
        print('    注: master 里有 %d 个 ping 不通的僵尸节点: %s'
              % (len(stale), sorted(stale)))

    # 真值服务的调用方查不到(ROS 不记录 service client), 所以退一步查:
    # 这些服务的**提供方**只应该是 gazebo 自己
    print('\n[3] 真值服务只应由 gazebo 提供 (我们不该有节点去 advertise 它)')
    bad_srv = []
    for s, providers in srv_map.items():
        for pat in FORBIDDEN_SERVICES:
            if re.search(pat, s):
                for n in providers:
                    if 'gazebo' not in n:
                        bad_srv.append((n, s))
    rec(not bad_srv, '真值服务提供方正常'
        if not bad_srv else '异常提供方: %s' % bad_srv)

    # 4. 开发期真值工具不得在跑
    print('\n[4] 开发期真值/注入工具不得出现在比赛轮次里')
    dev = [n for n in live if any(re.search(p, n) for p in DEV_ONLY_NODES)]
    rec(not dev, '没有开发期工具在跑'
        if not dev else '**正在跑**: %s —— 比赛前必须关掉' % dev)

    # 5. 相机下倾是否真的生效
    print('\n[5] 相机 25.0 度下倾: pose_relay 的姿态必须反解出 pitch=+25.0 度')
    print('    (四处约定之一错了坐标就整体偏; 这里查的是喂给 RACER 的那一路)')
    import numpy as np
    from geometry_msgs.msg import PoseStamped
    ok_cam = 0
    for i in range(drone_num):
        t = '/%s_%d/pose_relay' % (ns_prefix, i)
        try:
            m = rospy.wait_for_message(t, PoseStamped, timeout=3.0)
        except rospy.ROSException:
            rec(False, '%s 收不到消息' % t, 'odom_world_relay 没起来?')
            continue
        q = m.pose.orientation
        # pose_relay 发的是相机**光学系**姿态; 光轴 = 光学 +Z
        n = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        s = 2.0 / max(n, 1e-12)
        R02 = (q.x * q.z + q.w * q.y) * s
        R12 = (q.y * q.z - q.w * q.x) * s
        R22 = 1.0 - (q.x * q.x + q.y * q.y) * s
        elev = np.degrees(np.arctan2(R22, np.hypot(R02, R12)))
        good = abs(elev + 25.0) < 3.0
        ok_cam += 1 if good else 0
        rec(good, '%d 号机光轴俯角 %.2f 度 (期望 -25.0)' % (i + 1, elev),
            '' if good else '检查 sdf 的 <include><pose> pitch 与 relay 的 ~cam_pitch_deg')

    # 6. 图像流
    print('\n[6] 彩色图流 (检测的输入)')
    from sensor_msgs.msg import Image
    for i in range(drone_num):
        t = '/%s_%d/realsense/depth_camera/color/image_raw' % (ns_prefix, i)
        try:
            m = rospy.wait_for_message(t, Image, timeout=3.0)
            rec(m.width == 640 and m.height == 480,
                '%d 号机彩色图 %dx%d' % (i + 1, m.width, m.height))
        except rospy.ROSException:
            rec(False, '%d 号机彩色图收不到' % (i + 1), t)

    # 实际定位用相机 IMU，而不是理想云台姿态；每架机必须独立且有有效方向。
    if rospy.get_param('/swarm_vision/use_camera_imu', True):
        import math
        import numpy as np
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from camera_pose_provider import camera_imu_reference_rotation, quaternion_rotation
        from sensor_msgs.msg import Imu, CameraInfo
        imu_suffix = rospy.get_param('/swarm_vision/camera_imu_topic_suffix', 'imu_gazebo')
        reference = rospy.get_param('/swarm_vision/camera_imu_reference', 'initial')
        initial_rpy = rospy.get_param('/swarm_vision/camera_imu_initial_rpy_deg', [0., 25., 0.])
        reference_rotation = camera_imu_reference_rotation(reference, initial_rpy)
        print('\n[6a] 同期相机 IMU 与 CameraInfo')
        print('    原始IMU输入参考系: %s；初始相机rpy(deg): %s' % (reference, initial_rpy))
        for i in range(drone_num):
            ns = '/%s_%d' % (ns_prefix, i)
            topic = ns + '/' + imu_suffix.lstrip('/')
            try:
                m = rospy.wait_for_message(topic, Imu, timeout=3.0)
                q = m.orientation
                values = (q.x, q.y, q.z, q.w)
                age = (rospy.Time.now() - m.header.stamp).to_sec()
                good = (all(math.isfinite(v) for v in values)
                        and sum(v * v for v in values) > 1e-8
                        and m.orientation_covariance[0] != -1
                        and m.header.stamp.to_sec() > 0
                        and -0.05 <= age <= 0.5)
                publishers = [n for n in pub_map.get(topic, []) if n in live]
                rec(good and len(publishers) == 1,
                    '%d 号机相机 IMU %s age=%.3fs' % (i + 1, topic, age),
                    '' if good and len(publishers) == 1 else
                    '需有效采样时间、非零四元数和单一发布者；缺失时只允许方位导航。')
                if good:
                    # Match the algorithm's input-frame conversion, without
                    # editing the official IMU configuration or reading truth.
                    axis_x, axis_y, axis_z = (
                        reference_rotation @ quaternion_rotation(values))[:, 0]
                    elevation = math.degrees(math.atan2(axis_z, math.hypot(axis_x, axis_y)))
                    rec(abs(elevation + 25.0) < 10.0,
                        '%d 号机转换后 IMU 光轴俯角 %.2f 度（起飞前期望 -24）'
                        % (i + 1, elevation),
                        '保持官方传感器原件；检查算法参数camera_imu_reference：'
                        '旧XTDrone用initial，标准Noetic用world，以及初始安装姿态。'
                        if abs(elevation + 25.0) >= 10.0 else '')
            except rospy.ROSException:
                rec(False, '%d 号机相机 IMU 收不到' % (i + 1), topic)
            if rospy.get_param('/swarm_vision/require_camera_info', True):
                for stream in ('color', 'depth'):
                    topic = ns + '/realsense/depth_camera/%s/camera_info' % stream
                    try:
                        info = rospy.wait_for_message(topic, CameraInfo, timeout=3.0)
                        rec(info.width == 640 and info.height == 480
                            and all(math.isfinite(v) for v in info.K)
                            and info.K[0] > 0 and info.K[4] > 0,
                            '%d 号机 %s 内参有效' % (i + 1, stream), topic)
                    except rospy.ROSException:
                        rec(False, '%d 号机 %s CameraInfo 收不到' % (i + 1, stream), topic)
        print('\n[6b] 原子跟踪目标接口（状态时间与观测时间分离）')
        for i in range(drone_num):
            topic = '/tracking/state_%d' % (i + 1)
            rec(topic_types.get(topic) == 'track_msgs/TrackingTarget'
                and len(pub_map.get(topic, [])) == 1,
                '%d 号机 TrackingTarget 接口' % (i + 1), topic)

    # 7. 行人逃逸逻辑是否是活的
    print('\n[7] 行人逃逸判据的速度源是否存在')
    print('    (control_actor.py 要读 /xtdrone/*/ground_truth/odom 才能判"被追";')
    print('     本地没发布 => 逃逸是死的。赛场若有, 跟踪速度要压到 0.95m/s 以下)')
    gt = [t for t in pub_map if 'ground_truth' in t]
    if gt:
        rec(True, '存在真值 odom (%d 个) —— 逃逸逻辑可能是活的' % len(gt),
            '建议把 RACER 的 track_vel_locked 钳到 0.95')
    else:
        rec(True, '没有真值 odom —— 逃逸逻辑是死的, 跟踪速度限制可放宽')

    # 8. 原始 stereo 是否保留
    print('\n[8] 官方 RealSense 双目相机是否保留')
    st = [t for t in pub_map if 'stereo_camera' in t]
    if st:
        rec(True, '原始 stereo 已保留 (%d 个话题)' % len(st))
    else:
        rec(False, '未发现 stereo 话题',
            '确认加载官方完整RealSense模型及双目插件；不再删除传感器来降低负载。')

    # 汇总
    print('\n' + '=' * 74)
    bad = [r for r in RESULTS if not r[0]]
    if bad:
        print('%d 项未通过:' % len(bad))
        for _, n, _ in bad:
            print('   -', n)
        print('=' * 74)
        return 1 if strict else 0
    print('全部 %d 项通过, 可以放飞。' % len(RESULTS))
    print('=' * 74)
    return 0


if __name__ == '__main__':
    sys.exit(main())
