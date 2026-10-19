#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按扫描时间的插值位姿将二维雷达转换到 world 帧，过滤友机附近的点。

云台水平稳定，偏航跟随机体；lidar_x/z 给出扫描面相对机体的偏移。
建图点统一投影到 plane_z，与 EGO 二维规划平面保持一致；真实高度点另发给高度记忆。
雷达和新鲜的机体高度带障碍合成一帧，避免 grid_map 逐帧重建时相互覆盖。"""
import os
import sys
import math
from collections import deque

import rospy
import numpy as np
import tf.transformations as tft
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from body_band_cloud import cloud_xyz, xyz_cloud


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PoseTrack:
    """里程计环形缓存：按扫描时间戳插值；超出最新样本时按最近速度和偏航角速度外推。"""

    def __init__(self, maxlen=80, max_extrapolate=0.15):
        self.samples = deque(maxlen=maxlen)   # (t, x, y, z, yaw)
        self.max_extrapolate = float(max_extrapolate)

    def push(self, t, x, y, z, yaw):
        if self.samples and t <= self.samples[-1][0]:
            if t < self.samples[-1][0] - 1.0:      # 仿真时间回跳: 重新开始
                self.samples.clear()
            else:
                return
        self.samples.append((float(t), float(x), float(y), float(z), float(yaw)))

    def at(self, t):
        """(x, y, z, yaw, yaw_rate) 或 None。"""
        s = self.samples
        if not s:
            return None
        if len(s) == 1 or t <= s[0][0]:
            a = s[0]
            return a[1], a[2], a[3], a[4], 0.0
        if t >= s[-1][0]:
            a, b = s[-2], s[-1]
            dt_ab = max(b[0] - a[0], 1e-3)
            dt = min(t - b[0], self.max_extrapolate)
            wz = _wrap(b[4] - a[4]) / dt_ab
            return (b[1] + (b[1] - a[1]) / dt_ab * dt, b[2] + (b[2] - a[2]) / dt_ab * dt,
                    b[3], _wrap(b[4] + wz * dt), wz)
        lo, hi = 0, len(s) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if s[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        a, b = s[lo], s[hi]
        dt_ab = max(b[0] - a[0], 1e-6)
        u = (t - a[0]) / dt_ab
        dyaw = _wrap(b[4] - a[4])
        return (a[1] + u * (b[1] - a[1]), a[2] + u * (b[2] - a[2]), a[3] + u * (b[3] - a[3]),
                _wrap(a[4] + u * dyaw), dyaw / dt_ab)


class LaserScanToWorldCloud:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.min_range = rospy.get_param("~min_range", 0.5)
        self.max_range = rospy.get_param("~max_range", 0.0)
        self.throttle = rospy.get_param("~throttle", 0.033)
        self.skip = int(max(1, rospy.get_param("~skip", 1)))
        self.lidar_x = rospy.get_param("~lidar_x", -0.04)
        self.plane_z = rospy.get_param("~plane_z", 5.0)
        self.drone_num = rospy.get_param("~drone_num", 6)
        self.peer_filter_radius = rospy.get_param("~peer_filter_radius", 1.0)
        self.ns_prefix = rospy.get_param("~ns_prefix", "typhoon_h480")
        self.lidar_z = rospy.get_param("~lidar_z", -0.1125)
        self.band_timeout = rospy.get_param("~band_timeout", 0.5)
        self.band_xy = np.empty((0, 2))
        self.band_rx = None
        # 位姿按扫描时间戳取 (见 PoseTrack)。pose_time_offset: odom 时间戳本身偏晚/偏早时的补偿 (s)
        self.pose_time_offset = float(rospy.get_param("~pose_time_offset", 0.0))
        self.track = PoseTrack(max_extrapolate=float(rospy.get_param("~max_pose_extrapolate", 0.15)))
        # 偏航角速度超过它时不发给 body_band 的高度记忆 (残余错位点会在记忆里留 15s)
        self.max_yaw_rate_3d = math.radians(float(rospy.get_param("~max_yaw_rate_3d_deg", 60.0)))

        self.odom = None
        self.last_pub = rospy.Time(0)
        self.peer_pos = {}   # 机索引 -> (x, y) 世界坐标, 用于机间点过滤

        self.pub = rospy.Publisher("~cloud_out", PointCloud2, queue_size=10)
        self.pub_3d = rospy.Publisher("~cloud_out_3d", PointCloud2, queue_size=2)
        self.pub_merged = rospy.Publisher("~merged_out", PointCloud2, queue_size=10)
        rospy.Subscriber("~band_in", PointCloud2, self.band_cb, queue_size=2, tcp_nodelay=True)
        rospy.Subscriber("~scan_in", LaserScan, self.scan_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber("~odom_in", Odometry, self.odom_cb, queue_size=10, tcp_nodelay=True)
        for i in range(self.drone_num):
            topic = "/%s_%d/odom_world_relay" % (self.ns_prefix, i)
            rospy.Subscriber(topic, Odometry, self._make_peer_cb(i), queue_size=2, tcp_nodelay=True)

        rospy.loginfo(
            "[lidar_cloud] Ready: frame=%s range=[%.2f, %s] throttle=%.3fs "
            "skip=%d lidar_x=%.3f plane_z=%.2f peer_filter=%.2fm x%d",
            self.frame_id, self.min_range,
            "inf" if self.max_range <= 0 else "%.2f" % self.max_range,
            self.throttle, self.skip, self.lidar_x, self.plane_z,
            self.peer_filter_radius, self.drone_num)

    def _make_peer_cb(self, idx):
        def cb(msg):
            self.peer_pos[idx] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        return cb

    def odom_cb(self, msg):
        self.odom = msg
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.track.push(msg.header.stamp.to_sec(), p.x, p.y, p.z,
                        tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])

    def band_cb(self, msg):
        self.band_xy = cloud_xyz(msg)[:, :2]
        self.band_rx = rospy.Time.now()

    def fresh_band(self, now):
        if self.band_rx is None or not (rospy.Duration(0) <= now - self.band_rx <=
                                        rospy.Duration(self.band_timeout)):
            return np.empty((0, 2))
        return self.band_xy

    def scan_cb(self, scan):
        if self.odom is None:
            return

        now = rospy.Time.now()
        if now - self.last_pub < rospy.Duration(self.throttle):
            return

        n = len(scan.ranges)
        if n == 0:
            return

        # 机体 yaw (云台把 roll/pitch 稳定到 0, 扫描面水平, 只用 yaw), 取扫描时刻的位姿
        pose = self.track.at(scan.header.stamp.to_sec() + self.pose_time_offset)
        if pose is None:
            return
        ox, oy, oz, yaw, yaw_rate = pose
        cy, sy = np.cos(yaw), np.sin(yaw)

        # 预计算每个光束角 (角度以雷达系 +x 为 0, 逆时针; 雷达系与机体系同向)
        # [::skip] 做角向下采样, 控制点云密度.
        angles = (scan.angle_min + np.arange(n, dtype=np.float64) * scan.angle_increment)[::self.skip]
        ranges = np.array(scan.ranges, dtype=np.float64)[::self.skip]
        valid = np.isfinite(ranges) & (ranges >= self.min_range)
        if self.max_range > 0:
            valid &= (ranges <= self.max_range)

        r = ranges[valid]
        ang = angles[valid]

        # 雷达系点 -> 机体系 (雷达挂点偏移 + 光束方向)
        px_body = self.lidar_x + r * np.cos(ang)
        py_body = r * np.sin(ang)

        # 机体系 -> world (绕 z 转 yaw + 平移)。高度统一压到 plane_z, 不做
        # 真实 z: ego 二维规划在 plane_z 上做碰撞检查, 真实扫描面高度会随机体
        # 升降漂移(跟踪 3.8m), 压到 plane_z 才能让障碍对规划器可见。
        wx = ox + cy * px_body - sy * py_body
        wy = oy + sy * px_body + cy * py_body
        wz = self.plane_z

        # 剔除距自身和友机中心 peer_filter_radius 内的点，避免动态机体进入静态地图。
        # 机间避障由 EGO broadcast_bspline 处理。
        if self.peer_pos and len(wx):
            keep = np.ones(len(wx), dtype=bool)
            for (px, py) in self.peer_pos.values():
                keep &= (np.hypot(wx - px, wy - py) >= self.peer_filter_radius)
            wx = wx[keep]
            wy = wy[keep]

        header = scan.header
        header.frame_id = self.frame_id
        lidar = np.column_stack([wx, wy, np.full_like(wx, wz)])
        if len(lidar):
            self.pub.publish(xyz_cloud(header, lidar))
            if self.pub_3d.get_num_connections() > 0 and abs(yaw_rate) <= self.max_yaw_rate_3d:
                true_z = oz + self.lidar_z
                self.pub_3d.publish(xyz_cloud(header, np.column_stack(
                    [wx, wy, np.full_like(wx, true_z)])))
        band = self.fresh_band(now)
        merged = lidar
        if len(band):
            merged = np.vstack([lidar, np.column_stack([band, np.full(len(band), wz)])])
        if len(merged):
            self.pub_merged.publish(xyz_cloud(header, merged))
        if len(lidar) or len(merged):
            self.last_pub = now

if __name__ == "__main__":
    rospy.init_node("laser_scan_to_world_cloud", anonymous=True)
    try:
        LaserScanToWorldCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
