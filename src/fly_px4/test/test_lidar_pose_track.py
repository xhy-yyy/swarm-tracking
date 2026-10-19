#!/usr/bin/env python3
"""laser_scan_to_world_cloud.PoseTrack: 扫描点按扫描时刻的位姿转世界系。"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from laser_scan_to_world_cloud import PoseTrack  # noqa: E402


def wall_point_error(yaw_used, yaw_true, rng):
    """同一束雷达, 用 yaw_used 和 yaw_true 转到世界系后落点相差多少米。"""
    return 2.0 * rng * abs(math.sin(0.5 * (yaw_used - yaw_true)))


class PoseTrackTest(unittest.TestCase):
    def setUp(self):
        # 100 deg/s 匀速偏航, 2 m/s 前进, 30Hz odom
        self.rate = math.radians(100.0)
        self.track = PoseTrack()
        for k in range(30):
            t = 10.0 + k / 30.0
            self.track.push(t, 2.0 * (t - 10.0), 0.0, 3.2, self.rate * (t - 10.0))

    def test_interpolates_between_samples(self):
        x, y, z, yaw, wz = self.track.at(10.5)
        self.assertAlmostEqual(x, 1.0, places=6)
        self.assertAlmostEqual(yaw, self.rate * 0.5, places=6)
        self.assertAlmostEqual(wz, self.rate, places=6)

    def test_scan_newer_than_last_odom_is_extrapolated(self):
        last = 10.0 + 29 / 30.0
        x, _, _, yaw, wz = self.track.at(last + 0.04)
        self.assertAlmostEqual(yaw, self.rate * (last + 0.04 - 10.0), places=6)
        self.assertAlmostEqual(x, 2.0 * (last + 0.04 - 10.0), places=6)
        self.assertAlmostEqual(wz, self.rate, places=6)
        # 旧做法 (直接用最新 odom) 在 15m 处的落点误差 vs 新做法
        old = wall_point_error(self.rate * (last - 10.0), self.rate * (last + 0.04 - 10.0), 15.0)
        new = wall_point_error(yaw, self.rate * (last + 0.04 - 10.0), 15.0)
        self.assertGreater(old, 1.0)
        self.assertLess(new, 0.01)

    def test_extrapolation_is_bounded(self):
        last = 10.0 + 29 / 30.0
        _, _, _, yaw, _ = self.track.at(last + 5.0)
        self.assertAlmostEqual(yaw, _wrap_ref(self.rate * (last + self.track.max_extrapolate - 10.0)), places=6)

    def test_yaw_wraps_across_pi(self):
        tr = PoseTrack()
        tr.push(0.0, 0, 0, 0, math.radians(179.0))
        tr.push(0.1, 0, 0, 0, math.radians(-179.0))
        _, _, _, yaw, wz = tr.at(0.05)
        self.assertAlmostEqual(abs(yaw), math.pi, places=6)
        self.assertAlmostEqual(wz, math.radians(20.0), places=6)

    def test_time_jump_back_resets(self):
        tr = PoseTrack()
        tr.push(100.0, 1, 0, 0, 0.0)
        tr.push(101.0, 2, 0, 0, 0.0)
        tr.push(5.0, 9, 0, 0, 0.0)
        self.assertEqual(tr.at(5.0)[0], 9)
        tr.push(4.99, 7, 0, 0, 0.0)          # 乱序小回退: 忽略
        self.assertEqual(len(tr.samples), 1)

    def test_empty_returns_none(self):
        self.assertIsNone(PoseTrack().at(1.0))


def _wrap_ref(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


if __name__ == '__main__':
    unittest.main()
