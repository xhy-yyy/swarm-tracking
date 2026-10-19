import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import body_band_cloud as bb

FX, FY, CX, CY, W, H = bb.DEFAULT_INTRINSICS
# 本地/relay 帧高度 (世界高度 - 0.22): 顶棚檐口带 3.186~3.435, 顶面 3.793, 地面 -0.22
FASCIA_X, FASCIA_Z = 7.0, (3.186, 3.435)
ROOF_Z, ROOF_X = 3.793, (7.0, 14.0)
GROUND_Z = -0.22


def camera_rotation(yaw, pitch_deg=25.0):
    """与 odom_world_relay 相同: Rz(yaw) @ Ry(pitch) @ 光学系->相机系。"""
    p = math.radians(pitch_deg)
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    opt2cam = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    return rz @ ry @ opt2cam


def render_canopy(cam, r_cam, extra_wall=None):
    """逐像素求射线与 檐口立面/棚顶/地面 的最近交点, 输出沿光轴的 z 深度 (10m 外 NaN)。"""
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    rays = np.stack(((u - CX) / FX, (v - CY) / FY, np.ones_like(u)), axis=-1)  # z=1: t 即深度
    d = rays @ r_cam.T
    best = np.full(u.shape, np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (FASCIA_X - cam[0]) / d[..., 0]
        z = cam[2] + t * d[..., 2]
        ok = (t > 0) & (z >= FASCIA_Z[0]) & (z <= FASCIA_Z[1])
        best = np.where(ok, np.minimum(best, t), best)
        t = (ROOF_Z - cam[2]) / d[..., 2]
        x = cam[0] + t * d[..., 0]
        ok = (t > 0) & (x >= ROOF_X[0]) & (x <= ROOF_X[1])
        best = np.where(ok, np.minimum(best, t), best)
        t = (GROUND_Z - cam[2]) / d[..., 2]
        best = np.where(t > 0, np.minimum(best, t), best)
        if extra_wall is not None:
            t = (extra_wall - cam[0]) / d[..., 0]
            best = np.where(t > 0, np.minimum(best, t), best)
    best[best > 10.0] = np.nan              # openni 插件 10m 外截断为 NaN
    return best.astype(np.float32)


def observe(mem, body, r_cam, depth, target, stamp, store_margin=0.3):
    pooled, us, vs = bb.pool_nearest(depth, 4)
    cam = bb.camera_position(body, r_cam)
    pts = cam + bb.backproject(pooled, us, vs, FX, FY, CX, CY, 0.3, 9.0) @ r_cam.T
    lo = min(body[2], target) - 0.52 - store_margin
    hi = max(body[2], target) + 0.39 + store_margin
    pts = pts[(pts[:, 2] >= lo) & (pts[:, 2] <= hi)]
    hit = mem.insert(pts, stamp)
    return pts, hit, pooled, cam


class BodyBandCloudTest(unittest.TestCase):
    def test_pool_keeps_nearest_pixel_and_marks_empty_blocks(self):
        d = np.full((8, 8), 6.0, dtype=np.float32)
        d[1, 2] = 2.0                         # 单像素细杆
        d[4:8, 4:8] = np.nan
        pooled, us, vs = bb.pool_nearest(d, 4)
        self.assertEqual(pooled[0, 0], 2.0)
        self.assertTrue(np.isinf(pooled[1, 1]))
        self.assertEqual(list(us), [1.5, 5.5])

    def test_principal_ray_hits_ground_25_deg_ahead(self):
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 4.5])
        cam = bb.camera_position(body, r)
        self.assertAlmostEqual(cam[2], 4.30)
        p = cam + r @ np.array([0.0, 0.0, 1.0]) * 5.0
        self.assertAlmostEqual(math.degrees(math.atan2(cam[2] - p[2], p[0] - cam[0])), 25.0, 6)
        r90 = camera_rotation(math.pi / 2)
        self.assertAlmostEqual(bb.camera_position(body, r90)[1], 0.0205)   # 偏移随 yaw 转

    def test_canopy_seen_from_cruise_is_kept_for_track_band(self):
        """巡航高度 4.5 从上方看到檐口/棚顶 -> 降到跟踪高度 3.2 后仍在机体高度带内。"""
        mem = bb.VoxelMemory()
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 4.5])
        depth = render_canopy(bb.camera_position(body, r), r)
        for k in range(2):
            observe(mem, body, r, depth, target=3.2, stamp=10.0 + 0.1 * k)
        # 巡航带 [3.98, 4.89]: 顶棚 (顶面 3.79) 不在带内 -> 巡航飞行不受影响
        lo, hi = bb.band_limits(4.5, 4.5, 0.52, 0.39)
        self.assertEqual(len(mem.select(10.2, 15, 0, 0, 25, lo, hi, 2)), 0)
        # 跟踪带 [2.68, 3.59]: 檐口立面在带内
        lo, hi = bb.band_limits(3.2, 3.2, 0.52, 0.39)
        pts = mem.select(10.2, 15, 0, 0, 25, lo, hi, 2)
        self.assertGreater(len(pts), 20)
        self.assertTrue(np.all(np.abs(pts[:, 0] - FASCIA_X) < 0.3))
        # 下降途中带延伸到目标: 棚顶面也算
        lo, hi = bb.band_limits(4.5, 3.2, 0.52, 0.39)
        pts = mem.select(10.2, 15, 0, 0, 25, lo, hi, 2)
        self.assertTrue(np.any(pts[:, 2] > 3.7))

    def test_track_altitude_depth_sees_no_eave_and_stores_no_ground(self):
        """跟踪高度 3.2: 相机光心 3.00 低于檐口带底 3.186, 深度看不到顶棚 (盲区);
        地面在存储窗口以下, 不入库。所以檐口只能靠巡航/下降时的观测记住。"""
        mem = bb.VoxelMemory()
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 3.2])
        depth = render_canopy(bb.camera_position(body, r), r)
        self.assertTrue(np.any(np.isfinite(depth)))      # 看得到地面
        observe(mem, body, r, depth, target=3.2, stamp=1.0)
        self.assertEqual(len(mem), 0)

    def test_single_frame_is_not_trusted(self):
        mem = bb.VoxelMemory()
        mem.insert(np.array([[5.0, 0.0, 3.2]]), 1.0)
        self.assertEqual(len(mem.select(1.0, 15, 0, 0, 25, 2.68, 3.59, 2)), 0)
        mem.insert(np.array([[5.05, 0.02, 3.21]]), 1.1)
        self.assertEqual(len(mem.select(1.1, 15, 0, 0, 25, 2.68, 3.59, 2)), 1)

    def test_seen_through_clears_ghost_but_not_real_wall(self):
        mem = bb.VoxelMemory()
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 3.2])
        cam = bb.camera_position(body, r)
        # 路过的友机残影, 整个体素都在画面内 (贴画面上沿的体素按看不全处理, 不清)
        ghost = np.array([[2.0, 0.0, 2.65]])
        real = np.array([[6.0, 1.0, 2.9]])
        for t in (1.0, 1.1):
            mem.insert(np.vstack((ghost, real)), t)
        depth = render_canopy(cam, r, extra_wall=6.05)  # 6m 处真有墙, 4m 处没东西
        pooled, _, _ = bb.pool_nearest(depth, 4)
        for _ in range(2):
            mem.seen_through(cam, r, pooled, 4, FX, FY, CX, CY, 0.3, 9.0, 0.4)
        c = mem.centres()
        self.assertFalse(np.any(np.abs(c[:, 0] - 2.0) < 0.2))
        self.assertTrue(np.any(np.abs(c[:, 0] - 6.0) < 0.2))

    def test_edge_voxel_below_eave_bottom_is_not_eroded(self):
        """檐口底沿的体素中心可能比真实底边低几厘米; 从下方斜看时射线擦着底边过去,
        中心像素看穿了, 但投影邻域里有檐口 -> 不能清。"""
        mem = bb.VoxelMemory()
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 2.6])
        cam = bb.camera_position(body, r)
        edge = np.array([[FASCIA_X - 0.05, 0.0, FASCIA_Z[0] - 0.04]])
        for t in (1.0, 1.1):
            mem.insert(edge, t)
        depth = render_canopy(cam, r)
        pooled, _, _ = bb.pool_nearest(depth, 4)
        for _ in range(6):
            mem.seen_through(cam, r, pooled, 4, FX, FY, CX, CY, 0.3, 9.0, 0.4)
        self.assertEqual(len(mem), 1)

    def test_min_filter_treats_outside_image_as_blocked(self):
        a = np.full((4, 5), 9.0)
        a[1, 1] = 2.0
        f = bb.min_filter(a, 1)
        self.assertEqual(f[0, 0], 0.0)
        self.assertEqual(f[2, 2], 2.0)
        self.assertEqual(f[2, 3], 9.0)

    def test_eave_above_camera_is_never_cleared_by_depth(self):
        """跟踪高度下檐口在相机上方, 深度看不到它, 也不能把雷达记住的檐口清掉。"""
        mem = bb.VoxelMemory()
        r = camera_rotation(0.0)
        body = np.array([0.0, 0.0, 3.2])
        cam = bb.camera_position(body, r)
        eave = np.array([[3.0, 0.0, 3.09]])              # 雷达在别的高度扫到的檐口
        for t in (1.0, 1.1):
            mem.insert(eave, t)
        pooled = np.full((H // 4, W // 4), np.inf)        # 整幅无回波
        for _ in range(5):
            mem.seen_through(cam, r, pooled, 4, FX, FY, CX, CY, 0.3, 9.0, 0.4)
        self.assertEqual(len(mem), 1)

    def test_live_lidar_points_are_not_duplicated(self):
        mem = bb.VoxelMemory()
        mem.insert(np.array([[5.0, 0.0, 3.09]]), 1.0)
        mem.insert(np.array([[5.0, 0.0, 3.09]]), 1.05)
        mem.insert(np.array([[8.0, 0.0, 3.30]]), 0.2)
        mem.insert(np.array([[8.0, 0.0, 3.30]]), 0.3)
        pts = mem.select(1.1, 15, 0, 0, 25, 2.68, 3.59, 2, live_sec=0.25)
        self.assertEqual(len(pts), 1)
        self.assertAlmostEqual(pts[0, 0], 8.1)

    def test_prune_by_age_and_distance(self):
        mem = bb.VoxelMemory()
        mem.insert(np.array([[1.0, 0.0, 3.0], [40.0, 0.0, 3.0]]), 10.0)
        mem.insert(np.array([[2.0, 0.0, 3.0]]), 1.0)
        mem.prune(12.0, 5.0, 0.0, 0.0, 30.0)
        self.assertEqual(len(mem), 1)

    def test_peer_mask_respects_height(self):
        pts = np.array([[0.5, 0.0, 3.2], [0.5, 0.0, 1.0], [3.0, 0.0, 3.2]])
        keep = bb.peer_keep_mask(pts, [(0.0, 0.0, 3.2)], 1.0, 0.7)
        self.assertEqual(list(keep), [False, True, True])

    def test_pose_history_interpolates_and_bounds_extrapolation(self):
        h = bb.PoseHistory()
        h.push(1.0, (0, 0, 3), (0, 0, 0, 1))
        h.push(1.1, (1, 0, 3), (0, 0, 0, 1))
        self.assertAlmostEqual(h.at(1.05, 0.2)[0][0], 0.5)
        self.assertAlmostEqual(h.at(1.15, 0.1)[0][0], 1.0)   # 图像比位姿先到: 用最新
        self.assertIsNone(h.at(1.5, 0.1))
        self.assertIsNone(h.at(0.5, 0.1))

    @unittest.skipIf(bb.rospy is None, 'ROS python packages unavailable')
    def test_cloud_roundtrip(self):
        from std_msgs.msg import Header
        pts = np.array([[1.0, 2.0, 4.5], [-3.5, 0.25, 4.5]])
        back = bb.cloud_xyz(bb.xyz_cloud(Header(frame_id='world'), pts))
        np.testing.assert_allclose(back, pts, atol=1e-6)


if __name__ == '__main__':
    unittest.main()
