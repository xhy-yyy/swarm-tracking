#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将深度和真实高度雷达点记入体素，筛出机体高度带内的障碍并投影到 plane_z。

仅在跟踪或跟踪后归航请求激活时订阅深度和发布，升降时高度带延伸到目标高度。
当前雷达点由合并节点提供，不重复发送；深度看穿会削减旧体素命中数。
body_band_cloud 用于碰撞，body_band_overhead 提供上方结构证据，不作为普通视线障碍。
高度统一在 odom_world_relay 帧比较，本地目标高度加 offset_z 转换。"""
import json
import math
import re
import sys
import os
import threading
from collections import deque

import numpy as np

try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import Float32MultiArray, Header, String
except ImportError:      # 纯几何单测不需要 ROS
    rospy = None

_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from camera_pose_provider import normalized_quaternion, quaternion_rotation, slerp

# realsense_camera.sdf 深度: HFOV 60°, 640x480 -> fx = fy = 320/tan(30°)
DEFAULT_INTRINSICS = (554.255971, 554.255971, 320.0, 240.0, 640, 480)
# typhoon_h480_xhyyy.sdf realsense <include><pose> 平移 (机体系 FLU)
DEFAULT_CAMERA_OFFSET = (0.0205, 0.0, -0.20)


# 纯几何 (可单测)
def depth_from_image(encoding, height, width, data):
    """sensor_msgs/Image 数据 -> HxW float32 (m), 无效为 NaN。"""
    if encoding == '32FC1':
        return np.frombuffer(data, dtype=np.float32).reshape(height, width)
    if encoding == '16UC1':
        d = np.frombuffer(data, dtype=np.uint16).reshape(height, width).astype(np.float32) * 1e-3
        d[d <= 0.0] = np.nan
        return d
    raise ValueError('unsupported depth encoding: %s' % encoding)


def pool_nearest(depth, block):
    """block x block 取最近有效深度 (无效 = inf), 返回 (pooled, 列中心 u, 行中心 v)。"""
    h, w = depth.shape
    rows, cols = h // block, w // block
    d = depth[:rows * block, :cols * block]
    d = np.where(np.isfinite(d) & (d > 0.0), d, np.inf)
    pooled = d.reshape(rows, block, cols, block).min(axis=(1, 3))
    centre = (block - 1) / 2.0
    return pooled, np.arange(cols) * block + centre, np.arange(rows) * block + centre


def backproject(pooled, us, vs, fx, fy, cx, cy, min_range, max_range):
    """有效块 -> 光学系点 (N,3): x 右, y 下, z 前 (深度图是沿光轴的 z 深度)。"""
    valid = (pooled >= min_range) & (pooled <= max_range)
    rows, cols = np.nonzero(valid)
    d = pooled[rows, cols]
    return np.column_stack(((us[cols] - cx) / fx * d, (vs[rows] - cy) / fy * d, d))


def camera_position(body_pos, r_cam, offset=DEFAULT_CAMERA_OFFSET):
    """相机光心 = 机体位置 + 绕 yaw 旋转的安装偏移 (云台稳定 roll/pitch, yaw 跟随机体)。"""
    yaw = math.atan2(r_cam[1, 2], r_cam[0, 2])      # 光轴的水平朝向 = 机体 yaw
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy, oz = offset
    return np.asarray(body_pos, dtype=float) + np.array([c * ox - s * oy, s * ox + c * oy, oz])


def band_limits(z_body, z_target, below, above):
    """机体高度带: 当前与目标高度之间整段, 再加机体下/上包络。"""
    return min(z_body, z_target) - below, max(z_body, z_target) + above


def peer_keep_mask(points, peers, radius_xy, half_height):
    """剔除友机机体附近的点。peers: [(x, y, z), ...] 只含新鲜位置。"""
    keep = np.ones(len(points), dtype=bool)
    for px, py, pz in peers:
        near = ((np.hypot(points[:, 0] - px, points[:, 1] - py) < radius_xy) &
                (np.abs(points[:, 2] - pz) < half_height))
        keep &= ~near
    return keep


def min_filter(a, r):
    """(2r+1)^2 方窗最小值; 图像外按 0 (视为挡住) 处理, 靠边的体素不会被误判看穿。"""
    h, w = a.shape
    p = np.pad(a, r, mode='constant', constant_values=0.0)
    rows = np.min(np.stack([p[i:i + h, :] for i in range(2 * r + 1)]), axis=0)
    return np.min(np.stack([rows[:, j:j + w] for j in range(2 * r + 1)]), axis=0)


def xy_dedupe(points, res):
    """按 XY 栅格去重 (保留每格第一个点)。"""
    if len(points) == 0:
        return points
    cells = np.floor(points[:, :2] / res).astype(np.int64)
    _, first = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(first)]


class VoxelMemory:
    """障碍体素记忆: 键有序数组 + 最后命中时刻 + 命中次数 (numpy, 无 Python 逐点循环)。"""
    _XY_OFF = 1 << 20
    _Z_OFF = 1 << 11

    def __init__(self, res_xy=0.2, res_z=0.1, hit_cap=5):
        self.res_xy, self.res_z, self.hit_cap = float(res_xy), float(res_z), int(hit_cap)
        self.clear()

    def clear(self):
        self.keys = np.empty(0, dtype=np.int64)
        self.t = np.empty(0, dtype=np.float64)
        self.hits = np.empty(0, dtype=np.int16)

    def __len__(self):
        return len(self.keys)

    def keys_of(self, points):
        ix = np.floor(points[:, 0] / self.res_xy).astype(np.int64) + self._XY_OFF
        iy = np.floor(points[:, 1] / self.res_xy).astype(np.int64) + self._XY_OFF
        iz = np.floor(points[:, 2] / self.res_z).astype(np.int64) + self._Z_OFF
        return (ix << 33) | (iy << 12) | iz

    def centres(self, keys=None):
        k = self.keys if keys is None else keys
        ix = (k >> 33) - self._XY_OFF
        iy = ((k >> 12) & ((1 << 21) - 1)) - self._XY_OFF
        iz = (k & ((1 << 12) - 1)) - self._Z_OFF
        return np.column_stack(((ix + 0.5) * self.res_xy, (iy + 0.5) * self.res_xy,
                                (iz + 0.5) * self.res_z))

    def insert(self, points, stamp):
        """一帧的点: 每个体素本帧只计一次命中。返回本帧命中的键。"""
        if len(points) == 0:
            return np.empty(0, dtype=np.int64)
        new = np.unique(self.keys_of(points))
        if len(self.keys):
            idx = np.searchsorted(self.keys, new)
            at = np.minimum(idx, len(self.keys) - 1)
            found = self.keys[at] == new
            hit = at[found]
            self.t[hit] = np.maximum(self.t[hit], stamp)
            self.hits[hit] = np.minimum(self.hits[hit] + 1, self.hit_cap)
            add, pos = new[~found], idx[~found]
        else:
            add, pos = new, np.zeros(len(new), dtype=np.int64)
        if len(add):
            self.keys = np.insert(self.keys, pos, add)
            self.t = np.insert(self.t, pos, stamp)
            self.hits = np.insert(self.hits, pos, 1).astype(np.int16)
        return new

    def _keep(self, keep):
        self.keys, self.t, self.hits = self.keys[keep], self.t[keep], self.hits[keep]

    def prune(self, now, memory_sec, cx, cy, keep_radius):
        if not len(self.keys):
            return
        c = self.centres()
        keep = ((self.t >= now - memory_sec) & (self.t <= now + 1.0) &
                (np.hypot(c[:, 0] - cx, c[:, 1] - cy) <= keep_radius))
        self._keep(keep)

    def seen_through(self, cam_pos, r_cam, pooled, block, fx, fy, cx, cy,
                     min_range, max_range, margin, protect=None, max_radius=16):
        """体素投影范围内所有块都在其后方 margin 以外测到表面(或整块无回波 = 10m
        内没有表面) -> 被看穿, 命中次数减一, 减到 0 删除。本帧命中的体素不动。
        只看体素中心会把真实障碍的边缘(恰是檐口底沿)从侧面"看穿"掉, 所以按体素
        半对角线的投影半径取邻域最小深度; 太近(半径超过 max_radius 块)的不清。"""
        if not len(self.keys):
            return 0
        p = (self.centres() - np.asarray(cam_pos)) @ np.asarray(r_cam)   # 世界 -> 光学系
        z = p[:, 2]
        cand = (z >= min_range) & (z <= max_range)
        if protect is not None and len(protect):
            cand &= ~np.isin(self.keys, protect)
        if not np.any(cand):
            return 0
        idx = np.nonzero(cand)[0]
        u = fx * p[idx, 0] / z[idx] + cx
        v = fy * p[idx, 1] / z[idx] + cy
        rows, cols = pooled.shape
        col = np.floor(u / block).astype(np.int64)
        row = np.floor(v / block).astype(np.int64)
        inside = (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
        idx, col, row = idx[inside], col[inside], row[inside]
        half_diag = 0.5 * math.sqrt(2.0 * self.res_xy ** 2 + self.res_z ** 2)
        need = np.ceil(fx * half_diag / z[idx] / block).astype(np.int64) + 1
        ok = need <= max_radius
        idx, col, row, need = idx[ok], col[ok], row[ok], need[ok]
        if not len(idx):
            return 0
        radii = [1]
        while radii[-1] < need.max():
            radii.append(radii[-1] * 2)
        level = np.searchsorted(radii, need)
        seen = np.zeros(len(idx), dtype=bool)
        for li, r in enumerate(radii):
            sel = level == li
            if np.any(sel):
                seen[sel] = min_filter(pooled, r)[row[sel], col[sel]] > z[idx[sel]] + margin
        hit = idx[seen]
        if not len(hit):
            return 0
        self.hits[hit] -= 1
        self._keep(self.hits > 0)
        return int(len(hit))

    def select(self, now, memory_sec, cx, cy, radius, z_lo, z_hi, min_hits, live_sec=None):
        """高度带内(体素竖直范围与带相交)、足够可信、未过期的体素中心。
        live_sec: 跳过最近 live_sec 秒内刚命中的 (已在实时点云里)。"""
        if not len(self.keys):
            return np.empty((0, 3))
        c = self.centres()
        half = 0.5 * self.res_z
        keep = ((self.hits >= min_hits) & (self.t >= now - memory_sec) &
                (c[:, 2] + half >= z_lo) & (c[:, 2] - half <= z_hi) &
                (np.hypot(c[:, 0] - cx, c[:, 1] - cy) <= radius))
        if live_sec is not None:
            keep &= self.t < now - live_sec
        return c[keep]


class PoseHistory:
    """pose_relay 缓存: 区间内插值; 比最新样本新不超过 max_gap 时用最新样本
    (图像常比 MAVROS 位姿先到), 不做更远外推。"""

    def __init__(self, maxlen=200):
        self.buf = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def push(self, t, pos, quat):
        q = normalized_quaternion(quat)
        if q is None or not math.isfinite(t):
            return
        with self.lock:
            if self.buf and t < self.buf[-1][0]:
                if self.buf[-1][0] - t > 1.0:   # 仿真时钟回拨
                    self.buf.clear()
                else:
                    return
            self.buf.append((float(t), np.asarray(pos, dtype=float), q))

    def at(self, t, max_gap):
        with self.lock:
            rows = list(self.buf)
        if not rows or t < rows[0][0]:
            return None
        if t >= rows[-1][0]:
            if t - rows[-1][0] > max_gap:
                return None
            return rows[-1][1], quaternion_rotation(rows[-1][2])
        for i in range(len(rows) - 1, 0, -1):
            lo, hi = rows[i - 1], rows[i]
            if lo[0] <= t <= hi[0]:
                span = hi[0] - lo[0]
                if span > max_gap:
                    return None
                a = 0.0 if span <= 0 else (t - lo[0]) / span
                return (1 - a) * lo[1] + a * hi[1], quaternion_rotation(slerp(lo[2], hi[2], a))
        return None


def xyz_cloud(header, points):
    """(N,3) -> PointCloud2 (x/y/z float32, 与 create_cloud_xyz32 布局相同)。"""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = False
    msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return msg


def cloud_xyz(msg):
    """PointCloud2 -> (N,3) float64, 只支持 x/y/z float32 打头的常见布局。"""
    offsets = {f.name: f.offset for f in msg.fields if f.datatype == 7 and f.count == 1}
    if not all(k in offsets for k in ('x', 'y', 'z')) or msg.is_bigendian:
        return np.empty((0, 3))
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3))
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    out = np.empty((n, 3))
    for i, k in enumerate(('x', 'y', 'z')):
        out[:, i] = raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0]
    return out[np.all(np.isfinite(out), axis=1)]


# ROS 节点
class BodyBandCloud:
    def __init__(self):
        g = rospy.get_param
        self.ns = str(g('~ns', 'typhoon_h480_0')).strip('/')
        m = re.search(r'_(\d+)$', self.ns)
        self.self_index = int(g('~self_index', int(m.group(1)) if m else -1))
        self.ns_prefix = str(g('~ns_prefix', 'typhoon_h480'))
        self.drone_num = int(g('~drone_num', 6))
        self.offset_z = float(g('~offset_z', 0.0))        # = odom_world_relay 的 offset_z
        self.plane_z = float(g('~plane_z', 4.5))
        self.band_below = float(g('~band_below', 0.52))
        self.band_above = float(g('~band_above', 0.39))
        self.store_margin = float(g('~store_margin', 0.3))
        self.memory_sec = float(g('~memory_sec', 15.0))
        self.radius = float(g('~radius', 25.0))
        self.min_hits = int(g('~min_hits', 2))
        self.max_points = int(g('~max_points', 3000))
        self.publish_rate = float(g('~publish_rate', 10.0))
        self.request_timeout = float(g('~request_timeout', 1.0))
        self.odom_timeout = float(g('~odom_timeout', 0.3))
        self.enable_depth = bool(g('~enable_depth', True))
        self.enable_lidar_memory = bool(g('~enable_lidar_memory', True))
        self.lidar_live_sec = float(g('~lidar_live_sec', 0.25))
        self.lidar_z = float(g('~lidar_z', -0.1125))      # 扫描面相对机体原点
        self.depth_rate = float(g('~depth_rate', 10.0))
        self.depth_block = int(g('~depth_block', 4))
        self.depth_min_range = float(g('~depth_min_range', 0.3))
        self.depth_max_range = float(g('~depth_max_range', 9.0))
        self.clear_margin = float(g('~clear_margin', 0.4))
        self.self_radius = float(g('~self_radius', 0.6))
        self.pose_max_gap = float(g('~pose_max_gap', 0.1))
        self.camera_offset = tuple(float(v) for v in g('~camera_offset_body',
                                                        list(DEFAULT_CAMERA_OFFSET)))
        self.gimbal_tilt_max = float(g('~gimbal_tilt_max_deg', 6.0))
        self.peer_radius = float(g('~peer_radius', 1.0))
        self.peer_half_height = float(g('~peer_half_height', 0.7))
        self.peer_timeout = float(g('~peer_timeout', 0.3))
        cam = str(g('~camera_ns', '/%s/realsense/depth_camera' % self.ns))
        self.depth_topic = cam + '/depth/image_raw'
        self.info_topic = cam + '/depth/camera_info'

        self.lock = threading.Lock()
        self.depth_mem = VoxelMemory()
        self.lidar_mem = VoxelMemory()
        self.poses = PoseHistory()
        self.request = None            # (目标高度 本地, active, 接收时刻)
        self.odom = None               # (x, y, z, 接收时刻)
        self.peers = {}                # idx -> (x, y, z, 接收时刻)
        self.intrinsics = None
        self.gimbal = None             # (最大偏角 deg, 接收时刻)
        self.depth_sub = self.info_sub = None
        self.last_depth_stamp = -1e9
        self.last_prune = -1e9
        self.was_active = False
        self.stats = dict(frames=0, skipped_pose=0, skipped_gimbal=0, cleared=0)

        self.pub = rospy.Publisher('/%s/body_band_cloud' % self.ns, PointCloud2, queue_size=2)
        self.pub_overhead = rospy.Publisher('/%s/body_band_overhead' % self.ns, PointCloud2,
                                            queue_size=2)
        rospy.Subscriber('/%s/body_band_request' % self.ns, Float32MultiArray,
                         self._request_cb, queue_size=2)
        rospy.Subscriber('/%s/odom_world_relay' % self.ns, Odometry, self._odom_cb,
                         queue_size=2, tcp_nodelay=True)
        rospy.Subscriber('/%s/pose_relay' % self.ns, PoseStamped, self._pose_cb,
                         queue_size=20, tcp_nodelay=True)
        if self.enable_lidar_memory:
            rospy.Subscriber('/%s/lidar_cloud_world_3d' % self.ns, PointCloud2,
                             self._lidar_cb, queue_size=2, tcp_nodelay=True)
        rospy.Subscriber(str(g('~gimbal_status_topic', '')) or '/gimbal_recovery/status_%d'
                         % (self.self_index + 1), String, self._gimbal_cb, queue_size=2)
        for i in range(self.drone_num):
            if i != self.self_index:
                rospy.Subscriber('/%s_%d/odom_world_relay' % (self.ns_prefix, i), Odometry,
                                 self._make_peer_cb(i), queue_size=2, tcp_nodelay=True)
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self.publish_rate)), self._tick)
        rospy.loginfo('[body_band] %s: 高度带 -%.2f/+%.2fm 记忆 %.0fs 半径 %.0fm 深度=%s 雷达高度记忆=%s '
                      'offset_z=%.2f plane_z=%.2f', self.ns, self.band_below, self.band_above,
                      self.memory_sec, self.radius, self.enable_depth,
                      self.enable_lidar_memory, self.offset_z, self.plane_z)

    # 输入
    def _request_cb(self, msg):
        if len(msg.data) >= 2 and math.isfinite(msg.data[0]):
            self.request = (float(msg.data[0]), msg.data[1] > 0.5, rospy.get_time())

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z, rospy.get_time())

    def _make_peer_cb(self, idx):
        def cb(msg):
            p = msg.pose.pose.position
            self.peers[idx] = (p.x, p.y, p.z, rospy.get_time())
        return cb

    def _pose_cb(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.poses.push(msg.header.stamp.to_sec(), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _gimbal_cb(self, msg):
        try:
            d = json.loads(msg.data)
            roll, pitch = d.get('roll_deg'), d.get('pitch_deg')
            if roll is not None and pitch is not None:
                self.gimbal = (max(abs(float(roll)), abs(float(pitch))), rospy.get_time())
        except (ValueError, TypeError, AttributeError):
            pass

    def _info_cb(self, msg):
        k = msg.K
        if msg.width > 0 and msg.height > 0 and k[0] > 0 and k[4] > 0:
            self.intrinsics = (k[0], k[4], k[2], k[5], msg.width, msg.height)

    def _fresh_peers(self, now):
        return [(x, y, z) for (x, y, z, t) in list(self.peers.values())
                if 0.0 <= now - t <= self.peer_timeout]

    def _state(self, now):
        """(active, 本机 xyz, 目标高度[relay 帧]) ; 位置陈旧时 active=False。"""
        r, o = self.request, self.odom
        if r is None or o is None or not r[1] or not (0.0 <= now - r[2] <= self.request_timeout):
            return False, None, None
        if not (0.0 <= now - o[3] <= self.odom_timeout):
            return False, None, None
        return True, np.array(o[:3]), r[0] + self.offset_z

    def _store_mask(self, points, body, target):
        """只存可能进入高度带的点: 地面/行人这些永远低于带的不存。"""
        z_lo = min(body[2], target) - self.band_below - self.store_margin
        z_hi = max(body[2], target) + self.band_above + self.store_margin
        return (points[:, 2] >= z_lo) & (points[:, 2] <= z_hi)

    def _lidar_cb(self, msg):
        now = rospy.get_time()
        active, body, target = self._state(now)
        if not active:
            return
        pts = cloud_xyz(msg)
        if not len(pts):
            return
        pts = pts[self._store_mask(pts, body, target)]
        stamp = msg.header.stamp.to_sec() or now
        with self.lock:
            self.lidar_mem.insert(pts, min(stamp, now))

    def _depth_cb(self, msg):
        now = rospy.get_time()
        active, body, target = self._state(now)
        if not active:
            return
        stamp = msg.header.stamp.to_sec()
        if stamp - self.last_depth_stamp < 1.0 / max(1.0, self.depth_rate) - 1e-3 and \
                stamp >= self.last_depth_stamp:
            return
        self.last_depth_stamp = stamp
        if (self.gimbal is not None and 0.0 <= now - self.gimbal[1] <= 3.0 and
                self.gimbal[0] > self.gimbal_tilt_max):
            self.stats['skipped_gimbal'] += 1
            return
        pose = self.poses.at(stamp, self.pose_max_gap)
        if pose is None:
            self.stats['skipped_pose'] += 1
            return
        body_pos, r_cam = pose
        try:
            depth = depth_from_image(msg.encoding, msg.height, msg.width, msg.data)
        except ValueError as e:
            rospy.logwarn_throttle(5.0, '[body_band] %s', e)
            return
        fx, fy, cx, cy, w, h = self.intrinsics or DEFAULT_INTRINSICS
        if (w, h) != (msg.width, msg.height):
            s = msg.width / float(w)
            fx, fy, cx, cy = fx * s, fy * s, cx * s, cy * s
        b = self.depth_block
        pooled, us, vs = pool_nearest(depth, b)
        cam = camera_position(body_pos, r_cam, self.camera_offset)
        pts = cam + backproject(pooled, us, vs, fx, fy, cx, cy,
                                self.depth_min_range, self.depth_max_range) @ r_cam.T
        keep = self._store_mask(pts, body_pos, target)
        keep &= np.hypot(pts[:, 0] - body_pos[0], pts[:, 1] - body_pos[1]) >= self.self_radius
        keep &= peer_keep_mask(pts, self._fresh_peers(now), self.peer_radius, self.peer_half_height)
        pts = pts[keep]
        # 只清深度记忆 (路过的友机残影); 雷达记忆已滤友机, 靠过期淘汰, 不让相机从
        # 另一个角度把雷达扫到的檐口清掉
        with self.lock:
            hit = self.depth_mem.insert(pts, stamp)
            cleared = self.depth_mem.seen_through(cam, r_cam, pooled, b, fx, fy, cx, cy,
                                                  self.depth_min_range, self.depth_max_range,
                                                  self.clear_margin, protect=hit)
        self.stats['frames'] += 1
        self.stats['cleared'] += cleared

    # 深度订阅开关 + 发布
    def _set_depth(self, on):
        if on and self.depth_sub is None:
            self.info_sub = rospy.Subscriber(self.info_topic, CameraInfo, self._info_cb, queue_size=1)
            self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self._depth_cb,
                                              queue_size=1, buff_size=1 << 24, tcp_nodelay=True)
        elif not on and self.depth_sub is not None:
            self.depth_sub.unregister()
            self.info_sub.unregister()
            self.depth_sub = self.info_sub = None

    def _tick(self, _event):
        now = rospy.get_time()
        active, body, target = self._state(now)
        if self.enable_depth:
            self._set_depth(active)
        if active != self.was_active:
            rospy.loginfo('[body_band] %s %s (深度帧 %d, 位姿缺失跳过 %d, 云台偏角跳过 %d, 看穿清除 %d)',
                          self.ns, '启用: 跟踪相关阶段' if active else '停用: 巡逻',
                          self.stats['frames'], self.stats['skipped_pose'],
                          self.stats['skipped_gimbal'], self.stats['cleared'])
            self.was_active = active
        if not active:
            return
        z_lo, z_hi = band_limits(body[2], target, self.band_below, self.band_above)
        with self.lock:
            if now - self.last_prune >= 1.0 or now < self.last_prune:
                for mem in (self.depth_mem, self.lidar_mem):
                    mem.prune(now, self.memory_sec, body[0], body[1], self.radius + 5.0)
                self.last_prune = now
            depth = self.depth_mem.select(now, self.memory_sec, body[0], body[1], self.radius,
                                          z_lo, z_hi, self.min_hits)
            lidar = self.lidar_mem.select(now, self.memory_sec, body[0], body[1], self.radius,
                                          z_lo, z_hi, self.min_hits, live_sec=self.lidar_live_sec)
        # 偏离当前扫描面超过一个体素 = 在别的高度扫到的 (下降途中的檐口等)
        other = np.abs(lidar[:, 2] - (body[2] + self.lidar_z)) > self.lidar_mem.res_z
        pts = self._finish(np.vstack((depth, lidar)), body, now)
        overhead = self._finish(np.vstack((depth, lidar[other])), body, now)
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        self.pub.publish(xyz_cloud(header, pts))
        self.pub_overhead.publish(xyz_cloud(header, overhead))

    def _finish(self, pts, body, now):
        """剔除友机 -> XY 去重 -> 按距离封顶 -> 压到 plane_z。"""
        if len(pts):
            pts = pts[peer_keep_mask(pts, self._fresh_peers(now), self.peer_radius,
                                     self.peer_half_height)]
            pts = xy_dedupe(pts, self.depth_mem.res_xy)
            if len(pts) > self.max_points:
                d = np.hypot(pts[:, 0] - body[0], pts[:, 1] - body[1])
                pts = pts[np.argsort(d)[:self.max_points]]
            pts = pts.copy()
            pts[:, 2] = self.plane_z
        return pts.reshape(-1, 3)


if __name__ == '__main__':
    rospy.init_node('body_band_cloud')
    try:
        BodyBandCloud()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
