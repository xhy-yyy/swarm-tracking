#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""六机 YOLO 批量检测与图像时刻定位。

按 RGB 时间戳匹配机体位姿、相机 IMU 和深度帧。
可信深度独立输出；单目仅用于几何校验和导航。"""

import os
import sys
import threading
import time
import json
from collections import Counter, deque
from pathlib import Path

import numpy as np
try:
    import rospy
    from sensor_msgs.msg import Image, Imu, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int8, String
    from track_msgs.msg import Detection, DetectionArray
except ImportError:
    # 离线几何测试允许缺少 ROS 环境。
    rospy = None

# catkin devel 包装器不会自动加入脚本目录。
_SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from camera_pose_provider import (CameraModel, CameraPoseProvider,
                                  TemporalPoseBuffer, depth_surface_cluster)

# 回退相机内参与 realsense_camera.sdf 保持一致；彩色和深度共用像素模型。
CAMERA_FX = 554.255971
CAMERA_FY = 554.255971
CAMERA_CX = 320.0
CAMERA_CY = 240.0
IMG_W, IMG_H = 640, 480

# 相机光心偏移（机体 FLU，米）；SDF joint pose 是锚点，不能累加为外参。
CAM_OFFSET_BODY = np.array([0.0205, 0.0, -0.20])

# 行人模型固定几何（米），用于脚/头射线求交。
ACTOR_Z_ORIGIN = 1.0191
ACTOR_Z_FEET = 0.159
ACTOR_Z_HEAD = 1.879
ACTOR_HEIGHT = 1.720


# 类别顺序与模型一致；两个红人共用 red，由融合按位置区分。
# red1/red2 是视觉航迹槽，不绑定裁判 actor 编号。
CLS_NAMES = ['green', 'blue', 'brown', 'white', 'red']
DEFAULT_MODEL = os.environ.get('SWARM_WEIGHTS', str(Path(__file__).resolve().parents[3] / 'assets/weights/best_11s_new.pt'))

# Detection.method 位掩码。
M_DEPTH = 1          # 可信深度（体表到人体中心修正）
M_MONO_FOOT = 2      # 框底与脚平面求交
M_MONO_HEAD = 4      # 框顶与头平面求交
M_JOINT = 8          # 脚/头联合解（距离、相机高度）

# 深度按光轴 z_c 截断，超范围像素为 NaN。
DEPTH_CUTOFF = 10.0


def quat_to_R(x, y, z, w):
    """四元数 -> 3x3 旋转矩阵 (不依赖 tf, 这样 conda 环境也能跑)"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def parse_corr_list(s, default=1.0, n=6):
    """逗号分隔的浮点串 -> 长度 n 的系数表。解析失败/不足则补默认值。"""
    out = [default] * n
    try:
        vals = [float(x) for x in str(s).split(',')]
        for i in range(min(len(vals), n)):
            out[i] = vals[i]
    except Exception:
        pass
    return out


def imgmsg_to_bgr(msg):
    """sensor_msgs/Image -> HxWx3 uint8 BGR (只处理 gazebo 会发的那几种编码)"""
    if msg.encoding == 'rgb8':
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return a[:, :, ::-1].copy()
    if msg.encoding == 'bgr8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3).copy()
    raise ValueError('不支持的彩色编码: %s' % msg.encoding)


def imgmsg_to_depth(msg):
    """sensor_msgs/Image -> HxW float32, 单位 m。无效值为 NaN。"""
    if msg.encoding == '32FC1':
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return d.copy()
    if msg.encoding == '16UC1':
        # 16UC1 单位为毫米，0 表示无效。
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width).astype(np.float32) * 1e-3
        d[d <= 0.0] = np.nan
        return d
    raise ValueError('不支持的深度编码: %s' % msg.encoding)


class PoseBuffer:
    """按图像时间戳插值位姿，拒绝跨越断流区间。"""

    def __init__(self, maxlen=400):
        self.cache = TemporalPoseBuffer(maxlen=maxlen)

    def push(self, t, pos, quat):
        return self.cache.push(t, pos, quat)

    def query(self, t, max_gap=0.06):
        """返回 (pos, R, yaw, span) 或 None。span 是插值跨度, 太大说明位姿流断了。"""
        sample = self.cache.query(t, max_gap)
        if sample is None:
            return None
        pos, R, span = sample['pos'], sample['R'], sample['span']
        # pose_relay 为光学 RDF 姿态；从光轴水平投影恢复机体 yaw。
        axis = R @ np.array([0.0, 0.0, 1.0])
        yaw = float(np.arctan2(axis[1], axis[0]))
        return pos, R, yaw, span


class DroneStream:
    """一架飞机的图像/深度/位姿/模式"""

    def __init__(self, drone_id, ns, node):
        self.drone_id = drone_id
        self.ns = ns
        self.node = node
        self.pose = PoseBuffer()
        self.camera_pose = CameraPoseProvider(
            CAM_OFFSET_BODY, node.pose_position_sigma,
            node.camera_attitude_sigma_deg, node.lever_arm_sigma,
            imu_reference=getattr(node, 'camera_imu_reference', 'world'),
            imu_initial_rpy_deg=getattr(node, 'camera_imu_initial_rpy_deg', (0., 25., 0.)))
        self.camera_pose.lever_arm_model = node.lever_arm_model
        if node.gimbal_model_sdf:
            from camera_gimbal_kinematics import load_gimbal_geometry
            self.camera_pose.geometry = load_gimbal_geometry(node.gimbal_model_sdf)
        self.lock = threading.Lock()
        self.clock_lock = threading.Lock()
        self.last_ros_time = None
        self.updated = threading.Event()
        self.color_model = None
        self.depth_model = None
        self.frame = None        # (bgr, stamp_sec, seq, timing)
        self.frame_seq = -1
        self.last_done_seq = -1
        self.depth_buf = deque(maxlen=6)   # (float32 HxW, stamp_sec)
        self.mode = 0
        self.depth_sub = None

        cam = '/%s/realsense/depth_camera' % ns
        self.color_topic = cam + '/color/image_raw'
        self.depth_topic = cam + '/depth/image_raw'

        # 图像需要足够的 socket 缓冲，避免分片增加延迟。
        self.color_sub = rospy.Subscriber(
            self.color_topic, Image, self._color_cb,
            queue_size=1, buff_size=2 ** 24, tcp_nodelay=True)
        rospy.Subscriber('/%s/pose_relay' % ns, PoseStamped, self._pose_cb,
                         queue_size=20, tcp_nodelay=True)
        rospy.Subscriber('/%s/%s' % (ns, node.body_odom_topic_suffix),
                         Odometry, self._body_cb, queue_size=30, tcp_nodelay=True)
        rospy.Subscriber('/%s/%s' % (ns, node.camera_imu_topic_suffix),
                         Imu, self._imu_cb, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber(cam + '/color/camera_info', CameraInfo,
                         self._color_info_cb, queue_size=1)
        rospy.Subscriber(cam + '/depth/camera_info', CameraInfo,
                         self._depth_info_cb, queue_size=1)
        rospy.Subscriber('/tracking/mode_%d' % drone_id, Int8, self._mode_cb,
                         queue_size=2)

    def _epoch_valid(self, stamp):
        now = rospy.Time.now().to_sec()
        with self.clock_lock:
            if self.last_ros_time is not None and now < self.last_ros_time - 1e-3:
                self.camera_pose.clear()
                self.pose.cache.clear()
                with self.lock:
                    self.depth_buf.clear()
                    self.frame = None
                rospy.logwarn('[vision] drone%d ROS时间回退，清空传感器缓存', self.drone_id)
            self.last_ros_time = now
        # 时间回退后拒绝超前于新时钟的旧缓存样本。
        return bool(np.isfinite(stamp) and 0 <= stamp <= now + 0.1)

    def _color_cb(self, msg):
        receipt_stamp = rospy.Time.now().to_sec()
        decode_start = time.monotonic()
        if not self._epoch_valid(msg.header.stamp.to_sec()):
            return
        try:
            bgr = imgmsg_to_bgr(msg)
        except ValueError as e:
            rospy.logwarn_throttle(10.0, '[vision] drone%d 彩色图解码失败: %s',
                                   self.drone_id, e)
            return
        with self.lock:
            self.frame_seq += 1
            self.frame = (bgr, msg.header.stamp.to_sec(), self.frame_seq,
                          dict(image_receipt_stamp=receipt_stamp,
                               image_ready_stamp=rospy.Time.now().to_sec(),
                               image_decode_wall_ms=1000.*(time.monotonic()-decode_start)))
        self.updated.set()

    def _depth_cb(self, msg):
        if not self._epoch_valid(msg.header.stamp.to_sec()):
            return
        try:
            d = imgmsg_to_depth(msg)
        except ValueError as e:
            rospy.logwarn_throttle(10.0, '[vision] drone%d 深度图解码失败: %s',
                                   self.drone_id, e)
            return
        with self.lock:
            self.depth_buf.append((d, msg.header.stamp.to_sec()))
        self.updated.set()

    def _pose_cb(self, msg):
        if not self._epoch_valid(msg.header.stamp.to_sec()):
            return
        p = msg.pose
        args = (msg.header.stamp.to_sec(),
                np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))
        self.pose.push(*args)
        self.camera_pose.relay.push(*args)
        self.updated.set()

    def _body_cb(self, msg):
        if not self._epoch_valid(msg.header.stamp.to_sec()):
            return
        if msg.header.frame_id != 'world':
            rospy.logwarn_throttle(5., '[vision] %s body odom必须为world，不能再叠spawn', self.ns)
            return
        p = msg.pose.pose
        cov = np.asarray(msg.pose.covariance, dtype=float)
        if not np.all(np.isfinite(cov)):
            return
        sigma = float(np.sqrt(max(0., cov[0], cov[7], cov[14])))
        omega = msg.twist.twist.angular
        self.camera_pose.body.push(
            msg.header.stamp.to_sec(), [p.position.x, p.position.y, p.position.z],
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w],
            sigma, [omega.x, omega.y, omega.z])
        self.updated.set()

    def _imu_cb(self, msg):
        if (not self._epoch_valid(msg.header.stamp.to_sec())
                or msg.orientation_covariance[0] < 0
                or not np.all(np.isfinite(msg.orientation_covariance))):
            return
        q, w = msg.orientation, msg.angular_velocity
        cov = msg.orientation_covariance
        orientation_sigma = float(np.sqrt(max(0., cov[0], cov[4], cov[8])))
        self.camera_pose.imu.push(msg.header.stamp.to_sec(), [0., 0., 0.],
                                  [q.x, q.y, q.z, q.w], orientation_sigma, [w.x, w.y, w.z])
        self.updated.set()

    def _info_cb(self, msg, attr):
        try:
            model = CameraModel(msg.width, msg.height, msg.K, msg.D,
                                msg.distortion_model, msg.header.frame_id)
        except (ValueError, np.linalg.LinAlgError):
            rospy.logwarn_throttle(5., '[vision] %s 无效或不支持的CameraInfo', self.ns)
            return
        with self.lock:
            setattr(self, attr, model)
        self.updated.set()

    def _color_info_cb(self, msg):
        self._info_cb(msg, 'color_model')

    def _depth_info_cb(self, msg):
        self._info_cb(msg, 'depth_model')

    def _mode_cb(self, msg):
        new = int(msg.data)
        if new == self.mode:
            return
        self.mode = new
        # 仅跟踪时订阅深度，减少六路常驻反序列化开销。
        if new >= 1 and self.depth_sub is None:
            self.depth_sub = rospy.Subscriber(
                self.depth_topic, Image, self._depth_cb,
                queue_size=1, buff_size=2 ** 24, tcp_nodelay=True)
            rospy.loginfo('[vision] drone%d 进入跟踪, 订阅深度流', self.drone_id)
        elif new < 1 and self.depth_sub is not None:
            self.depth_sub.unregister()
            self.depth_sub = None
            with self.lock:
                self.depth_buf.clear()
            rospy.loginfo('[vision] drone%d 退出跟踪, 退订深度流', self.drone_id)

    def take_frame(self):
        """取一帧未处理过的图像; 没有新帧返回 None"""
        with self.lock:
            if self.frame is None or self.frame[2] == self.last_done_seq:
                return None
            return self.frame

    def mark_done(self, seq):
        with self.lock:
            self.last_done_seq = seq

    def get_depth(self, stamp, max_dt=None, with_dt=False):
        # 按彩色图时间戳选最近深度帧，并限制两者时间差。
        if max_dt is None:
            max_dt = self.node.depth_max_dt
        with self.lock:
            if not self.depth_buf:
                return None

            d, t = min(self.depth_buf, key=lambda item: abs(item[1] - stamp))
        dt = t - stamp
        if abs(dt) > max_dt:
            return None
        return (d, dt) if with_dt else d

    def query_inputs(self, stamp, deadline):
        """Bound waiting by wall time; Gazebo pause cannot extend it indefinitely."""
        while True:
            self.updated.clear()
            pose = self.camera_pose.query(stamp, self.node.pose_max_gap,
                                          self.node.imu_max_gap, self.node.use_camera_imu)
            depth_pair = self.get_depth(stamp, with_dt=True)
            with self.lock:
                model, depth_model = self.color_model, self.depth_model
            calibrated = model is not None and model.aligned_with(depth_model)
            if (pose is not None and (pose['depth_valid'] or not self.node.use_camera_imu)
                    and (self.mode < 1 or depth_pair is not None)
                    and (calibrated or not self.node.require_camera_info)):
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self.updated.wait(min(remaining, .005))
        return pose, depth_pair, model, calibrated


class SwarmVision:
    def __init__(self):
        self.drone_num = int(rospy.get_param('~drone_num', 6))
        self.ns_prefix = rospy.get_param('~ns_prefix', 'typhoon_h480')
        self.model_path = rospy.get_param('~model_path', DEFAULT_MODEL)
        self.conf = float(rospy.get_param('~conf', 0.6))
        self.red_conf = float(rospy.get_param('~red_conf', 0.8))
        for name, value in (('conf', self.conf), ('red_conf', self.red_conf)):
            if not np.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError('%s must be finite and between 0 and 1' % name)
        self.rate_hz = float(rospy.get_param('~rate', 10.0))
        self.device = rospy.get_param('~device', 'cuda:0')
        self.half = bool(rospy.get_param('~half', True))
        self.max_batch = int(rospy.get_param('~max_batch', 6))
        self.pose_max_gap = float(rospy.get_param('~pose_max_gap', 0.06))
        self.depth_max_dt = float(rospy.get_param('~depth_max_dt', 0.002))
        self.imu_max_gap = float(rospy.get_param('~imu_max_gap', 0.02))
        self.sync_wait_sec = max(0., float(rospy.get_param('~sync_wait_sec', 0.025)))
        self.use_camera_imu = bool(rospy.get_param('~use_camera_imu', True))
        self.require_camera_info = bool(rospy.get_param('~require_camera_info', True))
        self.camera_imu_topic_suffix = str(rospy.get_param('~camera_imu_topic_suffix', 'imu_gazebo')).lstrip('/')
        # XTDrone 插件使用 initial 姿态参考系，标准 Noetic 插件使用 world。
        self.camera_imu_reference = str(rospy.get_param('~camera_imu_reference', 'initial'))
        self.camera_imu_initial_rpy_deg = rospy.get_param(
            '~camera_imu_initial_rpy_deg', [0., 25., 0.])
        from camera_pose_provider import camera_imu_reference_rotation
        camera_imu_reference_rotation(self.camera_imu_reference, self.camera_imu_initial_rpy_deg)
        self.body_odom_topic_suffix = str(rospy.get_param('~body_odom_topic_suffix', 'odom_world_relay')).lstrip('/')
        self.pose_position_sigma = max(0., float(rospy.get_param('~pose_position_sigma', 0.10)))
        self.camera_attitude_sigma_deg = max(0., float(rospy.get_param('~camera_attitude_sigma_deg', 0.5)))
        self.lever_arm_sigma = max(0., float(rospy.get_param('~lever_arm_sigma', 0.03)))
        self.lever_arm_model = str(rospy.get_param('~lever_arm_model', 'gimbal_kinematics'))
        if self.lever_arm_model not in ('gimbal_kinematics', 'body_rotation'):
            raise ValueError('lever_arm_model must be gimbal_kinematics or body_rotation')
        self.gimbal_model_sdf = str(rospy.get_param('~gimbal_model_sdf', ''))
        self.fallback_model = CameraModel(IMG_W, IMG_H,
                                        [CAMERA_FX, 0., CAMERA_CX,
                                         0., CAMERA_FY, CAMERA_CY, 0., 0., 1.])
        # 检测距离门限（米）；超出深度范围的单目结果仅用于导航。
        self.min_range = float(rospy.get_param('~min_range', 3.0))
        self.max_range = float(rospy.get_param('~max_range', 100.0))
        self.publish_annotated = bool(rospy.get_param('~publish_annotated', False))

        # 深度体表到人体中心的距离修正（米）。
        self.body_half_depth = float(rospy.get_param('~body_half_depth', 0.15))

        # 深度为米制光轴分量，逐像素反投影使用单位尺度。
        self.depth_scale = float(rospy.get_param('~depth_scale', 1.0))
        # 超出可信深度范围时降级为单目导航。
        self.depth_trust_range = float(rospy.get_param('~depth_trust_range', 12.0))

        self.mono_scale = float(rospy.get_param('~mono_scale', 1.0))
        # 单目测距不确定度放大系数，用于导航和一致性校验。
        self.mono_sigma_scale = float(rospy.get_param('~mono_sigma_scale', 2.0))
        # 一致性否决后可保留较可信链路；关闭时丢弃整帧。
        self.veto_publish = bool(rospy.get_param('~veto_publish', True))
        # 有效像素数、覆盖率和 MAD 同时合格才生成 M_DEPTH。
        self.depth_min_pixels = int(rospy.get_param('~depth_min_pixels', 20))
        self.depth_min_valid_ratio = float(
            rospy.get_param('~depth_min_valid_ratio', 0.10))
        self.depth_max_mad = float(rospy.get_param('~depth_max_mad', 0.35))
        # 深度与脚/头联合解的近距离分歧上限（米）。
        self.depth_joint_max_diff = float(
            rospy.get_param('~depth_joint_max_diff', 0.70))
        self.depth_size_max_ratio = float(
            rospy.get_param('~depth_size_max_ratio', 2.5))

        # 从框底上收 foot_margin 个框高，减少框内地面对脚行的影响。
        self.foot_margin = float(rospy.get_param('~foot_margin', 0.12))

        # 深度类别尺度默认全 1；单目类别修正仅用于导航与互查。
        self.cls_depth_corr = parse_corr_list(
            rospy.get_param('~cls_depth_corr', '1,1,1,1,1,1'))
        self.cls_mono_corr = parse_corr_list(
            rospy.get_param('~cls_mono_corr', '1,1,1,1,1,1'))

        # 方位角补偿（度），默认不补偿。
        self.bearing_corr_deg = float(rospy.get_param('~bearing_corr_deg', 0.0))

        self.pub = rospy.Publisher('/vision/detections', DetectionArray, queue_size=5)
        self.diag_pub = rospy.Publisher('/vision/diagnostics', String, queue_size=20)
        self.health_lock = threading.Lock()
        self.tracking_input_counts = Counter()
        self.batch_timings = deque(maxlen=100)

        # 标注图输出到 /vision/annotated_N。
        self.cv2 = None
        self.ann_pubs = []
        if self.publish_annotated:
            try:
                import cv2
                self.cv2 = cv2
                self.ann_pubs = [
                    rospy.Publisher('/vision/annotated_%d' % (i + 1), Image, queue_size=2)
                    for i in range(self.drone_num)
                ]
                rospy.loginfo('[vision] 标注图发布已开启: /vision/annotated_1..%d',
                              self.drone_num)
            except ImportError:
                rospy.logwarn('[vision] 未找到 cv2, 关闭标注图发布')
                self.publish_annotated = False

        rospy.loginfo('[vision] 载入 YOLO: %s', self.model_path)
        if not os.path.isfile(self.model_path):
            rospy.logfatal('[vision] 模型文件不存在: %s', self.model_path)
            raise SystemExit(1)
        from ultralytics import YOLO
        import torch
        self.torch = torch
        self.model = YOLO(self.model_path)
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            rospy.logwarn('[vision] CUDA 不可用, 退回 CPU (会很慢)')
            self.device = 'cpu'
            self.half = False
        self.model.to(self.device)
        self.names = self.model.names

        self.streams = [
            DroneStream(i + 1, '%s_%d' % (self.ns_prefix, i), self)
            for i in range(self.drone_num)
        ]
        rospy.loginfo('[vision] 原始相机IMU参考系=%s，初始物理姿态rpy(deg)=%s',
                      self.camera_imu_reference, self.camera_imu_initial_rpy_deg)

        self.rr_cursor = 0
        self.n_pub = 0
        self.n_drop_pose = 0
        self.n_drop_range = 0
        self.n_drop_consist = 0
        self.n_veto_downgrade = 0

        rospy.loginfo('[vision] %d 机, conf=%.2f, red_conf=%.2f, rate=%.1fHz, device=%s, half=%s, '
                      'range=[%.1f, %.1f]m',
                      self.drone_num, self.conf, self.red_conf, self.rate_hz, self.device, self.half,
                      self.min_range, self.max_range)
        rospy.Timer(rospy.Duration(10.0), self._stats_cb)

    def _ray_world(self, R_cw, u, v, camera_model=None):
        if camera_model is None:
            r = np.array([(u - CAMERA_CX) / CAMERA_FX,
                          (v - CAMERA_CY) / CAMERA_FY, 1.0])
        else:
            r = camera_model.rays(u, v)
        r /= np.linalg.norm(r)
        return R_cw @ r

    @staticmethod
    def _corr(corr_list, cls_id):
        c = int(cls_id)
        return corr_list[c] if 0 <= c < len(corr_list) else 1.0

    @staticmethod
    def _plane_hit(p_cam, r_w, z_plane):
        """射线与水平面求交 -> (x, y, 斜距) 或 None (射线没朝下)"""
        if r_w[2] > -1e-3:
            return None
        t = (z_plane - p_cam[2]) / r_w[2]
        if t <= 0.0:
            return None
        p = p_cam + t * r_w
        return float(p[0]), float(p[1]), float(t)

    def _estimate(self, st, box, depth, p_cam, R_cw, cls_id=0,
                  camera_model=None, pose_quality=None, depth_authorized=True,
                  diagnostics=None):
        """返回定位结果或 None。

        A 为可信深度，J 为脚/头联合解，C/H 为水平面求交。
        可信 A 独立决定最终坐标；单目仅供导航及几何一致性校验。"""
        dc = self._corr(self.cls_depth_corr, cls_id)   # 深度类别尺度
        mc = self._corr(self.cls_mono_corr, cls_id)    # 单目类别尺度
        x1, y1, x2, y2 = box
        u_c = 0.5 * (x1 + x2)
        v_top, v_bot = y1, y2
        v_ctr = 0.5 * (y1 + y2)
        box_w = x2 - x1
        box_h = y2 - y1
        if not np.all(np.isfinite(box)) or box_w <= 0 or box_h <= 0:
            if diagnostics is not None:
                diagnostics['reason'] = 'invalid_box'
            return None
        if camera_model is None:
            camera_model = CameraModel(IMG_W, IMG_H,
                                       [CAMERA_FX, 0., CAMERA_CX, 0., CAMERA_FY,
                                        CAMERA_CY, 0., 0., 1.])
        fx, fy, cx, cy = (camera_model.fx, camera_model.fy,
                          camera_model.cx, camera_model.cy)
        img_w, img_h = camera_model.width, camera_model.height

        v_foot = v_bot - self.foot_margin * box_h
        truncated = (x1 <= 1.0 or x2 >= img_w - 1.0 or
                     v_top <= 1.0 or v_bot >= img_h - 2.0)

        p_use = p_cam

        r_ctr = self._ray_world(R_cw, u_c, v_ctr, camera_model)
        # 图像机头反馈独立于深度簇方向；relay 回退不用于快速机头反馈。
        image_bearing = float(np.arctan2(r_ctr[1], r_ctr[0]))
        image_error_x = float(u_c - camera_model.cx)
        image_bearing_valid = bool(
            pose_quality is not None
            and pose_quality.get('source') == 'camera_imu'
            and pose_quality.get('depth_valid', False)
            and np.all(np.isfinite(r_ctr))
            and np.hypot(r_ctr[0], r_ctr[1]) > 1e-6
            and np.isfinite(image_error_x))

        # 利用竖直消失点修正脚/头像素列，补偿下倾相机的透视偏差。
        d_c = R_cw.T @ np.array([0.0, 0.0, 1.0])
        u_bot, u_top = u_c, u_c
        if abs(d_c[2]) > 1e-6:
            u_vp = cx + fx * d_c[0] / d_c[2]
            v_vp = cy + fy * d_c[1] / d_c[2]
            if abs(v_vp - v_ctr) > 1e-6:
                slope_u = (u_vp - u_c) / (v_vp - v_ctr)
                u_bot = u_c + slope_u * (v_foot - v_ctr)
                u_top = u_c + slope_u * (v_top - v_ctr)
        r_bot = self._ray_world(R_cw, u_bot, v_foot, camera_model)
        r_top = self._ray_world(R_cw, u_top, v_top, camera_model)
        # 只修方向，不改变深度和脚/头联合解的距离。
        bearing = float(np.arctan2(r_ctr[1], r_ctr[0])) \
            + np.deg2rad(self.bearing_corr_deg)

        cands = []     # (水平距离, sigma, method_bit)
        method = 0

        def pose_sigma(d, dz):
            """按 1 度姿态不确定度折算的测距 sigma: dd/da = d*(1/tan a + tan a)"""
            a = np.arctan2(max(dz, 0.05), max(d, 0.1))
            return max(d * (1.0 / np.tan(a) + np.tan(a)) * np.pi / 180.0, 0.08)

        # A：躯干深度反投影，并修正到人体中心。
        depth_d = None
        surface_xy, depth_diag = depth_surface_cluster(
            depth if depth_authorized else None, box, camera_model, p_use, R_cw,
            self.depth_min_pixels, self.depth_min_valid_ratio, self.depth_max_mad,
            DEPTH_CUTOFF)
        if not depth_authorized:
            depth_diag['reason'] = 'pose_or_calibration_not_authorized'
        if diagnostics is not None:
            diagnostics['depth'] = depth_diag
        depth_bearing = None
        if surface_xy is not None:
            delta = surface_xy - p_use[:2]
            d_surf = float(np.linalg.norm(delta))
            d = (d_surf + self.body_half_depth) * self.depth_scale * dc
            depth_diag.update(surface_range_m=d_surf,
                              centre_offset_m=self.body_half_depth,
                              depth_scale_used=self.depth_scale,
                              class_depth_scale_used=dc, corrected_range_m=d)
            if d > .5:
                depth_bearing = float(np.arctan2(delta[1], delta[0])) + np.deg2rad(self.bearing_corr_deg)
                depth_d = d
                # 深度噪声保留稳健表面散布下限，位姿与时间误差在输出前合入。
                sd = max(.12+.02*d, float(depth_diag['surface_scatter']))
                cands.append((d, sd, M_DEPTH))
                method |= M_DEPTH

        # J：由脚/头射线联合求解距离和相机高度，不依赖输入高度。
        d_joint = None
        if not truncated and box_h > 8.0:
            def depression(r):
                horiz = np.hypot(r[0], r[1])
                if horiz < 1e-9 or r[2] >= -1e-9:
                    return None
                return float(np.arctan2(-r[2], horiz))
            a_f, a_h = depression(r_bot), depression(r_top)
            if a_f is not None and a_h is not None:
                cot_f, cot_h = 1.0 / np.tan(a_f), 1.0 / np.tan(a_h)
                den = cot_f - cot_h
                # 分母接近零时拒绝病态解。
                if abs(den) > 0.15:
                    h_joint = (ACTOR_Z_FEET * cot_f - ACTOR_Z_HEAD * cot_h) / den
                    d_joint = (h_joint - ACTOR_Z_FEET) * cot_f
                    d_joint *= self.mono_scale * mc
                    if 0.5 < d_joint < 60.0 and 1.0 < h_joint < 12.0:
                        method |= M_JOINT
                        # 单目像素噪声随距离平方传播。
                        sj = (0.10 + 0.0012 * d_joint * d_joint) * self.mono_sigma_scale
                        cands.append((d_joint, sj, M_JOINT))
                    else:
                        d_joint = None

        # J 不可用时，用框高排除远目标中的近距离假深度。
        if depth_d is not None and d_joint is None and box_h > 8.0:
            d_size = fy * ACTOR_HEIGHT / box_h
            if (d_size > self.depth_trust_range
                    and d_size > self.depth_size_max_ratio * depth_d):
                cands = [c for c in cands if c[2] != M_DEPTH]
                method &= ~M_DEPTH
                self.n_drop_consist += 1

        # A/J 都无解时仍允许脚/头平面求交兜底。
        if cands:
            w = np.array([1.0 / (s * s) for _, s, _ in cands])
            d_fused = float(np.dot(w, [c[0] for c in cands]) / w.sum())
            sigma_along = float(np.sqrt(1.0 / w.sum()))
        else:
            d_fused, sigma_along = 0., 0.

        # A/J 不一致时只保留可信链路。
        if d_joint is not None and (method & M_DEPTH):
            d_a = next(c[0] for c in cands if c[2] == M_DEPTH)
            tol = min(3.0 * sigma_along + 0.15 * d_fused,
                      self.depth_joint_max_diff)
            separation = np.hypot(d_joint*np.cos(bearing)-d_a*np.cos(depth_bearing),
                                  d_joint*np.sin(bearing)-d_a*np.sin(depth_bearing))
            if separation > tol:
                if not self.veto_publish:
                    self.n_drop_consist += 1
                    if diagnostics is not None:
                        diagnostics['reason'] = 'A_J_inconsistent'
                    return None
                self.n_veto_downgrade += 1
                # 按原始候选距离选择链路，避免坏深度影响选择。
                keep = (M_JOINT if max(d_joint, d_a) > self.depth_trust_range
                        else M_DEPTH)
                sel = [c for c in cands if c[2] == keep]
                if not sel:
                    sel = [c for c in cands if c[2] == M_DEPTH] or cands
                cands = sel
                method = keep
                d_fused = float(cands[0][0])
                sigma_along = float(cands[0][1])
                depth_diag['consistency'] = 'A_J_downgrade'
                d_joint = None

        # J 与脚/头平面解互查。
        if d_joint is not None:
            ch = []
            if not truncated:
                hit = self._plane_hit(p_use, r_bot, ACTOR_Z_FEET)
                if hit is not None:
                    d = float(np.hypot(hit[0] - p_use[0], hit[1] - p_use[1])) * self.mono_scale * mc
                    ch.append((d, pose_sigma(d, p_use[2] - ACTOR_Z_FEET), M_MONO_FOOT))
            if v_top > 1.0:
                hit = self._plane_hit(p_use, r_top, ACTOR_Z_HEAD)
                if hit is not None:
                    d = float(np.hypot(hit[0] - p_use[0], hit[1] - p_use[1])) * self.mono_scale * mc
                    ch.append((d, pose_sigma(d, p_use[2] - ACTOR_Z_HEAD), M_MONO_HEAD))
            if ch:
                wch = np.array([1.0 / (s * s) for _, s, _ in ch])
                d_ch = float(np.dot(wch, [c[0] for c in ch]) / wch.sum())
                tol = 3.0 * max(sigma_along, 0.15) + 0.25 * d_fused
                if abs(d_joint - d_ch) > tol:
                    if not self.veto_publish:
                        self.n_drop_consist += 1
                        if diagnostics is not None:
                            diagnostics['reason'] = 'J_planes_inconsistent'
                        return None

                    self.n_veto_downgrade += 1

        # 无深度或联合解时，使用脚/头平面求交作为导航兜底。
        if not (method & (M_DEPTH | M_JOINT)):
            if not truncated:
                hit = self._plane_hit(p_use, r_bot, ACTOR_Z_FEET)
                if hit is not None:
                    d = float(np.hypot(hit[0] - p_use[0], hit[1] - p_use[1])) * self.mono_scale * mc
                    cands.append((d, pose_sigma(d, p_use[2] - ACTOR_Z_FEET), M_MONO_FOOT))
                    method |= M_MONO_FOOT
            if v_top > 1.0:
                hit = self._plane_hit(p_use, r_top, ACTOR_Z_HEAD)
                if hit is not None:
                    d = float(np.hypot(hit[0] - p_use[0], hit[1] - p_use[1])) * self.mono_scale * mc
                    cands.append((d, pose_sigma(d, p_use[2] - ACTOR_Z_HEAD), M_MONO_HEAD))
                    method |= M_MONO_HEAD
            if not cands:
                if diagnostics is not None:
                    diagnostics['reason'] = 'no_geometric_candidate'
                return None
            w = np.array([1.0 / (s * s) for _, s, _ in cands])
            d_fused = float(np.dot(w, [c[0] for c in cands]) / w.sum())
            sigma_along = float(np.sqrt(1.0 / w.sum()))

        # 保留 M_DEPTH 时坐标只取纯深度；M_JOINT 仅表示互查通过。
        if method & M_DEPTH:
            depth_sel = next((c for c in cands if c[2] == M_DEPTH), None)
            if depth_sel is None:
                # method 标志必须与实际坐标来源一致。
                method &= ~M_DEPTH
            else:
                d_fused = float(depth_sel[0])
                sigma_along = float(depth_sel[1])
                bearing = depth_bearing

        if not (self.min_range <= d_fused <= self.max_range):
            self.n_drop_range += 1
            if diagnostics is not None:
                diagnostics['reason'] = 'range_gate'
            return None

        x = p_use[0] + d_fused * np.cos(bearing)
        y = p_use[1] + d_fused * np.sin(bearing)

        sigma_cross = max(d_fused * np.pi / 180.0, 0.05)
        if pose_quality is not None:
            # 合入位姿、杆臂及图像/深度不同步造成的不确定度。
            rgb_dt = abs(float(pose_quality.get('rgb_depth_dt') or 0.))
            depth_time_sigma = rgb_dt*(pose_quality.get('speed', 0.)+2.)
            extra = (pose_quality['position_sigma']**2
                     + pose_quality['lever_arm_sigma']**2
                     + pose_quality['time_position_sigma']**2
                     + depth_time_sigma**2
                     + (d_fused*pose_quality['attitude_sigma'])**2)
            sigma_along = float(np.sqrt(sigma_along**2 + extra))
            sigma_cross = float(np.sqrt(sigma_cross**2 + extra))
            if not pose_quality['depth_valid']:
                sigma_along = max(sigma_along, .75)
                sigma_cross = max(sigma_cross, .35)
        return dict(x=float(x), y=float(y), bearing=bearing,
                    sigma_along=sigma_along, sigma_cross=sigma_cross,
                    slant=float(np.hypot(d_fused, p_use[2] - ACTOR_Z_ORIGIN)),
                    method=method, truncated=bool(truncated),
                    image_bearing_valid=image_bearing_valid,
                    image_bearing=image_bearing, image_error_x=image_error_x,
                    u=float(u_c), v_top=float(v_top), v_bot=float(v_bot),
                    box_w=float(box_w), depth_diagnostics=depth_diag)

    def _pick_batch(self):
        """跟踪机优先进入每批，其余相机按轮询填充剩余位置。"""
        tracking, idle = [], []
        for st in self.streams:
            f = st.take_frame()
            if f is None:
                continue
            (tracking if st.mode >= 1 else idle).append((st, f))
        batch = tracking[:self.max_batch]
        room = self.max_batch - len(batch)
        if room > 0 and idle:
            n = len(idle)
            order = [idle[(self.rr_cursor + k) % n] for k in range(n)]
            batch.extend(order[:room])
            self.rr_cursor = (self.rr_cursor + min(room, n)) % max(n, 1)
        return batch

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                self._tick()
            except Exception:
                rospy.logerr_throttle(5.0, '[vision] tick 异常:\n%s',
                                      __import__('traceback').format_exc())
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def _filter_confidence(self, boxes_data):
        """红色使用独立阈值；定位、发布和标注共用同一批过滤后的框。"""
        boxes, clss, confs = boxes_data
        thresholds = np.where(clss == CLS_NAMES.index('red'), self.red_conf, self.conf)
        keep = np.isfinite(confs) & (confs >= thresholds)
        return boxes[keep], clss[keep], confs[keep]

    def _publish_annotated(self, st, bgr, stamp, res, boxes_data=None):
        """把一帧彩色图画上 YOLO 框后发到 /vision/annotated_{drone_id}。"""
        if not self.publish_annotated or self.cv2 is None:
            return
        cv2 = self.cv2
        ann = bgr.copy()
        if res.boxes is not None and len(res.boxes):
            if boxes_data is None:
                boxes_data = self._filter_confidence((res.boxes.xyxy.cpu().numpy(),
                                                     res.boxes.cls.cpu().numpy().astype(int),
                                                     res.boxes.conf.cpu().numpy()))
            boxes, clss, confs = boxes_data
            for b, c_id, cf in zip(boxes, clss, confs):
                x1, y1, x2, y2 = (int(round(v)) for v in b)
                name = CLS_NAMES[c_id] if 0 <= c_id < len(CLS_NAMES) else str(int(c_id))
                label = '%s %.2f' % (name, cf)
                cv2.rectangle(ann, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(ann, label, (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
        msg = Image()
        msg.height, msg.width = ann.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = ann.shape[1] * 3
        msg.data = ann.tobytes()
        msg.header.stamp = rospy.Time.from_sec(stamp)
        msg.header.frame_id = 'camera'
        self.ann_pubs[st.drone_id - 1].publish(msg)

    def _tick(self):
        batch = self._pick_batch()
        if not batch:
            return
        batch_start = time.monotonic()
        batch_start_stamp = rospy.Time.now().to_sec()
        frames = [f[0] for _, f in batch]
        results = self.model.predict(frames, imgsz=(IMG_H, IMG_W),
                                     conf=min(self.conf, self.red_conf),
                                     half=self.half, device=self.device,
                                     verbose=False)
        inference_done = time.monotonic()
        inference_done_stamp = rospy.Time.now().to_sec()
        now = rospy.Time.now()
        out = DetectionArray()
        out.header.stamp = now
        out.header.frame_id = 'world'
        earliest = None
        # 同步等待预算由整批共享，使用墙钟防止仿真暂停延长等待。
        sync_deadline = time.monotonic() + self.sync_wait_sec
        diagnostic_calls, annotation_jobs = [], []

        def record_diagnostic(*args, **kwargs):
            # 诊断延后发送，并保存本帧独立的时间统计。
            kwargs['timing'] = dict(frame_timing)
            diagnostic_calls.append((args, kwargs))

        for (st, frame), res in zip(batch, results):
            bgr, stamp, seq = frame[:3]
            frame_timing = dict(frame[3]) if len(frame) > 3 else {}
            st.mark_done(seq)
            if not st._epoch_valid(stamp):
                continue
            boxes_data = None
            if res.boxes is not None and len(res.boxes):
                # GPU 张量只复制一次，定位和标注共用过滤后的数组。
                boxes_data = self._filter_confidence((res.boxes.xyxy.cpu().numpy(),
                                                     res.boxes.cls.cpu().numpy().astype(int),
                                                     res.boxes.conf.cpu().numpy()))
            if self.publish_annotated:
                annotation_jobs.append((st, bgr, stamp, res, boxes_data))
            if boxes_data is None or len(boxes_data[0]) == 0:
                continue
            boxes, clss, confs = boxes_data
            query_start = time.monotonic()
            quality, depth_pair, camera_model, calibrated = st.query_inputs(stamp, sync_deadline)
            frame_timing['input_query_wall_ms'] = 1000.*(time.monotonic()-query_start)
            if quality is None:
                self.n_drop_pose += 1
                for c_id in clss:
                    record_diagnostic(st, stamp, int(c_id), None,
                                      False, 'no_bracketed_camera_pose')
                continue
            p_cam, R_cw = quality['pos'], quality['R']
            intrinsics_source = 'camera_info' if camera_model is not None else 'sdf_fallback'
            if camera_model is None:
                camera_model = self.fallback_model
            if (camera_model.height, camera_model.width) != bgr.shape[:2]:
                rospy.logwarn_throttle(5., '[vision] %s 图像尺寸与CameraInfo不符，拒帧', st.ns)
                continue
            calibration_ok = calibrated or not self.require_camera_info
            depth_authorized = bool(quality['depth_valid'] and calibration_ok and depth_pair is not None)
            if self.require_camera_info and not calibrated:
                rospy.logwarn_throttle(5., '[vision] %s 等待对齐的彩色/深度CameraInfo，只发导航单目', st.ns)
            if not quality['depth_valid']:
                rospy.logwarn_throttle(5., '[vision] %s 相机IMU不在图像同期缓存中，只发导航单目', st.ns)
            elif (self.lever_arm_model == 'gimbal_kinematics'
                  and quality['lever_arm_model'] != 'gimbal_kinematics_from_orientation'):
                rospy.logwarn_throttle(10., '[vision] %s 云台模型/反解不可用，光心杆臂使用机体旋转近似', st.ns)
            depth = depth_pair[0] if depth_pair is not None else None
            rgb_depth_dt = depth_pair[1] if depth_pair is not None else None
            quality = dict(quality, intrinsics_source=intrinsics_source,
                           rgb_depth_dt=rgb_depth_dt, calibration_ok=calibration_ok)
            for b, c_id, cf in zip(boxes, clss, confs):
                estimator_diagnostics = {}
                estimate_start = time.monotonic()
                est = self._estimate(st, b, depth, p_cam, R_cw, c_id,
                                     camera_model, quality, depth_authorized,
                                     estimator_diagnostics)
                frame_timing['estimate_wall_ms'] = 1000.*(time.monotonic()-estimate_start)
                if est is None:
                    record_diagnostic(st, stamp, int(c_id), quality,
                                      depth_authorized, estimator_diagnostics.get('reason', 'estimator_rejected'),
                                      rejection=estimator_diagnostics)
                    continue
                d = Detection()
                d.header.stamp = rospy.Time.from_sec(stamp)
                d.header.frame_id = 'world'
                d.drone_id = st.drone_id
                d.cls_id = int(c_id)
                d.conf = float(cf)
                d.u = est['u']
                d.v_top = est['v_top']
                d.v_bot = est['v_bot']
                d.box_w = est['box_w']
                d.x = est['x']
                d.y = est['y']
                d.sigma_along = est['sigma_along']
                d.sigma_cross = est['sigma_cross']
                d.bearing = est['bearing']
                d.slant = est['slant']
                d.method = est['method']
                d.truncated = est['truncated']
                d.image_bearing_valid = est['image_bearing_valid']
                d.image_bearing = est['image_bearing']
                d.image_error_x = est['image_error_x']
                d.pipeline_lag = float(rospy.Time.now().to_sec() - stamp)
                out.dets.append(d)
                record_diagnostic(st, stamp, int(c_id), quality,
                                  depth_authorized, 'accepted', est,
                                  measurement_stamp=d.header.stamp.to_sec())
                earliest = stamp if earliest is None else min(earliest, stamp)

        publish_time = None
        if out.dets:
            if earliest is not None:
                out.header.stamp = rospy.Time.from_sec(earliest)
            # 延迟统计使用整批实际发布时刻。
            publish_time = rospy.Time.now().to_sec()
            for d in out.dets:
                d.pipeline_lag = float(publish_time - d.header.stamp.to_sec())
            self.pub.publish(out)
            self.n_pub += len(out.dets)
        coordinates_done = time.monotonic()
        for args, kwargs in diagnostic_calls:
            kwargs['timing'].update(batch_start_stamp=batch_start_stamp,
                                   inference_done_stamp=inference_done_stamp,
                                   vision_publish_stamp=publish_time,
                                   inference_wall_ms=1000.*(inference_done-batch_start),
                                   coordinate_postprocess_wall_ms=1000.*(coordinates_done-inference_done))
            self._diagnostic(*args, **kwargs)
        for args in annotation_jobs:
            try:
                self._publish_annotated(*args)
            except Exception as exc:
                rospy.logwarn_throttle(5., '[vision] drone%d 标注图发布失败: %s',
                                       args[0].drone_id, exc)
        batch_done = time.monotonic()
        with self.health_lock:
            self.batch_timings.append((inference_done-batch_start,
                                       coordinates_done-inference_done,
                                       batch_done-coordinates_done))

    def _diagnostic(self, st, stamp, cls_id, quality, depth_authorized, reason,
                    est=None, rejection=None, measurement_stamp=None, timing=None):
        # 保留序列化后的 ROS 时间戳精度。
        payload = dict(image_stamp=float(stamp),
                       measurement_stamp=float(stamp if measurement_stamp is None else measurement_stamp),
                       drone_id=int(st.drone_id), cls_id=int(cls_id),
                       accepted=est is not None, depth_authorized=bool(depth_authorized),
                       reason=reason, source='unavailable')
        if timing is not None:
            payload['timing'] = timing
        payload['tracking_mode'] = int(st.mode)
        if quality is not None:
            payload.update(body_position=quality.get('body_position'),
                           camera_position=quality['pos'].tolist(),
                           camera_rotation=quality['R'].reshape(-1).tolist())
            for key in ('source', 'body_span', 'imu_span', 'speed', 'yaw_rate',
                        'camera_body_yaw_diff', 'rgb_depth_dt', 'intrinsics_source',
                        'lever_arm_model', 'position_sigma', 'attitude_sigma',
                        'depth_valid', 'calibration_ok'):
                payload[key] = quality.get(key)
        if est is not None:
            payload.update(method=int(est['method']), x=est['x'], y=est['y'],
                           sigma_along=est['sigma_along'], sigma_cross=est['sigma_cross'],
                           depth=est['depth_diagnostics'])
        elif rejection is not None:
            payload['depth'] = rejection.get('depth')

        if st.mode >= 1:
            if est is None:
                outcome = reason
            elif est['method'] & M_DEPTH:
                outcome = 'depth'
            elif not depth_authorized:
                if quality is None or not quality.get('depth_valid', False):
                    outcome = 'mono_camera_pose'
                elif not quality.get('calibration_ok', False):
                    outcome = 'mono_camera_calibration'
                else:
                    outcome = 'mono_depth_sync'
            elif est['depth_diagnostics'].get('reason') != 'accepted_surface':
                outcome = 'mono_' + est['depth_diagnostics'].get('reason', 'unknown')
            elif est['depth_diagnostics'].get('consistency') == 'A_J_downgrade':
                outcome = 'mono_geometry_veto'
            else:
                outcome = 'mono_other'
            with self.health_lock:
                self.tracking_input_counts['d%d:%s' % (st.drone_id, outcome)] += 1
        self.diag_pub.publish(String(data=json.dumps(payload, allow_nan=False, separators=(',', ':'))))

    def _stats_cb(self, _):
        rospy.loginfo('[vision] 已发 %d 个检测 | 丢弃: 位姿%d 距离%d 不一致%d '
                      '| 否决降级发布 %d', self.n_pub, self.n_drop_pose,
                      self.n_drop_range, self.n_drop_consist,
                      self.n_veto_downgrade)
        with self.health_lock:
            counts = dict(self.tracking_input_counts)
            timings = list(self.batch_timings)
        if timings:
            mean_ms = 1000.0 * np.mean(timings, axis=0)
            rospy.loginfo('[vision] 最近批次平均耗时: 推理%.1fms 坐标后处理%.1fms '
                          '显示/诊断%.1fms '
                          '| 跟踪观测累计 %s', mean_ms[0], mean_ms[1],
                          mean_ms[2], json.dumps(counts, sort_keys=True, separators=(',', ':')))


def main():
    if rospy is None:
        raise RuntimeError('ROS messages are unavailable; source the ROS/catkin environment')
    rospy.init_node('swarm_vision')
    node = SwarmVision()
    node.spin()


if __name__ == '__main__':
    main()
