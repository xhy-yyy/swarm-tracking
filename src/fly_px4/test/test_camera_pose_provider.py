#!/usr/bin/env python3
"""Offline regression for image-time camera attitude, depth pixels and gates."""
import os
import sys
import threading
import time
import unittest
from collections import Counter, deque
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from camera_pose_provider import (CameraModel, CameraPoseProvider, OPTICAL_TO_PHYSICAL,
                                  TemporalPoseBuffer, depth_surface_cluster,
                                  camera_imu_reference_rotation, quaternion_rotation)
import swarm_vision as sv
from test_vision_estimators import CAM_PITCH_DEG, Estimator, FakeStream, Ry, Rz, synth


def yaw_quat(degrees):
    a = np.deg2rad(degrees)*.5
    return np.array([0., 0., np.sin(a), np.cos(a)])


def matrix_quat(R):
    # Test fixtures use rotations below 180 degrees, away from trace singularity.
    w = np.sqrt(1+np.trace(R))*.5
    return np.array([(R[2, 1]-R[1, 2])/(4*w),
                     (R[0, 2]-R[2, 0])/(4*w),
                     (R[1, 0]-R[0, 1])/(4*w), w])


def default_model():
    return CameraModel(640, 480, [sv.CAMERA_FX, 0., 320., 0., sv.CAMERA_FY, 240., 0., 0., 1.])


class CameraPoseTests(unittest.TestCase):
    def test_strict_brackets_invalid_and_late_samples(self):
        b = TemporalPoseBuffer()
        self.assertFalse(b.push(1., [0, 0, 0], [0, 0, 0, 0]))
        self.assertFalse(b.push(float('nan'), [0, 0, 0], yaw_quat(0)))
        self.assertFalse(b.push(1., [float('inf'), 0, 0], yaw_quat(0)))
        self.assertTrue(b.push(1., [0, 0, 0], yaw_quat(0)))
        self.assertTrue(b.push(1.1, [1, 0, 0], yaw_quat(120)))
        self.assertFalse(b.push(1.05, [50, 0, 0], yaw_quat(0)))
        self.assertIsNone(b.query(.9999, .2))
        self.assertIsNone(b.query(1.1001, .2))
        self.assertIsNone(b.query(1.05, .05))
        mid = b.query(1.025, .2)
        self.assertAlmostEqual(mid['pos'][0], .25, places=12)
        np.testing.assert_allclose(mid['R'], quaternion_rotation(yaw_quat(30)), atol=1e-12)

    def test_antipodal_quaternions_and_clear_epoch(self):
        b = TemporalPoseBuffer()
        q = yaw_quat(179)
        b.push(100., [0, 0, 0], q)
        b.push(100.01, [0, 0, 0], -q)
        np.testing.assert_allclose(b.query(100.005, .02)['R'], quaternion_rotation(q), atol=1e-12)
        b.clear()
        b.push(1., [0, 0, 0], q)
        self.assertIsNone(b.query(100., .02))

    def test_camera_imu_yaw_lag_and_mount_not_added_twice(self):
        provider = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        provider.lever_arm_model = 'body_rotation'
        B = Rz(np.deg2rad(40)) @ Ry(np.deg2rad(15))
        I = Rz(np.deg2rad(35)) @ Ry(np.deg2rad(CAM_PITCH_DEG))
        for t in (10., 10.01):
            provider.body.push(t, [7., -3., 2.7], matrix_quat(B))
            provider.imu.push(t, [0., 0., 0.], matrix_quat(I))
        got = provider.query(10.005)
        self.assertTrue(got['depth_valid'])
        np.testing.assert_allclose(got['R'], I @ OPTICAL_TO_PHYSICAL, atol=1e-12)
        np.testing.assert_allclose(got['pos'], [7., -3., 2.7]+B @ sv.CAM_OFFSET_BODY, atol=1e-12)
        self.assertAlmostEqual(np.rad2deg(got['camera_body_yaw_diff']), -5., places=10)
        self.assertGreater(got['attitude_sigma'], 0.)

    def test_relay_fallback_cannot_authorize_depth(self):
        provider = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        R = Ry(np.deg2rad(CAM_PITCH_DEG)) @ OPTICAL_TO_PHYSICAL
        for t in (10., 10.02):
            provider.body.push(t, [0., 0., 2.7], yaw_quat(0))
            provider.relay.push(t, [0., 0., 2.7], matrix_quat(R))
        quality = provider.query(10.01)
        self.assertFalse(quality['depth_valid'])
        self.assertEqual(quality['source'], 'relay_navigation')
        box, depth, pc, Rc = synth((6.5, 0.), (0., 0.), 0., height=2.7)
        got = Estimator()._estimate(FakeStream(), box, depth, pc, Rc,
                                    pose_quality=quality, depth_authorized=False)
        self.assertFalse(got['method'] & sv.M_DEPTH)
        self.assertGreaterEqual(got['sigma_along'], .75)

    def test_original_startup_relative_imu_recovers_full_world_rotation(self):
        # A left transform is necessary: current yaw and mount pitch do not
        # commute. Include all three rotations and a nonzero initial yaw.
        initial_rpy = [4., CAM_PITCH_DEG, 30.]
        initial = Rz(np.deg2rad(30)) @ Ry(np.deg2rad(CAM_PITCH_DEG)) @ quaternion_rotation(
            [np.sin(np.deg2rad(4)/2), 0., 0., np.cos(np.deg2rad(4)/2)])
        world_camera = Rz(np.deg2rad(110)) @ Ry(np.deg2rad(23)) @ quaternion_rotation(
            [np.sin(np.deg2rad(-3)/2), 0., 0., np.cos(np.deg2rad(-3)/2)])
        provider = CameraPoseProvider(sv.CAM_OFFSET_BODY, imu_reference='initial',
                                      imu_initial_rpy_deg=initial_rpy)
        omega = np.array([.1, -.2, .3])
        for t in (10., 10.01):
            provider.body.push(t, [7., -3., 2.7], yaw_quat(115))
            provider.imu.push(t, [0., 0., 0.], matrix_quat(initial.T @ world_camera),
                              angular_velocity=omega)
        got = provider.query(10.005)
        self.assertTrue(got['depth_valid'])
        self.assertEqual(got['imu_reference'], 'initial')
        np.testing.assert_allclose(got['R'], world_camera @ OPTICAL_TO_PHYSICAL, atol=1e-12)
        self.assertAlmostEqual(got['yaw_rate'], (world_camera @ omega)[2], places=12)
        provider.clear()
        for t in (1., 1.01):
            provider.body.push(t, [0., 0., 0.], yaw_quat(30))
            provider.imu.push(t, [0., 0., 0.], yaw_quat(0))
        np.testing.assert_allclose(provider.query(1.005)['R'], initial @ OPTICAL_TO_PHYSICAL,
                                   atol=1e-12)

    def test_world_and_original_imu_give_same_depth_and_image_bearing(self):
        box, depth, pos, optical = synth((6.5, 0.), (0., 0.), 0., height=2.7)
        initial = Ry(np.deg2rad(CAM_PITCH_DEG))
        physical = optical @ OPTICAL_TO_PHYSICAL.T
        results = []
        for mode, reported in [('world', physical), ('initial', initial.T @ physical)]:
            provider = CameraPoseProvider(sv.CAM_OFFSET_BODY, imu_reference=mode)
            for t in (10., 10.01):
                provider.body.push(t, pos-sv.CAM_OFFSET_BODY, yaw_quat(0))
                provider.imu.push(t, [0., 0., 0.], matrix_quat(reported))
            quality = provider.query(10.005)
            results.append(Estimator()._estimate(FakeStream(), box, depth,
                                                 quality['pos'], quality['R'],
                                                 pose_quality=quality))
        for key in ('x', 'y', 'image_bearing'):
            self.assertAlmostEqual(results[0][key], results[1][key], places=12)
        self.assertLess(np.hypot(results[1]['x']-6.5, results[1]['y']), .02)

    def test_invalid_imu_reference_configuration_is_rejected(self):
        for mode, angles in [('guess', [0., 25., 0.]), ('initial', [0., 25.]),
                             ('initial', [0., float('nan'), 0.]), ('world', [0., 0., float('inf')])]:
            with self.assertRaises(ValueError):
                camera_imu_reference_rotation(mode, angles)

    def test_unknown_orientation_rejected_by_imu_adapter(self):
        stream = sv.DroneStream.__new__(sv.DroneStream)
        stream._epoch_valid = lambda stamp: True
        stream.camera_pose = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        stream.updated = threading.Event()
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 5.)),
                              orientation=SimpleNamespace(x=0., y=0., z=0., w=1.),
                              angular_velocity=SimpleNamespace(x=0., y=0., z=0.),
                              orientation_covariance=[-1.]+[0.]*8)
        stream._imu_cb(msg)
        self.assertIsNone(stream.camera_pose.imu.query(5., .02))
        msg.orientation_covariance = [0.]*9
        stream._imu_cb(msg)
        self.assertIsNotNone(stream.camera_pose.imu.query(5., .02))

    def test_ros_clock_rollback_clears_pose_images_and_depth(self):
        stream = sv.DroneStream.__new__(sv.DroneStream)
        stream.camera_pose = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        stream.pose = sv.PoseBuffer()
        stream.clock_lock, stream.lock = threading.Lock(), threading.Lock()
        stream.last_ros_time, stream.frame, stream.drone_id = 100., ('old', 100., 0), 1
        stream.depth_buf = deque([(np.zeros((1, 1)), 100.)])
        stream.camera_pose.body.push(100., [0, 0, 0], yaw_quat(0))
        fake_ros = SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 1.)),
                                   logwarn=lambda *args: None)
        with patch.object(sv, 'rospy', fake_ros):
            self.assertTrue(stream._epoch_valid(1.))
            self.assertFalse(stream._epoch_valid(100.))
        self.assertIsNone(stream.frame)
        self.assertFalse(stream.depth_buf)
        self.assertIsNone(stream.camera_pose.body.query(100., .1))

    def test_wait_budget_uses_wall_clock_and_depth_same_stamp(self):
        stream = sv.DroneStream.__new__(sv.DroneStream)
        stream.node = SimpleNamespace(depth_max_dt=.002, pose_max_gap=.06,
                                      imu_max_gap=.02, use_camera_imu=True, require_camera_info=True)
        stream.mode, stream.lock, stream.updated = 1, threading.Lock(), threading.Event()
        stream.color_model = stream.depth_model = default_model()
        stream.depth_buf = deque([(np.zeros((480, 640)), 10.05)])
        stream.camera_pose = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        self.assertIsNone(stream.get_depth(10.))
        started = time.monotonic()
        deadline = started+.02
        stream.query_inputs(10., deadline)
        stream.query_inputs(10., deadline)
        self.assertLess(time.monotonic()-started, .12)
        stream.depth_buf.append((np.zeros((480, 640)), 10.))
        self.assertEqual(stream.get_depth(10., with_dt=True)[1], 0.)


class DepthGeometryTests(unittest.TestCase):
    def test_same_pixel_surface_removes_roi_ray_bias(self):
        box, depth, p, R = synth((5.025, 0.), (0., 0.), 0., height=2.7)
        got = Estimator()._estimate(FakeStream(), box, depth, p, R)
        self.assertTrue(got['method'] & sv.M_DEPTH)
        self.assertLess(np.hypot(got['x']-5.025, got['y']), .012)
        # In the former implementation this exact plane had about 7 cm bias.
        self.assertGreater(got['depth_diagnostics']['support_pixels'], 20)

    def test_majority_surface_not_nearest_and_ambiguous_rejected(self):
        model = default_model()
        box = [260., 80., 380., 400.]
        depth = np.full((480, 640), np.nan, dtype=np.float32)
        r0, r1, c0, c1 = 160, 256, 302, 338
        depth[r0:r1, c0:c1] = 6.
        depth[r0:r1, c0:c0+5] = 3.
        xy, diag = depth_surface_cluster(depth, box, model, [0, 0, 0], OPTICAL_TO_PHYSICAL)
        self.assertIsNotNone(xy)
        self.assertAlmostEqual(xy[0], 6., places=6)
        depth[r0:r1, c0:c0+18] = 3.
        xy, diag = depth_surface_cluster(depth, box, model, [0, 0, 0], OPTICAL_TO_PHYSICAL)
        self.assertIsNone(xy)
        self.assertEqual(diag['reason'], 'ambiguous_surfaces')

    def test_camera_info_intrinsics_and_distortion(self):
        model = CameraModel(640, 480, [610., 0., 318.5, 0., 580., 242., 0., 0., 1.],
                            [.1, -.01, .001, .002, .005])
        x, y = .4, -.2
        r2 = x*x+y*y
        radial = 1+.1*r2-.01*r2*r2+.005*r2*r2*r2
        xd = x*radial+2*.001*x*y+.002*(r2+2*x*x)
        yd = y*radial+.001*(r2+2*y*y)+2*.002*x*y
        ray = model.rays(318.5+610*xd, 242.+580*yd)
        np.testing.assert_allclose(ray, [x, y, 1.], atol=1e-9)
        self.assertFalse(model.aligned_with(default_model()))
        with self.assertRaises(ValueError):
            CameraModel(640, 480, np.zeros(9))

    def test_truncated_navigation_fallback_and_pose_uncertainty(self):
        p, R = np.array([0., 0., 2.54]), Ry(np.deg2rad(CAM_PITCH_DEG)) @ OPTICAL_TO_PHYSICAL
        estimator = Estimator(max_range=30.)
        # Keep the head below the 22-degree horizon and within the navigation
        # range while the feet remain clipped at the image's bottom edge.
        got = estimator._estimate(FakeStream(), [300., 50., 340., 479.], None, p, R)
        self.assertIsNotNone(got)
        self.assertEqual(got['method'], sv.M_MONO_HEAD)
        box, depth, p, R = synth((6.5, 0.), (0., 0.), 0., height=2.7)
        plain = estimator._estimate(FakeStream(), box, depth, p, R)
        quality = dict(position_sigma=.3, lever_arm_sigma=.03, time_position_sigma=.2,
                       attitude_sigma=np.deg2rad(1.), depth_valid=True)
        got = estimator._estimate(FakeStream(), box, depth, p, R, pose_quality=quality)
        self.assertGreater(got['sigma_along'], plain['sigma_along'])
        self.assertGreater(got['sigma_cross'], .35)


class VisionPipelineTests(unittest.TestCase):
    def test_actual_tick_and_diagnostics_gate_depth_on_camera_imu(self):
        """Exercise scheduling/serialization, not only the geometry helper."""
        class Tensor:
            def __init__(self, a):
                self.a = np.asarray(a)
            def cpu(self):
                return self
            def numpy(self):
                return self.a

        class Boxes:
            def __init__(self, box):
                self.xyxy, self.cls, self.conf = Tensor([box]), Tensor([0]), Tensor([.95])
            def __len__(self):
                return 1

        class Publisher:
            def __init__(self):
                self.messages = []
            def publish(self, msg):
                self.messages.append(msg)

        class RosTime:
            def __init__(self, value):
                self.value = value
            def to_sec(self):
                return self.value
            @staticmethod
            def now():
                return RosTime(10.005)
            @staticmethod
            def from_sec(value):
                # ROS float-to-seconds/nanoseconds conversion can lose 1 ns.
                return RosTime(value - 1e-9)

        class ArrayMessage:
            def __init__(self):
                self.header, self.dets = SimpleNamespace(), []

        class DetectionMessage:
            def __init__(self):
                self.header = SimpleNamespace()

        node = sv.SwarmVision.__new__(sv.SwarmVision)
        node.__dict__.update(Estimator().__dict__)
        node.__dict__.update(dict(pose_max_gap=.06, imu_max_gap=.02, sync_wait_sec=0.,
                                 depth_max_dt=.002, use_camera_imu=True, require_camera_info=True,
                                 publish_annotated=False, max_batch=1, rr_cursor=0,
                                 conf=.8, red_conf=.8, half=False, device='cpu', n_drop_pose=0, n_pub=0,
                                 health_lock=threading.Lock(), tracking_input_counts=Counter(),
                                 batch_timings=deque(maxlen=100),
                                 lever_arm_model='body_rotation', fallback_model=default_model()))
        stream = sv.DroneStream.__new__(sv.DroneStream)
        stream.node, stream.drone_id, stream.ns, stream.mode = node, 1, 'test', 1
        stream.lock, stream.clock_lock, stream.updated = threading.Lock(), threading.Lock(), threading.Event()
        stream.last_ros_time, stream.last_done_seq = None, -1
        stream.color_model = stream.depth_model = default_model()
        stream.pose = sv.PoseBuffer()
        stream.camera_pose = CameraPoseProvider(sv.CAM_OFFSET_BODY)
        stream.camera_pose.lever_arm_model = 'body_rotation'
        box, depth, p, R = synth((5.025, 0.), (0., 0.), 0., height=2.7)
        for t in (10., 10.01):
            stream.camera_pose.body.push(t, [0., 0., 2.7], yaw_quat(0))
            stream.camera_pose.imu.push(t, [0., 0., 0.], matrix_quat(Ry(np.deg2rad(CAM_PITCH_DEG))))
            stream.camera_pose.relay.push(t, [0., 0., 2.7], matrix_quat(R))
        stream.depth_buf = deque([(depth, 10.005)])
        stream.frame = (np.zeros((480, 640, 3), dtype=np.uint8), 10.005, 0)
        node.streams = [stream]
        node.model = SimpleNamespace(predict=lambda *args, **kwargs: [SimpleNamespace(boxes=Boxes(box))])
        node.pub, node.diag_pub = Publisher(), Publisher()
        fake_ros = SimpleNamespace(Time=RosTime, logwarn_throttle=lambda *args: None)
        with patch.object(sv, 'rospy', fake_ros), \
                patch.object(sv, 'DetectionArray', ArrayMessage, create=True), \
                patch.object(sv, 'Detection', DetectionMessage, create=True), \
                patch.object(sv, 'String', SimpleNamespace, create=True):
            node._tick()
            self.assertTrue(node.pub.messages[-1].dets[0].method & sv.M_DEPTH)
            import json
            diag = json.loads(node.diag_pub.messages[-1].data)
            self.assertEqual(diag['source'], 'camera_imu')
            self.assertEqual(diag['image_stamp'], 10.005)
            self.assertEqual(diag['measurement_stamp'],
                             node.pub.messages[-1].dets[0].header.stamp.to_sec())
            self.assertNotEqual(diag['measurement_stamp'], diag['image_stamp'])
            self.assertTrue(diag['depth_authorized'])
            self.assertEqual(diag['rgb_depth_dt'], 0.)
            self.assertEqual(node.tracking_input_counts['d1:depth'], 1)
            stream.camera_pose.imu.clear()
            stream.frame = (stream.frame[0], 10.005, 1)
            node._tick()
            self.assertFalse(node.pub.messages[-1].dets[0].method & sv.M_DEPTH)
            diag = json.loads(node.diag_pub.messages[-1].data)
            self.assertEqual(diag['source'], 'relay_navigation')
            self.assertFalse(diag['depth_authorized'])
            self.assertEqual(node.tracking_input_counts['d1:mono_camera_pose'], 1)


if __name__ == '__main__':
    unittest.main()
