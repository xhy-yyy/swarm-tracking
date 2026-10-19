#!/usr/bin/env python3
"""Physical anchor invariants and orientation-to-centre checks, without ROS."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import camera_gimbal_kinematics as cg


class GimbalKinematicsTest(unittest.TestCase):
    def setUp(self):
        self.geometry = cg.load_gimbal_geometry()
        self.assertIsNotNone(self.geometry)
        # The submitted model locks yaw. Use an explicit yaw-enabled fixture
        # for the general anchor/forward-chain checks, without changing it.
        limits = self.geometry.limits.copy()
        limits[0] = [-np.pi, np.pi]
        self.yaw_geometry = cg.GimbalGeometry(
            self.geometry.anchors, limits, self.geometry.camera_offset,
            self.geometry.mount_rotation)

    def test_zero_joints_preserve_mount_offset_under_body_tilt(self):
        B = cg.rotation([1, 1, .2], .4)
        I = B @ self.geometry.mount_rotation
        np.testing.assert_allclose(cg.camera_offset_body(B, I), [.0205, 0., -.20], atol=1e-10)

    def test_yaw_quarter_turn_uses_joint_anchor_not_body_origin(self):
        I = cg.rotation([0, 0, 1], np.pi / 2) @ self.geometry.mount_rotation
        self.assertIsNone(cg.camera_offset_body(np.eye(3), I, self.geometry))
        # Offset (.0205,0,-.20) rotating around (-.026,0,-.10).
        np.testing.assert_allclose(cg.camera_offset_body(np.eye(3), I, self.yaw_geometry), [-.026, .0465, -.20], atol=1e-10)

    def test_pitch_rotation_changes_height_and_preserves_anchor_distance(self):
        angle = .4
        I = cg.rotation([0, 1, 0], angle) @ self.geometry.mount_rotation
        centre = cg.camera_offset_body(np.eye(3), I)
        pivot = self.geometry.anchors[2]
        self.assertAlmostEqual(np.linalg.norm(centre - pivot), np.linalg.norm(self.geometry.camera_offset - pivot), places=10)
        self.assertGreater(np.linalg.norm(centre - self.geometry.camera_offset), .01)

    def test_combined_joints_match_independent_homogeneous_forward_chain(self):
        B = cg.rotation([.2, 1., .7], .32)
        axes = ([0., 0., 1.], [-1., 0., 0.], [0., -1., 0.])
        for angles in [(.7, .35, -.5), (-1.2, -.6, -1.1), (2.8, .5, .3)]:
            T = np.eye(4)
            for pivot, axis, q in zip(self.geometry.anchors, axes, angles):
                R = cg.rotation(axis, q)
                H = np.eye(4)
                H[:3, :3] = R
                H[:3, 3] = (np.eye(3) - R) @ pivot
                T = T @ H
            I = B @ T[:3, :3] @ self.geometry.mount_rotation
            want = T @ np.r_[self.geometry.camera_offset, 1.]
            np.testing.assert_allclose(cg.camera_offset_body(B, I, self.yaw_geometry), want[:3], atol=1e-10)

    def test_outside_joint_limits_and_invalid_rotations_are_rejected(self):
        I = cg.rotation([-1, 0, 0], self.geometry.limits[1, 1] + .1) @ self.geometry.mount_rotation
        self.assertIsNone(cg.camera_offset_body(np.eye(3), I))
        self.assertIsNone(cg.camera_offset_body(np.eye(3), np.full((3, 3), np.nan)))
        self.assertIsNone(cg.camera_offset_body(np.eye(3) * 2, np.eye(3)))
        self.assertIsNone(cg.camera_offset_body(np.eye(3), np.diag([1., 1., -1.])))

    def test_unknown_zero_link_geometry_does_not_silently_use_constants(self):
        text = Path(cg.DEFAULT_MODEL_SDF).read_text()
        text = text.replace('<link name="cgo3_camera_link">', '<link name="cgo3_camera_link"><pose>0.1 0 0 0 0 0</pose>')
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'model.sdf')
            Path(path).write_text(text)
            self.assertIsNone(cg.load_gimbal_geometry(path))


if __name__ == '__main__':
    unittest.main()
