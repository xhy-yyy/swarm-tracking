#!/usr/bin/env python3
"""Physical unit/origin, covariance and optical lever-arm regression."""
import math
import os
import sys
import unittest
from types import SimpleNamespace as NS

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from world_frame_transform import transform_position, transform_odometry, validate_scale
from camera_pose_provider import CameraPoseProvider

SCALE = 6353000. / 6371000.


def point(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def odometry():
    # Off-diagonal position/attitude and velocity/angular covariance terms.
    rng = np.random.default_rng(3)
    a = rng.normal(size=(6, 6))
    covariance = a @ a.T
    return NS(header=NS(stamp=13.25, frame_id='map'), child_frame_id='base_link',
              pose=NS(pose=NS(position=point(40., -8., 2.7),
                              orientation=NS(x=0., y=0., z=0., w=1.)),
                      covariance=covariance.ravel().tolist()),
              twist=NS(twist=NS(linear=point(2., -1., .4), angular=point(.1, .2, .3)),
                       covariance=covariance.ravel().tolist()))


class WorldFrameTests(unittest.TestCase):
    def test_spawn_is_not_scaled(self):
        p = point(0., 0., 2.7)
        transform_position(p, (-17., 3., .2), SCALE)
        self.assertEqual((p.x, p.y), (-17., 3.))
        self.assertAlmostEqual(p.z, 2.9)

    def test_gps_spherical_projection_physical_displacement(self):
        # Independent full spherical geodesy: plugin inverse projection of a
        # 125m east / -20m north displacement, then PX4's forward projection.
        lat0, lon0 = map(math.radians, [47.397742, 8.545594])
        north, east = -20. / 6353000., 125. / 6353000.
        c = math.hypot(north, east)
        lat = math.asin(math.cos(c) * math.sin(lat0) +
                        north * math.sin(c) * math.cos(lat0) / c)
        lon = lon0 + math.atan2(east * math.sin(c),
                               c * math.cos(lat0) * math.cos(c) - north * math.sin(lat0) * math.sin(c))
        arg = math.sin(lat0) * math.sin(lat) + math.cos(lat0) * math.cos(lat) * math.cos(lon-lon0)
        distance = math.acos(min(1., max(-1., arg)))
        k = distance / math.sin(distance)
        estimated_north = k * (math.cos(lat0)*math.sin(lat) -
                               math.sin(lat0)*math.cos(lat)*math.cos(lon-lon0)) * 6371000.
        estimated_east = k * math.cos(lat)*math.sin(lon-lon0) * 6371000.
        self.assertGreater(estimated_east - 125., .35)
        p = point(estimated_east, estimated_north, 2.7)
        transform_position(p, (-17., 0., 0.), SCALE)
        self.assertAlmostEqual(p.x, 108., places=5)
        self.assertAlmostEqual(p.y, -20., places=5)
        self.assertEqual(p.z, 2.7)

    def test_world_horizontal_velocity_with_tilted_body(self):
        msg = odometry()
        angle = math.radians(35)
        rotation = np.array([[math.cos(angle), 0., math.sin(angle)],
                             [0., 1., 0.], [-math.sin(angle), 0., math.cos(angle)]])
        original_world_velocity = rotation @ np.array([2., -1., .4])
        transform_odometry(msg, (-14., -3., 0.), .8, rotation)
        actual_world_velocity = rotation @ np.array([
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        np.testing.assert_allclose(actual_world_velocity, original_world_velocity * [.8, .8, 1.])
        self.assertEqual(msg.twist.twist.angular.z, .3)
        self.assertEqual(msg.pose.pose.orientation.w, 1.)
        self.assertEqual(msg.header.stamp, 13.25)

    def test_covariance_remains_psd_and_scales_cross_terms(self):
        msg = odometry()
        before = np.array(msg.pose.covariance).reshape(6, 6)
        transform_odometry(msg, (-14., -3., 0.), .8, np.eye(3))
        after = np.array(msg.pose.covariance).reshape(6, 6)
        self.assertAlmostEqual(after[0, 0], before[0, 0] * .64)
        self.assertAlmostEqual(after[0, 3], before[0, 3] * .8)
        self.assertAlmostEqual(after[2, 5], before[2, 5])
        for cov in (msg.pose.covariance, msg.twist.covariance):
            matrix = np.array(cov).reshape(6, 6)
            np.testing.assert_allclose(matrix, matrix.T)
            self.assertGreaterEqual(np.linalg.eigvalsh(matrix).min(), -1e-10)

    def test_identity_preserves_existing_messages(self):
        msg = odometry()
        covariance = list(msg.pose.covariance)
        transform_odometry(msg, (-14., -3., 0.), 1., np.eye(3))
        self.assertEqual((msg.pose.pose.position.x, msg.pose.pose.position.y), (26., -11.))
        self.assertEqual(msg.pose.covariance, covariance)
        self.assertEqual((msg.twist.twist.linear.x, msg.twist.twist.linear.z), (2., .4))

    def test_camera_lever_arm_keeps_physical_size(self):
        p = point(125. / SCALE, 0., 2.7)
        transform_position(p, (-17., 0., 0.), SCALE)
        provider = CameraPoseProvider([.15, .03, -.16])
        provider.lever_arm_model = 'body_rotation'
        for t in (1., 1.01):
            provider.body.push(t, [p.x, p.y, p.z], [0., 0., 0., 1.])
            provider.imu.push(t, [0., 0., 0.], [0., 0., 0., 1.])
        camera = provider.query(1.005)
        np.testing.assert_allclose(camera['pos'], [108.15, .03, 2.54])
        self.assertEqual(camera['body_position'][0], 108.)

    def test_reject_invalid_unit_scale(self):
        for value in [0., -1., float('inf'), float('nan')]:
            with self.assertRaises(ValueError):
                validate_scale(value)


if __name__ == '__main__':
    unittest.main()
