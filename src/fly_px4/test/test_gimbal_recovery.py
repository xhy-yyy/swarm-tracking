#!/usr/bin/env python3
"""Exercise recovery policy with sensor observations, without ROS or Gazebo."""
from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from camera_pose_provider import camera_imu_reference_rotation
from gimbal_recovery_logic import GimbalRecovery, ImuObservation


def matrix_quaternion(rotation):
    # Test rotations avoid 180 degrees, so the positive-w branch is sufficient.
    w = math.sqrt(1. + float(np.trace(rotation))) / 2.
    return ((rotation[2, 1]-rotation[1, 2])/(4*w),
            (rotation[0, 2]-rotation[2, 0])/(4*w),
            (rotation[1, 0]-rotation[0, 1])/(4*w), w)


def observations(now, wall, camera_rpy=(0., 0., 0.), body_rpy=(0., 0., 0.),
                 camera_rate=(0., 0., 0.), body_rate=(0., 0., 0.), reference='initial'):
    mount = camera_imu_reference_rotation('initial', (0., 25., 0.))
    camera_rotation = camera_imu_reference_rotation('initial', camera_rpy) @ mount
    body_rotation = camera_imu_reference_rotation('initial', body_rpy)
    reference_rotation = mount if reference == 'initial' else np.eye(3)
    camera = ImuObservation(now, wall, matrix_quaternion(reference_rotation.T @ camera_rotation),
                            tuple(camera_rotation.T @ np.deg2rad(camera_rate)))
    body = ImuObservation(now, wall, matrix_quaternion(body_rotation),
                          tuple(body_rotation.T @ np.deg2rad(body_rate)))
    return camera, body


class RecoveryTests(unittest.TestCase):
    def setUp(self):
        self.monitor = GimbalRecovery()
        self.t = 100.

    def feed(self, duration, connected=True, **kwargs):
        decisions = []
        for _ in range(round(duration/.05)):
            self.t += .05
            camera, body = observations(self.t, self.t, **kwargs)
            decisions.append(self.monitor.step(self.t, self.t, camera, body, connected))
        return decisions

    def test_normal_flight_never_sends_commands(self):
        for i in range(2400):
            self.t += .05
            yaw = 120.*math.sin(i/100.)
            camera, body = observations(self.t, self.t,
                camera_rpy=(2.*math.sin(i/30.), 2.*math.cos(i/25.), yaw),
                body_rpy=(25.*math.sin(i/70.), 20.*math.cos(i/50.), yaw),
                camera_rate=(0., 0., 60.), body_rate=(20., 10., 60.))
            result = self.monitor.step(self.t, self.t, camera, body, True)
            self.assertFalse(result.command)
            self.assertEqual(result.attempts, 0)

    def test_fixed_mount_and_yaw_do_not_look_like_faults(self):
        for reference in ('initial', 'world'):
            monitor = GimbalRecovery(camera_imu_reference=reference)
            for yaw in (-120., -60., 0., 60., 120.):
                self.t += .05
                camera, body = observations(self.t, self.t, camera_rpy=(0., 0., yaw),
                                            reference=reference)
                result = monitor.step(self.t, self.t, camera, body, True)
                self.assertAlmostEqual(result.roll_deg, 0., places=7)
                self.assertAlmostEqual(result.pitch_deg, 0., places=7)
                self.assertFalse(result.command)

    def test_both_axes_and_both_directions_trigger_after_confirmation(self):
        for rpy in ((45., 0., 60.), (-45., 0., 0.), (0., 45., 0.), (0., -60., -60.)):
            self.monitor = GimbalRecovery()
            early = self.feed(1.4, camera_rpy=rpy)
            self.assertFalse(any(d.command for d in early))
            later = self.feed(.4, camera_rpy=rpy)
            commands = [d for d in later if d.command]
            self.assertEqual(len(commands), 1)
            self.assertAlmostEqual(commands[0].roll_deg, rpy[0], places=6)
            self.assertAlmostEqual(commands[0].pitch_deg, rpy[1], places=6)

    def test_maneuvers_delay_confirmation(self):
        for kwargs in ({'body_rpy': (25., 0., 0.)},
                       {'body_rate': (30., 0., 0.)},
                       {'camera_rate': (10., 0., 0.)}):
            self.monitor = GimbalRecovery()
            results = self.feed(4., camera_rpy=(45., 0., 0.), **kwargs)
            self.assertFalse(any(d.command for d in results))
            self.assertFalse(any(d.command for d in self.feed(1.4, camera_rpy=(45., 0., 0.))))
            self.assertTrue(any(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))))

    def test_transient_and_measured_motion_do_not_trigger(self):
        self.assertFalse(any(d.command for d in self.feed(1., camera_rpy=(45., 0., 0.))))
        self.assertFalse(any(d.command for d in self.feed(3.)))
        for i in range(120):
            self.t += .05
            # Gyro erroneously reads zero, but measured tilt is still recovering.
            camera, body = observations(self.t, self.t, camera_rpy=(45.-i*.2, 0., 0.))
            self.assertFalse(self.monitor.step(self.t, self.t, camera, body, True).command)

    def test_measured_recovery_stops_all_commands(self):
        commands = [d for d in self.feed(1.8, camera_rpy=(45., 0., 0.)) if d.command]
        self.assertEqual(len(commands), 1)
        results = self.feed(45.)
        self.assertFalse(any(d.command for d in results))
        self.assertTrue(any(d.reason == 'recovered' for d in results))
        self.assertEqual(results[-1].state, 'MONITORING')

    def test_retries_are_bounded_and_cooldown_is_silent(self):
        results = self.feed(10., camera_rpy=(0., 45., 0.))
        self.assertEqual(sum(d.command for d in results), 3)
        self.assertEqual(results[-1].reason, 'recovery_timeout')
        self.assertFalse(any(d.command for d in self.feed(25., camera_rpy=(0., 45., 0.))))

    def test_every_retry_requires_fresh_fault_confirmation(self):
        self.feed(1.8, camera_rpy=(45., 0., 0.))
        moving = self.feed(3., camera_rpy=(45., 0., 0.), camera_rate=(15., 0., 0.))
        self.assertFalse(any(d.command for d in moving))
        self.assertFalse(any(d.command for d in self.feed(1.4, camera_rpy=(45., 0., 0.))))

    def test_missing_stale_invalid_or_unsynchronized_feedback_never_commands(self):
        good_camera, good_body = observations(self.t, self.t, camera_rpy=(45., 0., 0.))
        invalid_pairs = [
            (None, good_body), (good_camera, None),
            (replace(good_camera, quaternion=(0., 0., 0., 0.)), good_body),
            (replace(good_camera, quaternion=(math.nan, 0., 0., 1.)), good_body),
            (replace(good_camera, angular_velocity=(math.nan, 0., 0.)), good_body),
            (replace(good_camera, available=False), good_body),
            (good_camera, replace(good_body, stamp=self.t-.2)),
            (replace(good_camera, stamp=self.t+.1), good_body),
            (replace(good_camera, received=self.t-1.), good_body)]
        for camera, body in invalid_pairs:
            monitor = GimbalRecovery()
            for i in range(60):
                result = monitor.step(self.t+i*.05, self.t+i*.05, camera, body, True)
                self.assertFalse(result.command)
                if i == 0 or i == 59:
                    self.assertEqual(result.state, 'NO_FEEDBACK')

    def test_sensor_dropout_resets_confirmation(self):
        self.feed(1.4, camera_rpy=(45., 0., 0.))
        self.t += .05
        result = self.monitor.step(self.t, self.t, None, None, True)
        self.assertFalse(result.command)
        self.assertFalse(any(d.command for d in self.feed(1.4, camera_rpy=(45., 0., 0.))))
        self.assertTrue(any(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))))

    def test_paused_clock_does_not_allow_blind_commands(self):
        self.feed(1.3, camera_rpy=(45., 0., 0.))
        camera, body = observations(self.t, self.t, camera_rpy=(45., 0., 0.))
        for i in range(1, 100):
            self.assertFalse(self.monitor.step(self.t, self.t+i*.05, camera, body, True).command)

    def test_out_of_order_feedback_resets_confirmation(self):
        self.feed(1.4, camera_rpy=(45., 0., 0.))
        camera, body = observations(self.t-.1, self.t, camera_rpy=(45., 0., 0.))
        self.assertEqual(self.monitor.step(self.t, self.t, camera, body, True).state, 'NO_FEEDBACK')
        self.assertFalse(any(d.command for d in self.feed(1.4, camera_rpy=(45., 0., 0.))))

    def test_clock_rollback_restarts_confirmation(self):
        self.feed(1.4, camera_rpy=(45., 0., 0.))
        self.t = 10.
        self.assertFalse(any(d.command for d in self.feed(1.4, camera_rpy=(45., 0., 0.))))
        self.assertTrue(any(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))))

    def test_disconnected_mavros_does_not_consume_attempts(self):
        results = self.feed(5., connected=False, camera_rpy=(45., 0., 0.))
        self.assertFalse(any(d.command for d in results))
        self.assertEqual(results[-1].state, 'WAITING_LINK')
        self.assertEqual(results[-1].attempts, 0)
        self.assertTrue(any(d.command for d in self.feed(.1, camera_rpy=(45., 0., 0.))))

    def test_invalid_configuration_is_rejected(self):
        for parameters in ({'fault_sec': 0.}, {'healthy_angle_deg': 20.},
                           {'feedback_timeout': math.nan}, {'max_attempts': 2.5},
                           {'require_stationary': 'false'}, {'require_stationary': 0}):
            with self.assertRaises(ValueError):
                GimbalRecovery(**parameters)


class PersistentTiltTests(unittest.TestCase):
    """Verify the three-second trigger while the aircraft and camera move."""
    feed = RecoveryTests.feed

    def setUp(self):
        self.monitor = GimbalRecovery(require_stationary=False,
                                      fault_angle_deg=20., fault_sec=3.)
        self.t = 100.

    def test_each_axis_and_direction_waits_three_seconds(self):
        for rpy in ((45., 0., 90.), (-45., 0., -90.), (0., 45., 0.), (0., -45., 0.)):
            self.setUp()
            early = self.feed(2.95, camera_rpy=rpy, body_rpy=(25., 20., 0.),
                              body_rate=(40., 30., 0.), camera_rate=(30., 20., 0.))
            self.assertFalse(any(d.command for d in early))
            self.assertEqual(early[-1].reason, 'confirming_persistent_tilt')
            later = self.feed(.25, camera_rpy=rpy, body_rpy=(25., 20., 0.),
                              body_rate=(40., 30., 0.), camera_rate=(30., 20., 0.))
            self.assertEqual(sum(d.command for d in later), 1)

    def test_changing_large_angles_do_not_restart_confirmation(self):
        commands = []
        for i in range(65):
            self.t += .05
            camera, body = observations(self.t, self.t,
                camera_rpy=(35.+10.*math.sin(i/8.), 0., i*2.),
                body_rpy=(30., 0., i*2.), camera_rate=(30., 0., 40.),
                body_rate=(40., 0., 40.))
            result = self.monitor.step(self.t, self.t, camera, body, True)
            if i < 60:
                self.assertFalse(result.command)
            if result.command:
                commands.append(result)
        self.assertEqual(len(commands), 1)

    def test_short_tilt_and_return_below_threshold_restart_timer(self):
        self.assertFalse(any(d.command for d in self.feed(2.9, camera_rpy=(45., 0., 0.))))
        self.assertFalse(any(d.command for d in self.feed(.1, camera_rpy=(19., 0., 0.))))
        self.assertFalse(any(d.command for d in self.feed(2.9, camera_rpy=(45., 0., 0.))))
        self.assertEqual(sum(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))), 1)

    def test_normal_mount_during_maneuvers_never_recenters(self):
        for i in range(2400):
            self.t += .05
            yaw = 120.*math.sin(i/100.)
            camera, body = observations(self.t, self.t,
                camera_rpy=(10.*math.sin(i/20.), 10.*math.cos(i/20.), yaw),
                body_rpy=(30.*math.sin(i/20.), 25.*math.cos(i/20.), yaw),
                camera_rate=(30., 20., 60.), body_rate=(50., 40., 60.))
            self.assertFalse(self.monitor.step(self.t, self.t, camera, body, True).command)

    def test_missing_or_unsynchronized_feedback_restarts_timer(self):
        for missing in (True, False):
            self.setUp()
            self.feed(2.9, camera_rpy=(45., 0., 0.))
            self.t += .05
            camera, body = observations(self.t, self.t, camera_rpy=(45., 0., 0.))
            body = None if missing else replace(body, stamp=self.t-.2)
            self.assertEqual(self.monitor.step(self.t, self.t, camera, body, True).state,
                             'NO_FEEDBACK')
            self.assertFalse(any(d.command for d in self.feed(2.9, camera_rpy=(45., 0., 0.))))
            self.assertTrue(any(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))))

    def test_retries_reconfirm_three_seconds_and_stop_on_recovery(self):
        self.assertEqual(sum(d.command for d in self.feed(3.3, camera_rpy=(45., 0., 0.))), 1)
        self.assertFalse(any(d.command for d in self.feed(2.8, camera_rpy=(45., 0., 0.))))
        self.assertEqual(sum(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))), 1)
        recovered = self.feed(45.)
        self.assertFalse(any(d.command for d in recovered))
        self.assertTrue(any(d.reason == 'recovered' for d in recovered))

    def test_attempt_limit_and_cooldown_still_apply(self):
        self.monitor = GimbalRecovery(require_stationary=False, fault_angle_deg=20.,
                                      fault_sec=3., recovery_timeout=10.)
        results = self.feed(15., camera_rpy=(0., -45., 0.), body_rate=(40., 0., 0.))
        commands = [d for d in results if d.command]
        self.assertEqual(len(commands), 3)
        self.assertEqual(commands[-1].attempts, 3)
        self.assertEqual(results[-1].reason, 'recovery_timeout')
        self.assertFalse(any(d.command for d in self.feed(25., camera_rpy=(0., -45., 0.))))

    def test_paused_or_reversed_clock_cannot_complete_confirmation(self):
        self.feed(2.8, camera_rpy=(45., 0., 0.))
        camera, body = observations(self.t, self.t, camera_rpy=(45., 0., 0.))
        for i in range(1, 100):
            self.assertFalse(self.monitor.step(self.t, self.t+i*.05, camera, body, True).command)
        self.t = 10.
        self.assertFalse(any(d.command for d in self.feed(2.9, camera_rpy=(45., 0., 0.))))
        self.assertTrue(any(d.command for d in self.feed(.4, camera_rpy=(45., 0., 0.))))

    def test_shipped_yaml_triggers_with_motion_at_configured_rate(self):
        import yaml
        path = Path(__file__).resolve().parents[1] / 'config/gimbal_recovery.yaml'
        config = yaml.safe_load(path.read_text())
        self.assertIsInstance(config['enabled'], bool)
        self.assertFalse(config['require_stationary'])
        self.assertEqual(config['fault_sec'], 3.)
        parameters = {key: value for key, value in config.items()
                      if key in GimbalRecovery.DEFAULTS or key.startswith('camera_imu_')
                      or key == 'camera_mount_rpy_deg'}
        monitor = GimbalRecovery(**parameters)
        step = 1./config['rate_hz']
        first_sample, first_command = None, None
        for i in range(round((config['fault_sec']+1.)/step)):
            self.t += step
            camera, body = observations(self.t, self.t,
                camera_rpy=(45., 0., 0.), body_rpy=(30., 0., 0.),
                camera_rate=(30., 0., 0.), body_rate=(40., 0., 0.))
            first_sample = self.t if first_sample is None else first_sample
            result = monitor.step(self.t, self.t, camera, body, True)
            if self.t-first_sample < config['fault_sec']:
                self.assertFalse(result.command)
            if result.command:
                first_command = self.t
                break
        self.assertIsNotNone(first_command)
        self.assertLessEqual(first_command-first_sample, config['fault_sec']+step+1e-8)


if __name__ == '__main__':
    unittest.main(verbosity=2)
