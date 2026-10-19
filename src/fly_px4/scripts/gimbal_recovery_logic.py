#!/usr/bin/env python3
"""Event-driven gimbal recovery using IMUs only; independent of ROS and Gazebo."""
from dataclasses import dataclass
import math

import numpy as np

from camera_pose_provider import camera_imu_reference_rotation, quaternion_rotation


@dataclass(frozen=True)
class ImuObservation:
    stamp: float
    received: float
    quaternion: tuple
    angular_velocity: tuple
    available: bool = True


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    command: bool = False
    roll_deg: float = math.nan
    pitch_deg: float = math.nan
    attempts: int = 0


class GimbalRecovery:
    DEFAULTS = dict(fault_angle_deg=15., healthy_angle_deg=3., fault_sec=1.5,
                    healthy_sec=.7, body_tilt_max_deg=15.,
                    body_rate_max_deg_s=20., camera_rate_max_deg_s=5.,
                    stuck_span_deg=2., feedback_timeout=.3,
                    pair_skew_max=.15, future_tolerance=.05,
                    retry_sec=1., recovery_timeout=6., max_attempts=3,
                    cooldown_sec=30., require_stationary=True)

    def __init__(self, camera_imu_reference='initial',
                 camera_imu_initial_rpy_deg=(0., 25., 0.),
                 camera_mount_rpy_deg=(0., 25., 0.), **parameters):
        unknown = set(parameters) - set(self.DEFAULTS)
        if unknown:
            raise ValueError('unknown parameters: ' + ', '.join(sorted(unknown)))
        self.params = dict(self.DEFAULTS, **parameters)
        for key, value in self.params.items():
            if key == 'require_stationary':
                if not isinstance(value, bool):
                    raise ValueError('require_stationary must be true or false')
                continue
            if not math.isfinite(float(value)) or value <= 0:
                raise ValueError(key + ' must be finite and positive')
        attempts = self.params['max_attempts']
        if int(attempts) != attempts or attempts > 10:
            raise ValueError('max_attempts must be an integer between 1 and 10')
        if self.params['healthy_angle_deg'] >= self.params['fault_angle_deg']:
            raise ValueError('healthy_angle_deg must be below fault_angle_deg')
        self.reference = camera_imu_reference_rotation(
            camera_imu_reference, camera_imu_initial_rpy_deg)
        self.mount = camera_imu_reference_rotation('initial', camera_mount_rpy_deg)
        self.reset()

    def reset(self):
        self.phase = 'watch'
        self.candidate = None
        self.healthy_since = None
        self.attempts = 0
        self.started = self.last_command = self.cooldown_until = None
        self.cooldown_reason = ''
        self.previous_now = self.previous_wall = None
        self.last_stamps = [None, None]

    def _rotation(self, observation, index, now, wall):
        p = self.params
        if observation is None or not observation.available:
            return None
        stamp, received = observation.stamp, observation.received
        if not all(math.isfinite(v) for v in (stamp, received)) or stamp <= 0:
            return None
        if not (-p['future_tolerance'] <= now-stamp <= p['feedback_timeout']):
            return None
        if not (0 <= wall-received <= p['feedback_timeout']):
            return None
        if self.last_stamps[index] is not None and stamp < self.last_stamps[index]:
            return None
        self.last_stamps[index] = stamp
        q = np.asarray(observation.quaternion, dtype=float)
        omega = np.asarray(observation.angular_velocity, dtype=float)
        if q.shape != (4,) or omega.shape != (3,):
            return None
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(omega)):
            return None
        if not .5 <= np.linalg.norm(q) <= 1.5:
            return None
        return quaternion_rotation(q), omega

    @staticmethod
    def _tilt(rotation):
        return np.rad2deg([math.atan2(rotation[2, 1], rotation[2, 2]),
                           math.asin(float(np.clip(-rotation[2, 0], -1., 1.)))])

    def _metrics(self, camera, body, now, wall):
        camera_values = self._rotation(camera, 0, now, wall)
        body_values = self._rotation(body, 1, now, wall)
        if camera_values is None or body_values is None:
            return None
        if abs(camera.stamp-body.stamp) > self.params['pair_skew_max']:
            return None
        camera_rotation = self.reference @ camera_values[0]
        # Remove the physical camera's fixed mounting rotation, not Euler angles.
        angles = self._tilt(camera_rotation @ self.mount.T)
        deviated = max(abs(angles)) >= self.params['fault_angle_deg']
        # 持续偏角模式不因机体运动、相机角速度或姿态变化范围而延后回中。
        if not self.params['require_stationary']:
            return angles, deviated
        body_rotation, body_omega = body_values
        body_angles = self._tilt(body_rotation)
        camera_rate = np.rad2deg(np.linalg.norm((camera_rotation @ camera_values[1])[:2]))
        body_rate = np.rad2deg(np.linalg.norm((body_rotation @ body_omega)[:2]))
        calm = (max(abs(body_angles)) <= self.params['body_tilt_max_deg'] and
                body_rate <= self.params['body_rate_max_deg_s'])
        stuck = (deviated and calm and
                 camera_rate <= self.params['camera_rate_max_deg_s'])
        return angles, stuck

    def _confirmed(self, angles, stuck, now, wall):
        if not stuck:
            self.candidate = None
            return False
        if self.candidate is None:
            self.candidate = (now, wall, angles.copy(), angles.copy())
        start_ros, start_wall, minimum, maximum = self.candidate
        minimum, maximum = np.minimum(minimum, angles), np.maximum(maximum, angles)
        if (self.params['require_stationary'] and
                max(maximum-minimum) > self.params['stuck_span_deg']):
            start_ros, start_wall, minimum, maximum = now, wall, angles.copy(), angles.copy()
        self.candidate = (start_ros, start_wall, minimum, maximum)
        return (now-start_ros >= self.params['fault_sec'] and
                wall-start_wall >= self.params['fault_sec'])

    def _finish(self, wall, reason):
        self.phase = 'cooldown'
        self.cooldown_until = wall + self.params['cooldown_sec']
        self.cooldown_reason = reason
        self.candidate = self.healthy_since = None

    def step(self, now, wall, camera, body, control_connected):
        """Return a recenter request after fresh feedback confirms persistent tilt.

        Wall time bounds retries even while simulation time is paused. Both
        clocks must agree on feedback freshness. Neither missing feedback nor
        absent MAVROS consumes command attempts.
        """
        if not all(math.isfinite(v) for v in (now, wall)):
            self.reset()
            return Decision('NO_FEEDBACK', 'invalid_clock')
        if ((self.previous_now is not None and now < self.previous_now) or
                (self.previous_wall is not None and wall < self.previous_wall)):
            self.reset()
        clock_advancing = self.previous_now is None or now > self.previous_now
        self.previous_now, self.previous_wall = now, wall
        metrics = self._metrics(camera, body, now, wall)
        if metrics is None:
            self.candidate = self.healthy_since = None
            if self.phase == 'recover' and wall-self.started >= self.params['recovery_timeout']:
                self._finish(wall, 'feedback_lost')
            return Decision('NO_FEEDBACK', 'fresh_paired_imus_required', attempts=self.attempts)
        angles, stuck = metrics

        def result(state, reason, command=False):
            return Decision(state, reason, command, float(angles[0]),
                            float(angles[1]), self.attempts)

        if self.phase == 'cooldown':
            if wall < self.cooldown_until:
                return result('COOLDOWN', self.cooldown_reason)
            self.phase, self.attempts = 'watch', 0
            self.candidate = None
        if self.phase == 'recover':
            if max(abs(angles)) <= self.params['healthy_angle_deg']:
                if self.healthy_since is None:
                    self.healthy_since = (now, wall)
                self.candidate = None
                if (now-self.healthy_since[0] >= self.params['healthy_sec'] and
                        wall-self.healthy_since[1] >= self.params['healthy_sec']):
                    self._finish(wall, 'recovered')
                    return result('COOLDOWN', 'recovered')
            else:
                self.healthy_since = None
            if wall-self.started >= self.params['recovery_timeout']:
                self._finish(wall, 'recovery_timeout')
                return result('COOLDOWN', 'recovery_timeout')
        confirmed = self._confirmed(angles, stuck, now, wall) and clock_advancing
        if self.phase == 'watch' and not confirmed:
            confirming = ('confirming_stationary_tilt' if self.params['require_stationary']
                          else 'confirming_persistent_tilt')
            return result('SUSPECT' if self.candidate else 'MONITORING',
                          confirming if self.candidate else 'observation_only')
        if confirmed and not control_connected:
            return result('WAITING_LINK', 'mount_command_subscriber_required')
        if confirmed and (self.phase == 'watch' or
                          (self.attempts < self.params['max_attempts'] and
                           wall-self.last_command >= self.params['retry_sec'])):
            if self.phase == 'watch':
                self.phase, self.started = 'recover', wall
            self.attempts += 1
            self.last_command = wall
            self.candidate = None  # A retry needs a new period of persistent tilt.
            return result('RECOVERING', 'persistent_tilt_recenter', command=True)
        return result('RECOVERING', 'waiting_for_measured_recovery')
