#!/usr/bin/env python3
"""Bound stale CV displacement during observed turns; no ROS or truth inputs."""
from collections import deque
import math

import numpy as np


class PublicationPrediction:
    def __init__(self, enabled=True, window=0.5, turn_deg=45.0, decay_sec=0.15):
        if not (math.isfinite(window) and window >= 0.2
                and math.isfinite(turn_deg) and 0.0 < turn_deg <= 180.0
                and math.isfinite(decay_sec) and decay_sec > 0.0):
            raise ValueError('invalid publication prediction parameters')
        self.enabled = bool(enabled)
        self.window = float(window)
        self.turn_deg = float(turn_deg)
        self.decay_sec = float(decay_sec)
        self.samples = deque(maxlen=32)

    def reset(self):
        self.samples.clear()

    def observe(self, stamp, source, velocity):
        """Only call after an accepted depth update, using its posterior velocity."""
        velocity = np.asarray(velocity, dtype=float)
        if not math.isfinite(stamp) or velocity.shape != (2,) or not np.all(np.isfinite(velocity)):
            return False
        if self.samples and stamp < self.samples[-1][0]:
            return False
        if self.samples and (source != self.samples[-1][1]
                             or stamp - self.samples[-1][0] > 0.4):
            self.reset()
        # Same image time can have a second accepted posterior; replace it
        # without manufacturing another independent observation.
        if self.samples and stamp == self.samples[-1][0]:
            self.samples.pop()
        self.samples.append((float(stamp), int(source), velocity.copy()))
        while len(self.samples) > 1 and stamp - self.samples[0][0] > self.window:
            self.samples.popleft()
        return True

    def predict(self, position, velocity, measurement_stamp, publication_stamp):
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        age = max(0.0, float(publication_stamp) - float(measurement_stamp))
        horizon = age
        mode, angle, deceleration = 'constant_velocity', 0.0, 0.0
        if (self.enabled and len(self.samples) >= 3
                and abs(self.samples[-1][0] - measurement_stamp) <= 1e-6):
            span = self.samples[-1][0] - self.samples[0][0]
            if span + 1e-6 >= 0.2:
                old_velocity = self.samples[0][2]
                old_speed = float(np.linalg.norm(old_velocity))
                speed = float(np.linalg.norm(velocity))
                if old_speed * speed > 1e-9:
                    cosine = float(old_velocity @ velocity) / (old_speed * speed)
                    angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
                deceleration = (old_speed - speed) / span
                turning = speed >= 0.3 and angle >= self.turn_deg
                braking = deceleration >= 1.2 and speed < old_speed * 0.65
                if old_speed >= 0.4 and (turning or braking):
                    # Integrate v(age)=v0*exp(-age/tau). Displacement is
                    # continuous and bounded by |v0|*tau; keep the true age.
                    horizon = -self.decay_sec * math.expm1(-age / self.decay_sec)
                    mode = 'turn_velocity_decay'
        xy = position + velocity * horizon
        metadata = dict(mode=mode, measurement_age=age, effective_horizon=horizon,
                        turn_angle_deg=angle, deceleration=deceleration,
                        history_size=len(self.samples),
                        posterior_xy=position.tolist(), kf_velocity_xy=velocity.tolist(),
                        cv_disagreement_m=float(np.linalg.norm(velocity) * (age - horizon)))
        return xy, metadata
