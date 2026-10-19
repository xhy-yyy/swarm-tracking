#!/usr/bin/env python3
"""Convert local EKF XY units; preserve physical lever arms, Z and attitude."""
import math

import numpy as np


def validate_scale(scale):
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.:
        raise ValueError('local_xy_scale must be finite and positive')
    return scale


def transform_position(position, offset, scale):
    """Spawn is a world position, so scale the displacement before translation."""
    position.x = position.x * scale + offset[0]
    position.y = position.y * scale + offset[1]
    position.z += offset[2]


def transform_odometry(msg, offset, scale, body_to_world):
    """MAVROS pose is in map; twist is in the child body frame.

    Transform covariance with the corresponding Jacobian, including the
    position/orientation and linear/angular cross terms. A world-horizontal
    scale acts on body velocity as R.T @ diag(s,s,1) @ R.
    """
    transform_position(msg.pose.pose.position, offset, scale)
    if scale == 1.:
        return
    horizontal = np.diag([scale, scale, 1.])
    jacobian_pose = np.eye(6)
    jacobian_pose[:3, :3] = horizontal
    covariance = np.asarray(msg.pose.covariance).reshape(6, 6)
    msg.pose.covariance = (jacobian_pose @ covariance @ jacobian_pose.T).ravel().tolist()
    child_linear = np.asarray(body_to_world).T @ horizontal @ np.asarray(body_to_world)
    v = msg.twist.twist.linear
    v.x, v.y, v.z = child_linear @ np.array([v.x, v.y, v.z])
    jacobian_twist = np.eye(6)
    jacobian_twist[:3, :3] = child_linear
    covariance = np.asarray(msg.twist.covariance).reshape(6, 6)
    msg.twist.covariance = (jacobian_twist @ covariance @ jacobian_twist.T).ravel().tolist()
