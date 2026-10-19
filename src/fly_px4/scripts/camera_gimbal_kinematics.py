#!/usr/bin/env python3
"""Recover camera optical-centre translation from validated SDF gimbal geometry.
Z-X-Y joint rotations act about their anchors; invalid geometry or angles return None.
Only model geometry is read, without Gazebo state or services."""
import functools
import os
import xml.etree.ElementTree as ET

import numpy as np


def _default_model_sdf():
    configured = os.environ.get('SWARM_MODEL_SDF')
    if configured:
        return os.path.expanduser(configured)
    from pathlib import Path
    sibling = Path(__file__).resolve().parents[2] / 'swarm_simulation'
    if sibling.is_dir():
        return str(sibling / 'models/typhoon_h480_xhyyy/typhoon_h480_xhyyy.sdf')
    import rospkg
    return os.path.join(rospkg.RosPack().get_path('swarm_simulation'),
                        'models/typhoon_h480_xhyyy/typhoon_h480_xhyyy.sdf')


DEFAULT_MODEL_SDF = _default_model_sdf()


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return (np.eye(3) * np.cos(angle) + (1. - np.cos(angle)) * np.outer(axis, axis)
            + np.sin(angle) * skew)


def _pose(element):
    if element is None:
        return np.zeros(6)
    if element.get('relative_to') or element.get('frame'):
        raise ValueError('Non-default SDF pose reference is unsupported')
    value = np.fromstring(element.text or '', sep=' ')
    if value.shape != (6,) or not np.all(np.isfinite(value)):
        raise ValueError('Invalid six-element SDF pose')
    return value


class GimbalGeometry:
    def __init__(self, anchors, limits, camera_offset, mount_rotation):
        self.anchors = np.asarray(anchors, dtype=float)
        self.limits = np.asarray(limits, dtype=float)
        self.camera_offset = np.asarray(camera_offset, dtype=float)
        self.mount_rotation = np.asarray(mount_rotation, dtype=float)


@functools.lru_cache(maxsize=8)
def load_gimbal_geometry(model_sdf=DEFAULT_MODEL_SDF):
    """Validate the zero-pose chain and obtain anchors, mounting pose and limits."""
    try:
        model = ET.parse(os.path.expanduser(model_sdf)).getroot().find('model')
        if model is None:
            return None
        links = ['base_link', 'cgo3_mount_link', 'cgo3_vertical_arm_link',
                 'cgo3_horizontal_arm_link', 'cgo3_camera_link']
        for name in links:
            link = model.find("link[@name='%s']" % name)
            if link is None or np.linalg.norm(_pose(link.find('pose'))) > 1e-9:
                return None
        mount = model.find("joint[@name='cgo3_mount_joint']")
        if (mount is None or mount.findtext('parent') != links[0]
                or mount.findtext('child') != links[1]
                or abs(float(mount.findtext('axis/limit/lower'))) > 1e-9
                or abs(float(mount.findtext('axis/limit/upper'))) > 1e-9):
            return None
        specs = [('cgo3_vertical_arm_joint', [0, 0, 1]),
                 ('cgo3_horizontal_arm_joint', [-1, 0, 0]),
                 ('cgo3_camera_joint', [0, -1, 0])]
        anchors, limits = [], []
        for index, (name, axis) in enumerate(specs):
            joint = model.find("joint[@name='%s']" % name)
            if (joint is None or joint.get('type') != 'revolute'
                    or joint.findtext('parent') != links[index + 1]
                    or joint.findtext('child') != links[index + 2]
                    or joint.findtext('axis/use_parent_model_frame') != '1'):
                return None
            xyz = np.fromstring(joint.findtext('axis/xyz') or '', sep=' ')
            if xyz.shape != (3,) or not np.allclose(xyz, axis, atol=1e-9):
                return None
            pose = _pose(joint.find('pose'))
            if np.linalg.norm(pose[3:]) > 1e-9:
                return None
            anchors.append(pose[:3])
            limits.append([float(joint.findtext('axis/limit/lower')),
                           float(joint.findtext('axis/limit/upper'))])
        include = next((i for i in model.findall('include')
                        if (i.findtext('uri') or '').strip() == 'model://realsense_camera'), None)
        camera_joint = model.find("joint[@name='realsense_camera_joint']")
        if (include is None or camera_joint is None or camera_joint.get('type') != 'fixed'
                or camera_joint.findtext('parent') != 'cgo3_camera_link'
                or camera_joint.findtext('child') != 'realsense_camera::link'
                or np.linalg.norm(_pose(camera_joint.find('pose'))) > 1e-9):
            return None
        pose = _pose(include.find('pose'))
        roll, pitch, yaw = pose[3:]
        mount_rotation = rotation([0, 0, 1], yaw) @ rotation([0, 1, 0], pitch) @ rotation([1, 0, 0], roll)
        limits = np.asarray(limits)
        if not np.all(np.isfinite(limits)) or np.any(limits[:, 0] > limits[:, 1]):
            return None
        # The inverse uses the nonsingular ZXY branch; roll limits must stay inside it.
        if limits[1, 0] <= -np.pi / 2 or limits[1, 1] >= np.pi / 2:
            return None
        return GimbalGeometry(anchors, limits, pose[:3], mount_rotation)
    except (OSError, ET.ParseError, ValueError, TypeError, StopIteration):
        return None


def camera_offset_body(R_body_world, R_imu_world, geometry=None):
    """Return image-time camera centre in body coordinates, or None.
    Inputs map body and physical-camera IMUs to world. Remove mounting rotation only
    for inverse joint kinematics; do not add it to the measured optical orientation."""
    if geometry is None:
        geometry = load_gimbal_geometry()
    if geometry is None:
        return None
    B, I = np.asarray(R_body_world, dtype=float), np.asarray(R_imu_world, dtype=float)
    for R in (B, I):
        if (R.shape != (3, 3) or not np.all(np.isfinite(R))
                or not np.allclose(R.T @ R, np.eye(3), atol=1e-6)
                or abs(np.linalg.det(R) - 1.) > 1e-6):
            return None
    J = B.T @ I @ geometry.mount_rotation.T
    b = float(np.arcsin(np.clip(J[2, 1], -1., 1.)))
    if abs(np.cos(b)) < 1e-4:
        return None
    a = float(np.arctan2(-J[0, 1], J[1, 1]))
    c = float(np.arctan2(-J[2, 0], J[2, 2]))
    angles = np.array([a, -b, -c])  # SDF signed Z, -X, -Y joint axes.
    tolerance = 0.03  # Allow minor soft-limit / EKF orientation discrepancies.
    if np.any(angles < geometry.limits[:, 0] - tolerance) or np.any(angles > geometry.limits[:, 1] + tolerance):
        return None
    rotations = [rotation([0, 0, 1], a), rotation([1, 0, 0], b), rotation([0, 1, 0], c)]
    if not np.allclose(rotations[0] @ rotations[1] @ rotations[2], J, atol=1e-6):
        return None
    centre = geometry.camera_offset.copy()
    for pivot, R in reversed(list(zip(geometry.anchors, rotations))):
        centre = pivot + R @ (centre - pivot)
    return centre if np.all(np.isfinite(centre)) else None
