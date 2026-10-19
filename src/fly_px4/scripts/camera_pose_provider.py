#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Camera geometry with strictly bracketed sensor buffers.
Convert configured camera-IMU reference to world and recover gimbal translation
from body/camera rotations. Invalid geometry uses a labelled body lever-arm approximation."""
import bisect
import threading
from collections import deque

import numpy as np

try:
    from camera_gimbal_kinematics import camera_offset_body, load_gimbal_geometry
except ImportError:
    camera_offset_body = load_gimbal_geometry = None


OPTICAL_TO_PHYSICAL = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def normalized_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        return None
    norm = float(np.linalg.norm(q))
    return q / norm if norm > 1e-9 else None


def quaternion_rotation(q):
    q = normalized_quaternion(q)
    if q is None:
        raise ValueError('invalid quaternion')
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def camera_imu_reference_rotation(reference, initial_rpy_deg=(0., 25., 0.)):
    """Return world-from-reference for the configured camera IMU plugin.
    Legacy startup-relative orientation needs LEFT multiplication by the known initial
    physical-camera pose; the world-reference plugin needs no mounting correction."""
    if reference not in ('world', 'initial'):
        raise ValueError('camera_imu_reference must be world or initial')
    rpy = np.asarray(initial_rpy_deg, dtype=float)
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        raise ValueError('camera_imu_initial_rpy_deg must contain three finite angles')
    if reference == 'world':
        return np.eye(3)
    roll, pitch, yaw = np.deg2rad(rpy) / 2.
    cr, sr, cp, sp, cy, sy = (np.cos(roll), np.sin(roll), np.cos(pitch),
                             np.sin(pitch), np.cos(yaw), np.sin(yaw))
    return quaternion_rotation([sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy])


def slerp(q0, q1, fraction):
    dot = float(np.dot(q0, q1))
    if dot < 0.:
        q1, dot = -q1, -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        return normalized_quaternion((1-fraction)*q0 + fraction*q1)
    theta = np.arccos(dot)
    return (np.sin((1-fraction)*theta)*q0 + np.sin(fraction*theta)*q1) / np.sin(theta)


class TemporalPoseBuffer:
    """Reject invalid/late samples; never extrapolate beyond an endpoint.

    Epoch reset is explicit: the ROS adapter calls clear() on clock rollback so
    a late packet cannot be mistaken for a simulation restart.
    """
    def __init__(self, maxlen=1500):
        self.buf = deque(maxlen=maxlen)
        self.lock = threading.Lock()

    def clear(self):
        with self.lock:
            self.buf.clear()

    def push(self, t, pos, quat, sigma=0., angular_velocity=None):
        q = normalized_quaternion(quat)
        pos = np.asarray(pos, dtype=float)
        if (not np.isfinite(t) or t < 0 or q is None or pos.shape != (3,)
                or not np.all(np.isfinite(pos)) or not np.isfinite(sigma) or sigma < 0):
            return False
        omega = np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity, dtype=float)
        if omega.shape != (3,) or not np.all(np.isfinite(omega)):
            return False
        row = (float(t), pos.copy(), q, float(sigma), omega.copy())
        with self.lock:
            if self.buf and t < self.buf[-1][0]:
                return False
            if self.buf and t == self.buf[-1][0]:
                self.buf[-1] = row
            else:
                self.buf.append(row)
        return True

    def query(self, t, max_gap):
        if not np.isfinite(t) or max_gap <= 0:
            return None
        with self.lock:
            rows = list(self.buf)
        if not rows or t < rows[0][0] or t > rows[-1][0]:
            return None
        idx = bisect.bisect_left([r[0] for r in rows], t)
        if idx < len(rows) and rows[idx][0] == t:
            lo = hi = rows[idx]
        else:
            lo, hi = rows[idx-1], rows[idx]
        span = hi[0] - lo[0]
        if span > max_gap:
            return None
        a = 0. if span == 0 else (t-lo[0])/span
        quat = slerp(lo[2], hi[2], a)
        pos = (1-a)*lo[1] + a*hi[1]
        omega = (1-a)*lo[4] + a*hi[4]
        rotation = quaternion_rotation(quat)
        # Exact matches use adjoining samples for a useful motion diagnostic.
        before, after = lo, hi
        if span == 0 and len(rows) > 1:
            before, after = rows[max(0, idx-1)], rows[min(len(rows)-1, idx+1)]
        motion_dt = after[0] - before[0]
        speed = float(np.linalg.norm(after[1]-before[1])/motion_dt) if motion_dt > 0 else 0.
        return dict(pos=pos, R=rotation, quat=quat, span=span, speed=speed,
                    position_sigma=max(lo[3], hi[3]), omega=omega,
                    angular_speed=float(np.linalg.norm(omega)),
                    yaw_rate=float((rotation @ omega)[2]))


class CameraPoseProvider:
    def __init__(self, offset, position_sigma=.10, attitude_sigma_deg=.5,
                 lever_arm_sigma=.03, imu_reference='world',
                 imu_initial_rpy_deg=(0., 25., 0.)):
        self.body = TemporalPoseBuffer()
        self.imu = TemporalPoseBuffer(maxlen=5000)
        self.relay = TemporalPoseBuffer()
        self.offset = np.asarray(offset, dtype=float)
        self.position_sigma = float(position_sigma)
        self.attitude_sigma = np.deg2rad(float(attitude_sigma_deg))
        self.lever_arm_sigma = float(lever_arm_sigma)
        self.imu_reference = imu_reference
        self.imu_reference_rotation = camera_imu_reference_rotation(
            imu_reference, imu_initial_rpy_deg)
        self.lever_arm_model = 'gimbal_kinematics'
        self.geometry = load_gimbal_geometry() if load_gimbal_geometry is not None else None

    def clear(self):
        self.body.clear()
        self.imu.clear()
        self.relay.clear()

    def query(self, t, body_gap=.06, imu_gap=.02, use_camera_imu=True):
        body = self.body.query(t, body_gap)
        if body is None:
            return None
        imu = self.imu.query(t, imu_gap) if use_camera_imu else None
        if imu is not None:
            imu['R'] = self.imu_reference_rotation @ imu['R']
            imu['yaw_rate'] = float((imu['R'] @ imu['omega'])[2])
        relay = self.relay.query(t, body_gap) if imu is None else None
        if imu is None and relay is None:
            return None
        r_cam = imu['R'] @ OPTICAL_TO_PHYSICAL if imu is not None else relay['R']
        axis = r_cam[:, 2]
        body_yaw = float(np.arctan2(body['R'][1, 0], body['R'][0, 0]))
        cam_yaw = float(np.arctan2(axis[1], axis[0]))
        imu_span = imu['span'] if imu is not None else None
        ang_speed = imu['angular_speed'] if imu is not None else 0.
        offset, lever_model = self.offset, 'body_rotation_fixed_offset_approx'
        if (self.lever_arm_model == 'gimbal_kinematics' and imu is not None
                and camera_offset_body is not None and self.geometry is not None):
            inferred = camera_offset_body(body['R'], imu['R'], self.geometry)
            if inferred is not None:
                offset, lever_model = inferred, 'gimbal_kinematics_from_orientation'
        return dict(pos=body['pos'] + body['R'] @ offset, R=r_cam,
                    body_position=body['pos'].tolist(),
                    depth_valid=imu is not None,
                    source='camera_imu' if imu is not None else 'relay_navigation',
                    imu_reference=self.imu_reference if imu is not None else None,
                    lever_arm_model=lever_model,
                    body_span=body['span'], imu_span=imu_span, speed=body['speed'],
                    body_yaw=body_yaw, camera_yaw=cam_yaw,
                    camera_body_yaw_diff=float(np.arctan2(np.sin(cam_yaw-body_yaw), np.cos(cam_yaw-body_yaw))),
                    yaw_rate=imu['yaw_rate'] if imu is not None else None,
                    position_sigma=max(self.position_sigma, body['position_sigma']),
                    attitude_sigma=max(self.attitude_sigma,
                                       imu['position_sigma'] if imu is not None else 0.,
                                       (imu_span or 0.)*ang_speed*.5),
                    lever_arm_sigma=self.lever_arm_sigma,
                    time_position_sigma=body['speed']*body['span']*.5)


class CameraModel:
    """Raw pinhole/Brown-Conrady CameraInfo geometry, without ROS or OpenCV."""
    def __init__(self, width, height, K, D=(), distortion_model='plumb_bob', frame_id=''):
        self.width, self.height = int(width), int(height)
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.D = np.asarray(D, dtype=float)
        if (self.width <= 0 or self.height <= 0 or not np.all(np.isfinite(self.K))
                or self.K[0, 0] <= 0 or self.K[1, 1] <= 0
                or not np.allclose(self.K[2], [0., 0., 1.])
                or not np.all(np.isfinite(self.D))
                or len(self.D) not in (0, 4, 5)
                or distortion_model not in ('', 'plumb_bob')):
            raise ValueError('unsupported/invalid CameraInfo')
        self.inv_K = np.linalg.inv(self.K)
        self.frame_id = frame_id

    @property
    def fx(self):
        return float(self.K[0, 0])

    @property
    def fy(self):
        return float(self.K[1, 1])

    @property
    def cx(self):
        return float(self.K[0, 2])

    @property
    def cy(self):
        return float(self.K[1, 2])

    def aligned_with(self, other):
        return (other is not None and self.width == other.width and self.height == other.height
                and np.allclose(self.K, other.K, rtol=1e-6, atol=1e-6)
                and np.allclose(np.pad(self.D, (0, 5-len(self.D))),
                                np.pad(other.D, (0, 5-len(other.D))), rtol=1e-6, atol=1e-8))

    def rays(self, u, v):
        u, v = np.broadcast_arrays(np.asarray(u, dtype=float), np.asarray(v, dtype=float))
        rays = np.stack((u, v, np.ones_like(u)), axis=-1) @ self.inv_K.T
        if self.D.size and np.max(np.abs(self.D)) > 1e-7:
            k1, k2, p1, p2, k3 = np.pad(self.D, (0, 5-len(self.D)))
            xd, yd = rays[..., 0].copy(), rays[..., 1].copy()
            x, y = xd.copy(), yd.copy()
            for _ in range(10):
                r2 = x*x + y*y
                radial = 1+k1*r2+k2*r2*r2+k3*r2*r2*r2
                x, y = ((xd-2*p1*x*y-p2*(r2+2*x*x))/radial,
                        (yd-p1*(r2+2*y*y)-2*p2*x*y)/radial)
            rays[..., 0], rays[..., 1] = x, y
        return rays


def depth_surface_cluster(depth, box, model, p_cam, r_cam, min_pixels=20,
                          min_valid_ratio=.10, max_mad=.35, cutoff=10.):
    """Backproject a dominant connected torso surface, never choose nearest.

    This is a foreground guard rather than person segmentation. A homogeneous
    occluder can still need rejection by box geometry/track consistency.
    """
    diag = dict(reason='no_depth', support_pixels=0, valid_ratio=0., mad=None)
    if depth is None or depth.shape != (model.height, model.width):
        return None, diag
    x1, y1, x2, y2 = box
    uc, bw, bh = (x1+x2)*.5, x2-x1, y2-y1
    r0, r1 = max(0, int(np.floor(y1+.25*bh))), min(model.height, int(np.ceil(y1+.55*bh)))
    c0, c1 = max(0, int(np.floor(uc-.15*bw))), min(model.width, int(np.ceil(uc+.15*bw)))
    patch = depth[r0:r1, c0:c1]
    if not patch.size:
        diag['reason'] = 'empty_roi'
        return None, diag
    valid = np.isfinite(patch) & (patch > .1) & (patch < cutoff)
    diag['valid_ratio'] = float(valid.sum())/patch.size
    if valid.sum() < min_pixels or diag['valid_ratio'] < min_valid_ratio:
        diag['reason'] = 'depth_coverage'
        return None, diag
    # Four-neighbour depth continuity. A second substantial separated surface
    # is ambiguous; don't let its smaller distance give it priority.
    visited = np.zeros_like(valid)
    components = []
    nr, nc = patch.shape
    for seed_r, seed_c in zip(*np.nonzero(valid)):
        if visited[seed_r, seed_c]:
            continue
        visited[seed_r, seed_c] = True
        stack, rows, cols = [(seed_r, seed_c)], [], []
        while stack:
            rr, cc = stack.pop()
            rows.append(rr)
            cols.append(cc)
            for rn, cn in ((rr-1, cc), (rr+1, cc), (rr, cc-1), (rr, cc+1)):
                if (0 <= rn < nr and 0 <= cn < nc and valid[rn, cn]
                        and not visited[rn, cn]
                        and abs(float(patch[rn, cn]-patch[rr, cc])) <= .20):
                    visited[rn, cn] = True
                    stack.append((rn, cn))
        components.append((np.asarray(rows), np.asarray(cols)))
    components.sort(key=lambda c: len(c[0]), reverse=True)
    rows, cols = components[0]
    values = patch[rows, cols]
    med = float(np.median(values))
    mad = float(np.median(np.abs(values-med)))
    diag.update(support_pixels=len(rows), valid_ratio=float(len(rows))/patch.size, mad=mad)
    if len(rows) < min_pixels or diag['valid_ratio'] < min_valid_ratio or mad > max_mad:
        diag['reason'] = 'cluster_quality'
        return None, diag
    if len(components) > 1 and len(components[1][0]) >= .35*len(rows):
        other_r, other_c = components[1]
        if abs(float(np.median(patch[other_r, other_c]))-med) > .25:
            diag['reason'] = 'ambiguous_surfaces'
            return None, diag
    # A sliver at the edge of the ROI is insufficient support for its centre.
    central = (cols >= .25*(nc-1)) & (cols <= .75*(nc-1))
    if not np.any(central):
        diag['reason'] = 'off_centre_cluster'
        return None, diag
    points_c = model.rays(cols+c0, rows+r0)*values[:, None]
    points_w = points_c @ np.asarray(r_cam).T + np.asarray(p_cam)
    xy = np.median(points_w[:, :2], axis=0)
    diag['reason'] = 'accepted_surface'
    diag['surface_xy'] = xy.tolist()
    diag['surface_scatter'] = float(np.max(1.4826*np.median(np.abs(points_w[:, :2]-xy), axis=0)))
    return xy, diag
