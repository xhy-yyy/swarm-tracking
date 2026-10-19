#!/usr/bin/env python3
"""在独立 ROS master 验证高度带与雷达点云合并。
覆盖巡逻停用、下降建记忆、扫描面低于檐口后保留障碍、归航后停用。
需先构建 fly_px4 并 source 工作空间。"""
import math
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))

EAVE_X, EAVE_Z = 7.0, (3.047, 3.793)
NS = 'typhoon_h480_0'


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def main():
    port = free_port()
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port,
               ROS_HOME=tempfile.mkdtemp(prefix='smoke_band_'), ROS_IP='127.0.0.1')
    os.environ.update(env)
    procs = [subprocess.Popen(['roscore', '-p', str(port)], env=env,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)]
    time.sleep(3.0)
    procs.append(subprocess.Popen(
        ['rosrun', 'fly_px4', 'body_band_cloud.py', '__name:=body_band_1',
         '_ns:=' + NS, '_plane_z:=4.5', '_offset_z:=0.0'], env=env))
    procs.append(subprocess.Popen(
        ['rosrun', 'fly_px4', 'laser_scan_to_world_cloud.py', '__name:=lidar_cloud_1',
         '~scan_in:=/%s/scan' % NS, '~odom_in:=/%s/odom_world_relay' % NS,
         '~cloud_out:=/%s/lidar_cloud_world' % NS,
         '~cloud_out_3d:=/%s/lidar_cloud_world_3d' % NS,
         '~band_in:=/%s/body_band_cloud' % NS,
         '~merged_out:=/%s/obstacle_cloud_world' % NS, '_plane_z:=4.5'], env=env))
    try:
        return run(procs)
    finally:
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()


def run(procs):
    import rosgraph
    import rospy
    import tf.transformations as tft
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan, PointCloud2
    from std_msgs.msg import Float32MultiArray
    import body_band_cloud as bb
    from test_body_band_cloud import camera_rotation, render_canopy

    rospy.init_node('smoke_body_band', anonymous=True)
    odom_pub = rospy.Publisher('/%s/odom_world_relay' % NS, Odometry, queue_size=5)
    pose_pub = rospy.Publisher('/%s/pose_relay' % NS, PoseStamped, queue_size=5)
    req_pub = rospy.Publisher('/%s/body_band_request' % NS, Float32MultiArray, queue_size=2)
    scan_pub = rospy.Publisher('/%s/scan' % NS, LaserScan, queue_size=2)
    depth_pub = rospy.Publisher('/%s/realsense/depth_camera/depth/image_raw' % NS, Image,
                                queue_size=1)
    got = {'band': [], 'merged': [], 'overhead': []}
    rospy.Subscriber('/%s/body_band_cloud' % NS, PointCloud2,
                     lambda m: got['band'].append((time.time(), bb.cloud_xyz(m))))
    rospy.Subscriber('/%s/body_band_overhead' % NS, PointCloud2,
                     lambda m: got['overhead'].append((time.time(), bb.cloud_xyz(m))))
    rospy.Subscriber('/%s/obstacle_cloud_world' % NS, PointCloud2,
                     lambda m: got['merged'].append((time.time(), bb.cloud_xyz(m))))
    master = rosgraph.Master('/smoke_body_band')
    r_cam = camera_rotation(0.0)
    m4 = np.eye(4)
    m4[:3, :3] = r_cam
    q_cam = tft.quaternion_from_matrix(m4)

    def depth_subscribed():
        topic = '/%s/realsense/depth_camera/depth/image_raw' % NS
        for name, nodes in master.getSystemState()[1]:
            if name == topic and '/body_band_1' in nodes:
                return True
        return False

    def scan_for(z):
        lidar = z - 0.1125
        n = 512
        ang = -math.pi + np.arange(n) * (2 * math.pi / n)
        ranges = np.full(n, np.inf)
        if EAVE_Z[0] <= lidar <= EAVE_Z[1]:
            c = np.cos(ang)
            with np.errstate(divide='ignore'):
                t = (EAVE_X - (-0.04)) / c
            y = t * np.sin(ang)
            hit = (c > 0) & (np.abs(y) <= 9.0) & (t <= 20.0)
            ranges[hit] = t[hit]
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'laser_2d'
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2 * math.pi / n
        msg.range_min, msg.range_max = 0.5, 20.0
        msg.ranges = ranges.tolist()
        return msg

    def step(z, target, active, seconds):
        t_end = time.time() + seconds
        k = 0
        while time.time() < t_end and not rospy.is_shutdown():
            zz = z(time.time()) if callable(z) else z
            now = rospy.Time.now()
            o = Odometry()
            o.header.stamp = now
            o.header.frame_id = 'world'
            o.pose.pose.position.z = zz
            o.pose.pose.orientation.w = 1.0
            odom_pub.publish(o)
            p = PoseStamped()
            p.header = o.header
            p.pose.position.z = zz
            p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, \
                p.pose.orientation.w = q_cam
            pose_pub.publish(p)
            if k % 2 == 0:
                req_pub.publish(Float32MultiArray(data=[target, 1.0 if active else 0.0]))
                scan_pub.publish(scan_for(zz))
            if k % 5 == 0 and depth_pub.get_num_connections() > 0:
                body = np.array([0.0, 0.0, zz])
                d = render_canopy(bb.camera_position(body, r_cam), r_cam)
                img = Image()
                img.header.stamp = now
                img.height, img.width = d.shape
                img.encoding = '32FC1'
                img.step = d.shape[1] * 4
                img.data = d.tobytes()
                depth_pub.publish(img)
            k += 1
            time.sleep(0.02)

    def near_eave(pts):
        return int(np.sum((np.abs(pts[:, 0] - EAVE_X) < 0.35) & (np.abs(pts[:, 1]) < 9.5)))

    failures = []

    def check(ok, name):
        print(('PASS: ' if ok else 'FAIL: ') + name)
        if not ok:
            failures.append(name)

    time.sleep(2.0)
    # A 巡逻
    step(4.5, 4.5, False, 2.0)
    check(not depth_subscribed(), 'patrol: depth not subscribed')
    check(not got['band'], 'patrol: no band cloud published')
    # B 跟踪开始: 4.5 -> 3.2, 0.5m/s
    t0 = time.time()
    step(lambda t: max(3.2, 4.5 - 0.5 * (t - t0)), 3.2, True, 3.2)
    check(depth_subscribed(), 'track: depth subscribed')
    step(3.2, 3.2, True, 1.0)
    band_b = got['band'][-1][1] if got['band'] else np.empty((0, 3))
    check(near_eave(band_b) > 20, 'track 3.2: band cloud holds eave (%d pts)' % near_eave(band_b))
    # C 掉高 5cm: 雷达扫描面 3.0375 < 檐口底 3.047
    n_band = len(got['band'])
    step(3.15, 3.2, True, 1.5)
    band_c = got['band'][-1][1] if len(got['band']) > n_band else np.empty((0, 3))
    merged_c = got['merged'][-1][1] if got['merged'] else np.empty((0, 3))
    check(near_eave(band_c) > 20, 'dip to 3.15 (lidar under eave): eave kept (%d pts)'
          % near_eave(band_c))
    check(near_eave(merged_c) > 20 and np.allclose(merged_c[:, 2], 4.5),
          'EGO obstacle cloud contains eave at plane_z')
    over_c = got['overhead'][-1][1] if got['overhead'] else np.empty((0, 3))
    check(near_eave(over_c) > 20, 'eave seen while descending is overhead evidence (%d pts)'
          % near_eave(over_c))
    # D 归航完成
    step(4.5, 4.5, False, 2.0)
    n_band = len(got['band'])
    step(4.5, 4.5, False, 1.0)
    check(not depth_subscribed(), 'patrol again: depth unsubscribed')
    check(len(got['band']) == n_band, 'patrol again: band cloud stopped')
    for p in procs:
        if p.poll() is not None:
            failures.append('process exited: %s' % p.args)
    print('%d failures' % len(failures))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
