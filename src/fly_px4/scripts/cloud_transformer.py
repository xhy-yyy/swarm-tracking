#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class CloudTransformer:
    def __init__(self):
        self.odom = None
        self.skip = rospy.get_param("~skip", 3)
        self.pub = rospy.Publisher("~cloud_world", PointCloud2, queue_size=10)
        rospy.Subscriber("~cloud_in", PointCloud2, self.cloud_cb, queue_size=1)
        rospy.Subscriber("~odom_in", Odometry, self.odom_cb, queue_size=10)
        self.last_print = rospy.Time.now()
        self.print_interval = rospy.Duration(2.0)
        rospy.loginfo("[cloud_tf] Ready. skip=%d (no rotation, same as EGO cloudCallback)", self.skip)

    def odom_cb(self, msg):
        self.odom = msg

    def cloud_cb(self, msg):
        if self.odom is None:
            return
        w, h = msg.width, msg.height
        if w == 0 or h == 0:
            return
        raw = np.frombuffer(msg.data, dtype=np.float32)
        stride = msg.point_step // 4
        num_pts = len(raw) // stride
        if num_pts == 0 or stride < 3:
            return
        raw = raw[:num_pts * stride].reshape(num_pts, stride)

        offsets = {f.name: f.offset for f in msg.fields}
        xi, yi, zi = offsets.get('x', 0)//4, offsets.get('y', 4)//4, offsets.get('z', 8)//4
        if max(xi, yi, zi) >= stride:
            return

        pts = np.column_stack([raw[:, xi], raw[:, yi], raw[:, zi]])
        pts_img = pts.reshape(h, w, 3)
        pts_ds = pts_img[::self.skip, ::self.skip, :].reshape(-1, 3)
        valid = ~np.isnan(pts_ds).any(axis=1) & (np.abs(pts_ds).sum(axis=1) > 1e-6)
        pts_ds = pts_ds[valid]
        if len(pts_ds) == 0:
            return

        # 不加旋转 — 和 EGO cloudCallback 一样，只用平移
        # 仿真中无人机近悬停，相机系 ≈ 世界系
        t = np.array([self.odom.pose.pose.position.x,
                      self.odom.pose.pose.position.y,
                      self.odom.pose.pose.position.z])
        world = pts_ds + t

        # 打印最近障碍物
        now = rospy.Time.now()
        if now - self.last_print > self.print_interval:
            self.last_print = now
            dists = np.linalg.norm(pts_ds, axis=1)
            mask = (dists > 1.5) & (dists < 8.0)
            obs = world[mask]
            if len(obs) > 0:
                near = obs[np.argsort(dists[mask])[:min(5, len(obs))]]
                rospy.loginfo("[cloud_tf] 无人机(%.1f,%.1f,%.1f) 检测到障碍物:", t[0], t[1], t[2])
                for pt in near:
                    rospy.loginfo("  → world(%.2f, %.2f, %.2f) 距离=%.1fm", pt[0], pt[1], pt[2],
                                  np.linalg.norm(pt - t))
            else:
                rospy.loginfo("[cloud_tf] 无人机(%.1f,%.1f,%.1f) 无远处障碍物", t[0], t[1], t[2])

        out = pc2.create_cloud_xyz32(msg.header, world.tolist())
        out.header.frame_id = "world"
        out.width = len(world)
        out.height = 1
        self.pub.publish(out)

if __name__ == "__main__":
    rospy.init_node("cloud_transformer")
    CloudTransformer()
    rospy.spin()
