#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""将 MAVROS 本地估计转换到共享 world 帧，使用已知起点和水平尺度。

world_xy = local_xy_scale * local_xy + offset_xy；world_z = local_z + offset_z。
速度和协方差使用对应变换；start_positions 优先于手动水平偏移。
相机光学姿态由配置安装角和云台模型计算，不使用仿真真值。"""
import os
import sys

# catkin's devel executable evaluates this source without adding its directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rospy
import numpy as np
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from world_frame_transform import transform_odometry, transform_position, validate_scale
from start_positions import validate_start_positions


class OdomWorldRelay:
    def __init__(self):
        start_key = rospy.get_param("~start_position_key", "")
        if start_key:
            positions = validate_start_positions(rospy.get_param("~start_positions"))
            if start_key not in positions:
                raise ValueError('未知起点编号: ' + str(start_key))
            self.offset_x, self.offset_y = positions[start_key]
        else:
            self.offset_x = rospy.get_param("~offset_x", 0.0)
            self.offset_y = rospy.get_param("~offset_y", 0.0)
        self.offset_z = rospy.get_param("~offset_z", 0.0)
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.local_xy_scale = validate_scale(rospy.get_param(
            "/swarm_coordinates/local_xy_scale", 1.0))
        self.offset = (self.offset_x, self.offset_y, self.offset_z)

        odom_in = rospy.get_param("~odom_in", "")
        odom_out = rospy.get_param("~odom_out", "")
        pose_in = rospy.get_param("~pose_in", "")
        pose_out = rospy.get_param("~pose_out", "")

        # pose 输出为相机光学系 RDF 到 world 的姿态；MAVROS 输入为机体系 FLU。
        # 仅相机 pose 叠加光学旋转，odom 保留机体姿态。
        self.apply_cam_tf = rospy.get_param("~apply_cam_transform", True)
        cam02body = np.array([[0.0, 0.0, 1.0, 0.0],
                              [-1.0, 0.0, 0.0, 0.0],
                              [0.0, -1.0, 0.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])

        # 安装下倾随云台偏航旋转：R_cam_to_world = Rz(yaw_body) @ Ry(pitch_mount) @ R_opt2cam。
        # 旋转顺序不能交换，安装角须与模型一致。
        self.cam_pitch_deg = rospy.get_param("~cam_pitch_deg", 25.0)
        q_opt2cam = tft.quaternion_from_matrix(cam02body)   # [x,y,z,w]
        q_pitch = tft.quaternion_from_euler(0.0, np.deg2rad(self.cam_pitch_deg), 0.0)
        # cam02body = T_cam_to_body, 其旋转部分转四元数 (用于 q_cam_world = q_body_world * q_cam_to_body)
        # 叠加静态下倾: q_cam_to_body = q_pitch ⊗ q_opt2cam
        self.q_cam_to_body = tft.quaternion_multiply(q_pitch, q_opt2cam)

        # gimbal_stabilized=True 时云台世界稳定 roll/pitch，只用机体 yaw 计算相机姿态。
        # False 时使用机体完整姿态。
        self.gimbal_stabilized = rospy.get_param("~gimbal_stabilized", True)

        self.odom_pub = None
        self.pose_pub = None

        if odom_in and odom_out:
            self.odom_pub = rospy.Publisher(odom_out, Odometry, queue_size=10)
            rospy.Subscriber(odom_in, Odometry, self.odom_cb, tcp_nodelay=True)
        if pose_in and pose_out:
            self.pose_pub = rospy.Publisher(pose_out, PoseStamped, queue_size=10)
            rospy.Subscriber(pose_in, PoseStamped, self.pose_cb, tcp_nodelay=True)

        rospy.loginfo(
            "[odom_relay] offset=(%.2f,%.2f,%.2f) frame=%s cam_tf=%s gimbal_stab=%s"
            " cam_pitch=%.1fdeg local_xy_scale=%.12f | odom %s->%s, pose %s->%s",
            self.offset_x, self.offset_y, self.offset_z, self.frame_id, self.apply_cam_tf,
            self.gimbal_stabilized, self.cam_pitch_deg, self.local_xy_scale,
            odom_in, odom_out, pose_in, pose_out)

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        rotation = tft.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        transform_odometry(msg, self.offset, self.local_xy_scale, rotation)
        msg.header.frame_id = self.frame_id
        self.odom_pub.publish(msg)

    def pose_cb(self, msg):
        transform_position(msg.pose.position, self.offset, self.local_xy_scale)
        msg.header.frame_id = self.frame_id
        if self.apply_cam_tf:
            # 机体姿态(base_link→world) → 相机光学位姿(optical→world):
            #   q_cam_world = q_mount_world ⊗ q_cam_to_body
            q_body = [msg.pose.orientation.x, msg.pose.orientation.y,
                      msg.pose.orientation.z, msg.pose.orientation.w]
            if self.gimbal_stabilized:
                # 云台把 roll/pitch 稳定到 0, yaw 跟随机体 → 相机挂点姿态只有 yaw
                yaw = tft.euler_from_quaternion(q_body)[2]
                q_mount = tft.quaternion_from_euler(0.0, 0.0, yaw)
            else:
                q_mount = q_body
            q_cam = tft.quaternion_multiply(q_mount, self.q_cam_to_body)
            msg.pose.orientation.x, msg.pose.orientation.y, \
                msg.pose.orientation.z, msg.pose.orientation.w = q_cam
        self.pose_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("odom_world_relay", anonymous=True)
    try:
        OdomWorldRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
