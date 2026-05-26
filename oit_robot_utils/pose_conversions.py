#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from tf2_ros import Duration, tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def get_yaw_from_quaternion(orientation):
    """クォータニオンからYaw角(ラジアン)を抽出するヘルパー"""
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, _, yaw = euler_from_quaternion(q)
    return yaw


def pose2d_from_odom(msg):
    """
    Odometryメッセージから (x, y, yaw) を抽出する
    """
    pose = Pose2D()
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y
    pose.theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
    return pose


def velocity2d_from_odom(msg):
    """
    Odometryメッセージから速度を抽出する
    """
    return (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)


def pose_stamped_from(x, y, yaw, time_msg, frame_id='map'):
    """
    (x, y, yaw) から PoseStampedメッセージを生成する
    """
    ps = PoseStamped()
    ps.header.stamp = time_msg
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    q = quaternion_from_euler(0, 0, yaw)
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def pose2d_from_amcl(msg):
    """
    AMCLのPoseWithCovarianceStampedメッセージから (x, y, yaw) を抽出する
    """
    pose = Pose2D()
    pose.x = msg.pose.pose.position.x
    pose.y = msg.pose.pose.position.y
    pose.theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
    return pose


class TFPoseGetter:
    def __init__(self, node, tf_buffer, from_frame='map', to_frame='base_link', timeout_sec=5):
        self.node = node
        self.tf_buffer = tf_buffer
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.timeout_sec = timeout_sec

    def get_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.from_frame,
                self.to_frame,
                rclpy.time.Time(seconds=0),
                timeout=Duration(seconds=self.timeout_sec)
            )
            pose = Pose2D()
            pose.x = trans.transform.translation.x
            pose.y = trans.transform.translation.y
            pose.theta = get_yaw_from_quaternion(trans.transform.rotation)
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.node.get_logger().debug(f'TF lookup failed: {e}')
            return None
