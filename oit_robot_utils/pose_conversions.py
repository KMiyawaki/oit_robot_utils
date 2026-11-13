#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import rclpy
import rclpy.time
import tf2_ros
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (Pose2D, PoseStamped, PoseWithCovarianceStamped,
                               Quaternion)
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def get_yaw_from_quaternion(orientation: Quaternion) -> float:
    """クォータニオンから Yaw 角（ラジアン）を抽出するヘルパー。"""
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, _, yaw = euler_from_quaternion(q)
    return float(yaw)


def pose2d_from_odom(msg: Odometry) -> Pose2D:
    """Odometry メッセージから 2D 姿勢 (x, y, theta) を抽出する。"""
    pose = Pose2D()
    pose.x = float(msg.pose.pose.position.x)
    pose.y = float(msg.pose.pose.position.y)
    pose.theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
    return pose


def velocity2d_from_odom(msg: Odometry) -> tuple[float, float, float]:
    """Odometry メッセージから 2D 速度 (vx, vy, omega) を抽出する。"""
    return (
        float(msg.twist.twist.linear.x),
        float(msg.twist.twist.linear.y),
        float(msg.twist.twist.angular.z),
    )


def pose_stamped_from(
    x: float,
    y: float,
    yaw: float,
    time_msg: TimeMsg,
    frame_id: str = 'map',
) -> PoseStamped:
    """(x, y, yaw) から PoseStamped メッセージを生成する。"""
    ps = PoseStamped()
    ps.header.stamp = time_msg
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    q = quaternion_from_euler(0.0, 0.0, float(yaw))
    ps.pose.orientation.x = float(q[0])
    ps.pose.orientation.y = float(q[1])
    ps.pose.orientation.z = float(q[2])
    ps.pose.orientation.w = float(q[3])
    return ps


def pose2d_from_amcl(msg: PoseWithCovarianceStamped) -> Pose2D:
    """AMCL の PoseWithCovarianceStamped メッセージから 2D 姿勢 (x, y, theta) を抽出する。"""
    pose = Pose2D()
    pose.x = float(msg.pose.pose.position.x)
    pose.y = float(msg.pose.pose.position.y)
    pose.theta = get_yaw_from_quaternion(msg.pose.pose.orientation)
    return pose


class TFPoseGetter:
    """TF2 バッファから指定フレーム間の 2D 姿勢を取得するヘルパー。"""

    def __init__(
        self,
        node: Node,
        tf_buffer: tf2_ros.Buffer,
        from_frame: str = 'map',
        to_frame: str = 'base_link',
        timeout_sec: float = 0.01,
    ) -> None:
        self.node = node
        self.tf_buffer = tf_buffer
        self.from_frame = from_frame
        self.to_frame = to_frame
        self.timeout_sec = timeout_sec

    def get_pose(self, time: rclpy.time.Time | None = None) -> Pose2D | None:
        try:
            lookup_time = time if time is not None else rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.from_frame,
                self.to_frame,
                lookup_time,
                timeout=Duration(seconds=self.timeout_sec),
            )
            pose = Pose2D()
            pose.x = float(trans.transform.translation.x)
            pose.y = float(trans.transform.translation.y)
            pose.theta = get_yaw_from_quaternion(trans.transform.rotation)
            return pose
        except Exception as e:
            self.node.get_logger().info(
                f'lookup_transform failed: {e}', throttle_duration_sec=1.0
            )
            return None
