#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from threading import Thread

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from oit_robot_utils.pose_conversions import (pose2d_from_amcl,
                                              pose_stamped_from)
from oit_robot_utils.waypoint_manager import WayPointManager


class NavCommander(Node):
    def __init__(self, csv_path: str):
        super().__init__('waypoint_nav_commander')
        self.nav = BasicNavigator()
        self.scan = None
        self.amcl_pose = None
        self.odom = None
        self.waypoints = WayPointManager.load_csv(csv_path)

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cp_group = ReentrantCallbackGroup()
        self.sub_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10, callback_group=self.cp_group)
        self.sub_amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.amcl_pose_callback,
            10,
            callback_group=self.cp_group,
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=self.cp_group,
        )

    def scan_callback(self, msg):
        self.scan = msg

    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg

    def odom_callback(self, msg):
        self.odom = msg

    def loop(self):
        self.get_logger().info('Waypoint navigation loop start')
        rate = self.create_rate(10)
        self.nav.waitUntilNav2Active()

        while self.pub_cmd_vel.get_subscription_count() == 0 and rclpy.ok():
            self.get_logger().info('Waiting for cmd_vel subscriber...', throttle_duration_sec=1.0)
            rate.sleep()

        if len(self.waypoints) == 0:
            self.get_logger().error('No waypoints loaded from CSV.')
            return

        for waypoint in self.waypoints:
            self.get_logger().info(
                f'Sending waypoint {waypoint.id}: ({waypoint.x:.3f}, {waypoint.y:.3f}) threshold={waypoint.threshold:.3f}')
            target_pose = pose_stamped_from(
                waypoint.x,
                waypoint.y,
                0.0,
                self.get_clock().now().to_msg(),
            )
            self.nav.goToPose(target_pose)

            while rclpy.ok() and not self.nav.isTaskComplete():
                if self.amcl_pose:
                    pose_2d = pose2d_from_amcl(self.amcl_pose)
                    self.get_logger().info(
                        f'現在位置: x={pose_2d.x:.2f}, y={pose_2d.y:.2f}, theta={pose_2d.theta:.2f}'
                    )
                feedback = self.nav.getFeedback()
                distance = feedback.distance_remaining if feedback is not None else float(
                    'inf')
                self.get_logger().info(f'目標までの距離: {distance:.2f} m')
                if distance < waypoint.threshold:
                    self.get_logger().info(
                        f'WayPoint {waypoint.id} reached by threshold {waypoint.threshold:.3f} m')
                    break
                rate.sleep()

        self.get_logger().info('All waypoints complete')
        raise KeyboardInterrupt()


def parse_args():
    parser = argparse.ArgumentParser(
        description='Follow waypoints loaded from a CSV file.'
    )
    parser.add_argument('csv_file', help='Path to the waypoint CSV file.')
    return parser.parse_args()


def main(args=None):
    parsed = parse_args() if args is None else args
    rclpy.init()
    node = None
    try:
        node = NavCommander(parsed.csv_file)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        Thread(target=executor.spin, daemon=True).start()
        node.loop()
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
