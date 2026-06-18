#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import sys

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from oit_robot_utils.pose_conversions import TFPoseGetter


class TfEchoNode(Node):
    def __init__(self, parent_frame, child_frame, rate_hz=10.0):
        super().__init__('tf_echo_py')
        self.buffer = Buffer()
        self.tl = TransformListener(self.buffer, self)
        self.pose_getter = TFPoseGetter(
            self, self.buffer, parent_frame, child_frame, timeout_sec=0.01)
        self.create_timer(1.0 / rate_hz, self.timer_cb)

    def timer_cb(self):
        pose = self.pose_getter.get_pose()
        if pose is None:
            return

        self.get_logger().info(
            f'x={pose.x:.2f}, y={pose.y:.2f}, theta={math.degrees(pose.theta):.2f}(deg)')


def main():
    rclpy.init()
    parent = 'map'
    child = 'base_link'
    if len(sys.argv) >= 3:
        parent = sys.argv[1]
        child = sys.argv[2]
    node = None
    try:
        node = TfEchoNode(parent, child)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
