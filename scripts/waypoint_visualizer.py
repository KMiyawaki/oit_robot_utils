#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from oit_robot_utils.waypoint_manager import WayPointManager


class WayPointVisualizer(Node):

    def __init__(self):
        super().__init__("waypoint_visualizer")

        # launchファイルから受け取るパラメータ
        self.declare_parameter("csv_path", "")

        # Publisher
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "waypoints",
            10
        )

        # CSV読み込み
        csv_path = self.get_parameter(
            "csv_path"
        ).get_parameter_value().string_value

        if not os.path.exists(csv_path):
            self.get_logger().error(f"CSVが見つかりません: {csv_path}")
            return

        self.manager = WayPointManager.load_csv(csv_path)

        self.get_logger().info(
            f"{len(self.manager)}個のウェイポイントを読み込みました。"
        )

        # 1秒ごとに配信
        self.timer = self.create_timer(
            1.0,
            self.publish_waypoints
        )

    def publish_waypoints(self):

        marker_array = MarkerArray()

        for wp in self.manager:

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "waypoints"
            marker.id = wp.id

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = wp.x
            marker.pose.position.y = wp.y
            marker.pose.position.z = 0.0

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):

    rclpy.init(args=args)

    node = WayPointVisualizer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()