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
            wp_id = int(wp.id)

            # --------------------------------------------------
            # 1. 赤い球体マーカー (SPHERE)
            # --------------------------------------------------
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()

            sphere_marker.ns = "waypoints_sphere"
            sphere_marker.id = wp_id

            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD

            sphere_marker.pose.position.x = wp.x
            sphere_marker.pose.position.y = wp.y
            sphere_marker.pose.position.z = 0.0

            sphere_marker.pose.orientation.w = 1.0
            scale = 1.0
            sphere_marker.scale.x = scale
            sphere_marker.scale.y = scale
            sphere_marker.scale.z = scale

            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 1.0

            marker_array.markers.append(sphere_marker)

            # --------------------------------------------------
            # 2. IDテキスト表示用マーカー (TEXT_VIEW_FACING)
            # --------------------------------------------------
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()

            text_marker.ns = "waypoints_text"
            text_marker.id = wp_id

            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # 球体の少し上（z = 0.35）に表示
            text_marker.pose.position.x = wp.x
            text_marker.pose.position.y = wp.y
            text_marker.pose.position.z = scale * 1.5

            # 表示するテキスト（ID）
            text_marker.text = str(wp.id)

            # 文字の高さ（スケール）
            text_marker.scale.z = scale

            # 文字の色（白色・不透明）
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):

    rclpy.init(args=args)

    node = WayPointVisualizer()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()