#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from tf2_msgs.msg import TFMessage


class RemoveMapTfNode(Node):
    def __init__(self):
        super().__init__('remove_map_tf')
        self.remove_frame_id = self.declare_parameter(
            'remove_frame_id', 'map').value

        # /tf 標準の QoS に合わせる
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_tf = self.create_publisher(TFMessage, '/tf', tf_qos)
        self.sub_tf = self.create_subscription(
            TFMessage, '/tf_remap', self.callback, tf_qos
        )

    def callback(self, msg: TFMessage):
        if not msg.transforms:
            return

        filtered_transforms = []
        for transform in msg.transforms:
            if transform.header.frame_id != self.remove_frame_id:
                filtered_transforms.append(transform)
            else:
                pass
                # self.get_logger().info(
                #     f'{self.remove_frame_id} frame removed!',
                #     throttle_duration_sec=5.0,
                # )

        if filtered_transforms:
            out_msg = TFMessage()
            out_msg.transforms = filtered_transforms
            self.pub_tf.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RemoveMapTfNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
