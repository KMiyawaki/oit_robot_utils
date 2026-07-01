#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import datetime
import math
import os
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from oit_robot_utils.pose_conversions import *


class PoseCsvExporter(Node):
    def __init__(self):
        super().__init__('export_pose_to_csv')

        # パラメータを宣言
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        # パラメータを取得
        parent_frame = self.get_parameter('parent_frame').value
        child_frame = self.get_parameter('child_frame').value

        self.buffer = Buffer()
        # Attach a TransformListener to populate the buffer using this node
        self.tl = TransformListener(self.buffer, self)
        # Use this node for the pose getter so spin_once operates on the same node
        self.pose_getter = TFPoseGetter(
            self, self.buffer, parent_frame, child_frame, timeout_sec=0.01)

    def wait_for_pose(self, timeout_sec: float = 10.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            pose = self.pose_getter.get_pose()
            if pose is not None:
                return pose
            rclpy.spin_once(self, timeout_sec=0.5)
        return None


def make_filename(output_name: Optional[str], output_dir: str) -> str:
    """
    Determines the final output file path.
    - If output_name is an absolute path, it is used as is.
    - If output_name is a relative path or just a filename, it's joined with output_dir.
    - If output_name is empty, a timestamped filename is generated in output_dir.
    """
    if output_name and os.path.isabs(output_name):
        # output_name is absolute, use it directly
        final_dir = os.path.dirname(output_name)
        if final_dir:
            os.makedirs(final_dir, exist_ok=True)
        return output_name

    # The base directory for output is output_dir.
    os.makedirs(output_dir, exist_ok=True)

    if output_name:
        # output_name is relative or just a filename
        return os.path.join(output_dir, output_name)
    else:
        # output_name is empty, create a timestamped file in output_dir
        now = datetime.datetime.now()
        timestamped_filename = now.strftime('%Y%m%d_%H%M%S.%f')[:-3] + '.csv'
        return os.path.join(output_dir, timestamped_filename)


def main():
    rclpy.init()
    node = PoseCsvExporter()

    # パラメータを宣言
    node.declare_parameter('output_name', '')
    # The default value for output_dir is defined in the YAML file.
    node.declare_parameter('output_dir', '')
    node.declare_parameter('count', 0)
    node.declare_parameter('distance_threshold', 0.1)
    node.declare_parameter('timeout', 10.0)
    node.declare_parameter('waypoint_threshold', 1.5)
    node.declare_parameter('append', False)

    # パラメータを取得
    output_name = node.get_parameter('output_name').value
    output_dir = os.path.expanduser(node.get_parameter('output_dir').value)
    count = node.get_parameter('count').value
    distance_threshold = node.get_parameter('distance_threshold').value
    timeout = node.get_parameter('timeout').value
    waypoint_threshold = node.get_parameter('waypoint_threshold').value
    append = node.get_parameter('append').value
    # parent_frame/child_frameはログ出力用に取得
    parent_frame = node.get_parameter('parent_frame').value
    child_frame = node.get_parameter('child_frame').value

    filename = make_filename(output_name, output_dir)

    # Appendモードなら既存ファイルの最後のIDを読む
    next_id = 1
    write_mode = 'w'
    if append and os.path.exists(filename):
        write_mode = 'a'
        # 既存ファイルの最後のIDを読み取る
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f if line.strip()
                         and not line.strip().startswith('#')]
                if lines:
                    last_id = int(lines[-1].split(',')[0])
                    next_id = last_id + 1
        except Exception as e:
            node.get_logger().warn(
                f'Could not read last ID from {filename}: {e}')
    elif os.path.exists(filename) and not append:
        node.get_logger().error(f'Output file already exists: {filename}')
        return 1

    try:
        node.get_logger().info(f'Writing waypoints to {filename}')
        with open(filename, write_mode, newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            if write_mode == 'w':
                writer.writerow(['#ID', 'コマンド', 'x', 'y', '閾値'])

            row_id = next_id
            recorded_count = 0
            last_recorded_pose = None

            if count <= 0:
                node.get_logger().info('Infinite capture mode started. Press Ctrl+C to stop.')
            else:
                node.get_logger().info(
                    f'Starting to capture {count} waypoints.')

            while rclpy.ok():
                # countが指定されている場合、記録回数に達したらループを抜ける
                if count > 0 and recorded_count >= count:
                    break

                pose = node.wait_for_pose(timeout_sec=timeout)
                if pose is None:
                    if recorded_count == 0:
                        node.get_logger().error(
                            f'Failed to receive TF transform {parent_frame} -> {child_frame} within timeout.'
                        )
                        return 2
                    else:
                        node.get_logger().error(
                            'Lost TF transform while capturing rows.')
                        return 3

                should_record = False
                if last_recorded_pose is None:
                    # 最初の1点は無条件で記録
                    should_record = True
                else:
                    dist = math.hypot(
                        pose.x - last_recorded_pose.x, pose.y - last_recorded_pose.y)
                    if dist >= distance_threshold:
                        should_record = True

                if should_record:
                    writer.writerow([
                        row_id,
                        'goto_point',
                        f'{pose.x:.6f}',
                        f'{pose.y:.6f}',
                        f'{waypoint_threshold:.6f}',
                    ])
                    csvfile.flush()
                    node.get_logger().info(
                        f'Wrote waypoint {row_id}: x={pose.x:.6f}, y={pose.y:.6f}')

                    last_recorded_pose = pose
                    row_id += 1
                    recorded_count += 1

                # CPU使用率を抑えつつ、ROSのコールバックを処理するための短いスピン
                rclpy.spin_once(node, timeout_sec=0.1)

        node.get_logger().info(f'Successfully created/updated CSV: {filename}')
        return 0
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
        if os.path.exists(filename) and os.path.getsize(filename) > 0:
            node.get_logger().info(f'Partially saved to {filename}')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
