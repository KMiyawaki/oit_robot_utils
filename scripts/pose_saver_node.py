#!/usr/bin/env python3
import math
import os
import re
import tempfile
import time

import rclpy
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from oit_robot_utils.node_parameter import declare_and_get
from oit_robot_utils.pose_conversions import pose2d_from_amcl


class PoseSaverNode(Node):
    def __init__(self):
        super().__init__('pose_saver_node')

        self.file_path = declare_and_get(self, 'file_path', '')
        self.env_var_name = declare_and_get(
            self, 'env_var_name', 'INITIAL_POSE')
        self.pose_topic = declare_and_get(self, 'pose_topic', '/amcl_pose')
        self.startup_delay = declare_and_get(self, 'startup_delay', 5.0)
        self.period = declare_and_get(self, 'period', 0.5)

        if not self.file_path or not os.path.isfile(self.file_path):
            self.get_logger().warn(
                f'file_path not found: "{self.file_path}". Node disabled.')
            return

        if not self.env_var_name:
            self.get_logger().warn('env_var_name is empty. Node disabled.')
            return

        self.pose_line_re = re.compile(
            rf'^(\s*)(#\s*)?(export\s+{re.escape(self.env_var_name)}=)(["\']?).*?\4(\s*(?:#.*)?)$'
        )

        self.latest_pose: Pose2D | None = None
        self.start_time = time.monotonic()
        self.last_str: str | None = None

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self._on_pose,
            10,
        )

        self.get_logger().info(
            f'PoseSaverNode started: {self.pose_topic} -> {self.env_var_name} ({self.file_path})')
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            self.period, self._on_timer, clock=steady_clock)

    def _on_pose(self, msg: PoseWithCovarianceStamped):
        self.latest_pose = pose2d_from_amcl(msg)

    def _on_timer(self):
        if (time.monotonic() - self.start_time) < self.startup_delay:
            return

        if self.latest_pose is None:
            return

        pose = self.latest_pose
        yaw_deg = int(round(math.degrees(pose.theta)))
        pose_str = f'"{pose.x:.2f} {pose.y:.2f} {yaw_deg}"'
        if pose_str == self.last_str:
            return

        if self._write_atomic(pose_str):
            self.last_str = pose_str

    def _write_atomic(self, pose_str: str) -> bool:
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()

            handled = False
            new_lines = []
            for line in lines:
                stripped = line.rstrip('\r\n')
                m = self.pose_line_re.match(stripped)
                if m and not handled:
                    if m.group(2) is not None:
                        return True
                    new_lines.append(
                        f'{m.group(1)}{m.group(3)}{pose_str}{m.group(5)}\n')
                    handled = True
                else:
                    new_lines.append(line)

            if not handled:
                if new_lines and not new_lines[-1].endswith('\n'):
                    new_lines[-1] += '\n'
                new_lines.append(f'export {self.env_var_name}={pose_str}\n')

            dir_name = os.path.dirname(os.path.abspath(self.file_path))
            fd, tmp_path = tempfile.mkstemp(
                dir=dir_name, prefix='.settings_tmp_')
            try:
                with os.fdopen(fd, 'w', encoding='utf-8') as f:
                    f.writelines(new_lines)
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(tmp_path, self.file_path)
                return True
            except Exception:
                if os.path.exists(tmp_path):
                    os.remove(tmp_path)
                raise
        except Exception as e:
            self.get_logger().error(f'Failed to update {self.file_path}: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PoseSaverNode()
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
