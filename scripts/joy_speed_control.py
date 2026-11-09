#!/usr/bin/env python3
import math
import time
from typing import Any, TypeVar

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import Joy

from oit_robot_utils.joy_button_monitor import JoyButtonMonitor

# モジュールレベルで定義するプライベートな TypeVar
_T = TypeVar('_T', int, float, str, bool)


class JoySpeedControl(Node):
    class Param:
        name: str           # パラメータ名（例: 'scale_linear_turbo.x'）
        min_val: float      # 最小値
        max_val: float      # 最大値
        step: float         # 変化ステップ幅
        current: float | None = None  # 現在値（初期値は None）
        is_radian: bool

        @staticmethod
        def declare_and_get(node: Node, name: str, default_val: _T) -> _T:
            """パラメータを宣言し、指定された型にキャストして安全に取得するスタティックヘルパー"""
            node.declare_parameter(name, default_val)
            param = node.get_parameter(name)
            if param is None or param.value is None:
                return default_val
            return type(default_val)(param.value)

        def __init__(self, node, name, min_val, max_val, step, is_radian=False):
            self.name = name
            self.min_val = self.declare_and_get(
                node, f'{self.name}.min', min_val)
            self.max_val = self.declare_and_get(
                node, f'{self.name}.max', max_val)
            self.step = self.declare_and_get(node, f'{self.name}.step', step)
            self.is_radian = is_radian

        def set_current(self, current) -> None:
            self.current = current
            self.clamp()

        def clamp(self) -> None:
            """現在の値を min_val ~ max_val の範囲に収める"""
            if self.current is not None:
                self.current = max(self.min_val, min(
                    self.max_val, self.current))

        def change(self, addar):
            """値を増減させる（変化があったら True を返す）"""
            if self.current is None or self.step <= 0.0:
                return False
            old_val = self.current
            self.current = self.current + addar
            self.clamp()
            return self.current != old_val

        def step_up(self) -> bool:
            return self.change(self.step)

        def step_down(self) -> bool:
            return self.change(-self.step)

        def __str__(self):
            if self.is_radian:
                return f'{self.name} min:{math.degrees(self.min_val):.1f} deg/s max:{math.degrees(self.max_val):.1f} deg/s cur:{math.degrees(self.current):.1f} deg/s step:{math.degrees(self.step):.1f} deg/s'
            else:
                return f'{self.name} min:{self.min_val:.1f} m/s max:{self.max_val:.1f} m/s cur:{self.current:.1f} m/s step:{self.step:.1f} m/s'

    def __init__(self):
        super().__init__('joy_speed_control')

        self.teleop_node_name = self.Param.declare_and_get(
            self, 'teleop_node_name', '/teleop_node')
        self.speed_down_button = self.Param.declare_and_get(
            self, 'speed_down_button', 4)
        self.speed_up_button = self.Param.declare_and_get(
            self, 'speed_up_button', 5)

        self.target_params: dict[str, JoySpeedControl.Param] = {}
        n = 'scale_linear_turbo.x'
        self.target_params[n] = JoySpeedControl.Param(self, n, 0.5, 1.12, 0.1)
        n = 'scale_angular_turbo.yaw'
        self.target_params[n] = JoySpeedControl.Param(
            self, n, math.radians(20), math.radians(60), 0.0, True)

        self.btn_monitor = JoyButtonMonitor()
        self.client = AsyncParameterClient(self, self.teleop_node_name)
        self.sub_joy = None

    def start(self):
        """相手ノードから確実に初期パラメータを取得できるまでループし、完了後にサブスクライバを開始する"""
        self.get_logger().info(
            f'Waiting for {self.teleop_node_name} parameter service...')

        while rclpy.ok() and not self.client.services_are_ready():
            self.get_logger().info(
                f'Waiting for {self.teleop_node_name} service to become available...')
            time.sleep(1.0)

        param_names = list(self.target_params.keys())
        while rclpy.ok():
            self.get_logger().info('Fetching initial parameters...')
            try:
                future = self.client.get_parameters(param_names)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done() and future.result() is not None:
                    values = future.result().values
                    for v, param in zip(self.target_params.values(), values):
                        if param.type == Parameter.Type.DOUBLE.value:
                            v.set_current(float(param.double_value))
                    if all(p.current is not None for p in self.target_params.values()):
                        break
            except Exception as e:
                self.get_logger().warn(f'Fetch attempt failed: {e}')
            time.sleep(1.0)

        self.get_logger().info('\n'+str(self))
        self.get_logger().info('Started subscribing to /joy.')
        self.sub_joy = self.create_subscription(Joy, '/joy', self.on_joy, 10)

    def on_joy(self, msg: Joy):
        self.btn_monitor.update_from_rosmsg(msg)
        updated = {}
        if self.btn_monitor.is_pressed(self.speed_up_button):
            for k, v in self.target_params.items():
                if v.step_up():
                    updated[k] = v.current
        elif self.btn_monitor.is_pressed(self.speed_down_button):
            for k, v in self.target_params.items():
                if v.step_down():
                    updated[k] = v.current
        self.sync_parameters(updated)

    def sync_parameters(self, name_value):
        if not name_value or not self.client.services_are_ready():
            return
        params_to_set = [
            Parameter(name, value=val)
            for name, val in name_value.items()
        ]
        future = self.client.set_parameters(params_to_set)
        future.add_done_callback(self._on_param_set_done)

    def _on_param_set_done(self, future: Any):
        try:
            if future.done() == False or future.result() is None:
                return
            for r in future.result().results:
                if r.successful == False:
                    self.get_logger().info(
                        f'Failed to set parameter on {self.teleop_node_name} {r.reason}')
            self.get_logger().info('\n'+str(self))
        except Exception as e:
            self.get_logger().error(f'Parameter service call failed: {e}')

    def __str__(self) -> str:
        return '\n'.join(str(v) for v in self.target_params.values())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JoySpeedControl()
        node.start()
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
