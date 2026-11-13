#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

from collections.abc import Sequence
from enum import IntEnum

import rclpy
from sensor_msgs.msg import Joy


class JoyButtonMonitor:
    """
    ジョイスティックのボタン状態を管理し、
    update時に各ボタンの状態をIntEnumの配列として保持するクラス
    """
    class State(IntEnum):
        UNCHANGED = 0
        PRESSED = 1   # 0 -> 1 (押された瞬間)
        RELEASED = 2  # 1 -> 0 (離された瞬間)

    def __init__(self) -> None:
        self.last_buttons: list[int] = []
        self.states: list[JoyButtonMonitor.State] = []

    def update_from_rosmsg(self, msg: Joy) -> None:
        """ROS2のJoyメッセージからbuttonsを取り出して更新する"""
        self.update(msg.buttons)

    def update(self, current_buttons: Sequence[int]) -> None:
        """新しいJoyメッセージを受け取り、全ボタンの状態を1つの配列で更新する"""
        current_list = list(current_buttons)

        # ボタン数が変わった（または初回）場合のみ再初期化
        if len(self.last_buttons) != len(current_list):
            self.last_buttons = [0] * len(current_list)

        # エッジ検出（立ち上がり / 立ち下がり）
        self.states = [
            self.State.PRESSED if prev == 0 and curr == 1
            else self.State.RELEASED if prev == 1 and curr == 0
            else self.State.UNCHANGED
            for prev, curr in zip(self.last_buttons, current_list)
        ]
        self.last_buttons = current_list

    def is_pressed(self, button_index: int) -> bool:
        """指定したボタンが今回新たに押された瞬間か判定する"""
        if 0 <= button_index < len(self.states):
            return self.states[button_index] == self.State.PRESSED
        return False

    def is_released(self, button_index: int) -> bool:
        """指定したボタンが今回離された瞬間か判定する"""
        if 0 <= button_index < len(self.states):
            return self.states[button_index] == self.State.RELEASED
        return False


def main() -> None:
    rclpy.init()
    node = rclpy.create_node('joy_button_monitor_test')
    monitor = JoyButtonMonitor()

    def on_joy(msg: Joy) -> None:
        monitor.update_from_rosmsg(msg)
        for i in range(len(msg.buttons)):
            if monitor.is_pressed(i):
                node.get_logger().info(f'Button [{i}] PRESSED')
            elif monitor.is_released(i):
                node.get_logger().info(f'Button [{i}] RELEASED')

    node.create_subscription(Joy, '/joy', on_joy, 10)
    node.get_logger().info('/joy 待機中...')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
