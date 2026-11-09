#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import IntEnum
from typing import Any, Sequence


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

    def update_from_rosmsg(self, msg: Any) -> None:
        """ROS2のJoyメッセージからbuttonsを取り出して更新する"""
        self.update(msg.buttons)

    def update(self, current_buttons: Sequence[int]) -> None:
        """新しいJoyメッセージを受け取り、全ボタンの状態を1つの配列で更新する"""
        current_list = list(current_buttons)

        # ボタン数が変わった（または初回）場合のみ再初期化
        if len(self.last_buttons) != len(current_list):
            self.last_buttons = [0] * len(current_list)
            self.states = [self.State.UNCHANGED] * len(current_list)

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
