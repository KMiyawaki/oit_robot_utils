#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from enum import IntEnum


class JoyButtonMonitor:
    """
    ジョイスティックのボタン状態を管理し、
    update時に各ボタンの状態をIntEnumの配列として保持するクラス
    """
    class State(IntEnum):
        UNCHANGED = 0
        PRESSED = 1   # 0 -> 1 (押された瞬間)
        RELEASED = 2  # 1 -> 0 (離された瞬間)

    def __init__(self):
        self.last_buttons = []
        self.states = []

    def update_from_rosmsg(self, msg):
        """ROS2のJoyメッセージからbuttonsを取り出して更新する"""
        self.update(msg.buttons)

    def update(self, current_buttons):
        """新しいJoyメッセージを受け取り、全ボタンの状態を1つの配列で更新する"""
        current_list = list(current_buttons)

        if not self.last_buttons or len(self.last_buttons) != len(current_list):
            self.last_buttons = [0] * len(current_list)
            self.states = [self.State.UNCHANGED] * len(current_list)

        new_states = []
        for prev, curr in zip(self.last_buttons, current_list):
            if prev == 0 and curr == 1:
                new_states.append(self.State.PRESSED)
            elif prev == 1 and curr == 0:
                new_states.append(self.State.RELEASED)
            else:
                new_states.append(self.State.UNCHANGED)

        self.states = new_states
        self.last_buttons = current_list

    def is_pressed(self, button_index):
        """指定したボタンが今回新たに押された瞬間か判定する"""
        if 0 <= button_index < len(self.states):
            return self.states[button_index] == self.State.PRESSED
        return False

    def is_released(self, button_index):
        """指定したボタンが今回離された瞬間か判定する"""
        if 0 <= button_index < len(self.states):
            return self.states[button_index] == self.State.RELEASED
        return False
