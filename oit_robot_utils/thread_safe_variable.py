#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from threading import Lock


class ThreadSafeVariable:
    """
    スレッドセーフな箱。
    最新データのみを保持し、読み書き時に自動で競合を回避する。
    ただし、ROS2メッセージのスレッド間での受け渡しは、原則参照渡しとし、データ書き換えはしない。
    したがって、メッセージをこのクラスでスレッドセーフにする必要はない。
    """

    def __init__(self, initial_value):
        self._var = initial_value
        self._lock = Lock()

    def set(self, value):
        """
        生産者用：新しいデータを投入する。
        既存の古いデータは自動的に破棄される。
        """
        with self._lock:
            self._var = value

    def get(self):
        """
        消費者用：最新のデータを取得する。
        データがない場合は None を返す。
        """
        with self._lock:
            return self._var
