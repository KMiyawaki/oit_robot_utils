#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging


class LastMessages:
    def __init__(self, logger=None):
        self._logger = logger or logging.getLogger(__name__)
        self._data = {}

    def add_topic(self, topic: str, initial_value=None):
        # 必要なデータはこのメソッドをプログラムの最初に呼び出して追加しておくこと。
        # 辞書の構造を動的に変更しないならばスレッドセーフになる。
        topic = self.normalize_topic(topic)
        if topic in self._data:
            self._logger.warning(f"すでに登録されているトピックです: {topic}")
            return
        self._data[topic] = initial_value

    def update(self, topic: str, value):
        topic = self.normalize_topic(topic)
        if topic not in self._data:
            # 開発中にすぐ気づけるように警告を出す（または例外を投げる）
            self._logger.error(f"Contextに登録されていないトピックです: {topic}")
            return
        # 既存のキーに対する代入なら、ロックなしでアトミック
        self._data[topic] = value

    def get(self, topic: str):
        topic = self.normalize_topic(topic)
        if topic not in self._data:
            self._logger.error(f"Contextに登録されていないトピックです: {topic}")
            return None
        # 既存のキーの読み取りなら、ロックなしでアトミック
        return self._data[topic]

    def normalize_topic(self, topic):
        """
        トピック名の正規化:
        """
        return topic.strip().lstrip('/').rstrip('/')
