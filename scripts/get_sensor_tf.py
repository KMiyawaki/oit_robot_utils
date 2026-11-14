#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion


class SensorTfExporter(Node):
    """指定トピック群の header.frame_id を取得し、base_link との静的TFを抽出するノード"""

    def __init__(self, topic_names: list[str], parent_frame: str = 'base_link'):
        super().__init__('export_sensor_tf')
        self.topic_names = topic_names
        self.parent_frame = parent_frame
        self.child_frames: dict[str, str] = {}
        self._subs: dict[str, object] = {}

        self.buffer = Buffer()
        self.tl = TransformListener(self.buffer, self)

    def _resolve_topic_type(self, topic_name: str):
        target = topic_name if topic_name.startswith('/') else f'/{topic_name}'
        for name, types in self.get_topic_names_and_types():
            if (name == target or name == topic_name) and types:
                return get_message(types[0])
        return None

    def _make_cb(self, topic_name: str):
        def _cb(msg) -> None:
            if topic_name not in self.child_frames and hasattr(msg, 'header') and msg.header.frame_id:
                self.child_frames[topic_name] = msg.header.frame_id
        return _cb

    def fetch_all(self, timeout_sec: float = 10.0) -> dict[str, tuple[str, tuple[float, ...]]]:
        deadline = time.monotonic() + timeout_sec
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        results: dict[str, tuple[str, tuple[float, ...]]] = {}

        while rclpy.ok() and time.monotonic() < deadline and len(results) < len(self.topic_names):
            for topic in self.topic_names:
                if topic in results:
                    continue

                if topic not in self._subs:
                    msg_cls = self._resolve_topic_type(topic)
                    if msg_cls is not None:
                        self._subs[topic] = self.create_subscription(
                            msg_cls, topic, self._make_cb(topic), qos
                        )

                child_frame = self.child_frames.get(topic)
                if not child_frame:
                    continue

                if child_frame == self.parent_frame:
                    results[topic] = (
                        child_frame, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                    continue

                try:
                    if self.buffer.can_transform(
                        self.parent_frame,
                        child_frame,
                        Time(),
                        timeout=Duration(seconds=0.1),
                    ):
                        tf_msg = self.buffer.lookup_transform(
                            self.parent_frame, child_frame, Time()
                        )
                        t = tf_msg.transform.translation
                        q = tf_msg.transform.rotation
                        roll, pitch, yaw = euler_from_quaternion(
                            [q.x, q.y, q.z, q.w])
                        results[topic] = (
                            child_frame,
                            (float(t.x), float(t.y), float(t.z),
                             float(roll), float(pitch), float(yaw)),
                        )
                except Exception:
                    pass

            if len(results) < len(self.topic_names):
                rclpy.spin_once(self, timeout_sec=0.1)

        return results


def main() -> int:
    topics = sys.argv[1:] if len(sys.argv) >= 2 else ['scan_low']
    parent_frame = 'base_link'

    rclpy.init()
    node = SensorTfExporter(topics, parent_frame)
    try:
        results = node.fetch_all(timeout_sec=10.0)
        missing = [t for t in topics if t not in results]
        if missing:
            print(
                f'Error: Timed out for topics={missing} '
                f'(detected frames={node.child_frames}) -> "{parent_frame}"',
                file=sys.stderr,
            )
            return 1

        names_str = ', '.join(f"'{t}'" for t in topics)
        print('topics:')
        print(f'  names: [{names_str}]')
        for topic in topics:
            child_frame, (x, y, z, roll, pitch, yaw) = results[topic]
            print(f'  {topic}: # {child_frame} -> {parent_frame}')
            print(f'    x: {x:.6f}')
            print(f'    y: {y:.6f}')
            print(f'    z: {z:.6f}')
            print(f'    roll: {roll:.6f}')
            print(f'    pitch: {pitch:.6f}')
            print(f'    yaw: {yaw:.6f}')
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
