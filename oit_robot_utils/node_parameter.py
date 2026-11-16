#!/usr/bin/env python3
import array
import copy
import math
from typing import Any, TypeVar, cast

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

from oit_robot_utils.service_client import ServiceClientBlocking

T = TypeVar('T')


def declare_and_get(node: Node, name: str, default: T) -> T:
    return cast(T, node.declare_parameter(name, default).value)


def set_node_parameter(node: Node, service_name: str, param_name: str, value: Any, timeout_sec: float = 2.0) -> bool:
    """rclpy.parameter.Parameter を利用してパラメータを変更する汎用関数"""
    logger = node.get_logger()
    try:
        param_msg = Parameter(name=param_name, value=value).to_parameter_msg()
    except Exception as e:
        logger.error(
            f'未対応のパラメータ型です: {param_name}={value} ({type(value)}) / {e}')
        return False

    client_wrapper = ServiceClientBlocking(node, SetParameters, service_name)
    response = client_wrapper.call(SetParameters.Request(
        parameters=[param_msg]), timeout_sec=timeout_sec)
    client_wrapper.destroy()

    if response is None or not response.results:
        logger.error(
            f'パラメータ設定失敗/タイムアウト: {service_name} -> {param_name}={value}')
        return False
    result = response.results[0]
    if result.successful:
        logger.info(f'パラメータ変更成功: {param_name} -> {value}')
    else:
        logger.error(f'パラメータ変更拒否: {param_name} ({result.reason})')
    return result.successful


def get_node_parameter(node: Node, service_name: str, param_name: str, timeout_sec: float = 2.0) -> Any:
    """rclpy.parameter.Parameter を利用してパラメータ値を取得する"""
    client_wrapper = ServiceClientBlocking(node, GetParameters, service_name)
    response = client_wrapper.call(GetParameters.Request(
        names=[param_name]), timeout_sec=timeout_sec)
    client_wrapper.destroy()

    if response is None or not response.values:
        return None

    val = Parameter.from_parameter_msg(ParameterMsg(
        name=param_name, value=response.values[0])).value
    # double_array 等は array.array で返ることがあるため list に正規化
    if isinstance(val, array.array):
        return val.tolist()
    if isinstance(val, tuple):
        return list(val)
    return val


class ParameterEntry:
    """単一パラメータの取得・変更・復元・状態保持を完結させるクラス"""

    def __init__(self, owner_node: Node, target_node_name: str, name: str, fallback: Any = None):
        self._owner: Node = owner_node
        self.node_name: str = target_node_name.rstrip('/')
        self.name: str = name
        self._get_srv: str = f'{self.node_name}/get_parameters'
        self._set_srv: str = f'{self.node_name}/set_parameters'
        self.fallback: Any = self.normalize(
            fallback) if fallback is not None else None
        self.default: Any = None
        self.current: Any = None
        self.initialized: bool = False

    def set_fallback(self, fallback: Any) -> None:
        self.fallback = self.normalize(
            fallback) if fallback is not None else None

    def init(self) -> bool:
        """実機からデフォルト値を取得（失敗時はフォールバック）して確定する"""
        if self.initialized:
            return True
        fetched = get_node_parameter(self._owner, self._get_srv, self.name)
        if fetched is not None:
            norm = self.normalize(fetched)
            self.default = norm
            self.current = copy.deepcopy(norm)
            self.initialized = True
            return True
        if self.fallback is not None:
            self.default = copy.deepcopy(self.fallback)
            self.current = copy.deepcopy(self.fallback)
            self.initialized = True
            self._owner.get_logger().warn(
                f'[{self.node_name}] {self.name} の取得失敗。フォールバック値を使用: {self.fallback}'
            )
            return True
        return False

    def copy_default(self) -> Any:
        self.init()
        return copy.deepcopy(self.default)

    def set(self, value: Any) -> bool:
        self.init()
        norm_val = self.normalize(value)
        if self.initialized and self.is_equal(self.current, norm_val):
            return True
        if set_node_parameter(self._owner, self._set_srv, self.name, norm_val):
            self.current = norm_val
            return True
        return False

    def reset(self) -> bool:
        if not self.initialized or self.is_equal(self.current, self.default):
            return True
        if set_node_parameter(self._owner, self._set_srv, self.name, self.default):
            self.current = copy.deepcopy(self.default)
            return True
        return False

    @staticmethod
    def normalize(val: Any) -> Any:
        if isinstance(val, (array.array, tuple)):
            return list(val)
        return copy.deepcopy(val)

    @classmethod
    def is_equal(cls, a: Any, b: Any) -> bool:
        if isinstance(a, float) and isinstance(b, float):
            return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
        if isinstance(a, list) and isinstance(b, list):
            return len(a) == len(b) and all(cls.is_equal(x, y) for x, y in zip(a, b))
        return a == b

    def __str__(self) -> str:
        if not self.initialized:
            return f'{self.name}: <uninitialized>'
        status = '=' if self.is_equal(self.current, self.default) else '*'
        return f'{self.name}: {self.current} (default{status}{self.default})'


class NodeParameterOverride:
    """指定ノードの特定パラメータ群を ParameterEntry 単位で管理するコンテナクラス"""

    def __init__(
        self,
        owner_node: Node,
        target_node_name: str,
        param_name_and_fallbacks: dict[str, Any] | None = None,
    ):
        self._owner = owner_node
        self._node_name = target_node_name.rstrip('/')
        self._entries: dict[str, ParameterEntry] = {}
        if param_name_and_fallbacks:
            for k, v in param_name_and_fallbacks.items():
                self.add(k, fallback=v)

    def add(self, name: str, fallback: Any = None, fetch_now: bool = False) -> ParameterEntry:
        if name not in self._entries:
            self._entries[name] = ParameterEntry(
                self._owner, self._node_name, name, fallback=fallback)
        elif fallback is not None:
            self._entries[name].set_fallback(fallback)

        entry = self._entries[name]
        if fetch_now:
            entry.init()
        return entry

    def initialize(self) -> None:
        for entry in self._entries.values():
            entry.init()

    def copy_default(self, name: str) -> Any:
        return self.add(name).copy_default()

    def set(self, name: str, value: Any) -> bool:
        return self.add(name).set(value)

    def reset(self, name: str | None = None) -> bool:
        targets = [self._entries[name]] if (
            name is not None and name in self._entries) else list(self._entries.values())
        return all(entry.reset() for entry in targets)

    def __str__(self) -> str:
        if not self._entries:
            return f'NodeParameterOverride({self._node_name}: empty)'
        body = '\n'.join(str(entry) for entry in self._entries.values())
        return f'[{self._node_name}]\n{body}'


class ParameterOverrideManager:
    """複数ノードの NodeParameterOverride をノード名キーで統括管理するクラス"""

    def __init__(
        self,
        owner_node: Node,
        fallbacks: dict[tuple[str, str], Any] | None = None,
    ):
        self._owner = owner_node
        self._nodes: dict[str, NodeParameterOverride] = {}

        if fallbacks:
            for (node_name, param_name), fallback_val in fallbacks.items():
                self.node(node_name).add(param_name, fallback=fallback_val)

    @staticmethod
    def _normalize_node_name(node_name: str) -> str:
        name = node_name.strip().rstrip('/')
        return name if name.startswith('/') else f'/{name}'

    def node(
        self,
        target_node_name: str,
        param_name_and_fallbacks: dict[str, Any] | None = None,
    ) -> NodeParameterOverride:
        """対象ノードの NodeParameterOverride を取得（未登録なら自動生成して登録）"""
        key = self._normalize_node_name(target_node_name)
        if key not in self._nodes:
            self._nodes[key] = NodeParameterOverride(
                self._owner, key, param_name_and_fallbacks
            )
        elif param_name_and_fallbacks:
            for k, v in param_name_and_fallbacks.items():
                self._nodes[key].add(k, fallback=v)
        return self._nodes[key]

    def initialize(self) -> None:
        """登録済み全ノードの全パラメータのデフォルト値を取得・確定する"""
        for override in self._nodes.values():
            override.initialize()

    def copy_default(self, target_node_name: str, param_name: str) -> Any:
        """デフォルト値のディープコピーを取得する"""
        return self.node(target_node_name).copy_default(param_name)

    def set(self, target_node_name: str, param_name: str, value: Any) -> bool:
        """ノード名とパラメータ名を指定して値を変更する"""
        return self.node(target_node_name).set(param_name, value)

    def reset(self, target_node_name: str | None = None, param_name: str | None = None) -> bool:
        """指定パラメータ（省略時は対象ノード全体、両方省略時は全ノード全体）をデフォルト値に戻す"""
        if target_node_name is not None:
            return self.node(target_node_name).reset(param_name)
        return all(override.reset() for override in self._nodes.values())

    def __getitem__(self, target_node_name: str) -> NodeParameterOverride:
        return self.node(target_node_name)

    def __str__(self) -> str:
        if not self._nodes:
            return 'ParameterOverrideManager(empty)'
        return '\n\n'.join(str(override) for override in self._nodes.values())


def main():
    PARAM_MAX_VELOCITY = 'max_velocity'
    PARAM_MIN_VELOCITY = 'min_velocity'
    PARAM_SCALE_VELOCITY = 'scale_velocities'
    PARAM_OBSTACLE_LAYER = 'obstacle_layer.enabled'
    PARAM_INFLATION_RADIUS = 'inflation_layer.inflation_radius'
    PARAM_COST_SCALING_FACTOR = 'inflation_layer.cost_scaling_factor'
    rclpy.init()
    node = Node('test_velocity_smoother_override')
    params = ParameterOverrideManager(node)

    try:
        params.node('/velocity_smoother', {PARAM_MAX_VELOCITY: [0.7, 0.0, math.radians(40)],
                                           PARAM_MIN_VELOCITY: [-0.1, 0.0, -math.radians(40)],
                                           PARAM_SCALE_VELOCITY: False})
        params.node('/global_costmap/global_costmap', {PARAM_OBSTACLE_LAYER: True,
                                                       PARAM_INFLATION_RADIUS: 0.8,
                                                       PARAM_COST_SCALING_FACTOR: 3.0}
                    )
        params.node('/local_costmap/local_costmap',
                    {PARAM_OBSTACLE_LAYER: True,
                     PARAM_INFLATION_RADIUS: 0.5,
                     PARAM_COST_SCALING_FACTOR: 10.0})
        print('1. 初期化前:', params)
        params.initialize()
        print('2. 実機 取得後:', params)

        # 徐行ON（並進 0.2 m/s、必要なら旋回も左右対称に制限）
        max_vel = params['/velocity_smoother'].copy_default(PARAM_MAX_VELOCITY)
        max_vel[0] = 0.2
        params.set('/velocity_smoother', PARAM_MAX_VELOCITY, max_vel)
        params.set('/global_costmap/global_costmap',
                   PARAM_OBSTACLE_LAYER, False)
        params.set('/local_costmap/local_costmap',
                   PARAM_OBSTACLE_LAYER, False)
        print('3. 徐行ON後:', params)
        input('>>> ros2 param get /velocity_smoother max_velocity 等で確認後、Enterでリセットします...')
        # リセット（徐行OFF）
        params.reset()
        print('4. リセット後:', params)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
