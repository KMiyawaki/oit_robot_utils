#!/usr/bin/env python3
import array
import copy
import math
from typing import Any

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter

from oit_robot_utils.service_client import ServiceClientBlocking


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


class NodeParameterOverride:
    """指定ノードの特定パラメータのデフォルト値を記憶し、一時変更・復元を行う汎用クラス"""

    def __init__(self, owner_node, target_node_name: str, param_names: list[str], fallbacks: dict[str, Any] | None = None):
        self._owner = owner_node
        self._node_name = target_node_name.rstrip('/')
        self._get_srv = f'{self._node_name}/get_parameters'
        self._set_srv = f'{self._node_name}/set_parameters'
        self._param_names = list(param_names)
        self._fallbacks = dict(fallbacks) if fallbacks else {}

        self._defaults: dict[str, Any] = {}
        self._current: dict[str, Any] = {}
        self._initialized = False

    def initialize(self) -> None:
        """起動時（Nav2アクティブ化後）にデフォルト値を取得してキャッシュする"""
        for name in self._param_names:
            val = get_node_parameter(self._owner, self._get_srv, name)
            if val is not None:
                self._defaults[name] = self._normalize(val)
                self._current[name] = self._normalize(val)
            elif name in self._fallbacks:
                self._defaults[name] = self._normalize(self._fallbacks[name])
                self._current[name] = self._normalize(self._fallbacks[name])
                self._owner.get_logger().warn(
                    f'[{self._node_name}] {name} の取得失敗。フォールバック値を使用: {self._fallbacks[name]}'
                )
        self._initialized = True

    def copy_default(self, name: str) -> Any:
        if not self._initialized:
            self.initialize()
        return copy.deepcopy(self._defaults.get(name))

    def set(self, name: str, value: Any) -> bool:
        if not self._initialized:
            self.initialize()
        norm_val = self._normalize(value)
        if name in self._current and self._is_equal(self._current[name], norm_val):
            return True
        if set_node_parameter(self._owner, self._set_srv, name, norm_val):
            self._current[name] = self._normalize(norm_val)
            return True
        return False

    def reset(self, name: str | None = None) -> bool:
        if not self._initialized:
            return True
        targets = [name] if name is not None else self._param_names
        all_ok = True
        for key in targets:
            if key not in self._defaults:
                continue
            default_val = self._defaults[key]
            if key in self._current and self._is_equal(self._current[key], default_val):
                continue
            if set_node_parameter(self._owner, self._set_srv, key, default_val):
                self._current[key] = self._normalize(default_val)
            else:
                all_ok = False
        return all_ok

    def __str__(self) -> str:
        if not self._initialized:
            return f'NodeParameterOverride({self._node_name}: uninitialized)'

        items = []
        for name in self._param_names:
            default_val = self._defaults.get(name)
            current_val = self._current.get(name)
            status = '*' if not self._is_equal(current_val,
                                               default_val) else '='
            items.append(
                f'{name}: {current_val} (default{status}{default_val})')

        return f'[{self._node_name}]\n' + '\n'.join(items)

    def _normalize(self, val: Any) -> Any:
        if isinstance(val, (array.array, tuple)):
            return list(val)
        return copy.deepcopy(val)

    def _is_equal(self, a: Any, b: Any) -> bool:
        if isinstance(a, float) and isinstance(b, float):
            return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9)
        if isinstance(a, list) and isinstance(b, list):
            return len(a) == len(b) and all(
                self._is_equal(x, y) for x, y in zip(a, b)
            )
        return a == b


def main():
    PARAM_MAX_VELOCITY = 'max_velocity'
    PARAM_MIN_VELOCITY = 'min_velocity'
    PARAM_SCALE_VELOCITY = 'scale_velocities'
    PARAM_OBSTACLE_LAYER = 'obstacle_layer.enabled'
    PARAM_INFLATION_RADIUS = 'inflation_layer.inflation_radius'
    PARAM_COST_SCALING_FACTOR = 'inflation_layer.cost_scaling_factor'
    rclpy.init()
    node = Node('test_velocity_smoother_override')

    try:
        velocity_smoother_params = NodeParameterOverride(
            node, '/velocity_smoother',
            param_names=[PARAM_MAX_VELOCITY,
                         PARAM_MIN_VELOCITY,
                         PARAM_SCALE_VELOCITY],
            fallbacks={PARAM_MAX_VELOCITY: [0.7, 0.0, math.radians(40)],
                       PARAM_MIN_VELOCITY: [-0.1, 0.0, -math.radians(40)],
                       PARAM_SCALE_VELOCITY: False},
        )
        global_costmap_params = NodeParameterOverride(
            node, '/global_costmap/global_costmap',
            param_names=[PARAM_OBSTACLE_LAYER,
                         PARAM_INFLATION_RADIUS,
                         PARAM_COST_SCALING_FACTOR],
            fallbacks={PARAM_OBSTACLE_LAYER: True,
                       PARAM_INFLATION_RADIUS: 0.8,
                       PARAM_COST_SCALING_FACTOR: 3.0},
        )
        local_costmap_params = NodeParameterOverride(
            node, '/local_costmap/local_costmap',
            param_names=[PARAM_OBSTACLE_LAYER,
                         PARAM_INFLATION_RADIUS,
                         PARAM_COST_SCALING_FACTOR],
            fallbacks={PARAM_OBSTACLE_LAYER: True,
                       PARAM_INFLATION_RADIUS: 0.5,
                       PARAM_COST_SCALING_FACTOR: 10.0},
        )
        print('1. 初期化前:', velocity_smoother_params,
              global_costmap_params, local_costmap_params, sep='\n')

        velocity_smoother_params.initialize()
        global_costmap_params.initialize()
        local_costmap_params.initialize()
        print('2. 実機 /velocity_smoother 取得後:',
              velocity_smoother_params, global_costmap_params, local_costmap_params, sep='\n')

        # 徐行ON（並進 0.2 m/s、必要なら旋回も左右対称に制限）
        max_vel = velocity_smoother_params.copy_default('max_velocity')
        max_vel[0] = 0.2
        velocity_smoother_params.set('max_velocity', max_vel)
        global_costmap_params.set('obstacle_layer.enabled', False)
        local_costmap_params.set('obstacle_layer.enabled', False)
        print('3. 徐行ON後:', velocity_smoother_params, sep='\n')
        input('>>> ros2 param get /velocity_smoother max_velocity 等で確認後、Enterでリセットします...')
        # リセット（徐行OFF）
        velocity_smoother_params.reset()
        global_costmap_params.reset()
        local_costmap_params.reset()
        print('4. リセット後:', velocity_smoother_params,
              global_costmap_params, local_costmap_params, sep='\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
