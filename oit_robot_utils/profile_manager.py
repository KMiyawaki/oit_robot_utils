#!/usr/bin/env python3
from pathlib import Path
from typing import Any

import rclpy
import yaml
from rclpy.node import Node

from oit_robot_utils.launch_utils import PackagePath
from oit_robot_utils.node_parameter import (ParameterOverrideManager,
                                            normalize_node_name,
                                            tree_to_flat_dict)

NONE_PROFILE: str = 'none'


def load_profile_names(
    yaml_path: str | Path,
    root_key: str = 'nav_profiles',
) -> list[str]:
    """nav_profiles.yaml から定義済みのプロファイル名一覧を読み取る（GUIエディタ用）"""
    path = Path(yaml_path)
    if not path.exists():
        return []

    with path.open('r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}

    profiles = raw.get(root_key)
    if not isinstance(profiles, dict):
        return []

    return [str(name) for name in profiles.keys()]


class ProfileManager:
    """
    yaml を読み込み、既存の ParameterOverrideManager を通じて
    プロファイル単位での一括適用・リセットを行うステートレスなマネージャ。
    """

    def __init__(self, owner_node: Node):
        self._owner = owner_node
        self._logger = owner_node.get_logger()
        self._override: ParameterOverrideManager | None = None
        # {profile_name: {'/node_name': {'param.name', value}}}
        self._profiles: dict[str, dict[str, dict[str, Any]]] = {}

    @property
    def profile_names(self) -> list[str]:
        return list(self._profiles.keys())

    @property
    def override_manager(self) -> ParameterOverrideManager | None:
        return self._override

    def load(self, yaml_path: str | Path, required_profiles: list[str] = []) -> bool:
        if self.override_manager is not None:
            self.override_manager.reset()
            self._override = None
        if not Path(yaml_path).exists():
            self._logger.error(f'nav_profiles.yaml not found: {yaml_path}')
            return False

        with Path(yaml_path).open('r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}
            profiles_raw = raw.get('nav_profiles')
            if not isinstance(profiles_raw, dict):
                self._logger.error(
                    f"Invalid YAML in {yaml_path}: 'nav_profiles' key missing")
                return False

        self._profiles = {}
        self._override = ParameterOverrideManager(self._owner)

        for profile_name, nodes_dict in profiles_raw.items():
            if not isinstance(nodes_dict, dict):
                continue
            node_params: dict[str, dict[str, Any]] = {}
            for k, params_tree in nodes_dict.items():
                if not isinstance(params_tree, dict):
                    continue
                node_name = normalize_node_name(k)
                params_flat: dict[str, Any] = tree_to_flat_dict(params_tree)
                self._override.node(node_name).add_list(
                    list(params_flat.keys()))
                node_params[node_name] = params_flat
            self._profiles[profile_name] = node_params

        missing = [p for p in required_profiles if p not in self._profiles]
        if missing:
            self._logger.error(
                f'Essential profiles missing in {yaml_path}: {missing}')
            self._override = None
            self._profiles = {}
            return False

        self._logger.info(
            f'Loaded {len(self._profiles)} profiles: {list(self._profiles.keys())}'
        )
        if not self.initialize():
            self._logger.error('Failed to initialize')
            self._override = None
            self._profiles = {}
            return False
        return True

    def initialize(self) -> bool:
        if self.override_manager is None:
            return False
        """登録済み全パラメータの実機デフォルト値を事前取得する"""
        return self.override_manager.initialize()

    def apply(self, profile_name: str, reset_first: bool = True) -> bool:
        """
        指定したプロファイルを適用する。
        reset_first=True の場合、既存のオーバーライドを一旦デフォルトに戻してから適用する。
        """
        all_ok: bool = True
        if self._override is None:
            self._logger.error(f"Parameters are not loaded")
            return False
        if profile_name == NONE_PROFILE or not profile_name:
            return self._override.reset()
        if profile_name not in self._profiles:
            self._logger.error(f"Unknown profile: '{profile_name}'")
            return False

        if reset_first:
            if not self._override.reset():
                all_ok = False
                self._logger.warn('Partially failed to reset')

        for node_name, params in self._profiles[profile_name].items():
            if not self._override.set_params(node_name, params):
                all_ok = False

        if all_ok:
            self._logger.info(f"Applied nav profile: '{profile_name}'")
        else:
            self._logger.warn(
                f"Applied nav profile '{profile_name}' with partial errors")
        return all_ok

    def reset(self) -> bool:
        if self.override_manager is None:
            return False
        """変更したすべてのパラメータを初期値（default.yaml相当）へ復元する"""
        ok = self.override_manager.reset()
        if ok:
            self._logger.info('Reset all nav parameters to defaults.')
        else:
            self._logger.warn('Partially failed to reset params')
        return ok

    def __str__(self) -> str:
        return str(self._override)


def main() -> None:
    oit_robot_utils_path = PackagePath('oit_robot_utils')
    nav_profile = oit_robot_utils_path.join('config', 'nav_profiles.yaml')

    rclpy.init()
    node = Node('test_nav_profile_manager')
    manager = ProfileManager(node)

    try:
        print(f'=== 1. YAML読み込み: {nav_profile} ===')
        if not manager.load(nav_profile):
            print('YAMLの読み込みに失敗しました。パスを確認してください。')
            return
        print('利用可能なプロファイル:', manager.profile_names)

        print('\n=== 2. 実機デフォルト値の取得 (initialize) ===')
        manager.initialize()
        print(manager)

        # 定義されているプロファイルを順番にテスト
        test_targets = [p for p in (
            'slow', 'narrow', 'blind') if p in manager.profile_names]
        for prof in test_targets:
            input(f'\n>>> Enterキーを押すとプロファイル [{prof}] を適用します...')
            manager.apply(prof)
            print(f'--- [{prof}] 適用後の状態 ---')
            print(manager)
            print('※ 別ターミナルで `ros2 param get ...` で実機値を確認できます。')

        input('\n>>> Enterキーを押すと全パラメータをデフォルトにリセットします...')
        manager.reset()
        print('--- リセット後の状態 ---')
        print(manager)

    except KeyboardInterrupt:
        print('\n中断されました。安全のためパラメータをリセットします...')
        manager.reset()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
