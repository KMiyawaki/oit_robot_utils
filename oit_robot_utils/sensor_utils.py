#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf_transformations import compose_matrix, euler_from_quaternion

# =============================================================================
# 1. 4x4 同次変換行列の生成・適用（純粋幾何演算）
# =============================================================================


def make_tf_matrix(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    roll: float = 0.0,
    pitch: float = 0.0,
    yaw: float = 0.0,
) -> np.ndarray:
    """(x, y, z, roll, pitch, yaw) から 4x4 同次変換行列 (np.ndarray) を生成する。"""
    return compose_matrix(translate=[x, y, z], angles=[roll, pitch, yaw])


def pose_to_tf_matrix(pose_msg: Pose | PoseWithCovarianceStamped) -> np.ndarray:
    """Pose または PoseWithCovarianceStamped (amcl_pose) から 4x4 同次変換行列を生成する。"""
    p = pose_msg.pose.pose if isinstance(
        pose_msg, PoseWithCovarianceStamped) else pose_msg
    roll, pitch, yaw = euler_from_quaternion([
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w,
    ])
    return make_tf_matrix(p.position.x, p.position.y, p.position.z, roll, pitch, yaw)


def apply_tf(points: np.ndarray, tf_matrix: np.ndarray) -> np.ndarray:
    """点群 (N, 2) / (N, 3) または単点 (2,) / (3,) に 4x4 同次変換行列を適用する。"""
    pts = np.asarray(points, dtype=np.float64)
    is_1d = pts.ndim == 1
    if is_1d:
        pts = pts.reshape(1, -1)
    if pts.size == 0:
        return pts.copy()

    dim = pts.shape[1]
    pts3 = np.column_stack((pts, np.zeros(pts.shape[0]))) if dim == 2 else pts
    out = (pts3 @ tf_matrix[:3, :3].T + tf_matrix[:3, 3])[:, :dim]
    return out[0] if is_1d else out


# =============================================================================
# 2. ROS メッセージ <-> NumPy 点群変換
# =============================================================================

def scan_to_points(scan_msg: LaserScan | None, min_range: float | None = None) -> np.ndarray:
    """LaserScan の有効ビームをセンサ座標系の 2D 点群 (N, 2) に一括変換する。

    Args:
        scan_msg: 入力の LaserScan メッセージ。None または空の場合は (0, 2) の空配列を返す。
        min_range: 最小有効距離の追加下限 [m]。指定時は scan_msg.range_min との大きい方を採用。
    """
    if scan_msg is None or not scan_msg.ranges:
        return np.empty((0, 2), dtype=np.float64)

    ranges = np.asarray(scan_msg.ranges, dtype=np.float64)
    r_min = max(scan_msg.range_min,
                min_range) if min_range is not None else scan_msg.range_min

    valid = np.isfinite(ranges) & (ranges >= r_min) & (
        ranges <= scan_msg.range_max)
    if not np.any(valid):
        return np.empty((0, 2), dtype=np.float64)

    angles = scan_msg.angle_min + \
        np.arange(ranges.size, dtype=np.float64) * scan_msg.angle_increment
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def points_to_pointcloud2(
    points: np.ndarray,
    frame_id: str = 'map',
    stamp: Time | None = None,
    z_fallback: float = 0.0,
) -> PointCloud2:
    """NumPy 点群 (N, 2) または (N, 3) から sensor_msgs/PointCloud2 を生成する。

    Args:
        points: 入力点群 (N, 2) / (N, 3) または単点 (2,) / (3,)。
        frame_id: PointCloud2 のフレームID（デフォルト: 'map'）。
        stamp: タイムスタンプ（scan_msg.header.stamp 等）。None の場合はデフォルト Time()。
        z_fallback: 入力が 2D 点群 (N, 2) の場合に補完する Z 座標 [m]。
    """
    pts = np.asarray(points, dtype=np.float32)
    if pts.size == 0:
        pts3 = np.empty((0, 3), dtype=np.float32)
    else:
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)
        if pts.shape[1] == 2:
            z_col = np.full((pts.shape[0], 1), z_fallback, dtype=np.float32)
            pts3 = np.hstack((pts, z_col))
        else:
            pts3 = pts[:, :3]

    header = Header(frame_id=frame_id,
                    stamp=stamp if stamp is not None else Time())
    return point_cloud2.create_cloud_xyz32(header, pts3)


# =============================================================================
# 3. センサオフセット管理クラス
# =============================================================================

@dataclass(frozen=True)
class SensorOffset:
    """YAMLパラメータから読み込んだセンサの 4x4 静的変換行列を保持するクラス。"""
    topic: str
    matrix: np.ndarray  # shape: (4, 4)

    @classmethod
    def load_all(cls, node: Node) -> dict[str, SensorOffset]:
        """ROS 2 パラメータ (topics.names / topics.<name>.*) から全センサの 4x4 行列を読み込む。"""
        if not node.has_parameter('topics.names'):
            node.declare_parameter('topics.names', ['scan_low'])
        names_val = node.get_parameter('topics.names').value
        names = list(names_val) if isinstance(names_val, (list, tuple)) else []

        offsets: dict[str, SensorOffset] = {}
        for name in names:
            if not isinstance(name, str) or not name:
                continue
            vals: dict[str, float] = {}
            for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
                param_name = f'topics.{name}.{key}'
                if not node.has_parameter(param_name):
                    node.declare_parameter(param_name, 0.0)
                raw_val = node.get_parameter(param_name).value
                vals[key] = float(raw_val) if isinstance(
                    raw_val, (int, float, str)) else 0.0
            offsets[name] = cls(topic=name, matrix=make_tf_matrix(**vals))
        return offsets

    def to_robot(self, source: np.ndarray | LaserScan | None) -> np.ndarray:
        """センサ座標系の点群または LaserScan をロボット座標系 (base_link) へ変換する。"""
        pts = source if isinstance(
            source, np.ndarray) else scan_to_points(source)
        return apply_tf(pts, self.matrix)

    def transform_to(
        self,
        source: np.ndarray | LaserScan | None,
        parent_matrix: np.ndarray,
    ) -> np.ndarray:
        """別の 4x4 同次変換行列 (map<-base_link 等) を与えて一気にその座標系へ変換する。"""
        pts = source if isinstance(
            source, np.ndarray) else scan_to_points(source)
        return apply_tf(pts, parent_matrix @ self.matrix)


def project_scan_to_pointcloud2(
    scan_msg: LaserScan | None,
    pose_msg: Pose | PoseWithCovarianceStamped | None,
    offset: SensorOffset | None,
    frame_id: str = 'map',
    min_range: float | None = None,
) -> PointCloud2 | None:
    """LaserScan と自己位置 (amcl_pose 等) を SensorOffset で投影し、PointCloud2 を生成する。

    Args:
        scan_msg: 入力の LaserScan メッセージ。None または範囲データ空の場合は None を返す。
        pose_msg: 自己位置メッセージ (Pose / PoseWithCovarianceStamped)。None の場合は None を返す。
        offset: センサオフセット情報 (SensorOffset)。None の場合は None を返す。
        frame_id: 出力 PointCloud2 のフレームID（デフォルト: 'map'）。
        min_range: 最小有効距離の追加下限 [m]。

    Returns:
        投影済みの PointCloud2 メッセージ。入力不足や異常値の場合は None。
    """
    if scan_msg is None or not scan_msg.ranges or pose_msg is None or offset is None:
        return None

    sensor_pts = scan_to_points(scan_msg, min_range=min_range)
    if sensor_pts.size == 0:
        return None

    try:
        map_T_robot = pose_to_tf_matrix(pose_msg)
    except Exception:
        return None

    if not np.all(np.isfinite(map_T_robot)):
        return None

    map_pts = offset.transform_to(sensor_pts, map_T_robot)
    return points_to_pointcloud2(
        map_pts,
        frame_id=frame_id,
        stamp=scan_msg.header.stamp,
        z_fallback=float(offset.matrix[2, 3]),
    )

# =============================================================================
# 4. 既存の簡易障害物距離判定（後方互換・軽量判定用）
# =============================================================================


def _get_kth_nearest(distances: list[float], min_points: int) -> float:
    """単発ノイズ除去のため min_points 番目に近い距離を返す（点数不足なら inf）。"""
    k = max(1, min_points)
    if len(distances) < k:
        return float('inf')
    distances.sort()
    return distances[k - 1]


def get_obstacle_dist_in_rect(
    scan_msg: LaserScan | None,
    width: float = 0.70,
    min_x: float = 0.20,
    min_points: int = 2,
) -> float:
    """前方矩形領域（|y| <= width / 2, x >= min_x）にある最も近い障害物の前方距離 x [m] を返す。

    Note:
        距離は scan_msg のフレーム基準（`hesai_lidar` 基準の場合は `base_link` より後方 0.12m）。
        実機全幅 0.44m に対し width=0.70m は左右約 0.13m ずつのマージンに相当する。
    """
    if scan_msg is None or not scan_msg.ranges:
        return float('inf')

    distances: list[float] = []
    half_width = abs(width / 2.0)
    angle_min = scan_msg.angle_min
    angle_inc = scan_msg.angle_increment

    for i, r in enumerate(scan_msg.ranges):
        if math.isfinite(r) and scan_msg.range_min <= r <= scan_msg.range_max:
            angle = angle_min + i * angle_inc
            x = r * math.cos(angle)
            if x >= min_x:
                y = r * math.sin(angle)
                if abs(y) <= half_width:
                    distances.append(x)

    return _get_kth_nearest(distances, min_points)


def get_obstacle_dist_in_sector(
    scan_msg: LaserScan | None,
    angle_rad: float = math.radians(30.0),
    min_range: float = 0.12,
    min_points: int = 2,
) -> float:
    """前方扇形領域（|angle| <= angle_rad / 2, r >= min_range）にある最も近い障害物の直線距離 r [m] を返す。"""
    if scan_msg is None or not scan_msg.ranges:
        return float('inf')

    half_rad = abs(angle_rad / 2.0)
    distances: list[float] = []
    angle_min = scan_msg.angle_min
    angle_inc = scan_msg.angle_increment

    for i, r in enumerate(scan_msg.ranges):
        if math.isfinite(r) and scan_msg.range_min <= r <= scan_msg.range_max and r >= min_range:
            raw_angle = angle_min + i * angle_inc
            # [0, 2π] 表現のスキャナでも正面左右を正しく判定できるよう [-π, π] に正規化
            norm_angle = ((raw_angle + math.pi) % (2.0 * math.pi)) - math.pi
            if -half_rad <= norm_angle <= half_rad:
                distances.append(r)

    return _get_kth_nearest(distances, min_points)
