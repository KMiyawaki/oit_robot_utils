#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math

from sensor_msgs.msg import LaserScan


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
