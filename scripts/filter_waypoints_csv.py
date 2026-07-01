#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
import sys
import tempfile

from oit_robot_utils.waypoint_manager import WayPoint, WayPointManager


def calculate_angle(p1: WayPoint, p2: WayPoint, p3: WayPoint) -> float:
    """3つのウェイポイントがなす角度を計算する (p2が角)。0-180度の範囲で返す。"""
    # ベクトル p2->p1 と p2->p3
    v1 = (p1.x - p2.x, p1.y - p2.y)
    v2 = (p3.x - p2.x, p3.y - p2.y)

    dot_product = v1[0] * v2[0] + v1[1] * v2[1]
    mag1 = math.hypot(v1[0], v1[1])
    mag2 = math.hypot(v2[0], v2[1])

    if mag1 == 0 or mag2 == 0:
        # 点が重なっている場合などは直線とみなし、角度0とする
        return 0.0

    # 浮動小数点誤差を考慮して値を[-1, 1]の範囲にクランプ
    cos_theta = max(-1.0, min(1.0, dot_product / (mag1 * mag2)))

    # acosの結果はラジアン [0, pi]
    angle_rad = math.acos(cos_theta)

    return math.degrees(angle_rad)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Filter waypoints in a CSV file by commenting out redundant points on straight lines.'
    )
    parser.add_argument(
        'input_csv', help='Path to the input waypoint CSV file')
    parser.add_argument(
        '-o', '--output',
        help='Path to the output CSV file. If not specified, a new file with suffix "_filtered" is created.'
    )
    parser.add_argument(
        '--overwrite',
        action='store_true',
        help='Overwrite the input file. This flag is ignored if -o is specified.'
    )
    parser.add_argument(
        '--angle-threshold',
        type=float,
        default=20.0,
        help='Angle threshold in degrees. Points forming an angle < this are considered straight. (default: 20.0)'
    )
    parser.add_argument(
        '--min-distance',
        type=float,
        default=1.0,
        help='Minimum distance in meters. Points are kept if distance from last kept point >= this. (default: 1.0)'
    )
    return parser.parse_args()


def main():
    """
    WayPoint CSVをフィルタリングし、不要なポイントをコメントアウトするスクリプト。

    直線上の冗長なウェイポイントを、角度のしきい値と最小距離に基づいて間引きます。
    カーブ部分は細かくウェイポイントを残し、直線部分は間隔を空けて残します。

    usage:
      ros2 run oit_robot_utils filter_waypoints_csv.py [-h] [-o OUTPUT] [--overwrite] [--angle-threshold ANGLE_THRESHOLD] [--min-distance MIN_DISTANCE] input_csv

    引数:
      input_csv                   入力するWaypoint CSVファイルのパス

    オプション引数:
      -h, --help                  ヘルプメッセージを表示して終了します
      -o OUTPUT, --output OUTPUT  出力CSVファイルのパス。指定しない場合、入力ファイル名に"_filtered"を付けた新しいファイルが作成されます。
      --overwrite                 入力ファイルを上書きします。-oが指定されている場合は無視されます。
      --angle-threshold ANGLE_THRESHOLD
                                  角度のしきい値(deg)。この値より小さい変化を持つ点を直線上の点と見なします。(デフォルト: 5.0)
      --min-distance MIN_DISTANCE
                                  最小距離(m)。直線上でも、最後に保持した点からこの距離以上離れていれば維持します。(デフォルト: 1.0)

    実行例:
    1. 新しいファイルに保存 (デフォルト):
       (例: input.csv -> input_filtered.csv)
       $ ros2 run oit_robot_utils filter_waypoints_csv.py path/to/input.csv

    2. 元のファイルを上書き:
       $ ros2 run oit_robot_utils filter_waypoints_csv.py path/to/input.csv --overwrite

    3. パラメータを指定:
       $ ros2 run oit_robot_utils filter_waypoints_csv.py input.csv --angle-threshold 10 --min-distance 0.5
    """
    args = parse_args()

    # --- 出力パスの決定 ---
    if args.output:
        output_path = args.output
    elif args.overwrite:
        output_path = args.input_csv
    else:
        base, ext = os.path.splitext(args.input_csv)
        output_path = f"{base}_filtered{ext}"

    try:
        manager = WayPointManager.load_csv(args.input_csv)
    except FileNotFoundError:
        print(
            f"Error: Input file not found at '{args.input_csv}'", file=sys.stderr)
        return 1
    except ValueError as e:
        print(
            f"Error parsing CSV file '{args.input_csv}': {e}", file=sys.stderr)
        return 1

    active_waypoints = list(manager)

    if len(active_waypoints) < 3:
        print("Not enough waypoints to filter (requires at least 3). No changes made.")
        return 0

    # 保持するウェイポイントのIDを格納するセット
    # 最初と最後のウェイポイントは常に保持する
    kept_ids = {active_waypoints[0].id, active_waypoints[-1].id}
    last_kept_wp = active_waypoints[0]

    for i in range(1, len(active_waypoints) - 1):
        # 局所的な曲がり具合を評価するため、直前・現在・直後の3点を取得
        prev_wp = active_waypoints[i-1]
        current_wp = active_waypoints[i]
        next_wp = active_waypoints[i+1]

        # 現在の点がなす角の大きさを計算
        angle = 180 - calculate_angle(prev_wp, current_wp, next_wp)

        # 最後に保持した点からの距離を計算
        distance = math.hypot(current_wp.x - last_kept_wp.x,
                              current_wp.y - last_kept_wp.y)

        # 角度がしきい値より大きい（=カーブしている）か、
        # または距離が最小距離より大きい場合は、このウェイポイントを保持する
        if angle > args.angle_threshold or distance >= args.min_distance:
            kept_ids.add(current_wp.id)
            last_kept_wp = current_wp

    # --- ファイルの書き換え処理 ---
    try:
        with open(args.input_csv, 'r', newline='', encoding='utf-8') as infile:
            original_lines = infile.readlines()

        temp_fd, temp_path = tempfile.mkstemp(
            dir=os.path.dirname(os.path.abspath(output_path)))

        num_commented = 0
        with os.fdopen(temp_fd, 'w', newline='', encoding='utf-8') as outfile:
            for line in original_lines:
                stripped_line = line.lstrip()
                # 空行または既にコメントアウトされている行はそのまま出力
                if not stripped_line or stripped_line.startswith('#'):
                    outfile.write(line)
                    continue

                try:
                    row = next(csv.reader([line.strip()]))
                    if not row:
                        outfile.write(line)
                        continue

                    waypoint_id = int(row[0].strip())

                    if waypoint_id in kept_ids:
                        outfile.write(line)
                    else:
                        outfile.write(f'#{line}')
                        num_commented += 1
                except (ValueError, IndexError):
                    outfile.write(line)

        os.replace(temp_path, output_path)

        print(
            f"Filtering complete. {num_commented} waypoints were commented out.")
        print(f"Result saved to: {output_path}")

    except Exception as e:
        print(
            f"An error occurred during file processing: {e}", file=sys.stderr)
        if 'temp_path' in locals() and os.path.exists(temp_path):
            os.remove(temp_path)
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
