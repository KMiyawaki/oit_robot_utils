#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
import sys
import tempfile

import numpy as np
from rdp import rdp

from oit_robot_utils.waypoint_manager import WayPoint, WayPointManager


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
        '--epsilon',
        type=float,
        default=0.1,
        help='RDP epsilon in meters. (default: 0.1)'
    )
    parser.add_argument(
        '--min-distance',
        type=float,
        default=1.0,
        help='Minimum distance in meters to keep points on straight lines. (default: 1.0)'
    )
    return parser.parse_args()


def main():
    """
    Ramer-Douglas-Peucker (RDP) アルゴリズムを使用してWayPoint CSVをフィルタリングし、
    直線上の冗長なウェイポイントをコメントアウトするスクリプト。
    RDPで軌跡の形状を維持しつつ、直線部分でもウェイポイントが最小距離以上離れないようにします。

    usage:
      ros2 run oit_robot_utils filter_waypoints_csv.py [-h] [-o OUTPUT] [--overwrite] [--epsilon EPSILON] [--min-distance MIN_DISTANCE] input_csv

    引数:
      input_csv                   入力するWaypoint CSVファイルのパス

    オプション引数:
      -h, --help                  ヘルプメッセージを表示して終了します
      -o OUTPUT, --output OUTPUT  出力CSVファイルのパス。指定しない場合、入力ファイル名に"_filtered"を付けた新しいファイルが作成されます。
      --overwrite                 入力ファイルを上書きします。-oが指定されている場合は無視されます。
      --epsilon EPSILON           RDPアルゴリズムの許容誤差(メートル単位)。(デフォルト: 0.1)
      --min-distance MIN_DISTANCE
                                  直線上で点を維持する最小距離(m)。(デフォルト: 1.0)

    実行例:
    1. 新しいファイルに保存 (デフォルト):
       (例: input.csv -> input_filtered.csv)
       $ ros2 run oit_robot_utils filter_waypoints_csv.py path/to/input.csv

    2. 元のファイルを上書き:
       $ ros2 run oit_robot_utils filter_waypoints_csv.py path/to/input.csv --overwrite

    3. パラメータを指定:
       $ ros2 run oit_robot_utils filter_waypoints_csv.py input.csv --epsilon 0.2 --min-distance 2.0
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

    if len(active_waypoints) < 2:
        print("Not enough waypoints to filter (requires at least 2). No changes made.")
        return 0

    # --- RDPと距離のハイブリッドアプローチでフィルタリング ---

    # 1. RDPアルゴリズムで、軌跡の形状を維持するための主要な点を特定
    points = np.array([[wp.x, wp.y] for wp in active_waypoints])
    rdp_mask = rdp(points, epsilon=args.epsilon, return_mask=True)
    rdp_indices = set(np.where(rdp_mask)[0])

    # 2. 直線上で点が離れすぎないように、一定間隔で点を追加
    kept_ids = set()
    if active_waypoints:
        # 最初の点は必ず保持
        last_kept_wp = active_waypoints[0]
        kept_ids.add(last_kept_wp.id)

        for i in range(1, len(active_waypoints)):
            current_wp = active_waypoints[i]

            # RDPで特定された主要点かどうか
            is_rdp_point = i in rdp_indices

            # 最後に保持した点からの距離
            distance = math.hypot(current_wp.x - last_kept_wp.x,
                                  current_wp.y - last_kept_wp.y)

            # RDPの主要点であるか、または最小距離を超えていれば、この点を保持
            if is_rdp_point or distance >= args.min_distance:
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
