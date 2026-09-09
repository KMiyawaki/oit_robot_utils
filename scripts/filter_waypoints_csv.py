#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import math
import os
import sys
import tempfile
import datetime
import shutil

import numpy as np
from rdp import rdp

from oit_robot_utils.waypoint_manager import WayPoint, WayPointManager


def parse_args():
    parser = argparse.ArgumentParser(
        description='Filter waypoints in a CSV file by removing redundant points on straight lines.'
    )
    parser.add_argument(
        'input_csv', help='Path to the input waypoint CSV file')
    parser.add_argument(
        '-o', '--output',
        help='Path to the output CSV file. If not specified, the input file is overwritten and a backup is created.'
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
    parser.add_argument(
        '--no-plot',
        action='store_true',
        help='Disable plotting of the original and filtered waypoints.'
    )
    
    return parser.parse_args()


def main():
    args = parse_args()
    input_path = os.path.abspath(args.input_csv)

    # --- 出力パスの決定とバックアップの作成 ---
    if not args.output:
        # デフォルト動作: 上書き ＋ バックアップ
        output_path = input_path
        timestamp = input_path.split('/')[-2].replace('_', '')
        backup_path = f"{input_path}.{timestamp}"
        
        try:
            shutil.copy2(input_path, backup_path)
            print(f"Backup created at: {backup_path}")
        except IOError as e:
            print(f"Error creating backup file: {e}", file=sys.stderr)
            return 1
    else:
        # 出力先が指定された場合はバックアップを取らずにそこへ出力
        output_path = os.path.abspath(args.output)

    try:
        manager = WayPointManager.load_csv(input_path)
    except FileNotFoundError:
        print(
            f"Error: Input file not found at '{input_path}'", file=sys.stderr)
        return 1
    except ValueError as e:
        print(
            f"Error parsing CSV file '{input_path}': {e}", file=sys.stderr)
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

    if not args.no_plot:
        try:
            import matplotlib.pyplot as plt
            
            # 元の軌跡データ
            orig_x = [wp.x for wp in active_waypoints]
            orig_y = [wp.y for wp in active_waypoints]
            
            # フィルタリング後の軌跡データ
            filtered_x = [wp.x for wp in active_waypoints if wp.id in kept_ids]
            filtered_y = [wp.y for wp in active_waypoints if wp.id in kept_ids]
            
            plt.figure(figsize=(10, 8))
            
            # 元の点群（グレーで薄く表示）
            plt.plot(orig_x, orig_y, label='Original Path', color='gray', linestyle='--', alpha=0.6)
            plt.scatter(orig_x, orig_y, label='Original Points', color='blue', s=15, alpha=0.3)
            
            # フィルタリング後の点群（赤で強調して表示）
            plt.plot(filtered_x, filtered_y, label='Filtered Path', color='red', linewidth=1.5, alpha=0.8)
            plt.scatter(filtered_x, filtered_y, label='Kept Points', color='red', s=35, zorder=5)
            
            plt.title(f'Waypoint Filtering\n(epsilon={args.epsilon}m, min_dist={args.min_distance}m)')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.legend()
            plt.axis('equal') # アスペクト比を1:1に固定して形状を正確に表示
            plt.grid(True)
            
            plot_output_path = os.path.splitext(output_path)[0] + '.png'
            plt.savefig(plot_output_path) # 画像として保存
            plt.close() # メモリ解放
            print(f"Plot image saved to: {plot_output_path}")
            
        except ImportError:
            print("Warning: 'matplotlib' is not installed. Skipping visualization.", file=sys.stderr)

    # --- ファイルの書き換え処理 ---
    temp_path = None

    try:
        with open(input_path, 'r', newline='', encoding='utf-8') as infile:
            original_lines = infile.readlines()

        temp_fd, temp_path = tempfile.mkstemp(
            dir=os.path.dirname(output_path)
        )

        num_removed = 0
        with os.fdopen(temp_fd, 'w', newline='', encoding='utf-8') as outfile:
            for line in original_lines:
                stripped_line = line.lstrip()
                
                # 空行や元のファイルに存在したコメント行はそのまま出力
                if not stripped_line or stripped_line.startswith('#'):
                    outfile.write(line)
                    continue

                try:
                    row = next(csv.reader([line.strip()]))
                    if not row:
                        outfile.write(line)
                        continue

                    # IDは文字列として取得
                    waypoint_id = row[0].strip()

                    if waypoint_id in kept_ids:
                        outfile.write(line)
                    else:
                        # 条件に合わないウェイポイントは削除（ファイルに書き込まない）
                        num_removed += 1
                except (ValueError, IndexError):
                    outfile.write(line)

        os.replace(temp_path, output_path)
        temp_path = None  # 移動済みなので削除不要

        print(f"Filtering complete. {num_removed} waypoints were removed.")
        print(f"Result saved to: {output_path}")

    except Exception as e:
        print(f"An error occurred during file processing: {e}", file=sys.stderr)

        if temp_path is not None and os.path.exists(temp_path):
            os.remove(temp_path)

        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())