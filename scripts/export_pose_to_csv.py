#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import csv
import datetime
import os
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


DEFAULT_THRESHOLD = 1.5
DEFAULT_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'csv')


class PoseCsvExporter(Node):
    def __init__(self, parent_frame: str = 'map', child_frame: str = 'base_link'):
        super().__init__('export_pose_to_csv')
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.buffer = Buffer()
        self.tl = TransformListener(self.buffer, self)

    def lookup_pose(self):
        try:
            now = Time()
            trans = self.buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                now,
            )
            ts = trans.header.stamp
            secs = ts.sec + ts.nanosec * 1e-9
            return trans.transform.translation.x, trans.transform.translation.y, secs
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {e}')
            return None

    def wait_for_pose(self, timeout_sec: float = 10.0):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            pose = self.lookup_pose()
            if pose is not None:
                return pose
            rclpy.spin_once(self, timeout_sec=0.5)
        return None

    def sleep_with_spin(self, duration_sec: float):
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.5, remaining))


def make_filename(output_path: Optional[str] = None) -> str:
    if output_path:
        output_dir = os.path.dirname(os.path.abspath(output_path))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        return output_path
    os.makedirs(DEFAULT_OUTPUT_DIR, exist_ok=True)
    now = datetime.datetime.now()
    return os.path.join(
        DEFAULT_OUTPUT_DIR,
        now.strftime('%Y%m%d_%H%M%S.%f')[:-3] + '.csv'
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Export current robot pose to a waypoint CSV file.'
    )
    parser.add_argument(
        '--output', '-o',
        help='Output CSV filename. Default is timestamped YYYYMMDD_HHMMSS.SSS.csv in oit_robot_utils/csv',
        default=None,
    )
    parser.add_argument(
        '--count', '-c',
        type=int,
        default=1,
        help='How many pose rows to record into the CSV.',
    )
    parser.add_argument(
        '--interval', '-i',
        type=float,
        default=1.0,
        help='Interval in seconds between multiple pose captures.',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=10.0,
        help='Timeout in seconds to wait for the first TF transform.',
    )
    parser.add_argument(
        '--append',
        action='store_true',
        help='Append to existing CSV file instead of creating a new one.',
    )
    parser.add_argument(
        '--infinite',
        action='store_true',
        help='Continue capturing poses indefinitely until Ctrl+C.',
    )
    parser.add_argument(
        '--parent-frame',
        default='map',
        help='Parent frame for TF lookup. Default is map.',
    )
    parser.add_argument(
        '--child-frame',
        default='base_link',
        help='Child frame for TF lookup. Default is base_link.',
    )
    return parser.parse_args()


def main(args=None):
    args = parse_args() if args is None else args
    filename = make_filename(args.output)
    
    # Appendモードなら既存ファイルの最後のIDを読む
    next_id = 1
    write_mode = 'w'
    if args.append and os.path.exists(filename):
        write_mode = 'a'
        # 既存ファイルの最後のIDを読み取る
        try:
            with open(filename, 'r', encoding='utf-8') as f:
                lines = [line.strip() for line in f if line.strip() and not line.strip().startswith('#')]
                if lines:
                    last_id = int(lines[-1].split(',')[0])
                    next_id = last_id + 1
        except Exception as e:
            print(f'Warning: could not read last ID from {filename}: {e}', file=sys.stderr)
    elif os.path.exists(filename) and not args.append:
        print(f'Error: output file already exists: {filename}', file=sys.stderr)
        return 1

    rclpy.init()
    node = PoseCsvExporter(parent_frame=args.parent_frame, child_frame=args.child_frame)
    try:
        pose = node.wait_for_pose(timeout_sec=args.timeout)
        if pose is None:
            print(
                f'Error: failed to receive TF transform {args.parent_frame} -> {args.child_frame} within timeout.',
                file=sys.stderr,
            )
            return 2

        with open(filename, write_mode, newline='', encoding='utf-8') as csvfile:
            writer = csv.writer(csvfile)
            # ヘッダは新規ファイルのときだけ書く
            if write_mode == 'w':
                writer.writerow(['#ID', 'コマンド', 'x', 'y', '閾値'])
            
            # 無限ループモード
            if args.infinite:
                row_id = next_id
                print(f'Infinite capture mode started. Press Ctrl+C to stop.')
                # 最初の位置で1行書く
                x, y, ts = pose
                writer.writerow([
                    row_id,
                    'goto_point',
                    f'{x:.6f}',
                    f'{y:.6f}',
                    f'{DEFAULT_THRESHOLD:.6f}',
                ])
                csvfile.flush()
                print(f'Waypoint {row_id}: x={x:.6f}, y={y:.6f} (ts={ts:.6f})')
                row_id += 1
                
                # その後のループで定期取得
                while rclpy.ok():
                    try:
                        node.sleep_with_spin(args.interval)
                        new_pose = node.wait_for_pose(timeout_sec=args.timeout)
                        if new_pose is None:
                            print('Warning: lost TF transform, retrying...', file=sys.stderr)
                            continue
                        x, y, ts = new_pose
                        writer.writerow([
                            row_id,
                            'goto_point',
                            f'{x:.6f}',
                            f'{y:.6f}',
                            f'{DEFAULT_THRESHOLD:.6f}',
                        ])
                        csvfile.flush()
                        print(f'Waypoint {row_id}: x={x:.6f}, y={y:.6f} (ts={ts:.6f})')
                        row_id += 1
                    except KeyboardInterrupt:
                        break
            else:
                # 通常モード（--count指定）
                for row_id in range(next_id, next_id + args.count):
                    if row_id > next_id:
                        node.sleep_with_spin(args.interval)
                    pose = node.wait_for_pose(timeout_sec=args.timeout)
                    if pose is None:
                        print('Error: lost TF transform while capturing rows.', file=sys.stderr)
                        return 3
                    x, y, ts = pose
                    writer.writerow([
                        row_id,
                        'goto_point',
                        f'{x:.6f}',
                        f'{y:.6f}',
                        f'{DEFAULT_THRESHOLD:.6f}',
                    ])
                    print(f'Wrote waypoint {row_id}: x={x:.6f}, y={y:.6f}, threshold={DEFAULT_THRESHOLD:.2f} (ts={ts:.6f})')

        print(f'Created/updated CSV: {filename}')
        return 0
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
