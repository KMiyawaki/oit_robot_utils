#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import sys

from oit_robot_utils.waypoint_manager import WayPointManager


def parse_args():
    parser = argparse.ArgumentParser(
        description='Load a waypoint CSV and print loaded WayPoint objects.'
    )
    parser.add_argument('csv_file', help='Path to the waypoint CSV file.')
    parser.add_argument(
        '--nearest',
        nargs=3,
        metavar=('X', 'Y', 'N'),
        type=float,
        help='Print the N nearest waypoints to the given X,Y coordinate.',
    )
    return parser.parse_args()


def main(args=None):
    args = parse_args() if args is None else args
    manager = WayPointManager.load_csv(args.csv_file)
    print(manager)
    for wp in manager:
        print(wp)

    if args.nearest:
        x, y, n = args.nearest
        nearest = manager.nearest(x, y, int(n))
        print(f'Nearest {int(n)} waypoints to ({x:.3f}, {y:.3f}):')
        for wp in nearest:
            print(f'  {wp}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
