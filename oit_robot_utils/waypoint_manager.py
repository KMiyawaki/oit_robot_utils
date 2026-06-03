#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
from dataclasses import dataclass
from typing import Iterable, List, Optional


@dataclass
class WayPoint:
    id: int
    command: str
    x: float
    y: float
    threshold: float

    def __str__(self) -> str:
        return (
            f'WayPoint(id={self.id}, command={self.command}, '
            f'x={self.x:.3f}, y={self.y:.3f}, threshold={self.threshold:.3f})'
        )


class WayPointManager:
    def __init__(self, waypoints: Optional[Iterable[WayPoint]] = None):
        self._waypoints: List[WayPoint] = list(waypoints or [])
        self._by_id = {wp.id: wp for wp in self._waypoints}

    def __len__(self) -> int:
        return len(self._waypoints)

    def __iter__(self):
        return iter(self._waypoints)

    def __getitem__(self, index):
        return self._waypoints[index]

    def __str__(self) -> str:
        return f'WayPointManager({len(self._waypoints)} waypoints)'

    def get_by_id(self, waypoint_id: int) -> Optional[WayPoint]:
        return self._by_id.get(waypoint_id)

    def nearest(self, x: float, y: float, n: int = 1) -> List[WayPoint]:
        if n <= 0:
            return []
        distances = [((wp.x - x) ** 2 + (wp.y - y) ** 2, wp)
                     for wp in self._waypoints]
        distances.sort(key=lambda item: item[0])
        return [wp for _, wp in distances[:min(n, len(distances))]]

    @classmethod
    def load_csv(cls, path: str) -> 'WayPointManager':
        waypoints: List[WayPoint] = []
        with open(path, newline='', encoding='utf-8') as infile:
            reader = csv.reader(cls._skip_comments(infile))
            for row_number, row in enumerate(reader, start=1):
                if not row:
                    continue
                if len(row) < 5:
                    raise ValueError(
                        f'Invalid row {row_number} in CSV: expected 5 columns, got {len(row)}'
                    )
                waypoint_id = int(row[0].strip())
                command = row[1].strip()
                x = float(row[2].strip())
                y = float(row[3].strip())
                threshold = float(row[4].strip())
                if waypoint_id in [wp.id for wp in waypoints]:
                    raise ValueError(
                        f'Duplicate waypoint ID {waypoint_id} in CSV')
                waypoints.append(
                    WayPoint(waypoint_id, command, x, y, threshold))
        return cls(waypoints)

    @staticmethod
    def _skip_comments(lines):
        for line in lines:
            stripped = line.lstrip()
            if not stripped or stripped.startswith('#'):
                continue
            yield line
