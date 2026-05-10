#!/usr/bin/env python3
"""Dynamic occupancy-grid coverage metrics without hardcoded world ROI."""

from dataclasses import dataclass
from typing import Deque, Iterable, Optional, Tuple
from collections import deque
import math


@dataclass
class CoverageMetrics:
    timestamp: float
    resolution: float
    width: int
    height: int
    global_total_cells: int
    global_known_cells: int
    global_unknown_cells: int
    global_explored_ratio: Optional[float]
    global_unknown_ratio: Optional[float]
    active_total_cells: Optional[int]
    active_known_cells: Optional[int]
    active_unknown_cells: Optional[int]
    active_explored_ratio: Optional[float]
    active_unknown_ratio: Optional[float]
    active_area_m2: Optional[float]
    active_known_area_m2: Optional[float]
    active_unknown_area_m2: Optional[float]
    explored_area_m2: float
    active_bbox_area_m2: Optional[float]
    explored_area_growth_m2: Optional[float]
    active_bbox_growth_m2: Optional[float]
    explored_area_growth_rate_m2ps: Optional[float]
    bbox: Optional[Tuple[int, int, int, int]]


class DynamicCoverageTracker:
    def __init__(
        self,
        active_bbox_margin_m: float = 1.0,
        progress_window_sec: float = 30.0,
    ) -> None:
        self.active_bbox_margin_m = active_bbox_margin_m
        self.progress_window_sec = progress_window_sec
        self.history: Deque[CoverageMetrics] = deque()
        self.latest: Optional[CoverageMetrics] = None

    def update_from_occupancy_grid(self, msg, timestamp: Optional[float] = None) -> CoverageMetrics:
        stamp = timestamp
        if stamp is None:
            try:
                stamp = msg.header.stamp.to_sec()
            except Exception:
                stamp = 0.0
        return self.update_from_values(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            data=msg.data,
            timestamp=float(stamp),
        )

    def update_from_values(
        self,
        width: int,
        height: int,
        resolution: float,
        data: Iterable[int],
        timestamp: float,
    ) -> CoverageMetrics:
        cells = list(data)
        total = width * height
        if total != len(cells):
            raise ValueError(f"map data length {len(cells)} does not match {width}x{height}")

        known_cells = 0
        min_i = height
        max_i = -1
        min_j = width
        max_j = -1
        for index, value in enumerate(cells):
            if value != -1:
                known_cells += 1
                i = index // width
                j = index - i * width
                if i < min_i:
                    min_i = i
                if i > max_i:
                    max_i = i
                if j < min_j:
                    min_j = j
                if j > max_j:
                    max_j = j

        unknown_cells = total - known_cells
        cell_area = resolution * resolution
        global_explored_ratio = known_cells / float(total) if total else None
        global_unknown_ratio = unknown_cells / float(total) if total else None
        explored_area_m2 = known_cells * cell_area

        active_total = None
        active_known = None
        active_unknown = None
        active_explored_ratio = None
        active_unknown_ratio = None
        active_area_m2 = None
        active_known_area_m2 = None
        active_unknown_area_m2 = None
        active_bbox_area_m2 = None
        bbox = None

        if known_cells > 0 and resolution > 0.0:
            margin_cells = int(math.ceil(self.active_bbox_margin_m / resolution))
            min_i = max(0, min_i - margin_cells)
            max_i = min(height - 1, max_i + margin_cells)
            min_j = max(0, min_j - margin_cells)
            max_j = min(width - 1, max_j + margin_cells)
            bbox = (min_i, max_i, min_j, max_j)

            active_total = (max_i - min_i + 1) * (max_j - min_j + 1)
            active_known = 0
            for i in range(min_i, max_i + 1):
                row_start = i * width
                for j in range(min_j, max_j + 1):
                    if cells[row_start + j] != -1:
                        active_known += 1
            active_unknown = active_total - active_known
            active_explored_ratio = active_known / float(active_total) if active_total else None
            active_unknown_ratio = active_unknown / float(active_total) if active_total else None
            active_area_m2 = active_total * cell_area
            active_known_area_m2 = active_known * cell_area
            active_unknown_area_m2 = active_unknown * cell_area
            active_bbox_area_m2 = active_area_m2

        previous = self._window_baseline(timestamp)
        explored_growth = None
        bbox_growth = None
        explored_rate = None
        if previous is not None:
            elapsed = timestamp - previous.timestamp
            explored_growth = explored_area_m2 - previous.explored_area_m2
            if active_bbox_area_m2 is not None and previous.active_bbox_area_m2 is not None:
                bbox_growth = active_bbox_area_m2 - previous.active_bbox_area_m2
            if elapsed > 0.0:
                explored_rate = explored_growth / elapsed

        metrics = CoverageMetrics(
            timestamp=timestamp,
            resolution=resolution,
            width=width,
            height=height,
            global_total_cells=total,
            global_known_cells=known_cells,
            global_unknown_cells=unknown_cells,
            global_explored_ratio=global_explored_ratio,
            global_unknown_ratio=global_unknown_ratio,
            active_total_cells=active_total,
            active_known_cells=active_known,
            active_unknown_cells=active_unknown,
            active_explored_ratio=active_explored_ratio,
            active_unknown_ratio=active_unknown_ratio,
            active_area_m2=active_area_m2,
            active_known_area_m2=active_known_area_m2,
            active_unknown_area_m2=active_unknown_area_m2,
            explored_area_m2=explored_area_m2,
            active_bbox_area_m2=active_bbox_area_m2,
            explored_area_growth_m2=explored_growth,
            active_bbox_growth_m2=bbox_growth,
            explored_area_growth_rate_m2ps=explored_rate,
            bbox=bbox,
        )
        self.latest = metrics
        self.history.append(metrics)
        self._trim_history(timestamp)
        return metrics

    def _window_baseline(self, timestamp: float) -> Optional[CoverageMetrics]:
        if not self.history:
            return None
        target = timestamp - self.progress_window_sec
        baseline = self.history[0]
        for item in self.history:
            if item.timestamp <= target:
                baseline = item
            else:
                break
        if timestamp - baseline.timestamp < self.progress_window_sec * 0.8:
            return None
        return baseline

    def _trim_history(self, timestamp: float) -> None:
        min_time = timestamp - self.progress_window_sec * 3.0
        while len(self.history) > 1 and self.history[0].timestamp < min_time:
            self.history.popleft()


def format_metric(value: Optional[float], precision: int = 6) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{precision}f}"
