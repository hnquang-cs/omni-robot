#!/usr/bin/env python3
"""Evaluate dynamic map coverage without hardcoded ROI."""

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from evaluate_map_basic import parse_simple_yaml, load_image
from map_coverage_tracker import DynamicCoverageTracker, CoverageMetrics, format_metric


FIELDNAMES = [
    "map_yaml",
    "width",
    "height",
    "resolution",
    "global_total_cells",
    "global_known_cells",
    "global_unknown_cells",
    "global_explored_ratio",
    "global_unknown_ratio",
    "active_total_cells",
    "active_known_cells",
    "active_unknown_cells",
    "active_explored_ratio",
    "active_unknown_ratio",
    "explored_area_m2",
    "active_bbox_area_m2",
    "active_known_area_m2",
    "active_unknown_area_m2",
]


def image_pixels_to_occupancy(
    pixels: Iterable[int],
    occupied_thresh: float,
    free_thresh: float,
    negate: int,
) -> List[int]:
    data: List[int] = []
    for value in pixels:
        occ = value / 255.0 if negate else (255.0 - value) / 255.0
        if occ > occupied_thresh:
            data.append(100)
        elif occ < free_thresh:
            data.append(0)
        else:
            data.append(-1)
    return data


def metrics_to_row(metrics: CoverageMetrics, map_yaml: str = "/map") -> Dict[str, str]:
    return {
        "map_yaml": map_yaml,
        "width": str(metrics.width),
        "height": str(metrics.height),
        "resolution": format_metric(metrics.resolution),
        "global_total_cells": str(metrics.global_total_cells),
        "global_known_cells": str(metrics.global_known_cells),
        "global_unknown_cells": str(metrics.global_unknown_cells),
        "global_explored_ratio": format_metric(metrics.global_explored_ratio),
        "global_unknown_ratio": format_metric(metrics.global_unknown_ratio),
        "active_total_cells": "N/A" if metrics.active_total_cells is None else str(metrics.active_total_cells),
        "active_known_cells": "N/A" if metrics.active_known_cells is None else str(metrics.active_known_cells),
        "active_unknown_cells": "N/A" if metrics.active_unknown_cells is None else str(metrics.active_unknown_cells),
        "active_explored_ratio": format_metric(metrics.active_explored_ratio),
        "active_unknown_ratio": format_metric(metrics.active_unknown_ratio),
        "explored_area_m2": format_metric(metrics.explored_area_m2),
        "active_bbox_area_m2": format_metric(metrics.active_bbox_area_m2),
        "active_known_area_m2": format_metric(metrics.active_known_area_m2),
        "active_unknown_area_m2": format_metric(metrics.active_unknown_area_m2),
    }


def evaluate_map_yaml(map_yaml: Path) -> CoverageMetrics:
    yaml_data = parse_simple_yaml(map_yaml)
    if "image" not in yaml_data:
        raise ValueError(f"Missing 'image' entry in {map_yaml}")
    image_path = Path(yaml_data["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    width, height, pixels = load_image(image_path)
    resolution = float(yaml_data.get("resolution", "0.0"))
    occupied_thresh = float(yaml_data.get("occupied_thresh", "0.65"))
    free_thresh = float(yaml_data.get("free_thresh", "0.196"))
    negate = int(float(yaml_data.get("negate", "0")))
    occupancy = image_pixels_to_occupancy(pixels, occupied_thresh, free_thresh, negate)
    tracker = DynamicCoverageTracker(active_bbox_margin_m=1.0, progress_window_sec=30.0)
    return tracker.update_from_values(
        width=width,
        height=height,
        resolution=resolution,
        data=occupancy,
        timestamp=time.monotonic(),
    )


def evaluate_live_map() -> CoverageMetrics:
    import rospy
    from nav_msgs.msg import OccupancyGrid

    rospy.init_node("evaluate_map_coverage", anonymous=True, disable_signals=True)
    msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=15.0)
    tracker = DynamicCoverageTracker(active_bbox_margin_m=1.0, progress_window_sec=30.0)
    return tracker.update_from_occupancy_grid(msg, timestamp=time.monotonic())


def default_output(map_yaml: Optional[Path]) -> Path:
    if map_yaml is not None:
        return map_yaml.with_name(f"{map_yaml.stem}_coverage_metrics.csv")
    package_path = Path(__file__).resolve().parents[1]
    out_dir = package_path / "results" / "stage9" / "maps"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir / "live_map_coverage_metrics.csv"


def print_metrics(row: Dict[str, str]) -> None:
    print("global_explored_ratio may be low if gmapping bounds are large; active metrics are preferred.")
    for key in FIELDNAMES:
        print(f"{key}={row[key]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", nargs="?", type=Path, default=None)
    parser.add_argument("-o", "--output", type=Path, default=None)
    args = parser.parse_args()

    try:
        if args.map_yaml is not None:
            metrics = evaluate_map_yaml(args.map_yaml.resolve())
            map_label = str(args.map_yaml)
        else:
            metrics = evaluate_live_map()
            map_label = "/map"
    except Exception as exc:
        print(f"FAIL: evaluate_map_coverage failed: {exc}", file=sys.stderr)
        return 1

    row = metrics_to_row(metrics, map_label)
    output = (args.output or default_output(args.map_yaml)).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(row)

    print_metrics(row)
    print(f"PASS: wrote {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
