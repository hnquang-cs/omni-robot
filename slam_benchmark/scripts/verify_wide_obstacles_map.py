#!/usr/bin/env python3
"""Verify basic occupancy metrics for the final wide-obstacles Gmapping map."""

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

import evaluate_map_basic  # noqa: E402


PKG_PATH = Path(__file__).resolve().parents[1]
DEFAULT_MAP = PKG_PATH / "results" / "stage9" / "maps" / "gmapping_lidar_wide_obstacles_final.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", nargs="?", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--unknown-warning", type=float, default=0.85)
    parser.add_argument("--free-warning", type=float, default=0.05)
    args = parser.parse_args()

    map_yaml = args.map_yaml
    if not map_yaml.is_file():
        print(f"FAIL: map_yaml_not_found {map_yaml}")
        return 1

    try:
        row = evaluate_map_basic.evaluate_map(map_yaml.resolve())
    except Exception as exc:
        print(f"FAIL: evaluate_map_error {exc}")
        return 1

    keys = [
        "width",
        "height",
        "resolution",
        "occupied_ratio",
        "free_ratio",
        "unknown_ratio",
    ]
    for key in keys:
        print(f"{key}={row.get(key, 'N/A')}")

    occupied = float(row.get("occupied_ratio", "0") or 0)
    free = float(row.get("free_ratio", "0") or 0)
    unknown = float(row.get("unknown_ratio", "1") or 1)

    status = 0
    if occupied <= 0.001:
      print("WARN: occupied_ratio very low; obstacle/wall returns may be insufficient")
      status = 2
    if unknown > args.unknown_warning:
      print(f"WARN: unknown_ratio>{args.unknown_warning:.2f}")
      status = 2
    if free < args.free_warning:
      print(f"WARN: free_ratio<{args.free_warning:.2f}")
      status = 2

    print("PASS" if status == 0 else "PASS_WITH_WARNINGS")
    return status


if __name__ == "__main__":
    sys.exit(main())
