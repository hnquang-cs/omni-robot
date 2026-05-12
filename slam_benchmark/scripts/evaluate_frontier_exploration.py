#!/usr/bin/env python3
"""Stage 11: compute frontier-exploration metrics from a bag and/or saved map.

Inputs (any combination):
  --bag PATH              recorded trial bag (rosbag1)
  --map-yaml PATH         saved /map yaml (and PGM next to it)
  --output PATH           CSV output path
  --duration-hint SEC     fallback duration if bag has no clock data

Metrics emitted (one row, key/value):
  duration_sec
  final_map_global_explored_ratio
  final_map_global_unknown_ratio
  active_explored_ratio
  active_unknown_ratio
  explored_area_m2
  active_bbox_area_m2
  number_of_frontier_goals_sent
  number_of_move_base_success
  number_of_move_base_aborted
  total_distance_traveled
  min_lidar_range_over_trial
  cmd_vel_active_ratio
  final_status

Missing data -> the metric value is "N/A:<reason>". The script never crashes.
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

# Add the slam_benchmark scripts dir to sys.path for map_coverage_tracker import.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

NA = "N/A"


def na(reason: str) -> str:
    return f"N/A:{reason}"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Stage 11 frontier exploration metrics")
    p.add_argument("--bag", type=str, default="", help="recorded trial bag")
    p.add_argument("--map-yaml", type=str, default="", help="saved map yaml path")
    p.add_argument("--output", type=str, required=True, help="output CSV path")
    p.add_argument("--duration-hint", type=float, default=0.0,
                   help="fallback duration in seconds if not derivable from bag")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Map (yaml + pgm) static analysis
# ---------------------------------------------------------------------------

def parse_simple_yaml(path: Path):
    data = {}
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip().strip("'\"")
    return data


def read_pgm(path: Path):
    content = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4 and index < len(content):
        while index < len(content) and content[index:index + 1].isspace():
            index += 1
        if index < len(content) and content[index:index + 1] == b"#":
            while index < len(content) and content[index:index + 1] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(content) and not content[index:index + 1].isspace():
            index += 1
        tokens.append(content[start:index])
    if len(tokens) < 4:
        raise ValueError(f"Invalid PGM header: {path}")
    magic = tokens[0]
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    if max_value <= 0 or max_value > 255:
        raise ValueError("Only 8-bit PGM maps are supported")
    while index < len(content) and content[index:index + 1].isspace():
        index += 1
    if magic == b"P5":
        pixels = list(content[index:index + width * height])
    elif magic == b"P2":
        pixels = [int(v) for v in content[index:].decode("ascii", errors="replace").split()]
    else:
        raise ValueError(f"Unsupported PGM magic {magic!r}")
    if len(pixels) != width * height:
        raise ValueError(f"Pixel count {len(pixels)} != {width}x{height}")
    return width, height, pixels


def map_static_metrics(map_yaml: Path):
    """Return dict with global_explored_ratio, global_unknown_ratio, area_m2,
    width_m, height_m, plus N/A reasons on missing files."""
    out = {
        "final_map_global_explored_ratio": na("no_map_yaml"),
        "final_map_global_unknown_ratio":  na("no_map_yaml"),
        "explored_area_m2":                na("no_map_yaml"),
        "active_bbox_area_m2":             na("no_map_yaml"),
    }
    if not map_yaml or not map_yaml.exists():
        return out
    try:
        meta = parse_simple_yaml(map_yaml)
        image_rel = meta.get("image", "")
        resolution = float(meta.get("resolution", "0.05"))
        negate = int(float(meta.get("negate", "0")))
        occupied_thresh = float(meta.get("occupied_thresh", "0.65"))
        free_thresh = float(meta.get("free_thresh", "0.196"))
        pgm_path = (map_yaml.parent / image_rel).resolve()
        width, height, pixels = read_pgm(pgm_path)
    except Exception as exc:
        for k in out:
            out[k] = na(f"map_parse_error:{exc.__class__.__name__}")
        return out

    # map_server convention: cell_value = (255 - p)/255 if !negate else p/255
    free = 0
    occupied = 0
    unknown = 0
    min_i = height
    max_i = -1
    min_j = width
    max_j = -1
    for idx, p in enumerate(pixels):
        if negate:
            occ = p / 255.0
        else:
            occ = (255 - p) / 255.0
        if occ > occupied_thresh:
            occupied += 1
            known = True
        elif occ < free_thresh:
            free += 1
            known = True
        else:
            unknown += 1
            known = False
        if known:
            i = idx // width
            j = idx % width
            if i < min_i:
                min_i = i
            if i > max_i:
                max_i = i
            if j < min_j:
                min_j = j
            if j > max_j:
                max_j = j

    total = width * height
    if total <= 0:
        return out
    known_total = free + occupied
    out["final_map_global_explored_ratio"] = f"{known_total / total:.6f}"
    out["final_map_global_unknown_ratio"] = f"{unknown / total:.6f}"
    out["explored_area_m2"] = f"{free * (resolution ** 2):.4f}"
    if max_i >= min_i and max_j >= min_j:
        bbox_w = (max_j - min_j + 1) * resolution
        bbox_h = (max_i - min_i + 1) * resolution
        out["active_bbox_area_m2"] = f"{bbox_w * bbox_h:.4f}"
    return out


# ---------------------------------------------------------------------------
# Bag analysis
# ---------------------------------------------------------------------------

def bag_metrics(bag_path: Path, duration_hint: float):
    """Walk the bag once and accumulate counters."""
    out = {
        "duration_sec":                    na("no_bag"),
        "active_explored_ratio":           na("no_bag"),
        "active_unknown_ratio":            na("no_bag"),
        "number_of_frontier_goals_sent":   na("no_bag"),
        "number_of_move_base_success":     na("no_bag"),
        "number_of_move_base_aborted":     na("no_bag"),
        "total_distance_traveled":         na("no_bag"),
        "min_lidar_range_over_trial":      na("no_bag"),
        "cmd_vel_active_ratio":            na("no_bag"),
        "final_status":                    na("no_bag"),
    }
    if not bag_path or not bag_path.exists():
        if duration_hint > 0:
            out["duration_sec"] = f"{duration_hint:.2f}"
        return out

    try:
        import rosbag  # type: ignore
    except Exception as exc:
        for k in out:
            out[k] = na(f"rosbag_import:{exc.__class__.__name__}")
        return out

    # Lazy import map coverage tracker; ignore if unavailable.
    coverage_tracker = None
    try:
        from map_coverage_tracker import DynamicCoverageTracker
        coverage_tracker = DynamicCoverageTracker()
    except Exception:
        coverage_tracker = None

    goal_count = 0
    success_count = 0
    aborted_count = 0
    last_status = None
    distance = 0.0
    last_xy = None
    min_range = math.inf
    cmd_active_count = 0
    cmd_total_count = 0
    t_start = None
    t_end = None
    coverage_metrics = None

    try:
        bag = rosbag.Bag(str(bag_path))
    except Exception as exc:
        for k in out:
            out[k] = na(f"rosbag_open:{exc.__class__.__name__}")
        return out

    try:
        for topic, msg, t in bag.read_messages():
            ts = t.to_sec() if hasattr(t, "to_sec") else float(t)
            if t_start is None or ts < t_start:
                t_start = ts
            if t_end is None or ts > t_end:
                t_end = ts

            if topic in ("/move_base/goal", "/move_base_simple/goal"):
                goal_count += 1
            elif topic == "/move_base/result":
                # actionlib_msgs/GoalStatus: 3=SUCCEEDED, 4=ABORTED, 5=REJECTED
                try:
                    s = int(msg.status.status)
                    if s == 3:
                        success_count += 1
                    elif s in (4, 5):
                        aborted_count += 1
                    last_status = s
                except Exception:
                    pass
            elif topic == "/move_base/status":
                try:
                    statuses = msg.status_list
                    if statuses:
                        last_status = int(statuses[-1].status)
                except Exception:
                    pass
            elif topic == "/odom":
                try:
                    p = msg.pose.pose.position
                    if last_xy is not None:
                        dx = p.x - last_xy[0]
                        dy = p.y - last_xy[1]
                        d = math.hypot(dx, dy)
                        # filter extreme jumps from TF resets
                        if d < 1.0:
                            distance += d
                    last_xy = (p.x, p.y)
                except Exception:
                    pass
            elif topic == "/lidar/scan":
                try:
                    rng_min_msg = float(msg.range_min)
                    for r in msg.ranges:
                        rf = float(r)
                        if math.isfinite(rf) and rf >= rng_min_msg and rf < min_range:
                            min_range = rf
                except Exception:
                    pass
            elif topic == "/cmd_vel":
                try:
                    cmd_total_count += 1
                    if (abs(msg.linear.x) > 1e-3 or
                            abs(msg.linear.y) > 1e-3 or
                            abs(msg.angular.z) > 1e-3):
                        cmd_active_count += 1
                except Exception:
                    pass
            elif topic == "/map" and coverage_tracker is not None:
                try:
                    coverage_metrics = coverage_tracker.update_from_occupancy_grid(msg, ts)
                except Exception:
                    coverage_metrics = None
    finally:
        bag.close()

    if t_start is not None and t_end is not None and t_end > t_start:
        out["duration_sec"] = f"{t_end - t_start:.2f}"
    elif duration_hint > 0:
        out["duration_sec"] = f"{duration_hint:.2f}"
    else:
        out["duration_sec"] = na("no_clock")

    out["number_of_frontier_goals_sent"] = str(goal_count)
    out["number_of_move_base_success"] = str(success_count)
    out["number_of_move_base_aborted"] = str(aborted_count)
    out["total_distance_traveled"] = f"{distance:.4f}" if last_xy is not None else na("no_odom")
    out["min_lidar_range_over_trial"] = (
        f"{min_range:.4f}" if math.isfinite(min_range) else na("no_lidar")
    )
    if cmd_total_count > 0:
        out["cmd_vel_active_ratio"] = f"{cmd_active_count / cmd_total_count:.4f}"
    else:
        out["cmd_vel_active_ratio"] = na("no_cmd_vel")
    out["final_status"] = str(last_status) if last_status is not None else na("no_status")

    if coverage_metrics is not None:
        if coverage_metrics.active_explored_ratio is not None:
            out["active_explored_ratio"] = f"{coverage_metrics.active_explored_ratio:.6f}"
        else:
            out["active_explored_ratio"] = na("no_active_bbox")
        if coverage_metrics.active_unknown_ratio is not None:
            out["active_unknown_ratio"] = f"{coverage_metrics.active_unknown_ratio:.6f}"
        else:
            out["active_unknown_ratio"] = na("no_active_bbox")
    else:
        out["active_explored_ratio"] = na("coverage_tracker_unavailable")
        out["active_unknown_ratio"] = na("coverage_tracker_unavailable")

    return out


def main() -> int:
    args = parse_args()
    bag_path = Path(args.bag) if args.bag else None
    map_yaml = Path(args.map_yaml) if args.map_yaml else None
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    metrics = {}
    metrics.update(bag_metrics(bag_path, args.duration_hint))
    metrics.update(map_static_metrics(map_yaml))

    # Stable output ordering.
    field_order = [
        "duration_sec",
        "final_map_global_explored_ratio",
        "final_map_global_unknown_ratio",
        "active_explored_ratio",
        "active_unknown_ratio",
        "explored_area_m2",
        "active_bbox_area_m2",
        "number_of_frontier_goals_sent",
        "number_of_move_base_success",
        "number_of_move_base_aborted",
        "total_distance_traveled",
        "min_lidar_range_over_trial",
        "cmd_vel_active_ratio",
        "final_status",
    ]
    row = {k: metrics.get(k, na("missing")) for k in field_order}
    row["bag"] = str(bag_path) if bag_path else ""
    row["map_yaml"] = str(map_yaml) if map_yaml else ""

    with output.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=field_order + ["bag", "map_yaml"])
        writer.writeheader()
        writer.writerow(row)

    for k in field_order:
        print(f"{k}: {row[k]}")
    print(f"CSV written: {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
