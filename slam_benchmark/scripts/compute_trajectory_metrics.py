#!/usr/bin/env python3
"""Compute ATE and RPE from TUM-format trajectory files without requiring evo.

TUM format: timestamp tx ty tz qx qy qz qw  (one pose per line)
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

# (timestamp, (x, y, z))
Pose = Tuple[float, Tuple[float, float, float]]


def load_tum(path: Path) -> List[Pose]:
    poses: List[Pose] = []
    if not path.is_file():
        return poses
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                t = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                poses.append((t, (x, y, z)))
            except ValueError:
                continue
    return sorted(poses, key=lambda p: p[0])


def nearest_match(ref_poses: List[Pose], query_time: float, max_diff: float) -> Optional[Pose]:
    if not ref_poses:
        return None
    lo, hi = 0, len(ref_poses) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if ref_poses[mid][0] < query_time:
            lo = mid + 1
        else:
            hi = mid
    best: Optional[Pose] = None
    best_diff = float("inf")
    for idx in (lo - 1, lo):
        if 0 <= idx < len(ref_poses):
            diff = abs(ref_poses[idx][0] - query_time)
            if diff < best_diff:
                best_diff = diff
                best = ref_poses[idx]
    if best is not None and best_diff <= max_diff:
        return best
    return None


def sync_trajectories(
    gt: List[Pose], est: List[Pose], max_time_diff: float
) -> Tuple[List[Pose], List[Pose]]:
    synced_gt: List[Pose] = []
    synced_est: List[Pose] = []
    for gt_pose in gt:
        match = nearest_match(est, gt_pose[0], max_time_diff)
        if match is not None:
            synced_gt.append(gt_pose)
            synced_est.append(match)
    return synced_gt, synced_est


def compute_ate(
    gt: List[Pose], est: List[Pose]
) -> Tuple[float, float, float, float]:
    errors = [
        math.sqrt(sum((g - e) ** 2 for g, e in zip(gp[1], ep[1])))
        for gp, ep in zip(gt, est)
    ]
    n = len(errors)
    ate_rmse = math.sqrt(sum(e ** 2 for e in errors) / n)
    ate_mean = sum(errors) / n
    ate_max = max(errors)
    ate_std = math.sqrt(sum((e - ate_mean) ** 2 for e in errors) / n)
    return ate_rmse, ate_mean, ate_max, ate_std


def compute_rpe(
    gt: List[Pose], est: List[Pose], delta: int = 1
) -> Optional[Tuple[float, float, float, float]]:
    n = len(gt)
    if n < delta + 2:
        return None
    errors = []
    for i in range(n - delta):
        gt_rel = tuple(gt[i + delta][1][j] - gt[i][1][j] for j in range(3))
        est_rel = tuple(est[i + delta][1][j] - est[i][1][j] for j in range(3))
        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(gt_rel, est_rel)))
        errors.append(err)
    if not errors:
        return None
    n_err = len(errors)
    rpe_rmse = math.sqrt(sum(e ** 2 for e in errors) / n_err)
    rpe_mean = sum(errors) / n_err
    rpe_max = max(errors)
    rpe_std = math.sqrt(sum((e - rpe_mean) ** 2 for e in errors) / n_err)
    return rpe_rmse, rpe_mean, rpe_max, rpe_std


def write_metrics(
    output: Path,
    ate: Optional[Tuple],
    rpe: Optional[Tuple],
    source: str,
    reason: str = "",
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    rows = []
    if ate is not None:
        rows += [
            ("ate_rmse", f"{ate[0]:.6f}"),
            ("ate_mean", f"{ate[1]:.6f}"),
            ("ate_max", f"{ate[2]:.6f}"),
            ("ate_std", f"{ate[3]:.6f}"),
        ]
    else:
        rows += [
            ("ate_rmse", "N/A"),
            ("ate_mean", "N/A"),
            ("ate_max", "N/A"),
            ("ate_std", "N/A"),
        ]
    if rpe is not None:
        rows += [
            ("rpe_rmse", f"{rpe[0]:.6f}"),
            ("rpe_mean", f"{rpe[1]:.6f}"),
            ("rpe_max", f"{rpe[2]:.6f}"),
            ("rpe_std", f"{rpe[3]:.6f}"),
        ]
    else:
        rows += [
            ("rpe_rmse", "N/A"),
            ("rpe_mean", "N/A"),
            ("rpe_max", "N/A"),
            ("rpe_std", "N/A"),
        ]
    rows.append(("trajectory_metrics_source", source))
    if reason:
        rows.append(("na_reason", reason))
    with output.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["metric", "value"])
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--groundtruth", type=Path, required=True)
    parser.add_argument("--estimated", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    args = parser.parse_args()

    gt_path = args.groundtruth
    est_path = args.estimated
    output = args.output

    if not gt_path.is_file():
        print(f"WARN: groundtruth not found: {gt_path}", file=sys.stderr)
        write_metrics(output, None, None, "python_fallback", "groundtruth_missing")
        return 0

    if not est_path.is_file():
        print(f"WARN: estimated not found: {est_path}", file=sys.stderr)
        write_metrics(output, None, None, "python_fallback", "estimated_missing")
        return 0

    gt = load_tum(gt_path)
    est = load_tum(est_path)

    if len(gt) < 10:
        msg = f"groundtruth_too_short:{len(gt)}_samples"
        write_metrics(output, None, None, "python_fallback", msg)
        print(f"WARN: groundtruth has only {len(gt)} samples (need >=10)", file=sys.stderr)
        return 0

    if len(est) < 10:
        msg = f"estimated_too_short:{len(est)}_samples"
        write_metrics(output, None, None, "python_fallback", msg)
        print(f"WARN: estimated has only {len(est)} samples (need >=10)", file=sys.stderr)
        return 0

    # Try sync with requested max_time_diff; fall back to 0.10s for sparse data
    synced_gt, synced_est = sync_trajectories(gt, est, args.max_time_diff)
    if len(synced_gt) < 10:
        synced_gt, synced_est = sync_trajectories(gt, est, 0.10)
    if len(synced_gt) < 10:
        msg = f"sync_failed:only_{len(synced_gt)}_pairs"
        write_metrics(output, None, None, "python_fallback", msg)
        print(f"WARN: only {len(synced_gt)} synchronized pairs after retry", file=sys.stderr)
        return 0

    try:
        ate = compute_ate(synced_gt, synced_est)
    except Exception as exc:
        write_metrics(output, None, None, "python_fallback", f"ate_error:{exc}")
        print(f"FAIL: ATE computation error: {exc}", file=sys.stderr)
        return 1

    try:
        rpe = compute_rpe(synced_gt, synced_est)
    except Exception:
        rpe = None

    write_metrics(output, ate, rpe, "python_fallback")
    print(f"PASS: ate_rmse={ate[0]:.6f} ate_mean={ate[1]:.6f} pairs={len(synced_gt)}")
    if rpe:
        print(f"PASS: rpe_rmse={rpe[0]:.6f}")
    else:
        print("WARN: RPE not computed (not_enough_pairs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
