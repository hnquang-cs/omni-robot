#!/usr/bin/env python3
"""Aggregate Stage 8 trial metrics into summary CSV files."""

import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional


PKG_PATH = Path(__file__).resolve().parents[1]
STAGE8 = PKG_PATH / "results" / "stage8"
RAW = STAGE8 / "raw"
CSV_DIR = STAGE8 / "csv"


def read_first_csv(path: Path) -> Dict[str, str]:
    if not path.is_file():
        return {}
    with path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return rows[0] if rows else {}


def read_metric_csv(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    if not path.is_file():
        return data
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            metric = row.get("metric", "")
            if metric:
                data[metric] = row.get("value", "N/A")
    return data


def numeric(value: str) -> Optional[float]:
    try:
        if value in ("", "N/A", None):
            return None
        val = float(value)
        if math.isnan(val):
            return None
        return val
    except Exception:
        return None


def find_file(trial_dir: Path, names: List[str], pattern: str = "") -> Optional[Path]:
    for name in names:
        path = trial_dir / name
        if path.is_file():
            return path
    if pattern:
        matches = sorted(trial_dir.glob(pattern))
        if matches:
            return matches[0]
    return None


def aggregate_rows() -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    if not RAW.is_dir():
        return rows
    for rep_dir in sorted(RAW.glob("*/*/rep_*")):
        if not rep_dir.is_dir():
            continue
        scenario = rep_dir.parents[1].name
        algorithm = rep_dir.parents[0].name
        repetition = rep_dir.name.replace("rep_", "")
        status = read_first_csv(rep_dir / "runtime_status.csv")
        traj_metrics = read_metric_csv(rep_dir / "trajectory_metrics.csv")
        if not traj_metrics:
            metrics_path = find_file(rep_dir, [], "*metrics*/trajectory_metrics.csv")
            if metrics_path:
                traj_metrics = read_metric_csv(metrics_path)
        map_path = find_file(rep_dir, ["map_metrics.csv"], "*map*metrics*.csv")
        map_metrics = read_first_csv(map_path) if map_path else {}
        success = status.get("success", "false")
        if traj_metrics and traj_metrics.get("ate_rmse", "N/A") not in ("N/A", ""):
            success = "true" if success != "false" else success
        notes = status.get("notes", "")
        if not traj_metrics:
            notes = (notes + ";missing_trajectory_metrics").strip(";")
        if not map_metrics:
            notes = (notes + ";missing_map_metrics").strip(";")
        rows.append({
            "scenario": scenario,
            "algorithm": algorithm,
            "repetition": repetition,
            "success": success,
            "ate_rmse": traj_metrics.get("ate_rmse", "N/A"),
            "ate_mean": traj_metrics.get("ate_mean", "N/A"),
            "ate_max": traj_metrics.get("ate_max", "N/A"),
            "ate_std": traj_metrics.get("ate_std", "N/A"),
            "rpe_rmse": traj_metrics.get("rpe_rmse", "N/A"),
            "map_occupied_ratio": map_metrics.get("occupied_ratio", "N/A"),
            "map_free_ratio": map_metrics.get("free_ratio", "N/A"),
            "map_unknown_ratio": map_metrics.get("unknown_ratio", "N/A"),
            "runtime_sec": status.get("runtime_sec", "N/A"),
            "notes": notes or "N/A",
        })
    return rows


def mean_std(values: List[float]):
    if not values:
        return "N/A", "N/A"
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{mean:.6f}", f"{std:.6f}"


def aggregate_means(rows: List[Dict[str, str]]) -> List[Dict[str, str]]:
    groups: Dict[tuple, List[Dict[str, str]]] = {}
    for row in rows:
        groups.setdefault((row["scenario"], row["algorithm"]), []).append(row)
    mean_rows: List[Dict[str, str]] = []
    for (scenario, algorithm), group in sorted(groups.items()):
        ate_values = [v for v in (numeric(r["ate_rmse"]) for r in group) if v is not None]
        rpe_values = [v for v in (numeric(r["rpe_rmse"]) for r in group) if v is not None]
        unknown_values = [v for v in (numeric(r["map_unknown_ratio"]) for r in group) if v is not None]
        runtime_values = [v for v in (numeric(r["runtime_sec"]) for r in group) if v is not None]
        ate_mean, ate_std = mean_std(ate_values)
        rpe_mean, rpe_std = mean_std(rpe_values)
        unknown_mean, _unknown_std = mean_std(unknown_values)
        runtime_mean, _runtime_std = mean_std(runtime_values)
        mean_rows.append({
            "scenario": scenario,
            "algorithm": algorithm,
            "n_success": str(sum(1 for r in group if r["success"].lower() == "true")),
            "ate_rmse_mean": ate_mean,
            "ate_rmse_std": ate_std,
            "rpe_rmse_mean": rpe_mean,
            "rpe_rmse_std": rpe_std,
            "map_unknown_ratio_mean": unknown_mean,
            "runtime_sec_mean": runtime_mean,
            "rank_by_ate": "N/A",
        })
    for scenario in sorted({r["scenario"] for r in mean_rows}):
        scenario_rows = [r for r in mean_rows if r["scenario"] == scenario and numeric(r["ate_rmse_mean"]) is not None]
        scenario_rows.sort(key=lambda r: numeric(r["ate_rmse_mean"]))
        for rank, row in enumerate(scenario_rows, start=1):
            row["rank_by_ate"] = str(rank)
    return mean_rows


def write_csv(path: Path, rows: List[Dict[str, str]], fieldnames: List[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    rows = aggregate_rows()
    summary_fields = [
        "scenario", "algorithm", "repetition", "success", "ate_rmse", "ate_mean", "ate_max",
        "ate_std", "rpe_rmse", "map_occupied_ratio", "map_free_ratio", "map_unknown_ratio",
        "runtime_sec", "notes",
    ]
    mean_fields = [
        "scenario", "algorithm", "n_success", "ate_rmse_mean", "ate_rmse_std",
        "rpe_rmse_mean", "rpe_rmse_std", "map_unknown_ratio_mean", "runtime_sec_mean", "rank_by_ate",
    ]
    write_csv(CSV_DIR / "stage8_summary.csv", rows, summary_fields)
    means = aggregate_means(rows)
    write_csv(CSV_DIR / "stage8_summary_mean.csv", means, mean_fields)
    print(f"PASS: wrote {CSV_DIR / 'stage8_summary.csv'} ({len(rows)} rows)")
    print(f"PASS: wrote {CSV_DIR / 'stage8_summary_mean.csv'} ({len(means)} rows)")
    if not rows:
        print("WARN: no trial directories found under results/stage8/raw")
    return 0


if __name__ == "__main__":
    sys.exit(main())
