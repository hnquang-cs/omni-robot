#!/usr/bin/env python3
"""Create Stage 8 benchmark plots from summary CSV files."""

import csv
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple


PKG_PATH = Path(__file__).resolve().parents[1]
STAGE8 = PKG_PATH / "results" / "stage8"
CSV_DIR = STAGE8 / "csv"
PLOT_DIR = STAGE8 / "plots"


def load_matplotlib():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception as exc:
        raise RuntimeError(
            "matplotlib is not available. Suggested install: pip3 install matplotlib"
        ) from exc


def numeric(value: str) -> Optional[float]:
    try:
        if value in ("", "N/A", None):
            return None
        return float(value)
    except Exception:
        return None


def read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def grouped_algorithm_means(rows: List[Dict[str, str]], field: str) -> Tuple[List[str], List[float]]:
    groups: Dict[str, List[float]] = defaultdict(list)
    for row in rows:
        value = numeric(row.get(field, "N/A"))
        if value is not None:
            groups[row.get("algorithm", "unknown")].append(value)
    labels = sorted(groups)
    values = [sum(groups[label]) / len(groups[label]) for label in labels]
    return labels, values


def bar_plot(plt, labels: List[str], values: List[float], title: str, ylabel: str, out_path: Path):
    if not labels:
        print(f"WARN: skipping {out_path.name}; no numeric data")
        return
    fig, ax = plt.subplots(figsize=(8, 4.8))
    ax.bar(labels, values, color=["#2f6f9f", "#cc7a29", "#4b8f4f", "#7a4a9e"][:len(labels)])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Algorithm")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", rotation=20)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"PASS: wrote {out_path}")


def read_tum(path: Path) -> List[Tuple[float, float, float]]:
    poses: List[Tuple[float, float, float]] = []
    if not path.is_file():
        return poses
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) >= 4:
            try:
                poses.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue
    return poses


def trajectory_overlays(plt):
    trajectory_dir = STAGE8 / "trajectories"
    raw_dir = STAGE8 / "raw"
    pairs = []
    for gt in sorted(trajectory_dir.glob("*groundtruth*.tum")):
        stem = gt.stem.replace("_groundtruth", "")
        est_candidates = list(trajectory_dir.glob(stem + "*estimated*.tum"))
        for est in est_candidates:
            pairs.append((stem, gt, est))
    for gt in sorted(raw_dir.glob("*/*/rep_*/*groundtruth*.tum")):
        for est in gt.parent.glob("*estimated*.tum"):
            pairs.append((gt.parent.parent.parent.name + "_" + gt.parent.parent.name + "_" + gt.parent.name, gt, est))
    seen = set()
    for label, gt_path, est_path in pairs:
        key = (str(gt_path), str(est_path))
        if key in seen:
            continue
        seen.add(key)
        gt = read_tum(gt_path)
        est = read_tum(est_path)
        if not gt or not est:
            continue
        fig, ax = plt.subplots(figsize=(6.2, 6.0))
        ax.plot([p[1] for p in gt], [p[2] for p in gt], label="Gazebo ground truth", linewidth=2)
        ax.plot([p[1] for p in est], [p[2] for p in est], label="SLAM estimate", linewidth=1.5)
        ax.set_title(f"Trajectory overlay: {label}")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.axis("equal")
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend()
        fig.tight_layout()
        out_path = PLOT_DIR / f"trajectory_overlay_{label}.png"
        fig.savefig(out_path, dpi=300)
        plt.close(fig)
        print(f"PASS: wrote {out_path}")


def main() -> int:
    try:
        plt = load_matplotlib()
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        print("Suggested install: pip3 install matplotlib")
        return 0
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    mean_rows = read_csv(CSV_DIR / "stage8_summary_mean.csv")
    if not mean_rows:
        print(f"WARN: no rows in {CSV_DIR / 'stage8_summary_mean.csv'}")
    for field, filename, title, ylabel in [
        ("ate_rmse_mean", "bar_ate_rmse_by_algorithm.png", "ATE RMSE by Algorithm", "ATE RMSE [m]"),
        ("rpe_rmse_mean", "bar_rpe_rmse_by_algorithm.png", "RPE RMSE by Algorithm", "RPE RMSE [m]"),
        ("runtime_sec_mean", "bar_runtime_by_algorithm.png", "Runtime by Algorithm", "Runtime [s]"),
        ("map_unknown_ratio_mean", "bar_unknown_ratio_by_algorithm.png", "Unknown Map Ratio by Algorithm", "Unknown ratio"),
    ]:
        labels, values = grouped_algorithm_means(mean_rows, field)
        bar_plot(plt, labels, values, title, ylabel, PLOT_DIR / filename)
    trajectory_overlays(plt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
