#!/usr/bin/env python3
"""Create Stage 9 Gmapping vs Hector comparison plots."""

import csv
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple


PKG_PATH = Path(__file__).resolve().parents[1]
STAGE9 = PKG_PATH / "results" / "stage9"
CSV_DIR = STAGE9 / "csv"
PLOT_DIR = STAGE9 / "plots"


def load_matplotlib():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception as exc:
        raise RuntimeError("matplotlib is not available. Suggested install: pip3 install matplotlib") from exc


def numeric(value: Optional[str]) -> Optional[float]:
    try:
        if value in (None, "", "N/A"):
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


def placeholder_plot(plt, out_path: Path, title: str, reason: str):
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.text(0.5, 0.55, title, ha="center", va="center", fontsize=12)
    ax.text(0.5, 0.42, reason, ha="center", va="center", fontsize=9)
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"WARN: wrote placeholder {out_path}: {reason}")


def bar_plot(plt, labels: List[str], values: List[float], title: str, ylabel: str, out_path: Path):
    if not labels:
        placeholder_plot(plt, out_path, title, "No numeric values available yet")
        return
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    colors = {"gmapping_lidar": "#2f6f9f", "hector_lidar": "#cc7a29"}
    ax.bar(labels, values, color=[colors.get(label, "#666666") for label in labels])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xlabel("Algorithm")
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    ax.tick_params(axis="x", rotation=12)
    fig.tight_layout()
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"PASS: wrote {out_path}")


def main() -> int:
    try:
        plt = load_matplotlib()
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 0
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    rows = read_csv(CSV_DIR / "slam_comparison_mean.csv")
    if not rows:
        print(f"WARN: no rows in {CSV_DIR / 'slam_comparison_mean.csv'}")
    for field, filename, title, ylabel in [
        ("ate_rmse_mean", "bar_ate_rmse_by_algorithm.png", "ATE RMSE by Algorithm", "ATE RMSE [m]"),
        ("map_unknown_ratio_mean", "bar_unknown_ratio_by_algorithm.png", "Unknown Map Ratio by Algorithm", "Unknown ratio"),
        ("runtime_sec_mean", "bar_runtime_by_algorithm.png", "Runtime by Algorithm", "Runtime [s]"),
    ]:
        labels, values = grouped_algorithm_means(rows, field)
        bar_plot(plt, labels, values, title, ylabel, PLOT_DIR / filename)
    return 0


if __name__ == "__main__":
    sys.exit(main())
