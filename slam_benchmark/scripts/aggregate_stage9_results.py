#!/usr/bin/env python3
"""Aggregate Stage 9 Gmapping vs Hector benchmark outputs.

Distinguishes:
- trial_not_run: rep directory does not exist at all
- incomplete_trial: directory exists but no runtime_status.csv (trial started but crashed)
- missing_artifact: trial ran but specific files are absent
- metric_na_with_reason: trajectory_metrics.csv exists but values are N/A
- map_metric_na_with_reason: map_metrics.csv exists but values are N/A
"""

import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional


PKG_PATH = Path(__file__).resolve().parents[1]
STAGE9 = PKG_PATH / "results" / "stage9"
RAW = STAGE9 / "raw"
CSV_DIR = STAGE9 / "csv"
LATEX_DIR = STAGE9 / "latex"
MARKDOWN_DIR = STAGE9 / "markdown"
PLOT_DIR = STAGE9 / "plots"

SCENARIOS = ["corridor_static", "open_room_obstacles", "narrow_turn", "wide_obstacles"]
ALGORITHMS = ["gmapping_lidar", "hector_lidar"]
REPETITIONS = ["1", "2", "3"]


def algorithms_for_scenario(scenario: str) -> List[str]:
    if scenario == "wide_obstacles":
        return ["gmapping_lidar"]
    return ALGORITHMS


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


def numeric(value: Optional[str]) -> Optional[float]:
    try:
        if value in (None, "", "N/A"):
            return None
        val = float(value)
        if math.isnan(val):
            return None
        return val
    except Exception:
        return None


def mean_std(values: List[float]):
    if not values:
        return "N/A", "N/A"
    mean = statistics.mean(values)
    std = statistics.stdev(values) if len(values) > 1 else 0.0
    return f"{mean:.6f}", f"{std:.6f}"


def add_note(notes: str, note: str) -> str:
    if not note:
        return notes
    if not notes or notes in ("N/A", ""):
        return note
    existing = notes.split(";")
    if note in existing:
        return notes
    return f"{notes};{note}"


def _short_reason(long_text: str, max_len: int = 60) -> str:
    """Truncate a reason string to avoid overly long notes."""
    text = long_text.strip().replace("\n", " ")
    return text[:max_len] if len(text) > max_len else text


def build_trial_notes(
    trial_dir: Path,
    status: Dict[str, str],
    traj: Dict[str, str],
    map_row: Dict[str, str],
) -> str:
    """Build notes string that clearly distinguishes N/A causes."""

    if not trial_dir.is_dir():
        return "trial_not_run"

    if not (trial_dir / "trial.log").is_file():
        return "incomplete_trial:no_trial_log"

    notes = ""

    # Driver / system failures from runtime_status
    raw_notes = status.get("notes", "") or ""
    if raw_notes and raw_notes not in ("N/A", "complete", "failed"):
        # Keep only driver-level reasons, not missing-file reasons (added below)
        skip_prefixes = (
            "missing_", "ate_", "rpe_", "map_occupied", "map_free", "map_unknown",
            "trajectory_metrics_reason",
        )
        for part in raw_notes.split(";"):
            if part and not any(part.startswith(p) for p in skip_prefixes):
                notes = add_note(notes, part)

    # Mandatory artifact checks
    required_files = {
        "trial.bag":              "missing_trial_bag",
        "map.yaml":               "missing_map_yaml",
        "map.pgm":                "missing_map_pgm",
        "groundtruth.tum":        "missing_groundtruth_tum",
        "estimated.tum":          "missing_estimated_tum",
        "trajectory_metrics.csv": "missing_trajectory_metrics",
        "map_metrics.csv":        "missing_map_metrics",
    }
    for filename, note in required_files.items():
        if not (trial_dir / filename).is_file():
            notes = add_note(notes, note)

    # Trajectory metric N/A with reason
    traj_path = trial_dir / "trajectory_metrics.csv"
    if traj_path.is_file():
        ate_rmse = traj.get("ate_rmse", "N/A")
        if ate_rmse in ("N/A", ""):
            source = traj.get("trajectory_metrics_source", "")
            na_reason = traj.get("na_reason", "")
            if na_reason:
                notes = add_note(notes, f"ate_na:{_short_reason(na_reason)}")
            elif source:
                notes = add_note(notes, f"ate_na:source={source}")
            else:
                notes = add_note(notes, "ate_na")

    # Map metric N/A with reason
    map_path = trial_dir / "map_metrics.csv"
    if map_path.is_file():
        occ = map_row.get("occupied_ratio", "N/A")
        if occ in ("N/A", ""):
            na_reason = map_row.get("na_reason", "")
            if na_reason:
                notes = add_note(notes, f"map_na:{_short_reason(na_reason)}")
            else:
                notes = add_note(notes, "map_na")

    # Extra notes file (e.g. odom fallback)
    for extra_file in ("estimated_notes.txt",):
        extra_path = trial_dir / extra_file
        if extra_path.is_file():
            try:
                extra = extra_path.read_text(errors="replace").strip()
                if extra:
                    notes = add_note(notes, extra)
            except Exception:
                pass

    return notes or "complete"


def aggregate_rows() -> List[Dict[str, str]]:
    rows: List[Dict[str, str]] = []
    for scenario in SCENARIOS:
        for algorithm in algorithms_for_scenario(scenario):
            for repetition in REPETITIONS:
                trial_dir = RAW / scenario / algorithm / f"rep_{repetition}"
                status = read_first_csv(trial_dir / "runtime_status.csv")
                traj = read_metric_csv(trial_dir / "trajectory_metrics.csv")
                map_row = read_first_csv(trial_dir / "map_metrics.csv")
                notes = build_trial_notes(trial_dir, status, traj, map_row)

                if not trial_dir.is_dir():
                    success = "false"
                elif not (trial_dir / "runtime_status.csv").is_file():
                    success = "false"
                else:
                    success = status.get("success", "false") or "false"

                rows.append({
                    "scenario":           scenario,
                    "algorithm":          algorithm,
                    "repetition":         repetition,
                    "success":            success,
                    "ate_rmse":           traj.get("ate_rmse", "N/A"),
                    "ate_mean":           traj.get("ate_mean", "N/A"),
                    "ate_max":            traj.get("ate_max", "N/A"),
                    "ate_std":            traj.get("ate_std", "N/A"),
                    "rpe_rmse":           traj.get("rpe_rmse", "N/A"),
                    "map_occupied_ratio": map_row.get("occupied_ratio", "N/A"),
                    "map_free_ratio":     map_row.get("free_ratio", "N/A"),
                    "map_unknown_ratio":  map_row.get("unknown_ratio", "N/A"),
                    "runtime_sec":        status.get("runtime_sec", "N/A"),
                    "notes":              notes,
                })
    return rows


def aggregate_means(rows: List[Dict[str, str]]) -> List[Dict[str, str]]:
    mean_rows: List[Dict[str, str]] = []
    for scenario in SCENARIOS:
        for algorithm in algorithms_for_scenario(scenario):
            group = [
                r for r in rows
                if r["scenario"] == scenario and r["algorithm"] == algorithm
            ]
            values = {
                "ate_rmse":           [v for v in (numeric(r["ate_rmse"]) for r in group) if v is not None],
                "ate_mean":           [v for v in (numeric(r["ate_mean"]) for r in group) if v is not None],
                "ate_max":            [v for v in (numeric(r["ate_max"]) for r in group) if v is not None],
                "ate_std":            [v for v in (numeric(r["ate_std"]) for r in group) if v is not None],
                "rpe_rmse":           [v for v in (numeric(r["rpe_rmse"]) for r in group) if v is not None],
                "map_occupied_ratio": [v for v in (numeric(r["map_occupied_ratio"]) for r in group) if v is not None],
                "map_free_ratio":     [v for v in (numeric(r["map_free_ratio"]) for r in group) if v is not None],
                "map_unknown_ratio":  [v for v in (numeric(r["map_unknown_ratio"]) for r in group) if v is not None],
                "runtime_sec":        [v for v in (numeric(r["runtime_sec"]) for r in group) if v is not None],
            }
            ate_rmse_mean, ate_rmse_std = mean_std(values["ate_rmse"])
            n_success = sum(1 for r in group if r["success"].lower() == "true")
            n_ate_valid = len(values["ate_rmse"])
            n_map_valid = len(values["map_occupied_ratio"])
            mean_rows.append({
                "scenario":               scenario,
                "algorithm":              algorithm,
                "n_trials":               str(len(group)),
                "n_success":              str(n_success),
                "n_metric_valid_ate":     str(n_ate_valid),
                "n_metric_valid_map":     str(n_map_valid),
                "ate_rmse_mean":          ate_rmse_mean,
                "ate_rmse_std":           ate_rmse_std,
                "ate_mean_mean":          mean_std(values["ate_mean"])[0],
                "ate_max_mean":           mean_std(values["ate_max"])[0],
                "ate_std_mean":           mean_std(values["ate_std"])[0],
                "rpe_rmse_mean":          mean_std(values["rpe_rmse"])[0],
                "map_occupied_ratio_mean": mean_std(values["map_occupied_ratio"])[0],
                "map_free_ratio_mean":    mean_std(values["map_free_ratio"])[0],
                "map_unknown_ratio_mean": mean_std(values["map_unknown_ratio"])[0],
                "runtime_sec_mean":       mean_std(values["runtime_sec"])[0],
                "notes":                  summarize_notes(group),
            })
    return mean_rows


def summarize_notes(group: List[Dict[str, str]]) -> str:
    """Collect unique non-trivial notes from the group, deduplicating prefixes."""
    all_parts = sorted({
        part
        for row in group
        for part in row["notes"].split(";")
        if part and part not in ("N/A", "complete")
    })
    return ";".join(all_parts) if all_parts else "N/A"


def write_csv(path: Path, rows: List[Dict[str, str]], fieldnames: List[str]):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: str) -> str:
    if value in ("", "N/A", None):
        return "N/A"
    try:
        return f"{float(value):.4f}"
    except Exception:
        return str(value)


def write_markdown(rows: List[Dict[str, str]]):
    MARKDOWN_DIR.mkdir(parents=True, exist_ok=True)
    path = MARKDOWN_DIR / "table_slam_comparison.md"
    with path.open("w") as out:
        out.write(
            "| Scenario | Algorithm | Trials | Success | ATE Valid | "
            "ATE RMSE | Map Valid | Unknown Ratio | Runtime | Notes |\n"
        )
        out.write("|---|---|---:|---:|---:|---:|---:|---:|---:|---|\n")
        for row in rows:
            notes_short = row["notes"][:60] if len(row["notes"]) > 60 else row["notes"]
            out.write(
                f"| {row['scenario']} | {row['algorithm']} "
                f"| {row['n_trials']} | {row['n_success']} "
                f"| {row['n_metric_valid_ate']} "
                f"| {fmt(row['ate_rmse_mean'])} "
                f"| {row['n_metric_valid_map']} "
                f"| {fmt(row['map_unknown_ratio_mean'])} "
                f"| {fmt(row['runtime_sec_mean'])} "
                f"| {notes_short} |\n"
            )
    print(f"PASS: wrote {path}")


def latex_escape(value: str) -> str:
    return value.replace("_", "\\_")


def write_latex(rows: List[Dict[str, str]]):
    LATEX_DIR.mkdir(parents=True, exist_ok=True)
    path = LATEX_DIR / "table_slam_comparison.tex"
    with path.open("w") as out:
        out.write("\\begin{tabular}{llrrrrrrr}\n")
        out.write("\\hline\n")
        out.write(
            "Scenario & Algorithm & Trials & Success & ATE RMSE & "
            "ATE valid & Map valid & Unknown & Runtime \\\\\n"
        )
        out.write("\\hline\n")
        for row in rows:
            out.write(
                f"{latex_escape(row['scenario'])} & {latex_escape(row['algorithm'])} & "
                f"{row['n_trials']} & {row['n_success']} & "
                f"{fmt(row['ate_rmse_mean'])} & "
                f"{row['n_metric_valid_ate']} & "
                f"{row['n_metric_valid_map']} & "
                f"{fmt(row['map_unknown_ratio_mean'])} & "
                f"{fmt(row['runtime_sec_mean'])} \\\\\n"
            )
        out.write("\\hline\n")
        out.write("\\end{tabular}\n")
    print(f"PASS: wrote {path}")


def wide_obstacles_mean_row(rows: List[Dict[str, str]]) -> Dict[str, str]:
    for row in rows:
        if row["scenario"] == "wide_obstacles" and row["algorithm"] == "gmapping_lidar":
            return row
    return {
        "scenario": "wide_obstacles",
        "algorithm": "gmapping_lidar",
        "n_trials": "3",
        "n_success": "0",
        "ate_rmse_mean": "N/A",
        "map_occupied_ratio_mean": "N/A",
        "map_free_ratio_mean": "N/A",
        "map_unknown_ratio_mean": "N/A",
        "runtime_sec_mean": "N/A",
        "notes": "trial_not_run",
    }


def write_wide_obstacles_tables(rows: List[Dict[str, str]]) -> None:
    row = wide_obstacles_mean_row(rows)
    MARKDOWN_DIR.mkdir(parents=True, exist_ok=True)
    LATEX_DIR.mkdir(parents=True, exist_ok=True)

    md_path = MARKDOWN_DIR / "table_gmapping_baseline_wide_obstacles.md"
    with md_path.open("w") as out:
        out.write(
            "| Scenario | Algorithm | Repetitions | Success | ATE RMSE mean | "
            "Map occupied ratio | Map free ratio | Map unknown ratio | Runtime mean | Notes |\n"
        )
        out.write("|---|---|---:|---:|---:|---:|---:|---:|---:|---|\n")
        out.write(
            f"| {row['scenario']} | {row['algorithm']} | {row.get('n_trials', '3')} "
            f"| {row.get('n_success', '0')} | {fmt(row.get('ate_rmse_mean', 'N/A'))} "
            f"| {fmt(row.get('map_occupied_ratio_mean', 'N/A'))} "
            f"| {fmt(row.get('map_free_ratio_mean', 'N/A'))} "
            f"| {fmt(row.get('map_unknown_ratio_mean', 'N/A'))} "
            f"| {fmt(row.get('runtime_sec_mean', 'N/A'))} "
            f"| {row.get('notes', 'N/A')} |\n"
        )
    print(f"PASS: wrote {md_path}")

    tex_path = LATEX_DIR / "table_gmapping_baseline_wide_obstacles.tex"
    with tex_path.open("w") as out:
        out.write("\\begin{tabular}{llrrrrrrrl}\n")
        out.write("\\hline\n")
        out.write("Scenario & Algorithm & Reps & Success & ATE RMSE & Occupied & Free & Unknown & Runtime & Notes \\\\\n")
        out.write("\\hline\n")
        out.write(
            f"{latex_escape(row['scenario'])} & {latex_escape(row['algorithm'])} & "
            f"{row.get('n_trials', '3')} & {row.get('n_success', '0')} & "
            f"{fmt(row.get('ate_rmse_mean', 'N/A'))} & "
            f"{fmt(row.get('map_occupied_ratio_mean', 'N/A'))} & "
            f"{fmt(row.get('map_free_ratio_mean', 'N/A'))} & "
            f"{fmt(row.get('map_unknown_ratio_mean', 'N/A'))} & "
            f"{fmt(row.get('runtime_sec_mean', 'N/A'))} & "
            f"{latex_escape(row.get('notes', 'N/A')[:48])} \\\\\n"
        )
        out.write("\\hline\n")
        out.write("\\end{tabular}\n")
    print(f"PASS: wrote {tex_path}")


def _load_matplotlib():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        return plt
    except Exception as exc:
        print(f"WARN: matplotlib unavailable; install with: pip3 install matplotlib ({exc})")
        return None


def _plot_values(filename: str, title: str, labels: List[str], values: List[Optional[float]], ylabel: str) -> None:
    plt = _load_matplotlib()
    if plt is None:
        return
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    numeric_values = [v if v is not None else 0.0 for v in values]
    ax.bar(labels, numeric_values, color=["#2f6f9f", "#4f8f5f", "#9f6b2f"][: len(labels)])
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", linestyle="--", alpha=0.35)
    if not any(v is not None for v in values):
        ax.text(0.5, 0.5, "No numeric wide_obstacles data yet", transform=ax.transAxes, ha="center")
    fig.tight_layout()
    out_path = PLOT_DIR / filename
    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    print(f"PASS: wrote {out_path}")


def write_wide_obstacles_plots(rows: List[Dict[str, str]]) -> None:
    row = wide_obstacles_mean_row(rows)
    _plot_values(
        "gmapping_wide_obstacles_map_metrics.png",
        "Gmapping Wide Obstacles Map Metrics",
        ["occupied", "free", "unknown"],
        [
            numeric(row.get("map_occupied_ratio_mean")),
            numeric(row.get("map_free_ratio_mean")),
            numeric(row.get("map_unknown_ratio_mean")),
        ],
        "ratio",
    )
    _plot_values(
        "gmapping_wide_obstacles_runtime.png",
        "Gmapping Wide Obstacles Runtime",
        ["runtime"],
        [numeric(row.get("runtime_sec_mean"))],
        "seconds",
    )


def main() -> int:
    raw_fields = [
        "scenario", "algorithm", "repetition", "success",
        "ate_rmse", "ate_mean", "ate_max", "ate_std", "rpe_rmse",
        "map_occupied_ratio", "map_free_ratio", "map_unknown_ratio",
        "runtime_sec", "notes",
    ]
    mean_fields = [
        "scenario", "algorithm", "n_trials", "n_success",
        "n_metric_valid_ate", "n_metric_valid_map",
        "ate_rmse_mean", "ate_rmse_std",
        "ate_mean_mean", "ate_max_mean", "ate_std_mean", "rpe_rmse_mean",
        "map_occupied_ratio_mean", "map_free_ratio_mean", "map_unknown_ratio_mean",
        "runtime_sec_mean", "notes",
    ]
    rows = aggregate_rows()
    means = aggregate_means(rows)
    write_csv(CSV_DIR / "slam_comparison_raw.csv", rows, raw_fields)
    write_csv(CSV_DIR / "slam_comparison_mean.csv", means, mean_fields)
    write_markdown(means)
    write_latex(means)
    write_wide_obstacles_tables(means)
    write_wide_obstacles_plots(means)
    print(f"PASS: wrote {CSV_DIR / 'slam_comparison_raw.csv'} ({len(rows)} rows)")
    print(f"PASS: wrote {CSV_DIR / 'slam_comparison_mean.csv'} ({len(means)} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
