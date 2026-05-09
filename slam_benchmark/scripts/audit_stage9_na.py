#!/usr/bin/env python3
"""Audit Stage 9 benchmark results for N/A values and missing artifacts."""

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, List

PKG_PATH = Path(__file__).resolve().parents[1]
STAGE9 = PKG_PATH / "results" / "stage9"
RAW = STAGE9 / "raw"
CSV_DIR = STAGE9 / "csv"
LOGS_DIR = STAGE9 / "logs"

DEFAULT_CSV = CSV_DIR / "slam_comparison_raw.csv"
RERUN_CSV = CSV_DIR / "trials_to_rerun.csv"
REPORT_MD = LOGS_DIR / "audit_stage9_na_report.md"

SCENARIOS = ["corridor_static", "open_room_obstacles", "narrow_turn"]
ALGORITHMS = ["gmapping_lidar", "hector_lidar"]
REPETITIONS = ["1", "2", "3"]


def read_csv_rows(csv_path: Path) -> List[Dict[str, str]]:
    if not csv_path.is_file():
        return []
    with csv_path.open(newline="") as f:
        return list(csv.DictReader(f))


def notes_has(notes: str, key: str) -> bool:
    return key in notes.split(";")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        default=DEFAULT_CSV,
        help="Input slam_comparison_raw.csv",
    )
    args = parser.parse_args()

    rows = read_csv_rows(args.input)

    stats = {
        "total": 0,
        "success": 0,
        "trial_not_run": 0,
        "missing_bag": 0,
        "missing_map_yaml_or_pgm": 0,
        "missing_groundtruth_tum": 0,
        "missing_estimated_tum": 0,
        "missing_trajectory_metrics": 0,
        "missing_map_metrics": 0,
        "ate_na_evo_missing": 0,
        "blocked_by_obstacle": 0,
        "hector_not_run": 0,
    }

    rerun_reasons: List[Dict[str, str]] = []

    for row in rows:
        stats["total"] += 1
        scenario = row.get("scenario", "")
        algorithm = row.get("algorithm", "")
        rep = row.get("repetition", "")
        success = row.get("success", "false").lower()
        notes = row.get("notes", "")
        ate_rmse = row.get("ate_rmse", "N/A")

        if success == "true":
            stats["success"] += 1

        trial_dir = RAW / scenario / algorithm / f"rep_{rep}"
        reasons: List[str] = []

        if notes_has(notes, "trial_not_run") or not trial_dir.is_dir():
            stats["trial_not_run"] += 1
            if algorithm == "hector_lidar":
                stats["hector_not_run"] += 1
                reasons.append("hector_not_run")
            else:
                reasons.append("trial_not_run")
        else:
            # Trial directory exists — check each artifact
            if not (trial_dir / "trial.bag").is_file():
                stats["missing_bag"] += 1
                reasons.append("missing_artifacts")

            if not (trial_dir / "map.yaml").is_file() or not (trial_dir / "map.pgm").is_file():
                stats["missing_map_yaml_or_pgm"] += 1
                if "missing_artifacts" not in reasons:
                    reasons.append("missing_artifacts")

            if not (trial_dir / "groundtruth.tum").is_file():
                stats["missing_groundtruth_tum"] += 1
                if "missing_artifacts" not in reasons:
                    reasons.append("missing_artifacts")

            if not (trial_dir / "estimated.tum").is_file():
                stats["missing_estimated_tum"] += 1
                if "missing_artifacts" not in reasons:
                    reasons.append("missing_artifacts")

            if not (trial_dir / "trajectory_metrics.csv").is_file():
                stats["missing_trajectory_metrics"] += 1
                if "missing_artifacts" not in reasons:
                    reasons.append("missing_artifacts")

            if not (trial_dir / "map_metrics.csv").is_file():
                stats["missing_map_metrics"] += 1
                if "missing_artifacts" not in reasons:
                    reasons.append("missing_artifacts")

            # ATE N/A due to evo missing
            if ate_rmse in ("N/A", "") and "evo" in notes.lower():
                stats["ate_na_evo_missing"] += 1
                if "evo_missing_need_fallback" not in reasons:
                    reasons.append("evo_missing_need_fallback")

            # ATE N/A but trajectory files exist — need fallback
            traj_path = trial_dir / "trajectory_metrics.csv"
            gt_path = trial_dir / "groundtruth.tum"
            est_path = trial_dir / "estimated.tum"
            if (
                traj_path.is_file()
                and gt_path.is_file()
                and est_path.is_file()
                and ate_rmse in ("N/A", "")
                and "evo_missing_need_fallback" not in reasons
            ):
                reasons.append("evo_missing_need_fallback")
                stats["ate_na_evo_missing"] += 1

            if notes_has(notes, "blocked_by_obstacle"):
                stats["blocked_by_obstacle"] += 1
                if "blocked_by_obstacle" not in reasons:
                    reasons.append("blocked_by_obstacle")

        if reasons:
            rerun_reasons.append({
                "scenario": scenario,
                "algorithm": algorithm,
                "repetition": rep,
                "reason": ";".join(reasons),
            })

    # Print stats to stdout
    print("=== Stage 9 N/A Audit Report ===")
    print(f"Total trials:                {stats['total']}")
    print(f"Success:                     {stats['success']}")
    print(f"trial_not_run:               {stats['trial_not_run']}")
    print(f"Missing trial.bag:           {stats['missing_bag']}")
    print(f"Missing map.yaml/pgm:        {stats['missing_map_yaml_or_pgm']}")
    print(f"Missing groundtruth.tum:     {stats['missing_groundtruth_tum']}")
    print(f"Missing estimated.tum:       {stats['missing_estimated_tum']}")
    print(f"Missing trajectory_metrics:  {stats['missing_trajectory_metrics']}")
    print(f"Missing map_metrics:         {stats['missing_map_metrics']}")
    print(f"ATE N/A (evo missing):       {stats['ate_na_evo_missing']}")
    print(f"Blocked by obstacle:         {stats['blocked_by_obstacle']}")
    print(f"Hector not run:              {stats['hector_not_run']}")
    print(f"Trials needing rerun:        {len(rerun_reasons)}")

    # Write markdown report
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    with REPORT_MD.open("w") as f:
        f.write("# Stage 9 N/A Audit Report\n\n")
        f.write(f"Input: `{args.input}`\n\n")
        f.write("## Statistics\n\n")
        f.write("| Metric | Count |\n|---|---|\n")
        for key, value in stats.items():
            f.write(f"| {key} | {value} |\n")
        f.write("\n## Trials to Rerun\n\n")
        if rerun_reasons:
            f.write("| Scenario | Algorithm | Rep | Reason |\n|---|---|---|---|\n")
            for r in rerun_reasons:
                f.write(f"| {r['scenario']} | {r['algorithm']} | {r['repetition']} | {r['reason']} |\n")
        else:
            f.write("No trials need rerunning.\n")
    print(f"\nReport written: {REPORT_MD}")

    # Write trials_to_rerun.csv
    CSV_DIR.mkdir(parents=True, exist_ok=True)
    with RERUN_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["scenario", "algorithm", "repetition", "reason"])
        writer.writeheader()
        writer.writerows(rerun_reasons)
    print(f"Trials to rerun: {RERUN_CSV} ({len(rerun_reasons)} rows)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
