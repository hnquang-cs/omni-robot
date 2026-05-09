#!/usr/bin/env python3
"""Aggregate Stage 10 navigation evaluation CSV files."""

import csv
from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
BASE = PKG / "results" / "stage10"
CSV_DIR = BASE / "csv"
LATEX_DIR = BASE / "latex"
MD_DIR = BASE / "markdown"


def read_rows():
    rows = []
    for path in sorted(CSV_DIR.glob("*.csv")):
        if path.name in {"navigation_summary.csv", "navigation_trials.csv"}:
            continue
        with path.open() as handle:
            for row in csv.DictReader(handle):
                rows.append(row)
    return rows


def main():
    rows = read_rows()
    CSV_DIR.mkdir(parents=True, exist_ok=True)
    LATEX_DIR.mkdir(parents=True, exist_ok=True)
    MD_DIR.mkdir(parents=True, exist_ok=True)
    summary = CSV_DIR / "navigation_summary.csv"
    fields = ["trial", "goal", "success", "duration_sec", "final_position_error", "final_yaw_error", "travel_distance", "min_range_over_trial", "notes"]
    with summary.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, "N/A") for field in fields})

    latex = LATEX_DIR / "table_navigation_results.tex"
    with latex.open("w") as handle:
        handle.write("\\begin{tabular}{lllrrrrl}\n\\hline\n")
        handle.write("Trial & Goal & Success & Duration & Pos. Error & Yaw Error & Travel & Min Range \\\\\n\\hline\n")
        for row in rows:
            handle.write(
                f"{row.get('trial','N/A')} & {row.get('goal','N/A')} & {row.get('success','N/A')} & "
                f"{row.get('duration_sec','N/A')} & {row.get('final_position_error','N/A')} & "
                f"{row.get('final_yaw_error','N/A')} & {row.get('travel_distance','N/A')} & "
                f"{row.get('min_range_over_trial','N/A')} \\\\\n"
            )
        handle.write("\\hline\n\\end{tabular}\n")

    markdown = MD_DIR / "table_navigation_results.md"
    with markdown.open("w") as handle:
        handle.write("| Trial | Goal | Success | Duration | Final Position Error | Final Yaw Error | Travel Distance | Min Range | Notes |\n")
        handle.write("|---|---|---:|---:|---:|---:|---:|---:|---|\n")
        for row in rows:
            handle.write(
                f"| {row.get('trial','N/A')} | {row.get('goal','N/A')} | {row.get('success','N/A')} | "
                f"{row.get('duration_sec','N/A')} | {row.get('final_position_error','N/A')} | "
                f"{row.get('final_yaw_error','N/A')} | {row.get('travel_distance','N/A')} | "
                f"{row.get('min_range_over_trial','N/A')} | {row.get('notes','N/A')} |\n"
            )
    print(f"PASS: wrote {summary}")
    print(f"PASS: wrote {latex}")
    print(f"PASS: wrote {markdown}")


if __name__ == "__main__":
    main()
