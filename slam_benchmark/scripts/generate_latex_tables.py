#!/usr/bin/env python3
"""Generate Stage 8 Markdown and LaTeX comparison tables."""

import csv
import sys
from pathlib import Path


PKG_PATH = Path(__file__).resolve().parents[1]
STAGE8 = PKG_PATH / "results" / "stage8"
SUMMARY = STAGE8 / "csv" / "stage8_summary_mean.csv"
LATEX_OUT = STAGE8 / "latex" / "table_slam_comparison.tex"
MD_OUT = STAGE8 / "markdown" / "table_slam_comparison.md"


def fmt(value: str, digits: int = 3) -> str:
    try:
        if value in ("", "N/A", None):
            return "N/A"
        return f"{float(value):.{digits}f}"
    except Exception:
        return value or "N/A"


def read_rows(path: Path):
    if not path.is_file():
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def latex_escape(text: str) -> str:
    return text.replace("_", "\\_")


def main() -> int:
    rows = read_rows(SUMMARY)
    LATEX_OUT.parent.mkdir(parents=True, exist_ok=True)
    MD_OUT.parent.mkdir(parents=True, exist_ok=True)

    md_lines = [
        "| Scenario | Algorithm | ATE RMSE | RPE RMSE | Runtime | Unknown Ratio | Success Count |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        md_lines.append(
            f"| {row.get('scenario','N/A')} | {row.get('algorithm','N/A')} | "
            f"{fmt(row.get('ate_rmse_mean','N/A'))} | {fmt(row.get('rpe_rmse_mean','N/A'))} | "
            f"{fmt(row.get('runtime_sec_mean','N/A'), 1)} | {fmt(row.get('map_unknown_ratio_mean','N/A'))} | "
            f"{row.get('n_success','0')} |"
        )
    if not rows:
        md_lines.append("| N/A | N/A | N/A | N/A | N/A | N/A | 0 |")
    MD_OUT.write_text("\n".join(md_lines) + "\n")

    latex_lines = [
        "\\begin{table}[htbp]",
        "\\centering",
        "\\caption{Comparison of SLAM algorithms using stereo-derived LaserScan in Gazebo simulation.}",
        "\\begin{tabular}{llrrrrr}",
        "\\hline",
        "Scenario & Algorithm & ATE RMSE & RPE RMSE & Runtime & Unknown Ratio & Success \\\\",
        "\\hline",
    ]
    if rows:
        for row in rows:
            latex_lines.append(
                f"{latex_escape(row.get('scenario','N/A'))} & {latex_escape(row.get('algorithm','N/A'))} & "
                f"{fmt(row.get('ate_rmse_mean','N/A'))} & {fmt(row.get('rpe_rmse_mean','N/A'))} & "
                f"{fmt(row.get('runtime_sec_mean','N/A'), 1)} & {fmt(row.get('map_unknown_ratio_mean','N/A'))} & "
                f"{row.get('n_success','0')} \\\\"
            )
    else:
        latex_lines.append("N/A & N/A & N/A & N/A & N/A & N/A & 0 \\\\")
    latex_lines += ["\\hline", "\\end{tabular}", "\\end{table}", ""]
    LATEX_OUT.write_text("\n".join(latex_lines))
    print(f"PASS: wrote {LATEX_OUT}")
    print(f"PASS: wrote {MD_OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
