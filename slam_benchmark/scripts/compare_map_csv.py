#!/usr/bin/env python3
"""Combine multiple evaluate_map_basic.py CSV outputs into one table."""

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, List


def read_first_row(path: Path) -> Dict[str, str]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        try:
            row = next(reader)
        except StopIteration as exc:
            raise ValueError(f"CSV has no data rows: {path}") from exc
    row["source_csv"] = str(path)
    return row


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_files", nargs="+", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=None)
    args = parser.parse_args()

    rows: List[Dict[str, str]] = []
    for csv_path in args.csv_files:
        path = csv_path.resolve()
        if not path.is_file():
            print(f"FAIL: CSV not found: {path}", file=sys.stderr)
            return 1
        try:
            rows.append(read_first_row(path))
        except Exception as exc:
            print(f"FAIL: {exc}", file=sys.stderr)
            return 1

    package_path = Path(__file__).resolve().parents[1]
    output = args.output.resolve() if args.output else package_path / "results" / "stage7_map_comparison.csv"
    output.parent.mkdir(parents=True, exist_ok=True)

    fieldnames: List[str] = []
    for row in rows:
        for key in row.keys():
            if key not in fieldnames:
                fieldnames.append(key)

    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"PASS: wrote {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
