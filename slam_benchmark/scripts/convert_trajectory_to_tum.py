#!/usr/bin/env python3
"""Normalize a whitespace or CSV trajectory file to TUM format."""

import argparse
import csv
import sys
from pathlib import Path


def parse_rows(path: Path):
    text = path.read_text().splitlines()
    for line in text:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if "," in line:
            values = next(csv.reader([line]))
        else:
            values = line.split()
        if len(values) < 8:
            continue
        yield values[:8]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_path", type=Path)
    parser.add_argument("output_path", type=Path)
    args = parser.parse_args()
    if not args.input_path.is_file():
        print(f"FAIL: input not found: {args.input_path}", file=sys.stderr)
        return 1
    rows = list(parse_rows(args.input_path))
    if not rows:
        print("FAIL: no trajectory rows with at least 8 fields", file=sys.stderr)
        return 1
    args.output_path.parent.mkdir(parents=True, exist_ok=True)
    with args.output_path.open("w") as out:
        out.write("# timestamp tx ty tz qx qy qz qw\n")
        for row in rows:
            out.write(" ".join(row) + "\n")
    print(f"PASS: wrote {len(rows)} TUM poses to {args.output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
