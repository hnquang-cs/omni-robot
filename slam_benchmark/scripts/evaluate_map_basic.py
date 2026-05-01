#!/usr/bin/env python3
"""Compute simple occupancy statistics for a map_server YAML map."""

import argparse
import csv
import struct
import sys
import time
import zlib
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


def parse_simple_yaml(path: Path) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = value.strip().strip("'\"")
    return data


def read_pgm(path: Path) -> Tuple[int, int, List[int]]:
    content = path.read_bytes()
    tokens: List[bytes] = []
    index = 0
    while len(tokens) < 4 and index < len(content):
        while index < len(content) and content[index:index + 1].isspace():
            index += 1
        if index < len(content) and content[index:index + 1] == b"#":
            while index < len(content) and content[index:index + 1] not in b"\r\n":
                index += 1
            continue
        start = index
        while index < len(content) and not content[index:index + 1].isspace():
            index += 1
        tokens.append(content[start:index])

    if len(tokens) < 4:
        raise ValueError(f"Invalid PGM header: {path}")

    magic = tokens[0]
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    if max_value <= 0 or max_value > 255:
        raise ValueError("Only 8-bit PGM maps are supported")

    while index < len(content) and content[index:index + 1].isspace():
        index += 1

    if magic == b"P5":
        pixels = list(content[index:index + width * height])
    elif magic == b"P2":
        text = content[index:].decode("ascii", errors="replace")
        pixels = [int(value) for value in text.split()]
    else:
        raise ValueError(f"Unsupported PGM magic {magic!r}")

    if len(pixels) != width * height:
        raise ValueError(f"Expected {width * height} pixels, got {len(pixels)}")
    return width, height, pixels


def paeth(left: int, up: int, up_left: int) -> int:
    estimate = left + up - up_left
    left_distance = abs(estimate - left)
    up_distance = abs(estimate - up)
    up_left_distance = abs(estimate - up_left)
    if left_distance <= up_distance and left_distance <= up_left_distance:
        return left
    if up_distance <= up_left_distance:
        return up
    return up_left


def unfilter_scanlines(raw: bytes, width: int, height: int, channels: int) -> List[int]:
    stride = width * channels
    previous = [0] * stride
    pixels: List[int] = []
    offset = 0

    for _row in range(height):
        filter_type = raw[offset]
        offset += 1
        current = list(raw[offset:offset + stride])
        offset += stride

        for i, value in enumerate(current):
            left = current[i - channels] if i >= channels else 0
            up = previous[i]
            up_left = previous[i - channels] if i >= channels else 0
            if filter_type == 0:
                restored = value
            elif filter_type == 1:
                restored = value + left
            elif filter_type == 2:
                restored = value + up
            elif filter_type == 3:
                restored = value + ((left + up) // 2)
            elif filter_type == 4:
                restored = value + paeth(left, up, up_left)
            else:
                raise ValueError(f"Unsupported PNG filter type {filter_type}")
            current[i] = restored & 0xFF

        for x in range(width):
            base = x * channels
            if channels == 1:
                gray = current[base]
            elif channels == 2:
                gray = current[base]
            elif channels == 3:
                gray = int(round(sum(current[base:base + 3]) / 3.0))
            elif channels == 4:
                gray = int(round(sum(current[base:base + 3]) / 3.0))
            else:
                raise ValueError(f"Unsupported channel count {channels}")
            pixels.append(gray)
        previous = current

    return pixels


def read_png(path: Path) -> Tuple[int, int, List[int]]:
    data = path.read_bytes()
    signature = b"\x89PNG\r\n\x1a\n"
    if not data.startswith(signature):
        raise ValueError(f"Invalid PNG signature: {path}")

    offset = len(signature)
    width = height = bit_depth = color_type = None
    idat_parts: List[bytes] = []

    while offset < len(data):
        length = struct.unpack(">I", data[offset:offset + 4])[0]
        chunk_type = data[offset + 4:offset + 8]
        chunk_data = data[offset + 8:offset + 8 + length]
        offset += 12 + length

        if chunk_type == b"IHDR":
            width, height, bit_depth, color_type, _compression, _filter, interlace = struct.unpack(
                ">IIBBBBB",
                chunk_data,
            )
            if bit_depth != 8:
                raise ValueError("Only 8-bit PNG maps are supported")
            if interlace != 0:
                raise ValueError("Interlaced PNG maps are not supported")
        elif chunk_type == b"IDAT":
            idat_parts.append(chunk_data)
        elif chunk_type == b"IEND":
            break

    if width is None or height is None or bit_depth is None or color_type is None:
        raise ValueError(f"Missing PNG IHDR: {path}")

    channel_map = {0: 1, 2: 3, 4: 2, 6: 4}
    if color_type not in channel_map:
        raise ValueError(f"Unsupported PNG color type {color_type}")

    raw = zlib.decompress(b"".join(idat_parts))
    pixels = unfilter_scanlines(raw, width, height, channel_map[color_type])
    if len(pixels) != width * height:
        raise ValueError(f"Expected {width * height} pixels, got {len(pixels)}")
    return width, height, pixels


def load_image(path: Path) -> Tuple[int, int, List[int]]:
    suffix = path.suffix.lower()
    if suffix == ".pgm":
        return read_pgm(path)
    if suffix == ".png":
        return read_png(path)
    raise ValueError(f"Unsupported map image extension: {path.suffix}")


def classify_pixels(
    pixels: Iterable[int],
    occupied_thresh: float,
    free_thresh: float,
    negate: int,
) -> Tuple[int, int, int]:
    occupied = 0
    free = 0
    unknown = 0
    for value in pixels:
        occ = value / 255.0 if negate else (255.0 - value) / 255.0
        if occ > occupied_thresh:
            occupied += 1
        elif occ < free_thresh:
            free += 1
        else:
            unknown += 1
    return occupied, free, unknown


def evaluate_map(map_yaml: Path, ground_truth_map: Path = None) -> Dict[str, str]:
    yaml_data = parse_simple_yaml(map_yaml)
    if "image" not in yaml_data:
        raise ValueError(f"Missing 'image' entry in {map_yaml}")

    image_path = Path(yaml_data["image"])
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path

    width, height, pixels = load_image(image_path)
    resolution = float(yaml_data.get("resolution", "0.0"))
    occupied_thresh = float(yaml_data.get("occupied_thresh", "0.65"))
    free_thresh = float(yaml_data.get("free_thresh", "0.196"))
    negate = int(float(yaml_data.get("negate", "0")))
    occupied, free, unknown = classify_pixels(pixels, occupied_thresh, free_thresh, negate)
    total = width * height

    row = {
        "map_yaml": str(map_yaml),
        "image": str(image_path),
        "width": str(width),
        "height": str(height),
        "resolution": f"{resolution:.6f}",
        "area_m2": f"{width * height * resolution * resolution:.6f}",
        "occupied_cell_count": str(occupied),
        "free_cell_count": str(free),
        "unknown_cell_count": str(unknown),
        "total_cell_count": str(total),
        "occupied_ratio": f"{occupied / total:.6f}" if total else "0.000000",
        "free_ratio": f"{free / total:.6f}" if total else "0.000000",
        "unknown_ratio": f"{unknown / total:.6f}" if total else "0.000000",
        "occupied_iou": "N/A",
    }
    if ground_truth_map:
        try:
            gt_yaml = parse_simple_yaml(ground_truth_map)
            gt_image = Path(gt_yaml["image"])
            if not gt_image.is_absolute():
                gt_image = ground_truth_map.parent / gt_image
            gt_width, gt_height, gt_pixels = load_image(gt_image)
            if gt_width == width and gt_height == height:
                gt_occ, _gt_free, _gt_unknown = classify_pixels(
                    gt_pixels,
                    float(gt_yaml.get("occupied_thresh", "0.65")),
                    float(gt_yaml.get("free_thresh", "0.196")),
                    int(float(gt_yaml.get("negate", "0"))),
                )
                pred_occ_set = {
                    i for i, value in enumerate(pixels)
                    if ((value / 255.0 if negate else (255.0 - value) / 255.0) > occupied_thresh)
                }
                gt_negate = int(float(gt_yaml.get("negate", "0")))
                gt_occupied_thresh = float(gt_yaml.get("occupied_thresh", "0.65"))
                gt_occ_set = {
                    i for i, value in enumerate(gt_pixels)
                    if ((value / 255.0 if gt_negate else (255.0 - value) / 255.0) > gt_occupied_thresh)
                }
                union = len(pred_occ_set | gt_occ_set)
                intersection = len(pred_occ_set & gt_occ_set)
                row["occupied_iou"] = f"{intersection / union:.6f}" if union else "N/A"
                row["ground_truth_occupied_cells"] = str(gt_occ)
            else:
                row["occupied_iou"] = "N/A(size_mismatch)"
        except Exception as exc:
            row["occupied_iou"] = f"N/A({exc})"
    return row


def default_output_path(map_yaml: Path) -> Path:
    package_path = Path(__file__).resolve().parents[1]
    results_dir = package_path / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return results_dir / f"{map_yaml.stem}_basic_metrics_{stamp}.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("output_positional", nargs="?", type=Path, default=None)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--ground-truth-map", type=Path, default=None)
    args = parser.parse_args()

    map_yaml = args.map_yaml.resolve()
    if not map_yaml.is_file():
        print(f"FAIL: map yaml not found: {map_yaml}", file=sys.stderr)
        return 1

    try:
        gt_map = args.ground_truth_map.resolve() if args.ground_truth_map else None
        row = evaluate_map(map_yaml, gt_map)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    output_arg = args.output or args.output_positional
    output = output_arg.resolve() if output_arg else default_output_path(map_yaml)
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(row.keys())
    with output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(row)

    print(f"PASS: wrote {output}")
    print(
        "occupied={occupied_cell_count} free={free_cell_count} unknown={unknown_cell_count} "
        "occupied_ratio={occupied_ratio} free_ratio={free_ratio} unknown_ratio={unknown_ratio}".format(**row)
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
