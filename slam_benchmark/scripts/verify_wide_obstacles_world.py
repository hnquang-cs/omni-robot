#!/usr/bin/env python3
"""Verify the Stage 9 wide-obstacles Gazebo world structure."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


PKG_PATH = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_PATH.parent
WORLD = REPO_ROOT / "robot_description" / "worlds" / "test_arena_wide_obstacles.world"
REPORT = PKG_PATH / "results" / "stage9" / "logs" / "verify_wide_obstacles_world.md"


def child_text(element: ET.Element, name: str) -> str:
    child = element.find(name)
    return child.text.strip() if child is not None and child.text else ""


def main() -> int:
    lines = ["# Wide Obstacles World Verification", ""]
    ok = True

    if not WORLD.is_file():
        print(f"FAIL: world_not_found {WORLD}")
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(f"# Wide Obstacles World Verification\n\nFAIL: world_not_found `{WORLD}`\n")
        return 1

    try:
        root = ET.parse(WORLD).getroot()
    except Exception as exc:
        print(f"FAIL: xml_parse_error {exc}")
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(f"# Wide Obstacles World Verification\n\nFAIL: xml_parse_error `{exc}`\n")
        return 1

    models = root.findall(".//model")
    obstacle_models = [m for m in models if (m.get("name") or "").startswith("obstacle_box_")]
    wall_models = [m for m in models if (m.get("name") or "").startswith("wall_") or (m.get("name") == "missing_wall_stage9_fixed")]

    lines.append(f"- world: `{WORLD}`")
    lines.append(f"- obstacle_box_count: {len(obstacle_models)}")
    lines.append(f"- wall_count: {len(wall_models)}")

    if len(obstacle_models) < 6:
        ok = False
        lines.append("- obstacle_check: FAIL, expected at least 6 `obstacle_box_` models")
    elif len(obstacle_models) > 7:
        ok = False
        lines.append("- obstacle_check: FAIL, expected 6 or 7 `obstacle_box_` models")
    else:
        lines.append("- obstacle_check: PASS")

    if not any(m.get("name") == "missing_wall_stage9_fixed" for m in models) and not wall_models:
        ok = False
        lines.append("- wall_check: FAIL, no `missing_wall_stage9_fixed` or `wall_...` model found")
    else:
        lines.append("- wall_check: PASS")

    lines.append("")
    lines.append("| Model | Static | Collision | Visual |")
    lines.append("|---|---:|---:|---:|")
    for model in obstacle_models:
        name = model.get("name", "")
        is_static = child_text(model, "static").lower() == "true"
        has_collision = model.find(".//collision") is not None
        has_visual = model.find(".//visual") is not None
        if not (is_static and has_collision and has_visual):
            ok = False
        lines.append(f"| {name} | {is_static} | {has_collision} | {has_visual} |")

    lines.append("")
    lines.append(f"Result: {'PASS' if ok else 'FAIL'}")

    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text("\n".join(lines) + "\n")

    print(f"world={WORLD}")
    print(f"obstacle_box_count={len(obstacle_models)}")
    print(f"wall_count={len(wall_models)}")
    print(f"report={REPORT}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
