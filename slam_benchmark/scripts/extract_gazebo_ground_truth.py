#!/usr/bin/env python3
"""Extract Gazebo model ground-truth trajectory from a rosbag into TUM format.

Model name can be given explicitly or auto-detected from /gazebo/model_states
by matching keywords (omni, robot, mobile) and selecting the model whose XY
position changes most over the first N messages.
"""

import argparse
import sys
from pathlib import Path
from typing import List, Optional


AUTO_DETECT_KEYWORDS = ("omni", "robot", "mobile", "turtlebot", "husky", "pioneer")
AUTO_DETECT_SAMPLES = 30  # number of model_states messages to read for detection


def load_rosbag():
    try:
        import rosbag  # type: ignore
        return rosbag
    except Exception as exc:
        raise RuntimeError(
            "Could not import rosbag. Run after sourcing ROS Noetic, for example: "
            "source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash"
        ) from exc


def stamp_to_sec(stamp) -> float:
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def _position_range(positions: List) -> float:
    """Return combined XY range (max-min x) + (max-min y) as a movement score."""
    if len(positions) < 2:
        return 0.0
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    return (max(xs) - min(xs)) + (max(ys) - min(ys))


def auto_detect_model(bag_path: Path, log_lines: List[str]) -> Optional[str]:
    """Detect the moving robot model from the first messages in the bag."""
    rosbag = load_rosbag()
    model_xy: dict = {}
    count = 0
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _topic, msg, _bag_time in bag.read_messages(topics=["/gazebo/model_states"]):
            for i, name in enumerate(msg.name):
                if name not in model_xy:
                    model_xy[name] = []
                pose = msg.pose[i]
                model_xy[name].append((pose.position.x, pose.position.y))
            count += 1
            if count >= AUTO_DETECT_SAMPLES:
                break

    if not model_xy:
        return None

    log_lines.append(f"auto_detect: found models: {list(model_xy.keys())}")

    # Prefer models matching known keywords
    keyword_matches = [
        name for name in model_xy
        if any(kw in name.lower() for kw in AUTO_DETECT_KEYWORDS)
    ]
    candidates = keyword_matches if keyword_matches else list(model_xy.keys())

    # Pick the one that moved the most
    scored = [(name, _position_range(model_xy[name])) for name in candidates]
    scored.sort(key=lambda x: x[1], reverse=True)
    log_lines.append(f"auto_detect: candidates by movement: {scored}")

    # Filter out static objects (ground_plane, walls, etc.) — movement < 0.01 m
    moving = [(name, score) for name, score in scored if score > 0.01]
    if moving:
        chosen = moving[0][0]
        log_lines.append(f"auto_detect: selected '{chosen}' (movement={moving[0][1]:.4f})")
        return chosen

    # Fallback: just pick the first keyword match or first overall
    chosen = scored[0][0] if scored else None
    log_lines.append(f"auto_detect: fallback selected '{chosen}'")
    return chosen


def write_ground_truth(
    bag_path: Path,
    model_name: str,
    output_path: Path,
    log_path: Optional[Path] = None,
) -> int:
    rosbag = load_rosbag()
    log_lines: List[str] = []
    count = 0
    missing = 0

    # Auto-detect if model_name contains a placeholder
    if model_name in ("auto", ""):
        log_lines.append("model_name=auto: running auto-detection")
        detected = auto_detect_model(bag_path, log_lines)
        if detected is None:
            raise RuntimeError(
                "Could not auto-detect robot model from /gazebo/model_states"
            )
        model_name = detected
        log_lines.append(f"model_name resolved to: {model_name}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with rosbag.Bag(str(bag_path), "r") as bag, output_path.open("w") as out:
        out.write("# timestamp tx ty tz qx qy qz qw\n")
        for _topic, msg, bag_time in bag.read_messages(topics=["/gazebo/model_states"]):
            if model_name not in msg.name:
                missing += 1
                continue
            index = msg.name.index(model_name)
            pose = msg.pose[index]
            stamp = stamp_to_sec(bag_time)
            out.write(
                f"{stamp:.9f} {pose.position.x:.9f} {pose.position.y:.9f} {pose.position.z:.9f} "
                f"{pose.orientation.x:.9f} {pose.orientation.y:.9f} {pose.orientation.z:.9f} "
                f"{pose.orientation.w:.9f}\n"
            )
            count += 1

    log_lines.append(f"model_name={model_name}")
    log_lines.append(f"poses_written={count}")
    log_lines.append(f"messages_without_model={missing}")
    log_lines.append(f"output={output_path}")

    if log_path is not None:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a") as lf:
            for line in log_lines:
                lf.write(f"[extract_gt] {line}\n")

    if count == 0:
        raise RuntimeError(
            f"No /gazebo/model_states samples found for model '{model_name}' in {bag_path}"
        )
    print(f"PASS: wrote {count} ground-truth poses to {output_path}")
    if missing:
        print(f"WARN: {missing} model_states messages did not contain model '{model_name}'")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument(
        "model_name",
        help="Model name in /gazebo/model_states, or 'auto' to auto-detect.",
    )
    parser.add_argument("output_path", type=Path)
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="Optional path to append extraction log lines (trajectory_extract.log).",
    )
    args = parser.parse_args()

    if not args.bag_path.is_file():
        print(f"FAIL: bag not found: {args.bag_path}", file=sys.stderr)
        return 1
    try:
        write_ground_truth(args.bag_path, args.model_name, args.output_path, args.log)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        if args.log:
            try:
                args.log.parent.mkdir(parents=True, exist_ok=True)
                with args.log.open("a") as lf:
                    lf.write(f"[extract_gt] FAIL: {exc}\n")
            except Exception:
                pass
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
