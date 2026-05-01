#!/usr/bin/env python3
"""Extract Gazebo model ground-truth trajectory from a rosbag into TUM format."""

import argparse
import sys
from pathlib import Path


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


def write_ground_truth(bag_path: Path, model_name: str, output_path: Path) -> int:
    rosbag = load_rosbag()
    count = 0
    missing = 0
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
                f"{pose.orientation.x:.9f} {pose.orientation.y:.9f} {pose.orientation.z:.9f} {pose.orientation.w:.9f}\n"
            )
            count += 1
    if count == 0:
        raise RuntimeError(f"No /gazebo/model_states samples found for model '{model_name}' in {bag_path}")
    print(f"PASS: wrote {count} ground-truth poses to {output_path}")
    if missing:
        print(f"WARN: {missing} model_states messages did not contain model {model_name}")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("model_name")
    parser.add_argument("output_path", type=Path)
    args = parser.parse_args()

    if not args.bag_path.is_file():
        print(f"FAIL: bag not found: {args.bag_path}", file=sys.stderr)
        return 1
    try:
        write_ground_truth(args.bag_path, args.model_name, args.output_path)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
