#!/usr/bin/env python3
"""Analyze /cmd_vel from a live ROS topic or a rosbag."""

import argparse
import csv
import math
import sys
from pathlib import Path


def load_rosbag():
    try:
        import rosbag  # type: ignore
        return rosbag
    except Exception as exc:
        raise RuntimeError("Could not import rosbag. Source ROS Noetic before running.") from exc


def summarize(samples, threshold):
    if not samples:
        return {
            "sample_count": 0,
            "max_abs_vx": 0.0,
            "max_abs_vy": 0.0,
            "max_abs_wz": 0.0,
            "mean_abs_vx": 0.0,
            "mean_abs_vy": 0.0,
            "mean_abs_wz": 0.0,
            "ratio_abs_wz_to_linear_speed": "N/A",
            "number_of_large_yaw_commands": 0,
        }

    abs_vx = [abs(s[1]) for s in samples]
    abs_vy = [abs(s[2]) for s in samples]
    abs_wz = [abs(s[3]) for s in samples]
    speeds = [math.hypot(s[1], s[2]) for s in samples]
    mean_speed = sum(speeds) / len(speeds)
    mean_wz = sum(abs_wz) / len(abs_wz)
    ratio = mean_wz / mean_speed if mean_speed > 1e-3 else float("inf")
    return {
        "sample_count": len(samples),
        "max_abs_vx": max(abs_vx),
        "max_abs_vy": max(abs_vy),
        "max_abs_wz": max(abs_wz),
        "mean_abs_vx": sum(abs_vx) / len(abs_vx),
        "mean_abs_vy": sum(abs_vy) / len(abs_vy),
        "mean_abs_wz": mean_wz,
        "ratio_abs_wz_to_linear_speed": ratio,
        "number_of_large_yaw_commands": sum(1 for value in abs_wz if value > threshold),
    }


def read_bag(path, topic):
    rosbag = load_rosbag()
    samples = []
    with rosbag.Bag(str(path), "r") as bag:
        for bag_topic, msg, t in bag.read_messages(topics=[topic]):
            ts = float(t.secs) + float(t.nsecs) * 1e-9
            samples.append((ts, msg.linear.x, msg.linear.y, msg.angular.z))
    return samples


def read_live(topic, duration):
    import rospy
    from geometry_msgs.msg import Twist

    samples = []

    def callback(msg):
        samples.append((rospy.Time.now().to_sec(), msg.linear.x, msg.linear.y, msg.angular.z))

    rospy.init_node("analyze_cmd_vel_stage10", anonymous=True)
    rospy.Subscriber(topic, Twist, callback, queue_size=100)
    rospy.sleep(duration)
    return samples


def write_csv(path, row):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "source",
        "topic",
        "duration_sec",
        "sample_count",
        "max_abs_vx",
        "max_abs_vy",
        "max_abs_wz",
        "mean_abs_vx",
        "mean_abs_vy",
        "mean_abs_wz",
        "ratio_abs_wz_to_linear_speed",
        "number_of_large_yaw_commands",
        "large_yaw_threshold",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerow(row)


def fmt(value):
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if math.isinf(value):
        return "inf"
    return f"{value:.6f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--large-yaw-threshold", type=float, default=0.4)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("$(rospack find nav_bringup)/results/stage10/csv/cmd_vel_analysis.csv"),
    )
    args = parser.parse_args()

    if str(args.output).startswith("$(rospack"):
        import subprocess

        pkg = subprocess.check_output(["rospack", "find", "nav_bringup"], text=True).strip()
        args.output = Path(pkg) / "results/stage10/csv/cmd_vel_analysis.csv"

    try:
        if args.bag:
            samples = read_bag(args.bag, args.topic)
            source = str(args.bag)
            duration = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0
        else:
            samples = read_live(args.topic, args.duration)
            source = "live"
            duration = args.duration
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1

    metrics = summarize(samples, args.large_yaw_threshold)
    row = {
        "source": source,
        "topic": args.topic,
        "duration_sec": fmt(duration),
        "large_yaw_threshold": fmt(args.large_yaw_threshold),
    }
    row.update({key: fmt(value) for key, value in metrics.items()})
    write_csv(args.output, row)
    print(f"PASS: wrote {args.output}")
    for key in row:
        print(f"{key}: {row[key]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
