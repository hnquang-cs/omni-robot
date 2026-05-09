#!/usr/bin/env python3
"""Evaluate one Stage 10 navigation bag against a goal."""

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


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stamp_to_sec(stamp):
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def evaluate(bag_path, goal_x, goal_y, goal_yaw):
    rosbag = load_rosbag()
    poses = []
    cmd_count = 0
    max_linear_cmd = 0.0
    max_angular_cmd = 0.0
    statuses = []
    min_range = None
    global_plan_count = 0
    local_plan_count = 0
    start = None
    end = None

    with rosbag.Bag(str(bag_path), "r") as bag:
      for topic, msg, t in bag.read_messages():
        ts = stamp_to_sec(t)
        start = ts if start is None else min(start, ts)
        end = ts if end is None else max(end, ts)
        if topic == "/gazebo/model_states" and "omni_robot" in msg.name:
            idx = msg.name.index("omni_robot")
            pose = msg.pose[idx]
            poses.append((ts, pose.position.x, pose.position.y, yaw_from_quat(pose.orientation)))
        elif topic == "/cmd_vel":
            cmd_count += 1
            linear = math.hypot(msg.linear.x, msg.linear.y)
            angular = abs(msg.angular.z)
            max_linear_cmd = max(max_linear_cmd, linear)
            max_angular_cmd = max(max_angular_cmd, angular)
        elif topic == "/move_base/status":
            for status in msg.status_list:
                statuses.append(status.status)
        elif topic == "/lidar/scan":
            finite = [r for r in msg.ranges if math.isfinite(r)]
            if finite:
                value = min(finite)
                min_range = value if min_range is None else min(min_range, value)
        elif topic in ("/move_base/GlobalPlanner/plan", "/move_base/NavfnROS/plan"):
            global_plan_count += 1
        elif topic == "/move_base/TebLocalPlannerROS/local_plan":
            local_plan_count += 1

    travel = 0.0
    for prev, cur in zip(poses, poses[1:]):
        travel += math.hypot(cur[1] - prev[1], cur[2] - prev[2])

    if poses:
        final = poses[-1]
        final_position_error = math.hypot(final[1] - goal_x, final[2] - goal_y)
        final_yaw_error = abs(angle_diff(final[3], goal_yaw))
    else:
        final_position_error = None
        final_yaw_error = None

    success = 3 in statuses
    return {
        "trial": bag_path.stem,
        "goal": f"{goal_x:.3f} {goal_y:.3f} {goal_yaw:.3f}",
        "success": str(success).lower(),
        "duration_sec": f"{(end - start):.3f}" if start is not None and end is not None else "N/A",
        "final_position_error": f"{final_position_error:.3f}" if final_position_error is not None else "N/A",
        "final_yaw_error": f"{final_yaw_error:.3f}" if final_yaw_error is not None else "N/A",
        "travel_distance": f"{travel:.3f}" if poses else "N/A",
        "cmd_count": str(cmd_count),
        "cmd_vel_published": str(cmd_count > 0).lower(),
        "max_linear_cmd": f"{max_linear_cmd:.3f}",
        "max_angular_cmd": f"{max_angular_cmd:.3f}",
        "min_range_over_trial": f"{min_range:.3f}" if min_range is not None else "N/A",
        "global_plan_appeared": str(global_plan_count > 0).lower(),
        "local_plan_appeared": str(local_plan_count > 0).lower(),
        "notes": "ok" if poses else "missing_gazebo_model_states",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("goal_x", type=float)
    parser.add_argument("goal_y", type=float)
    parser.add_argument("goal_yaw", type=float)
    parser.add_argument("output_csv", type=Path)
    args = parser.parse_args()

    if not args.bag_path.is_file():
        print(f"FAIL: bag not found: {args.bag_path}", file=sys.stderr)
        return 1
    try:
        row = evaluate(args.bag_path, args.goal_x, args.goal_y, args.goal_yaw)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_csv.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        writer.writeheader()
        writer.writerow(row)
    print(f"PASS: wrote {args.output_csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
