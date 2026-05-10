#!/usr/bin/env python3
"""Measure Stage 9 /cmd_vel speed and forward-only constraints."""

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from typing import List

import rospy
import rospkg
from geometry_msgs.msg import Twist


@dataclass
class Sample:
    linear_x: float
    linear_y: float
    angular_z: float


def default_output() -> str:
    package_path = rospkg.RosPack().get_path("slam_benchmark")
    return os.path.join(
        package_path, "results", "stage9", "logs", "stage9_runtime_speed_check.csv"
    )


def thresholds(profile: str) -> tuple:
    if profile == "fast":
        return 0.30, 0.45
    if profile == "safe":
        return 0.08, 0.12
    return 0.20, 0.30


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--profile", choices=["safe", "normal", "fast"], default="normal")
    parser.add_argument("--output", default=default_output())
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> int:
    args = parse_args()
    rospy.init_node("stage9_runtime_speed_check", anonymous=True)
    samples: List[Sample] = []

    def callback(msg: Twist) -> None:
        samples.append(Sample(msg.linear.x, msg.linear.y, msg.angular.z))

    rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=100)

    deadline = time.monotonic() + args.duration
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        rate.sleep()

    count = len(samples)
    max_linear_x = max((sample.linear_x for sample in samples), default=0.0)
    mean_linear_x_abs = (
        sum(abs(sample.linear_x) for sample in samples) / count if count else 0.0
    )
    max_angular_z_abs = max((abs(sample.angular_z) for sample in samples), default=0.0)
    mean_angular_z_abs = (
        sum(abs(sample.angular_z) for sample in samples) / count if count else 0.0
    )
    any_negative_x = any(sample.linear_x < -1e-6 for sample in samples)
    any_nonzero_y = any(abs(sample.linear_y) > 1e-6 for sample in samples)

    min_x, min_z = thresholds(args.profile)
    passed = (
        count > 0
        and max_linear_x >= min_x
        and max_angular_z_abs >= min_z
        and not any_negative_x
        and not any_nonzero_y
    )

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "profile",
                "duration",
                "message_count",
                "max_linear_x",
                "mean_linear_x_abs",
                "max_angular_z_abs",
                "mean_angular_z_abs",
                "any_negative_x",
                "any_nonzero_y",
                "pass",
            ]
        )
        writer.writerow(
            [
                args.profile,
                f"{args.duration:.2f}",
                count,
                f"{max_linear_x:.4f}",
                f"{mean_linear_x_abs:.4f}",
                f"{max_angular_z_abs:.4f}",
                f"{mean_angular_z_abs:.4f}",
                str(any_negative_x).lower(),
                str(any_nonzero_y).lower(),
                str(passed).lower(),
            ]
        )

    print(f"profile={args.profile}")
    print(f"message_count={count}")
    print(f"max_linear_x={max_linear_x:.4f}")
    print(f"mean_linear_x_abs={mean_linear_x_abs:.4f}")
    print(f"max_angular_z_abs={max_angular_z_abs:.4f}")
    print(f"mean_angular_z_abs={mean_angular_z_abs:.4f}")
    print(f"any_negative_x={str(any_negative_x).lower()}")
    print(f"any_nonzero_y={str(any_nonzero_y).lower()}")
    print(f"output={args.output}")
    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
