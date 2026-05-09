#!/usr/bin/env python3
"""Check whether /cmd_vel carries non-zero commands during a short window."""

import argparse
import csv
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist


class CmdVelActivity:
    def __init__(self, topic: str) -> None:
        self.topic = topic
        self.message_count = 0
        self.max_linear_x = 0.0
        self.max_abs_angular_z = 0.0
        self.sub = rospy.Subscriber(topic, Twist, self.callback, queue_size=50)

    def callback(self, msg: Twist) -> None:
        self.message_count += 1
        self.max_linear_x = max(self.max_linear_x, msg.linear.x)
        self.max_abs_angular_z = max(self.max_abs_angular_z, abs(msg.angular.z))

    @property
    def active(self) -> bool:
        return self.message_count > 0 and (self.max_linear_x > 0.01 or self.max_abs_angular_z > 0.01)


def write_csv(path: Path, checker: CmdVelActivity) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["message_count", "max_linear_x", "max_abs_angular_z", "active"])
        writer.writerow([
            checker.message_count,
            f"{checker.max_linear_x:.6f}",
            f"{checker.max_abs_angular_z:.6f}",
            str(checker.active).lower(),
        ])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/cmd_vel")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--csv", default="")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("check_cmd_vel_activity", anonymous=True)
    checker = CmdVelActivity(args.topic)
    end_time = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < end_time:
        time.sleep(0.05)

    if args.csv:
        write_csv(Path(args.csv), checker)

    active = "true" if checker.active else "false"
    print(
        "message_count,max_linear_x,max_abs_angular_z,active\n"
        f"{checker.message_count},{checker.max_linear_x:.6f},{checker.max_abs_angular_z:.6f},{active}"
    )
    return 0 if checker.active else 1


if __name__ == "__main__":
    sys.exit(main())
