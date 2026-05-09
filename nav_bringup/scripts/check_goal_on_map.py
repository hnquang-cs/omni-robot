#!/usr/bin/env python3
"""Check whether a world-frame navigation goal lies in free map space."""

import argparse
import math
import sys

import rospy
from nav_msgs.msg import OccupancyGrid


def world_to_cell(map_info, x, y):
    mx = int(math.floor((x - map_info.origin.position.x) / map_info.resolution))
    my = int(math.floor((y - map_info.origin.position.y) / map_info.resolution))
    return mx, my


def classify(value):
    if value < 0:
        return "UNKNOWN"
    if value == 0:
        return "FREE"
    return "OCCUPIED"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--topic", default="/map")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rospy.init_node("check_goal_on_map", anonymous=True)
    try:
        grid = rospy.wait_for_message(args.topic, OccupancyGrid, timeout=args.timeout)
    except rospy.ROSException as exc:
        print(f"FAIL: no OccupancyGrid received on {args.topic}: {exc}")
        return 1

    info = grid.info
    origin = info.origin.position
    mx, my = world_to_cell(info, args.x, args.y)

    print(f"map width: {info.width}")
    print(f"map height: {info.height}")
    print(f"resolution: {info.resolution:.6f}")
    print(f"origin: [{origin.x:.6f}, {origin.y:.6f}, {info.origin.orientation.z:.6f}]")
    print(f"goal world: x={args.x:.6f}, y={args.y:.6f}, yaw={args.yaw:.6f}")
    print(f"goal cell index: x={mx}, y={my}")

    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
        print("cell value: N/A")
        print("status: OUT_OF_BOUNDS")
        return 1

    index = my * info.width + mx
    value = int(grid.data[index])
    status = classify(value)
    print(f"linear index: {index}")
    print(f"cell value: {value}")
    print(f"status: {status}")
    return 0 if status == "FREE" else 1


if __name__ == "__main__":
    sys.exit(main())
