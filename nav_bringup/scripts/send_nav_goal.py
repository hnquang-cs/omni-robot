#!/usr/bin/env python3
"""Send a simple RViz-style navigation goal to move_base."""

import argparse
import math

import rospy
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--frame", default="map")
    args = parser.parse_args()

    rospy.init_node("send_nav_goal", anonymous=True)
    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(0.5)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = args.frame
    msg.pose.position.x = args.x
    msg.pose.position.y = args.y
    msg.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(args.yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    pub.publish(msg)
    rospy.loginfo("Sent goal frame=%s x=%.3f y=%.3f yaw=%.3f", args.frame, args.x, args.y, args.yaw)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
