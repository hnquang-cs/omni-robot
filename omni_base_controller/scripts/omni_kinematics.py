#!/usr/bin/env python3
"""Placeholder omni-directional controller node.

TODO:
- subscribe to /cmd_vel
- convert body twist to wheel targets
- publish wheel commands and/or /odom
- integrate encoder feedback or simulation feedback
"""

import rospy
from geometry_msgs.msg import Twist


def cmd_vel_callback(msg: Twist) -> None:
    rospy.loginfo_throttle(
        5.0,
        "[omni_kinematics] Placeholder received cmd_vel: "
        "vx=%.3f vy=%.3f wz=%.3f",
        msg.linear.x,
        msg.linear.y,
        msg.angular.z,
    )


def main() -> None:
    rospy.init_node("omni_kinematics")
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback, queue_size=10)
    rospy.loginfo("omni_kinematics skeleton started. No wheel control logic is implemented yet.")
    rospy.spin()


if __name__ == "__main__":
    main()
