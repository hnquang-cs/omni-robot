#!/usr/bin/env python3
"""Clamp planner velocity commands to forward-only differential-style motion."""

import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist


class CmdVelForwardOnlyFilter:
    def __init__(self):
        self.cmd_vel_in = rospy.get_param("~cmd_vel_in", "/cmd_vel_raw")
        self.cmd_vel_out = rospy.get_param("~cmd_vel_out", "/cmd_vel")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.max_vel_x = abs(float(rospy.get_param("~max_vel_x", 0.45)))
        self.max_vel_theta = abs(float(rospy.get_param("~max_vel_theta", 0.30)))
        self.max_acc_x = abs(float(rospy.get_param("~max_acc_x", 0.60)))
        self.max_acc_theta = abs(float(rospy.get_param("~max_acc_theta", 0.35)))
        self.force_no_reverse = bool(rospy.get_param("~force_no_reverse", True))
        self.force_zero_y = bool(rospy.get_param("~force_zero_y", True))

        self.lock = threading.RLock()
        self.target = Twist()
        self.current = Twist()
        self.last_msg_time = None
        self.last_update_time = None

        self.pub = rospy.Publisher(self.cmd_vel_out, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.cmd_vel_in, Twist, self.callback, queue_size=10)
        self.worker = threading.Thread(target=self.publish_loop, daemon=True)
        self.worker.start()

        rospy.loginfo(
            "[cmd_vel_forward_only_filter] %s -> %s max_x=%.3f max_theta=%.3f",
            self.cmd_vel_in,
            self.cmd_vel_out,
            self.max_vel_x,
            self.max_vel_theta,
        )

    def callback(self, msg):
        with self.lock:
            self.target = self.sanitize(msg)
            self.last_msg_time = time.monotonic()

    def sanitize(self, msg):
        out = Twist()

        if self.force_no_reverse and msg.linear.x < 0.0:
            rospy.logwarn_throttle(
                1.0,
                "[cmd_vel_forward_only_filter] received negative linear.x, clamped to zero",
            )
            vx = 0.0
        else:
            vx = msg.linear.x
        out.linear.x = max(0.0, min(self.max_vel_x, vx))

        if self.force_zero_y and abs(msg.linear.y) > 1e-6:
            rospy.logwarn_throttle(
                1.0,
                "[cmd_vel_forward_only_filter] received non-zero linear.y, clamped to zero",
            )
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, msg.angular.z))
        return out

    def publish_loop(self):
        period = 1.0 / self.publish_rate
        while not rospy.is_shutdown():
            self.update_once(time.monotonic())
            time.sleep(period)

    def update_once(self, now):
        with self.lock:
            if self.last_update_time is None:
                self.last_update_time = now
                return

            dt = now - self.last_update_time
            self.last_update_time = now
            if dt <= 0.0 or dt > 1.0:
                return

            target = self.target
            if self.last_msg_time is None or (now - self.last_msg_time) > self.cmd_timeout:
                target = Twist()

            out = Twist()
            out.linear.x = self.ramp(self.current.linear.x, target.linear.x, self.max_acc_x, dt)
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.x = 0.0
            out.angular.y = 0.0
            out.angular.z = self.ramp(
                self.current.angular.z,
                target.angular.z,
                self.max_acc_theta,
                dt,
            )
            out.linear.x = max(0.0, min(self.max_vel_x, out.linear.x))
            out.angular.z = max(-self.max_vel_theta, min(self.max_vel_theta, out.angular.z))
            self.current = out

        try:
            self.pub.publish(out)
        except rospy.ROSException:
            pass

    @staticmethod
    def ramp(current, target, max_acc, dt):
        delta = target - current
        step = max_acc * dt
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)


def main():
    rospy.init_node("cmd_vel_forward_only_filter")
    CmdVelForwardOnlyFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
