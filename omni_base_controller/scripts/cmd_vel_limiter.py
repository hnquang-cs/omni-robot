#!/usr/bin/env python3
"""Rate-limit move_base Twist commands before they reach the Gazebo controller."""

import math
import threading

import rospy
from geometry_msgs.msg import Twist


class CmdVelLimiter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_raw")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))

        self.max_vx = abs(float(rospy.get_param("~max_vx", 0.25)))
        self.max_vy = abs(float(rospy.get_param("~max_vy", 0.20)))
        self.max_wz = abs(float(rospy.get_param("~max_wz", 0.35)))
        self.max_acc_x = abs(float(rospy.get_param("~max_acc_x", 0.30)))
        self.max_acc_y = abs(float(rospy.get_param("~max_acc_y", 0.30)))
        self.max_acc_wz = abs(float(rospy.get_param("~max_acc_wz", 0.45)))

        self.lock = threading.RLock()
        self.target = Twist()
        self.current = Twist()
        self.last_msg_time = rospy.Time(0)
        self.last_update_time = None

        self.pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Twist, self.callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.update)

        rospy.loginfo(
            "[cmd_vel_limiter] %s -> %s, max=(%.2f, %.2f, %.2f), acc=(%.2f, %.2f, %.2f)",
            self.input_topic,
            self.output_topic,
            self.max_vx,
            self.max_vy,
            self.max_wz,
            self.max_acc_x,
            self.max_acc_y,
            self.max_acc_wz,
        )

    def callback(self, msg):
        with self.lock:
            self.target = self.clamp(msg)
            self.last_msg_time = rospy.Time.now()

    def update(self, event):
        now = rospy.Time.now()
        if now == rospy.Time(0):
            return

        with self.lock:
            if self.last_update_time is None:
                self.last_update_time = now
                return
            dt = (now - self.last_update_time).to_sec()
            self.last_update_time = now
            if dt <= 0.0 or dt > 1.0:
                return

            target = self.target
            if self.last_msg_time == rospy.Time(0) or (now - self.last_msg_time).to_sec() > self.cmd_timeout:
                target = Twist()

            out = Twist()
            out.linear.x = self.ramp(self.current.linear.x, target.linear.x, self.max_acc_x, dt)
            out.linear.y = self.ramp(self.current.linear.y, target.linear.y, self.max_acc_y, dt)
            out.angular.z = self.ramp(self.current.angular.z, target.angular.z, self.max_acc_wz, dt)
            self.current = out

        self.pub.publish(out)

    def clamp(self, msg):
        out = Twist()
        out.linear.x = max(-self.max_vx, min(self.max_vx, msg.linear.x))
        out.linear.y = max(-self.max_vy, min(self.max_vy, msg.linear.y))
        out.angular.z = max(-self.max_wz, min(self.max_wz, msg.angular.z))
        return out

    @staticmethod
    def ramp(current, target, max_acc, dt):
        delta = target - current
        step = max_acc * dt
        if abs(delta) <= step:
            return target
        return current + math.copysign(step, delta)


def main():
    rospy.init_node("cmd_vel_limiter")
    CmdVelLimiter()
    rospy.spin()


if __name__ == "__main__":
    main()
