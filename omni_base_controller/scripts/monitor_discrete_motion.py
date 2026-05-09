#!/usr/bin/env python3
"""Live monitor for discrete 45-degree motion safety invariants."""

import csv
import math
from pathlib import Path
from typing import Optional

import rospy
import rospkg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf import transformations


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DiscreteMotionMonitor:
    def __init__(self) -> None:
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.state_topic = rospy.get_param("~state_topic", "/motion_primitive_state")
        self.publish_rate = float(rospy.get_param("~publish_rate", 5.0))
        self.max_turn_delta_deg = float(rospy.get_param("~max_turn_delta_deg", 48.0))

        package_path = Path(rospkg.RosPack().get_path("omni_base_controller"))
        self.csv_path = package_path / "results" / "discrete_45_motion_log.csv"
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.csv_path.open("w", newline="")
        self.closed = False
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["time", "yaw_deg", "linear_x", "linear_y", "angular_z", "state", "warning"])

        self.yaw: Optional[float] = None
        self.cmd = Twist()
        self.state = "UNKNOWN"
        self.turn_start_yaw: Optional[float] = None

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.cmd_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_callback, queue_size=10)
        self.state_sub = rospy.Subscriber(self.state_topic, String, self.state_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.report)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("[monitor_discrete_motion] writing %s", self.csv_path)

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        _roll, _pitch, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw = normalize_angle(yaw)

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd = msg

    def state_callback(self, msg: String) -> None:
        previous = self.state
        self.state = msg.data
        if self.state.startswith("TURNING") and not previous.startswith("TURNING"):
            self.turn_start_yaw = self.yaw
        if not self.state.startswith("TURNING"):
            self.turn_start_yaw = None

    def report(self, _event) -> None:
        if self.closed:
            return
        yaw_deg = math.degrees(self.yaw) if self.yaw is not None else float("nan")
        warnings = []
        if abs(self.cmd.linear.y) > 1e-5:
            warnings.append("linear_y_nonzero")
        if self.cmd.linear.x < -1e-5:
            warnings.append("linear_x_negative")
        if abs(self.cmd.linear.x) > 1e-5 and abs(self.cmd.angular.z) > 1e-5:
            warnings.append("drive_and_turn_simultaneous")
        if self.turn_start_yaw is not None and self.yaw is not None:
            delta = abs(math.degrees(normalize_angle(self.yaw - self.turn_start_yaw)))
            if delta > self.max_turn_delta_deg:
                warnings.append(f"turn_delta_gt_{self.max_turn_delta_deg:.1f}")

        warning_text = ";".join(warnings)
        print(
            "yaw_deg={:.2f} linear.x={:.4f} linear.y={:.4f} angular.z={:.4f} state={}{}".format(
                yaw_deg,
                self.cmd.linear.x,
                self.cmd.linear.y,
                self.cmd.angular.z,
                self.state,
                f" WARN={warning_text}" if warning_text else "",
            )
        )
        self.writer.writerow(
            [
                f"{rospy.Time.now().to_sec():.6f}",
                f"{yaw_deg:.6f}",
                f"{self.cmd.linear.x:.6f}",
                f"{self.cmd.linear.y:.6f}",
                f"{self.cmd.angular.z:.6f}",
                self.state,
                warning_text,
            ]
        )
        self.csv_file.flush()

    def shutdown(self) -> None:
        self.closed = True
        self.csv_file.close()


def main() -> None:
    rospy.init_node("monitor_discrete_motion")
    DiscreteMotionMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
