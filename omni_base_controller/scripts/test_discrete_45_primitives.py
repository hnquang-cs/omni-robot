#!/usr/bin/env python3
"""Acceptance test for discrete 45-degree motion primitives.

Run while Gazebo, /odom, gazebo_model_cmd_vel, and discrete_45_motion_controller
are already running:

    roslaunch omni_base_controller discrete_45_demo.launch
    rosrun  omni_base_controller test_discrete_45_primitives.py
"""

import csv
import math
import os
import sys
import time
from typing import Dict, List, Optional

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class PrimitiveTester:
    def __init__(self) -> None:
        self.primitive_topic = rospy.get_param("~primitive_topic", "/motion_primitive_cmd")
        self.state_topic = rospy.get_param("~state_topic", "/motion_primitive_state")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.turn_tolerance_deg = float(rospy.get_param("~turn_tolerance_deg", 3.0))
        self.turn_timeout = float(rospy.get_param("~turn_timeout_sec", 12.0))
        self.odom_wait_sec = float(rospy.get_param("~odom_wait_sec", 10.0))
        self.results_path = rospy.get_param(
            "~results_path",
            os.path.expanduser(
                "~/catkin_ws/src/omni-robot/omni_base_controller/results/discrete_45_test_result.csv"
            ),
        )

        self.yaw: Optional[float] = None
        self.state = ""
        self.cmd_samples: List[Twist] = []

        self.pub = rospy.Publisher(self.primitive_topic, String, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.state_sub = rospy.Subscriber(self.state_topic, String, self.state_callback, queue_size=10)
        self.cmd_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_callback, queue_size=100)

    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        self.yaw = normalize_angle(yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def state_callback(self, msg: String) -> None:
        self.state = msg.data

    def cmd_callback(self, msg: Twist) -> None:
        self.cmd_samples.append(msg)
        if len(self.cmd_samples) > 2000:
            self.cmd_samples = self.cmd_samples[-2000:]

    def wait_for_yaw(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.yaw is not None:
                return True
            rospy.sleep(0.1)
        return False

    def publish_primitive(self, primitive: str, repeats: int = 3) -> None:
        for _ in range(repeats):
            self.pub.publish(String(data=primitive))
            rospy.sleep(0.1)

    def wait_turn_done(self, start_yaw: float, expected_delta: float) -> Optional[float]:
        deadline = time.monotonic() + self.turn_timeout
        target = normalize_angle(start_yaw + expected_delta)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.yaw is None:
                rospy.sleep(0.05)
                continue
            error = abs(normalize_angle(target - self.yaw))
            if self.state == "STOPPED" and error <= math.radians(self.turn_tolerance_deg):
                return math.degrees(normalize_angle(self.yaw - start_yaw))
            rospy.sleep(0.05)
        if self.yaw is not None:
            return math.degrees(normalize_angle(self.yaw - start_yaw))
        return None

    def sample_cmd_vel(self, duration: float) -> List[Twist]:
        self.cmd_samples = []
        deadline = time.monotonic() + duration
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            rospy.sleep(0.05)
        return list(self.cmd_samples)

    @staticmethod
    def cmd_invariants(samples: List[Twist]) -> Dict[str, bool]:
        active = [s for s in samples if abs(s.linear.x) > 1e-5 or abs(s.linear.y) > 1e-5 or abs(s.angular.z) > 1e-5]
        return {
            "no_lateral": all(abs(s.linear.y) <= 1e-5 for s in samples),
            "no_reverse": all(s.linear.x >= -1e-5 for s in samples),
            "no_drive_and_turn": all(
                not (abs(s.linear.x) > 1e-5 and abs(s.angular.z) > 1e-5)
                for s in samples
            ),
            "has_forward": any(s.linear.x > 0.01 for s in active),
            "forward_no_turn": all(abs(s.angular.z) <= 1e-5 for s in active if s.linear.x > 0.01),
        }

    def run(self) -> int:
        results: Dict[str, bool] = {}
        left_delta: Optional[float] = None
        right_delta: Optional[float] = None

        if not self.wait_for_yaw(self.odom_wait_sec):
            print("FAIL: odom yaw not available within %.1fs" % self.odom_wait_sec)
            self._write_csv(results, left_delta, right_delta, odom_ok=False)
            return 1

        rospy.sleep(0.5)
        # ---------- STOP ----------
        self.publish_primitive("STOP")
        rospy.sleep(0.5)
        stop_samples = self.sample_cmd_vel(0.5)
        results["STOP"] = all(
            abs(s.linear.x) <= 1e-5 and abs(s.linear.y) <= 1e-5 and abs(s.angular.z) <= 1e-5
            for s in stop_samples
        )

        # ---------- TURN_LEFT_45 ----------
        start = self.yaw
        self.publish_primitive("TURN_LEFT_45")
        left_samples = self.sample_cmd_vel(0.4)
        left_delta = self.wait_turn_done(start, math.radians(45.0))
        results["TURN_LEFT_45"] = (
            left_delta is not None and abs(left_delta - 45.0) <= self.turn_tolerance_deg
        )

        rospy.sleep(0.4)

        # ---------- TURN_RIGHT_45 ----------
        start = self.yaw
        self.publish_primitive("TURN_RIGHT_45")
        right_samples = self.sample_cmd_vel(0.4)
        right_delta = self.wait_turn_done(start, math.radians(-45.0))
        results["TURN_RIGHT_45"] = (
            right_delta is not None and abs(right_delta + 45.0) <= self.turn_tolerance_deg
        )

        rospy.sleep(0.4)

        # ---------- FORWARD ----------
        self.publish_primitive("FORWARD")
        forward_samples = self.sample_cmd_vel(3.0)
        inv = self.cmd_invariants(forward_samples)
        results["FORWARD"] = inv["has_forward"] and inv["forward_no_turn"]

        # ---------- STOP again ----------
        self.publish_primitive("STOP")
        rospy.sleep(0.5)

        all_motion_samples = left_samples + right_samples + forward_samples + stop_samples + list(self.cmd_samples)
        inv_all = self.cmd_invariants(all_motion_samples)
        results["no_lateral"] = inv_all["no_lateral"]
        results["no_reverse"] = inv_all["no_reverse"]
        results["no_drive_and_turn_simultaneous"] = inv_all["no_drive_and_turn"]

        for key in [
            "STOP",
            "TURN_LEFT_45",
            "TURN_RIGHT_45",
            "FORWARD",
            "no_lateral",
            "no_reverse",
            "no_drive_and_turn_simultaneous",
        ]:
            print(f"{'PASS' if results.get(key, False) else 'FAIL'}: {key}")

        print(f"left_turn_delta_deg={left_delta if left_delta is not None else 'N/A'}")
        print(f"right_turn_delta_deg={right_delta if right_delta is not None else 'N/A'}")

        ok = all(results.values())
        print(f"OVERALL: {'PASS' if ok else 'FAIL'}")
        self._write_csv(results, left_delta, right_delta, odom_ok=True)
        return 0 if ok else 1

    def _write_csv(
        self,
        results: Dict[str, bool],
        left_delta: Optional[float],
        right_delta: Optional[float],
        odom_ok: bool,
    ) -> None:
        try:
            os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
            with open(self.results_path, "w", newline="") as fp:
                writer = csv.writer(fp)
                writer.writerow(["check", "result", "value"])
                writer.writerow(["odom_available", "PASS" if odom_ok else "FAIL", ""])
                for key in [
                    "STOP",
                    "TURN_LEFT_45",
                    "TURN_RIGHT_45",
                    "FORWARD",
                    "no_lateral",
                    "no_reverse",
                    "no_drive_and_turn_simultaneous",
                ]:
                    writer.writerow([
                        key,
                        "PASS" if results.get(key, False) else "FAIL",
                        "",
                    ])
                writer.writerow(["left_turn_delta_deg", "INFO", f"{left_delta if left_delta is not None else 'N/A'}"])
                writer.writerow(["right_turn_delta_deg", "INFO", f"{right_delta if right_delta is not None else 'N/A'}"])
            print(f"Wrote results to {self.results_path}")
        except OSError as exc:
            print(f"WARN: could not write CSV: {exc}")


def main() -> None:
    rospy.init_node("test_discrete_45_primitives", anonymous=True)
    sys.exit(PrimitiveTester().run())


if __name__ == "__main__":
    main()
