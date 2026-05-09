#!/usr/bin/env python3
"""LiDAR-gated mapping driver for repeatable Stage 9 SLAM trials.

Conservative state machine: FORWARD -> STOP -> ROTATE_LEFT/RIGHT -> FORWARD.
Only fails with blocked_by_obstacle when truly stuck (global_min < critical for
too long, or rotate_attempts > max_rotate_attempts).
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from enum import Enum
from typing import List, Optional, Tuple

import rospy
import rospkg
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class State(Enum):
    IDLE = "IDLE"
    FORWARD = "FORWARD"
    ROTATE_LEFT = "ROTATE_LEFT"
    ROTATE_RIGHT = "ROTATE_RIGHT"
    STOP_ONLY = "STOP_ONLY"
    DONE = "DONE"


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SafeMappingDriver:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.scan_topic = rospy.get_param("~scan_topic", "/lidar/scan")
        self.cmd_topic = args.cmd_topic
        self.use_discrete_45 = args.use_discrete_45
        self.primitive_topic = args.primitive_topic
        self.primitive_state_topic = args.primitive_state_topic
        self.forward_speed = clamp(args.forward_speed, 0.0, 0.12)
        self.rotate_speed = abs(clamp(args.rotate_speed, 0.0, 0.25))
        self.safety_stop_distance = args.safety_stop_distance
        self.critical_stop_distance = args.critical_stop_distance
        self.front_angle_rad = math.radians(args.front_angle_deg)
        self.scan_timeout = args.scan_timeout
        self.max_rotate_segment = args.max_rotate_segment
        self.max_rotate_attempts = args.max_rotate_attempts
        self.blocked_timeout = args.blocked_timeout

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time: Optional[rospy.Time] = None
        self.state = State.IDLE
        self.state_started = time.monotonic()
        self.rotate_attempts = 0
        self.blocked_since: Optional[float] = None
        self.critical_since: Optional[float] = None
        self.exit_code = 0
        self.reason = "completed"
        self.shutdown_requested = False
        self.latest_primitive_state: Optional[str] = None
        self.latest_primitive_state_time: Optional[float] = None
        self.discrete_started = time.monotonic()
        self.discrete_fallback_warned = False

        self.log_dir = self._trial_dir(args.scenario, args.algorithm, args.rep)
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, "safe_driver_log.csv")
        self.status_path = os.path.join(self.log_dir, "safe_driver_status.txt")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            ["timestamp", "state", "front_min", "global_min", "cmd_x", "cmd_z", "event"]
        )
        self.csv_file.flush()

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.primitive_pub = rospy.Publisher(self.primitive_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1
        )
        self.primitive_state_sub = rospy.Subscriber(
            self.primitive_state_topic,
            String,
            self.primitive_state_callback,
            queue_size=10,
        )

    @staticmethod
    def _trial_dir(scenario: str, algorithm: str, rep: str) -> str:
        package_path = rospkg.RosPack().get_path("slam_benchmark")
        return os.path.join(
            package_path, "results", "stage9", "raw", scenario, algorithm, f"rep_{rep}"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_scan_time = rospy.Time.now()

    def primitive_state_callback(self, msg: String) -> None:
        self.latest_primitive_state = msg.data
        self.latest_primitive_state_time = time.monotonic()

    def run(self) -> int:
        rospy.loginfo(
            "[safe_mapping_driver] scenario=%s algorithm=%s rep=%s pattern=%s "
            "duration=%.1fs fwd=%.3f rot=%.3f safety=%.2f critical=%.2f cmd=%s discrete45=%s",
            self.args.scenario,
            self.args.algorithm,
            self.args.rep,
            self.args.pattern,
            self.args.duration,
            self.forward_speed,
            self.rotate_speed,
            self.safety_stop_distance,
            self.critical_stop_distance,
            self.cmd_topic,
            self.use_discrete_45,
        )
        start = time.monotonic()
        self.state = State.ROTATE_LEFT if self.args.pattern == "rotate_scan" else State.FORWARD
        self.state_started = start
        rate = rospy.Rate(10)

        # Wait for first scan
        initial_deadline = time.monotonic() + max(2.0, self.scan_timeout * 3.0)
        while (
            self.latest_scan is None
            and not rospy.is_shutdown()
            and time.monotonic() < initial_deadline
        ):
            self.publish(Twist())
            rate.sleep()
        if self.latest_scan is None:
            self.state = State.DONE
            self.reason = "scan_timeout"
            self.exit_code = 5

        while not rospy.is_shutdown() and not self.shutdown_requested:
            now = time.monotonic()
            if now - start >= self.args.duration:
                self.reason = "duration_complete"
                break

            front_min, global_min, left_min, right_min = self.scan_ranges()
            event = ""
            cmd = self.select_command(now, front_min, global_min, left_min, right_min)
            if self._last_event:
                event = self._last_event
                self._last_event = ""
            self.publish(cmd)
            self.log_sample(front_min, global_min, cmd, event)

            if self.state == State.DONE:
                break
            rate.sleep()

        self.stop_robot(repeats=8)
        self.write_status()
        self.csv_file.close()
        rospy.loginfo(
            "[safe_mapping_driver] finished reason=%s exit_code=%d",
            self.reason,
            self.exit_code,
        )
        return self.exit_code

    # Temporary event string set by select_command for the current tick
    _last_event: str = ""

    def select_command(
        self,
        now: float,
        front_min: float,
        global_min: float,
        left_min: float,
        right_min: float,
    ) -> Twist:
        self._last_event = ""
        cmd = Twist()

        # Stale scan
        if not self.scan_is_fresh():
            self.state = State.DONE
            self.reason = "scan_timeout"
            self.exit_code = 5
            self._last_event = "scan_timeout"
            return cmd

        # Track critical proximity — only fail if stuck there for a while
        side_critical = max(0.20, self.critical_stop_distance - 0.05)
        if global_min < side_critical:
            if self.critical_since is None:
                self.critical_since = now
                self._last_event = "critical_proximity_start"
            elif now - self.critical_since > 5.0:
                # Truly pinned — abort
                self.state = State.DONE
                self.reason = "collision_risk"
                self.exit_code = 3
                self._last_event = "collision_risk_abort"
                return cmd
        else:
            if self.critical_since is not None:
                self._last_event = "critical_proximity_cleared"
            self.critical_since = None

        # rotate_scan pattern — just spin in place
        if self.args.pattern == "rotate_scan":
            self.state = State.ROTATE_LEFT
            cmd.angular.z = self.rotate_speed
            return self.sanitize_cmd(cmd)

        # Side stop distance: slightly less aggressive than safety_stop
        side_stop_distance = max(0.30, self.critical_stop_distance + 0.10)

        if front_min >= self.safety_stop_distance and global_min >= side_stop_distance:
            # Clear path — drive forward
            if self.state in (State.ROTATE_LEFT, State.ROTATE_RIGHT):
                if now - self.state_started < 1.0:
                    # Finish the last rotation segment briefly before going forward
                    cmd.angular.z = (
                        self.rotate_speed if self.state == State.ROTATE_LEFT else -self.rotate_speed
                    )
                    return self.sanitize_cmd(cmd)
                else:
                    self._last_event = "rotation_done_go_forward"
            self.blocked_since = None
            self.rotate_attempts = 0
            self.set_state(State.FORWARD, now)
            cmd.linear.x = self.forward_speed
            return self.sanitize_cmd(cmd)

        # Obstacle within safety zone — need to rotate
        if self.blocked_since is None:
            self.blocked_since = now
            self._last_event = "obstacle_detected"

        # Decide rotation direction: away from the nearer wall
        prefer_left = right_min < left_min

        # Check global blocked timeout
        if now - self.blocked_since > self.blocked_timeout:
            if self.args.pattern == "wide_obstacles_safe":
                self.rotate_attempts += 1
                if self.rotate_attempts > self.max_rotate_attempts:
                    self.state = State.DONE
                    self.reason = "blocked_by_obstacle"
                    self.exit_code = 4
                    self._last_event = "max_rotate_attempts_exceeded_blocked_timeout"
                    rospy.logwarn(
                        "[safe_mapping_driver] giving up after %d wide-obstacles recovery attempts",
                        self.rotate_attempts - 1,
                    )
                    return Twist()
                new_state = (
                    State.ROTATE_RIGHT if self.state == State.ROTATE_LEFT else State.ROTATE_LEFT
                )
                if self.state not in (State.ROTATE_LEFT, State.ROTATE_RIGHT):
                    new_state = State.ROTATE_LEFT if prefer_left else State.ROTATE_RIGHT
                self.set_state(new_state, now)
                self.blocked_since = now
                self._last_event = f"wide_obstacles_recovery_rotate_{new_state.value.lower()}_attempt_{self.rotate_attempts}"
                rospy.loginfo(
                    "[safe_mapping_driver] wide_obstacles recovery attempt=%d state=%s front=%.2f global=%.2f",
                    self.rotate_attempts,
                    new_state.value,
                    front_min,
                    global_min,
                )
                cmd.angular.z = (
                    self.rotate_speed if self.state == State.ROTATE_LEFT else -self.rotate_speed
                )
                return self.sanitize_cmd(cmd)
            if self.state in (State.ROTATE_LEFT, State.ROTATE_RIGHT):
                # Still rotating but cannot clear — give up
                self.state = State.DONE
                self.reason = "blocked_by_obstacle"
                self.exit_code = 4
                self._last_event = "blocked_timeout_abort"
                rospy.logwarn(
                    "[safe_mapping_driver] blocked_timeout_abort after %.1fs",
                    now - self.blocked_since,
                )
                return Twist()

        if self.state not in (State.ROTATE_LEFT, State.ROTATE_RIGHT):
            # Start rotating
            self.rotate_attempts += 1
            if self.rotate_attempts > self.max_rotate_attempts:
                self.state = State.DONE
                self.reason = "blocked_by_obstacle"
                self.exit_code = 4
                self._last_event = f"max_rotate_attempts_{self.max_rotate_attempts}_exceeded"
                rospy.logwarn(
                    "[safe_mapping_driver] giving up after %d rotation attempts",
                    self.rotate_attempts - 1,
                )
                return Twist()
            new_state = State.ROTATE_LEFT if prefer_left else State.ROTATE_RIGHT
            self.set_state(new_state, now)
            self._last_event = f"start_rotate_{new_state.value.lower()}_attempt_{self.rotate_attempts}"
            rospy.loginfo(
                "[safe_mapping_driver] %s attempt=%d front=%.2f global=%.2f",
                new_state.value,
                self.rotate_attempts,
                front_min,
                global_min,
            )
        elif now - self.state_started > self.max_rotate_segment:
            # Current rotation segment expired — switch direction or give up
            self.rotate_attempts += 1
            if self.rotate_attempts > self.max_rotate_attempts:
                self.state = State.DONE
                self.reason = "blocked_by_obstacle"
                self.exit_code = 4
                self._last_event = "max_rotate_attempts_exceeded_mid_rotation"
                return Twist()
            # Switch direction
            new_state = (
                State.ROTATE_RIGHT if self.state == State.ROTATE_LEFT else State.ROTATE_LEFT
            )
            self.set_state(new_state, now)
            self._last_event = f"switch_rotate_{new_state.value.lower()}_attempt_{self.rotate_attempts}"

        cmd.angular.z = (
            self.rotate_speed if self.state == State.ROTATE_LEFT else -self.rotate_speed
        )
        return self.sanitize_cmd(cmd)

    def set_state(self, state: State, now: float) -> None:
        if self.state != state:
            self.state = state
            self.state_started = now

    def scan_is_fresh(self) -> bool:
        if self.latest_scan is None or self.latest_scan_time is None:
            return False
        return (rospy.Time.now() - self.latest_scan_time).to_sec() <= self.scan_timeout

    def scan_ranges(self) -> Tuple[float, float, float, float]:
        scan = self.latest_scan
        if scan is None:
            return float("inf"), float("inf"), float("inf"), float("inf")

        front: List[float] = []
        left: List[float] = []
        right: List[float] = []
        all_ranges: List[float] = []
        angle = scan.angle_min
        for value in scan.ranges:
            if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                all_ranges.append(value)
                if abs(angle) <= self.front_angle_rad:
                    front.append(value)
                if math.radians(25.0) <= angle <= math.radians(95.0):
                    left.append(value)
                if math.radians(-95.0) <= angle <= math.radians(-25.0):
                    right.append(value)
            angle += scan.angle_increment

        return (
            min(front) if front else float("inf"),
            min(all_ranges) if all_ranges else float("inf"),
            min(left) if left else float("inf"),
            min(right) if right else float("inf"),
        )

    def sanitize_cmd(self, cmd: Twist) -> Twist:
        out = Twist()
        out.linear.x = clamp(cmd.linear.x, 0.0, self.forward_speed)
        out.linear.y = 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(cmd.angular.z, -self.rotate_speed, self.rotate_speed)
        return out

    def publish(self, cmd: Twist) -> None:
        if self.use_discrete_45 and self.discrete_state_available():
            primitive = self.twist_to_primitive(cmd)
            try:
                self.primitive_pub.publish(String(data=primitive))
            except rospy.ROSException:
                pass
            return

        if self.use_discrete_45 and not self.discrete_fallback_warned:
            if time.monotonic() - self.discrete_started > 2.0:
                rospy.logwarn(
                    "[safe_mapping_driver] --use-discrete-45 requested but %s has no messages; "
                    "falling back to Twist output on %s",
                    self.primitive_state_topic,
                    self.cmd_topic,
                )
                self.discrete_fallback_warned = True

        try:
            self.pub.publish(self.sanitize_cmd(cmd))
        except rospy.ROSException:
            pass

    def discrete_state_available(self) -> bool:
        return (
            self.latest_primitive_state_time is not None
            and (time.monotonic() - self.latest_primitive_state_time) <= 2.0
        )

    @staticmethod
    def twist_to_primitive(cmd: Twist) -> str:
        if cmd.linear.x > 1e-4 and abs(cmd.angular.z) <= 1e-4:
            return "FORWARD"
        if cmd.angular.z > 1e-4:
            return "TURN_LEFT_45"
        if cmd.angular.z < -1e-4:
            return "TURN_RIGHT_45"
        return "STOP"

    def stop_robot(self, repeats: int = 5) -> None:
        zero = Twist()
        for _ in range(repeats):
            try:
                if self.use_discrete_45 and self.discrete_state_available():
                    self.primitive_pub.publish(String(data="STOP"))
                else:
                    self.pub.publish(zero)
                rospy.sleep(0.05)
            except rospy.ROSException:
                break

    def log_sample(self, front_min: float, global_min: float, cmd: Twist, event: str = "") -> None:
        self.csv_writer.writerow(
            [
                f"{rospy.Time.now().to_sec():.6f}",
                self.state.value,
                self.format_range(front_min),
                self.format_range(global_min),
                f"{cmd.linear.x:.4f}",
                f"{cmd.angular.z:.4f}",
                event,
            ]
        )
        self.csv_file.flush()

    @staticmethod
    def format_range(value: float) -> str:
        return "inf" if math.isinf(value) else f"{value:.4f}"

    def write_status(self) -> None:
        with open(self.status_path, "w") as status_file:
            status_file.write(f"result={'PASS' if self.exit_code == 0 else 'FAIL'}\n")
            status_file.write(f"reason={self.reason}\n")
            status_file.write(f"exit_code={self.exit_code}\n")
            status_file.write(f"csv={self.csv_path}\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Safe LiDAR mapping driver for Stage 9.")
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--algorithm", required=True)
    parser.add_argument("--rep", required=True)
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument(
        "--pattern",
        choices=["corridor_safe", "rotate_scan", "explore_safe", "wide_obstacles_safe"],
        default="corridor_safe",
    )
    parser.add_argument("--front-angle-deg", type=float, default=20.0)
    parser.add_argument("--forward-speed", type=float, default=0.06)
    parser.add_argument("--rotate-speed", type=float, default=0.12)
    parser.add_argument("--safety-stop-distance", type=float, default=0.80)
    parser.add_argument("--critical-stop-distance", type=float, default=0.30)
    parser.add_argument("--cmd-topic", default="/cmd_vel")
    parser.add_argument("--use-discrete-45", action="store_true")
    parser.add_argument("--primitive-topic", default="/motion_primitive_cmd")
    parser.add_argument("--primitive-state-topic", default="/motion_primitive_state")
    parser.add_argument("--scan-timeout", type=float, default=1.0)
    parser.add_argument("--max-rotate-segment", type=float, default=5.0)
    parser.add_argument("--max-rotate-attempts", type=int, default=6)
    parser.add_argument("--blocked-timeout", type=float, default=30.0)
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def main() -> None:
    args = parse_args()
    rospy.init_node("safe_mapping_driver")
    driver = SafeMappingDriver(args)

    def request_shutdown(_signum, _frame) -> None:
        driver.shutdown_requested = True
        driver.reason = "interrupted"
        driver.stop_robot(repeats=8)

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    rospy.on_shutdown(lambda: driver.stop_robot(repeats=8))
    sys.exit(driver.run())


if __name__ == "__main__":
    main()
