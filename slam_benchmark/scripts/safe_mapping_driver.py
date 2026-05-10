#!/usr/bin/env python3
"""LiDAR-gated mapping driver for repeatable Stage 9 SLAM trials.

Stage 9 completion is decided by fixed duration/coverage/safety/stuck
conditions. A blocked front sector is a recovery condition, not a completion
condition: the robot stops briefly, rotates/searches, escapes if needed, and
then tries forward motion again.
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
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class State(Enum):
    IDLE = "IDLE"
    FORWARD = "FORWARD"
    SLOW_FORWARD = "SLOW_FORWARD"
    STOP_AND_SCAN = "STOP_AND_SCAN"
    ROTATE_SEARCH_LEFT = "ROTATE_SEARCH_LEFT"
    ROTATE_SEARCH_RIGHT = "ROTATE_SEARCH_RIGHT"
    ESCAPE_ROTATE = "ESCAPE_ROTATE"
    DONE = "DONE"
    ERROR = "ERROR"


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
        self.forward_speed = clamp(args.forward_speed, 0.0, 0.45)
        self.rotate_speed = abs(clamp(args.rotate_speed, 0.0, 0.45))
        self.safety_stop_distance = args.safety_stop_distance
        self.min_mapping_duration_sec = 120.0
        self.max_duration_sec = 600.0
        self.explored_ratio_threshold = 0.30
        self.unknown_ratio_threshold = 0.50
        self.critical_stop_distance = 0.30
        self.collision_risk_duration_sec = 2.0
        self.max_stuck_count = 12
        self.front_angle_rad = math.radians(args.front_angle_deg)
        self.scan_timeout = args.scan_timeout
        self.max_rotate_segment = args.max_rotate_segment
        self.blocked_timeout = args.blocked_timeout
        self.stop_and_scan_duration = 0.5
        self.escape_rotate_duration = max(2.0, min(6.0, args.max_rotate_segment))

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time: Optional[rospy.Time] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.explored_ratio: Optional[float] = None
        self.unknown_ratio: Optional[float] = None
        self.coverage_available = False
        self.state = State.IDLE
        self.state_started = time.monotonic()
        self.rotate_attempts = 0
        self.stuck_count = 0
        self.blocked_since: Optional[float] = None
        self.critical_since: Optional[float] = None
        self.exit_code = 0
        self.stop_reason = ""
        self.reason = ""
        self.shutdown_requested = False
        self.start_time = time.monotonic()
        self.latest_primitive_state: Optional[str] = None
        self.latest_primitive_state_time: Optional[float] = None
        self.discrete_started = time.monotonic()
        self.discrete_fallback_warned = False

        self.log_dir = self._trial_dir(args.scenario, args.algorithm, args.rep)
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, "safe_driver_log.csv")
        self.status_path = os.path.join(self.log_dir, "safe_driver_status.txt")
        if os.path.exists(self.status_path):
            os.remove(self.status_path)
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(
            [
                "timestamp",
                "elapsed",
                "state",
                "event",
                "front_min",
                "global_min",
                "cmd_x",
                "cmd_z",
                "stuck_count",
                "explored_ratio",
                "unknown_ratio",
                "stop_reason",
            ]
        )
        self.csv_file.flush()

        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.primitive_pub = rospy.Publisher(self.primitive_topic, String, queue_size=10)
        self.sub = rospy.Subscriber(
            self.scan_topic, LaserScan, self.scan_callback, queue_size=1
        )
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
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

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        total = len(msg.data)
        if total <= 0:
            self.coverage_available = False
            self.explored_ratio = None
            self.unknown_ratio = None
            return
        unknown = sum(1 for cell in msg.data if cell < 0)
        explored = total - unknown
        self.explored_ratio = explored / float(total)
        self.unknown_ratio = unknown / float(total)
        self.coverage_available = True

    def primitive_state_callback(self, msg: String) -> None:
        self.latest_primitive_state = msg.data
        self.latest_primitive_state_time = time.monotonic()

    def run(self) -> int:
        rospy.loginfo(
            "[safe_mapping_driver] scenario=%s algorithm=%s rep=%s pattern=%s "
            "min_duration=%.1fs max_duration=%.1fs fwd=%.3f rot=%.3f safety=%.2f "
            "critical=%.2f cmd=%s discrete45=%s",
            self.args.scenario,
            self.args.algorithm,
            self.args.rep,
            self.args.pattern,
            self.min_mapping_duration_sec,
            self.max_duration_sec,
            self.forward_speed,
            self.rotate_speed,
            self.safety_stop_distance,
            self.critical_stop_distance,
            self.cmd_topic,
            self.use_discrete_45,
        )
        self.start_time = time.monotonic()
        self.state = State.ROTATE_SEARCH_LEFT if self.args.pattern == "rotate_scan" else State.FORWARD
        self.state_started = self.start_time
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
            self.finish("ERROR_no_scan", 5, "initial_scan_timeout")

        while not rospy.is_shutdown() and not self.shutdown_requested:
            now = time.monotonic()
            front_min, global_min, left_min, right_min = self.scan_ranges()
            event = ""
            cmd = self.select_command(now, front_min, global_min, left_min, right_min)
            if self._last_event:
                event = self._last_event
                self._last_event = ""
            self.publish(cmd)
            self.log_sample(front_min, global_min, cmd, event)

            if self.state in (State.DONE, State.ERROR):
                break
            rate.sleep()

        if self.shutdown_requested and not self.stop_reason:
            self.finish("ERROR_unknown", 1, "shutdown_requested")
        self.stop_robot(repeats=8)
        self.write_status()
        self.csv_file.close()
        print(f"[SAFE_MAPPING] stop_reason={self.stop_reason}", flush=True)
        print(f"[SAFE_MAPPING] elapsed={self.elapsed():.2f}", flush=True)
        print(
            f"[SAFE_MAPPING] explored_ratio={self.format_ratio(self.explored_ratio)}",
            flush=True,
        )
        print(
            f"[SAFE_MAPPING] unknown_ratio={self.format_ratio(self.unknown_ratio)}",
            flush=True,
        )
        rospy.loginfo(
            "[safe_mapping_driver] finished stop_reason=%s elapsed=%.2f explored_ratio=%s unknown_ratio=%s exit_code=%d",
            self.stop_reason,
            self.elapsed(),
            self.format_ratio(self.explored_ratio),
            self.format_ratio(self.unknown_ratio),
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
        elapsed = now - self.start_time

        # Stale scan
        if not self.scan_is_fresh():
            self.finish("ERROR_no_scan", 5, "scan_timeout")
            return cmd

        if global_min < self.critical_stop_distance:
            if self.critical_since is None:
                self.critical_since = now
                self._last_event = "critical_proximity_start"
            elif now - self.critical_since > self.collision_risk_duration_sec:
                self.finish("ERROR_collision_risk", 3, "collision_risk_abort")
                return cmd
        else:
            if self.critical_since is not None:
                self._last_event = "critical_proximity_cleared"
            self.critical_since = None

        if elapsed >= self.max_duration_sec:
            self.finish("DONE_duration", 0, "max_duration_reached")
            return cmd

        if (
            elapsed >= self.min_mapping_duration_sec
            and self.coverage_available
            and self.explored_ratio is not None
            and self.unknown_ratio is not None
            and (
                self.explored_ratio >= self.explored_ratio_threshold
                or self.unknown_ratio <= self.unknown_ratio_threshold
            )
        ):
            self.finish("DONE_coverage", 0, "coverage_reached")
            return cmd

        if elapsed >= self.min_mapping_duration_sec and self.stuck_count > self.max_stuck_count:
            self.finish("ERROR_stuck", 4, "max_stuck_count_exceeded")
            return cmd

        # rotate_scan pattern — just spin in place
        if self.args.pattern == "rotate_scan":
            self.state = State.ROTATE_SEARCH_LEFT
            cmd.angular.z = self.rotate_speed
            return self.sanitize_cmd(cmd)

        blocked = front_min < self.safety_stop_distance
        side_caution_distance = max(0.35, self.critical_stop_distance + 0.15)

        if not blocked:
            if self.state in (
                State.ROTATE_SEARCH_LEFT,
                State.ROTATE_SEARCH_RIGHT,
                State.ESCAPE_ROTATE,
            ) and now - self.state_started < 0.8:
                cmd.angular.z = self.rotation_sign() * self.rotate_speed
                return self.sanitize_cmd(cmd)
            self.blocked_since = None
            self.rotate_attempts = 0
            self.stuck_count = 0
            
            if front_min < self.safety_stop_distance * 2:
                self.set_state(State.SLOW_FORWARD, now)
                cmd.linear.x = min(self.forward_speed * 0.75, 0.1)
                return self.sanitize_cmd(cmd)
            elif front_min < self.safety_stop_distance * 1.5:
                self.set_state(State.SLOW_FORWARD, now)
                cmd.linear.x = min(self.forward_speed * 0.5, 0.1)
                return self.sanitize_cmd(cmd)

            # if global_min < side_caution_distance:
            #     self.set_state(State.SLOW_FORWARD, now)
            #     cmd.linear.x = min(self.forward_speed * 0.5, 0.08)
            #     return self.sanitize_cmd(cmd)
            self.set_state(State.FORWARD, now)
            cmd.linear.x = self.forward_speed
            return self.sanitize_cmd(cmd)

        if self.blocked_since is None:
            self.blocked_since = now
            self.stuck_count += 1
            self.set_state(State.STOP_AND_SCAN, now)
            self._last_event = "obstacle_detected"

        if elapsed >= self.min_mapping_duration_sec and self.stuck_count > self.max_stuck_count:
            self.finish("ERROR_stuck", 4, "max_stuck_count_exceeded")
            return cmd

        if self.state not in (
            State.STOP_AND_SCAN,
            State.ROTATE_SEARCH_LEFT,
            State.ROTATE_SEARCH_RIGHT,
            State.ESCAPE_ROTATE,
        ):
            self.set_state(State.STOP_AND_SCAN, now)

        if self.state == State.STOP_AND_SCAN:
            if now - self.state_started < self.stop_and_scan_duration:
                return cmd
            new_state = self.preferred_rotation_state(left_min, right_min)
            self.rotate_attempts += 1
            self.set_state(new_state, now)
            self._last_event = f"rotate_search_start_{new_state.value.lower()}_attempt_{self.rotate_attempts}"
            rospy.loginfo(
                "[safe_mapping_driver] blocked recovery state=%s attempt=%d stuck_count=%d elapsed=%.1f front=%.2f global=%.2f",
                new_state.value,
                self.rotate_attempts,
                self.stuck_count,
                elapsed,
                front_min,
                global_min,
            )

        if self.state in (State.ROTATE_SEARCH_LEFT, State.ROTATE_SEARCH_RIGHT):
            if now - self.state_started > self.max_rotate_segment:
                self.stuck_count += 1
                self.rotate_attempts += 1
                if elapsed >= self.min_mapping_duration_sec and self.stuck_count > self.max_stuck_count:
                    self.finish("ERROR_stuck", 4, "max_stuck_count_exceeded")
                    return Twist()
                if self.rotate_attempts % 3 == 0 or (
                    self.blocked_since is not None and now - self.blocked_since > self.blocked_timeout
                ):
                    self.set_state(State.ESCAPE_ROTATE, now)
                    self._last_event = "escape_rotate_start"
                else:
                    new_state = self.opposite_rotation_state()
                    self.set_state(new_state, now)
                    self._last_event = f"rotate_search_switch_{new_state.value.lower()}_attempt_{self.rotate_attempts}"

        if self.state == State.ESCAPE_ROTATE:
            if now - self.state_started > self.escape_rotate_duration:
                self.stuck_count += 1
                if elapsed >= self.min_mapping_duration_sec and self.stuck_count > self.max_stuck_count:
                    self.finish("ERROR_stuck", 4, "max_stuck_count_exceeded")
                    return Twist()
                self.set_state(State.STOP_AND_SCAN, now)
                self.blocked_since = now
                self._last_event = "escape_rotate_done_retry"
                return Twist()

        cmd.angular.z = self.rotation_sign() * self.rotate_speed
        return self.sanitize_cmd(cmd)

    def set_state(self, state: State, now: float) -> None:
        if self.state != state:
            self.state = state
            self.state_started = now

    def finish(self, stop_reason: str, exit_code: int, event: str) -> None:
        if self.stop_reason:
            return
        self.stop_reason = stop_reason
        self.reason = stop_reason
        self.exit_code = exit_code
        self.state = State.ERROR if stop_reason.startswith("ERROR_") else State.DONE
        self._last_event = event

    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def preferred_rotation_state(self, left_min: float, right_min: float) -> State:
        return State.ROTATE_SEARCH_LEFT if right_min < left_min else State.ROTATE_SEARCH_RIGHT

    def opposite_rotation_state(self) -> State:
        if self.state == State.ROTATE_SEARCH_LEFT:
            return State.ROTATE_SEARCH_RIGHT
        return State.ROTATE_SEARCH_LEFT

    def rotation_sign(self) -> float:
        if self.state == State.ROTATE_SEARCH_RIGHT:
            return -1.0
        if self.state == State.ESCAPE_ROTATE and self.rotate_attempts % 2 == 0:
            return -1.0
        return 1.0

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
                f"{self.elapsed():.3f}",
                self.state.value,
                event,
                self.format_range(front_min),
                self.format_range(global_min),
                f"{cmd.linear.x:.4f}",
                f"{cmd.angular.z:.4f}",
                str(self.stuck_count),
                self.format_ratio(self.explored_ratio),
                self.format_ratio(self.unknown_ratio),
                self.stop_reason,
            ]
        )
        self.csv_file.flush()

    @staticmethod
    def format_range(value: float) -> str:
        return "inf" if math.isinf(value) else f"{value:.4f}"

    @staticmethod
    def format_ratio(value: Optional[float]) -> str:
        return "N/A" if value is None else f"{value:.6f}"

    def write_status(self) -> None:
        if not self.stop_reason:
            self.finish("ERROR_unknown", 1, "missing_stop_reason")
        with open(self.status_path, "w") as status_file:
            status_file.write(f"result={'PASS' if self.exit_code == 0 else 'FAIL'}\n")
            status_file.write(f"reason={self.stop_reason}\n")
            status_file.write(f"stop_reason={self.stop_reason}\n")
            status_file.write(f"exit_code={self.exit_code}\n")
            status_file.write(f"elapsed={self.elapsed():.3f}\n")
            status_file.write(f"explored_ratio={self.format_ratio(self.explored_ratio)}\n")
            status_file.write(f"unknown_ratio={self.format_ratio(self.unknown_ratio)}\n")
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
