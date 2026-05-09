#!/usr/bin/env python3
"""Discrete 45-degree motion primitive controller.

Accepts FORWARD, TURN_LEFT_45, TURN_RIGHT_45, STOP on /motion_primitive_cmd
and emits forward-only Twist on /cmd_vel. Turning is closed-loop on /odom yaw.

Safety contract:
  - linear.x >= 0           (no reverse)
  - linear.y == 0           (no lateral / no holonomic strafing)
  - never linear.x>0 AND |angular.z|>0 simultaneously
  - if /odom is stale, command is zero and state is ERROR_NO_ODOM
"""

import math
import threading
from enum import Enum
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class State(Enum):
    IDLE = "IDLE"
    DRIVE_STRAIGHT = "DRIVE_STRAIGHT"
    TURNING_LEFT_45 = "TURNING_LEFT_45"
    TURNING_RIGHT_45 = "TURNING_RIGHT_45"
    STOPPED = "STOPPED"
    ERROR_NO_ODOM = "ERROR_NO_ODOM"
    ERROR_TURN_TIMEOUT = "ERROR_TURN_TIMEOUT"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class Discrete45MotionController:
    VALID_PRIMITIVES = {"FORWARD", "STOP", "TURN_LEFT_45", "TURN_RIGHT_45"}

    def __init__(self) -> None:
        self.input_mode = str(rospy.get_param("~input_mode", "primitive")).strip().lower()
        self.primitive_topic = rospy.get_param("~primitive_topic", "/motion_primitive_cmd")
        self.cmd_vel_raw_topic = rospy.get_param("~cmd_vel_raw_topic", "/cmd_vel_raw")
        self.cmd_vel_out_topic = rospy.get_param("~cmd_vel_out_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.state_topic = rospy.get_param("~state_topic", "/motion_primitive_state")
        self.debug_topic = rospy.get_param("~debug_topic", "/motion_primitive_debug")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))

        self.forward_speed = abs(float(rospy.get_param("~forward_speed", 0.25)))
        self.max_forward_speed = abs(float(rospy.get_param("~max_forward_speed", 0.35)))
        self.turn_speed = abs(float(rospy.get_param("~turn_speed", 0.25)))
        self.max_turn_speed = abs(float(rospy.get_param("~max_turn_speed", 0.35)))
        self.turn_angle = math.radians(float(rospy.get_param("~turn_angle_deg", 45.0)))
        self.yaw_tolerance = math.radians(float(rospy.get_param("~yaw_tolerance_deg", 2.0)))
        self.turn_timeout_sec = float(rospy.get_param("~turn_timeout_sec", 8.0))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 1.0))

        self.allow_continuous_forward = bool(rospy.get_param("~allow_continuous_forward", True))
        self.stop_after_turn = bool(rospy.get_param("~stop_after_turn", True))
        self.linear_threshold = float(rospy.get_param("~linear_threshold", 0.02))
        self.angular_threshold = float(rospy.get_param("~angular_threshold", 0.02))

        self.allow_reverse = bool(rospy.get_param("~allow_reverse", False))
        self.allow_lateral = bool(rospy.get_param("~allow_lateral", False))
        self.allow_drive_and_turn = bool(rospy.get_param("~allow_drive_and_turn_same_time", False))

        self.forward_speed = clamp(self.forward_speed, 0.0, self.max_forward_speed)
        self.turn_speed = clamp(self.turn_speed, 0.0, self.max_turn_speed)

        self.lock = threading.RLock()
        self.state = State.IDLE
        self.has_odom = False
        self.latest_yaw: Optional[float] = None
        self.latest_odom_stamp: Optional[rospy.Time] = None
        self.last_cmd_time: Optional[rospy.Time] = None
        self.turn_start_yaw = 0.0
        self.turn_target_yaw = 0.0
        self.turn_direction = 0
        self.turn_start_time: Optional[rospy.Time] = None
        self.last_published_state: Optional[State] = None
        self.pending_primitive: Optional[str] = None

        self.cmd_pub = rospy.Publisher(self.cmd_vel_out_topic, Twist, queue_size=10)
        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=10, latch=True)
        self.debug_pub = rospy.Publisher(self.debug_topic, String, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.primitive_sub = rospy.Subscriber(
            self.primitive_topic,
            String,
            self.primitive_callback,
            queue_size=10,
        )
        self.raw_sub = rospy.Subscriber(
            self.cmd_vel_raw_topic,
            Twist,
            self.cmd_vel_callback,
            queue_size=10,
        )
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.update)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "[discrete_45_motion_controller] mode=%s primitive=%s raw=%s out=%s odom=%s "
            "state=%s debug=%s forward=%.3f turn=%.3f angle=%.1fdeg tol=%.1fdeg odom_timeout=%.2fs",
            self.input_mode,
            self.primitive_topic,
            self.cmd_vel_raw_topic,
            self.cmd_vel_out_topic,
            self.odom_topic,
            self.state_topic,
            self.debug_topic,
            self.forward_speed,
            self.turn_speed,
            math.degrees(self.turn_angle),
            math.degrees(self.yaw_tolerance),
            self.odom_timeout_sec,
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        with self.lock:
            self.latest_yaw = normalize_angle(yaw)
            stamp = msg.header.stamp
            if stamp == rospy.Time(0):
                stamp = rospy.Time.now()
            self.latest_odom_stamp = stamp
            self.has_odom = True

    def primitive_callback(self, msg: String) -> None:
        if self.input_mode != "primitive":
            return
        primitive = msg.data.strip().upper()
        if primitive not in self.VALID_PRIMITIVES:
            rospy.logwarn("[discrete_45_motion_controller] invalid primitive: %s", msg.data)
            return
        self.accept_primitive(primitive)

    def cmd_vel_callback(self, msg: Twist) -> None:
        if self.input_mode != "cmd_vel":
            return
        if msg.angular.z > self.angular_threshold:
            primitive = "TURN_LEFT_45"
        elif msg.angular.z < -self.angular_threshold:
            primitive = "TURN_RIGHT_45"
        elif msg.linear.x > self.linear_threshold and abs(msg.angular.z) <= self.angular_threshold:
            primitive = "FORWARD"
        else:
            primitive = "STOP"
        self.accept_primitive(primitive)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def accept_primitive(self, primitive: str) -> None:
        now = rospy.Time.now()
        with self.lock:
            self.last_cmd_time = now

            # Allow recovery from ERROR_NO_ODOM as soon as a fresh sample is in.
            if self.state == State.ERROR_NO_ODOM and self.has_fresh_odom(now):
                self.set_state(State.IDLE)

            if primitive == "STOP":
                self.set_state(State.STOPPED)
                self.pending_primitive = None
                return

            if not self.has_fresh_odom(now):
                rospy.logwarn_throttle(
                    2.0,
                    "[discrete_45_motion_controller] cannot accept %s: no fresh odom",
                    primitive,
                )
                self.pending_primitive = primitive
                self.set_state(State.ERROR_NO_ODOM)
                return

            if primitive == "FORWARD":
                if self.state not in (State.TURNING_LEFT_45, State.TURNING_RIGHT_45):
                    self.set_state(State.DRIVE_STRAIGHT)
                return

            if primitive == "TURN_LEFT_45":
                self.start_turn(+1, now)
                return

            if primitive == "TURN_RIGHT_45":
                self.start_turn(-1, now)

    def start_turn(self, direction: int, now: rospy.Time) -> None:
        if self.state in (State.TURNING_LEFT_45, State.TURNING_RIGHT_45):
            return
        if self.latest_yaw is None or not self.has_fresh_odom(now):
            rospy.logwarn_throttle(
                2.0,
                "[discrete_45_motion_controller] cannot start turn: no fresh odom",
            )
            self.set_state(State.ERROR_NO_ODOM)
            return
        self.turn_direction = 1 if direction > 0 else -1
        self.turn_start_yaw = self.latest_yaw
        self.turn_target_yaw = normalize_angle(self.turn_start_yaw + self.turn_direction * self.turn_angle)
        self.turn_start_time = now
        self.set_state(State.TURNING_LEFT_45 if self.turn_direction > 0 else State.TURNING_RIGHT_45)
        rospy.loginfo(
            "[discrete_45_motion_controller] start %s from %.2f deg to %.2f deg",
            self.state.value,
            math.degrees(self.turn_start_yaw),
            math.degrees(self.turn_target_yaw),
        )

    def update(self, _event) -> None:
        now = rospy.Time.now()
        if now == rospy.Time(0):
            return
        with self.lock:
            self._maybe_recover(now)
            cmd = self.command_for_state(now)
            state = self.state
            yaw = self.latest_yaw
            target = self.turn_target_yaw if state in (State.TURNING_LEFT_45, State.TURNING_RIGHT_45) else None

        self.cmd_pub.publish(cmd)
        self.publish_state(state)
        self.publish_debug(state, yaw, target)

    def _maybe_recover(self, now: rospy.Time) -> None:
        if self.state == State.ERROR_NO_ODOM and self.has_fresh_odom(now):
            queued = self.pending_primitive
            self.pending_primitive = None
            self.set_state(State.IDLE)
            if queued is not None:
                rospy.loginfo(
                    "[discrete_45_motion_controller] odom restored, replaying %s",
                    queued,
                )
                # Re-queue command using normal path, without holding additional locks
                # (we are already inside self.lock).
                if queued == "FORWARD":
                    self.set_state(State.DRIVE_STRAIGHT)
                elif queued == "TURN_LEFT_45":
                    self.start_turn(+1, now)
                elif queued == "TURN_RIGHT_45":
                    self.start_turn(-1, now)

    def command_for_state(self, now: rospy.Time) -> Twist:
        cmd = Twist()

        if self.state == State.ERROR_NO_ODOM:
            return cmd

        if self.state == State.DRIVE_STRAIGHT:
            if not self.has_fresh_odom(now):
                rospy.logwarn_throttle(2.0, "[discrete_45_motion_controller] odom lost during forward")
                self.set_state(State.ERROR_NO_ODOM)
                return Twist()
            if self.input_mode == "cmd_vel" and self.command_timed_out(now):
                self.set_state(State.STOPPED)
                return cmd
            if self.input_mode == "primitive" and not self.allow_continuous_forward and self.command_timed_out(now):
                self.set_state(State.STOPPED)
                return cmd
            cmd.linear.x = self.forward_speed
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            return self.sanitize(cmd)

        if self.state in (State.TURNING_LEFT_45, State.TURNING_RIGHT_45):
            if self.latest_yaw is None or not self.has_fresh_odom(now):
                rospy.logwarn_throttle(2.0, "[discrete_45_motion_controller] odom lost during turn")
                self.set_state(State.ERROR_NO_ODOM)
                return Twist()
            if self.turn_start_time is not None and (now - self.turn_start_time).to_sec() > self.turn_timeout_sec:
                rospy.logerr("[discrete_45_motion_controller] turn timeout; stopping")
                self.set_state(State.ERROR_TURN_TIMEOUT)
                return Twist()
            if self.turn_complete():
                self.set_state(State.STOPPED if self.stop_after_turn else State.IDLE)
                return Twist()
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = self.turn_direction * self.turn_speed
            return self.sanitize(cmd)

        return Twist()

    def turn_complete(self) -> bool:
        if self.latest_yaw is None:
            return False
        error = normalize_angle(self.turn_target_yaw - self.latest_yaw)
        progress = self.turn_direction * normalize_angle(self.latest_yaw - self.turn_start_yaw)
        if abs(error) <= self.yaw_tolerance:
            return True
        return progress >= (self.turn_angle - self.yaw_tolerance)

    def command_timed_out(self, now: rospy.Time) -> bool:
        return self.last_cmd_time is None or (now - self.last_cmd_time).to_sec() > self.cmd_timeout

    def has_fresh_odom(self, now: Optional[rospy.Time] = None) -> bool:
        if not self.has_odom or self.latest_odom_stamp is None:
            return False
        if now is None:
            now = rospy.Time.now()
        if now == rospy.Time(0):
            return False
        delta = (now - self.latest_odom_stamp).to_sec()
        # Allow small clock skew (negative delta) when /clock catches up.
        if delta < -self.odom_timeout_sec:
            return False
        return abs(delta) <= self.odom_timeout_sec

    def set_state(self, state: State) -> None:
        if self.state != state:
            rospy.loginfo("[discrete_45_motion_controller] state %s -> %s", self.state.value, state.value)
        self.state = state

    def publish_state(self, state: State) -> None:
        if self.last_published_state != state:
            self.last_published_state = state
        self.state_pub.publish(String(data=state.value))

    def publish_debug(self, state: State, yaw: Optional[float], target: Optional[float]) -> None:
        yaw_deg = math.degrees(yaw) if yaw is not None else float("nan")
        target_deg = math.degrees(target) if target is not None else float("nan")
        if yaw is not None and target is not None:
            error_deg = math.degrees(normalize_angle(target - yaw))
        else:
            error_deg = float("nan")
        msg = (
            f"state={state.value} has_odom={self.has_odom} "
            f"yaw_deg={yaw_deg:.2f} target_deg={target_deg:.2f} error_deg={error_deg:.2f}"
        )
        self.debug_pub.publish(String(data=msg))

    def sanitize(self, cmd: Twist) -> Twist:
        out = Twist()
        if self.allow_reverse:
            out.linear.x = clamp(cmd.linear.x, -self.max_forward_speed, self.max_forward_speed)
        else:
            out.linear.x = clamp(cmd.linear.x, 0.0, self.max_forward_speed)
        out.linear.y = clamp(cmd.linear.y, -self.max_forward_speed, self.max_forward_speed) if self.allow_lateral else 0.0
        out.linear.z = 0.0
        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = clamp(cmd.angular.z, -self.max_turn_speed, self.max_turn_speed)

        if not self.allow_drive_and_turn and out.linear.x > 0.0 and abs(out.angular.z) > 1e-9:
            rospy.logwarn_throttle(
                1.0,
                "[discrete_45_motion_controller] drive-and-turn command blocked",
            )
            out.angular.z = 0.0
        return out

    def shutdown(self) -> None:
        zero = Twist()
        for _ in range(8):
            try:
                self.cmd_pub.publish(zero)
                rospy.sleep(0.03)
            except rospy.ROSException:
                break


def main() -> None:
    rospy.init_node("discrete_45_motion_controller")
    Discrete45MotionController()
    rospy.spin()


if __name__ == "__main__":
    main()
