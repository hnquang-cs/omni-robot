#!/usr/bin/env python3
"""Kinematic Gazebo model controller driven by geometry_msgs/Twist.

This controller is intentionally simple: it moves a Gazebo model by calling
/gazebo/set_model_state at a fixed rate. It is meant for repeatable SLAM
simulation when a full wheel dynamics controller is not available.
"""

import math
import threading
from typing import Optional, Tuple

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose, Quaternion, Twist
from tf import transformations


class GazeboModelCmdVel:
    """Integrate /cmd_vel and apply the resulting pose to a Gazebo model."""

    def __init__(self) -> None:
        self.model_name = self._get_param("model_name", "omni_robot")
        self.cmd_vel_topic = self._get_param("cmd_vel_topic", "/cmd_vel")
        self.publish_rate = self._get_positive_param("publish_rate", 30.0)
        self.cmd_timeout = self._get_positive_param("cmd_timeout", 0.5)
        self.max_linear_velocity = self._get_positive_param("max_linear_velocity", 0.4)
        self.max_angular_velocity = self._get_positive_param("max_angular_velocity", 0.8)
        self.service_name = self._get_param("set_model_state_service", "/gazebo/set_model_state")
        self.model_states_topic = self._get_param("model_states_topic", "/gazebo/model_states")

        self.lock = threading.RLock()
        self.current_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.last_update_time: Optional[rospy.Time] = None

        self.pose_ready = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.cmd_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=10)
        self.model_sub = rospy.Subscriber(
            self.model_states_topic,
            ModelStates,
            self.model_states_callback,
            queue_size=1,
        )
        self.set_model_state = rospy.ServiceProxy(self.service_name, SetModelState)
        self.service_available = False

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.update)

        rospy.loginfo(
            "[gazebo_model_cmd_vel] Started for model=%s cmd_vel=%s rate=%.1f Hz",
            self.model_name,
            self.cmd_vel_topic,
            self.publish_rate,
        )

    def _get_param(self, name: str, default):
        param_name = f"~{name}"
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)
        rospy.logwarn("[gazebo_model_cmd_vel] Missing parameter %s, using default %s", param_name, default)
        return default

    def _get_positive_param(self, name: str, default: float) -> float:
        value = float(self._get_param(name, default))
        if value <= 0.0:
            rospy.logwarn(
                "[gazebo_model_cmd_vel] Parameter ~%s must be positive. Using %.3f",
                name,
                default,
            )
            return default
        return value

    def cmd_vel_callback(self, msg: Twist) -> None:
        with self.lock:
            self.current_cmd = msg
            self.last_cmd_time = rospy.Time.now()

    def model_states_callback(self, msg: ModelStates) -> None:
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_model_cmd_vel] Model '%s' not found in %s",
                self.model_name,
                self.model_states_topic,
            )
            return

        pose = msg.pose[index]
        roll, pitch, yaw = self._pose_to_rpy(pose)

        with self.lock:
            # Keep z, roll, and pitch from Gazebo to avoid forcing the robot to fly.
            if not self.pose_ready:
                self.x = pose.position.x
                self.y = pose.position.y
                self.yaw = yaw
            self.z = pose.position.z
            self.roll = roll
            self.pitch = pitch
            self.pose_ready = True

    def update(self, event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now()
        if now == rospy.Time(0):
            rospy.logwarn_throttle(5.0, "[gazebo_model_cmd_vel] Waiting for valid ROS time.")
            return

        with self.lock:
            if not self.pose_ready:
                rospy.logwarn_throttle(
                    5.0,
                    "[gazebo_model_cmd_vel] Waiting for model '%s' pose from %s",
                    self.model_name,
                    self.model_states_topic,
                )
                return

            if self.last_update_time is None:
                self.last_update_time = now
                return

            dt = (now - self.last_update_time).to_sec()
            self.last_update_time = now
            if dt <= 0.0 or dt > 1.0:
                return

            vx, vy, wz = self._active_command(now)
            world_vx = math.cos(self.yaw) * vx - math.sin(self.yaw) * vy
            world_vy = math.sin(self.yaw) * vx + math.cos(self.yaw) * vy

            self.x += world_vx * dt
            self.y += world_vy * dt
            self.yaw = self._normalize_angle(self.yaw + wz * dt)

            request = self._build_set_state_request(world_vx, world_vy, wz)

        self._call_set_model_state(request)

    def _active_command(self, now: rospy.Time) -> Tuple[float, float, float]:
        if self.last_cmd_time == rospy.Time(0):
            return 0.0, 0.0, 0.0
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return 0.0, 0.0, 0.0

        vx = self.current_cmd.linear.x
        vy = self.current_cmd.linear.y
        wz = self.current_cmd.angular.z

        speed = math.hypot(vx, vy)
        if speed > self.max_linear_velocity:
            scale = self.max_linear_velocity / speed
            vx *= scale
            vy *= scale
        wz = max(-self.max_angular_velocity, min(self.max_angular_velocity, wz))
        return vx, vy, wz

    def _build_set_state_request(self, world_vx: float, world_vy: float, wz: float) -> SetModelStateRequest:
        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation = self._rpy_to_quaternion(self.roll, self.pitch, self.yaw)
        state.twist.linear.x = world_vx
        state.twist.linear.y = world_vy
        state.twist.linear.z = 0.0
        state.twist.angular.z = wz

        request = SetModelStateRequest()
        request.model_state = state
        return request

    def _call_set_model_state(self, request: SetModelStateRequest) -> None:
        if not self.service_available:
            try:
                rospy.wait_for_service(self.service_name, timeout=0.2)
                self.service_available = True
            except rospy.ROSException:
                rospy.logwarn_throttle(
                    5.0,
                    "[gazebo_model_cmd_vel] Waiting for service %s",
                    self.service_name,
                )
                return

        try:
            response = self.set_model_state(request)
        except rospy.ServiceException as exc:
            self.service_available = False
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_model_cmd_vel] Failed to call %s: %s",
                self.service_name,
                exc,
            )
            return

        if not response.success:
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_model_cmd_vel] %s rejected state update: %s",
                self.service_name,
                response.status_message,
            )

    @staticmethod
    def _pose_to_rpy(pose: Pose) -> Tuple[float, float, float]:
        q = pose.orientation
        return transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        quat = transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rospy.init_node("gazebo_model_cmd_vel")
    GazeboModelCmdVel()
    rospy.spin()


if __name__ == "__main__":
    main()
