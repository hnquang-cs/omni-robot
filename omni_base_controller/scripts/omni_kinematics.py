#!/usr/bin/env python3
"""Kinematic omni base controller for a 4-wheel holonomic platform.

This node uses a simple symmetric 4-wheel holonomic model and integrates
commanded body twist to produce kinematic odometry for early-stage testing.
"""

import math
from typing import Any, Dict, List

import rospy
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf import transformations
from tf.broadcaster import TransformBroadcaster


class OmniKinematicsNode:
    """Subscribe to cmd_vel and publish wheel speeds, odom, and TF."""

    def __init__(self) -> None:
        self.wheel_joint_names = [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ]

        self.wheel_radius = self._get_positive_param("wheel_radius", 0.06)
        self.lx = self._get_positive_param("lx", 0.19)
        self.ly = self._get_positive_param("ly", 0.16)
        self.publish_rate = self._get_positive_param("publish_rate", 30.0)
        self.cmd_timeout = self._get_positive_param("cmd_timeout", 0.5)

        self.odom_frame = self._get_param("odom_frame", "odom")
        self.base_frame = self._get_param("base_frame", "base_footprint")
        self.cmd_vel_topic = self._get_param("cmd_vel_topic", "/cmd_vel")
        self.odom_topic = self._get_param("odom_topic", "/odom")
        self.wheel_topic = self._get_param("wheel_topic", "/wheel_velocities")
        self.publish_odom = bool(self._get_param("publish_odom", True))
        self.publish_tf = bool(self._get_param("publish_tf", True))
        self.use_base_footprint = bool(self._get_param("use_base_footprint", True))

        initial_pose = self._get_pose_param("initial_pose")
        self.x = initial_pose["x"]
        self.y = initial_pose["y"]
        self.yaw = initial_pose["yaw"]

        if self.use_base_footprint and self.base_frame != "base_footprint":
            rospy.logwarn(
                "[omni_kinematics] use_base_footprint is true but base_frame is '%s'.",
                self.base_frame,
            )

        self.current_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10) if self.publish_odom else None
        self.wheel_pub = rospy.Publisher(self.wheel_topic, JointState, queue_size=10)
        # Mirror the same joint data for robot_state_publisher compatibility.
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.cmd_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster() if self.publish_tf else None

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.update)

        rospy.loginfo(
            "[omni_kinematics] Started with cmd_vel=%s, odom=%s, wheels=%s, odom_pub=%s, tf=%s",
            self.cmd_vel_topic,
            self.odom_topic,
            self.wheel_topic,
            self.publish_odom,
            self.publish_tf,
        )
        rospy.loginfo(
            "[omni_kinematics] Using 4-wheel holonomic inverse kinematics with wheel_radius=%.3f, lx=%.3f, ly=%.3f",
            self.wheel_radius,
            self.lx,
            self.ly,
        )

    def _get_param(self, name: str, default: Any) -> Any:
        param_name = f"~{name}"
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)
        rospy.logwarn("[omni_kinematics] Missing parameter %s, using default %s", param_name, default)
        return default

    def _get_positive_param(self, name: str, default: float) -> float:
        value = float(self._get_param(name, default))
        if value <= 0.0:
            rospy.logwarn(
                "[omni_kinematics] Parameter ~%s must be positive. Falling back to %.3f",
                name,
                default,
            )
            return default
        return value

    def _get_pose_param(self, name: str) -> Dict[str, float]:
        default_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        value = self._get_param(name, default_pose)
        if not isinstance(value, dict):
            rospy.logwarn("[omni_kinematics] Parameter ~%s must be a dictionary. Using defaults.", name)
            return default_pose
        return {
            "x": float(value.get("x", 0.0)),
            "y": float(value.get("y", 0.0)),
            "yaw": float(value.get("yaw", 0.0)),
        }

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.current_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def compute_wheel_velocities(self, vx: float, vy: float, wz: float) -> List[float]:
        """Return wheel angular velocities in rad/s.

        Assumption:
        - symmetric 4-wheel holonomic base
        - standard educational X-layout inverse mapping
        - wheels ordered as front_left, front_right, rear_left, rear_right
        """

        arm_sum = self.lx + self.ly
        return [
            (vx - vy - arm_sum * wz) / self.wheel_radius,
            (vx + vy + arm_sum * wz) / self.wheel_radius,
            (vx + vy - arm_sum * wz) / self.wheel_radius,
            (vx - vy + arm_sum * wz) / self.wheel_radius,
        ]

    def integrate_kinematic_odom(self, vx: float, vy: float, wz: float, dt: float) -> None:
        """Integrate body-frame velocity into the odom frame."""
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

    def get_active_command(self, now: rospy.Time) -> Twist:
        if self.last_cmd_time == rospy.Time(0):
            return Twist()
        if (now - self.last_cmd_time).to_sec() > self.cmd_timeout:
            return Twist()
        return self.current_cmd

    def publish_wheel_state(self, stamp: rospy.Time, wheel_velocities: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(self.wheel_joint_names)
        msg.position = list(self.wheel_positions)
        msg.velocity = list(wheel_velocities)
        self.wheel_pub.publish(msg)
        self.joint_state_pub.publish(msg)

    def publish_odometry(self, stamp: rospy.Time, vx: float, vy: float, wz: float) -> None:
        if self.odom_pub is None:
            return
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        self.odom_pub.publish(odom_msg)

    def publish_transform(self, stamp: rospy.Time) -> None:
        if self.tf_broadcaster is None:
            return
        quaternion = transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            quaternion,
            stamp,
            self.base_frame,
            self.odom_frame,
        )

    def update(self, event: rospy.timer.TimerEvent) -> None:
        if event.last_real is None:
            return
        dt = (event.current_real - event.last_real).to_sec()
        if dt <= 0.0:
            return

        now = rospy.Time.now()
        active_cmd = self.get_active_command(now)
        vx = active_cmd.linear.x
        vy = active_cmd.linear.y
        wz = active_cmd.angular.z

        wheel_velocities = self.compute_wheel_velocities(vx, vy, wz)
        for index, velocity in enumerate(wheel_velocities):
            self.wheel_positions[index] += velocity * dt

        self.integrate_kinematic_odom(vx, vy, wz, dt)
        self.publish_wheel_state(now, wheel_velocities)
        self.publish_odometry(now, vx, vy, wz)
        self.publish_transform(now)

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        quat = transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])


def main() -> None:
    rospy.init_node("omni_kinematics")
    OmniKinematicsNode()
    rospy.spin()


if __name__ == "__main__":
    main()
