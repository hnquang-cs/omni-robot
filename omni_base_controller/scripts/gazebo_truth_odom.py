#!/usr/bin/env python3
"""Publish odometry and TF from Gazebo model ground truth."""

import math
import threading
from typing import Optional, Tuple

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf import transformations
from tf.broadcaster import TransformBroadcaster


class GazeboTruthOdom:
    """Convert /gazebo/model_states for one model into nav_msgs/Odometry."""

    def __init__(self) -> None:
        self.model_name = self._get_param("model_name", "omni_robot")
        self.odom_topic = self._get_param("odom_topic", "/odom")
        self.odom_frame = self._get_param("odom_frame", "odom")
        self.base_frame = self._get_param("base_frame", "base_footprint")
        self.publish_tf = bool(self._get_param("publish_tf", True))
        self.publish_rate = self._get_positive_param("publish_rate", 30.0)
        self.model_states_topic = self._get_param("model_states_topic", "/gazebo/model_states")

        self.lock = threading.RLock()
        self.latest_msg: Optional[ModelStates] = None

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = TransformBroadcaster() if self.publish_tf else None
        self.model_sub = rospy.Subscriber(
            self.model_states_topic,
            ModelStates,
            self.model_states_callback,
            queue_size=1,
        )
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.publish)

        rospy.loginfo(
            "[gazebo_truth_odom] Started for model=%s odom=%s frame=%s child=%s tf=%s",
            self.model_name,
            self.odom_topic,
            self.odom_frame,
            self.base_frame,
            self.publish_tf,
        )

    def _get_param(self, name: str, default):
        param_name = f"~{name}"
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)
        rospy.logwarn("[gazebo_truth_odom] Missing parameter %s, using default %s", param_name, default)
        return default

    def _get_positive_param(self, name: str, default: float) -> float:
        value = float(self._get_param(name, default))
        if value <= 0.0:
            rospy.logwarn(
                "[gazebo_truth_odom] Parameter ~%s must be positive. Using %.3f",
                name,
                default,
            )
            return default
        return value

    def model_states_callback(self, msg: ModelStates) -> None:
        with self.lock:
            self.latest_msg = msg

    def publish(self, _event: rospy.timer.TimerEvent) -> None:
        with self.lock:
            msg = self.latest_msg

        if msg is None:
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_truth_odom] Waiting for %s",
                self.model_states_topic,
            )
            return

        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(
                5.0,
                "[gazebo_truth_odom] Model '%s' not found in %s",
                self.model_name,
                self.model_states_topic,
            )
            return

        stamp = rospy.Time.now()
        if stamp == rospy.Time(0):
            rospy.logwarn_throttle(5.0, "[gazebo_truth_odom] Waiting for valid ROS time.")
            return

        pose = msg.pose[index]
        twist_world = msg.twist[index]
        _, _, yaw = self._quaternion_to_rpy(pose.orientation)
        body_vx, body_vy = self._world_to_body_twist(
            twist_world.linear.x,
            twist_world.linear.y,
            yaw,
        )

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose
        odom_msg.twist.twist.linear.x = body_vx
        odom_msg.twist.twist.linear.y = body_vy
        odom_msg.twist.twist.linear.z = twist_world.linear.z
        odom_msg.twist.twist.angular = twist_world.angular
        self.odom_pub.publish(odom_msg)

        if self.tf_broadcaster is not None:
            q = pose.orientation
            self.tf_broadcaster.sendTransform(
                (pose.position.x, pose.position.y, pose.position.z),
                (q.x, q.y, q.z, q.w),
                stamp,
                self.base_frame,
                self.odom_frame,
            )

    @staticmethod
    def _quaternion_to_rpy(quaternion: Quaternion) -> Tuple[float, float, float]:
        return transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )

    @staticmethod
    def _world_to_body_twist(world_vx: float, world_vy: float, yaw: float) -> Tuple[float, float]:
        body_vx = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy
        body_vy = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy
        return body_vx, body_vy


def main() -> None:
    rospy.init_node("gazebo_truth_odom")
    GazeboTruthOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
