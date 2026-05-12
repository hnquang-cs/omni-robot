#!/usr/bin/env python3
"""Stage 11 fallback frontier detector.

Used ONLY if explore_lite is not available. Detects frontier cells (free cells
adjacent to unknown), clusters them by 4-connectivity, picks the largest
cluster (or the closest, configurable), and publishes:
  - /simple_frontiers (visualization_msgs/MarkerArray)  for RViz
  - /move_base_simple/goal (geometry_msgs/PoseStamped)  one goal at a time

This is intentionally simple. It does NOT track action-server result codes,
does NOT blacklist failed goals, and does NOT plan. Re-run it (or
simple_frontier_detector with rosrun --respawn) for continuous exploration.

Recommended path: install explore_lite. Use this only as a placeholder.
"""

import math
import sys
from typing import List, Optional, Tuple

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def grid_to_world(i: int, j: int, info) -> Tuple[float, float]:
    x = info.origin.position.x + (j + 0.5) * info.resolution
    y = info.origin.position.y + (i + 0.5) * info.resolution
    return x, y


class SimpleFrontierDetector:
    def __init__(self) -> None:
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.marker_topic = rospy.get_param("~marker_topic", "/simple_frontiers")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.publish_period = rospy.get_param("~publish_period", 5.0)
        self.min_cluster_cells = int(rospy.get_param("~min_cluster_cells", 8))
        self.selection = rospy.get_param("~selection", "nearest")  # or "nearest"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.last_map: Optional[OccupancyGrid] = None
        self.last_goal_xy: Optional[Tuple[float, float]] = None
        self.last_goal_time: rospy.Time = rospy.Time(0)

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=False)
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_cb, queue_size=1)

        rospy.Timer(rospy.Duration(self.publish_period), self.tick)
        rospy.loginfo("simple_frontier_detector ready: map=%s goal=%s markers=%s",
                      self.map_topic, self.goal_topic, self.marker_topic)

    def map_cb(self, msg: OccupancyGrid) -> None:
        self.last_map = msg

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            tr = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
        except Exception:
            return None
        return tr.transform.translation.x, tr.transform.translation.y

    @staticmethod
    def find_frontier_cells(data: List[int], width: int, height: int) -> List[Tuple[int, int]]:
        result: List[Tuple[int, int]] = []
        for i in range(height):
            for j in range(width):
                v = data[i * width + j]
                if v != 0:
                    continue  # only free cells can be frontier seeds
                if i > 0 and data[(i - 1) * width + j] == -1:
                    result.append((i, j)); continue
                if i < height - 1 and data[(i + 1) * width + j] == -1:
                    result.append((i, j)); continue
                if j > 0 and data[i * width + (j - 1)] == -1:
                    result.append((i, j)); continue
                if j < width - 1 and data[i * width + (j + 1)] == -1:
                    result.append((i, j)); continue
        return result

    @staticmethod
    def cluster(cells: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        cell_set = set(cells)
        clusters: List[List[Tuple[int, int]]] = []
        while cell_set:
            seed = next(iter(cell_set))
            stack = [seed]
            cluster: List[Tuple[int, int]] = []
            while stack:
                ci, cj = stack.pop()
                if (ci, cj) not in cell_set:
                    continue
                cell_set.remove((ci, cj))
                cluster.append((ci, cj))
                for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nb = (ci + di, cj + dj)
                    if nb in cell_set:
                        stack.append(nb)
            clusters.append(cluster)
        return clusters

    def tick(self, _evt) -> None:
        if self.last_map is None:
            return
        msg = self.last_map
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        data = list(msg.data)
        cells = self.find_frontier_cells(data, info.width, info.height)
        clusters = [c for c in self.cluster(cells) if len(c) >= self.min_cluster_cells]
        rospy.loginfo_throttle(10.0, "frontier clusters: %d", len(clusters))
        self.publish_markers(clusters, info, msg.header.stamp)

        if not clusters:
            return

        target_xy: Optional[Tuple[float, float]] = None
        robot = self.get_robot_xy()
        if robot is None:
            return
        best_d = math.inf
        for cluster in clusters:
            ci = sum(c[0] for c in cluster) / len(cluster)
            cj = sum(c[1] for c in cluster) / len(cluster)
            wx, wy = grid_to_world(int(ci), int(cj), info)
            d = math.hypot(wx - robot[0], wy - robot[1])
            if d < best_d:
                best_d = d
                target_xy = (wx, wy)

        if target_xy is None:
            return

        # Avoid spamming the same goal.
        now = rospy.Time.now()
        if (self.last_goal_xy is not None
                and (now - self.last_goal_time).to_sec() < self.publish_period * 0.9):
            dx = target_xy[0] - self.last_goal_xy[0]
            dy = target_xy[1] - self.last_goal_xy[1]
            if math.hypot(dx, dy) < 0.20:
                return

        goal = PoseStamped()
        goal.header.stamp = now
        goal.header.frame_id = self.global_frame
        goal.pose.position.x = target_xy[0]
        goal.pose.position.y = target_xy[1]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.last_goal_xy = target_xy
        self.last_goal_time = now
        rospy.loginfo("simple frontier goal -> (%.2f, %.2f)", *target_xy)

    def publish_markers(self, clusters, info, stamp) -> None:
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.global_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for idx, cluster in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.global_frame
            m.header.stamp = stamp
            m.ns = "frontiers"
            m.id = idx
            m.type = Marker.POINTS
            m.action = Marker.ADD
            m.scale.x = info.resolution
            m.scale.y = info.resolution
            m.color = ColorRGBA(0.0, 1.0, 0.5, 1.0)
            for ci, cj in cluster:
                wx, wy = grid_to_world(ci, cj, info)
                from geometry_msgs.msg import Point
                m.points.append(Point(x=wx, y=wy, z=0.0))
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main() -> int:
    rospy.init_node("simple_frontier_detector")
    SimpleFrontierDetector()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
