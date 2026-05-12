#!/usr/bin/env python3
"""Stage 11 frontier-exploration trial monitor.

Subscribes to live topics, decides when the trial should stop, and writes
state to a JSON file every tick.

Stop conditions, evaluated in priority order at every tick:

  1. ERROR_collision_risk
       global_min_range < collision_range  for >= collision_duration seconds.

  2. DONE_coverage_stable
       elapsed >= min_duration
       AND we have seen at least one frontier at some point
       AND the latest frontier marker count is 0
            (or no new frontier message has arrived for confirm_no_frontier sec)
       AND that condition has held for >= confirm_no_frontier seconds.

  3. ERROR_stuck   (only after elapsed >= min_duration)
       stuck_count >= stuck_count_threshold,
       where stuck_count is incremented every tick whose pose-window-max
       displacement is below stuck_dist, and reset otherwise.

  4. DONE_duration
       elapsed >= duration

Exit code:
  0  -> DONE_*
  2  -> ERROR_*
  1  -> shutdown without firing (e.g. external SIGINT)
"""

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

try:
    from move_base_msgs.msg import MoveBaseActionGoal
    HAVE_MOVE_BASE_MSGS = True
except Exception:
    HAVE_MOVE_BASE_MSGS = False

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
from map_coverage_tracker import DynamicCoverageTracker


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--duration",               type=float, default=300.0)
    p.add_argument("--min-duration",           type=float, default=120.0,
                   help="ERROR_stuck cannot fire before this elapsed time")
    p.add_argument("--growth-window",          type=float, default=30.0,
                   help="window for explored_area_growth_m2 reporting")
    p.add_argument("--collision-range",        type=float, default=0.30)
    p.add_argument("--collision-duration",     type=float, default=2.0)
    p.add_argument("--stuck-dist",             type=float, default=0.05,
                   help="metric, max displacement over stuck-duration that"
                        " still counts as stuck")
    p.add_argument("--stuck-duration",         type=float, default=20.0,
                   help="seconds of pose-window the stuck check looks back at")
    p.add_argument("--stuck-count-threshold",  type=int,   default=5,
                   help="ticks of sustained stuckness before ERROR_stuck")
    p.add_argument("--confirm-no-frontier",    type=float, default=5.0,
                   help="seconds of zero-frontier signal needed to fire"
                        " DONE_coverage_stable")
    p.add_argument("--min-frontier-cells",     type=int,   default=20,
                   help="frontier cells below this count are treated as zero"
                        " (filters tiny unreachable corners)")
    p.add_argument("--tick-period",            type=float, default=2.0)
    p.add_argument("--state-file",             required=True)
    p.add_argument("--log-file",               default="")
    return p.parse_args(rospy.myargv()[1:])


class FrontierMonitor:
    def __init__(self, args):
        self.args = args
        self.state_file = Path(args.state_file)
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = Path(args.log_file) if args.log_file else None
        if self.log_file:
            self.log_file.parent.mkdir(parents=True, exist_ok=True)
            self.log_file.write_text("")  # truncate

        self.t_start = time.time()
        self.tracker = DynamicCoverageTracker(
            progress_window_sec=args.growth_window)

        # --- coverage / map state ---
        self.last_metrics = None

        # --- frontier state ---
        # Latest count from any frontier topic. None until first message.
        self.frontier_count: Optional[int] = None
        self.last_frontier_msg_time: Optional[float] = None
        self.has_seen_frontier = False
        # When did the "no-frontier" condition first start being true?
        self.no_frontier_since: Optional[float] = None

        # --- search attempts (for reporting only) ---
        self.search_attempts = 0
        self.seen_goal_ids = set()

        # --- collision state ---
        self.global_min_range = math.inf
        self.collision_t0: Optional[float] = None

        # --- stuck state: sliding window of (t, x, y) ---
        self.pose_window: "deque[tuple[float,float,float]]" = deque()
        self.pose_window_sec = max(args.stuck_duration * 2.0, 5.0)
        self.stuck_count = 0

        # Subscribers
        rospy.Subscriber("/map", OccupancyGrid, self._cb_map, queue_size=1)
        rospy.Subscriber("/lidar/scan", LaserScan, self._cb_scan, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray,
                         self._cb_status, queue_size=10)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                         self._cb_simple_goal, queue_size=10)
        if HAVE_MOVE_BASE_MSGS:
            rospy.Subscriber("/move_base/goal", MoveBaseActionGoal,
                             self._cb_goal, queue_size=10)

        rospy.Timer(rospy.Duration(args.tick_period), self._tick)

        banner = (
            f"frontier_exploration_monitor up: "
            f"duration={args.duration:.0f} min_duration={args.min_duration:.0f} "
            f"collision={args.collision_range:.2f}m/{args.collision_duration:.1f}s "
            f"stuck={args.stuck_dist:.2f}m/{args.stuck_duration:.0f}s "
            f"stuck_count_threshold={args.stuck_count_threshold} "
            f"confirm_no_frontier={args.confirm_no_frontier:.0f}s"
        )
        rospy.loginfo(banner)
        print(banner, flush=True)

    # ---------------- Subscribers ----------------

    def _cb_map(self, msg: OccupancyGrid):
        try:
            self.last_metrics = self.tracker.update_from_occupancy_grid(msg)
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "coverage update failed: %s", exc)
        # Count frontier cells directly from the live /map. This is the
        # authoritative "frontier present?" signal; explore_lite/fallback are
        # not required.
        try:
            count = self._count_frontier_cells(msg)
            self.frontier_count = count
            self.last_frontier_msg_time = time.time()
            if count > self.args.min_frontier_cells:
                self.has_seen_frontier = True
        except Exception as exc:
            rospy.logwarn_throttle(10.0, "frontier count failed: %s", exc)

    @staticmethod
    def _count_frontier_cells(msg: OccupancyGrid) -> int:
        """Number of free cells (==0) that have at least one unknown
        4-neighbor (==-1). Vectorized via numpy."""
        w = msg.info.width
        h = msg.info.height
        if w == 0 or h == 0:
            return 0
        arr = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape((h, w))
        free = (arr == 0)
        unknown = (arr == -1)
        # OR of unknown neighbors in 4-connectivity
        nb = np.zeros_like(unknown)
        nb[1:, :]  |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:,  :]
        nb[:, 1:]  |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        return int((free & nb).sum())

    def _cb_scan(self, msg: LaserScan):
        rng_min = float(msg.range_min) if msg.range_min > 0 else 0.0
        rng_max = float(msg.range_max) if msg.range_max > 0 else math.inf
        rmin = math.inf
        for r in msg.ranges:
            rf = float(r)
            if math.isfinite(rf) and rng_min < rf < rng_max and rf < rmin:
                rmin = rf
        self.global_min_range = rmin

    def _cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.time()
        self.pose_window.append((now, x, y))
        # Trim
        cutoff = now - self.pose_window_sec
        while self.pose_window and self.pose_window[0][0] < cutoff:
            self.pose_window.popleft()

    def _cb_goal(self, _msg):
        self.search_attempts += 1

    def _cb_simple_goal(self, _msg):
        self.search_attempts += 1

    def _cb_status(self, msg: GoalStatusArray):
        for st in msg.status_list:
            try:
                gid = st.goal_id.id
            except Exception:
                continue
            if gid and gid not in self.seen_goal_ids:
                self.seen_goal_ids.add(gid)
                if not HAVE_MOVE_BASE_MSGS:
                    self.search_attempts += 1

    # ---------------- Stuck check (window-based) ----------------

    def _is_stuck_window(self, now: float) -> bool:
        """True if max displacement among poses in the last stuck_duration
        seconds is below stuck_dist. If we have no /odom yet, treat as stuck
        (the robot literally hasn't moved since the trial started)."""
        if not self.pose_window:
            return True
        window_start = now - self.args.stuck_duration
        recent = [p for p in self.pose_window if p[0] >= window_start]
        if len(recent) < 2:
            return True
        cx, cy = recent[-1][1], recent[-1][2]
        max_d = 0.0
        for _, x, y in recent:
            d = math.hypot(cx - x, cy - y)
            if d > max_d:
                max_d = d
        return max_d < self.args.stuck_dist

    # ---------------- Frontier-done check ----------------

    def _no_frontier_signal(self, now: float) -> bool:
        """True when /map currently has no meaningful frontier cells.
        Requires has_seen_frontier (we must have observed real frontiers
        at some point during the trial, otherwise an empty initial map
        would fire DONE immediately)."""
        if not self.has_seen_frontier:
            return False
        if self.frontier_count is None:
            return False
        return self.frontier_count <= self.args.min_frontier_cells

    # ---------------- Decision ----------------

    def _decide(self, now: float, elapsed: float) -> Optional[str]:
        # 1. Collision risk (highest priority).
        if (math.isfinite(self.global_min_range)
                and self.global_min_range < self.args.collision_range):
            if self.collision_t0 is None:
                self.collision_t0 = now
            if (now - self.collision_t0) >= self.args.collision_duration:
                return "ERROR_collision_risk"
        else:
            self.collision_t0 = None

        # 2. Coverage stable: no more frontier points (sustained for
        #    confirm_no_frontier seconds). NO min_duration gate — if the map
        #    is fully scanned in 60 s, the trial is done in 60 s.
        no_frontier_now = self._no_frontier_signal(now)
        if no_frontier_now:
            if self.no_frontier_since is None:
                self.no_frontier_since = now
        else:
            self.no_frontier_since = None
        if (self.no_frontier_since is not None
                and (now - self.no_frontier_since) >= self.args.confirm_no_frontier):
            return "DONE_coverage_stable"

        # 3. Stuck (count-based, only after min_duration).
        if self._is_stuck_window(now):
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        if (elapsed >= self.args.min_duration
                and self.stuck_count >= self.args.stuck_count_threshold):
            return "ERROR_stuck"

        # 4. Hard duration cap.
        if elapsed >= self.args.duration:
            return "DONE_duration"

        return None

    # ---------------- Snapshot ----------------

    def _snapshot(self, now: float, elapsed: float,
                  stop_reason: Optional[str]) -> dict:
        m = self.last_metrics
        no_frontier_for = (
            (now - self.no_frontier_since) if self.no_frontier_since else 0.0)
        last_frontier_age = (
            (now - self.last_frontier_msg_time)
            if self.last_frontier_msg_time else None)
        return {
            "timestamp": now,
            "elapsed_sec": round(elapsed, 2),
            "stop_reason": stop_reason,
            "search_attempt_count": self.search_attempts,
            "frontier_cells": self.frontier_count,
            "min_frontier_cells": self.args.min_frontier_cells,
            "has_seen_frontier": self.has_seen_frontier,
            "last_frontier_age_sec": (
                round(last_frontier_age, 2) if last_frontier_age is not None else None),
            "no_frontier_for_sec": round(no_frontier_for, 2),
            "stuck_count": self.stuck_count,
            "stuck_count_threshold": self.args.stuck_count_threshold,
            "global_min_range": (
                round(self.global_min_range, 4)
                if math.isfinite(self.global_min_range) else None),
            "collision_for_sec": (
                round(now - self.collision_t0, 2) if self.collision_t0 else 0.0),
            "global_explored_ratio": (m.global_explored_ratio if m else None),
            "global_unknown_ratio":  (m.global_unknown_ratio  if m else None),
            "active_explored_ratio": (m.active_explored_ratio if m else None),
            "active_unknown_ratio":  (m.active_unknown_ratio  if m else None),
            "explored_area_m2":      (m.explored_area_m2      if m else None),
            "explored_area_growth_m2": (m.explored_area_growth_m2 if m else None),
            "active_bbox_area_m2":   (m.active_bbox_area_m2   if m else None),
            "active_bbox_growth_m2": (m.active_bbox_growth_m2 if m else None),
        }

    def _tick(self, _evt):
        if rospy.is_shutdown():
            return
        now = time.time()
        elapsed = now - self.t_start
        stop_reason = self._decide(now, elapsed)
        snap = self._snapshot(now, elapsed, stop_reason)

        # Atomic state write.
        tmp = self.state_file.with_suffix(self.state_file.suffix + ".tmp")
        with tmp.open("w") as fh:
            json.dump(snap, fh, indent=2, sort_keys=True)
        os.replace(tmp, self.state_file)

        if self.log_file:
            with self.log_file.open("a") as fh:
                fh.write(json.dumps(snap, sort_keys=True) + "\n")

        # Per-tick visible progress line — flushed so it shows up in trial.log.
        gminr = (f"{self.global_min_range:.2f}"
                 if math.isfinite(self.global_min_range) else "inf")
        fcount = (str(self.frontier_count)
                  if self.frontier_count is not None else "?")
        seen = "Y" if self.has_seen_frontier else "N"
        print(
            f"[mon t={int(elapsed):4d}s] "
            f"stuck_cnt={self.stuck_count}/{self.args.stuck_count_threshold} "
            f"frontier_cells={fcount} (min={self.args.min_frontier_cells} seen={seen}) "
            f"no_frontier_for={int(snap['no_frontier_for_sec'])}s "
            f"search_attempts={self.search_attempts} "
            f"min_range={gminr}m "
            f"collision_for={int(snap['collision_for_sec'])}s "
            f"reason={stop_reason or '-'}",
            flush=True)

        if stop_reason is not None:
            print(f"STOP: {stop_reason} at t={elapsed:.1f}s", flush=True)
            rospy.loginfo("Stop condition fired: %s at %.1fs",
                          stop_reason, elapsed)
            rospy.signal_shutdown(stop_reason)


def main() -> int:
    rospy.init_node("frontier_exploration_monitor", anonymous=False)
    args = parse_args()
    mon = FrontierMonitor(args)
    rospy.spin()

    final = {}
    try:
        final = json.loads(Path(args.state_file).read_text())
    except Exception:
        pass
    sr = (final.get("stop_reason") if final else None)
    if isinstance(sr, str) and sr.startswith("ERROR"):
        return 2
    if isinstance(sr, str) and sr.startswith("DONE"):
        return 0
    return 1


if __name__ == "__main__":
    sys.exit(main())
