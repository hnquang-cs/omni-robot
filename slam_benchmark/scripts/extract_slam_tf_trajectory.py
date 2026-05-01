#!/usr/bin/env python3
"""Extract an estimated trajectory from /tf and /tf_static in a rosbag as TUM."""

import argparse
import bisect
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


Pose = Tuple[float, Tuple[float, float, float], Tuple[float, float, float, float]]


def load_rosbag():
    try:
        import rosbag  # type: ignore
        return rosbag
    except Exception as exc:
        raise RuntimeError(
            "Could not import rosbag. Source ROS Noetic before running this script."
        ) from exc


def norm_frame(frame: str) -> str:
    return frame[1:] if frame.startswith("/") else frame


def stamp_to_sec(stamp) -> float:
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    vx, vy, vz = v
    rotated = quat_mul(quat_mul(q, (vx, vy, vz, 0.0)), quat_conj(q))
    return rotated[:3]


def pose_inverse(pose: Pose) -> Pose:
    stamp, t, q = pose
    qi = quat_conj(q)
    rt = quat_rotate(qi, (-t[0], -t[1], -t[2]))
    return stamp, rt, qi


def pose_compose(a: Pose, b: Pose) -> Pose:
    stamp = max(a[0], b[0])
    ar = quat_rotate(a[2], b[1])
    t = (a[1][0] + ar[0], a[1][1] + ar[1], a[1][2] + ar[2])
    q = quat_mul(a[2], b[2])
    return stamp, t, q


def transform_to_pose(transform, bag_time) -> Pose:
    stamp = stamp_to_sec(transform.header.stamp)
    if stamp <= 0.0:
        stamp = stamp_to_sec(bag_time)
    tr = transform.transform.translation
    qr = transform.transform.rotation
    return stamp, (tr.x, tr.y, tr.z), (qr.x, qr.y, qr.z, qr.w)


def nearest_pose(series: List[Pose], time_sec: float, tolerance: float) -> Optional[Pose]:
    if not series:
        return None
    stamps = [p[0] for p in series]
    index = bisect.bisect_left(stamps, time_sec)
    candidates = []
    if index < len(series):
        candidates.append(series[index])
    if index > 0:
        candidates.append(series[index - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda p: abs(p[0] - time_sec))
    if abs(best[0] - time_sec) > tolerance:
        return None
    return best


def build_edge_lookup(bag_path: Path):
    rosbag = load_rosbag()
    dynamic: Dict[Tuple[str, str], List[Pose]] = defaultdict(list)
    static: Dict[Tuple[str, str], Pose] = {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, bag_time in bag.read_messages(topics=["/tf", "/tf_static"]):
            for transform in msg.transforms:
                parent = norm_frame(transform.header.frame_id)
                child = norm_frame(transform.child_frame_id)
                pose = transform_to_pose(transform, bag_time)
                if topic == "/tf_static":
                    static[(parent, child)] = pose
                else:
                    dynamic[(parent, child)].append(pose)
    for series in dynamic.values():
        series.sort(key=lambda p: p[0])
    return dynamic, static


def direct_at(dynamic, static, parent: str, child: str, time_sec: float) -> Optional[Pose]:
    if (parent, child) in dynamic:
        pose = nearest_pose(dynamic[(parent, child)], time_sec, 0.25)
        if pose:
            return pose
    if (parent, child) in static:
        return static[(parent, child)]
    if (child, parent) in dynamic:
        pose = nearest_pose(dynamic[(child, parent)], time_sec, 0.25)
        if pose:
            return pose_inverse(pose)
    if (child, parent) in static:
        return pose_inverse(static[(child, parent)])
    return None


def candidate_times(dynamic, parent: str, child: str) -> List[float]:
    for key in ((parent, child), (child, parent)):
        if key in dynamic:
            return [p[0] for p in dynamic[key]]
    times: List[float] = []
    for series in dynamic.values():
        times.extend(p[0] for p in series)
    return sorted(set(times))


def find_chain_pose(dynamic, static, parent: str, child: str, time_sec: float) -> Optional[Pose]:
    direct = direct_at(dynamic, static, parent, child, time_sec)
    if direct:
        return direct
    frames = sorted({f for edge in list(dynamic.keys()) + list(static.keys()) for f in edge})
    for mid in frames:
        if mid in (parent, child):
            continue
        first = direct_at(dynamic, static, parent, mid, time_sec)
        second = direct_at(dynamic, static, mid, child, time_sec)
        if first and second:
            return pose_compose(first, second)
    return None


def extract_tf(bag_path: Path, parent_frame: str, child_frame: str, output_path: Path) -> int:
    parent = norm_frame(parent_frame)
    child = norm_frame(child_frame)
    dynamic, static = build_edge_lookup(bag_path)
    times = candidate_times(dynamic, parent, child)
    if not times:
        raise RuntimeError("No dynamic TF samples found. Check that /tf is recorded.")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with output_path.open("w") as out:
        out.write("# timestamp tx ty tz qx qy qz qw\n")
        last_stamp = -math.inf
        for time_sec in times:
            pose = find_chain_pose(dynamic, static, parent, child, time_sec)
            if not pose:
                continue
            stamp, t, q = pose
            if stamp <= last_stamp:
                stamp = time_sec
            out.write(f"{stamp:.9f} {t[0]:.9f} {t[1]:.9f} {t[2]:.9f} {q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n")
            last_stamp = stamp
            count += 1
    if count == 0:
        raise RuntimeError(
            f"No transform found for {parent}->{child}. Try base_footprint/base_link or inspect with: rosrun tf tf_echo {parent} {child}"
        )
    print(f"PASS: wrote {count} estimated poses for {parent}->{child} to {output_path}")
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", type=Path)
    parser.add_argument("parent_frame")
    parser.add_argument("child_frame")
    parser.add_argument("output_path", type=Path)
    args = parser.parse_args()
    if not args.bag_path.is_file():
        print(f"FAIL: bag not found: {args.bag_path}", file=sys.stderr)
        return 1
    try:
        extract_tf(args.bag_path, args.parent_frame, args.child_frame, args.output_path)
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
