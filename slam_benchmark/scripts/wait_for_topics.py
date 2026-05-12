#!/usr/bin/env python3
"""Wait for one or more ROS topics to publish at least one message.

All topics are subscribed in parallel under a single node, so there's no
per-topic TCP/master setup cost. Exits 0 if every topic produces a message
before the global deadline, 1 otherwise. Prints a per-topic ready/MISSING
summary either way.

Usage:
  wait_for_topics.py --timeout 180 /lidar/scan /odom /map /move_base/status
"""

import argparse
import sys
import time
from threading import Event

import rospy
from rospy import AnyMsg


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--timeout", type=float, default=120.0,
                    help="seconds before giving up on the slowest topic")
    ap.add_argument("topics", nargs="+")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("wait_for_topics", anonymous=True, disable_signals=False)

    events = {t: Event() for t in args.topics}

    def make_cb(topic):
        def cb(_msg):
            if not events[topic].is_set():
                events[topic].set()
                rospy.loginfo("  ready: %s", topic)
        return cb

    for t in args.topics:
        rospy.Subscriber(t, AnyMsg, make_cb(t), queue_size=1)

    deadline = time.time() + args.timeout
    while time.time() < deadline and not rospy.is_shutdown():
        if all(e.is_set() for e in events.values()):
            print("ALL READY")
            for t in args.topics:
                print(f"  ready: {t}")
            return 0
        time.sleep(0.5)

    print("TIMEOUT after %.1fs" % args.timeout)
    rc = 0 if all(e.is_set() for e in events.values()) else 1
    for t in args.topics:
        status = "ready" if events[t].is_set() else "MISSING"
        print(f"  {status}: {t}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
