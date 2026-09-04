#!/usr/bin/env python3
"""Print one PoseStamped's position as "x y z" (metres) to stdout, then exit.

Used by run_master_left_sim.sh / run_slave_right_sim.sh to seed the sim
twin's initial joint state at the real device's actual current Cartesian
position (via ArmModel's has_init_pose_m / TOUCH_SIM_INIT_POS_*), instead of
its hardcoded default reset pose -- see arm_model.hpp for why that mismatch
mattered.

Usage:
    python3 read_current_pose.py /touch_right/pose --timeout 5

On timeout (real device not up / not yet publishing), exits non-zero and
prints nothing to stdout, so the caller can fall back to the sim's default
reset pose rather than seeding from a stale/absent read.
"""
from __future__ import annotations

import argparse
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--timeout", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = Node("read_current_pose")
    got: list[PoseStamped] = []
    node.create_subscription(PoseStamped, args.topic, lambda m: got.append(m), 10)
    end = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while rclpy.ok() and not got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    if not got:
        print(f"read_current_pose: no message on {args.topic} within "
              f"{args.timeout}s", file=sys.stderr)
        return 1

    p = got[0].pose.position
    print(f"{p.x} {p.y} {p.z}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
