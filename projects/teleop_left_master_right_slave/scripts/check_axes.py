#!/usr/bin/env python3
"""Show which slave axis responds to each master axis, and how well it tracks.

Run this with all three terminals up, then move the master along ONE physical
direction at a time and watch which row lights up.

Two different faults look similar from the chair and this separates them:

  wrong mapping   the slave moves, but on a different axis than you pushed.
                  Fix with AXIS_MAP in terminal 3.

  cannot track    the slave is commanded correctly but does not get there.
                  follow ratio stays low on that axis alone. Usually the
                  vertical axis, where the spring has to lift the arm's own
                  weight. Fix with more slave stiffness, more force headroom,
                  or gravity compensation -- not with AXIS_MAP.
"""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

AXES = ("x", "y", "z")


def verdict_for(cmd: float, act: float, min_travel: float) -> tuple[float, str]:
    """Classify one axis from its commanded and actual travel.

    Separated out so the classification can be tested without a live graph.
    """
    ratio = (act / cmd) if abs(cmd) > 1e-5 else 0.0
    if abs(cmd) < min_travel * 0.25:
        return ratio, ""
    if ratio > 0.7:
        return ratio, "tracking"
    if ratio > 0.2:
        return ratio, "PARTIAL - cannot reach commanded target"
    if ratio > -0.2:
        return ratio, "STUCK - commanded but not moving"
    return ratio, "INVERTED - flip this axis in AXIS_MAP"


class AxisCheck(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("touch_axis_check")
        self.args = args
        self.left0: list[float] | None = None
        self.right0: list[float] | None = None
        self.tgt0: list[float] | None = None
        self.left = [0.0] * 3
        self.right = [0.0] * 3
        self.tgt = [0.0] * 3
        self.peak_master = [0.0] * 3
        self.rows = 0

        self.create_subscription(PoseStamped, args.left_pose_topic, self._l, 10)
        self.create_subscription(PoseStamped, args.right_pose_topic, self._r, 10)
        self.create_subscription(PoseStamped, args.right_target_topic, self._t, 10)
        self.create_timer(1.0 / max(args.rate_hz, 0.1), self._tick)

    @staticmethod
    def _xyz(m: PoseStamped) -> list[float]:
        return [m.pose.position.x, m.pose.position.y, m.pose.position.z]

    def _l(self, m):
        v = self._xyz(m)
        if self.left0 is None:
            self.left0 = list(v)
        self.left = [v[i] - self.left0[i] for i in range(3)]

    def _r(self, m):
        v = self._xyz(m)
        if self.right0 is None:
            self.right0 = list(v)
        self.right = [v[i] - self.right0[i] for i in range(3)]

    def _t(self, m):
        v = self._xyz(m)
        if self.tgt0 is None:
            self.tgt0 = list(v)
        self.tgt = [v[i] - self.tgt0[i] for i in range(3)]

    def reset(self) -> None:
        self.left0 = self.right0 = self.tgt0 = None
        self.peak_master = [0.0] * 3

    def _tick(self) -> None:
        if self.left0 is None or self.right0 is None:
            self.get_logger().warn("waiting for both device poses", once=True)
            return

        for i in range(3):
            self.peak_master[i] = max(self.peak_master[i], abs(self.left[i]))

        moving = max(self.peak_master)
        if moving < self.args.min_travel_m:
            if self.rows % 10 == 0:
                print(f"move the master further; largest travel so far "
                      f"{moving*1000:5.1f} mm of {self.args.min_travel_m*1000:.0f} mm needed",
                      flush=True)
            self.rows += 1
            return

        driven = max(range(3), key=lambda i: self.peak_master[i])

        if self.rows % self.args.header_every == 0:
            print(f"\nmaster travel is mostly on {AXES[driven].upper()}"
                  f"  ({self.left[driven]*1000:+.1f} mm)")
            print(f"{'slave axis':>11}{'commanded':>12}{'actual':>10}{'follow':>9}   verdict")
            print("-" * 68)

        for i in range(3):
            cmd = self.tgt[i]
            act = self.right[i]
            ratio, verdict = verdict_for(cmd, act, self.args.min_travel_m)
            star = " <-" if i == driven else "   "
            print(f"{AXES[i].upper():>11}{cmd*1000:+10.1f}mm{act*1000:+8.1f}mm"
                  f"{ratio:+8.2f}  {verdict}{star}", flush=True)
        self.rows += 1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--left-pose-topic", default="/touch_left/pose")
    p.add_argument("--right-pose-topic", default="/touch_right/pose")
    p.add_argument("--right-target-topic", default="/touch_right/target_pose")
    p.add_argument("--rate-hz", type=float, default=2.0)
    p.add_argument("--header-every", type=int, default=6)
    p.add_argument("--min-travel-m", type=float, default=0.015,
                   help="Ignore samples until the master has moved at least this far.")
    return p.parse_args()


def main() -> None:
    rclpy.init()
    node = AxisCheck(parse_args())
    print("Move the master along ONE physical direction at a time.")
    print("Watch the follow column: 1.0 is perfect, 0 is stuck, negative is inverted.\n")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # On SIGTERM rclpy may tear the context down before spin unwinds, which
        # surfaces as RCLError rather than ExternalShutdownException. Either way
        # it is a signal, not a fault worth a traceback.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
