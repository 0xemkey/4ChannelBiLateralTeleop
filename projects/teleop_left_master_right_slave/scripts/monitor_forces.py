#!/usr/bin/env python3
"""Live view of the force actually rendered on both arms.

Reads /touch_<side>/applied_force from each device node. That topic carries what
the node really put on the motors this frame, after every clamp, so it is the
honest answer to "what is each arm doing right now" — unlike the bridge's own
report, which shows what it asked for before the hardware clamp.

Still a commanded quantity. Neither arm can measure force; see
docs/force-and-torque.html. The right arm's tracking error is the only column
here that reacts to the outside world.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

STALE_S = 0.5


class ForceMonitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("touch_force_monitor")
        self.args = args
        self.left: tuple[float, float, float] | None = None
        self.right: tuple[float, float, float] | None = None
        self.err: tuple[float, float, float] | None = None
        self.left_t = 0.0
        self.right_t = 0.0
        self.err_t = 0.0
        self.rows = 0

        self.left_back: tuple[float, float, float] | None = None
        self.right_back: tuple[float, float, float] | None = None
        self.max_drift = 0.0

        self.create_subscription(WrenchStamped, args.left_force_topic, self._left, 10)
        self.create_subscription(WrenchStamped, args.right_force_topic, self._right, 10)
        self.create_subscription(Vector3Stamped, args.tracking_error_topic, self._err, 10)
        self.create_subscription(WrenchStamped, args.left_readback_topic, self._left_back, 10)
        self.create_subscription(WrenchStamped, args.right_readback_topic, self._right_back, 10)
        self.create_timer(1.0 / max(args.rate_hz, 0.1), self._tick)

    def _left_back(self, m: WrenchStamped) -> None:
        self.left_back = (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)

    def _right_back(self, m: WrenchStamped) -> None:
        self.right_back = (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)

    def _left(self, m: WrenchStamped) -> None:
        self.left = (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)
        self.left_t = time.time()

    def _right(self, m: WrenchStamped) -> None:
        self.right = (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)
        self.right_t = time.time()

    def _err(self, m: Vector3Stamped) -> None:
        self.err = (m.vector.x, m.vector.y, m.vector.z)
        self.err_t = time.time()

    @staticmethod
    def _norm(v: tuple[float, float, float] | None) -> float:
        return math.sqrt(sum(x * x for x in v)) if v else 0.0

    def _cell(self, v, t, label):
        """One arm's column: xyz components and magnitude, or a staleness marker."""
        if v is None:
            return f"{label} {'waiting':>29}"
        if time.time() - t > STALE_S:
            return f"{label} {'STALE (node stopped?)':>29}"
        return (
            f"{label} "
            f"{v[0]:+6.3f} {v[1]:+6.3f} {v[2]:+6.3f} "
            f"| {self._norm(v):5.3f} N"
        )

    def _tick(self) -> None:
        if self.args.header_every > 0 and self.rows % self.args.header_every == 0:
            head = (
                f"{'':6}{'fx':>7}{'fy':>7}{'fz':>7}{'':3}{'|F|':>7}     "
                f"{'':6}{'fx':>7}{'fy':>7}{'fz':>7}{'':3}{'|F|':>7}      "
                f"{'track_err':>10}{'est_load':>10}"
            )
            print("\n" + head, flush=True)
            print("-" * len(head), flush=True)
        self.rows += 1

        line = self._cell(self.left, self.left_t, "LEFT ")
        line += "   " + self._cell(self.right, self.right_t, "RIGHT")

        if self.err is not None and time.time() - self.err_t <= STALE_S:
            e = self._norm(self.err)
            line += f"   {e:8.4f} m {e * self.args.stiffness_n_per_m:7.3f} N"
            if e * self.args.stiffness_n_per_m > self.args.load_flag_n:
                line += "  <-- resisted"
        else:
            line += f"   {'--':>8}   {'--':>7}"

        # Gap between what we set and what the driver actually shipped one frame
        # later. Nonzero means MAX_FORCE_CLAMPING, FORCE_RAMPING, SOFTWARE_FORCE_LIMIT
        # or ONE_FRAME_LIMIT is rewriting the command underneath us.
        drift = 0.0
        for sent, back in ((self.left, self.left_back), (self.right, self.right_back)):
            if sent is not None and back is not None:
                drift = max(drift, abs(self._norm(sent) - self._norm(back)))
        if self.left_back is not None or self.right_back is not None:
            self.max_drift = max(self.max_drift, drift)
            flag = "  <-- DRIVER MODIFYING" if drift > self.args.drift_flag_n else ""
            line += f"  drift {drift:5.3f}N (peak {self.max_drift:5.3f}){flag}"

        print(line, flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--left-force-topic", default="/touch_left/applied_force")
    p.add_argument("--right-force-topic", default="/touch_right/applied_force")
    p.add_argument("--tracking-error-topic", default="/touch_right/tracking_error")
    p.add_argument("--rate-hz", type=float, default=5.0)
    p.add_argument("--header-every", type=int, default=20,
                   help="Reprint the column header every N rows. 0 disables.")
    p.add_argument("--stiffness-n-per-m", type=float, default=35.0,
                   help="Right node spring gain, used only to turn tracking error into a load estimate.")
    p.add_argument("--load-flag-n", type=float, default=0.15,
                   help="Mark the row when the estimated load exceeds this.")
    p.add_argument("--left-readback-topic", default="/touch_left/last_force")
    p.add_argument("--right-readback-topic", default="/touch_right/last_force")
    p.add_argument("--drift-flag-n", type=float, default=0.02,
                   help="Flag when set-vs-shipped force differs by more than this.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ForceMonitor(args)
    print("[force_monitor] both columns are COMMANDED force after clamping.")
    print("[force_monitor] this hardware has no force sensor; track_err is the only")
    print("[force_monitor] column that responds to the outside world.")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
