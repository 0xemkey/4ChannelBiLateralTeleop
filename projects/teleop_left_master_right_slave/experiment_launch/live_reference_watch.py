#!/usr/bin/env python3
"""Read-only live MASTER-vs-fixed-reference-pose monitor.

Subscribes to /touch_left/pose and /touch_left/joint_angles only.
Publishes nothing, commands nothing. Shows live master Cartesian XYZ and
q (waist/shoulder/elbow) against a hardcoded reference pose (extracted
from the Trial 2 free-space hold's stable final-3s window), and the
residual to that reference, so the operator can manually return the
master to (approximately) the same pose before each gravity-comp
condition.

2026-09-09 PROVENANCE (traced during Y-axis hardware-validation prep) --
REF_POSE_M was extracted from
experiment_runs/teleop_setup/20260909T100330Z_dob_observe/
h2abs_freespace_hold_trial2/_touch_left_cartesian_controller_telemetry.txt
(trial_start=2026-09-09T14:28:53+03:00, trial_meta.txt: "repeat of
freespace hold, same conditions, both devices lifted clear of docks,
MASTER HELD STILL BY HAND, slave free") -- the final samples in that file
read position_m=(0.0073919, 0.0005772, 0.0077866), matching REF_POSE_M to
sub-mm precision. This was an UNPLANNED, HAND-HELD pose during a DOB/
free-space trial for a completely different purpose, never an engineered
or safety-reviewed Cartesian target -- it happens to be wherever the
operator's hand was at that moment, not a designed reference.

STALE / DO NOT USE AS AN AUTOMATED MOTION TARGET: as of 2026-09-09, the
live master was measured ~109.5mm away from this pose (dominated by a
~92mm Z offset, not Y) -- it does not reflect where the master currently
sits, and there is no basis for commanding motion TOWARD it
automatically. This file remains a manual, informational, read-only aid
only (an operator visually returning their own hand to roughly the same
grip for a repeated-measures comparison) -- see
hardware_experiments/2026-09-09_y_axis_validation_PENDING_SUPERVISION/
for the Y-axis validation task that must NOT use REF_POSE_M as a
commanded target; that task uses a freshly live-latched start pose
instead.

Usage: python3 live_reference_watch.py
Ctrl+C to stop.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Vector3Stamped

REF_POSE_M = (0.00738771, 0.00043958, 0.00795783)
REF_Q_RAD = (-0.04187797, 0.71123996, 0.60134046)


class ReferenceWatch(Node):
    def __init__(self):
        super().__init__("live_reference_watch")
        self.pose = None
        self.q = None
        self.create_subscription(PoseStamped, "/touch_left/pose", self._on_pose, 10)
        self.create_subscription(Vector3Stamped, "/touch_left/joint_angles", self._on_q, 10)
        self.create_timer(0.2, self._print)

    def _on_pose(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, p.z)

    def _on_q(self, msg):
        v = msg.vector
        self.q = (v.x, v.y, v.z)

    def _print(self):
        if self.pose is None or self.q is None:
            print("waiting for /touch_left/pose and /touch_left/joint_angles ...")
            return
        dp = tuple((self.pose[i] - REF_POSE_M[i]) * 1000.0 for i in range(3))
        dq = tuple(self.q[i] - REF_Q_RAD[i] for i in range(3))
        sys.stdout.write("\033[H\033[J")
        print("LIVE MASTER vs FIXED REFERENCE POSE -- read-only, no commands published -- Ctrl+C to stop\n")
        print(f"{'':14s} {'x':>10s} {'y':>10s} {'z':>10s}   (Cartesian, m)")
        print(f"{'reference':14s} {REF_POSE_M[0]:10.5f} {REF_POSE_M[1]:10.5f} {REF_POSE_M[2]:10.5f}")
        print(f"{'current':14s} {self.pose[0]:10.5f} {self.pose[1]:10.5f} {self.pose[2]:10.5f}")
        print(f"{'delta (mm)':14s} {dp[0]:10.2f} {dp[1]:10.2f} {dp[2]:10.2f}")
        dp_norm = math.sqrt(sum(v * v for v in dp))
        print(f"\n||delta_p|| = {dp_norm:.2f} mm\n")
        print(f"{'':14s} {'waist':>10s} {'shoulder':>10s} {'elbow':>10s}   (rad)")
        print(f"{'reference':14s} {REF_Q_RAD[0]:10.4f} {REF_Q_RAD[1]:10.4f} {REF_Q_RAD[2]:10.4f}")
        print(f"{'current':14s} {self.q[0]:10.4f} {self.q[1]:10.4f} {self.q[2]:10.4f}")
        print(f"{'delta':14s} {dq[0]:10.4f} {dq[1]:10.4f} {dq[2]:10.4f}")
        dq_norm = math.sqrt(sum(v * v for v in dq))
        print(f"\n||delta_q|| = {dq_norm:.4f} rad")


def main():
    rclpy.init()
    node = ReferenceWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
