#!/usr/bin/env python3
"""Part 2 (revised per 2026-09-09 correction): MASTER-ONLY smooth +Y motion,
bilateral coupling OFF, NO bridge/planner involved -- drives
/touch_left/target_pose directly against the master's own native controller
(touch_slave.cpp: K=15 N/m, D=0.10 Ns/m, Fmax=2.0N, max_position_error_m=
0.03m), matching how that controller already works with no new code.

DOES NOT USE REF_POSE_M (stale, hand-held, ~109.5mm from the live master as
of 2026-09-09 -- see live_reference_watch.py's updated docstring and
PREP_REPORT.md). Instead:

  1. Confirms the master telemetry stream is live and fresh.
  2. LATCHES the master's current live pose ONCE, atomically (single
     telemetry sample), as (x0, y0, z0) -- the origin for this test.
  3. Ramps target_pose SMOOTHLY (raised-cosine profile, not a step) from
     (x0, y0, z0) to (x0, y0 + DY_M, z0) over RAMP_DURATION_S, holding X
     and Z at their latched values throughout.
  4. Holds at the +DY_M target for HOLD_DURATION_S.
  5. Ramps back down to (x0, y0, z0) over RAMP_DURATION_S.
  6. Self-monitors every tick against the abort table in PREP_REPORT.md
     Part 6 and immediately republishes the LATCHED pose (not force=0 --
     touch_slave has no direct "zero force" command short of enable=false,
     and disabling mid-motion is itself a discontinuity; holding at the
     already-reached, already-safe pose is the smoother abort) and sets
     enable=false the instant any condition trips.

REQUIRES (not yet satisfied, not started by this script):
  - require_deadman:=true on the master node (this script does not restart
    nodes -- see PREP_REPORT.md Part 9).
  - DY_M filled in below (currently None -- deliberately unset, see
    PREP_REPORT.md Part 5 for why no validated number carries over
    directly from the simulation gap matrix / ramp-step corpus).
  - Explicit human confirmation of physical supervision + reachable E-stop
    for EACH run.

Usage (only after all of the above; NOT run yet):
    python3 part2_master_only_y_motion.py --out-dir <session_dir>
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time

from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from touch_haptics.msg import CartesianControllerTelemetry

sys.path.insert(0, str(Path(__file__).parent))
from frozen_config import FROZEN  # noqa: E402

# ---------------------------------------------------------------------------
# DELIBERATELY UNSET. Per the task's explicit "if no validated displacement
# exists, report that and stop": the simulation gap matrix (20/50/80/117/
# 140mm) validated the BILATERAL force-budget-planner control path, not this
# script's standalone master-only native-PD path -- not a direct match. The
# per-tick ramp step (~1-2mm) is a smoothness property, not a total
# displacement. Fill this in only after an explicit decision (see
# PREP_REPORT.md Part 5) -- do not invent a number here.
# ---------------------------------------------------------------------------
DY_M = None  # e.g. 0.010 for 10mm -- MUST be set explicitly before running

MASTER_TELEM_TOPIC = "/touch_left/cartesian_controller_telemetry"
MASTER_TARGET_TOPIC = "/touch_left/target_pose"
MASTER_ENABLE_TOPIC = "/touch_left/enable"

RAMP_DURATION_S = 2.0     # smooth, not a step -- slower than the validated
                          # 1.0s bilateral coupling ramp since this is a
                          # bare native-PD command with no force-budget
                          # planner bisecting it
HOLD_DURATION_S = 1.0
PUBLISH_HZ = 100.0        # matches the bridge's own RATE_HZ convention

# Abort thresholds -- from frozen_config.py (item 6, one canonical table).
# This script has no bridge/planner dependency (touch_slave's own native
# controller only), but the CONTROLLER limits themselves (Fmax, position-
# error clamp) are the same physical device/config regardless.
MASTER_MAX_FORCE_N = FROZEN.master_max_force_n
MASTER_MAX_POSITION_ERROR_M = FROZEN.master_max_position_error_m
# Part 2's master gets NO gravity/aux compensation (frozen_config:
# left_gravity_comp_n=0.0, and this script doesn't run the bridge at all) --
# unlike Part 1, there is no "expected constant force" to exclude here, so a
# flat applied-force threshold is not conflating gravity support with a real
# anomaly the way Part 1's old 0.5N-flat check was. Still conservative,
# well under MASTER_MAX_FORCE_N=2.0.
ABORT_APPLIED_FORCE_N = 0.5
ABORT_APPLIED_FORCE_SUSTAIN_S = 0.5
TELEMETRY_STALE_S = FROZEN.force_budget_telemetry_stale_s


DISABLE_REPEAT_N = 5           # repeat enable=false this many times on abort
DISABLE_REPEAT_INTERVAL_S = 0.02  # ~50Hz, bounded ~0.1s total -- best-effort
                                  # ROS transport, repeating guards against a
                                  # single dropped message


def abort_sequence():
    """2026-09-10 fix (item 3): the exact, testable ORDERING of actions an
    abort must take -- pure, no ROS, no side effects -- so a test can assert
    the order without needing a live node. Returns a list of
    ('enable', False) / ('record_pose',) tuples, IN ORDER.

    Critical invariant proven by this function's shape alone: enable=False
    is repeated MULTIPLE times, ALL of them BEFORE record_pose, and there is
    NO 'target' action anywhere -- an abort must never publish a new
    (possibly reverse-pulling, if the device has already moved past the
    latched origin) target position. The device simply stops receiving
    force once disabled; whatever target was last in flight becomes
    irrelevant."""
    actions = [("enable", False)] * DISABLE_REPEAT_N
    actions.append(("record_pose",))
    return actions


def _smoothstep(u: float) -> float:
    """Raised-cosine, C1-continuous (zero velocity at both ends) --
    smoother than the bilateral planner's own linear ramp, appropriate for
    a bare native-PD path with no adaptive force-budget bisection."""
    u = max(0.0, min(1.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


class MasterOnlyYMotion(Node):
    def __init__(self, dy_m: float, out_dir: str):
        super().__init__("part2_master_only_y_motion")
        if dy_m is None:
            raise ValueError("DY_M is unset -- see this script's module docstring")
        self.dy_m = dy_m
        self.out_dir = out_dir
        self.latest_telem = None
        self.origin = None
        self.rows = []
        self.aborted = False
        self.abort_reason = None
        self._force_over_since = None
        self.target_pub = self.create_publisher(PoseStamped, MASTER_TARGET_TOPIC, 10)
        self.enable_pub = self.create_publisher(Bool, MASTER_ENABLE_TOPIC, 10)
        self.create_subscription(CartesianControllerTelemetry, MASTER_TELEM_TOPIC,
                                  self._on_telem, 50)

    def _on_telem(self, m):
        self.latest_telem = (time.time(), m)

    def wait_for_fresh_telemetry(self, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest_telem is not None and time.time() - self.latest_telem[0] < TELEMETRY_STALE_S:
                return True
        return False

    def latch_origin(self):
        """Atomic single-sample latch -- one telemetry message, taken once."""
        assert self.latest_telem is not None
        _, m = self.latest_telem
        self.origin = (m.position_m[0], m.position_m[1], m.position_m[2])
        return self.origin

    def _publish_target(self, xyz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "touch_left_base"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def _publish_enable(self, on: bool):
        msg = Bool()
        msg.data = on
        self.enable_pub.publish(msg)

    def _check_abort(self, now: float) -> bool:
        if self.latest_telem is None:
            return False
        t_recv, m = self.latest_telem
        if now - t_recv > TELEMETRY_STALE_S:
            self.abort_reason = f"telemetry stale ({now - t_recv:.3f}s)"
            return True
        if m.position_error_clamped:
            self.abort_reason = "position_error_clamped"
            return True
        if m.primary_force_clamped or m.final_force_clamped:
            self.abort_reason = "force clamped"
            return True
        applied_norm = math.sqrt(sum(v * v for v in m.applied_force_n))
        if applied_norm > ABORT_APPLIED_FORCE_N:
            if self._force_over_since is None:
                self._force_over_since = now
            elif now - self._force_over_since > ABORT_APPLIED_FORCE_SUSTAIN_S:
                self.abort_reason = f"applied force {applied_norm:.3f}N sustained > {ABORT_APPLIED_FORCE_N}N"
                return True
        else:
            self._force_over_since = None
        return False

    def _record(self, phase, xyz_target=None):
        if self.latest_telem is None:
            return
        _, m = self.latest_telem
        tx, ty, tz = xyz_target if xyz_target is not None else (None, None, None)
        self.rows.append({
            "t": time.time(), "phase": phase,
            "target_x": tx, "target_y": ty, "target_z": tz,
            "x": m.position_m[0], "y": m.position_m[1], "z": m.position_m[2],
            "vx": m.velocity_mps[0], "vy": m.velocity_mps[1], "vz": m.velocity_mps[2],
            "err_x": m.position_error_raw_m[0], "err_y": m.position_error_raw_m[1], "err_z": m.position_error_raw_m[2],
            "spring_x": m.spring_force_n[0], "spring_y": m.spring_force_n[1], "spring_z": m.spring_force_n[2],
            "damping_x": m.damping_force_n[0], "damping_y": m.damping_force_n[1], "damping_z": m.damping_force_n[2],
            "aux_y": m.auxiliary_force_n[1],
            "applied_x": m.applied_force_n[0], "applied_y": m.applied_force_n[1], "applied_z": m.applied_force_n[2],
            "pos_clamped": bool(m.position_error_clamped),
            "primary_clamped": bool(m.primary_force_clamped),
            "final_clamped": bool(m.final_force_clamped),
        })

    def _do_abort(self):
        """Executes abort_sequence() -- disable FIRST (repeated), record
        the pose AT abort, NEVER publish a new target. See abort_sequence()'s
        docstring for why this ordering matters (avoiding a reverse spring
        pull toward a stale origin if the device has already moved)."""
        for action in abort_sequence():
            if action[0] == "enable":
                self._publish_enable(action[1])
                time.sleep(DISABLE_REPEAT_INTERVAL_S)
            elif action[0] == "record_pose":
                self._record("ABORT")  # xyz_target=None -- see _record()

    def run(self):
        if not self.wait_for_fresh_telemetry():
            print("[part2][ERROR] no fresh master telemetry -- aborting before any command", flush=True)
            return False
        x0, y0, z0 = self.latch_origin()
        print(f"[part2] LATCHED origin (single atomic sample): x0={x0:.6f} y0={y0:.6f} z0={z0:.6f}", flush=True)
        print(f"[part2] target: y0 + DY_M = {y0 + self.dy_m:.6f} (DY_M={self.dy_m:.6f}m = {self.dy_m*1000:.2f}mm, +Y = physically UP per FOUR_CHANNEL_TODO.md / Y_AXIS_DIAGNOSIS.md)", flush=True)

        # NOTE: enable=true is required for force to apply -- if the node
        # is (correctly, per Part 9) relaunched with require_deadman:=true,
        # force ALSO requires the physical stylus button held throughout,
        # regardless of this script's enable command.
        self._publish_enable(True)

        dt = 1.0 / PUBLISH_HZ
        phases = [
            ("RAMP_UP", RAMP_DURATION_S, lambda u: y0 + self.dy_m * _smoothstep(u)),
            ("HOLD", HOLD_DURATION_S, lambda u: y0 + self.dy_m),
            ("RAMP_DOWN", RAMP_DURATION_S, lambda u: y0 + self.dy_m * (1.0 - _smoothstep(u))),
        ]
        for phase_name, duration, y_fn in phases:
            t_start = time.time()
            while True:
                now = time.time()
                u = (now - t_start) / duration
                rclpy.spin_once(self, timeout_sec=0.0)
                if self._check_abort(now):
                    print(f"[part2][ABORT] {self.abort_reason} -- disabling (NOT re-targeting)", flush=True)
                    self.aborted = True
                    self._do_abort()
                    return False
                y_cmd = y_fn(min(1.0, u))
                target = (x0, y_cmd, z0)
                self._publish_target(target)
                self._record(phase_name, target)
                if u >= 1.0:
                    break
                time.sleep(dt)
        self._publish_enable(False)
        print("[part2] motion sequence complete, disabled.", flush=True)
        return True

    def save(self):
        with open(f"{self.out_dir}/part2_telemetry.csv", "w", newline="") as f:
            if not self.rows:
                return
            w = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            w.writeheader()
            w.writerows(self.rows)
        with open(f"{self.out_dir}/part2_meta.json", "w") as f:
            json.dump({"dy_m": self.dy_m, "ramp_duration_s": RAMP_DURATION_S,
                      "hold_duration_s": HOLD_DURATION_S, "aborted": self.aborted,
                      "abort_reason": self.abort_reason}, f, indent=2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--dy-m", type=float, default=DY_M)
    args = ap.parse_args()
    if args.dy_m is None:
        print("[part2][ERROR] --dy-m not provided and DY_M is unset in-script -- "
              "see module docstring, refusing to guess a displacement.", flush=True)
        sys.exit(2)

    rclpy.init()
    node = MasterOnlyYMotion(args.dy_m, args.out_dir)
    try:
        ok = node.run()
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
