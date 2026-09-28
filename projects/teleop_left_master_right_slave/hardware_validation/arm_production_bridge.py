#!/usr/bin/env python3
"""Production bridge arm sequencer (2026-09-10).

FIXES A REAL, CONFIRMED PRODUCTION BUG: `experiment_launch/03_start_position_
bridge.sh --arm-position` has NEVER actually set `arm_teleop=true` on the
bridge -- `--arm-position` was only ever a shell-level operator-confirmation
flag gating whether the launcher proceeds to start the bridge PROCESS
(`start_component bridge ...`); nothing in any `experiment_launch/*.sh`
script (confirmed by grep -- zero matches) ever called
`ros2 param set /touch_master_slave_bridge arm_teleop true`. Confirmed live
on real hardware, 2026-09-10: bridge process alive, C3 configured,
`/touch_right/enable == true`, but `ros2 param get /touch_master_slave_bridge
arm_teleop` returned False, `bridge_state` stayed READY forever, and moving
the master produced zero slave motion (coupling_ramp_alpha stuck at 0).

This script is the missing step. It performs the SAME proven, non-blocking
arm sequence H2's `part1_gravity_settle_capture.py` uses (imported directly,
not reimplemented): pre-launch single-instance check, publisher-ownership
check, a bounded fresh pre-arm window, an ASYNC `SetParameters` service call
(not a blocking `ros2 param set` CLI subprocess, which would starve this
same node's own telemetry callbacks), a post-arm freshness gate, and then
watches the state machine actually progress (READY -> GRAVITY_RAMP_UP ->
GRAVITY_SETTLE -> COUPLING_RAMP_UP -> ACTIVE, traced from
startup_alignment.py -- see PROGRESSION_STATES below) rather than assuming
arming succeeded just because the parameter call returned success.

UNLIKE H2's script, this one does NOT disarm on success -- the whole point
is to leave production teleoperation ACTIVE for the operator to use. It
DOES disarm (best-effort) and exit nonzero on any fail-closed condition.

This is orchestration only. No controller gains, gravity compensation, DOB
parameters, channel weights, or force limits are touched here -- this
script only ever sets the single `arm_teleop` boolean parameter, exactly
as the (currently missing) production step should have been doing all
along.

Does NOT require a deadman -- production teleoperation has never used one
(confirmed: run_bridge.sh never passes --require-master-deadman, whose own
CLI/FROZEN default is False). That requirement is H2-validation-only and
is entirely separate from this script.

Usage (called by experiment_launch/03_start_position_bridge.sh after the
bridge process is confirmed started -- not normally run standalone):

    python3 arm_production_bridge.py --expected-c3 true
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
import pre_arm_check  # noqa: E402
from frozen_config import FROZEN  # noqa: E402

H2_SCRIPT_DIR = (Path(__file__).resolve().parents[1] / "hardware_experiments"
                / "2026-09-09_y_axis_validation_PENDING_SUPERVISION")
sys.path.insert(0, str(H2_SCRIPT_DIR))
from part1_gravity_settle_capture import _set_bool_param_async, ARM_PARAM_TIMEOUT_S  # noqa: E402

# Traced directly from startup_alignment.py (2026-09-10): READY's own tick()
# branch calls _begin_phase(GRAVITY_RAMP_UP) on arm; GRAVITY_RAMP_UP ->
# GRAVITY_SETTLE once gravity_ramp_alpha>=1.0; GRAVITY_SETTLE -> COUPLING_
# RAMP_UP once the quiet-hold gate passes (hold_mode is an H2-only option,
# never set here); COUPLING_RAMP_UP -> ACTIVE via _transition(ACTIVE) once
# coupling_ramp_alpha>=1.0. Any state -> FAULT. Not assumed -- read from
# source for this exact task.
PROGRESSION_STATES = ["READY", "GRAVITY_RAMP_UP", "GRAVITY_SETTLE", "COUPLING_RAMP_UP", "ACTIVE"]

DEFAULT_LEFT_TELEM_TOPIC = "/touch_left/cartesian_controller_telemetry"
DEFAULT_RIGHT_TELEM_TOPIC = "/touch_right/cartesian_controller_telemetry"
DEFAULT_DIAG_TOPIC = "/bridge_diagnostics"


class BridgeArmNode(Node):
    def __init__(self, left_telem_topic: str, right_telem_topic: str, diag_topic: str):
        super().__init__("hardware_validation_arm_production_bridge")
        self.rows = {"left": [], "right": [], "bridge": []}
        self.latest = {}
        self.phase = None
        self.create_subscription(CartesianControllerTelemetry, left_telem_topic,
                                 lambda m: self._on_telem("left", m), 50)
        self.create_subscription(CartesianControllerTelemetry, right_telem_topic,
                                 lambda m: self._on_telem("right", m), 50)
        self.create_subscription(BridgeDiagnostics, diag_topic, self._on_diag, 50)
        self._param_client = self.create_client(SetParameters, "/touch_master_slave_bridge/set_parameters")

    def _on_telem(self, side, m):
        t = time.time()
        self.latest[side] = (t, m)
        self.rows[side].append({
            "t": t, "enabled": bool(m.enabled),
            "applied_force_n_x": m.applied_force_n[0], "applied_force_n_y": m.applied_force_n[1],
            "applied_force_n_z": m.applied_force_n[2],
            "position_error_clamped": bool(m.position_error_clamped),
            "primary_force_clamped": bool(m.primary_force_clamped),
            "final_force_clamped": bool(m.final_force_clamped),
        })

    def _on_diag(self, m):
        t = time.time()
        if m.bridge_state != self.phase:
            print(f"[arm] bridge_state -> {m.bridge_state} (gravity_ramp_alpha={m.gravity_ramp_alpha:.3f} "
                  f"coupling_ramp_alpha={m.coupling_ramp_alpha:.4f})", flush=True)
        self.phase = m.bridge_state
        self.latest["diag"] = (t, m)
        self.rows["bridge"].append({
            "t": t, "bridge_state": m.bridge_state, "coupling_ramp_alpha": m.coupling_ramp_alpha,
            "c1_enabled": bool(m.c1_enabled), "c2_enabled": bool(m.c2_enabled),
            "c3_enabled": bool(m.c3_enabled), "c4_enabled": bool(m.c4_enabled),
        })

    def set_arm_teleop(self, value: bool, timeout_s: float = ARM_PARAM_TIMEOUT_S):
        return _set_bool_param_async(self, self._param_client, "arm_teleop", value, timeout_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--left-telemetry-topic", default=DEFAULT_LEFT_TELEM_TOPIC)
    ap.add_argument("--right-telemetry-topic", default=DEFAULT_RIGHT_TELEM_TOPIC)
    ap.add_argument("--bridge-diagnostics-topic", default=DEFAULT_DIAG_TOPIC)
    ap.add_argument("--expected-c1", default="false")
    ap.add_argument("--expected-c2", default="false")
    ap.add_argument("--expected-c3", default="true")
    ap.add_argument("--expected-c4", default="false")
    ap.add_argument("--window-s", type=float, default=3.0)
    ap.add_argument("--target-samples", type=int, default=20)
    ap.add_argument("--min-samples", type=int, default=5)
    ap.add_argument("--force-tolerance-n", type=float, default=0.05)
    ap.add_argument("--progression-timeout-s", type=float, default=20.0,
                    help="max time to wait for READY->...->ACTIVE after arming")
    ap.add_argument("--post-arm-freshness-window-s", type=float, default=2.0)
    ap.add_argument("--ready-wait-timeout-s", type=float, default=30.0,
                    help="max time to wait for the bridge to reach READY (calibration/anchor "
                         "capture) before this script gives up -- this script may be invoked "
                         "immediately after the bridge process starts, well before READY")
    args = ap.parse_args()

    expected_channels = {
        "c1": args.expected_c1.lower() == "true", "c2": args.expected_c2.lower() == "true",
        "c3": args.expected_c3.lower() == "true", "c4": args.expected_c4.lower() == "true",
    }
    stale_s = FROZEN.force_budget_telemetry_stale_s

    print("================ production bridge arm sequence ================", flush=True)
    print(f"expected_channels={expected_channels}  telemetry_stale_s={stale_s}", flush=True)
    print(f"expected progression: {' -> '.join(PROGRESSION_STATES)}  (any state -> FAULT)", flush=True)
    print("===================================================================", flush=True)

    rclpy.init()
    node = BridgeArmNode(args.left_telemetry_topic, args.right_telemetry_topic, args.bridge_diagnostics_topic)

    def fail_closed(reason: str, extra=None):
        print(f"[arm][FAIL CLOSED] {reason}", flush=True)
        if extra is not None:
            print(f"[arm] details: {extra}", flush=True)
        print("[arm] disarming (best-effort) and exiting nonzero -- NOT reporting teleoperation active.",
             flush=True)
        try:
            node.set_arm_teleop(False)
        except Exception as exc:
            print(f"[arm][WARNING] disarm attempt itself raised {exc!r}", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # ---- Step 1: exactly one bridge instance, expected publisher ownership ----
    deadline = time.time() + 1.5
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    instance_check = pre_arm_check.check_single_bridge_node_instance(node)
    print(f"[arm] single-bridge-instance check: {instance_check}", flush=True)
    if not instance_check["ok"]:
        fail_closed("single-bridge-instance check failed", instance_check)

    ownership = pre_arm_check.check_enable_publisher_ownership(node)
    print(f"[arm] publisher ownership: {ownership}", flush=True)
    if not ownership["ok"]:
        fail_closed("publisher ownership check failed", ownership)

    # ---- Step 1.5: wait for the bridge to reach READY ----
    # This script may run immediately after `start_component bridge` starts
    # the process -- well before calibration/anchor-capture (WAIT_FOR_POSES
    # -> HOLD_STILL -> CAPTURE_ANCHORS -> VALIDATE_ALIGNMENT) completes.
    # Arming (setting arm_teleop=true) while still in an earlier state is
    # not itself unsafe (startup_alignment.py's READY branch only consumes
    # the pending arm request once it actually reaches READY), but this
    # script waits explicitly so its own pre-arm window/freshness checks
    # are evaluated against a genuinely stable READY state, not a
    # mid-calibration transient.
    ready_deadline = time.time() + args.ready_wait_timeout_s
    while time.time() < ready_deadline and node.phase != "READY":
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.phase == "FAULT":
            fail_closed("bridge entered FAULT while waiting to reach READY")
    if node.phase != "READY":
        fail_closed(f"bridge did not reach READY within {args.ready_wait_timeout_s}s "
                    f"(last observed state={node.phase})")
    print(f"[arm] bridge confirmed in READY.", flush=True)

    # ---- Step 2: bounded fresh pre-arm window ----
    node.rows["left"].clear()
    node.rows["right"].clear()
    node.rows["bridge"].clear()
    window_start_t = time.time()
    deadline = time.time() + args.window_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (len(node.rows["left"]) >= args.target_samples and len(node.rows["right"]) >= args.target_samples
                and len(node.rows["bridge"]) >= args.target_samples):
            break

    # require_enabled_false=False: production runs with no deadman gate
    # (require_master_deadman=False), under which bridge_master_slave.py
    # publishes enable=True UNCONDITIONALLY from the moment it starts,
    # regardless of arm state (deadman_gate_state()'s own `if not
    # require: return True, ...`) -- confirmed on the real hardware run
    # this task investigates. That is the expected, harmless production
    # state, not a stale latch -- H2's own stricter requirement
    # (enabled must read False pre-arm) does not apply here.
    pre_arm = pre_arm_check.evaluate_pre_arm_samples(
        node.rows["left"], node.rows["right"], node.rows["bridge"],
        force_tolerance_n=args.force_tolerance_n, telemetry_stale_s=stale_s,
        min_samples=args.min_samples, window_start_t=window_start_t,
        expected_channels=expected_channels, require_enabled_false=False,
    )
    print(f"[arm] pre-arm evaluation: status={pre_arm['status']} left_n={pre_arm.get('left_n_samples')} "
          f"right_n={pre_arm.get('right_n_samples')} bridge_n={pre_arm.get('bridge_n_samples')}", flush=True)
    if pre_arm["status"] != "PRE_ARM_PASS":
        for r in pre_arm["reasons"]:
            print(f"[arm][PRE_ARM_FAIL] {r}", flush=True)
        fail_closed("pre-arm verification failed", pre_arm)

    print("[arm] pre-arm verification PASSED -- arming now.", flush=True)

    # ---- Step 3: the actual fix -- set arm_teleop=true, async, non-blocking ----
    ok, reason = node.set_arm_teleop(True)
    if not ok:
        fail_closed(f"arm_teleop parameter rejected: {reason!r}")
    arm_t = time.time()
    print(f"[arm] arm_teleop=true ACCEPTED at t={arm_t:.3f}", flush=True)

    # ---- Step 4: post-arm freshness ----
    fresh_deadline = time.time() + args.post_arm_freshness_window_s
    post_arm_fresh = {"left": False, "right": False, "diag": False}
    while time.time() < fresh_deadline and not all(post_arm_fresh.values()):
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.time()
        for key in ("left", "right", "diag"):
            if key in node.latest and now - node.latest[key][0] <= stale_s:
                post_arm_fresh[key] = True
    if not all(post_arm_fresh.values()):
        fail_closed(f"post-arm telemetry freshness not established within "
                    f"{args.post_arm_freshness_window_s}s", post_arm_fresh)
    print(f"[arm] post-arm telemetry freshness confirmed: {post_arm_fresh}", flush=True)

    # ---- Step 5: verify the state machine actually progresses ----
    progression_deadline = time.time() + args.progression_timeout_s
    left_ready = False
    reached_active = False
    while time.time() < progression_deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.time()
        if "diag" not in node.latest:
            continue
        _, d = node.latest["diag"]
        if d.bridge_state == "FAULT":
            fail_closed("bridge entered FAULT during arm progression")
        if not left_ready and d.bridge_state != "READY":
            left_ready = True
            print(f"[arm] bridge LEFT READY at t={now - arm_t:.3f}s after arm (state={d.bridge_state})",
                 flush=True)
        if (bool(d.c1_enabled) != expected_channels["c1"] or bool(d.c2_enabled) != expected_channels["c2"]
                or bool(d.c3_enabled) != expected_channels["c3"]
                or bool(d.c4_enabled) != expected_channels["c4"]):
            fail_closed("unexpected channel state during progression", {
                "c1": d.c1_enabled, "c2": d.c2_enabled, "c3": d.c3_enabled, "c4": d.c4_enabled,
                "expected": expected_channels})
        if d.bridge_state == "ACTIVE" and d.coupling_ramp_alpha >= 0.999:
            reached_active = True
            print(f"[arm] ACTIVE reached at t={now - arm_t:.3f}s after arm, "
                  f"coupling_ramp_alpha={d.coupling_ramp_alpha:.4f} c3_enabled={d.c3_enabled}", flush=True)
            break

    if not left_ready:
        fail_closed(f"bridge_state remained READY for the full {args.progression_timeout_s}s after arming "
                    "-- arm_teleop was accepted but the state machine never progressed (this is exactly the "
                    "bug this script fixes -- if it fires here, the fix itself has a gap, investigate before "
                    "reporting teleoperation active)")
    if not reached_active:
        fail_closed(f"bridge did not reach ACTIVE within {args.progression_timeout_s}s of arming "
                    f"(last observed state={node.phase})")

    # ---- Step 6: positive force-budget margin, explicitly (2026-09-10) ----
    # Reaching ACTIVE does NOT by itself guarantee this: coupling_force_
    # budget.py's bisection accepts a step with margin < 0 once it is already
    # at the minimum bisection floor (min_alpha_step_fraction), specifically
    # to avoid deadlocking the ramp forever -- a documented edge case, not a
    # bug. margin already has force_reserve_n subtracted (margin = (max_force
    # - reserve) - |predicted force|), so margin > 0 is the correct pass
    # condition. Strictly greater than zero, not >=, per the explicit
    # requirement not to accept an exact-zero budget.
    margins = {
        "master_accepted_force_budget_margin_n": d.master_accepted_force_budget_margin_n,
        "slave_accepted_force_budget_margin_n": d.slave_accepted_force_budget_margin_n,
    }
    if not (margins["master_accepted_force_budget_margin_n"] > 0.0
            and margins["slave_accepted_force_budget_margin_n"] > 0.0):
        fail_closed("non-positive accepted force-budget margin at ACTIVE", margins)
    print(f"[arm] force-budget margins OK: master={margins['master_accepted_force_budget_margin_n']:.4f}N "
          f"slave={margins['slave_accepted_force_budget_margin_n']:.4f}N", flush=True)

    print("================ PRODUCTION TELEOPERATION ACTIVE ================", flush=True)
    print(f"bridge_state=ACTIVE  c1={expected_channels['c1']} c2={expected_channels['c2']} "
          f"c3={expected_channels['c3']} c4={expected_channels['c4']}", flush=True)
    print("Bridge remains armed -- this script does NOT disarm on success.", flush=True)
    print("===================================================================", flush=True)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
