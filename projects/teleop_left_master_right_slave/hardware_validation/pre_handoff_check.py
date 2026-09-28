#!/usr/bin/env python3
"""One-shot pre-handoff validation (2026-09-10), called by run_hardware_test.sh
AFTER arm_production_bridge.py has already reached ACTIVE with a passing
planner-match/force-budget-margin check.

arm_production_bridge.py already proves: single instance, publisher
ownership, pre/post-arm telemetry freshness, ACTIVE reached,
planner_parameter_match_left/right (which cross-checks K/D/Fmax/max_error
against the controller's own telemetry), and positive accepted force-budget
margins. What THIS script adds, reading a fresh telemetry sample from both
sides:

  - explicit printout of the resolved K/D/Fmax/max_error the controller is
    ACTUALLY running with (not re-derived, read directly from
    CartesianControllerTelemetry's own stiffness_n_per_m/damping_n_s_per_m/
    max_position_error_m/max_force_n fields)
  - no force/position clamp already latched at rest
    (position_error_clamped/primary_force_clamped/final_force_clamped all
    false)
  - applied_force_n at rest is a plausible magnitude (below max_force_n,
    which primary/final_force_clamped already being false implies, but
    checked again explicitly as a second, independent signal) -- ground
    truth per the project's own convention, never reconstructed from
    spring+damping+aux
  - DOB stays observation-only for these position-channel tests (dob_
    diagnostics.configured may be true or false; this script does not gate
    on it, only records it, since DOB write is a compile-time-gated
    concern the position-channel path does not touch)

Exits 0 (prints a JSON summary to stdout) if every check passes, exits 1
otherwise. Never publishes anything -- subscribe-only, like
arm_production_bridge.py's own telemetry subscriptions.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics, DobDiagnostics


class OneShotNode(Node):
    def __init__(self, left_topic, right_topic, diag_topic, left_dob_topic, right_dob_topic):
        super().__init__("hardware_validation_pre_handoff_check")
        self.latest = {}
        self.create_subscription(CartesianControllerTelemetry, left_topic,
                                 lambda m: self.latest.__setitem__("left", m), 10)
        self.create_subscription(CartesianControllerTelemetry, right_topic,
                                 lambda m: self.latest.__setitem__("right", m), 10)
        self.create_subscription(BridgeDiagnostics, diag_topic,
                                 lambda m: self.latest.__setitem__("diag", m), 10)
        self.create_subscription(DobDiagnostics, left_dob_topic,
                                 lambda m: self.latest.__setitem__("left_dob", m), 10)
        self.create_subscription(DobDiagnostics, right_dob_topic,
                                 lambda m: self.latest.__setitem__("right_dob", m), 10)


def _telem_summary(m: CartesianControllerTelemetry) -> dict:
    return {
        "stiffness_n_per_m": m.stiffness_n_per_m,
        "damping_n_s_per_m": m.damping_n_s_per_m,
        "max_position_error_m": m.max_position_error_m,
        "max_force_n": m.max_force_n,
        "applied_force_n": list(m.applied_force_n),
        "position_error_clamped": bool(m.position_error_clamped),
        "primary_force_clamped": bool(m.primary_force_clamped),
        "final_force_clamped": bool(m.final_force_clamped),
        "enabled": bool(m.enabled),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--left-telemetry-topic", default="/touch_left/cartesian_controller_telemetry")
    ap.add_argument("--right-telemetry-topic", default="/touch_right/cartesian_controller_telemetry")
    ap.add_argument("--bridge-diagnostics-topic", default="/bridge_diagnostics")
    ap.add_argument("--left-dob-topic", default="/touch_left/dob_diagnostics")
    ap.add_argument("--right-dob-topic", default="/touch_right/dob_diagnostics")
    ap.add_argument("--window-s", type=float, default=2.0)
    ap.add_argument("--expected-stiffness", type=float, required=True)
    ap.add_argument("--expected-damping", type=float, required=True)
    ap.add_argument("--expected-max-force", type=float, required=True)
    ap.add_argument("--expected-max-position-error", type=float, required=True)
    ap.add_argument("--tol", type=float, default=1e-6)
    args = ap.parse_args()

    rclpy.init()
    node = OneShotNode(args.left_telemetry_topic, args.right_telemetry_topic,
                       args.bridge_diagnostics_topic, args.left_dob_topic, args.right_dob_topic)
    deadline = time.time() + args.window_s
    while time.time() < deadline and not all(k in node.latest for k in ("left", "right", "diag")):
        rclpy.spin_once(node, timeout_sec=0.05)

    problems = []
    result = {"ok": False}
    if "left" not in node.latest or "right" not in node.latest or "diag" not in node.latest:
        problems.append(f"did not receive left+right+diag telemetry within {args.window_s}s")
    else:
        left, right, diag = node.latest["left"], node.latest["right"], node.latest["diag"]
        result["left"] = _telem_summary(left)
        result["right"] = _telem_summary(right)
        result["bridge_state"] = diag.bridge_state
        result["channels"] = {"c1": bool(diag.c1_enabled), "c2": bool(diag.c2_enabled),
                              "c3": bool(diag.c3_enabled), "c4": bool(diag.c4_enabled)}
        result["force_budget_margins_n"] = {
            "master_accepted": diag.master_accepted_force_budget_margin_n,
            "slave_accepted": diag.slave_accepted_force_budget_margin_n,
        }
        if "left_dob" in node.latest:
            result["left_dob_configured"] = bool(node.latest["left_dob"].configured)
        if "right_dob" in node.latest:
            result["right_dob_configured"] = bool(node.latest["right_dob"].configured)

        def _mismatch(name, got, expected):
            if abs(got - expected) > args.tol:
                problems.append(f"master {name}={got} != expected {expected}")

        _mismatch("stiffness_n_per_m", left.stiffness_n_per_m, args.expected_stiffness)
        _mismatch("damping_n_s_per_m", left.damping_n_s_per_m, args.expected_damping)
        _mismatch("max_force_n", left.max_force_n, args.expected_max_force)
        _mismatch("max_position_error_m", left.max_position_error_m, args.expected_max_position_error)

        if left.position_error_clamped or left.primary_force_clamped or left.final_force_clamped:
            problems.append(f"master has a clamp flag latched AT REST: "
                            f"position_error_clamped={left.position_error_clamped} "
                            f"primary_force_clamped={left.primary_force_clamped} "
                            f"final_force_clamped={left.final_force_clamped}")
        if right.position_error_clamped or right.primary_force_clamped or right.final_force_clamped:
            problems.append(f"slave has a clamp flag latched AT REST: "
                            f"position_error_clamped={right.position_error_clamped} "
                            f"primary_force_clamped={right.primary_force_clamped} "
                            f"final_force_clamped={right.final_force_clamped}")

        left_applied_norm = sum(v * v for v in left.applied_force_n) ** 0.5
        if left_applied_norm > left.max_force_n + args.tol:
            problems.append(f"master applied_force_n norm={left_applied_norm:.4f} exceeds its own "
                            f"max_force_n={left.max_force_n} -- device-write ground truth says clamp failed")

        if diag.bridge_state != "ACTIVE":
            problems.append(f"bridge_state={diag.bridge_state}, expected ACTIVE")
        if diag.master_accepted_force_budget_margin_n <= 0.0:
            problems.append(f"master_accepted_force_budget_margin_n="
                            f"{diag.master_accepted_force_budget_margin_n} is not positive")
        if diag.slave_accepted_force_budget_margin_n <= 0.0:
            problems.append(f"slave_accepted_force_budget_margin_n="
                            f"{diag.slave_accepted_force_budget_margin_n} is not positive")

    result["ok"] = not problems
    result["problems"] = problems
    print(json.dumps(result, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result["ok"] else 1)


if __name__ == "__main__":
    main()
