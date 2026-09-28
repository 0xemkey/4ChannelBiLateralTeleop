#!/usr/bin/env python3
"""LIVE post-disarm safety verification (2026-09-10 safety-hardening
change, requested before H2 real-hardware execution).

Subscribes fresh (NOT a re-read of a capture script's already-written CSV
-- a genuinely new rclpy subscription, started after the capture script's
own process has fully exited) to both /touch_left/cartesian_controller_
telemetry and /touch_right/cartesian_controller_telemetry, collects at
least MIN_SAMPLES on EACH side, and confirms across ALL collected samples
(not just the most recent one -- the same "cannot false-pass from a
single sample" principle H2's own analyzer already applies) that:

  - enabled == False on every sample, both arms
  - ||applied_force_n|| stays under FORCE_TOLERANCE_N on every sample,
    both arms

FORCE_TOLERANCE_N is imported directly from part1_gravity_settle_capture.py
(UNEXPECTED_SPRING_DAMPING_NORM_N) rather than a new invented number --
the same tolerance this project already uses elsewhere for "is there a
genuine unexpected force present."

Exit code / result:
  0  CONFIRMED_DISABLED   -- both arms verified enabled=False, force~0
                             across every collected sample.
  1  CONFIRMED_NOT_DISABLED -- at least one sample shows enabled=True or
                             a force above tolerance. This is the more
                             serious finding -- report immediately.
  2  SAFETY_INCOMPLETE    -- could not collect enough fresh samples on one
                             or both sides within the time window (topic
                             silent, telemetry stale, etc.) -- absence of
                             evidence is NOT evidence of a safe state.

Per the calling script's own instruction: if this does not exit 0, H2 must
be reported as safety-incomplete (or worse) and the operator must NOT
proceed to H3.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from touch_haptics.msg import CartesianControllerTelemetry

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                      / "hardware_experiments" / "2026-09-09_y_axis_validation_PENDING_SUPERVISION"))
from part1_gravity_settle_capture import UNEXPECTED_SPRING_DAMPING_NORM_N  # noqa: E402

FORCE_TOLERANCE_N = UNEXPECTED_SPRING_DAMPING_NORM_N
MIN_SAMPLES = 5
DEFAULT_WINDOW_S = 3.0
DEFAULT_TARGET_SAMPLES = 20


def _norm3(v):
    return math.sqrt(sum(x * x for x in v))


class DisarmVerifier(Node):
    def __init__(self):
        super().__init__("hardware_validation_verify_disarmed")
        self.samples = {"left": [], "right": []}
        self.create_subscription(CartesianControllerTelemetry, "/touch_left/cartesian_controller_telemetry",
                                 lambda m: self._on("left", m), 50)
        self.create_subscription(CartesianControllerTelemetry, "/touch_right/cartesian_controller_telemetry",
                                 lambda m: self._on("right", m), 50)

    def _on(self, side, m):
        self.samples[side].append({
            "t": time.time(),
            "enabled": bool(m.enabled),
            "applied_force_norm_n": _norm3(list(m.applied_force_n)),
        })


def run(window_s: float, target_samples: int) -> dict:
    if not rclpy.ok():
        rclpy.init()
    node = DisarmVerifier()
    deadline = time.time() + window_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if len(node.samples["left"]) >= target_samples and len(node.samples["right"]) >= target_samples:
            break
    samples = node.samples
    node.destroy_node()

    result = {"force_tolerance_n": FORCE_TOLERANCE_N, "min_samples_required": MIN_SAMPLES,
             "window_s": window_s}
    incomplete_reasons = []
    for side in ("left", "right"):
        n = len(samples[side])
        result[f"{side}_n_samples"] = n
        if n < MIN_SAMPLES:
            incomplete_reasons.append(f"{side}: only {n} fresh sample(s) collected within {window_s}s "
                                      f"(need >= {MIN_SAMPLES})")

    if incomplete_reasons:
        result["status"] = "SAFETY_INCOMPLETE"
        result["reasons"] = incomplete_reasons
        return result

    violations = []
    for side in ("left", "right"):
        for s in samples[side]:
            if s["enabled"]:
                violations.append(f"{side}: enabled=True at t={s['t']:.3f}")
            if s["applied_force_norm_n"] >= FORCE_TOLERANCE_N:
                violations.append(f"{side}: applied_force_norm={s['applied_force_norm_n']:.4f}N "
                                  f">= tolerance {FORCE_TOLERANCE_N}N at t={s['t']:.3f}")

    if violations:
        result["status"] = "CONFIRMED_NOT_DISABLED"
        result["violations"] = violations
        return result

    result["status"] = "CONFIRMED_DISABLED"
    for side in ("left", "right"):
        peaks = [s["applied_force_norm_n"] for s in samples[side]]
        result[f"{side}_peak_applied_force_norm_n"] = max(peaks) if peaks else None
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window-s", type=float, default=DEFAULT_WINDOW_S)
    ap.add_argument("--target-samples", type=int, default=DEFAULT_TARGET_SAMPLES)
    args = ap.parse_args()

    result = run(args.window_s, args.target_samples)
    print(json.dumps(result, indent=2))
    exit_codes = {"CONFIRMED_DISABLED": 0, "CONFIRMED_NOT_DISABLED": 1, "SAFETY_INCOMPLETE": 2}
    sys.exit(exit_codes[result["status"]])


if __name__ == "__main__":
    main()
