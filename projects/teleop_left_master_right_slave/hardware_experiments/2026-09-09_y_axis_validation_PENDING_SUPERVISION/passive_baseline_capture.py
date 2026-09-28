#!/usr/bin/env python3
"""PASSIVE baseline capture only -- NO commands published, NO force applied,
NO bridge armed. Subscribes to existing live telemetry from the already-
running /teleop_master_left and /teleop_slave_right nodes and records it
for a fixed window. Read-only, matching live_joint_watch.py's own
"safe to run while both devices are held zero-force" convention.
"""
import csv
import json
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from touch_haptics.msg import CartesianControllerTelemetry, DobDiagnostics

WINDOW_S = 10.0


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


class PassiveCapture(Node):
    def __init__(self):
        super().__init__("passive_baseline_capture")
        self.rows = {"left": [], "right": []}
        self.dob = {"left": [], "right": []}
        self.create_subscription(CartesianControllerTelemetry, "/touch_left/cartesian_controller_telemetry",
                                  lambda m: self._on_telem("left", m), 50)
        self.create_subscription(CartesianControllerTelemetry, "/touch_right/cartesian_controller_telemetry",
                                  lambda m: self._on_telem("right", m), 50)
        self.create_subscription(DobDiagnostics, "/touch_left/dob_diagnostics",
                                  lambda m: self._on_dob("left", m), 20)
        self.create_subscription(DobDiagnostics, "/touch_right/dob_diagnostics",
                                  lambda m: self._on_dob("right", m), 20)

    def _on_telem(self, side, m):
        self.rows[side].append({
            "t": time.time(),
            "x": m.position_m[0], "y": m.position_m[1], "z": m.position_m[2],
            "vx": m.velocity_mps[0], "vy": m.velocity_mps[1], "vz": m.velocity_mps[2],
            "target_x": m.target_position_m[0], "target_y": m.target_position_m[1], "target_z": m.target_position_m[2],
            "err_x": m.position_error_raw_m[0], "err_y": m.position_error_raw_m[1], "err_z": m.position_error_raw_m[2],
            "spring_y": m.spring_force_n[1], "damping_y": m.damping_force_n[1], "aux_y": m.auxiliary_force_n[1],
            "applied_x": m.applied_force_n[0], "applied_y": m.applied_force_n[1], "applied_z": m.applied_force_n[2],
            "pos_clamped": bool(m.position_error_clamped),
            "primary_clamped": bool(m.primary_force_clamped),
            "final_clamped": bool(m.final_force_clamped),
        })

    def _on_dob(self, side, m):
        self.dob[side].append({"t": time.time(), "tau_y": m.tau_dis_hat_nm[1] if len(m.tau_dis_hat_nm) > 1 else None})


def summarize(rows):
    if not rows:
        return {"n": 0}
    vy = [r["vy"] for r in rows]
    y = [r["y"] for r in rows]
    applied = [_norm([r["applied_x"], r["applied_y"], r["applied_z"]]) for r in rows]
    return {
        "n": len(rows),
        "mean_vy": statistics.fmean(vy), "rms_vy": math.sqrt(statistics.fmean([v * v for v in vy])),
        "peak_abs_vy": max(abs(v) for v in vy),
        "y_drift_m": y[-1] - y[0], "y_min": min(y), "y_max": y[-1] if False else max(y),
        "mean_aux_y": statistics.fmean([r["aux_y"] for r in rows]),
        "mean_applied_residual_n": statistics.fmean(applied),
        "max_applied_residual_n": max(applied),
        "any_clamp": any(r["pos_clamped"] or r["primary_clamped"] or r["final_clamped"] for r in rows),
    }


def main():
    rclpy.init()
    node = PassiveCapture()
    t0 = time.time()
    while time.time() - t0 < WINDOW_S:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()

    out = {
        "window_s": WINDOW_S,
        "left": summarize(node.rows["left"]),
        "right": summarize(node.rows["right"]),
    }
    print(json.dumps(out, indent=2, default=str))
    with open(sys.argv[1] if len(sys.argv) > 1 else "passive_baseline.json", "w") as f:
        json.dump({"summary": out, "left_raw": node.rows["left"], "right_raw": node.rows["right"],
                   "left_dob": node.dob["left"], "right_dob": node.dob["right"]}, f, indent=2, default=str)


if __name__ == "__main__":
    main()
