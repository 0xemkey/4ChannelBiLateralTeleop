#!/usr/bin/env python3
"""Dedicated stillness/noise-floor calibration -- Phase 2 of
docs/RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md, approved 2026-09-08 as the
FIRST step of any real identification session, replacing that document's
provisional (not data-grounded) qdot/qddot thresholds.

ENTIRELY PASSIVE. Subscribes only to /touch_right/dob_diagnostics
(DobDiagnostics.msg), which is "OBSERVATION ONLY... nothing here feeds the
command path" per that message's own docstring -- this script commands no
force whatsoever. It is meant to run against the already-safe zero-force
DOB-observation startup (dob_enable=true, joint_space_control=false,
enabled_on_start=false -- the normal 00_preflight.sh/01_start_master.sh/
02_start_slave.sh state), not against hdapi_pose_hold_sweep.cpp (which
can't run at the same time as touch_slave.cpp anyway -- single-owner
OpenHaptics constraint).

WHY THIS EXISTS: checked existing hardware logs for a clean stationary
window before writing RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md and found
neither available candidate trustworthy -- one was stiction-locked
(degenerate, not real noise), one was contaminated by hand tremor. This
script captures a real, dedicated, nothing-else-going-on window instead.

USAGE (right/slave device, after 00_preflight.sh + 02_start_slave.sh, with
NOTHING touching the stylus -- see the printed instruction at startup):

    python3 calibrate_stillness_noise.py --duration-s 8 \\
        --out-json ~/dob_logs/results/stillness_calibration_right.json

Prints qdot/qddot noise floor per joint, stationary position variance,
message-gap/dropout behavior, and DERIVED thresholds (k * std, k
configurable) -- ready to pass into analyze_pose_hold_sweep.py's
--qdot-threshold-rad-s.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from touch_haptics.msg import DobDiagnostics


class StillnessCalibrator(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stillness_noise_calibrator")
        self.args = args
        self.samples: list[dict] = []
        self.gaps_s: list[float] = []
        self._last_stamp_s: float | None = None
        self._last_wall_s: float | None = None
        self.create_subscription(DobDiagnostics, args.topic, self._cb, 50)
        self.start_wall_s = time.time()
        self.done = False

    def _cb(self, msg: DobDiagnostics) -> None:
        now_wall = time.time()
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_wall_s is not None:
            self.gaps_s.append(now_wall - self._last_wall_s)
        self._last_wall_s = now_wall
        self._last_stamp_s = stamp_s

        if now_wall - self.start_wall_s < self.args.settle_s:
            return  # discard the first settle_s seconds unconditionally
        if now_wall - self.start_wall_s > self.args.settle_s + self.args.duration_s:
            self.done = True
            return

        self.samples.append(dict(
            t=stamp_s, q=list(msg.q_rad), qdot=list(msg.qdot_rad_s),
            tau_dis_hat=list(msg.tau_dis_hat_nm), tau_cmd=list(msg.tau_cmd_nm),
            configured=msg.configured,
        ))


def analyze(samples: list[dict], gaps_s: list[float], k: float) -> dict:
    if len(samples) < 10:
        return dict(ok=False, reason=f"only {len(samples)} samples collected -- too few, "
                                      f"check the topic is publishing and nothing aborted early.")

    n_unconfigured = sum(1 for s in samples if not s["configured"])
    t = np.array([s["t"] for s in samples])
    q = np.array([s["q"] for s in samples])          # (N, 3)
    qdot_reported = np.array([s["qdot"] for s in samples])   # node's own backward-diff
    tau_dis = np.array([s["tau_dis_hat"] for s in samples])
    tau_cmd = np.array([s["tau_cmd"] for s in samples])

    # Independent qddot via second difference of the NODE's own qdot_rad_s
    # (not re-derived from q here, to match what the identification pipeline
    # will actually see from the same message).
    dt = np.diff(t)
    dt = np.where(dt > 1e-6, dt, np.nan)
    qddot = np.diff(qdot_reported, axis=0) / dt[:, None]

    q_std = q.std(axis=0)
    q_range = q.max(axis=0) - q.min(axis=0)
    qdot_std = qdot_reported.std(axis=0)
    qdot_absmax = np.abs(qdot_reported).max(axis=0)
    qddot_valid = np.isfinite(qddot).all(axis=1)
    qddot_std = np.nanstd(qddot[qddot_valid], axis=0) if qddot_valid.any() else np.full(3, np.nan)
    tau_dis_std = tau_dis.std(axis=0)
    tau_cmd_std = tau_cmd.std(axis=0)

    gaps = np.array(gaps_s)
    nominal_dt = np.median(np.diff(t)) if len(t) > 1 else float("nan")
    dropouts = int(np.sum(gaps > 3 * nominal_dt)) if len(gaps) else 0

    return dict(
        ok=True,
        n_samples=len(samples),
        n_unconfigured=n_unconfigured,
        duration_s=float(t[-1] - t[0]),
        nominal_dt_s=float(nominal_dt),
        n_dropouts_gt_3x_nominal_dt=dropouts,
        per_joint=dict(
            waist=dict(q_std=float(q_std[0]), q_range_rad=float(q_range[0]),
                       qdot_std_rad_s=float(qdot_std[0]), qdot_absmax_rad_s=float(qdot_absmax[0]),
                       qddot_std_rad_s2=float(qddot_std[0]) if np.isfinite(qddot_std[0]) else None,
                       tau_dis_hat_std_nm=float(tau_dis_std[0]), tau_cmd_std_nm=float(tau_cmd_std[0])),
            shoulder=dict(q_std=float(q_std[1]), q_range_rad=float(q_range[1]),
                          qdot_std_rad_s=float(qdot_std[1]), qdot_absmax_rad_s=float(qdot_absmax[1]),
                          qddot_std_rad_s2=float(qddot_std[1]) if np.isfinite(qddot_std[1]) else None,
                          tau_dis_hat_std_nm=float(tau_dis_std[1]), tau_cmd_std_nm=float(tau_cmd_std[1])),
            elbow=dict(q_std=float(q_std[2]), q_range_rad=float(q_range[2]),
                      qdot_std_rad_s=float(qdot_std[2]), qdot_absmax_rad_s=float(qdot_absmax[2]),
                      qddot_std_rad_s2=float(qddot_std[2]) if np.isfinite(qddot_std[2]) else None,
                      tau_dis_hat_std_nm=float(tau_dis_std[2]), tau_cmd_std_nm=float(tau_cmd_std[2])),
        ),
        derived_thresholds_k=k,
        derived_qdot_threshold_rad_s=float(k * qdot_std.max()),
        derived_qddot_threshold_rad_s2=(
            float(k * np.nanmax(qddot_std)) if np.isfinite(qddot_std).any() else None),
        qddot_reliable=bool(np.isfinite(qddot_std).all() and n_unconfigured == 0),
    )


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/touch_right/dob_diagnostics")
    p.add_argument("--duration-s", type=float, default=8.0,
                   help="Measurement window AFTER settling, seconds. Task minimum is 5-10s.")
    p.add_argument("--settle-s", type=float, default=2.0,
                   help="Discarded warm-up window before measurement starts.")
    p.add_argument("--k-sigma", type=float, default=4.5,
                   help="Threshold = k * measured std. Task proposes 4-5.")
    p.add_argument("--out-json", default=None)
    args = p.parse_args()

    print(f"[stillness_calibration] topic={args.topic}")
    print(f"[stillness_calibration] settle {args.settle_s}s, then measure {args.duration_s}s")
    print("[stillness_calibration] DO NOT TOUCH THE STYLUS. Let it rest completely "
          "still (or hands-off if the pose self-holds) for the full window.")
    print("[stillness_calibration] This is READ-ONLY -- no force is commanded by this script.\n")

    rclpy.init()
    node = StillnessCalibrator(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    result = analyze(node.samples, node.gaps_s, args.k_sigma)
    node.destroy_node()
    rclpy.try_shutdown()

    if not result["ok"]:
        print(f"[stillness_calibration] FAILED: {result['reason']}")
        sys.exit(1)

    print(json.dumps(result, indent=2))
    if result["n_unconfigured"] > 0:
        print(f"\nWARNING: {result['n_unconfigured']} samples had configured=false "
              "(DOB bank not loaded) -- tau_dis_hat/tau_cmd are meaningless for those.")
    if result["n_dropouts_gt_3x_nominal_dt"] > 0:
        print(f"\nWARNING: {result['n_dropouts_gt_3x_nominal_dt']} message gaps exceeded "
              "3x the nominal interval -- possible telemetry dropout during the window.")
    print(f"\nDerived qdot threshold: {result['derived_qdot_threshold_rad_s']:.5f} rad/s "
          f"(k={args.k_sigma})")
    if result["derived_qddot_threshold_rad_s2"] is not None:
        print(f"Derived qddot threshold: {result['derived_qddot_threshold_rad_s2']:.4f} rad/s^2 "
              f"(qddot_reliable={result['qddot_reliable']})")
    else:
        print("qddot threshold: NOT COMPUTABLE from this window (insufficient valid samples).")

    if args.out_json:
        with open(args.out_json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nwritten: {args.out_json}")


if __name__ == "__main__":
    main()
