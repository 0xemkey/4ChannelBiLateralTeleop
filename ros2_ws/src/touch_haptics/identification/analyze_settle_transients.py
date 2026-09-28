#!/usr/bin/env python3
"""Empirically derives a per-joint qdot reference from real settle
transients, per operator direction 2026-09-08: a single Cartesian-speed-to-
joint-qdot conversion (v/L) was rejected because x_dot = J(q) @ qdot is
configuration-dependent, not a fixed scalar. This instead reads
hdapi_pose_hold_sweep.cpp's --settle-log trace (recorded during every
hold's natural post-release settling, no extra hardware session needed --
it comes for free from the pilot itself) and finds, per hold attempt, the
qdot values AT the sample where Cartesian speed first sustains below the
existing, already-safety-reviewed kHoldSettleVelMps=0.006 m/s gate.

This produces a REPORTED reference distribution, not a hard threshold.
Per operator direction: "qdot threshold = diagnostic only, not hard
reject" until this calibration exists and has been reviewed -- this
script is that calibration, but promoting its output to a hard gate in
analyze_pose_hold_sweep.py is a separate, later decision, not automatic.

USAGE:
    python3 analyze_settle_transients.py <settle_log.csv> [--out-json out.json]
"""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict

import numpy as np

KHOLDSETTLEVELMPS = 0.006   # must match hdapi_pose_hold_sweep.cpp's kHoldSettleVelMps
SUSTAIN_SAMPLES = 5          # require the gate to hold for this many consecutive 10ms
                              # samples (50ms) before calling it a real crossing, not a
                              # transient dip through zero mid-oscillation


def load(path: str) -> dict[int, list[dict]]:
    by_attempt: dict[int, list[dict]] = defaultdict(list)
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("tool") != "settle_trace":
                continue
            by_attempt[int(r["hold_attempt_id"])].append(dict(
                t=float(r["t_since_release_s"]), speed=float(r["cart_speed_mps"]),
                qdot=np.array([float(r[f"qdot{i}"]) for i in (1, 2, 3)]),
                q=np.array([float(r[f"q{i}"]) for i in (1, 2, 3)]),
            ))
    for k in by_attempt:
        by_attempt[k].sort(key=lambda x: x["t"])
    return by_attempt


def find_crossing(samples: list[dict]) -> dict | None:
    speeds = [s["speed"] for s in samples]
    for i in range(len(speeds) - SUSTAIN_SAMPLES):
        if all(speeds[i + j] <= KHOLDSETTLEVELMPS for j in range(SUSTAIN_SAMPLES)):
            return dict(t_crossing_s=samples[i]["t"], qdot_at_crossing=samples[i]["qdot"],
                        q_at_crossing=samples[i]["q"], speed_at_crossing=samples[i]["speed"])
    return None   # never sustained below the gate within this hold's 2.0s window


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("settle_log_csv")
    p.add_argument("--out-json", default=None)
    args = p.parse_args()

    by_attempt = load(args.settle_log_csv)
    print(f"{len(by_attempt)} hold attempts in settle-transient trace\n")

    crossings = []
    never_settled = 0
    for hold_id, samples in sorted(by_attempt.items()):
        c = find_crossing(samples)
        if c is None:
            never_settled += 1
            print(f"hold {hold_id}: never sustained below {KHOLDSETTLEVELMPS} m/s "
                  f"within the 2.0s window -- excluded")
            continue
        c["hold_attempt_id"] = hold_id
        crossings.append(c)
        print(f"hold {hold_id}: crossed at t={c['t_crossing_s']:.2f}s, "
              f"qdot=[{c['qdot_at_crossing'][0]:+.4f} {c['qdot_at_crossing'][1]:+.4f} "
              f"{c['qdot_at_crossing'][2]:+.4f}] rad/s")

    if not crossings:
        print("\nNo usable crossings found -- cannot derive a qdot reference yet. "
              "Need more holds (each hold in the pilot contributes one data point here "
              "for free), or the settle window (200 x 10ms = 2.0s) may be too short for "
              "this rig's typical settle time.")
        return

    qdot_at_crossing = np.array([c["qdot_at_crossing"] for c in crossings])  # (N, 3)
    qdot_abs = np.abs(qdot_at_crossing)

    result = dict(
        n_holds_total=len(by_attempt), n_crossings_found=len(crossings),
        n_never_settled=never_settled,
        per_joint_qdot_at_crossing_rad_s=dict(
            waist=dict(mean=float(qdot_abs[:, 0].mean()), std=float(qdot_abs[:, 0].std()),
                       max=float(qdot_abs[:, 0].max()), p95=float(np.percentile(qdot_abs[:, 0], 95))),
            shoulder=dict(mean=float(qdot_abs[:, 1].mean()), std=float(qdot_abs[:, 1].std()),
                          max=float(qdot_abs[:, 1].max()), p95=float(np.percentile(qdot_abs[:, 1], 95))),
            elbow=dict(mean=float(qdot_abs[:, 2].mean()), std=float(qdot_abs[:, 2].std()),
                      max=float(qdot_abs[:, 2].max()), p95=float(np.percentile(qdot_abs[:, 2], 95))),
        ),
        suggested_reference_qdot_rad_s=float(np.percentile(qdot_abs.max(axis=1), 95)),
        note="This is a REPORTED reference (p95 of the max-over-joints qdot at the "
             "Cartesian-speed crossing, across real holds), not an automatically-applied "
             "threshold. Per operator direction, promoting this to a hard gate in "
             "analyze_pose_hold_sweep.py is a separate decision.",
    )
    print("\n=== per-joint |qdot| at Cartesian-speed crossing ===")
    for j, stats in result["per_joint_qdot_at_crossing_rad_s"].items():
        print(f"  {j:9s} mean={stats['mean']:.4f} std={stats['std']:.4f} "
              f"max={stats['max']:.4f} p95={stats['p95']:.4f} rad/s")
    print(f"\nSuggested reference (p95, max-over-joints): "
          f"{result['suggested_reference_qdot_rad_s']:.4f} rad/s")
    print(f"({never_settled}/{len(by_attempt)} holds never settled within 2.0s -- "
          f"investigate if this fraction is large)")

    if args.out_json:
        with open(args.out_json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nwritten: {args.out_json}")


if __name__ == "__main__":
    main()
