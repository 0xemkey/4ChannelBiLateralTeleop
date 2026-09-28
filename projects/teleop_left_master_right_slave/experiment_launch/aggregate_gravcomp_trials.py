#!/usr/bin/env python3
"""Aggregates all trial_*/metrics.json files under SESSION_DIR/analysis/ into
the per-condition mean/std/median/min/max table (Section 13/16 of the A2/B3
scientific-comparison protocol, 2026-09-09). Pure Python, no rclpy.

Usage: python3 aggregate_gravcomp_trials.py --session-dir SESSION_DIR
"""
from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path


def _get(d, path):
    for k in path.split("."):
        if d is None:
            return None
        d = d.get(k)
    return d


METRICS = [
    ("final_5s.mean_dy", "Final-5s mean dy (m)"),
    ("final_5s.mean_norm", "Final-5s mean ||dp|| (m)"),
    ("final_5s.std_norm", "Final-5s std ||dp|| (m)"),
    ("rms_tracking_error_m", "RMS tracking error (m)"),
    ("peak_to_peak_final5s_m", "Peak-to-peak final-5s (m)"),
    ("clamp_pct.left_position_error_clamped", "Master pos clamp %"),
    ("clamp_pct.right_position_error_clamped", "Slave pos clamp %"),
    ("clamp_pct.right_final_force_clamped", "Slave force clamp %"),
    ("slave_y_force.spring_y_mean", "Slave spring Y (N)"),
    ("slave_y_force.aux_y_mean", "Slave aux Y (N)"),
    ("slave_y_force.applied_y_mean", "Slave applied Y (N)"),
]


def summarize(values):
    values = [v for v in values if v is not None]
    if not values:
        return {"mean": None, "std": None, "median": None, "min": None, "max": None, "n": 0}
    return {
        "mean": statistics.fmean(values),
        "std": statistics.pstdev(values) if len(values) > 1 else 0.0,
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "n": len(values),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session-dir", required=True)
    args = ap.parse_args()

    analysis_dir = Path(args.session_dir) / "analysis"
    trials = sorted(analysis_dir.glob("trial_*/metrics.json"))
    by_condition = {"0.55": [], "0.70": []}
    for p in trials:
        with open(p) as f:
            m = json.load(f)
        cond = m.get("condition_n")
        if cond in by_condition:
            by_condition[cond].append(m)
        print(f"loaded {p} condition={cond} n_left_pose={m.get('n_samples_left_pose')}")

    print(f"\nTrials found: 0.55 N -> {len(by_condition['0.55'])}, 0.70 N -> {len(by_condition['0.70'])}\n")

    print(f"{'Metric':<32} | {'0.55N mean±std':>22} | {'0.70N mean±std':>22} | {'Diff (0.70-0.55)':>18}")
    print("-" * 100)
    table_rows = []
    for path, label in METRICS:
        s55 = summarize([_get(m, path) for m in by_condition["0.55"]])
        s70 = summarize([_get(m, path) for m in by_condition["0.70"]])
        diff = (s70["mean"] - s55["mean"]) if (s55["mean"] is not None and s70["mean"] is not None) else None
        m55 = f"{s55['mean']:.5f}±{s55['std']:.5f}" if s55["mean"] is not None else "n/a"
        m70 = f"{s70['mean']:.5f}±{s70['std']:.5f}" if s70["mean"] is not None else "n/a"
        d = f"{diff:+.5f}" if diff is not None else "n/a"
        print(f"{label:<32} | {m55:>22} | {m70:>22} | {d:>18}")
        table_rows.append({"metric": label, "path": path, "0.55": s55, "0.70": s70, "diff": diff})

    out = {
        "n_trials_055": len(by_condition["0.55"]),
        "n_trials_070": len(by_condition["0.70"]),
        "table": table_rows,
    }
    out_path = analysis_dir / "aggregate.json"
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2, default=str)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
