#!/usr/bin/env python3
"""Fits tau_g(q) from pose-hold-sweep pilot/campaign data (the
approach-direction method), using the SAME basis and fitting philosophy
as the existing fit_gravity.py (BASIS_MINIMAL = [cos q2, sin q2, cos q3,
sin q3] per joint, robust least_squares) -- a new script because the data
shape differs (one tau_g estimate per POSE here, not per axis-pair), not
because the physics/regressor choice differs.

Unlike the old 6-14 pose fits, this dataset has ~16 poses spanning a much
wider q2/q3 range (see docs/RIGHT_GRAVITY_IDENTIFICATION_TOOLING.md) and
is deliberately held out for validation rather than fit-and-trust.

USAGE:
    python3 fit_gravity_pose_hold.py combined_all_poses.csv --out gravity_right_full.json
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from analyze_pose_hold_sweep import load_pose_hold_sweep, analyze  # noqa: E402

JOINT_NAMES = ("waist", "shoulder", "elbow")


def basis(q2: float, q3: float) -> np.ndarray:
    return np.array([np.cos(q2), np.sin(q2), np.cos(q3), np.sin(q3)])


def fit_joint(q2q3: np.ndarray, tau: np.ndarray) -> tuple[np.ndarray, float]:
    """Robust (soft_l1) least squares, matching fit_gravity.py's approach."""
    X = np.array([basis(q2, q3) for q2, q3 in q2q3])

    def resid(theta):
        return X @ theta - tau

    theta0 = np.linalg.lstsq(X, tau, rcond=None)[0]
    res = least_squares(resid, theta0, loss="soft_l1", f_scale=0.005)
    design_cond = np.linalg.cond(X)
    return res.x, design_cond


def predict(theta_per_joint: dict, q2: float, q3: float) -> np.ndarray:
    b = basis(q2, q3)
    return np.array([theta_per_joint[j] @ b for j in JOINT_NAMES])


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("csv_path")
    p.add_argument("--out", default=None, help="Fitted model JSON output path")
    p.add_argument("--exclude-pose", type=int, action="append", default=[],
                   help="pose_id(s) to exclude from BOTH train and val (e.g. known-marginal poses)")
    p.add_argument("--holdout-poses", type=int, nargs="+", default=None,
                   help="pose_id(s) to hold out for validation; default picks ~25%% spanning edge+interior")
    args = p.parse_args()

    rows = load_pose_hold_sweep(args.csv_path)
    result = analyze(rows, qdot_threshold=0.02)
    pairs = [pr for pr in result["pairs"] if pr["pose_id"] not in args.exclude_pose]
    print(f"{len(pairs)} usable pose pairs after excluding {args.exclude_pose}")

    # q used per pose: midpoint of the + and - approach (same convention
    # as pose_delta itself), consistent with the fact that tau_g is only
    # meaningful at "the" configuration, not either single approach's.
    q_mid = {pr["pose_id"]: [(a + b) / 2 for a, b in zip(pr["q_plus"], pr["q_minus"])]
              for pr in pairs}
    tau_g = {pr["pose_id"]: pr["tau_g_nm"] for pr in pairs}

    all_ids = sorted(q_mid.keys())
    if args.holdout_poses is not None:
        holdout = args.holdout_poses
    else:
        # Default: every 4th pose_id (by sorted shoulder angle) for a
        # spread across the range rather than a purely random split --
        # small-N validation should test interpolation AND extrapolation
        # at the edges, not just average-case interior poses.
        by_shoulder = sorted(all_ids, key=lambda pid: q_mid[pid][1])
        holdout = by_shoulder[::4]
        if len(holdout) < 2:
            holdout = by_shoulder[-2:]
    train_ids = [i for i in all_ids if i not in holdout]
    print(f"train poses ({len(train_ids)}): {train_ids}")
    print(f"holdout poses ({len(holdout)}): {holdout}")

    train_q = np.array([[q_mid[i][1], q_mid[i][2]] for i in train_ids])
    model = {}
    design_conds = {}
    for j, jname in enumerate(JOINT_NAMES):
        tau_j = np.array([tau_g[i][j] for i in train_ids])
        theta, cond = fit_joint(train_q, tau_j)
        model[jname] = theta
        design_conds[jname] = cond

    print("\n=== fitted coefficients (cos q2, sin q2, cos q3, sin q3) ===")
    for jname in JOINT_NAMES:
        print(f"  {jname:9s} {model[jname]} (design cond={design_conds[jname]:.1f})")
        if design_conds[jname] > 1000:
            print(f"    WARNING: design cond > 1000 -- coefficients not individually trustworthy")

    # held-out validation
    residuals = {j: [] for j in JOINT_NAMES}
    print("\n=== held-out predictions ===")
    for i in holdout:
        q2, q3 = q_mid[i][1], q_mid[i][2]
        pred = predict(model, q2, q3)
        actual = np.array(tau_g[i])
        res = pred - actual
        for j, jname in enumerate(JOINT_NAMES):
            residuals[jname].append(res[j])
        print(f"  pose {i:2d}  pred=[{pred[0]:+.5f} {pred[1]:+.5f} {pred[2]:+.5f}]  "
              f"actual=[{actual[0]:+.5f} {actual[1]:+.5f} {actual[2]:+.5f}]  "
              f"resid=[{res[0]:+.5f} {res[1]:+.5f} {res[2]:+.5f}]")

    print("\n=== held-out validation metrics (N*m) ===")
    metrics = {}
    for jname in JOINT_NAMES:
        r = np.array(residuals[jname])
        m = dict(
            rmse=float(np.sqrt(np.mean(r ** 2))), mae=float(np.mean(np.abs(r))),
            mean_signed=float(np.mean(r)), p95_abs=float(np.percentile(np.abs(r), 95)),
            max_abs=float(np.max(np.abs(r))),
        )
        metrics[jname] = m
        print(f"  {jname:9s} RMSE={m['rmse']:.5f} MAE={m['mae']:.5f} "
              f"mean_signed={m['mean_signed']:+.5f} p95={m['p95_abs']:.5f} max={m['max_abs']:.5f}")

    # repeated-hold consistency, direct from repeated tau_g estimates at
    # the same nominal pose (poses 3/15 and 6/16 were designed as repeats)
    print("\n=== repeated-pose (design-intent) consistency ===")
    repeat_groups = [(3, 15, "high shoulder/elbow"), (6, 16, "low shoulder")]
    for a, b, label in repeat_groups:
        if a in tau_g and b in tau_g:
            diff = np.array(tau_g[a]) - np.array(tau_g[b])
            print(f"  pose {a} vs {b} ({label}): tau_g diff = "
                  f"[{diff[0]:+.5f} {diff[1]:+.5f} {diff[2]:+.5f}] N*m, "
                  f"|diff|={np.linalg.norm(diff):.5f}")

    out = dict(
        basis=["cos q2", "sin q2", "cos q3", "sin q3"],
        units="N*m (from tau_hold = J^T(q) * bias, bias in newtons, exactly known)",
        sign_convention="tau_g such that tau_cmd += tau_g_hat(q) cancels gravity at rest",
        source_csv=args.csv_path, train_poses=train_ids, holdout_poses=holdout,
        excluded_poses=args.exclude_pose,
        joints={jname: dict(coef=model[jname].tolist(), design_cond=design_conds[jname])
                for jname in JOINT_NAMES},
        holdout_validation_metrics=metrics,
    )
    if args.out:
        with open(args.out, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nwritten: {args.out}")


if __name__ == "__main__":
    main()
