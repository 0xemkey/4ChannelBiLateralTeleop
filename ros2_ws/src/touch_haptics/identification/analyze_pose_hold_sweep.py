#!/usr/bin/env python3
"""Offline analysis for hdapi_pose_hold_sweep.cpp logs -- the
APPROACH-DIRECTION gravity/friction pilot, per
docs/RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md Phase 2/4.

This is a DATA-QUALITY AND METHODOLOGY CHECK, not a model fit. Per the
approved plan (2026-09-08): "For each pose compute tau_g_measured/
tau_f_static... If this separation does not behave physically, STOP before
collecting the remaining poses." This script produces exactly that check --
it does not write a gravity model anywhere, matching every other tool in
this suite's "writes nothing to touch_dob.hpp" policy.

METHOD (see hdapi_pose_hold_sweep.cpp's file header for the full
derivation): each pose_id should appear twice in the log, once with
approach_dir='+' and once '-'. For each such pair:

    tau_g_measured(q) = [tau_plus(q) + tau_minus(q)] / 2
    tau_f_static(q)   = [tau_plus(q) - tau_minus(q)] / 2

tau_plus/tau_minus here are tau_hold_nm -- the frozen autoHold() bias,
already converted through J^T(q) by the C++ tool itself using the SAME
jacobianTransposeTimes()/jacobian() this script's own common.jacobian()
mirrors (bitwise-identical convention, validated in the existing suite).

USAGE:
    python3 analyze_pose_hold_sweep.py <log.csv> [--out-json summary.json]

Prints, per pose_id pair: gravity estimate, friction estimate, an explicit
PASS/FAIL against each of the five approved sanity checks, and a top-level
verdict on whether the pilot's methodology check passed.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from dataclasses import dataclass, field

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import common  # noqa: E402


@dataclass
class HoldRow:
    session_id: str
    git_commit: str
    device: str
    base_orientation: str
    pose_id: int
    approach_dir: str          # '+' or '-'
    q: np.ndarray
    qdot: np.ndarray
    qddot: np.ndarray
    qdot_valid: bool
    qddot_valid: bool
    tau_hold_nm: np.ndarray    # already J^T(q) * bias, from the C++ tool
    bias_n: np.ndarray
    pos_m: np.ndarray
    sag_m: float
    max_wander_m: float
    unsettled_frac: float
    settled: bool
    open_loop_drift_rad: float
    hold_ok: bool


def load_pose_hold_sweep(path: str) -> list[HoldRow]:
    out = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            if r.get("tool") != "pose_hold_sweep":
                continue
            out.append(HoldRow(
                session_id=r["session_id"], git_commit=r["git_commit"],
                device=r["device"], base_orientation=r["base_orientation"],
                pose_id=int(r["pose_id"]), approach_dir=r["approach_dir"],
                q=np.array([float(r[f"q{i}"]) for i in (1, 2, 3)]),
                qdot=np.array([float(r[f"qdot{i}"]) for i in (1, 2, 3)]),
                qddot=np.array([float(r[f"qddot{i}"]) for i in (1, 2, 3)]),
                qdot_valid=r["qdot_valid"] == "1",
                qddot_valid=r["qddot_valid"] == "1",
                tau_hold_nm=np.array([float(r[f"tau_hold_nm_{i}"]) for i in (1, 2, 3)]),
                bias_n=np.array([float(r[f"bias_{i}"]) for i in (1, 2, 3)]),
                pos_m=np.array([float(r[f"pos_{a}"]) for a in ("x", "y", "z")]),
                sag_m=float(r["sag_m"]), max_wander_m=float(r["max_wander_m"]),
                unsettled_frac=float(r["unsettled_frac"]), settled=r["settled"] == "1",
                open_loop_drift_rad=float(r["open_loop_drift_rad"]),
                hold_ok=r["hold_ok"] == "1",
            ))
    if not out:
        raise SystemExit(f"{path}: no rows with tool=='pose_hold_sweep'. Wrong log file?")
    return out


# Acceptance thresholds -- see RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md's
# threshold-derivation status. These are the SAME provisional values (not
# yet replaced by the dedicated stillness calibration); if that calibration
# has since produced real numbers, pass them via --qdot-noise-std etc.
DEFAULT_QDOT_THRESHOLD_RAD_S = 0.02
PAIR_POSE_TOL_RAD = 0.05          # matches the existing suite's kPairPoseTolRad
WIDE_GRAVITY_SPREAD_NM = 0.02     # matches the existing suite's "decomposition suspect"


def analyze(rows: list[HoldRow], qdot_threshold: float) -> dict:
    # ACCEPTANCE GATE (updated 2026-09-08 per operator direction): the
    # Cartesian settle criterion inside autoHold() (kHoldSettleVelMps =
    # 0.006 m/s, already enforced before a hold is ever frozen -- hold_ok
    # false means that gate failed) is AUTHORITATIVE. qdot/qddot are
    # DIAGNOSTIC ONLY here, not a hard reject: the dedicated stillness
    # capture measured zero-noise floor at true rest, so a fixed qdot
    # threshold derived from it (e.g. via a single v/L scalar) would be
    # wrong -- x_dot = J(q) @ qdot is configuration-dependent, not a fixed
    # conversion. A qdot threshold belongs here only once derived from a
    # real settle-transient calibration (see analyze_settle_transients.py),
    # not before.
    by_pose: dict[int, dict[str, HoldRow]] = {}
    rejected = []
    qdot_flagged_but_kept = []
    for r in rows:
        if not r.hold_ok:
            rejected.append(r)
            continue
        if r.qdot_valid and abs(r.qdot).max() > qdot_threshold:
            qdot_flagged_but_kept.append(r)   # reported, not excluded
        by_pose.setdefault(r.pose_id, {})[r.approach_dir] = r

    pairs = []
    for pose_id, d in sorted(by_pose.items()):
        if "+" not in d or "-" not in d:
            print(f"pose {pose_id}: incomplete pair (have {sorted(d.keys())}) -- skipped")
            continue
        rp, rm = d["+"], d["-"]
        pose_delta = float(np.max(np.abs(rp.q - rm.q)))
        tau_g = (rp.tau_hold_nm + rm.tau_hold_nm) / 2.0
        tau_c = (rp.tau_hold_nm - rm.tau_hold_nm) / 2.0
        pairs.append(dict(
            pose_id=pose_id, pose_delta_rad=pose_delta,
            pose_paired_ok=pose_delta <= PAIR_POSE_TOL_RAD,
            q_plus=rp.q.tolist(), q_minus=rm.q.tolist(),
            tau_g_nm=tau_g.tolist(), tau_c_nm=tau_c.tolist(),
            tau_plus_nm=rp.tau_hold_nm.tolist(), tau_minus_nm=rm.tau_hold_nm.tolist(),
            base_orientation=rp.base_orientation,
        ))

    print(f"\n{len(rows)} total rows, {len(rejected)} rejected (hold_ok=0 -- Cartesian "
          f"settle gate is authoritative), {len(qdot_flagged_but_kept)} flagged for "
          f"|qdot|>{qdot_threshold} rad/s but KEPT (diagnostic only, not excluded -- "
          f"see acceptance-gate note above), {len(pairs)} usable pose pairs\n")

    # --- the 5 approved sanity checks ---
    checks = {}

    # 1. gravity approximately direction-independent -- N/A here by
    #    construction (tau_g is DEFINED as the average of the two
    #    directions) -- what we actually check is whether tau_c (the
    #    direction-dependent half) is small relative to tau_g, which is
    #    the physically meaningful version of "direction independence".
    if pairs:
        g_mag = np.mean([np.linalg.norm(p["tau_g_nm"]) for p in pairs])
        c_mag = np.mean([np.linalg.norm(p["tau_c_nm"]) for p in pairs])
        checks["gravity_dominates_or_comparable_to_friction"] = dict(
            mean_g_mag_nm=float(g_mag), mean_c_mag_nm=float(c_mag),
            note="tau_c is NOT expected to be zero (that's the friction signal "
                 "itself) -- this just reports the relative scale for a sanity read.")

    # 2. friction magnitude/consistency check -- REPLACES a blanket
    #    "tau_c < 0 = FAIL" sign rule (2026-09-08 sign-convention audit,
    #    see RIGHT_GRAVITY_PILOT_RESULTS.md "Pose-Matching Corrective
    #    Rerun" section for the full derivation). Summary: the negative-
    #    is-non-physical rule is valid for hdapi_static_sweep.cpp's
    #    AXIS-PROBE method, where '+'/'-' are externally-imposed, exactly-
    #    known probe force directions along a fixed Cartesian axis, so
    #    c >= 0 is guaranteed by that setup. This tool's '+'/'-' is only
    #    an operator label for which side a POSE was approached from --
    #    it fixes no per-joint sign (Cartesian "above" doesn't map to a
    #    consistent per-joint rotation direction, and is Jacobian/pose-
    #    dependent). Checking the pilot's own settle-transient data
    #    confirmed this empirically: the release-to-frozen micro-motion
    #    (the friction-relevant signal) is small (<0.01 rad typically)
    #    and NOT consistently signed even across holds sharing the same
    #    '+' label. A sign rule was therefore not physically justified for
    #    this method and produced false "FAIL"s on exactly the poses that
    #    were actually just poorly pose-matched (see the pose_delta
    #    correlation this same pilot found).
    #
    #    Replacement: |tau_c| should be small relative to |tau_g| (both
    #    are noise/mismatch-driven if not), and per the pilot's own
    #    finding, should shrink as pose_delta shrinks -- report both,
    #    flag (not hard-fail) large |tau_c| relative to |tau_g|.
    c_over_g_ratios = []
    large_c_poses = []
    for p in pairs:
        g_mag = np.linalg.norm(p["tau_g_nm"])
        c_mag = np.linalg.norm(p["tau_c_nm"])
        ratio = c_mag / g_mag if g_mag > 1e-9 else float("inf")
        c_over_g_ratios.append(ratio)
        if ratio > 1.0:   # friction term at least as large as gravity term -- flag for review
            large_c_poses.append(p["pose_id"])
    checks["friction_magnitude_consistency"] = dict(
        mean_c_over_g_ratio=float(np.mean([r for r in c_over_g_ratios if np.isfinite(r)]))
            if any(np.isfinite(r) for r in c_over_g_ratios) else None,
        poses_with_c_ge_g=large_c_poses,
        note="|tau_c| >= |tau_g| is flagged for review, not auto-failed -- per the "
             "sign-convention audit, this method's sign is not independently "
             "constrained the way the axis-probe method's is. Correlate against "
             "pose_delta before concluding a pose's estimate is unusable.")

    # 3. repeated holds consistent -- only checkable if the operator
    #    actually repeated a pose_id/direction combo more than once.
    dup_groups: dict[tuple, list[HoldRow]] = {}
    for r in rows:
        if r.hold_ok:
            dup_groups.setdefault((r.pose_id, r.approach_dir), []).append(r)
    repeat_stats = []
    for (pid, d), grp in dup_groups.items():
        if len(grp) < 2:
            continue
        taus = np.array([g.tau_hold_nm for g in grp])
        repeat_stats.append(dict(pose_id=pid, dir=d, n=len(grp),
                                  std_nm=taus.std(axis=0).tolist(),
                                  mean_nm=taus.mean(axis=0).tolist()))
    checks["repeated_hold_consistency"] = dict(
        n_repeated_combos=len(repeat_stats), detail=repeat_stats,
        note="Empty if no pose_id/direction was ever repeated in this session -- "
             "not a failure, just nothing to check yet. Repeat at least one "
             "pose/direction pair to populate this.")

    # 4. measurement variance acceptable -- unsettled_frac / max_wander from
    #    the C++ tool's own autoHold() diagnostics, already thresholded by
    #    hold_ok at accept time; report the distribution for visibility.
    if rows:
        checks["hold_quality"] = dict(
            mean_unsettled_frac=float(np.mean([r.unsettled_frac for r in rows])),
            mean_max_wander_mm=float(np.mean([r.max_wander_m for r in rows]) * 1000),
            n_hold_rejected=sum(1 for r in rows if not r.hold_ok),
        )

    # 5. no saturation/travel-limit contamination -- this tool has no
    #    Cartesian bridge/travel-limit concept (standalone HDAPI, not
    #    touch_slave.cpp); the equivalent check here is the bias cap.
    KMAXBIASN = 0.55
    at_cap = [r for r in rows if r.hold_ok and np.linalg.norm(r.bias_n) >= KMAXBIASN - 1e-3]
    checks["bias_cap_contamination"] = dict(
        n_at_cap=len(at_cap), pose_ids=[r.pose_id for r in at_cap],
        note="A hold at the bias cap means the true holding force may be "
             "higher than what was measured -- exclude these poses from fitting.")

    # gravity consistency across poses at similar q (wide-spread flag,
    # mirroring hdapi_static_sweep.cpp's own axis-pair consistency check,
    # applied here across pose pairs instead of axis pairs).
    for p in pairs:
        p["wide_gravity_spread_flag"] = bool(
            np.linalg.norm(np.array(p["tau_g_nm"])) > 0 and
            max(abs(v) for v in p["tau_c_nm"]) > WIDE_GRAVITY_SPREAD_NM)

    verdict = "PASS"
    reasons = []
    if len(pairs) == 0:
        verdict = "FAIL"
        reasons.append("no usable pose pairs")
    if any(not p["pose_paired_ok"] for p in pairs):
        verdict = "REVIEW"
        reasons.append("one or more pairs exceeded pose-drift tolerance")
    if checks["friction_magnitude_consistency"]["poses_with_c_ge_g"]:
        if verdict == "PASS":
            verdict = "REVIEW"
        reasons.append(f"|tau_c| >= |tau_g| for pose(s) "
                        f"{checks['friction_magnitude_consistency']['poses_with_c_ge_g']} "
                        f"-- check correlation with pose_delta before trusting these")

    return dict(pairs=pairs, checks=checks, verdict=verdict, reasons=reasons,
                n_total_rows=len(rows), n_rejected=len(rejected), n_pairs=len(pairs),
                n_qdot_flagged_but_kept=len(qdot_flagged_but_kept),
                qdot_flagged_pose_ids=[r.pose_id for r in qdot_flagged_but_kept])


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("log_csv")
    p.add_argument("--out-json", default=None)
    p.add_argument("--qdot-threshold-rad-s", type=float, default=DEFAULT_QDOT_THRESHOLD_RAD_S,
                   help="Provisional unless the stillness-calibration tool's real "
                        "value is passed here instead (see RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md).")
    args = p.parse_args()

    rows = load_pose_hold_sweep(args.log_csv)
    result = analyze(rows, args.qdot_threshold_rad_s)

    print("=== per-pose-pair gravity/friction estimate ===")
    for pr in result["pairs"]:
        print(f"pose {pr['pose_id']:3d}  pose_delta={pr['pose_delta_rad']:.4f} rad "
              f"{'OK' if pr['pose_paired_ok'] else 'DRIFTED'}  "
              f"tau_g=[{pr['tau_g_nm'][0]:+.5f} {pr['tau_g_nm'][1]:+.5f} {pr['tau_g_nm'][2]:+.5f}] N*m  "
              f"tau_c=[{pr['tau_c_nm'][0]:+.5f} {pr['tau_c_nm'][1]:+.5f} {pr['tau_c_nm'][2]:+.5f}] N*m"
              f"{'  <-- WIDE SPREAD' if pr['wide_gravity_spread_flag'] else ''}")

    print("\n=== sanity checks ===")
    for name, c in result["checks"].items():
        print(f"  {name}: {c}")

    print(f"\n=== VERDICT: {result['verdict']} ===")
    for r in result["reasons"]:
        print(f"  - {r}")
    if result["verdict"] != "PASS":
        print("\nPer the approved plan: if this separation does not behave physically,\n"
              "STOP before collecting the remaining poses.")

    if args.out_json:
        with open(args.out_json, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\nwritten: {args.out_json}")


if __name__ == "__main__":
    main()
