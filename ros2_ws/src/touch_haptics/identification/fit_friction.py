#!/usr/bin/env python3
"""Coulomb friction floor per joint from hdapi_static_sweep logs.

WHAT THIS CAN AND CANNOT GIVE YOU
---------------------------------
COULOMB: yes. From the same +/- breakaway pairing used for gravity,
    c = (tau_plus - tau_minus) / 2
is the static (breakaway) friction magnitude in real N*m.

VISCOUS: no, and this script does not pretend otherwise. The static sweep is
quasi-static by construction -- the ramp is slow and the measurement is taken at
the instant of first motion, where qdot ~ 0. A viscous term b*qdot is
identically invisible in that data. Viscous friction is estimated instead by
fit_inertia.py from the dynamic-excitation captures, where qdot is large and
varying, and this script's Coulomb numbers serve as the independent cross-check
on the c term that regression also fits.

WHY THE NUMBERS MATTER
----------------------
tau_dis_hat sits at a nonzero floor at rest precisely because F_fric is zero in
the nominal model while the real device has stiction. threshold_nm has to clear
that floor, or the gate fires on friction and the observer cancels torque that
was never external. This measures the floor so the threshold can be set above
it by measurement rather than by guessing.

Usage:
    python3 fit_friction.py sweep.csv [--out friction_fit.json]
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np

import common
from fit_gravity import AXIS_PAIRS, PAIR_POSE_TOL_RAD


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sweep_csv")
    ap.add_argument("--out", default="friction_fit.json")
    args = ap.parse_args()

    rows = common.load_static_sweep(args.sweep_csv)
    by_pose: dict[int, dict[str, common.SweepRow]] = {}
    for r in rows:
        by_pose.setdefault(r.pose_id, {})[r.direction] = r

    print("=== Coulomb friction from breakaway pairs (REAL N*m) ===")
    print(common.CAVEAT_CARTESIAN_COUPLING)
    print()

    records = []
    print(f"{'pose':>4} {'pair':>5}  {'c = (tau+ - tau-)/2 [N*m]':^34}  {'slipped':>18}")
    for pose_id in sorted(by_pose):
        d = by_pose[pose_id]
        for plus, minus in AXIS_PAIRS:
            rp, rm = d.get(plus), d.get(minus)
            if rp is None or rm is None or not rp.broke or not rm.broke:
                continue
            if float(np.max(np.abs(rp.q0 - rm.q0))) > PAIR_POSE_TOL_RAD:
                continue
            c = (rp.tau_nm - rm.tau_nm) / 2.0
            slipped = f"{rp.slipped_joint}/{rm.slipped_joint}"
            neg = " <-- NEGATIVE, non-physical" if float(np.min(c)) < 0 else ""
            print(f"{pose_id:>4} {plus[1]:>5}  "
                  f"[{c[0]:+10.5f} {c[1]:+10.5f} {c[2]:+10.5f}]  {slipped:>18}{neg}")
            records.append(dict(pose_id=pose_id, axis=plus[1], c=c,
                                slipped=(rp.slipped_joint, rm.slipped_joint),
                                q=(rp.q0 + rm.q0) / 2.0))

    if not records:
        print("\nNo complete, non-drifted breakaway pairs. Nothing to estimate.")
        return 1

    C = np.array([r["c"] for r in records])

    print(f"\n=== Per-joint summary over {len(records)} pair(s) ===")
    print(f"{'joint':>9} {'median':>10} {'mean':>10} {'std':>10} {'min':>10} {'max':>10}   "
          f"{'n where this joint slipped':>28}")
    summary = {}
    for j, name in enumerate(common.JOINT_NAMES):
        col = C[:, j]
        n_dom = sum(1 for r in records if name in r["slipped"])
        print(f"{name:>9} {np.median(col):+10.5f} {col.mean():+10.5f} {col.std():10.5f} "
              f"{col.min():+10.5f} {col.max():+10.5f}   {n_dom:>28}")
        summary[name] = dict(
            median_nm=float(np.median(col)), mean_nm=float(col.mean()),
            std_nm=float(col.std()), min_nm=float(col.min()), max_nm=float(col.max()),
            n_pairs=len(col), n_pairs_this_joint_slipped=n_dom,
        )

    print("\nINTERPRETING THE SPREAD: a joint's c is best determined by the pairs where")
    print("that joint actually slipped (last column). Where it did not slip, its c is")
    print("mostly the projection of another joint's stiction through J^T and is not a")
    print("measurement of this joint at all. Prefer the median over the mean; one bad")
    print("pair moves the mean a long way with this few samples.")

    n_neg = int(np.sum(C < 0))
    if n_neg:
        print(f"\n{n_neg} of {C.size} per-joint entries are negative. A negative Coulomb")
        print("estimate is non-physical and means that pose was not a true static")
        print("equilibrium, or a different joint slipped in the + and - runs. Those")
        print("entries bound nothing and should not be averaged into a threshold.")

    floor = {n: summary[n]["median_nm"] for n in common.JOINT_NAMES}
    print("\n=== Friction floor for threshold_nm ===")
    for n in common.JOINT_NAMES:
        print(f"  {n:>9}: median |c| = {floor[n]:+.5f} N*m  "
              f"-> threshold_nm must exceed this AND the gravity term at that pose")
    print("\n  These are a FLOOR, not a threshold. touch_dob.hpp's guidance is to take")
    print("  the ~99th percentile of |tau_dis_hat| in free motion and multiply by")
    print("  1.5-2. That measurement needs the observer running, which needs M_n, so")
    print("  it comes after fit_inertia.py -- not from this script.")

    with open(args.out, "w") as fh:
        json.dump({"units": "N*m (real SI; J^T F with F in newtons)",
                   "caveat": common.CAVEAT_CARTESIAN_COUPLING,
                   "per_joint": summary,
                   "per_pair": [{"pose_id": r["pose_id"], "axis": r["axis"],
                                 "c_nm": r["c"].tolist(), "q": r["q"].tolist(),
                                 "slipped": list(r["slipped"])} for r in records]},
                  fh, indent=2)
    print(f"\nWritten: {args.out}")
    print("NOTHING was written to touch_dob.hpp. JointDobParams remain unset.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
