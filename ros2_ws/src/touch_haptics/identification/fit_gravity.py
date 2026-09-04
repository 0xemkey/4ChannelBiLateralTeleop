#!/usr/bin/env python3
"""Empirical gravity map G_n(q) from hdapi_static_sweep logs.

METHOD
------
At breakaway the joint is in static equilibrium against commanded torque,
gravity, and static friction. Pushing + and - along the same Cartesian axis:

    tau_plus  =  c - g
    tau_minus = -c - g
  =>    g = -(tau_plus + tau_minus) / 2      gravity torque
        c =  (tau_plus - tau_minus) / 2      Coulomb magnitude

Each of the three axis pairs at a pose gives an INDEPENDENT estimate of the same
gravity vector. Their spread is the internal consistency check: wide spread
means the decomposition did not hold at that pose and it is dropped.

WHY A FITTED FORM RATHER THAN A LOOKUP TABLE
--------------------------------------------
Both are produced. The lookup (per-pose table) is the raw truth and is always
written. The fitted analytical form is additionally attempted because the DOB
needs G_n(q) at arbitrary q, not only at the ~10 poses that were sampled, and
interpolating scattered 3-D data from a handful of points is worse-conditioned
than fitting a basis that already has the right physical shape.

The basis is not arbitrary. For a 2-link arm in a vertical plane the
gravitational potential depends only on the shoulder and elbow angles, so
gravity torque is a linear combination of {1, cos q2, sin q2, cos q3, sin q3,
cos(q2-q3), sin(q2-q3)} and NOT a function of q1 at all -- the waist rotates
about the vertical axis, which cannot change potential energy. That q1
independence is a physical prediction, and the script checks it: if the fitted
waist gravity is not near zero, something is wrong with either J(q) or the
assumption that the base is level. It is reported, not silently absorbed.

The fit is linear in its coefficients, but scipy.optimize.least_squares is used
anyway so that a robust loss (soft_l1) can down-weight poses where the
breakaway decomposition partly failed, instead of letting one bad pose drag the
whole map.

Usage:
    python3 fit_gravity.py sweep.csv [--out gravity_fit.json] [--robust]
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np
from scipy.optimize import least_squares

import common
from common import jacobian

AXIS_PAIRS = (("+X", "-X"), ("+Y", "-Y"), ("+Z", "-Z"))
PAIR_POSE_TOL_RAD = 0.05      # matches kPairPoseTolRad in the C++ tool
WIDE_SPREAD_NM = 0.02         # matches the C++ tool's "decomposition suspect"


# Two bases. MINIMAL is the default and is what the kinematics actually implies.
#
# With this FK (y = L1*sin q2 - L2*cos q3) the two links' heights depend on q2
# and q3 SEPARATELY -- the angles are not chained -- so potential energy is a
# sum of a q2 term and a q3 term with no cross term. The minimal basis says
# exactly that and nothing more.
#
# FULL adds a constant and the cos/sin(q2-q3) cross terms. Over the limited
# angular range a real pose set covers, those are very nearly collinear with
# the minimal terms: on a 10-pose synthetic set the full basis reached a design
# condition number of ~5e4 and produced coefficients of +/-0.55 to fit data
# whose true coefficients were ~0.02. Residuals stayed tiny -- collinearity
# always fits the training points -- but such coefficients are individually
# meaningless and extrapolate badly, which is fatal here because the DOB must
# evaluate G_n(q) at poses that were never sampled. Use FULL only with many
# well-spread poses AND a design condition number that stays low.
BASIS_MINIMAL = ["cos q2", "sin q2", "cos q3", "sin q3"]
BASIS_FULL = ["1", "cos q2", "sin q2", "cos q3", "sin q3", "cos(q2-q3)", "sin(q2-q3)"]

# TILT: for a base that is not level. A tilted base gives gravity a horizontal
# component, which CAN load the waist -- a joint that on a level base cannot be
# loaded by gravity at all. The signature is a sinusoid in q1.
#
# Measured on this rig 2026-08-24 (session B, 10 poses): correlation between q1
# and waist gravity -0.841, in-sample R^2 0.78, implying roughly 3-6 degrees of
# tilt depending on assumed arm mass.
#
# BUT -- and this is why it is not the default -- leave-one-out cross-validation
# says these two extra terms make the map WORSE out of sample on every joint:
#
#     basis            waist LOO   shoulder LOO   elbow LOO
#     minimal (4)        0.01133      0.00732      0.00692
#     tilt plain (6)     0.01516      0.00783      0.01355
#     tilt x reach (6)   0.01416      0.00788      0.01194
#
# The waist TRAINING residual halves (0.0081 -> 0.0033), which is exactly the
# trap: two more parameters always fit ten points better. With this many poses
# the tilt terms are fitting noise. Use --basis tilt only with substantially
# more poses, and check the LOO numbers again before believing it.
#
# MOUNT-SPECIFIC: a fit using these terms encodes the angle of THIS table and
# THIS placement. Move or re-level the device and the map is silently wrong.
BASIS_TILT = ["cos q2", "sin q2", "cos q3", "sin q3", "sin q1", "cos q1"]
BASIS_TILT_REACH = ["cos q2", "sin q2", "cos q3", "sin q3", "r*sin q1", "r*cos q1"]

# Module-level so fit_inertia.py's gravity_predictor stays in step with whatever
# basis the fit was produced with; the JSON records the names explicitly.
BASIS_NAMES = BASIS_MINIMAL

# Above this, coefficients are not individually trustworthy.
COND_WARN = 1.0e3


_BASES = {"minimal": BASIS_MINIMAL, "full": BASIS_FULL,
          "tilt": BASIS_TILT, "tilt-reach": BASIS_TILT_REACH}


def set_basis(name: str) -> None:
    global BASIS_NAMES
    BASIS_NAMES = _BASES.get(name, BASIS_MINIMAL)
    if name in ("tilt", "tilt-reach"):
        print("NOTE: tilt basis selected. On the 10-pose session-B data these terms")
        print("      IMPROVED the training residual but WORSENED leave-one-out error")
        print("      on every joint. They also make the map specific to this exact")
        print("      mounting. See the BASIS_TILT comment before trusting the result.")


def basis(q: np.ndarray, names: list[str] | None = None) -> np.ndarray:
    """Gravity basis evaluated at q.

    The q1 terms are present only for a non-level base: on a level one gravity
    cannot load a joint that rotates about the vertical, so including them
    invites the fit to absorb friction asymmetry as if it were gravity.
    """
    names = names if names is not None else BASIS_NAMES
    q1, q2, q3 = q[0], q[1], q[2]
    r = common.L1_METERS * np.cos(q2) + common.L2_METERS * np.sin(q3)
    table = {
        "1": 1.0,
        "cos q2": np.cos(q2), "sin q2": np.sin(q2),
        "cos q3": np.cos(q3), "sin q3": np.sin(q3),
        "cos(q2-q3)": np.cos(q2 - q3), "sin(q2-q3)": np.sin(q2 - q3),
        "sin q1": np.sin(q1), "cos q1": np.cos(q1),
        "r*sin q1": r * np.sin(q1), "r*cos q1": r * np.cos(q1),
    }
    return np.array([table[n] for n in names])


def decompose(rows):
    """Per pose, return (q0_mean, g_mean[3], g_spread[3], c_per_pair, n_pairs, flags)."""
    by_pose: dict[int, dict[str, common.SweepRow]] = {}
    for r in rows:
        by_pose.setdefault(r.pose_id, {})[r.direction] = r

    out = []
    for pose_id in sorted(by_pose):
        d = by_pose[pose_id]
        g_est, c_est, q_used, flags = [], [], [], []
        for plus, minus in AXIS_PAIRS:
            rp, rm = d.get(plus), d.get(minus)
            if rp is None or rm is None or not rp.broke or not rm.broke:
                continue
            drift = float(np.max(np.abs(rp.q0 - rm.q0)))
            if drift > PAIR_POSE_TOL_RAD:
                flags.append(f"{plus}/{minus} pose drifted {drift:.3f} rad -- pair dropped")
                continue
            # Remove the known support bias. With a commanded bias B held
            # through both ramps:
            #     tau_plus = c - (g + J^T B)     tau_minus = -c - (g + J^T B)
            # so the raw estimate below is (g + J^T B) and the bias cancels
            # from c entirely. Subtracting J^T(q0) @ B recovers true gravity.
            # Without this the bias would masquerade as gravity -- and it is
            # pose-dependent through J(q), so it would not even look like a
            # constant offset.
            # Each ramp's bias torque must be evaluated at ITS OWN pose and the
            # two averaged. Using J at the mid-pose instead is NOT equivalent:
            # J is nonlinear in q, so J((q+ + q-)/2) != (J(q+) + J(q-))/2. With
            # a 0.05 rad pose tolerance and a 0.4 N bias that shortcut leaves a
            # ~1e-4 N*m error -- about 1% of the gravity signal being measured,
            # i.e. small but entirely avoidable. Caught by the synthetic
            # bias-removal check, which requires exact cancellation.
            q_mid = (rp.q0 + rm.q0) / 2.0
            tau_bias = np.zeros(3)
            if np.any(rp.bias_n) or np.any(rm.bias_n):
                tau_bias = 0.5 * (jacobian(rp.q0).T @ rp.bias_n
                                  + jacobian(rm.q0).T @ rm.bias_n)
            if np.any(np.abs(rp.bias_n - rm.bias_n) > 1e-9):
                flags.append(f"{plus}/{minus} bias differed between ramps "
                             f"({rp.bias_n} vs {rm.bias_n}) -- pair suspect")

            # SIGN CONVENTION -- read this before changing anything here.
            #
            # The breakaway algebra gives -(tau_plus + tau_minus)/2, which is
            # MINUS the torque needed to hold the arm up. Emitting that directly
            # is a trap: the standard manipulator form is
            #
            #     tau = M*qddot + C*qdot + G(q)      =>  at rest, tau = G(q)
            #
            # so every consumer reasonably expects G(q) to BE the holding
            # torque. Emitting its negative caused fit_inertia to subtract
            # gravity twice over: on real data (2026-08-24) that left 86% of the
            # shoulder's variance and 76% of the elbow's as an unexplained
            # per-capture offset of ~0.11 N*m -- larger than gravity itself --
            # and drove R^2 to -6.9 and -12.3 with non-physical negative damping.
            #
            # Negated here so the output matches the standard convention, and
            # the JSON records "sign_convention" so a consumer can verify rather
            # than assume. fit_inertia refuses to run without that field.
            g_est.append((rp.tau_nm + rm.tau_nm) / 2.0 + tau_bias)
            c_est.append((rp.tau_nm - rm.tau_nm) / 2.0)
            q_used.append(q_mid)

        if not g_est:
            out.append(dict(pose_id=pose_id, usable=False, n_pairs=0,
                            flags=flags + ["no usable axis pair"]))
            continue

        G = np.array(g_est)
        spread = G.max(axis=0) - G.min(axis=0) if len(G) > 1 else np.zeros(3)
        if len(G) > 1 and float(spread.max()) > WIDE_SPREAD_NM:
            flags.append(f"wide gravity spread {spread.max():.4f} N*m -- decomposition suspect")
        out.append(dict(
            pose_id=pose_id, usable=True, n_pairs=len(G),
            q=np.mean(q_used, axis=0), g=G.mean(axis=0), g_spread=spread,
            c=np.array(c_est), flags=flags,
        ))
    return out


def fit_joint(qs: np.ndarray, gs: np.ndarray, robust: bool):
    """Least-squares fit of one joint's gravity torque over the basis."""
    A = np.array([basis(q) for q in qs])

    def resid(coef):
        return A @ coef - gs

    x0 = np.zeros(A.shape[1])
    loss = "soft_l1" if robust else "linear"
    sol = least_squares(resid, x0, loss=loss, f_scale=0.005)
    pred = A @ sol.x
    res = pred - gs
    ss_tot = float(np.sum((gs - gs.mean()) ** 2))
    r2 = 1.0 - float(np.sum(res ** 2)) / ss_tot if ss_tot > 1e-18 else float("nan")
    cond = float(np.linalg.cond(A)) if A.shape[0] >= A.shape[1] else float("inf")
    return sol.x, res, r2, cond


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sweep_csv")
    ap.add_argument("--out", default="gravity_fit.json")
    ap.add_argument("--basis", choices=("minimal", "full", "tilt", "tilt-reach"),
                    default="minimal",
                    help="minimal (default, 4 terms, what a LEVEL base implies); "
                         "full (7 terms, needs many poses); "
                         "tilt / tilt-reach (6 terms, for a non-level base -- "
                         "mount-specific, and on 10 poses these worsened "
                         "leave-one-out error on every joint)")
    ap.add_argument("--robust", action="store_true",
                    help="soft_l1 loss, down-weights poses whose decomposition partly failed")
    args = ap.parse_args()

    set_basis(args.basis)
    rows = common.load_static_sweep(args.sweep_csv)
    poses = decompose(rows)
    usable = [p for p in poses if p.get("usable")]

    print("=== Per-pose gravity decomposition (REAL N*m, no torque calibration needed) ===")
    print(f"{'pose':>4} {'pairs':>5}  {'q [rad]':^26}  {'g = gravity torque [N*m]':^30}  {'spread':^10}")
    for p in poses:
        if not p.get("usable"):
            print(f"{p['pose_id']:>4} {p['n_pairs']:>5}  -- unusable: {'; '.join(p['flags'])}")
            continue
        q, g, s = p["q"], p["g"], p["g_spread"]
        print(f"{p['pose_id']:>4} {p['n_pairs']:>5}  "
              f"[{q[0]:+.3f} {q[1]:+.3f} {q[2]:+.3f}]  "
              f"[{g[0]:+9.5f} {g[1]:+9.5f} {g[2]:+9.5f}]  {s.max():9.5f}")
        for f in p["flags"]:
            print(f"        NOTE: {f}")

    if len(usable) < 3:
        print(f"\nOnly {len(usable)} usable pose(s). The per-pose table above IS the gravity")
        print("map for now; a basis fit needs more poses than free coefficients (7).")
        print("Collect more poses before fitting an analytical form.")
        return 1

    qs = np.array([p["q"] for p in usable])
    gs = np.array([p["g"] for p in usable])

    print(f"\n=== Analytical fit over {len(usable)} poses, basis {BASIS_NAMES} ===")
    if len(usable) < len(BASIS_NAMES) + 2:
        print(f"WARNING: {len(usable)} poses against {len(BASIS_NAMES)} coefficients is")
        print("         barely determined. Treat the fit as provisional and add poses.")

    result = {"n_poses": len(usable), "basis": BASIS_NAMES,
              "sign_convention": "G(q) as in tau = M*qddot + C*qdot + G(q); "
                                 "at rest the commanded joint torque EQUALS G(q)",
              "units": "N*m (real SI; derived from J^T F with F in newtons)",
              "robust_loss": bool(args.robust), "joints": {}}

    for j, name in enumerate(common.JOINT_NAMES):
        coef, res, r2, cond = fit_joint(qs, gs[:, j], args.robust)
        rms = float(np.sqrt(np.mean(res ** 2)))
        print(f"\n  {name}:")
        print("    coef " + "  ".join(f"{n}={c:+.5f}" for n, c in zip(BASIS_NAMES, coef)))
        warn = "   <-- ILL-CONDITIONED, coefficients not individually meaningful" \
               if cond > COND_WARN else ""
        print(f"    RMS residual {rms:.5f} N*m   R^2 {r2:+.4f}   design cond {cond:.1f}{warn}")
        if cond > COND_WARN:
            print("        A low residual here does NOT mean a good map: collinear terms")
            print("        always fit the sampled poses. Add poses spanning a wider q2/q3")
            print("        range, or use --basis minimal, before trusting extrapolation.")
        worst = int(np.argmax(np.abs(res)))
        print(f"    worst pose {usable[worst]['pose_id']} residual {res[worst]:+.5f} N*m")
        result["joints"][name] = {
            "coef": coef.tolist(), "rms_residual_nm": rms, "r2": r2,
            "design_cond": cond,
            "per_pose_residual_nm": {str(usable[i]["pose_id"]): float(res[i])
                                     for i in range(len(usable))},
        }

    # Physical check: gravity must not act on the waist.
    waist_mag = float(np.max(np.abs(gs[:, 0])))
    print(f"\n=== Physical check: waist gravity torque should be ~0 ===")
    print(f"  max |g_waist| across poses: {waist_mag:.5f} N*m")
    if waist_mag > 0.01:
        print("  <-- NOT near zero. Gravity cannot load a joint that rotates about the")
        print("      vertical. Suspect the J(q) convention (run hdapi_fk_residual), a")
        print("      non-level base, or friction leaking into the g estimate.")
    else:
        print("  consistent with a level base and a correct J(q) convention.")
    result["waist_gravity_max_abs_nm"] = waist_mag

    result["per_pose_table"] = [
        {"pose_id": p["pose_id"], "q": p["q"].tolist(), "g_nm": p["g"].tolist(),
         "g_spread_nm": p["g_spread"].tolist(), "n_pairs": p["n_pairs"],
         "flags": p["flags"]}
        for p in usable
    ]

    with open(args.out, "w") as fh:
        json.dump(result, fh, indent=2)
    print(f"\nWritten: {args.out}")
    print("\nNOTHING was written to touch_dob.hpp. JointDobParams remain unset.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
