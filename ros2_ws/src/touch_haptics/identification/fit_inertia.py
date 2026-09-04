#!/usr/bin/env python3
"""Nominal inertia M_n per joint from hdapi_dynamic_excitation logs.

THE REGRESSION
--------------
Per joint j, over every sample of every capture:

    tau_j(t) - g_j(q(t))  =  M_j * qddot_j(t)  +  b_j * qdot_j(t)  +  c_j * sgn(qdot_j(t))
                             \_ inertia _/       \_ viscous _/       \_ Coulomb _/

tau_j comes from J^T(q) @ F_cmd, which is REAL N*m because F_cmd is a Cartesian
force we chose, in newtons (see common.py's units note). No torque-scale
calibration is required, so this runs before hdapi_torque_calib.cpp.

g_j is subtracted using the fit from fit_gravity.py. If that fit is missing or
poor, M_n absorbs the gravity error -- gravity is constant at a fixed pose while
inertia multiplies qddot, so the chirp's frequency sweep does separate them, but
only if there is real bandwidth in the data. Hence the chirp rather than a
single sinusoid.

WHY OFFLINE DIFFERENTIATION IS LEGITIMATE HERE
----------------------------------------------
qdot and qddot come from a Savitzky-Golay filter: a local polynomial fit whose
analytic derivative is evaluated at the window centre. It is NON-CAUSAL -- it
uses samples from both before and after each point -- so it has ZERO phase lag
and heavily attenuated high-frequency noise, which is exactly what naive finite
differencing of quantised encoder data gets wrong.

The real-time DOB cannot do this: at time t it does not have t+1. It is stuck
with a causal filter, which must trade phase lag against noise rejection, and
that lag is precisely what the audit flagged as destabilising in the reflection
loop's damping term. Offline identification has no such constraint, so it should
not inherit the online filter's compromises. Using a worse derivative here
purely to "match" the observer would inject avoidable error into the very
constants the observer depends on.

WHAT M_n IS AND IS NOT
----------------------
This fits a CONSTANT M_j per joint, ignoring the configuration dependence of the
true M(q) and all Coriolis/centrifugal coupling. That is deliberate and matches
touch_dob.hpp: a constant nominal inertia is what keeps the Q-filter LTI and is
the true Ohnishi form. The fitted value is therefore a NOMINAL inertia that
absorbs mean coupling over the sampled poses -- not a rigid-body inertia you
could look up in a CAD model. Fitting it from data spanning the poses the
observer will actually run at is the point.

Usage:
    python3 fit_inertia.py dyn.csv --gravity gravity_fit.json [--friction friction_fit.json]
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

import common
import fit_gravity

# Manufacturer figure for the cross-check.
# Omni Device Guide, Appendix F: "Inertia (apparent mass at tip) ~45 g".
TIP_INERTIA_SPEC_KG = 0.045

SAVGOL_WINDOW = 51      # samples @ ~1 kHz -> 51 ms; > one period at the top
SAVGOL_ORDER = 3        # cubic: 2 derivatives available, minimal over-smoothing
SGN_EPS = 0.02          # rad/s, tanh width standing in for sgn()


def derivatives(t: np.ndarray, q: np.ndarray, dt: float):
    """qdot, qddot by non-causal Savitzky-Golay. See module docstring."""
    n = len(t)
    win = min(SAVGOL_WINDOW, n if n % 2 else n - 1)
    if win <= SAVGOL_ORDER + 1:
        raise ValueError(f"capture too short for SG filter ({n} samples)")
    qs = np.column_stack([savgol_filter(q[:, j], win, SAVGOL_ORDER) for j in range(3)])
    qd = np.column_stack([savgol_filter(q[:, j], win, SAVGOL_ORDER, deriv=1, delta=dt)
                          for j in range(3)])
    qdd = np.column_stack([savgol_filter(q[:, j], win, SAVGOL_ORDER, deriv=2, delta=dt)
                           for j in range(3)])
    return qs, qd, qdd


def gravity_predictor(gravity_json: str | None):
    """Return g(q) -> (3,) in N*m, or a zero function if no fit was supplied."""
    if not gravity_json:
        return (lambda q: np.zeros(3)), False
    with open(gravity_json) as fh:
        data = json.load(fh)
    coefs = {n: np.array(data["joints"][n]["coef"]) for n in common.JOINT_NAMES}
    # Read the basis back from the JSON rather than assuming the default. The
    # fit may have been produced with --basis full, and silently evaluating a
    # 7-coefficient fit against a 4-term basis would be a wrong-but-plausible
    # gravity subtraction -- the worst kind of error to have in this pipeline.
    names = data.get("basis")
    if not names:
        raise SystemExit(f"{gravity_json}: no 'basis' field; regenerate with fit_gravity.py")
    # Refuse a fit whose sign convention is not stated. An inverted gravity map
    # does not fail loudly -- it produces a plausible-looking fit with a large
    # per-pose offset, negative damping and, on this data, negative inertia.
    # Requiring the field makes that failure impossible to reach silently.
    sc = data.get("sign_convention", "")
    if "EQUALS G(q)" not in sc:
        raise SystemExit(
            f"{gravity_json}: missing or unrecognised 'sign_convention'.\n"
            f"  got: {sc!r}\n"
            "  This file predates the sign fix of 2026-08-24, when fit_gravity\n"
            "  emitted MINUS the holding torque. Regenerate it with fit_gravity.py.")
    for n in common.JOINT_NAMES:
        if len(coefs[n]) != len(names):
            raise SystemExit(
                f"{gravity_json}: joint {n} has {len(coefs[n])} coefficients but the "
                f"recorded basis has {len(names)} terms. Refusing to guess.")

    def g(q):
        b = fit_gravity.basis(q, names)
        return np.array([float(b @ coefs[n]) for n in common.JOINT_NAMES])

    return g, True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dynamic_csv")
    ap.add_argument("--gravity", default=None, help="gravity_fit.json from fit_gravity.py")
    ap.add_argument("--friction", default=None, help="friction_fit.json, for cross-check only")
    ap.add_argument("--out", default="inertia_fit.json")
    ap.add_argument("--keep-clipped", action="store_true",
                    help="use captures whose chirp hit the force clamp. They are "
                         "distorted and bias M_n upward; only for when there is "
                         "nothing else available, and say so in any result.")
    ap.add_argument("--trim", type=float, default=0.10,
                    help="fraction trimmed from each end (envelope ramp region)")
    args = ap.parse_args()

    caps = common.load_dynamic(args.dynamic_csv)
    g_of_q, have_gravity = gravity_predictor(args.gravity)

    print("=== Dynamic excitation captures ===")
    print(f"{'pose':>4} {'axis':>4} {'n':>7} {'dt [ms]':>9} {'rate [Hz]':>10} "
          f"{'peak |qdot| [rad/s]':>20} {'peak |qddot|':>14}")

    rows_A = {j: [] for j in range(3)}
    rows_y = {j: [] for j in range(3)}
    peak_qd = np.zeros(3)
    q_all = []

    n_clipped_dropped = 0
    for c in caps:
        dt = c.dt
        if not np.isfinite(dt) or dt <= 0:
            print(f"{c.pose_id:>4} {c.axis:>4}  unusable timestamps -- skipped")
            continue
        # A clipped chirp is distorted, not attenuated: the flattened peaks are
        # broadband, and that energy gets attributed to inertia. Dropping such a
        # capture loses data; keeping it biases M_n. Dropping is the lesser harm,
        # and --keep-clipped exists for when there is nothing else to work with.
        if c.clipped and not args.keep_clipped:
            print(f"{c.pose_id:>4} {c.axis:>4}  CLIPPED on {c.clipped} frame(s) -- dropped "
                  f"(--keep-clipped to override)")
            n_clipped_dropped += 1
            continue
        try:
            qs, qd, qdd = derivatives(c.t, c.q, dt)
        except ValueError as exc:
            print(f"{c.pose_id:>4} {c.axis:>4}  {exc} -- skipped")
            continue

        k = int(len(c.t) * args.trim)
        sl = slice(k, len(c.t) - k) if len(c.t) - 2 * k > 50 else slice(None)

        print(f"{c.pose_id:>4} {c.axis:>4} {len(c.t):>7} {dt*1e3:>9.3f} {1.0/dt:>10.1f} "
              f"{np.abs(qd[sl]).max():>20.4f} {np.abs(qdd[sl]).max():>14.2f}")

        for i in range(*sl.indices(len(c.t))):
            q_i = qs[i]
            tau = common.tau_from_force(q_i, c.cmd[i])      # REAL N*m
            g_i = g_of_q(q_i)
            for j in range(3):
                rows_A[j].append([qdd[i, j], qd[i, j], np.tanh(qd[i, j] / SGN_EPS)])
                rows_y[j].append(tau[j] - g_i[j])
            q_all.append(q_i)
        peak_qd = np.maximum(peak_qd, np.abs(qd[sl]).max(axis=0))

    if not q_all:
        print("\nNo usable captures. Nothing to fit.")
        return 1

    if n_clipped_dropped:
        print(f"\n{n_clipped_dropped} capture(s) dropped for clipping. If that leaves too")
        print("few, re-run the hardware step with a lower --amp rather than --keep-clipped.")

    if not have_gravity:
        print("\nWARNING: no --gravity fit supplied. Gravity was NOT subtracted, so the")
        print("         fitted Coulomb term absorbs it and M_n is biased. Run")
        print("         fit_gravity.py first and pass --gravity. Results below are")
        print("         provisional only.")

    print(f"\n=== Least-squares fit, {len(q_all)} samples per joint ===")
    print("model:  tau - g(q)  =  M*qddot + b*qdot + c*tanh(qdot/eps)")
    print(f"        eps = {SGN_EPS} rad/s (smooth stand-in for sgn, avoids a discontinuity)")
    print(common.CAVEAT_CARTESIAN_COUPLING)

    result = {
        "units": {"M_n": "kg*m^2", "b": "N*m*s/rad", "c": "N*m",
                  "basis": "REAL SI -- from J^T F with F in newtons; "
                           "no HD_LAST_JOINT_TORQUE scale factor involved"},
        "gravity_subtracted": have_gravity,
        "n_samples": len(q_all),
        "savgol": {"window": SAVGOL_WINDOW, "order": SAVGOL_ORDER, "noncausal": True},
        "joints": {},
    }

    M_fit = np.zeros(3)
    print(f"\n{'joint':>9} {'M_n [kg m^2]':>14} {'b [N m s]':>12} {'c [N m]':>10} "
          f"{'RMS res':>10} {'R^2':>8} {'cond':>10} {'peak qdot':>10}")
    for j, name in enumerate(common.JOINT_NAMES):
        A = np.array(rows_A[j])
        y = np.array(rows_y[j])

        def resid(p, A=A, y=y):
            return A @ p - y

        sol = least_squares(resid, np.array([1e-4, 1e-3, 1e-3]))
        M, b, c = sol.x
        M_fit[j] = M
        r = A @ sol.x - y
        rms = float(np.sqrt(np.mean(r ** 2)))
        ss_tot = float(np.sum((y - y.mean()) ** 2))
        r2 = 1.0 - float(np.sum(r ** 2)) / ss_tot if ss_tot > 1e-18 else float("nan")
        cond = float(np.linalg.cond(A))

        flag = ""
        if M <= 0:
            flag = "  <-- NON-PHYSICAL (M<=0)"
        elif cond > 1e4:
            flag = "  <-- ill-conditioned"
        elif peak_qd[j] < 0.05:
            flag = "  <-- joint barely moved"

        print(f"{name:>9} {M:>14.6e} {b:>12.5f} {c:>10.5f} {rms:>10.5f} {r2:>8.4f} "
              f"{cond:>10.1f} {peak_qd[j]:>10.4f}{flag}")

        result["joints"][name] = dict(
            M_n_kg_m2=float(M), b_visc=float(b), c_coulomb_nm=float(c),
            rms_residual_nm=rms, r2=r2, design_cond=cond,
            peak_qdot_rad_s=float(peak_qd[j]),
            well_conditioned=bool(M > 0 and cond < 1e4 and peak_qd[j] >= 0.05),
        )

    # ---- Coulomb cross-check against the static sweep -------------------
    if args.friction:
        try:
            with open(args.friction) as fh:
                fr = json.load(fh)
        except (json.JSONDecodeError, OSError) as exc:
            print(f"\nNOTE: could not read --friction {args.friction} ({exc}).")
            print("      Skipping the Coulomb cross-check; the fit above is unaffected.")
            fr = None
    else:
        fr = None
    if fr is not None:
        print("\n=== Coulomb cross-check: dynamic regression vs static breakaway ===")
        print(f"{'joint':>9} {'dynamic c':>12} {'static median':>15} {'ratio':>8}")
        for name in common.JOINT_NAMES:
            dyn = result["joints"][name]["c_coulomb_nm"]
            sta = fr["per_joint"][name]["median_nm"]
            ratio = dyn / sta if abs(sta) > 1e-9 else float("nan")
            print(f"{name:>9} {dyn:>12.5f} {sta:>15.5f} {ratio:>8.2f}")
        print("  These are different experiments (breakaway vs sliding), so exact")
        print("  agreement is not expected -- sliding friction is normally BELOW")
        print("  breakaway. A ratio far above 1, or an opposite sign, means one of the")
        print("  two is wrong and neither should be used to set threshold_nm.")

    # ---- Apparent tip inertia cross-check -------------------------------
    print(f"\n=== Cross-check: apparent mass at tip vs manufacturer spec ===")
    print(f"spec: Omni Device Guide, Appendix F -- ~{TIP_INERTIA_SPEC_KG*1e3:.0f} g")
    if np.any(M_fit <= 0):
        print("  skipped: a fitted M_n is non-positive, so M^-1 is meaningless.")
        result["tip_inertia_check"] = {"status": "skipped_nonpositive_M"}
    else:
        Minv = np.diag(1.0 / M_fit)
        axes = {"x": np.array([1.0, 0, 0]), "y": np.array([0, 1.0, 0]), "z": np.array([0, 0, 1.0])}
        qs_sample = np.array(q_all)[:: max(1, len(q_all) // 400)]
        print(f"{'axis':>5} {'median [g]':>12} {'p10 [g]':>10} {'p90 [g]':>10}")
        tip = {}
        for an, u in axes.items():
            vals = []
            for q_i in qs_sample:
                J = common.jacobian(q_i)
                denom = float(u @ (J @ Minv @ J.T) @ u)
                if denom > 1e-12:
                    vals.append(1.0 / denom)
            if not vals:
                continue
            v = np.array(vals) * 1e3   # grams
            print(f"{an:>5} {np.median(v):>12.1f} {np.percentile(v,10):>10.1f} "
                  f"{np.percentile(v,90):>10.1f}")
            tip[an] = dict(median_g=float(np.median(v)),
                           p10_g=float(np.percentile(v, 10)),
                           p90_g=float(np.percentile(v, 90)))
        med = np.median([t["median_g"] for t in tip.values()]) if tip else float("nan")
        ratio = med / (TIP_INERTIA_SPEC_KG * 1e3)
        print(f"\n  overall median {med:.1f} g vs spec {TIP_INERTIA_SPEC_KG*1e3:.0f} g "
              f"-> ratio {ratio:.2f}x")
        if 0.33 <= ratio <= 3.0:
            verdict = ("PLAUSIBLE -- same order as the spec. This is a sanity check, not "
                       "a validation: it cannot detect a fit that is wrong by less than "
                       "about 3x, and the spec is a single tip figure while M_n is per joint.")
        else:
            verdict = ("IMPLAUSIBLE -- off by more than 3x. Suspect, in order: the gravity "
                       "fit, the J(q) convention (run hdapi_fk_residual), too little "
                       "acceleration in the captures, or a joint that barely moved.")
        print(f"  VERDICT: {verdict}")
        result["tip_inertia_check"] = dict(spec_g=TIP_INERTIA_SPEC_KG * 1e3,
                                           per_axis=tip, overall_median_g=float(med),
                                           ratio_vs_spec=float(ratio), verdict=verdict)

    with open(args.out, "w") as fh:
        json.dump(result, fh, indent=2)
    print(f"\nWritten: {args.out}")
    print("\nNOTHING was written to touch_dob.hpp. JointDobParams.m_n_kg_m2 remains")
    print("at its unset sentinel and dob_bank_ stays inactive. Assigning these values")
    print("is a separate, explicit step for the operator to authorise.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
