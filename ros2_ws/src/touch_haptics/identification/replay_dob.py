#!/usr/bin/env python3
"""Offline replay of the disturbance observer over logged captures.

WHAT THIS IS FOR
----------------
With M_n identified, the two remaining unset JointDobParams fields can be
determined WITHOUT running the observer on hardware:

  g_dis_rad_s   Q-filter cutoff. Trades noise against responsiveness. Sweeping
                it offline over real logged data shows that trade directly,
                which guessing from "2*pi*[20..30] Hz" does not.

  threshold_nm  Gate threshold. touch_dob.hpp's own guidance is the ~99th
                percentile of |tau_dis_hat| in free motion, times 1.5-2. That
                measurement needs the observer running -- but the observer is
                a pure function of (tau_cmd, q, t), all of which are already in
                the dynamic-excitation log. Replaying is equivalent to running
                it, and costs no hardware time or risk.

FAITHFULNESS TO THE C++ IMPLEMENTATION
--------------------------------------
The observer, verbatim from touch_dob.hpp:

    tau_dis_hat = g/(s+g) * [ tau_cmd + g*M_n*qdot ] - g*M_n*qdot

qdot MUST come from a single backward difference of joint angles, because that
is what JointDobBank does internally. This script deliberately does NOT reuse
the Savitzky-Golay derivative from fit_inertia.py: that filter is non-causal
and far cleaner than anything the real observer can see, so using it here would
produce an optimistic tau_dis_hat and a threshold set too low to be safe. The
identification step is allowed to use the better derivative because it runs
offline; the threshold must be derived from what the observer actually sees.

WHAT tau_dis_hat CONTAINS HERE
------------------------------
No contact occurred during these captures, so every newton-metre of
tau_dis_hat is friction, gravity, model error, or noise. That is precisely the
floor the gate has to clear. The header flags gravity as an open question --
whether a threshold above worst-case gravity would also sit above genuine
contact. This script answers it with numbers by reporting the floor both as the
observer stands (no gravity model) and with G(q) subtracted from tau_cmd.

Usage:
    python3 replay_dob.py dyn.csv --inertia inertia_left.json [--gravity g.json]
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np

import common
import fit_gravity

# Candidate Q-filter cutoffs. touch_dob.hpp suggests 2*pi*[20..30] Hz as a
# starting point; the range is widened here so the trade is visible rather
# than assumed.
CANDIDATE_HZ = (5.0, 10.0, 20.0, 30.0, 50.0, 80.0)


def observer(tau_cmd: np.ndarray, q: np.ndarray, dt: float, m_n: float,
             g_dis: float) -> np.ndarray:
    """tau_dis_hat for one joint, exactly the header's form.

    qdot is a single backward difference -- causal, quantisation-noisy, and
    what the real observer sees. Do not substitute a smoother derivative.
    """
    qdot = np.zeros_like(q)
    qdot[1:] = (q[1:] - q[:-1]) / dt

    u = tau_cmd + g_dis * m_n * qdot          # LPF input
    # Exact discrete first-order lag, not the Euler approximation: at 1 kHz
    # with g up to ~500 rad/s the Euler form is visibly off and would
    # misrepresent the very noise/lag trade being measured.
    alpha = 1.0 - np.exp(-g_dis * dt)
    y = np.empty_like(u)
    acc = 0.0
    for k in range(len(u)):
        acc += alpha * (u[k] - acc)
        y[k] = acc
    return y - g_dis * m_n * qdot


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dynamic_csv")
    ap.add_argument("--inertia", required=True, help="inertia_fit.json from fit_inertia.py")
    ap.add_argument("--gravity", default=None,
                    help="gravity_fit.json; if given, the floor is ALSO reported with "
                         "G(q) removed from tau_cmd, showing what gravity compensation "
                         "would buy")
    ap.add_argument("--out", default="dob_tuning.json")
    ap.add_argument("--trim", type=float, default=0.10)
    args = ap.parse_args()

    with open(args.inertia) as fh:
        inert = json.load(fh)
    m_n = {n: inert["joints"][n]["M_n_kg_m2"] for n in common.JOINT_NAMES}
    if any(v <= 0 for v in m_n.values()):
        raise SystemExit("inertia file contains a non-positive M_n; refusing to replay.")

    g_of_q = None
    if args.gravity:
        with open(args.gravity) as fh:
            gd = json.load(fh)
        if "EQUALS G(q)" not in gd.get("sign_convention", ""):
            raise SystemExit(f"{args.gravity}: missing/unrecognised sign_convention. "
                             "Regenerate with fit_gravity.py.")
        names = gd["basis"]
        coef = {n: np.array(gd["joints"][n]["coef"]) for n in common.JOINT_NAMES}
        g_of_q = lambda qq: np.array(  # noqa: E731
            [float(fit_gravity.basis(qq, names) @ coef[n]) for n in common.JOINT_NAMES])

    caps = [c for c in common.load_dynamic(args.dynamic_csv) if len(c.t) > 2000]
    if not caps:
        raise SystemExit("no usable captures")
    print(f"replaying {len(caps)} captures, M_n = "
          + ", ".join(f"{n}={m_n[n]:.3e}" for n in common.JOINT_NAMES))
    print("\nNo contact occurred in these captures, so ALL of tau_dis_hat below is")
    print("friction + gravity + model error + noise. That is the floor the gate")
    print("must clear -- not a contact signal.\n")

    result = {"m_n": m_n, "candidates": {}, "units": "N*m",
              "qdot_source": "single backward difference (matches JointDobBank)"}

    for label, use_g in (("as implemented (NO gravity model)", False),
                         ("with G(q) subtracted", True)):
        if use_g and g_of_q is None:
            continue
        print(f"=== {label} ===")
        print(f"{'g_dis':>8} {'Hz':>6}  " + "".join(f"{n[:5]+' p99':>13}" for n in common.JOINT_NAMES)
              + "".join(f"{n[:5]+' max':>13}" for n in common.JOINT_NAMES))
        for hz in CANDIDATE_HZ:
            g_dis = 2.0 * np.pi * hz
            p99 = {}
            mx = {}
            for j, name in enumerate(common.JOINT_NAMES):
                vals = []
                for c in caps:
                    dt = c.dt
                    tau = np.array([common.tau_from_force(c.q[i], c.cmd[i])[j]
                                    for i in range(len(c.t))])
                    if use_g:
                        tau = tau - np.array([g_of_q(c.q[i])[j] for i in range(len(c.t))])
                    th = observer(tau, c.q[:, j], dt, m_n[name], g_dis)
                    k = int(len(th) * args.trim)
                    vals.append(np.abs(th[k:len(th) - k]))
                v = np.concatenate(vals)
                p99[name] = float(np.percentile(v, 99))
                mx[name] = float(v.max())
            print(f"{g_dis:8.1f} {hz:6.1f}  "
                  + "".join(f"{p99[n]:13.5f}" for n in common.JOINT_NAMES)
                  + "".join(f"{mx[n]:13.5f}" for n in common.JOINT_NAMES))
            result["candidates"].setdefault(label, {})[f"{hz:g}"] = {
                "g_dis_rad_s": g_dis, "p99_nm": p99, "max_nm": mx}
        print()

    # ---- suggested settings ------------------------------------------------
    key = "as implemented (NO gravity model)"
    print("=== suggested threshold_nm (p99 x 2, per touch_dob.hpp guidance) ===")
    print(f"{'g_dis Hz':>9}  " + "".join(f"{n:>12}" for n in common.JOINT_NAMES))
    for hz in CANDIDATE_HZ:
        e = result["candidates"][key][f"{hz:g}"]
        print(f"{hz:9.1f}  " + "".join(f"{2*e['p99_nm'][n]:12.5f}" for n in common.JOINT_NAMES))

    print("\n=== max_torque_nm, from the header's own helper ===")
    print("  derivePerJointTorqueLimits(f_max) = f_max * [L1+L2, L1, L2]")
    for f_max in (0.6, 1.0, 1.5):
        lim = [f_max * (common.L1_METERS + common.L2_METERS),
               f_max * common.L1_METERS, f_max * common.L2_METERS]
        print(f"  f_max = {f_max:.2f} N -> " +
              "  ".join(f"{n}={v:.5f}" for n, v in zip(common.JOINT_NAMES, lim)))
    print("\n  CAVEAT from touch_dob.hpp: three independent per-joint limits are NOT")
    print("  equivalent to the Cartesian clampNorm they replace. All three at their")
    print("  limit can exceed the old Cartesian ceiling. Choose conservatively.")

    with open(args.out, "w") as fh:
        json.dump(result, fh, indent=2)
    print(f"\nWritten: {args.out}")
    print("\nNOTHING was written to touch_dob.hpp. These are candidates for the")
    print("operator to choose from, not values that have been applied.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
