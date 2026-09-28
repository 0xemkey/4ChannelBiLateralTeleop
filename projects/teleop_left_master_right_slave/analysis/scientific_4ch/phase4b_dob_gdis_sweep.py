#!/usr/bin/env python3
"""Phase 4B: DOB free-space floor across the 3 CANDIDATE g_dis values that
already exist in this codebase's own history (touch_slave.cpp's own
comment table, hardware-measured, 2026-08-25) -- no new search range
invented:

    g_dis = 2*pi*5.0 rad/s (~31.4159)   "previous" setting
    g_dis = 10.0 rad/s                   current waist/shoulder
    g_dis = 5.0 rad/s                    current elbow

Applied UNIFORMLY to all 3 joints per run (via the new
dob_g_dis_override_rad_s ROS param, default -1.0 = no override / shipped
per-joint values unchanged -- see touch_slave.cpp), so each candidate is
directly comparable to the historical hardware table's own single-value
rows, and so sim vs. hardware can be compared per candidate.

100% simulation (touch_slave_sim / hd_sim), confirmed hardware-independent
the same way as every other script in this directory.

Settle-window fix (per user decision, 2026-09-08): the 1.0s discard window
used in Phase 4A is lengthened to 3.0s here, before a 3.0s analysis
window, addressing Phase 4A's own finding that 1.0s does not reliably
cover the arm's settling transient.

N=10 repeats per candidate (matching Phase 4A's own scale, needed to
screen the rare-outlier failure mode Phase 4A characterized).
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import Collector, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import DobDiagnostics  # noqa: E402

JOINT_INDEX = {"waist": 0, "shoulder": 1, "elbow": 2}
CANDIDATES = {
    "g_dis_2pi5": 2.0 * np.pi * 5.0,   # ~31.4159 rad/s, "previous" setting
    "g_dis_10": 10.0,                  # current waist/shoulder
    "g_dis_5": 5.0,                    # current elbow
}
HARDWARE_TABLE_NM = {
    # From touch_slave.cpp's own comment (2026-08-25), p99 |tau_dis_hat|,
    # gravity compensated, measured on real hardware over dyn_Left.csv.
    "g_dis_2pi5": {"waist": 0.0654, "shoulder": 0.0458, "elbow": 0.0373},
    "g_dis_10":   {"waist": 0.0382, "shoulder": 0.0320, "elbow": 0.0305},
    "g_dis_5":    {"waist": 0.0255, "shoulder": 0.0298, "elbow": 0.0250},
}
N_REPEATS = 10
SETTLE_S = 3.0   # lengthened from Phase 4A's 1.0s, per user decision
WINDOW_S = 3.0
OUT_JSON = Path(__file__).parent / "phase4b_gdis_sweep_raw.json"


def one_run(condition: str, g_dis: float, run_idx: int) -> dict:
    ns = unique_ns(f"phase4b_{condition}_r{run_idx}")
    with SimNode(ns, device="Left",
                 params={"dob_g_dis_override_rad_s": str(g_dis)}) as sim:
        assert "DOB bank configured" in sim.log_text(), f"{condition} run {run_idx}: DOB never configured"
        diag_c = Collector(sim.topic("dob_diagnostics"), DobDiagnostics)
        try:
            diag_c.spin_for(SETTLE_S)
            diag_c.messages.clear()
            diag_c.spin_for(WINDOW_S)
            configured = [m for m in diag_c.messages if m.configured]
            record = {"condition": condition, "g_dis": g_dis, "run_idx": run_idx,
                      "n_samples": len(configured), "joints": {}}
            if len(configured) < 50:
                record["error"] = f"only {len(configured)} samples"
                return record
            for name, idx in JOINT_INDEX.items():
                tau = np.abs([m.tau_dis_hat_nm[idx] for m in configured])
                qdot = np.array([m.qdot_rad_s[idx] for m in configured])
                record["joints"][name] = {
                    "tau_mean": float(np.mean(tau)),
                    "tau_rms": float(np.sqrt(np.mean(tau ** 2))),
                    "tau_p95": float(np.percentile(tau, 95)),
                    "tau_p99": float(np.percentile(tau, 99)),
                    "qdot_std": float(np.std(qdot)),
                }
        finally:
            diag_c.close()
    return record


def main() -> None:
    results = []
    for cond, g_dis in CANDIDATES.items():
        for r in range(N_REPEATS):
            t0 = time.time()
            rec = one_run(cond, g_dis, r)
            dur = time.time() - t0
            j = rec.get("joints", {}).get("waist", {})
            print(f"{cond:12s} run {r:2d}: n={rec.get('n_samples','?')} "
                  f"waist_p99={j.get('tau_p99', float('nan')):.5f} "
                  f"qdot_std={j.get('qdot_std', float('nan')):.4f} dur={dur:.2f}s")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
