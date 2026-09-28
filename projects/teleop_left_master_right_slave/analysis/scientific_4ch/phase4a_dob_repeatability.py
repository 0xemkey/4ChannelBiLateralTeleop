#!/usr/bin/env python3
"""Phase 4A.2: N identical-condition repeats of the free-space DOB floor
capture (same recipe as sim/tests/test_dob_floor.py), to determine whether
a single-run DOB characterization is scientifically repeatable.

100% simulation: single touch_slave_sim instance ("Left" device), no wall,
no hand, no bilateral_4ch.py. Never touches physical hardware -- confirmed
hardware-independent the same way as Phase 1/2's sign test (touch_slave_sim
links libhd_sim.so, not libHD.so).

Records, per run and per joint: mean/std/RMS/p95/p99 of |tau_dis_hat|,
mean/std of q and qdot, sample count, actual capture duration, and enough
run-to-run metadata (wall-clock start time, PID) to check for state leakage
or timing drift across the run sequence.
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
OUT_DIR = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def one_run(run_idx: int) -> dict:
    ns = unique_ns(f"phase4a_r{run_idx}")
    t_start = time.time()
    with SimNode(ns, device="Left") as sim:
        assert "DOB bank configured" in sim.log_text(), f"run {run_idx}: DOB never configured"
        diag_c = Collector(sim.topic("dob_diagnostics"), DobDiagnostics)
        try:
            # IDENTICAL recipe to test_dob_floor.py: 1.0s settle (discarded),
            # then 3.0s analysis window.
            diag_c.spin_for(1.0)
            n_before_clear = len(diag_c.messages)
            diag_c.messages.clear()
            t_window_start = time.time()
            diag_c.spin_for(3.0)
            t_window_end = time.time()

            configured = [m for m in diag_c.messages if m.configured]
            record = {
                "run_idx": run_idx,
                "namespace": ns,
                "wall_clock_start": t_start,
                "settle_phase_sample_count": n_before_clear,
                "analysis_window_requested_s": 3.0,
                "analysis_window_actual_s": t_window_end - t_window_start,
                "n_samples": len(configured),
                "joints": {},
            }
            if len(configured) < 50:
                record["error"] = f"only {len(configured)} configured samples"
                return record

            for name, idx in JOINT_INDEX.items():
                tau = np.abs([m.tau_dis_hat_nm[idx] for m in configured])
                q = np.array([m.q_rad[idx] for m in configured])
                qdot = np.array([m.qdot_rad_s[idx] for m in configured])
                record["joints"][name] = {
                    "tau_mean": float(np.mean(tau)),
                    "tau_std": float(np.std(tau)),
                    "tau_rms": float(np.sqrt(np.mean(tau ** 2))),
                    "tau_p95": float(np.percentile(tau, 95)),
                    "tau_p99": float(np.percentile(tau, 99)),
                    "tau_max": float(np.max(tau)),
                    "q_mean": float(np.mean(q)),
                    "q_std": float(np.std(q)),
                    "q_first": float(q[0]),
                    "qdot_mean": float(np.mean(qdot)),
                    "qdot_std": float(np.std(qdot)),
                }
        finally:
            diag_c.close()
    return record


def main(n_runs: int) -> None:
    results = []
    for i in range(n_runs):
        t0 = time.time()
        rec = one_run(i)
        rec["run_wall_duration_s"] = time.time() - t0
        results.append(rec)
        j = rec.get("joints", {}).get("waist", {})
        print(f"run {i:2d}: n={rec.get('n_samples','?')} "
              f"waist_p99={j.get('tau_p99', float('nan')):.5f} "
              f"waist_mean={j.get('tau_mean', float('nan')):.5f} "
              f"dur={rec['run_wall_duration_s']:.2f}s "
              f"window_actual={rec.get('analysis_window_actual_s', float('nan')):.3f}s")

    out_path = OUT_DIR.parent / "root_cause_plots"  # unused, keep json beside script
    out_json = Path(__file__).parent / "phase4a_repeatability_raw.json"
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {out_json}")


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    main(n)
