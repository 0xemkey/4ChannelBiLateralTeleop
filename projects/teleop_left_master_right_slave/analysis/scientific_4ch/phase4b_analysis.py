#!/usr/bin/env python3
"""Phase 4B analysis: aggregate phase4b_gdis_sweep_raw.json, compare
sim distributions per g_dis candidate against the historical hardware
table, and render the comparison plot. Offline only -- no simulation
execution here, pure post-processing."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).parent
OUT = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
OUT.mkdir(parents=True, exist_ok=True)

d = json.load(open(HERE / "phase4b_gdis_sweep_raw.json"))
JOINTS = ["waist", "shoulder", "elbow"]
CONDS = ["g_dis_2pi5", "g_dis_10", "g_dis_5"]
HARDWARE_TABLE_NM = {
    "g_dis_2pi5": {"waist": 0.0654, "shoulder": 0.0458, "elbow": 0.0373},
    "g_dis_10":   {"waist": 0.0382, "shoulder": 0.0320, "elbow": 0.0305},
    "g_dis_5":    {"waist": 0.0255, "shoulder": 0.0298, "elbow": 0.0250},
}

by_cond = {c: [r for r in d if r["condition"] == c] for c in CONDS}

print("=" * 70)
print("Phase 4B: sim free-space p99 floor per g_dis candidate, N=10 each")
print("(outliers flagged, not excluded from the printed stats -- median")
print(" and IQR are the robust readout; mean/std are shown for completeness)")
print("=" * 70)

summary = {}
for c in CONDS:
    summary[c] = {}
    for j in JOINTS:
        vals = np.array([r["joints"][j]["tau_p99"] for r in by_cond[c] if "joints" in r and j in r["joints"]])
        qdot_std = np.array([r["joints"][j]["qdot_std"] for r in by_cond[c] if "joints" in r and j in r["joints"]])
        med = float(np.median(vals))
        iqr = float(np.percentile(vals, 75) - np.percentile(vals, 25))
        hw = HARDWARE_TABLE_NM[c][j]
        n_outlier_like = int(np.sum(qdot_std < 0.05))  # heuristic: "settled/stopped" flag from Phase 4A
        summary[c][j] = dict(mean=float(vals.mean()), median=med, std=float(vals.std()),
                              iqr=iqr, min=float(vals.min()), max=float(vals.max()),
                              n=len(vals), n_outlier_like=n_outlier_like, hardware_nm=hw,
                              sim_median_over_hw=med / hw)
        print(f"{c:12s} {j:9s}: median={med:.5f} IQR={iqr:.5f} mean={vals.mean():.5f} "
              f"min={vals.min():.5f} max={vals.max():.5f} n_settled_flag={n_outlier_like}/{len(vals)} "
              f"hw={hw:.4f} sim/hw={med/hw:.2f}")

with open(HERE / "phase4b_summary.json", "w") as f:
    json.dump(summary, f, indent=2)

# ---- Plot: sim (median+IQR) vs hardware table, per joint, across g_dis candidates ----
fig, axes = plt.subplots(1, 3, figsize=(15, 5))
x = np.arange(len(CONDS))
for i, j in enumerate(JOINTS):
    ax = axes[i]
    sim_med = [summary[c][j]["median"] for c in CONDS]
    sim_lo = [summary[c][j]["median"] - summary[c][j]["iqr"] / 2 for c in CONDS]
    sim_hi = [summary[c][j]["median"] + summary[c][j]["iqr"] / 2 for c in CONDS]
    hw = [HARDWARE_TABLE_NM[c][j] for c in CONDS]
    ax.errorbar(x - 0.05, sim_med, yerr=[np.array(sim_med) - np.array(sim_lo),
                                          np.array(sim_hi) - np.array(sim_med)],
                fmt="o-", label="SIM median +/- IQR/2 (N=10)", color="tab:blue", capsize=4)
    ax.plot(x + 0.05, hw, "s--", label="HARDWARE (real device, prior session)", color="tab:red")
    ax.set_xticks(x)
    ax.set_xticklabels(["2pi*5\n(prev)", "10 rad/s\n(waist/sh)", "5 rad/s\n(elbow)"], fontsize=8)
    ax.set_ylabel("p99 |tau_dis_hat| (N*m)")
    ax.set_title(j)
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
fig.suptitle("Phase 4B: SIM (N=10 repeats/candidate) vs HARDWARE free-space floor, per g_dis candidate\n"
             "SIM and HARDWARE are different physical systems (sim intentionally mismatched from identified "
             "params) -- NOT expected to match numerically, only to show the SAME monotonic trend")
fig.tight_layout()
fig.savefig(OUT / "phase4b_sim_vs_hardware_gdis.png", dpi=130)
plt.close(fig)
print("\nwrote", OUT / "phase4b_sim_vs_hardware_gdis.png")
