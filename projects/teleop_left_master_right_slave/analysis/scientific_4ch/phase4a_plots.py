#!/usr/bin/env python3
"""Phase 4A.3 required plots, from phase4a_repeatability_raw.json.
Offline plotting only -- no simulation/hardware execution."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).parent
OUT = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
OUT.mkdir(parents=True, exist_ok=True)
d = json.load(open(HERE / "phase4a_repeatability_raw.json"))
JOINTS = ["waist", "shoulder", "elbow"]
COLORS = {"waist": "tab:blue", "shoulder": "tab:orange", "elbow": "tab:green"}

runs = [r["run_idx"] for r in d]
p99 = {j: np.array([r["joints"][j]["tau_p99"] for r in d]) for j in JOINTS}
mean_ = {j: np.array([r["joints"][j]["tau_mean"] for r in d]) for j in JOINTS}
q_first = {j: np.array([r["joints"][j]["q_first"] for r in d]) for j in JOINTS}

# ---- Plot 1: run-by-run DOB floor (p99) ----
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
for i, j in enumerate(JOINTS):
    ax = axes[i]
    ax.plot(runs, p99[j], "o-", color=COLORS[j])
    med = np.median(p99[j])
    ax.axhline(med, color="k", ls="--", lw=0.8, label=f"median={med:.5f}")
    ax.set_ylabel(f"{j}\np99 |tau_dis_hat| (N*m)")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)
axes[-1].set_xlabel("run index (identical conditions)")
fig.suptitle("Phase 4A.3.1: run-by-run free-space DOB floor (SIMULATION, N=20 identical-condition repeats)")
fig.tight_layout()
fig.savefig(OUT / "phase4a_1_run_by_run_floor.png", dpi=130)
plt.close(fig)

# ---- Plot 2: per-joint distribution (box + strip) ----
fig, ax = plt.subplots(figsize=(8, 5))
data = [p99[j] for j in JOINTS]
bp = ax.boxplot(data, labels=JOINTS, showmeans=True)
for i, j in enumerate(JOINTS):
    xs = np.random.default_rng(0).normal(i + 1, 0.04, size=len(p99[j]))
    ax.scatter(xs, p99[j], alpha=0.6, s=20, color=COLORS[j], zorder=3)
ax.set_ylabel("p99 |tau_dis_hat| (N*m)")
ax.set_title("Phase 4A.3.2: per-joint distribution across N=20 identical-condition runs (SIMULATION)")
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "phase4a_2_per_joint_distribution.png", dpi=130)
plt.close(fig)

# ---- Plot 3: initial-pose (q at window start) vs floor scatter ----
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
for i, j in enumerate(JOINTS):
    ax = axes[i]
    ax.scatter(q_first[j], p99[j], color=COLORS[j])
    ax.set_xlabel(f"{j} q at analysis-window start (rad)")
    ax.set_ylabel("p99 |tau_dis_hat| (N*m)")
    ax.set_title(j)
    ax.grid(alpha=0.3)
fig.suptitle("Phase 4A.3.3: initial pose (at window start) vs floor -- SIMULATION, deterministic initial q={0,0.45,0.90},\n"
             "spread here is from HOW FAR the settling swing had progressed by t=1.0s (wall-clock), not RNG")
fig.tight_layout()
fig.savefig(OUT / "phase4a_3_initial_pose_vs_floor.png", dpi=130)
plt.close(fig)

# ---- Plot 4: floor vs run-order (state leakage check) ----
fig, ax = plt.subplots(figsize=(9, 5))
for j in JOINTS:
    ax.plot(runs, p99[j] / np.median(p99[j]), "o-", color=COLORS[j], label=j, alpha=0.8)
ax.axhline(1.0, color="k", lw=0.5)
ax.set_xlabel("run index")
ax.set_ylabel("p99 / median(p99) -- normalized")
ax.set_title("Phase 4A.3.4: floor vs run-order, normalized (checks for drift/state-leakage across the sequence)")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "phase4a_4_floor_vs_run_order.png", dpi=130)
plt.close(fig)

# ---- Plot 5: N/A -- no seed exists (confirmed no RNG in sim source), so
# "floor vs seed" is replaced with "floor vs actual analysis-window
# duration" -- the only other run-varying quantity recorded, explicitly
# labeled as a substitute, not silently omitted. ----
window_actual = np.array([r["analysis_window_actual_s"] for r in d])
fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
for i, j in enumerate(JOINTS):
    ax = axes[i]
    ax.scatter(window_actual, p99[j], color=COLORS[j])
    ax.set_xlabel("actual analysis-window duration (s)")
    ax.set_ylabel("p99 |tau_dis_hat| (N*m)")
    ax.set_title(j)
    ax.grid(alpha=0.3)
fig.suptitle("Phase 4A.3.5 (substitute for floor-vs-seed: NO SEED EXISTS, confirmed no RNG in "
             "arm_model.hpp/hd_sim_api.cpp -- this checks window-duration jitter instead)")
fig.tight_layout()
fig.savefig(OUT / "phase4a_5_floor_vs_window_duration.png", dpi=130)
plt.close(fig)

print("wrote 5 plots to", OUT)
