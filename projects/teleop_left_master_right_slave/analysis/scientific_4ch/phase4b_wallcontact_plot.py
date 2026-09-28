#!/usr/bin/env python3
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).parent
OUT = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
d = json.load(open(HERE / "phase4b_wallcontact_raw.json"))
CONDS = ["g_dis_2pi5", "g_dis_10", "g_dis_5"]
LABELS = ["2*pi*5\n(~31.4 rad/s, prev)", "10 rad/s\n(current w/s)", "5 rad/s\n(current elbow)"]

fig, ax = plt.subplots(figsize=(8, 5))
data = [[r["gain"] for r in d if r["condition"] == c] for c in CONDS]
bp = ax.boxplot(data, labels=LABELS, showmeans=True)
for i, vals in enumerate(data):
    xs = np.random.default_rng(0).normal(i + 1, 0.04, size=len(vals))
    ax.scatter(xs, vals, alpha=0.6, s=25, zorder=3)
ax.axhline(1.0, color="gray", ls=":", lw=1, label="ideal (gain=1.0)")
ax.set_ylabel("steady-state gain = mean|tau_dis_hat| / mean|tau_ext_true| (wall contact tail)")
ax.set_title("Phase 4B (wall-contact redesign): DOB contact-tracking gain vs g_dis, N=10/candidate\n"
             "SIMULATION ground truth (hd_sim wall contact) -- ESTIMATED tau_dis_hat vs SIM GROUND TRUTH tau_ext_true")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "phase4b_wallcontact_gain_vs_gdis.png", dpi=130)
print("wrote", OUT / "phase4b_wallcontact_gain_vs_gdis.png")
