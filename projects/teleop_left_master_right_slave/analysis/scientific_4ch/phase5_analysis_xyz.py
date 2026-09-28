#!/usr/bin/env python3
"""Phase 5 X/Y/Z analysis: metrics + cross-axis coupling + Pareto plots
across all three driven axes and three configurations. Offline
post-processing only, no simulation execution here.

Per user instruction (2026-09-09): settling time is NOT computed with an
invented epsilon -- reported as INSUFFICIENT_REQUIREMENT / not defined.
"""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).parent
OUT = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
OUT.mkdir(parents=True, exist_ok=True)
CONDS = ["baseline_asymmetric", "symmetric_soft", "symmetric_stiff"]
AXES = ["x", "y", "z"]
AXIS_IDX = {"x": 0, "y": 1, "z": 2}
PUSH_S = 4.0


def metrics_for_run(rec, driven_axis):
    s = rec["series"]
    t = np.array(s["t"])
    m = np.array(s["master"])
    sl = np.array(s["slave"])
    di = AXIS_IDX[driven_axis]
    md, sd = m[:, di], sl[:, di]  # driven-axis components

    push_mask = t < PUSH_S
    tail_mask = (t > PUSH_S - 0.5) & (t < PUSH_S)
    m_final = md[tail_mask].mean()
    s_final = sd[tail_mask].mean()
    m_init = md[t < 0.1].mean()
    s_init = sd[t < 0.1].mean()

    e = sd - md
    e_push = e[push_mask]
    rmse = float(np.sqrt(np.mean(e_push ** 2)))
    mae = float(np.mean(np.abs(e_push)))
    signed_bias = float(np.mean(e_push))
    peak_err = float(np.max(np.abs(e_push)))

    mv = np.gradient(md, t)
    sv = np.gradient(sd, t)
    e_v_push = (sv - mv)[push_mask]
    rmse_v = float(np.sqrt(np.mean(e_v_push ** 2)))

    def rise_time_90(sig, init, final, tvec):
        if abs(final - init) < 1e-6:
            return float("nan")
        target = init + 0.9 * (final - init)
        crossed = np.where((sig - init) * np.sign(final - init) >= (target - init) * np.sign(final - init))[0]
        return float(tvec[crossed[0]]) if len(crossed) else float("nan")

    m_rise = rise_time_90(md, m_init, m_final, t)
    s_rise = rise_time_90(sd, s_init, s_final, t)
    lag = s_rise - m_rise if not (np.isnan(m_rise) or np.isnan(s_rise)) else float("nan")

    push_after_rise = (t > (s_rise if not np.isnan(s_rise) else 0)) & push_mask
    if push_after_rise.any() and abs(s_final - s_init) > 1e-6:
        peak_s = sd[push_after_rise].max() if (s_final - s_init) > 0 else sd[push_after_rise].min()
        overshoot_pct = float(100.0 * abs(peak_s - s_final) / abs(s_final - s_init))
    else:
        overshoot_pct = float("nan")

    reach_ratio = float((s_final - s_init) / (m_final - m_init)) if abs(m_final - m_init) > 1e-6 else float("nan")

    # cross-axis coupling: slave motion on the OTHER two axes, relative to
    # the driven-axis displacement -- computed for all 3 driven directions.
    other_axes = [a for a in AXES if a != driven_axis]
    disp = abs(s_final - s_init)
    cross = {}
    for oa in other_axes:
        oi = AXIS_IDX[oa]
        so = sl[:, oi]
        rng = float(so[push_mask].max() - so[push_mask].min())
        cross[oa] = float(rng / disp) if disp > 1e-6 else float("nan")

    return dict(rmse_m=rmse, mae_m=mae, signed_bias_m=signed_bias, peak_err_m=peak_err,
                rmse_v_mps=rmse_v, lag_s=lag, overshoot_pct=overshoot_pct,
                reach_ratio=reach_ratio, m_rise_s=m_rise, s_rise_s=s_rise,
                settling_time_s="INSUFFICIENT_REQUIREMENT",
                **{f"cross_{oa}": v for oa, v in cross.items()})


all_results = {}
for axis in AXES:
    path = HERE / f"phase5_position_raw_{axis}.json"
    d = json.load(open(path))
    all_results[axis] = {}
    for c in CONDS:
        runs = [r for r in d if r["condition"] == c and "series" in r]
        per_run = [metrics_for_run(r, axis) for r in runs]
        agg = {}
        for key in per_run[0]:
            if key == "settling_time_s":
                agg[key] = "INSUFFICIENT_REQUIREMENT"
                continue
            vals = np.array([p[key] for p in per_run if isinstance(p[key], float) and not np.isnan(p[key])])
            agg[key] = dict(mean=float(vals.mean()), std=float(vals.std()), n=len(vals)) if len(vals) else dict(mean=float("nan"), std=float("nan"), n=0)
        all_results[axis][c] = agg
        print(f"\n=== axis={axis} config={c} (N={len(runs)}) ===")
        for key, v in agg.items():
            if key == "settling_time_s":
                print(f"  {key:16s} {v}")
            else:
                print(f"  {key:16s} mean={v['mean']:.5f} std={v['std']:.5f} n={v['n']}")

with open(HERE / "phase5_summary_xyz.json", "w") as f:
    json.dump(all_results, f, indent=2)

# ---- Combined Pareto plot: RMSE vs lag, all 3 axes x 3 configs ----
colors = {"baseline_asymmetric": "tab:blue", "symmetric_soft": "tab:orange", "symmetric_stiff": "tab:green"}
markers = {"x": "o", "y": "s", "z": "^"}
fig, axes_plt = plt.subplots(1, 3, figsize=(17, 5.5))
metric_pairs = [("rmse_m", "lag_s", "lag (s)"), ("rmse_m", "overshoot_pct", "overshoot (%)"),
                ("rmse_m", "reach_ratio", "reach ratio")]
for ax_i, (mx, my, ylabel) in enumerate(metric_pairs):
    ax = axes_plt[ax_i]
    for axis in AXES:
        for c in CONDS:
            v = all_results[axis][c]
            ax.errorbar(v[mx]["mean"] * 1000 if mx == "rmse_m" else v[mx]["mean"],
                        v[my]["mean"], xerr=v[mx]["std"] * 1000 if mx == "rmse_m" else v[mx]["std"],
                        yerr=v[my]["std"], fmt=markers[axis], color=colors[c], markersize=9, capsize=3,
                        label=f"{c} [{axis}]" if ax_i == 0 else None)
    ax.set_xlabel("position RMSE, e_driven (mm)")
    ax.set_ylabel(ylabel)
    ax.grid(alpha=0.3)
axes_plt[0].legend(fontsize=6, ncol=1, loc="upper left", bbox_to_anchor=(1.02, 1))
fig.suptitle("Phase 5 X/Y/Z: Pareto comparison across all driven axes and configs "
             "(marker shape = driven axis, color = config)")
fig.tight_layout()
fig.savefig(OUT / "phase5_pareto_xyz.png", dpi=130, bbox_inches="tight")
print("\nwrote", OUT / "phase5_pareto_xyz.png")

# ---- Cross-axis coupling summary plot ----
fig, ax = plt.subplots(figsize=(10, 6))
bar_w = 0.1
x_pos = np.arange(len(AXES))
offset = -bar_w * 4
for c in CONDS:
    for oa_slot, oa_label in enumerate(["cross_1", "cross_2"]):
        pass
# simpler: print a table-style bar chart, one group per driven axis, bars = config x other-axis
labels = []
values = []
errs = []
for axis in AXES:
    other = [a for a in AXES if a != axis]
    for c in CONDS:
        for oa in other:
            key = f"cross_{oa}"
            v = all_results[axis][c][key]
            labels.append(f"{axis}->{oa}\n{c[:4]}")
            values.append(v["mean"])
            errs.append(v["std"])
xpos = np.arange(len(labels))
colors_bar = []
for axis in AXES:
    other = [a for a in AXES if a != axis]
    for c in CONDS:
        for oa in other:
            colors_bar.append(colors[c])
fig, ax = plt.subplots(figsize=(16, 6))
ax.bar(xpos, values, yerr=errs, color=colors_bar, capsize=3)
ax.set_xticks(xpos)
ax.set_xticklabels(labels, fontsize=6, rotation=45, ha="right")
ax.set_ylabel("cross-axis coupling ratio (other-axis range / driven-axis displacement)")
ax.set_title("Phase 5 X/Y/Z: cross-axis coupling for all driven directions and configs")
ax.grid(alpha=0.3, axis="y")
fig.tight_layout()
fig.savefig(OUT / "phase5_cross_axis_xyz.png", dpi=130)
print("wrote", OUT / "phase5_cross_axis_xyz.png")
