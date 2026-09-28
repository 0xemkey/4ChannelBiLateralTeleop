#!/usr/bin/env python3
"""Phase 5 analysis: metrics + plots from phase5_position_raw.json.
Offline post-processing only, no simulation execution here."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).parent
OUT = Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/docs/scientific_4ch_plots")
OUT.mkdir(parents=True, exist_ok=True)
d = json.load(open(HERE / "phase5_position_raw.json"))
CONDS = ["baseline_asymmetric", "symmetric_soft", "symmetric_stiff"]
PUSH_S = 4.0


def metrics_for_run(rec):
    s = rec["series"]
    t = np.array(s["t"])
    m = np.array(s["master"])  # Nx3
    sl = np.array(s["slave"])
    mx, sx = m[:, 0], sl[:, 0]
    my, sy = m[:, 1], sl[:, 1]
    mz, sz = m[:, 2], sl[:, 2]

    push_mask = t < PUSH_S
    tail_mask = (t > PUSH_S - 0.5) & (t < PUSH_S)  # last 0.5s of push, "final" value
    m_final = mx[tail_mask].mean()
    s_final = sx[tail_mask].mean()
    m_init = mx[t < 0.1].mean()
    s_init = sx[t < 0.1].mean()

    e_x = sx - mx  # tracking error, slave - master (X, the pushed axis)
    e_push = e_x[push_mask]
    rmse = float(np.sqrt(np.mean(e_push ** 2)))
    mae = float(np.mean(np.abs(e_push)))
    signed_bias = float(np.mean(e_push))
    peak_err = float(np.max(np.abs(e_push)))

    # velocity (finite difference)
    dt = np.gradient(t)
    mv = np.gradient(mx, t)
    sv = np.gradient(sx, t)
    e_v = sv - mv
    e_v_push = e_v[push_mask]
    rmse_v = float(np.sqrt(np.mean(e_v_push ** 2)))

    # rise time (90%) for master and slave, relative to their own init/final
    def rise_time_90(sig, init, final, tvec):
        target = init + 0.9 * (final - init)
        crossed = np.where((sig - init) * np.sign(final - init) >= (target - init) * np.sign(final - init))[0]
        return float(tvec[crossed[0]]) if len(crossed) else float("nan")

    m_rise = rise_time_90(mx, m_init, m_final, t)
    s_rise = rise_time_90(sx, s_init, s_final, t)
    lag = s_rise - m_rise if not (np.isnan(m_rise) or np.isnan(s_rise)) else float("nan")

    # overshoot: does slave exceed master's final value beyond it, before settling
    push_after_rise = (t > (s_rise if not np.isnan(s_rise) else 0)) & push_mask
    if push_after_rise.any():
        peak_s = sx[push_after_rise].max() if (s_final - s_init) > 0 else sx[push_after_rise].min()
        overshoot_pct = float(100.0 * abs(peak_s - s_final) / abs(s_final - s_init)) if abs(s_final - s_init) > 1e-6 else 0.0
    else:
        overshoot_pct = float("nan")

    # settling time: first t after which |e_x| stays < 2mm for the rest of the push
    eps = 0.002
    settle_t = float("nan")
    for i in range(len(t)):
        if push_mask[i] and np.all(np.abs(e_x[i:np.sum(push_mask)]) < eps):
            settle_t = float(t[i])
            break

    reach_ratio = float((s_final - s_init) / (m_final - m_init)) if abs(m_final - m_init) > 1e-6 else float("nan")

    # cross-axis coupling: Y/Z slave motion relative to X final displacement
    sy_range = float(sy[push_mask].max() - sy[push_mask].min())
    sz_range = float(sz[push_mask].max() - sz[push_mask].min())
    x_disp = abs(s_final - s_init)
    cross_yz = float((sy_range + sz_range) / x_disp) if x_disp > 1e-6 else float("nan")

    return dict(rmse_m=rmse, mae_m=mae, signed_bias_m=signed_bias, peak_err_m=peak_err,
                rmse_v_mps=rmse_v, lag_s=lag, overshoot_pct=overshoot_pct,
                settling_time_s=settle_t, reach_ratio=reach_ratio, cross_axis_yz_ratio=cross_yz,
                m_rise_s=m_rise, s_rise_s=s_rise)


results = {}
for c in CONDS:
    runs = [r for r in d if r["condition"] == c and "series" in r]
    per_run = [metrics_for_run(r) for r in runs]
    agg = {}
    for key in per_run[0]:
        vals = np.array([p[key] for p in per_run if not np.isnan(p[key])])
        if len(vals) == 0:
            agg[key] = dict(mean=float("nan"), std=float("nan"), n=0)
        else:
            agg[key] = dict(mean=float(vals.mean()), std=float(vals.std()), n=len(vals))
    results[c] = agg
    print(f"\n=== {c} (N={len(runs)}) ===")
    for key, v in agg.items():
        print(f"  {key:22s} mean={v['mean']:.5f} std={v['std']:.5f} n={v['n']}")

with open(HERE / "phase5_summary.json", "w") as f:
    json.dump(results, f, indent=2)

# ---- Pareto-style comparison plot: RMSE vs cross-axis coupling, RMSE vs lag ----
fig, axes = plt.subplots(1, 3, figsize=(16, 5))
colors = {"baseline_asymmetric": "tab:blue", "symmetric_soft": "tab:orange", "symmetric_stiff": "tab:green"}
for c in CONDS:
    m = results[c]
    axes[0].errorbar(m["rmse_m"]["mean"] * 1000, m["lag_s"]["mean"],
                      xerr=m["rmse_m"]["std"] * 1000, yerr=m["lag_s"]["std"],
                      fmt="o", color=colors[c], label=c, markersize=10, capsize=4)
    axes[1].errorbar(m["rmse_m"]["mean"] * 1000, m["overshoot_pct"]["mean"],
                      xerr=m["rmse_m"]["std"] * 1000, yerr=m["overshoot_pct"]["std"],
                      fmt="o", color=colors[c], label=c, markersize=10, capsize=4)
    axes[2].errorbar(m["rmse_m"]["mean"] * 1000, m["cross_axis_yz_ratio"]["mean"],
                      xerr=m["rmse_m"]["std"] * 1000, yerr=m["cross_axis_yz_ratio"]["std"],
                      fmt="o", color=colors[c], label=c, markersize=10, capsize=4)
axes[0].set_xlabel("position RMSE, e_x (mm)"); axes[0].set_ylabel("lag, s (slave rise - master rise)")
axes[1].set_xlabel("position RMSE, e_x (mm)"); axes[1].set_ylabel("overshoot (%)")
axes[2].set_xlabel("position RMSE, e_x (mm)"); axes[2].set_ylabel("cross-axis Y+Z / X-disp ratio")
for ax in axes:
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
fig.suptitle("Phase 5: C2/C3 position-subsystem Pareto comparison (N=5/config, error bars = std across repeats)\n"
             "No scalar collapse -- each axis pair shown separately, per task's own rule")
fig.tight_layout()
fig.savefig(OUT / "phase5_pareto_comparison.png", dpi=130)
print("\nwrote", OUT / "phase5_pareto_comparison.png")

# ---- Step-response time series, one representative run per condition ----
fig, ax = plt.subplots(figsize=(10, 6))
for c in CONDS:
    r = next(r for r in d if r["condition"] == c and "series" in r)
    s = r["series"]
    t = np.array(s["t"]); mx = np.array(s["master"])[:, 0]; sx = np.array(s["slave"])[:, 0]
    ax.plot(t, mx, color=colors[c], ls="--", alpha=0.6, label=f"{c} master")
    ax.plot(t, sx, color=colors[c], ls="-", label=f"{c} slave")
ax.axvline(PUSH_S, color="k", lw=0.5, ls=":")
ax.set_xlabel("time (s)"); ax.set_ylabel("X position (m)")
ax.set_title("Phase 5: representative step response per configuration (dashed=master, solid=slave)")
ax.legend(fontsize=7, ncol=2)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT / "phase5_step_response.png", dpi=130)
print("wrote", OUT / "phase5_step_response.png")
