#!/usr/bin/env python3
"""2026-09-10: focused per-axis re-check for a full4ch baseline session,
originally requested to re-verify whether historically-observed abnormal
Y-axis behavior (previously associated with masking) is still present now
that the baseline is confirmed unmasked (see CURRENT_FULL4CH_AUDIT.md),
later extended to X and Z on request for the same comparison.

Reads raw_master.csv / raw_slave.csv DIRECTLY (via analyze_manual_
motion.py's own _read_csv/_nearest/merge_streams -- the same anchor-
relative convention already established there: master_{x,y,z} =
mapped_rel_{x,y,z}, slave_{x,y,z} = rel_{x,y,z}). Does NOT read
metrics.json for any plotted series or reported metric -- every number
below is recomputed from the raw rows in this script. Publishes/moves
nothing; pure post-hoc analysis.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path

HV_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(HV_DIR))
from analyze_manual_motion import _read_csv, merge_streams, _nearest, JOIN_TOLERANCE_S  # noqa: E402

# 2026-09-10: lowered on request ("make more transparent"). Position-error
# clamp is the far more frequent event (dozens of intervals covering many
# seconds) so it gets the lighter value to avoid washing out the trace;
# final-force clamp is rarer/safety-relevant so it stays a bit more visible,
# but still lighter than the original 0.25 both used.
POS_ERR_CLAMP_ALPHA = 0.10
FINAL_FORCE_CLAMP_ALPHA = 0.18

AXES = ("x", "y", "z")


def _num(v):
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _bool(v):
    if v is None or v == "":
        return None
    return str(v).strip().lower() in ("true", "1")


def _pearson(xs, ys):
    if len(xs) < 3:
        return None
    if statistics.pstdev(xs) < 1e-12 or statistics.pstdev(ys) < 1e-12:
        return None
    mx, my = statistics.fmean(xs), statistics.fmean(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx < 1e-12 or dy < 1e-12:
        return None
    return num / (dx * dy)


def _cross_corr_lag(a, b, ts, max_lag_s=1.0):
    """Best-fit lag (seconds) that maximizes correlation between a and b
    when b is shifted relative to a, via a direct time-domain search over
    the actual (non-uniform) timestamps -- no resampling/interpolation of
    the underlying signal values, only a search over shift amounts."""
    n = len(a)
    if n < 10:
        return None
    dt_est = (ts[-1] - ts[0]) / (n - 1) if n > 1 else 0.01
    best_lag, best_corr = 0.0, -2.0
    steps = max(1, int(max_lag_s / max(dt_est, 1e-6)))
    for k in range(-steps, steps + 1):
        if k >= 0:
            xa, xb = a[k:], b[: n - k] if k > 0 else b
        else:
            xa, xb = a[: n + k], b[-k:]
        if len(xa) < 10:
            continue
        c = _pearson(xa, xb)
        if c is not None and c > best_corr:
            best_corr = c
            best_lag = k * dt_est
    return best_lag if best_corr > -2.0 else None


def _intervals(flags, times):
    out = []
    i = 0
    n_ = len(flags)
    while i < n_:
        if flags[i]:
            j = i
            while j < n_ and flags[j]:
                j += 1
            out.append((times[i], times[j - 1]))
            i = j
        else:
            i += 1
    return out


def _shade(ax, intervals, color, label, alpha):
    first = True
    for a, b in intervals:
        ax.axvspan(a, max(b, a + 0.001), color=color, alpha=alpha,
                  label=(label if first else None))
        first = False


def _run_axis(axis: str, merged: list, ts: list, analysis_dir: Path,
             master_pec: list, slave_pec: list, master_ffc: list, slave_ffc: list,
             master_pec_intervals, slave_pec_intervals, master_ffc_intervals, slave_ffc_intervals) -> dict:
    import matplotlib.pyplot as plt

    mkey, skey = f"master_{axis}", f"slave_{axis}"
    m_series = [r[mkey] for r in merged]
    s_series = [r[skey] for r in merged]

    valid_idx = [i for i in range(len(merged)) if m_series[i] is not None and s_series[i] is not None]
    if not valid_idx:
        print(f"[axis-recheck][{axis}] SKIPPED: no timestamp had both master and slave {axis} "
             f"available within the join tolerance")
        return {"axis": axis, "result": "NOT_EVALUABLE"}

    m_v = [m_series[i] for i in valid_idx]
    s_v = [s_series[i] for i in valid_idx]
    err_v = [m - s for m, s in zip(m_v, s_v)]
    ts_v = [ts[i] for i in valid_idx]

    rmse = math.sqrt(sum(e * e for e in err_v) / len(err_v))
    peak_abs_error = max(abs(e) for e in err_v)
    correlated_gain = _pearson(m_v, s_v)
    lag_s = _cross_corr_lag(m_v, s_v, ts_v, max_lag_s=1.0)
    master_ptp = max(m_v) - min(m_v)
    slave_ptp = max(s_v) - min(s_v)

    print(f"[axis-recheck][{axis}] rows valid-paired={len(valid_idx)}/{len(merged)}")
    print(f"[axis-recheck][{axis}] RMSE = {rmse:.5f} m")
    print(f"[axis-recheck][{axis}] peak |error| = {peak_abs_error:.5f} m")
    print(f"[axis-recheck][{axis}] correlated gain (Pearson r) = {correlated_gain}")
    print(f"[axis-recheck][{axis}] estimated lag (+/-1.0s search) = {lag_s} s")
    print(f"[axis-recheck][{axis}] master peak-to-peak = {master_ptp:.5f} m")
    print(f"[axis-recheck][{axis}] slave peak-to-peak = {slave_ptp:.5f} m")

    m_plot = [v if v is not None else float("nan") for v in m_series]
    s_plot = [v if v is not None else float("nan") for v in s_series]
    err_plot = [(m - s) if (m is not None and s is not None) else float("nan")
               for m, s in zip(m_series, s_series)]
    combined_pec = sorted(set(master_pec_intervals) | set(slave_pec_intervals))
    combined_ffc = sorted(set(master_ffc_intervals) | set(slave_ffc_intervals))

    # 1. Master {axis} position vs time
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(ts, m_plot, color="tab:blue", linewidth=0.9)
    _shade(ax, master_pec_intervals, "orange", "position-error clamp", POS_ERR_CLAMP_ALPHA)
    _shade(ax, master_ffc_intervals, "red", "final-force clamp", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel(f"master {axis.upper()}, anchor-rel (m)")
    ax.set_title(f"Master {axis.upper()} position vs time (mapped_rel_{axis})")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / f"{axis}_recheck_01_master_{axis}_vs_t.png", dpi=150)
    plt.close(fig)

    # 2. Slave {axis} position vs time
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(ts, s_plot, color="tab:green", linewidth=0.9)
    _shade(ax, slave_pec_intervals, "orange", "position-error clamp", POS_ERR_CLAMP_ALPHA)
    _shade(ax, slave_ffc_intervals, "red", "final-force clamp", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel(f"slave {axis.upper()}, anchor-rel (m)")
    ax.set_title(f"Slave {axis.upper()} position vs time (rel_{axis})")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / f"{axis}_recheck_02_slave_{axis}_vs_t.png", dpi=150)
    plt.close(fig)

    # 3. Master and slave {axis} together
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(ts, m_plot, color="tab:blue", linewidth=0.9, label=f"master {axis.upper()} (mapped, anchor-rel)")
    ax.plot(ts, s_plot, color="tab:green", linewidth=0.9, label=f"slave {axis.upper()} (native, anchor-rel)")
    _shade(ax, combined_pec, "orange", "position-error clamp (either arm)", POS_ERR_CLAMP_ALPHA)
    _shade(ax, combined_ffc, "red", "final-force clamp (either arm)", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel(f"{axis.upper()}, anchor-rel (m)")
    ax.set_title(f"Master vs Slave {axis.upper()} position (full4ch baseline, unmasked)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / f"{axis}_recheck_03_master_vs_slave_{axis}.png", dpi=150)
    plt.close(fig)

    # 4. {axis} tracking error vs time
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(ts, err_plot, color="tab:red", linewidth=0.9)
    ax.axhline(0.0, color="gray", linewidth=0.6, linestyle="--")
    _shade(ax, combined_pec, "orange", "position-error clamp (either arm)", POS_ERR_CLAMP_ALPHA)
    _shade(ax, combined_ffc, "red", "final-force clamp (either arm)", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel(f"master_{axis} - slave_{axis} (m)")
    ax.set_title(f"{axis.upper()} tracking error vs time (RMSE={rmse:.4f}m, peak|err|={peak_abs_error:.4f}m)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / f"{axis}_recheck_04_tracking_error_{axis}.png", dpi=150)
    plt.close(fig)

    return {
        "axis": axis,
        "result": "EVALUATED",
        "n_valid_paired_samples": len(valid_idx),
        "rmse_m": rmse,
        "peak_abs_error_m": peak_abs_error,
        "correlated_gain_pearson_r": correlated_gain,
        "estimated_lag_s": lag_s,
        "master_peak_to_peak_m": master_ptp,
        "slave_peak_to_peak_m": slave_ptp,
    }


def _run_ee(merged: list, ts: list, analysis_dir: Path,
           master_pec_intervals, slave_pec_intervals, master_ffc_intervals, slave_ffc_intervals) -> dict:
    """Combined-axis (X+Y+Z together) end-effector position comparison --
    complements the per-axis breakdowns with the actual 3D path each
    end-effector took and a single scalar 'how far apart are they right
    now' signal, rather than three separate per-axis views. Still built
    only from the same raw mapped_rel_{x,y,z}/rel_{x,y,z} fields, nothing
    reconstructed beyond a vector norm/Euclidean distance of already-real
    per-axis values."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3d projection)

    mx = [r["master_x"] for r in merged]; my = [r["master_y"] for r in merged]; mz = [r["master_z"] for r in merged]
    sx = [r["slave_x"] for r in merged]; sy = [r["slave_y"] for r in merged]; sz = [r["slave_z"] for r in merged]

    valid_idx = [i for i in range(len(merged))
                if None not in (mx[i], my[i], mz[i], sx[i], sy[i], sz[i])]
    if not valid_idx:
        print("[axis-recheck][ee] SKIPPED: no timestamp had a complete master+slave XYZ sample")
        return {"result": "NOT_EVALUABLE"}

    m_norm = [math.sqrt(mx[i] ** 2 + my[i] ** 2 + mz[i] ** 2) for i in valid_idx]
    s_norm = [math.sqrt(sx[i] ** 2 + sy[i] ** 2 + sz[i] ** 2) for i in valid_idx]
    dist = [math.sqrt((mx[i] - sx[i]) ** 2 + (my[i] - sy[i]) ** 2 + (mz[i] - sz[i]) ** 2) for i in valid_idx]
    ts_v = [ts[i] for i in valid_idx]

    rmse_dist = math.sqrt(sum(d * d for d in dist) / len(dist))
    peak_dist = max(dist)
    correlated_gain = _pearson(m_norm, s_norm)
    lag_s = _cross_corr_lag(m_norm, s_norm, ts_v, max_lag_s=1.0)
    master_norm_ptp = max(m_norm) - min(m_norm)
    slave_norm_ptp = max(s_norm) - min(s_norm)

    print(f"[axis-recheck][ee] rows valid-paired={len(valid_idx)}/{len(merged)}")
    print(f"[axis-recheck][ee] EE-distance RMSE = {rmse_dist:.5f} m")
    print(f"[axis-recheck][ee] peak EE distance (master-slave) = {peak_dist:.5f} m")
    print(f"[axis-recheck][ee] correlated gain (||master|| vs ||slave||, Pearson r) = {correlated_gain}")
    print(f"[axis-recheck][ee] estimated lag (+/-1.0s search) = {lag_s} s")
    print(f"[axis-recheck][ee] master ||pos|| peak-to-peak = {master_norm_ptp:.5f} m")
    print(f"[axis-recheck][ee] slave ||pos|| peak-to-peak = {slave_norm_ptp:.5f} m")

    combined_pec = sorted(set(master_pec_intervals) | set(slave_pec_intervals))
    combined_ffc = sorted(set(master_ffc_intervals) | set(slave_ffc_intervals))
    ts_all = ts
    dist_plot = [math.sqrt((mx[i] - sx[i]) ** 2 + (my[i] - sy[i]) ** 2 + (mz[i] - sz[i]) ** 2)
                if None not in (mx[i], my[i], mz[i], sx[i], sy[i], sz[i]) else float("nan")
                for i in range(len(merged))]
    m_norm_plot = [math.sqrt(mx[i] ** 2 + my[i] ** 2 + mz[i] ** 2) if None not in (mx[i], my[i], mz[i])
                  else float("nan") for i in range(len(merged))]
    s_norm_plot = [math.sqrt(sx[i] ** 2 + sy[i] ** 2 + sz[i] ** 2) if None not in (sx[i], sy[i], sz[i])
                  else float("nan") for i in range(len(merged))]

    # 1. 3D trajectory -- the actual spatial path each end-effector took,
    # anchor-relative, both on the same axes for direct visual comparison.
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    mx_v = [mx[i] for i in valid_idx]; my_v = [my[i] for i in valid_idx]; mz_v = [mz[i] for i in valid_idx]
    sx_v = [sx[i] for i in valid_idx]; sy_v = [sy[i] for i in valid_idx]; sz_v = [sz[i] for i in valid_idx]
    ax.plot(mx_v, mz_v, my_v, color="tab:blue", linewidth=0.9, label="master EE (mapped, anchor-rel)")
    ax.plot(sx_v, sz_v, sy_v, color="tab:green", linewidth=0.9, label="slave EE (native, anchor-rel)")
    ax.scatter([mx_v[0]], [mz_v[0]], [my_v[0]], color="tab:blue", marker="o", s=40)
    ax.scatter([sx_v[0]], [sz_v[0]], [sy_v[0]], color="tab:green", marker="o", s=40)
    ax.set_xlabel("X (m)"); ax.set_ylabel("Z (m)"); ax.set_zlabel("Y (m, up)")
    ax.set_title("Master vs Slave end-effector 3D path (anchor-rel, unmasked full4ch baseline)")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / "ee_recheck_01_3d_trajectory.png", dpi=150)
    plt.close(fig)

    # 2. Position norm (||pos|| from anchor) vs time, both EEs together.
    fig, ax = plt.subplots(figsize=(11, 4.5))
    ax.plot(ts_all, m_norm_plot, color="tab:blue", linewidth=0.9, label="||master EE pos|| (mapped, anchor-rel)")
    ax.plot(ts_all, s_norm_plot, color="tab:green", linewidth=0.9, label="||slave EE pos|| (native, anchor-rel)")
    _shade(ax, combined_pec, "orange", "position-error clamp (either arm)", POS_ERR_CLAMP_ALPHA)
    _shade(ax, combined_ffc, "red", "final-force clamp (either arm)", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel("||pos|| from anchor (m)")
    ax.set_title("Master vs Slave EE position magnitude vs time")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / "ee_recheck_02_position_norm_vs_t.png", dpi=150)
    plt.close(fig)

    # 3. EE-to-EE distance (Euclidean, all 3 axes combined) vs time.
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(ts_all, dist_plot, color="tab:red", linewidth=0.9)
    _shade(ax, combined_pec, "orange", "position-error clamp (either arm)", POS_ERR_CLAMP_ALPHA)
    _shade(ax, combined_ffc, "red", "final-force clamp (either arm)", FINAL_FORCE_CLAMP_ALPHA)
    ax.set_xlabel("t (s)"); ax.set_ylabel("|master EE - slave EE| (m)")
    ax.set_title(f"EE-to-EE distance vs time (RMSE={rmse_dist:.4f}m, peak={peak_dist:.4f}m)")
    ax.legend(["EE distance", "position-error clamp", "final-force clamp"], loc="upper right", fontsize=8)
    fig.tight_layout(); fig.savefig(analysis_dir / "ee_recheck_03_distance_vs_t.png", dpi=150)
    plt.close(fig)

    return {
        "result": "EVALUATED",
        "n_valid_paired_samples": len(valid_idx),
        "ee_distance_rmse_m": rmse_dist,
        "ee_distance_peak_m": peak_dist,
        "position_norm_correlated_gain_pearson_r": correlated_gain,
        "position_norm_estimated_lag_s": lag_s,
        "master_position_norm_peak_to_peak_m": master_norm_ptp,
        "slave_position_norm_peak_to_peak_m": slave_norm_ptp,
    }


def _run_force(master_rows: list, slave_rows: list, ts: list, analysis_dir: Path,
               master_pec_intervals, slave_pec_intervals, master_ffc_intervals, slave_ffc_intervals) -> dict:
    """Master applied force vs slave felt force -- both are applied_force_n,
    the device-write GROUND TRUTH this project's own convention requires
    (never reconstructed from spring+damping+aux+relay). Previously Y-axis
    only (merge_streams() only carries applied_force_n_y); extended on
    request to cover X and Z the same way the position re-check already
    does, by reading applied_force_n_x/y/z directly from raw_master.csv/
    raw_slave.csv (nearest-timestamp matched, same JOIN_TOLERANCE_S
    merge_streams itself uses) rather than extending that shared
    function's row shape for a one-off plot. Also reports the combined
    force-vector norm, since a per-axis-only comparison would miss net
    force felt across all three axes at once."""
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MultipleLocator

    def _clean_time_axis(ax, times, step_s=5):
        """2026-09-10, on request ('too much clutter, put time values on
        the axis and label these times between them'): force plots keep
        ONLY the two force traces + legend -- no clamp shading, no zero
        line -- plus explicit, evenly-spaced time tick labels (every
        step_s seconds) with a light gridline at each, instead of relying
        on matplotlib's default sparse/arbitrary ticks."""
        t_max = max(times) if times else 0.0
        ax.set_xlim(0, t_max)
        ax.xaxis.set_major_locator(MultipleLocator(step_s))
        ax.set_xticks([i for i in range(0, int(t_max) + step_s, step_s)])
        ax.grid(True, axis="x", alpha=0.25, linewidth=0.6)

    n = len(master_rows)
    aligned_slave = [_nearest(slave_rows, m["t_recv_monotonic"], JOIN_TOLERANCE_S) for m in master_rows]

    per_axis_result = {}

    mf = {a: [_num(m.get(f"applied_force_n_{a}")) for m in master_rows] for a in AXES}
    sf = {a: [_num(s.get(f"applied_force_n_{a}")) if s is not None else None for s in aligned_slave] for a in AXES}

    for axis in AXES:
        mfv_all, sfv_all = mf[axis], sf[axis]
        valid_idx = [i for i in range(n) if mfv_all[i] is not None and sfv_all[i] is not None]
        if not valid_idx:
            print(f"[axis-recheck][force-{axis}] SKIPPED: no paired applied_force_n_{axis} samples")
            per_axis_result[axis] = {"result": "NOT_EVALUABLE"}
            continue

        mfv = [mfv_all[i] for i in valid_idx]
        sfv = [sfv_all[i] for i in valid_idx]
        rmse = math.sqrt(sum((m - s) ** 2 for m, s in zip(mfv, sfv)) / len(mfv))
        peak_master = max(abs(v) for v in mfv)
        peak_slave = max(abs(v) for v in sfv)
        corr = _pearson(mfv, sfv)

        print(f"[axis-recheck][force-{axis}] rows valid-paired={len(valid_idx)}/{n}")
        print(f"[axis-recheck][force-{axis}] master peak|F|={peak_master:.4f}N slave peak|F|={peak_slave:.4f}N "
             f"master-vs-slave RMSE={rmse:.4f}N correlation={corr}")

        mf_plot = [v if v is not None else float("nan") for v in mfv_all]
        sf_plot = [v if v is not None else float("nan") for v in sfv_all]
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(ts, mf_plot, color="tab:blue", linewidth=1.1, label="master applied force")
        ax.plot(ts, sf_plot, color="tab:green", linewidth=1.1, label="slave felt force")
        _clean_time_axis(ax, ts)
        ax.set_xlabel("t (s)"); ax.set_ylabel(f"applied_force_n_{axis} (N)")
        ax.legend(loc="upper right", fontsize=9)
        fig.tight_layout(); fig.savefig(analysis_dir / f"force_recheck_01_applied_force_{axis}.png", dpi=150)
        plt.close(fig)

        per_axis_result[axis] = {
            "result": "EVALUATED",
            "rmse_n": rmse,
            "master_peak_abs_force_n": peak_master,
            "slave_peak_abs_force_n": peak_slave,
            "master_vs_slave_correlation": corr,
        }

    # Combined force-vector norm (all 3 axes together).
    valid_idx_norm = [i for i in range(n)
                      if all(mf[a][i] is not None for a in AXES) and all(sf[a][i] is not None for a in AXES)]
    norm_result = {"result": "NOT_EVALUABLE"}
    if valid_idx_norm:
        m_norm_v = [math.sqrt(sum(mf[a][i] ** 2 for a in AXES)) for i in valid_idx_norm]
        s_norm_v = [math.sqrt(sum(sf[a][i] ** 2 for a in AXES)) for i in valid_idx_norm]
        rmse_norm = math.sqrt(sum((m - s) ** 2 for m, s in zip(m_norm_v, s_norm_v)) / len(m_norm_v))
        peak_master_norm = max(m_norm_v)
        peak_slave_norm = max(s_norm_v)
        corr_norm = _pearson(m_norm_v, s_norm_v)
        print(f"[axis-recheck][force-norm] master peak||F||={peak_master_norm:.4f}N "
             f"slave peak||F||={peak_slave_norm:.4f}N RMSE={rmse_norm:.4f}N correlation={corr_norm}")

        m_norm_plot = [math.sqrt(sum(mf[a][i] ** 2 for a in AXES)) if all(mf[a][i] is not None for a in AXES)
                      else float("nan") for i in range(n)]
        s_norm_plot = [math.sqrt(sum(sf[a][i] ** 2 for a in AXES)) if all(sf[a][i] is not None for a in AXES)
                      else float("nan") for i in range(n)]
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(ts, m_norm_plot, color="tab:blue", linewidth=1.1, label="||master applied force||")
        ax.plot(ts, s_norm_plot, color="tab:green", linewidth=1.1, label="||slave felt force||")
        _clean_time_axis(ax, ts)
        ax.set_xlabel("t (s)"); ax.set_ylabel("||applied_force_n|| (N)")
        ax.legend(loc="upper right", fontsize=9)
        fig.tight_layout(); fig.savefig(analysis_dir / "force_recheck_02_applied_force_norm.png", dpi=150)
        plt.close(fig)

        norm_result = {
            "result": "EVALUATED",
            "rmse_n": rmse_norm,
            "master_peak_norm_n": peak_master_norm,
            "slave_peak_norm_n": peak_slave_norm,
            "master_vs_slave_correlation": corr_norm,
        }

    return {"per_axis": per_axis_result, "combined_norm": norm_result}


def main():
    trial_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "/home/nural/HIL-RL/experiment_runs/teleop_setup/"
        "20260910T155604Z_full4ch_current_setup_baseline/manual_motion/current_setup_baseline")
    analysis_dir = trial_dir.parents[1] / "analysis"
    analysis_dir.mkdir(parents=True, exist_ok=True)

    master_rows = _read_csv(trial_dir / "raw_master.csv")
    slave_rows = _read_csv(trial_dir / "raw_slave.csv")
    bridge_rows = _read_csv(trial_dir / "raw_bridge.csv")
    bilateral_rows = _read_csv(trial_dir / "raw_bilateral.csv")
    if not master_rows or not slave_rows:
        raise SystemExit(f"refusing to proceed: raw_master.csv ({len(master_rows)} rows) or "
                         f"raw_slave.csv ({len(slave_rows)} rows) missing/empty -- real recorded "
                         f"hardware data is required, nothing will be synthesized")

    merged = merge_streams(master_rows, slave_rows, bridge_rows, bilateral_rows)

    # position_error_clamped (no axis suffix) is the OVERALL boolean flag
    # (position_error_clamped_m_x/y/z, by contrast, are the numeric CLAMPED
    # ERROR VALUES per axis, not per-axis booleans -- there is no separate
    # per-axis clamp flag in this telemetry, so the same overall-flag
    # shading is reused across all three axis plots). Not carried by
    # merge_streams() already (it only carries final_force_clamped) --
    # pulled directly here via the same nearest-timestamp matching
    # merge_streams itself uses.
    for i, m in enumerate(master_rows):
        t = m["t_recv_monotonic"]
        s = _nearest(slave_rows, t, JOIN_TOLERANCE_S)
        merged[i]["master_pos_err_clamped_overall"] = _bool(m.get("position_error_clamped"))
        merged[i]["slave_pos_err_clamped_overall"] = _bool(s.get("position_error_clamped")) if s is not None else None

    n = len(merged)
    t0 = merged[0]["t"]
    ts = [r["t"] - t0 for r in merged]

    master_ffc = [bool(r["master_final_force_clamped"]) if r["master_final_force_clamped"] is not None else False
                 for r in merged]
    slave_ffc = [bool(r["slave_final_force_clamped"]) if r["slave_final_force_clamped"] is not None else False
                for r in merged]
    master_pec = [bool(r["master_pos_err_clamped_overall"]) if r["master_pos_err_clamped_overall"] is not None else False
                 for r in merged]
    slave_pec = [bool(r["slave_pos_err_clamped_overall"]) if r["slave_pos_err_clamped_overall"] is not None else False
                for r in merged]

    master_ffc_intervals = _intervals(master_ffc, ts)
    slave_ffc_intervals = _intervals(slave_ffc, ts)
    master_pec_intervals = _intervals(master_pec, ts)
    slave_pec_intervals = _intervals(slave_pec, ts)

    print(f"[axis-recheck] rows: master={len(master_rows)} slave={len(slave_rows)} merged={n}")
    print(f"[axis-recheck] master position-error-clamp intervals: {len(master_pec_intervals)}, "
         f"total {sum(b-a for a,b in master_pec_intervals):.3f}s")
    print(f"[axis-recheck] slave position-error-clamp intervals: {len(slave_pec_intervals)}, "
         f"total {sum(b-a for a,b in slave_pec_intervals):.3f}s")
    print(f"[axis-recheck] master final-force-clamp intervals: {len(master_ffc_intervals)}")
    print(f"[axis-recheck] slave final-force-clamp intervals: {len(slave_ffc_intervals)}, "
         f"total {sum(b-a for a,b in slave_ffc_intervals):.3f}s")

    import matplotlib
    matplotlib.use("Agg")

    per_axis = {}
    for axis in AXES:
        per_axis[axis] = _run_axis(axis, merged, ts, analysis_dir, master_pec, slave_pec,
                                   master_ffc, slave_ffc, master_pec_intervals, slave_pec_intervals,
                                   master_ffc_intervals, slave_ffc_intervals)

    ee = _run_ee(merged, ts, analysis_dir, master_pec_intervals, slave_pec_intervals,
                master_ffc_intervals, slave_ffc_intervals)

    force = _run_force(master_rows, slave_rows, ts, analysis_dir, master_pec_intervals, slave_pec_intervals,
                       master_ffc_intervals, slave_ffc_intervals)

    summary = {
        "trial_dir": str(trial_dir),
        "generated_from": "raw_master.csv / raw_slave.csv (this script's own recomputation, "
                          "not metrics.json)",
        "n_master_rows": len(master_rows),
        "n_slave_rows": len(slave_rows),
        "shading_alpha": {"position_error_clamp": POS_ERR_CLAMP_ALPHA,
                          "final_force_clamp": FINAL_FORCE_CLAMP_ALPHA},
        "per_axis": per_axis,
        "end_effector_combined": ee,
        "applied_force_master_vs_slave": force,
        "master_position_error_clamp_intervals_s": master_pec_intervals,
        "slave_position_error_clamp_intervals_s": slave_pec_intervals,
        "master_final_force_clamp_intervals_s": master_ffc_intervals,
        "slave_final_force_clamp_intervals_s": slave_ffc_intervals,
    }
    (analysis_dir / "y_axis_recheck_summary.json").write_text(json.dumps(summary, indent=2))
    print(f"[axis-recheck] wrote plots + summary to {analysis_dir}")


if __name__ == "__main__":
    main()
