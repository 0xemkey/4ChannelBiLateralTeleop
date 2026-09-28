#!/usr/bin/env python3
"""Plotting for the Y-axis hardware validation task (Part 6 of the task
spec). Consumes:
  - part1_gravity_settle.csv / part1_meta.json (Part 1, gravity-only,
    written by part1_gravity_settle_capture.py, once that script exists
    and has been run)
  - part2_telemetry.csv / part2_meta.json (Part 2, master-only +Y,
    written by part2_master_only_y_motion.py)
  - the STANDARD recorder CSV schema from
    projects/teleop_left_master_right_slave/experiment_launch/
    run_gravcomp_trial.py (left_telem.csv, right_telem.csv,
    bridge_diag.csv, left_pose.csv, right_pose.csv) for Part 4 (bilateral)

Pure post-hoc plotting -- reads files, writes PNGs, publishes/commands
nothing. Safe to run at any time, including now (against no real data --
each plot function is skipped with a printed note if its inputs are
missing, rather than failing the whole script).
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _read_csv(path):
    p = Path(path)
    if not p.exists():
        return None
    with open(p) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return None
    return rows


def _col(rows, key, cast=float):
    return [cast(r[key]) for r in rows]


def _t0(rows):
    ts = _col(rows, "t") if "t" in rows[0] else _col(rows, "wall_t")
    t0 = ts[0]
    return [t - t0 for t in ts], ts


def _mark_states(ax, rows, phase_key="phase"):
    if phase_key not in rows[0]:
        return
    ts, _ = _t0(rows)
    last_phase = None
    for t, r in zip(ts, rows):
        p = r[phase_key]
        if p != last_phase:
            ax.axvline(t, color="gray", linestyle=":", linewidth=0.8, alpha=0.6)
            ax.annotate(p, (t, ax.get_ylim()[1]), rotation=90, fontsize=6,
                       va="top", ha="right", color="gray")
            last_phase = p


def plot_part1_gravity_settle(out_dir: Path, plot_dir: Path):
    rows_m = _read_csv(out_dir / "part1_gravity_settle_master.csv")
    rows_s = _read_csv(out_dir / "part1_gravity_settle_slave.csv")
    if rows_m is None or rows_s is None:
        print("[plot] Part 1 CSVs not found -- skipping gravity-settle plots")
        return
    ts_m, _ = _t0(rows_m)
    ts_s, _ = _t0(rows_s)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(ts_m, _col(rows_m, "vy"), label="master vy")
    axes[0].plot(ts_s, _col(rows_s, "vy"), label="slave vy")
    axes[0].set_ylabel("vy (m/s)"); axes[0].legend(); axes[0].set_title("Part 1: GRAVITY_SETTLE velocity")
    axes[1].plot(ts_m, _col(rows_m, "aux_y"), label="master aux_y")
    axes[1].plot(ts_s, _col(rows_s, "aux_y"), label="slave aux_y")
    axes[1].set_ylabel("gravity/aux force (N)"); axes[1].legend()
    axes[2].plot(ts_m, _col(rows_m, "dob_y"), label="master DOB_y")
    axes[2].plot(ts_s, _col(rows_s, "dob_y"), label="slave DOB_y")
    axes[2].set_ylabel("DOB_y (Nm)"); axes[2].set_xlabel("t (s)"); axes[2].legend()
    _mark_states(axes[0], rows_m)
    fig.tight_layout()
    fig.savefig(plot_dir / "part1_gravity_settle.png", dpi=150)
    plt.close(fig)
    print("[plot] wrote part1_gravity_settle.png")


def plot_part2_master_only(out_dir: Path, plot_dir: Path):
    rows = _read_csv(out_dir / "part2_telemetry.csv")
    if rows is None:
        print("[plot] Part 2 CSV not found -- skipping master-only-Y plots")
        return
    ts, _ = _t0(rows)
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axes[0].plot(ts, _col(rows, "y"), label="y (actual)")
    axes[0].plot(ts, _col(rows, "target_y"), label="y (target)", linestyle="--")
    axes[0].set_ylabel("Y position (m)"); axes[0].legend(); axes[0].set_title("Part 2: master-only +Y motion")
    axes[1].plot(ts, _col(rows, "vy"))
    axes[1].set_ylabel("vy (m/s)")
    axes[2].plot(ts, _col(rows, "err_y"), label="err_y")
    axes[2].set_ylabel("Y tracking error (m)"); axes[2].legend()
    axes[3].plot(ts, _col(rows, "applied_y"), label="applied_y")
    axes[3].plot(ts, _col(rows, "spring_y"), label="spring_y")
    axes[3].plot(ts, _col(rows, "damping_y"), label="damping_y")
    axes[3].plot(ts, _col(rows, "aux_y"), label="aux_y")
    axes[3].set_ylabel("force (N)"); axes[3].set_xlabel("t (s)"); axes[3].legend(fontsize=7)
    _mark_states(axes[0], rows)
    fig.tight_layout()
    fig.savefig(plot_dir / "part2_master_only_y.png", dpi=150)
    plt.close(fig)
    print("[plot] wrote part2_master_only_y.png")

    # X/Z excursion check (should stay ~0 relative to latched origin).
    fig2, ax = plt.subplots(figsize=(10, 4))
    x0, z0 = float(rows[0]["x"]), float(rows[0]["z"])
    ax.plot(ts, [float(r["x"]) - x0 for r in rows], label="x - x0")
    ax.plot(ts, [float(r["z"]) - z0 for r in rows], label="z - z0")
    ax.axhline(0.0, color="k", linewidth=0.5)
    ax.set_ylabel("excursion (m)"); ax.set_xlabel("t (s)"); ax.legend()
    ax.set_title("Part 2: X/Z excursion during Y-only motion (should stay ~0)")
    fig2.tight_layout()
    fig2.savefig(plot_dir / "part2_xz_excursion.png", dpi=150)
    plt.close(fig2)
    print("[plot] wrote part2_xz_excursion.png")


def plot_part4_bilateral(out_dir: Path, plot_dir: Path):
    """Standard recorder CSV schema from run_gravcomp_trial.py."""
    lt = _read_csv(out_dir / "left_telem.csv")
    rt = _read_csv(out_dir / "right_telem.csv")
    lp = _read_csv(out_dir / "left_pose.csv")
    rp = _read_csv(out_dir / "right_pose.csv")
    bd = _read_csv(out_dir / "bridge_diag.csv")
    if not all([lt, rt, lp, rp, bd]):
        print("[plot] Part 4 (bilateral) recorder CSVs not found -- skipping")
        return

    ts_p, _ = _t0(lp)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(ts_p, _col(lp, "y"), label="master y")
    ts_p2, _ = _t0(rp)
    ax.plot(ts_p2, _col(rp, "y"), label="slave y")
    ax.set_ylabel("Y position (m)"); ax.set_xlabel("t (s)"); ax.legend()
    ax.set_title("Part 4: master vs slave Y position")
    _mark_states(ax, lp)
    fig.tight_layout(); fig.savefig(plot_dir / "part4_master_vs_slave_y.png", dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ts_bd, _ = _t0(bd)
    ax.plot(ts_bd, _col(bd, "coupling_ramp_alpha"), label="coupling_ramp_alpha")
    ax.plot(ts_bd, _col(bd, "gravity_ramp_alpha"), label="gravity_ramp_alpha")
    ax.set_xlabel("t (s)"); ax.legend()
    ax.set_title("Part 4: ramp alphas vs time")
    fig.tight_layout(); fig.savefig(plot_dir / "part4_ramp_alphas.png", dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts_bd, _col(bd, "master_accepted_force_margin_n"), label="master accepted force margin")
    ax.plot(ts_bd, _col(bd, "slave_accepted_force_margin_n"), label="slave accepted force margin")
    ax.axhline(0.0, color="r", linestyle="--", linewidth=0.8)
    ax.set_ylabel("N"); ax.set_xlabel("t (s)"); ax.legend()
    ax.set_title("Part 4: accepted force margins vs time")
    fig.tight_layout(); fig.savefig(plot_dir / "part4_accepted_margins.png", dpi=150); plt.close(fig)
    print("[plot] wrote part4_*.png")


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    plot_dir = out_dir / "plots"
    plot_dir.mkdir(exist_ok=True)
    plot_part1_gravity_settle(out_dir, plot_dir)
    plot_part2_master_only(out_dir, plot_dir)
    plot_part4_bilateral(out_dir, plot_dir)


if __name__ == "__main__":
    main()
