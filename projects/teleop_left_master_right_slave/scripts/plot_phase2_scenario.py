#!/usr/bin/env python3
"""Phase 2 (2026-09-04 fine-tuning task) -- plots from a log_phase2_scenario.py
CSV capture.

Produces nine PNGs per input CSV, one plot per axis/joint (no combined
multi-line figures) -- source (solid) vs received (dashed), same style as
a DOB-vs-ground-truth force comparison plot, minus the ground-truth line
since this rig has no force sensor:

    <csv_stem>_position_x.png       master x vs slave x over time
    <csv_stem>_position_y.png       master y vs slave y over time
    <csv_stem>_position_z.png       master z vs slave z over time
    <csv_stem>_force_c4_waist.png   C4 (master->slave), waist: master's own
                                     force_ext_hat (source) vs slave's
                                     joint_torque_ext_cmd (received)
    <csv_stem>_force_c4_shoulder.png  same, shoulder
    <csv_stem>_force_c4_elbow.png     same, elbow
    <csv_stem>_force_c1_waist.png   C1 (slave->master), waist: slave's own
                                     force_ext_hat (source) vs master's
                                     joint_torque_ext_cmd (received)
    <csv_stem>_force_c1_shoulder.png  same, shoulder
    <csv_stem>_force_c1_elbow.png     same, elbow
    <csv_stem>_haptic_force_c4_x/y/z.png  the REAL comparison: master's own
                                     rendered Cartesian force (applied_force,
                                     Newtons -- what the operator actually
                                     feels) vs the slave's joint-torque
                                     estimate CONVERTED to the slave's own
                                     Cartesian frame via its Jacobian, so
                                     both sides are in the same units. This
                                     is the "DOB estimate vs haptic force"
                                     comparison, analogous to a force-sensor-
                                     vs-robot-estimate plot except neither
                                     side is a real sensor -- this rig has
                                     none.
    <csv_stem>_haptic_force_c1_x/y/z.png  same idea, C1 direction: slave's
                                     applied_force vs master's converted
                                     estimate

USAGE:
    python3 plot_phase2_scenario.py path/to/capture.csv [more.csv ...]
    python3 plot_phase2_scenario.py --out-dir ~/dob_logs/phase2_plots path/to/capture.csv

No ROS dependency -- reads CSV only, safe to run on any machine with the
capture files copied over (matplotlib + standard library only).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless-safe; this script writes files, doesn't show()
import matplotlib.pyplot as plt

# Link lengths, Koul et al. (2010) -- same constants touch_dob.hpp uses
# (kL1Meters/kL2Meters). Ported here, not imported, since this script is
# deliberately dependency-free (matplotlib + stdlib only, per the module
# docstring) so it can run on a machine with no ROS/touch_haptics install.
L1_M = 0.135
L2_M = 0.135


def jacobian(q):
    """Row-major 3x3, exact port of touch_dob.hpp's jacobian() -- Jarillo-
    Silva et al. (2009) Eq. (15). q = (waist, shoulder, elbow), rad."""
    c1, s1 = math.cos(q[0]), math.sin(q[0])
    c2, s2 = math.cos(q[1]), math.sin(q[1])
    c3, s3 = math.cos(q[2]), math.sin(q[2])
    return [
        -(L1_M * c1 * c2 + L2_M * s3 * c1), L1_M * s1 * s2, -L2_M * c3 * s1,
        0.0, L1_M * c2, L2_M * s3,
        -L1_M * c2 * s1 - L2_M * s3 * s1, -L1_M * s2 * c1, L2_M * c3 * c1,
    ]


def singularity_det(q):
    """det(J) = -L1*L2*R*cos(q2-q3), R = L1*cos(q2)+L2*sin(q3). Exact port
    of touch_dob.hpp's singularityMetrics().det."""
    r = L1_M * math.cos(q[1]) + L2_M * math.sin(q[2])
    return -L1_M * L2_M * r * math.cos(q[1] - q[2])


def cartesian_force_from_joint_torque(q, tau):
    """Solve J^T F = tau for F -- exact port of touch_dob.hpp's
    cartesianForceFromJointTorque(). Returns None near a singularity
    (|det| too small), matching touch_slave.cpp's own guard
    (singularityMetrics(q).near_singular) rather than dividing by ~0."""
    det = singularity_det(q)
    if abs(det) < 1e-6:
        return None
    j = jacobian(q)
    # M = J^T, row-major
    m = [j[0], j[3], j[6], j[1], j[4], j[7], j[2], j[5], j[8]]
    inv_det = 1.0 / det
    minv = [
        (m[4]*m[8]-m[5]*m[7])*inv_det, (m[2]*m[7]-m[1]*m[8])*inv_det, (m[1]*m[5]-m[2]*m[4])*inv_det,
        (m[5]*m[6]-m[3]*m[8])*inv_det, (m[0]*m[8]-m[2]*m[6])*inv_det, (m[2]*m[3]-m[0]*m[5])*inv_det,
        (m[3]*m[7]-m[4]*m[6])*inv_det, (m[1]*m[6]-m[0]*m[7])*inv_det, (m[0]*m[4]-m[1]*m[3])*inv_det,
    ]
    return (
        minv[0]*tau[0] + minv[1]*tau[1] + minv[2]*tau[2],
        minv[3]*tau[0] + minv[4]*tau[1] + minv[5]*tau[2],
        minv[6]*tau[0] + minv[7]*tau[1] + minv[8]*tau[2],
    )


def load_csv(path: Path) -> dict:
    cols: dict = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        for name in fieldnames:
            cols[name] = []
        for row in reader:
            for name in fieldnames:
                cols[name].append(float(row[name]))
    return cols


def plot_position(cols: dict, out_dir: Path, stem: str, title_prefix: str) -> list:
    """One separate figure per axis: master vs slave over time. Returns the
    list of paths written."""
    t = cols["t_sync"]
    axis_colors = {"x": "tab:blue", "y": "tab:orange", "z": "tab:green"}
    paths = []
    for axis in ("x", "y", "z"):
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(t, cols[f"master_{axis}"], color=axis_colors[axis],
                 linestyle="-", label=f"master {axis}")
        ax.plot(t, cols[f"slave_{axis}"], color=axis_colors[axis],
                 linestyle="--", label=f"slave {axis}")
        ax.set_xlabel("time, s")
        ax.set_ylabel("position, m")
        ax.set_title(f"{title_prefix} -- {axis} axis, master (solid) vs slave (dashed)")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

        out_path = out_dir / f"{stem}_position_{axis}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def _contact_spans(t, force_a, force_b, threshold_n, min_duration_s):
    """Real, derived (not fabricated) contact detection: a sample counts as
    'contact' when EITHER side's actual rendered force magnitude clears
    threshold_n. Spans shorter than min_duration_s are dropped -- ordinary
    C2/C3 tracking-error noise produces brief, small force blips that clear
    a low threshold for a single sample and would otherwise flicker the
    shading; a real deliberate push stays above threshold for a sustained
    stretch. This is a debounce on real data, not a fabricated event --
    every remaining span still starts/ends exactly where the real signal
    crosses threshold_n. Returns (t_start, t_end) spans for axvspan shading,
    matching the reference figure's boxed 'Contact' regions."""
    in_contact = [abs(a) > threshold_n or abs(b) > threshold_n
                  for a, b in zip(force_a, force_b)]
    raw_spans = []
    start = None
    for i, flag in enumerate(in_contact):
        if flag and start is None:
            start = t[i]
        elif not flag and start is not None:
            raw_spans.append((start, t[i]))
            start = None
    if start is not None:
        raw_spans.append((start, t[-1]))
    return [(t0, t1) for t0, t1 in raw_spans if (t1 - t0) >= min_duration_s]


def plot_combined_position_force(cols: dict, out_dir: Path, stem: str, title_prefix: str,
                                  contact_threshold_n: float = 0.15,
                                  contact_min_duration_s: float = 0.3) -> list:
    """Position (top) + REAL rendered force (bottom) on a shared time axis,
    master vs slave, one figure per axis -- matches the reference figure's
    layout exactly (master/remote position and force stacked, contact
    windows shaded). Uses applied_force directly (the actual Cartesian
    force each device renders) for both master and slave -- not the joint-
    torque estimate, not a Jacobian conversion, no derived/mocked values
    except the contact-window boundaries themselves, which come straight
    from thresholding this same real force data (see _contact_spans)."""
    t = cols["t_sync"]
    axis_colors = {"x": "tab:blue", "y": "tab:orange", "z": "tab:green"}
    paths = []
    for axis in ("x", "y", "z"):
        force_m = cols[f"master_applied_force_{axis}"]
        force_s = cols[f"slave_applied_force_{axis}"]
        spans = _contact_spans(t, force_m, force_s, contact_threshold_n, contact_min_duration_s)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        color = axis_colors[axis]

        for (t0, t1) in spans:
            ax1.axvspan(t0, t1, color="gray", alpha=0.15)
            ax2.axvspan(t0, t1, color="gray", alpha=0.15)

        pos_m_mm = [v * 1000.0 for v in cols[f"master_{axis}"]]
        pos_s_mm = [v * 1000.0 for v in cols[f"slave_{axis}"]]
        ax1.plot(t, pos_m_mm, color=color, linestyle="-", linewidth=1.3, label="master")
        ax1.plot(t, pos_s_mm, color=color, linestyle="--", linewidth=1.3, label="slave")
        ax1.axhline(0.0, color="k", linewidth=0.5)
        ax1.set_ylabel("position, mm")
        ax1.set_title(f"{title_prefix} -- {axis} axis")
        ax1.legend(loc="upper right", fontsize=9)
        ax1.grid(True, alpha=0.3)

        ax2.plot(t, force_m, color=color, linestyle="-", linewidth=1.3, label="master")
        ax2.plot(t, force_s, color=color, linestyle="--", linewidth=1.3, label="slave")
        ax2.axhline(0.0, color="k", linewidth=0.5)
        ax2.set_xlabel("time, s")
        ax2.set_ylabel("force, N (applied_force, real)")
        ax2.legend(loc="upper right", fontsize=9)
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()
        out_path = out_dir / f"{stem}_combined_{axis}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def plot_force_channel(cols: dict, out_dir: Path, stem: str, title_prefix: str,
                        source_prefix: str, received_prefix: str,
                        channel_label: str, channel_tag: str) -> list:
    """source_prefix/received_prefix: e.g. 'master_force_ext_hat' (C4 source)
    vs 'slave_joint_torque_ext_cmd' (C4 received). One separate figure per
    joint, source (solid) vs received (dashed) -- same style as a DOB-vs-
    ground-truth force comparison plot, just without a real force sensor to
    plot a third line from (this rig has none, by design -- see
    RUNNING_SCRIPTS.txt's "Reading torque" section). Returns the list of
    paths written."""
    t = cols["t_sync"]
    joint_color = "tab:red"
    paths = []
    for joint in ("waist", "shoulder", "elbow"):
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(t, cols[f"{source_prefix}_{joint}"], color=joint_color,
                 linestyle="-", linewidth=1.4, label=f"{channel_label} source ({joint})")
        ax.plot(t, cols[f"{received_prefix}_{joint}"], color="tab:blue",
                 linestyle="--", linewidth=1.4, label=f"{channel_label} received ({joint})")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_xlabel("time, s")
        ax.set_ylabel("joint torque, N*m")
        ax.set_title(f"{title_prefix} -- {channel_label}, {joint}")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

        out_path = out_dir / f"{stem}_force_{channel_tag}_{joint}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def plot_haptic_force(cols: dict, out_dir: Path, stem: str, title_prefix: str,
                       estimate_q_prefix: str, estimate_tau_prefix: str,
                       rendered_prefix: str, channel_label: str, channel_tag: str) -> list:
    """THE comparison you actually feel: converts the estimating arm's
    joint-torque DOB output into a Cartesian force via its own Jacobian
    (same math as touch_dob.hpp's cartesianForceFromJointTorque -- see the
    port at the top of this file), then plots it against the OTHER arm's
    real rendered Cartesian force (applied_force -- what that device
    actually writes, i.e. what the operator actually feels). Both sides
    end up in Newtons, in each device's own local frame -- not a shared
    global frame, same convention a real force-sensor-vs-robot-estimate
    comparison plot would use (a device only ever renders/senses force in
    its own frame anyway).

    estimate_q_prefix/estimate_tau_prefix: e.g. 'slave_q'/'slave_force_ext_hat'
    rendered_prefix: e.g. 'master_applied_force'
    """
    t = cols["t_sync"]
    n = len(t)
    est = {"x": [], "y": [], "z": []}
    dropped_singular = 0
    for i in range(n):
        q = (cols[f"{estimate_q_prefix}_waist"][i],
             cols[f"{estimate_q_prefix}_shoulder"][i],
             cols[f"{estimate_q_prefix}_elbow"][i])
        tau = (cols[f"{estimate_tau_prefix}_waist"][i],
               cols[f"{estimate_tau_prefix}_shoulder"][i],
               cols[f"{estimate_tau_prefix}_elbow"][i])
        f = cartesian_force_from_joint_torque(q, tau)
        if f is None:
            f = (float("nan"), float("nan"), float("nan"))
            dropped_singular += 1
        est["x"].append(f[0]); est["y"].append(f[1]); est["z"].append(f[2])

    if dropped_singular:
        print(f"[plot_phase2] {stem}: {dropped_singular}/{n} samples near a "
              f"singularity for {channel_label}, plotted as gaps")

    axis_colors = {"x": "tab:blue", "y": "tab:orange", "z": "tab:green"}
    paths = []
    for axis in ("x", "y", "z"):
        fig, ax = plt.subplots(figsize=(11, 4.5))
        ax.plot(t, est[axis], color=axis_colors[axis], linestyle="-", linewidth=1.4,
                 label=f"estimated (from joint torque, Jacobian-converted)")
        ax.plot(t, cols[f"{rendered_prefix}_{axis}"], color="black", linestyle="--",
                 linewidth=1.4, label=f"actually rendered (applied_force)")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_xlabel("time, s")
        ax.set_ylabel("force, N")
        ax.set_title(f"{title_prefix} -- {channel_label} haptic force, {axis}")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()

        out_path = out_dir / f"{stem}_haptic_force_{channel_tag}_{axis}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def process_one(csv_path: Path, out_dir: Path) -> None:
    cols = load_csv(csv_path)
    if not cols.get("t_sync"):
        print(f"[plot_phase2] {csv_path}: no data rows, skipping")
        return

    n = len(cols["t_sync"])
    stem = csv_path.stem
    title_prefix = stem

    out_dir.mkdir(parents=True, exist_ok=True)

    pos_paths = plot_position(cols, out_dir, stem, title_prefix)

    combined_paths = []
    if "master_applied_force_x" in cols:
        combined_paths = plot_combined_position_force(cols, out_dir, stem, title_prefix)
    else:
        print(f"[plot_phase2] {csv_path.name}: no applied_force columns "
              f"(older capture) -- skipping combined position+force plots")

    # C4: master force_ext_hat (source) -> slave joint_torque_ext_cmd (received)
    c4_paths = plot_force_channel(cols, out_dir, stem, title_prefix,
                                   "master_force_ext_hat", "slave_joint_torque_ext_cmd",
                                   "C4 (master->slave)", "c4")

    # C1: slave force_ext_hat (source) -> master joint_torque_ext_cmd (received)
    c1_paths = plot_force_channel(cols, out_dir, stem, title_prefix,
                                   "slave_force_ext_hat", "master_joint_torque_ext_cmd",
                                   "C1 (slave->master)", "c1")

    haptic_paths = []
    if "master_q_waist" in cols and "master_applied_force_x" in cols:
        # C4 haptic: master's own estimate (its joint torque, its own q) vs
        # the SLAVE's real rendered force -- does what the master senses
        # match what the slave actually renders.
        haptic_paths += plot_haptic_force(
            cols, out_dir, stem, title_prefix,
            "master_q", "master_force_ext_hat", "slave_applied_force",
            "C4 (master->slave)", "c4")
        # C1 haptic: slave's own estimate vs the MASTER's real rendered force.
        haptic_paths += plot_haptic_force(
            cols, out_dir, stem, title_prefix,
            "slave_q", "slave_force_ext_hat", "master_applied_force",
            "C1 (slave->master)", "c1")
    else:
        print(f"[plot_phase2] {csv_path.name}: no joint_angles/applied_force columns "
              f"(older capture, predates this feature) -- skipping haptic-force plots")

    all_paths = pos_paths + combined_paths + c4_paths + c1_paths + haptic_paths
    names = ", ".join(p.name for p in all_paths)
    print(f"[plot_phase2] {csv_path.name}: {n} samples -> {names}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv_files", nargs="+", type=Path)
    p.add_argument("--out-dir", type=Path, default=None,
                   help="Defaults to each CSV's own parent directory.")
    args = p.parse_args()

    for csv_path in args.csv_files:
        if not csv_path.exists():
            print(f"[plot_phase2] {csv_path}: not found, skipping")
            continue
        out_dir = args.out_dir if args.out_dir else csv_path.parent
        process_one(csv_path, out_dir)


if __name__ == "__main__":
    main()
