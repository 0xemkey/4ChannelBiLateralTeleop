#!/usr/bin/env python3
"""Overlays a 2-channel phase and a 4-channel phase (both from
log_phase2_scenario.py captures) on the same axes, so the effect of adding
C1/C4 can be read directly off one figure instead of eyeballing two
separate plot sets.

TWO INPUT MODES:
  --single-csv ONE_FILE.csv   ONE continuous recording that started
                               2-channel (C3 only) and had bilateral_4ch.py
                               started partway through, without stopping
                               the logger -- the recommended way to do this
                               now (added 2026-09-07), since it means one
                               session instead of two separate runs. Splits
                               automatically using the c1_active/c4_active
                               columns log_phase2_scenario.py now records
                               (real BilateralStatus.channel_active data,
                               not a guessed timestamp).
  --2ch FILE1.csv --4ch FILE2.csv
                               Two SEPARATE recording sessions, no shared
                               wall-clock -- the original mode, kept for
                               when you already have two separate files
                               (e.g. from before this feature existed).

IMPORTANT CAVEAT, stated on every figure this script produces: in EITHER
mode, the x-axis is each phase's own elapsed time since IT started
(t_sync, re-zeroed per phase), not a shared real-time alignment --
overlaying them answers "how did behavior differ between the two
phases/runs," not "what happened at the same instant." If the motion
wasn't similar in both phases, the overlay will look different for that
reason alone, not because of C1/C4.

PRODUCES, per axis/joint:
    compare_position_x/y/z.png    master & slave position, 2ch vs 4ch
    compare_force_x/y/z.png       master & slave REAL applied_force
                                   (Newtons), 2ch vs 4ch -- the main
                                   "does 4ch actually add felt force"
                                   comparison
    compare_force_c1_waist/shoulder/elbow.png
    compare_force_c4_waist/shoulder/elbow.png
                                   per-joint relay signal (source vs
                                   received, N*m), 2ch vs 4ch overlaid --
                                   confirms the relay is literally flat
                                   zero in the 2ch capture (nothing was
                                   relaying) and live in the 4ch one

COLOR CONVENTION (same across every plot this script makes):
    2ch -- blue family   (tab:blue master, tab:cyan slave)
    4ch -- red family    (tab:red master, tab:orange slave... see below)
    master -- solid line
    slave  -- dashed line

Does NOT reproduce plot_phase2_scenario.py's haptic_force (Jacobian-
converted estimate-vs-reality) comparison -- kept out of scope for this
first version; ask if you want it added.

USAGE:
    python3 plot_comparison.py --single-csv path/to/one_continuous_capture.csv \\
        --out-dir ~/dob_logs/2ch_vs_4ch/compare

    python3 plot_comparison.py --2ch path/to/2ch_capture.csv \\
        --4ch path/to/4ch_capture.csv --out-dir ~/dob_logs/2ch_vs_4ch/compare

No ROS dependency -- reads the two CSVs only (matplotlib + stdlib), same
as plot_phase2_scenario.py, whose load_csv() this reuses directly rather
than re-implementing CSV parsing a second time.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from plot_phase2_scenario import load_csv  # noqa: E402 -- reuse, don't duplicate

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 2ch: blue family. 4ch: red/orange family. Master solid, slave dashed --
# fixed regardless of which axis/joint, so the legend means the same thing
# on every figure this script produces.
COLOR_2CH_MASTER = "tab:blue"
COLOR_2CH_SLAVE = "tab:cyan"
COLOR_4CH_MASTER = "tab:red"
COLOR_4CH_SLAVE = "tab:orange"

CAVEAT_TEXT = ("x-axis: elapsed time since each phase's own start, re-zeroed per "
               "phase for overlay -- not a shared timeline, even in --single-csv mode")


def _add_caveat(fig) -> None:
    fig.text(0.5, 0.005, CAVEAT_TEXT, ha="center", va="bottom",
              fontsize=7.5, color="dimgray", style="italic")


def plot_position_comparison(cols_2ch: dict, cols_4ch: dict, out_dir: Path,
                             label_2ch: str, label_4ch: str) -> list:
    paths = []
    for axis in ("x", "y", "z"):
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.plot(cols_2ch["t_sync"], cols_2ch[f"master_{axis}"],
                color=COLOR_2CH_MASTER, linestyle="-", linewidth=1.3,
                label=f"master ({label_2ch})")
        ax.plot(cols_2ch["t_sync"], cols_2ch[f"slave_{axis}"],
                color=COLOR_2CH_SLAVE, linestyle="--", linewidth=1.3,
                label=f"slave ({label_2ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[f"master_{axis}"],
                color=COLOR_4CH_MASTER, linestyle="-", linewidth=1.3,
                label=f"master ({label_4ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[f"slave_{axis}"],
                color=COLOR_4CH_SLAVE, linestyle="--", linewidth=1.3,
                label=f"slave ({label_4ch})")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_xlabel("elapsed time in capture, s")
        ax.set_ylabel("position, m")
        ax.set_title(f"Position comparison -- {axis} axis, {label_2ch} vs {label_4ch}")
        ax.legend(loc="upper right", fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)
        fig.tight_layout(rect=(0, 0.02, 1, 1))
        _add_caveat(fig)

        out_path = out_dir / f"compare_position_{axis}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def plot_force_comparison(cols_2ch: dict, cols_4ch: dict, out_dir: Path,
                          label_2ch: str, label_4ch: str) -> list:
    """Real rendered Cartesian force (applied_force), master & slave,
    2ch vs 4ch overlaid. This is the direct answer to 'does adding C1/C4
    actually put force on the devices' -- in a clean 2ch (C3-only) capture
    this should sit near zero throughout; if 4ch shows real nonzero force
    during the same kind of motion, the relay is doing something."""
    paths = []
    for axis in ("x", "y", "z"):
        key = f"applied_force_{axis}"
        if f"master_{key}" not in cols_2ch or f"master_{key}" not in cols_4ch:
            print(f"[plot_comparison] axis {axis}: applied_force columns missing in "
                  f"one of the two captures (older capture?) -- skipping")
            continue
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.plot(cols_2ch["t_sync"], cols_2ch[f"master_{key}"],
                color=COLOR_2CH_MASTER, linestyle="-", linewidth=1.3,
                label=f"master ({label_2ch})")
        ax.plot(cols_2ch["t_sync"], cols_2ch[f"slave_{key}"],
                color=COLOR_2CH_SLAVE, linestyle="--", linewidth=1.3,
                label=f"slave ({label_2ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[f"master_{key}"],
                color=COLOR_4CH_MASTER, linestyle="-", linewidth=1.3,
                label=f"master ({label_4ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[f"slave_{key}"],
                color=COLOR_4CH_SLAVE, linestyle="--", linewidth=1.3,
                label=f"slave ({label_4ch})")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_xlabel("elapsed time in capture, s")
        ax.set_ylabel("force, N (applied_force, real)")
        ax.set_title(f"Real rendered force comparison -- {axis} axis, "
                     f"{label_2ch} vs {label_4ch}")
        ax.legend(loc="upper right", fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)
        fig.tight_layout(rect=(0, 0.02, 1, 1))
        _add_caveat(fig)

        out_path = out_dir / f"compare_force_{axis}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def plot_relay_channel_comparison(cols_2ch: dict, cols_4ch: dict, out_dir: Path,
                                  source_prefix: str, received_prefix: str,
                                  channel_label: str, channel_tag: str,
                                  label_2ch: str, label_4ch: str) -> list:
    """Per-joint relay signal (source vs received, N*m), 2ch vs 4ch
    overlaid. In a real C3-only 2ch capture there is no bilateral_4ch.py
    running at all, so `received` there is log_phase2_scenario.py's own
    zero-force fallback (see that script's _ZERO_FORCE_ESTIMATE) -- a flat
    zero line, correctly representing 'nothing was being relayed', not a
    missing-data gap."""
    paths = []
    for joint in ("waist", "shoulder", "elbow"):
        src_key = f"{source_prefix}_{joint}"
        rcv_key = f"{received_prefix}_{joint}"
        if src_key not in cols_2ch or src_key not in cols_4ch:
            print(f"[plot_comparison] {channel_label} {joint}: columns missing -- skipping")
            continue
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.plot(cols_2ch["t_sync"], cols_2ch[src_key],
                color=COLOR_2CH_MASTER, linestyle="-", linewidth=1.2,
                label=f"source ({label_2ch})")
        ax.plot(cols_2ch["t_sync"], cols_2ch[rcv_key],
                color=COLOR_2CH_SLAVE, linestyle="--", linewidth=1.2,
                label=f"received ({label_2ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[src_key],
                color=COLOR_4CH_MASTER, linestyle="-", linewidth=1.2,
                label=f"source ({label_4ch})")
        ax.plot(cols_4ch["t_sync"], cols_4ch[rcv_key],
                color=COLOR_4CH_SLAVE, linestyle="--", linewidth=1.2,
                label=f"received ({label_4ch})")
        ax.axhline(0.0, color="k", linewidth=0.5)
        ax.set_xlabel("elapsed time in capture, s")
        ax.set_ylabel("joint torque, N*m")
        ax.set_title(f"{channel_label} relay -- {joint}, {label_2ch} vs {label_4ch}")
        ax.legend(loc="upper right", fontsize=8, ncol=2)
        ax.grid(True, alpha=0.3)
        fig.tight_layout(rect=(0, 0.02, 1, 1))
        _add_caveat(fig)

        out_path = out_dir / f"compare_force_{channel_tag}_{joint}.png"
        fig.savefig(out_path, dpi=130)
        plt.close(fig)
        paths.append(out_path)
    return paths


def split_by_channel_activity(cols: dict, csv_path: Path) -> tuple:
    """Splits ONE continuous log_phase2_scenario.py capture into a (2ch, 4ch)
    pair of cols-shaped dicts, using the c1_active/c4_active columns it now
    records (added 2026-09-07 specifically so a single session -- start
    2-channel, start bilateral_4ch.py partway through, keep recording --
    doesn't need two separate runs). 2ch = rows where NEITHER C1 nor C4 is
    active; 4ch = rows where EITHER is. Each half's own t_sync is re-zeroed
    to start at 0, matching the two-separate-file mode's "elapsed time since
    this phase started" convention.

    Raises SystemExit with a clear message if the columns aren't present
    (an older capture, predating this feature) -- use --2ch/--4ch with two
    separate files for those instead."""
    if "c1_active" not in cols or "c4_active" not in cols:
        print(f"[plot_comparison] {csv_path}: no c1_active/c4_active columns -- "
              f"this capture predates the single-session split feature "
              f"(2026-09-07). Re-record with the current log_phase2_scenario.py, "
              f"or use --2ch/--4ch with two separate capture files instead.",
              file=sys.stderr)
        sys.exit(1)

    n = len(cols["t_sync"])
    is_4ch = [bool(cols["c1_active"][i]) or bool(cols["c4_active"][i]) for i in range(n)]

    def _subset(mask_value: bool) -> dict:
        idx = [i for i in range(n) if is_4ch[i] == mask_value]
        sub = {k: [v[i] for i in idx] for k, v in cols.items()}
        if sub["t_sync"]:
            t0 = sub["t_sync"][0]
            sub["t_sync"] = [t - t0 for t in sub["t_sync"]]
        return sub

    cols_2ch = _subset(False)
    cols_4ch = _subset(True)
    print(f"[plot_comparison] {csv_path.name}: split into {len(cols_2ch['t_sync'])} "
          f"2ch samples + {len(cols_4ch['t_sync'])} 4ch samples")
    if not cols_2ch["t_sync"]:
        print("[plot_comparison] WARNING: zero 2ch-phase samples -- was bilateral_4ch.py "
              "already running for the whole capture?", file=sys.stderr)
    if not cols_4ch["t_sync"]:
        print("[plot_comparison] WARNING: zero 4ch-phase samples -- was bilateral_4ch.py "
              "ever started during this capture?", file=sys.stderr)
    return cols_2ch, cols_4ch


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--single-csv", type=Path,
                      help="ONE continuous capture spanning both phases (started 2ch, "
                           "bilateral_4ch.py started partway through) -- auto-split by "
                           "the c1_active/c4_active columns instead of taking two files.")
    mode.add_argument("--2ch", dest="csv_2ch", type=Path,
                      help="CSV from a separate 2-channel (C3-only) capture. Requires --4ch too.")
    p.add_argument("--4ch", dest="csv_4ch", type=Path,
                   help="CSV from a separate 4-channel (C1+C3+C4) capture. Requires --2ch too.")
    p.add_argument("--out-dir", type=Path, default=None,
                   help="Defaults to the input CSV's own parent directory.")
    p.add_argument("--label-2ch", default="2ch")
    p.add_argument("--label-4ch", default="4ch")
    args = p.parse_args()

    if args.csv_2ch is not None and args.csv_4ch is None:
        p.error("--2ch requires --4ch too")
    if args.csv_4ch is not None and args.csv_2ch is None:
        p.error("--4ch requires --2ch too")

    if args.single_csv is not None:
        if not args.single_csv.exists():
            print(f"[plot_comparison] {args.single_csv}: not found", file=sys.stderr)
            sys.exit(1)
        cols_all = load_csv(args.single_csv)
        if not cols_all.get("t_sync"):
            print(f"[plot_comparison] {args.single_csv}: no data rows", file=sys.stderr)
            sys.exit(1)
        cols_2ch, cols_4ch = split_by_channel_activity(cols_all, args.single_csv)
        out_dir = args.out_dir if args.out_dir else args.single_csv.parent
    else:
        for path in (args.csv_2ch, args.csv_4ch):
            if not path.exists():
                print(f"[plot_comparison] {path}: not found", file=sys.stderr)
                sys.exit(1)
        cols_2ch = load_csv(args.csv_2ch)
        cols_4ch = load_csv(args.csv_4ch)
        if not cols_2ch.get("t_sync") or not cols_4ch.get("t_sync"):
            print("[plot_comparison] one of the two captures has no data rows", file=sys.stderr)
            sys.exit(1)
        out_dir = args.out_dir if args.out_dir else args.csv_4ch.parent

    if not cols_2ch.get("t_sync") or not cols_4ch.get("t_sync"):
        print("[plot_comparison] one of the two phases has zero samples -- nothing to "
              "compare, see the WARNING above", file=sys.stderr)
        sys.exit(1)

    out_dir.mkdir(parents=True, exist_ok=True)

    all_paths = []
    all_paths += plot_position_comparison(cols_2ch, cols_4ch, out_dir, args.label_2ch, args.label_4ch)
    all_paths += plot_force_comparison(cols_2ch, cols_4ch, out_dir, args.label_2ch, args.label_4ch)
    all_paths += plot_relay_channel_comparison(
        cols_2ch, cols_4ch, out_dir,
        "master_force_ext_hat", "slave_joint_torque_ext_cmd",
        "C4 (master->slave)", "c4", args.label_2ch, args.label_4ch)
    all_paths += plot_relay_channel_comparison(
        cols_2ch, cols_4ch, out_dir,
        "slave_force_ext_hat", "master_joint_torque_ext_cmd",
        "C1 (slave->master)", "c1", args.label_2ch, args.label_4ch)

    names = "\n  ".join(p.name for p in all_paths)
    print(f"[plot_comparison] {len(all_paths)} figures written to {out_dir}:\n  {names}")


if __name__ == "__main__":
    main()
