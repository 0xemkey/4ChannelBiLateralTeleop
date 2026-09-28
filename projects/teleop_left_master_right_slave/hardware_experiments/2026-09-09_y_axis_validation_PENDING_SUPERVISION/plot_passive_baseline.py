#!/usr/bin/env python3
"""H0/H1 plotting: reads the JSON written by passive_baseline_capture.py
(summary + left_raw/right_raw/left_dob/right_dob) and writes PNGs. Pure
post-hoc -- reads a file, writes images, publishes/commands nothing.

Usage:
    python3 plot_passive_baseline.py <path/to/passive_baseline.json> [out_dir]
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def _norm3(r, prefix):
    return math.sqrt(r[f"{prefix}_x"] ** 2 + r[f"{prefix}_y"] ** 2 + r[f"{prefix}_z"] ** 2)


def _t0(rows):
    t0 = rows[0]["t"]
    return [r["t"] - t0 for r in rows]


def plot_side(rows, dob_rows, side: str, plot_dir: Path):
    if not rows:
        print(f"[plot] no {side} rows -- skipping")
        return
    ts = _t0(rows)

    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True)
    axes[0].plot(ts, [r["y"] for r in rows], label="y")
    axes[0].plot(ts, [r["x"] for r in rows], label="x", alpha=0.6)
    axes[0].plot(ts, [r["z"] for r in rows], label="z", alpha=0.6)
    axes[0].set_ylabel("position (m)"); axes[0].legend()
    axes[0].set_title(f"H0/H1 passive baseline: {side} position")

    axes[1].plot(ts, [r["vy"] for r in rows], label="vy")
    axes[1].axhline(0.0, color="k", linewidth=0.5)
    axes[1].set_ylabel("vy (m/s)"); axes[1].legend()

    axes[2].plot(ts, [r["spring_y"] for r in rows], label="spring_y")
    axes[2].plot(ts, [r["damping_y"] for r in rows], label="damping_y")
    axes[2].plot(ts, [r["aux_y"] for r in rows], label="aux_y")
    axes[2].plot(ts, [r["applied_y"] for r in rows], label="applied_y")
    axes[2].set_ylabel("force (N)"); axes[2].legend(fontsize=7)

    applied_norm = [_norm3(r, "applied") for r in rows]
    axes[3].plot(ts, applied_norm, label="|applied force|")
    axes[3].set_ylabel("N"); axes[3].set_xlabel("t (s)"); axes[3].legend()

    fig.tight_layout()
    fig.savefig(plot_dir / f"h0h1_{side}_telemetry.png", dpi=150)
    plt.close(fig)
    print(f"[plot] wrote h0h1_{side}_telemetry.png")

    if dob_rows:
        dts = _t0(dob_rows)
        tau = [r["tau_y"] for r in dob_rows if r["tau_y"] is not None]
        dts2 = [d for d, r in zip(dts, dob_rows) if r["tau_y"] is not None]
        if tau:
            fig, ax = plt.subplots(figsize=(10, 3))
            ax.plot(dts2, tau)
            ax.set_ylabel("tau_dis_hat_y (Nm)"); ax.set_xlabel("t (s)")
            ax.set_title(f"H0/H1 passive baseline: {side} DOB disturbance estimate (Y)")
            fig.tight_layout()
            fig.savefig(plot_dir / f"h0h1_{side}_dob_y.png", dpi=150)
            plt.close(fig)
            print(f"[plot] wrote h0h1_{side}_dob_y.png")


def main():
    if len(sys.argv) < 2:
        print("usage: plot_passive_baseline.py <passive_baseline.json> [out_dir]", file=sys.stderr)
        sys.exit(2)
    json_path = Path(sys.argv[1])
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else json_path.parent
    plot_dir = out_dir / "plots"
    plot_dir.mkdir(parents=True, exist_ok=True)

    data = json.loads(json_path.read_text())
    print(json.dumps(data.get("summary", {}), indent=2))

    plot_side(data.get("left_raw", []), data.get("left_dob", []), "master_left", plot_dir)
    plot_side(data.get("right_raw", []), data.get("right_dob", []), "slave_right", plot_dir)


if __name__ == "__main__":
    main()
