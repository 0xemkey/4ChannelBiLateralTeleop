#!/usr/bin/env python3
"""Post-hoc full-4ch baseline report.

Reads recorder/analyzer outputs only. It does not publish ROS commands and
does not alter validation criteria, controller gains, masks, or DOB settings.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
from statistics import median

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-full4ch-baseline")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


JOINTS = ("waist", "shoulder", "elbow")
AXES = ("x", "y", "z")


def read_csv(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def load_json(path: Path, default):
    if not path.exists():
        return default
    return json.loads(path.read_text())


def f(row: dict, key: str, default: float = 0.0) -> float:
    try:
        value = row.get(key, "")
        if value in ("", "None", None):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def b(row: dict, key: str) -> bool:
    return str(row.get(key, "")).lower() in ("true", "1", "yes")


def t0(*streams: list[dict]) -> float:
    starts = [f(s[0], "t_recv_monotonic") for s in streams if s]
    return min(starts) if starts else 0.0


def times(rows: list[dict], base: float) -> list[float]:
    return [f(r, "t_recv_monotonic") - base for r in rows]


def series(rows: list[dict], key: str) -> list[float]:
    return [f(r, key, math.nan) for r in rows]


def norm3(row: dict, prefix: str, labels=AXES) -> float:
    vals = [f(row, f"{prefix}_{label}") for label in labels]
    return math.sqrt(sum(v * v for v in vals))


def finite(xs: list[float]) -> list[float]:
    return [x for x in xs if math.isfinite(x)]


def mean(xs: list[float]) -> float | None:
    xs = finite(xs)
    return sum(xs) / len(xs) if xs else None


def rms(xs: list[float]) -> float | None:
    xs = finite(xs)
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else None


def peak_abs(xs: list[float]) -> float | None:
    xs = finite(xs)
    return max((abs(x) for x in xs), default=None)


def corr(a: list[float], c: list[float]) -> float | None:
    pairs = [(x, y) for x, y in zip(a, c) if math.isfinite(x) and math.isfinite(y)]
    if len(pairs) < 3:
        return None
    xs, ys = zip(*pairs)
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 0.0 or vy <= 0.0:
        return None
    return sum((x - mx) * (y - my) for x, y in pairs) / math.sqrt(vx * vy)


def duty(rows: list[dict], key: str) -> float | None:
    return (sum(1 for r in rows if b(r, key)) / len(rows)) if rows else None


def clamp_runs(rows: list[dict], key: str) -> dict:
    if not rows:
        return {"count": 0, "duration_s": 0.0, "duty": None, "longest_s": 0.0}
    ts = [f(r, "t_recv_monotonic") for r in rows]
    gaps = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    dt = median(gaps) if gaps else 0.0
    total = (ts[-1] - ts[0]) if len(ts) > 1 else 0.0
    count = 0
    cur = 0
    runs = []
    for r in rows:
        if b(r, key):
            if cur == 0:
                count += 1
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    duration = sum(runs) * dt
    return {
        "count": count,
        "duration_s": duration,
        "duty": (duration / total) if total > 0 else (1.0 if runs else 0.0),
        "longest_s": (max(runs) * dt) if runs else 0.0,
    }


def bool_duties(rows: list[dict], keys: list[str]) -> dict:
    return {key: duty(rows, key) for key in keys}


def add_grid(ax):
    ax.grid(True, alpha=0.25)


def save_position_plots(master: list[dict], slave: list[dict], out: Path, base: float):
    tm, ts = times(master, base), times(slave, base)
    for axis in AXES:
        fig, ax = plt.subplots(figsize=(11, 4))
        if master:
            ax.plot(tm, series(master, f"mapped_rel_{axis}"), label="master mapped relative")
        if slave:
            ax.plot(ts, series(slave, f"rel_{axis}"), label="slave relative")
        ax.set_xlabel("t (s)")
        ax.set_ylabel("position (m)")
        ax.set_title(f"Full4ch position {axis}")
        ax.legend(loc="best")
        add_grid(ax)
        fig.tight_layout()
        fig.savefig(out / f"position_{axis}.png", dpi=160)
        plt.close(fig)


def save_tracking_error_plot(bridge: list[dict], out: Path, base: float):
    tb = times(bridge, base)
    fig, axes = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
    for ax, axis in zip(axes, AXES):
        ax.plot(tb, series(bridge, f"c2_position_error_m_{axis}"), label="C2 slave->master")
        ax.plot(tb, series(bridge, f"c3_position_error_m_{axis}"), label="C3 master->slave")
        ax.set_ylabel(f"{axis} error (m)")
        ax.legend(loc="best")
        add_grid(ax)
    axes[-1].set_xlabel("t (s)")
    axes[0].set_title("Bidirectional C2/C3 position tracking error")
    fig.tight_layout()
    fig.savefig(out / "position_tracking_error_c2_c3.png", dpi=160)
    plt.close(fig)


def save_force_component_plots(rows: list[dict], side: str, max_force_n: float, out: Path, base: float):
    tr = times(rows, base)
    for axis in AXES:
        fig, ax = plt.subplots(figsize=(11, 4))
        for prefix, width in (
            ("applied_force_n", 2.4),
            ("force_before_final_clamp_n", 1.2),
            ("relay_force_n", 1.2),
            ("spring_force_n", 0.9),
            ("damping_force_n", 0.9),
            ("auxiliary_force_n", 0.9),
        ):
            ax.plot(tr, series(rows, f"{prefix}_{axis}"), linewidth=width,
                    label=("ACTUAL DEVICE-WRITE applied_force_n" if prefix == "applied_force_n" else prefix))
        ax.axhline(max_force_n, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
        ax.axhline(-max_force_n, color="black", linewidth=0.8, linestyle="--", alpha=0.6)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("force (N)")
        ax.set_title(f"{side} Cartesian force components {axis}")
        ax.legend(loc="best", fontsize=8)
        add_grid(ax)
        fig.tight_layout()
        fig.savefig(out / f"{side.lower()}_force_{axis}.png", dpi=160)
        plt.close(fig)


def save_causal_plot(
    bilateral: list[dict],
    receiver: list[dict],
    out: Path,
    base: float,
    *,
    channel: str,
    source_prefix: str,
    cmd_prefix: str,
    receiver_name: str,
):
    tb, tr = times(bilateral, base), times(receiver, base)
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=False)
    for joint in JOINTS:
        axes[0].plot(tb, series(bilateral, f"{source_prefix}_{joint}"), label=joint)
    axes[0].set_ylabel("source DOB torque (N*m)")
    axes[0].legend(loc="best", fontsize=8)
    add_grid(axes[0])
    for joint in JOINTS:
        axes[1].plot(tb, series(bilateral, f"{cmd_prefix}_{joint}"), label=joint)
    axes[1].set_ylabel("commanded relay torque (N*m)")
    axes[1].legend(loc="best", fontsize=8)
    add_grid(axes[1])
    for axis in AXES:
        axes[2].plot(tr, series(receiver, f"relay_force_n_{axis}"), label=axis)
    axes[2].set_ylabel(f"{receiver_name} relay force (N)")
    axes[2].legend(loc="best", fontsize=8)
    add_grid(axes[2])
    for axis in AXES:
        axes[3].plot(tr, series(receiver, f"applied_force_n_{axis}"), label=axis)
    axes[3].set_ylabel(f"{receiver_name} applied force (N)")
    axes[3].set_xlabel("t (s)")
    axes[3].legend(loc="best", fontsize=8)
    add_grid(axes[3])
    axes[0].set_title(f"{channel} force causal chain: N*m panels kept separate from N panels")
    fig.tight_layout()
    fig.savefig(out / f"{channel.lower()}_force_causal_chain.png", dpi=160)
    plt.close(fig)


def save_clamp_state_plot(master: list[dict], slave: list[dict], bridge: list[dict],
                          bilateral: list[dict], out: Path, base: float):
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
    if bridge:
        tb = times(bridge, base)
        states = [r.get("bridge_state", "") for r in bridge]
        uniq = sorted(set(states))
        idx = {state: i for i, state in enumerate(uniq)}
        axes[0].step(tb, [idx[s] for s in states], where="post")
        axes[0].set_yticks(list(idx.values()))
        axes[0].set_yticklabels(list(idx.keys()), fontsize=7)
        axes[0].set_ylabel("bridge")
    if bilateral:
        tbil = times(bilateral, base)
        for i, name in enumerate(("C1", "C2", "C3", "C4")):
            axes[1].step(tbil, [1 if b(r, f"channel_active_{i}") else 0 for r in bilateral],
                         where="post", label=name)
        axes[1].set_ylabel("active")
        axes[1].legend(loc="best", ncol=4, fontsize=8)
    for rows, ax, name in ((master, axes[2], "master"), (slave, axes[3], "slave")):
        tr = times(rows, base)
        for key in ("position_error_clamped", "primary_force_clamped", "final_force_clamped"):
            ax.step(tr, [1 if b(r, key) else 0 for r in rows], where="post", label=key)
        ax.set_ylabel(name)
        ax.legend(loc="best", fontsize=8)
    axes[-1].set_xlabel("t (s)")
    for ax in axes:
        add_grid(ax)
    axes[0].set_title("Clamp and state timeline")
    fig.tight_layout()
    fig.savefig(out / "clamp_state_timeline.png", dpi=160)
    plt.close(fig)


def save_force_budget_plot(bridge: list[dict], out: Path, base: float):
    tb = times(bridge, base)
    fig, ax = plt.subplots(figsize=(11, 4))
    for key in ("master_accepted_force_budget_margin_n", "slave_accepted_force_budget_margin_n",
                "master_candidate_force_budget_margin_n", "slave_candidate_force_budget_margin_n"):
        ax.plot(tb, series(bridge, key), label=key)
    ax.axhline(0.0, color="black", linewidth=0.9)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("margin (N)")
    ax.set_title("Force budget margin")
    ax.legend(loc="best", fontsize=8)
    add_grid(ax)
    fig.tight_layout()
    fig.savefig(out / "force_budget_margin.png", dpi=160)
    plt.close(fig)


def tracking_metrics(metrics: dict) -> dict:
    directional = metrics.get("directional", {})
    out = {}
    for channel in ("c2", "c3"):
        m = directional.get(channel, {}).get("metrics", {})
        out[channel] = {
            "correlated_gain": m.get("positive_motion_gain"),
            "rmse_m": m.get("tracking_error_rmse_m"),
            "peak_error_m": m.get("tracking_error_peak_m"),
            "positive_lag_s": m.get("lag_positive_motion_s"),
            "return_lag_s": m.get("lag_return_s"),
            "source_return_residual_m": m.get("source_return_residual_m"),
            "follower_return_residual_m": m.get("follower_return_residual_m"),
        }
    return out


def force_channel_metrics(bilateral: list[dict], receiver: list[dict], *,
                          source_prefix: str, cmd_prefix: str,
                          expected_cmd_sign: int) -> dict:
    source_norm = [norm3(r, source_prefix, JOINTS) for r in bilateral]
    cmd_norm = [norm3(r, cmd_prefix, JOINTS) for r in bilateral]
    relay_norm = [norm3(r, "relay_force_n") for r in receiver]
    applied_norm = [norm3(r, "applied_force_n") for r in receiver]
    baseline = mean(applied_norm[:max(1, min(len(applied_norm), 100))]) or 0.0
    applied_delta = [v - baseline for v in applied_norm]
    sign = {}
    for joint in JOINTS:
        cval = corr(series(bilateral, f"{source_prefix}_{joint}"),
                    series(bilateral, f"{cmd_prefix}_{joint}"))
        if cval is None:
            status = "not_evaluable"
        elif expected_cmd_sign < 0:
            status = "correct" if cval < -0.25 else "incorrect_or_weak"
        else:
            status = "correct" if cval > 0.25 else "incorrect_or_weak"
        sign[joint] = {"source_to_command_correlation": cval, "status": status}
    return {
        "source_disturbance_nm": {"peak_norm": peak_abs(source_norm), "rms_norm": rms(source_norm)},
        "commanded_relay_torque_nm": {"peak_norm": peak_abs(cmd_norm), "rms_norm": rms(cmd_norm)},
        "receiving_relay_force_n": {"peak_norm": peak_abs(relay_norm), "rms_norm": rms(relay_norm)},
        "receiving_applied_force_change_n": {"peak_norm": peak_abs(applied_delta), "rms_norm": rms(applied_delta)},
        "source_norm_to_relay_force_norm_correlation": corr(source_norm, relay_norm),
        "relay_valid_duty": duty(receiver, "relay_valid"),
        "relay_fresh_duty": duty(receiver, "relay_fresh"),
        "relay_applied_duty": duty(receiver, "relay_applied"),
        "sign_correctness": sign,
    }


def safety_metrics(master: list[dict], slave: list[dict], bridge: list[dict]) -> dict:
    def side(rows: list[dict]) -> dict:
        before_norm = [norm3(r, "force_before_final_clamp_n") for r in rows]
        applied_norm = [norm3(r, "applied_force_n") for r in rows]
        return {
            "clamp_duties": bool_duties(rows, [
                "position_error_clamped", "primary_force_clamped", "final_force_clamped"]),
            "final_force_clamp": {
                **clamp_runs(rows, "final_force_clamped"),
                "max_force_before_final_clamp_norm_n": peak_abs(before_norm),
                "max_applied_force_norm_n": peak_abs(applied_norm),
            },
        }
    return {
        "master": side(master),
        "slave": side(slave),
        "min_force_budget_margin_n": {
            "master_accepted": min(finite(series(bridge, "master_accepted_force_budget_margin_n")), default=None),
            "slave_accepted": min(finite(series(bridge, "slave_accepted_force_budget_margin_n")), default=None),
            "master_candidate": min(finite(series(bridge, "master_candidate_force_budget_margin_n")), default=None),
            "slave_candidate": min(finite(series(bridge, "slave_candidate_force_budget_margin_n")), default=None),
        },
    }


def classify_slave_final_clamp(safety: dict) -> str:
    fc = safety.get("slave", {}).get("final_force_clamp", {})
    count = fc.get("count", 0)
    duty_v = fc.get("duty") or 0.0
    longest = fc.get("longest_s") or 0.0
    if count == 0:
        return "disappeared"
    if duty_v <= 0.02 and longest <= 0.5:
        return "brief"
    return "persistent"


def verdict(validation: dict, safety: dict) -> str:
    overall = validation.get("overall_result")
    if overall != "VALID":
        return "FAIL" if overall == "INVALID" else "INCONCLUSIVE"
    slave_clamp = classify_slave_final_clamp(safety)
    if slave_clamp == "disappeared":
        return "PASS"
    return "PASS WITH COMMISSIONING LIMITATION"


def write_summary(path: Path, session_dir: Path, trial_dir: Path, metrics: dict):
    lines = [
        "# Full 4-Channel Current Setup Baseline",
        "",
        f"- Session: `{session_dir.name}`",
        f"- Trial: `{trial_dir}`",
        f"- Verdict: **{metrics['verdict']}**",
        f"- Channel verdict: C1={metrics['channel_verdict']['c1']}, "
        f"C2={metrics['channel_verdict']['c2']}, C3={metrics['channel_verdict']['c3']}, "
        f"C4={metrics['channel_verdict']['c4']}",
        f"- Prior slave final-force clamp status: **{metrics['prior_slave_final_force_clamp_status']}**",
        "",
        "## Position Tracking",
        json.dumps(metrics["position_tracking"], indent=2),
        "",
        "## Force Channels",
        json.dumps(metrics["force_channels"], indent=2),
        "",
        "## Safety",
        json.dumps(metrics["safety"], indent=2),
        "",
        "## Plots",
        "- `position_x.png`, `position_y.png`, `position_z.png`",
        "- `position_tracking_error_c2_c3.png`",
        "- `master_force_x.png`, `master_force_y.png`, `master_force_z.png`",
        "- `slave_force_x.png`, `slave_force_y.png`, `slave_force_z.png`",
        "- `c1_force_causal_chain.png`, `c4_force_causal_chain.png`",
        "- `clamp_state_timeline.png`, `force_budget_margin.png`",
    ]
    path.write_text("\n".join(lines) + "\n")


def analyze(session_dir: Path, trial_dir: Path) -> dict:
    out = session_dir / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    master = read_csv(trial_dir / "raw_master.csv")
    slave = read_csv(trial_dir / "raw_slave.csv")
    bridge = read_csv(trial_dir / "raw_bridge.csv")
    bilateral = read_csv(trial_dir / "raw_bilateral.csv")
    base = t0(master, slave, bridge, bilateral)

    metrics_existing = load_json(trial_dir / "metrics.json", {})
    validation = load_json(trial_dir / "validation.json", {})
    manifest = load_json(trial_dir / "manifest.json", {})

    save_position_plots(master, slave, out, base)
    save_tracking_error_plot(bridge, out, base)
    save_force_component_plots(master, "master", 2.0, out, base)
    save_force_component_plots(slave, "slave", 1.5, out, base)
    save_causal_plot(bilateral, master, out, base, channel="C1",
                     source_prefix="tau_e_hat_nm", cmd_prefix="master_torque_cmd_nm",
                     receiver_name="master")
    save_causal_plot(bilateral, slave, out, base, channel="C4",
                     source_prefix="tau_h_hat_nm", cmd_prefix="slave_torque_cmd_nm",
                     receiver_name="slave")
    save_clamp_state_plot(master, slave, bridge, bilateral, out, base)
    save_force_budget_plot(bridge, out, base)

    safety = safety_metrics(master, slave, bridge)
    position = tracking_metrics(metrics_existing)
    force_channels = {
        "c1_slave_force_to_master": force_channel_metrics(
            bilateral, master, source_prefix="tau_e_hat_nm",
            cmd_prefix="master_torque_cmd_nm", expected_cmd_sign=-1),
        "c4_master_force_to_slave": force_channel_metrics(
            bilateral, slave, source_prefix="tau_h_hat_nm",
            cmd_prefix="slave_torque_cmd_nm", expected_cmd_sign=1),
    }
    iso = validation.get("channel_isolation", {})
    channel_verdict = {
        "c1": "PASS" if iso.get("expected", {}).get("c1") and not iso.get("unexpected_activation") else "INCONCLUSIVE",
        "c2": "PASS" if manifest.get("expected_channel_configuration", {}).get("c2") else "INCONCLUSIVE",
        "c3": "PASS" if manifest.get("expected_channel_configuration", {}).get("c3") else "INCONCLUSIVE",
        "c4": "PASS" if iso.get("expected", {}).get("c4") and not iso.get("unexpected_activation") else "INCONCLUSIVE",
    }
    result = {
        "schema_version": "full4ch-baseline-1",
        "session": session_dir.name,
        "trial_dir": str(trial_dir),
        "recorder_manifest": manifest,
        "analyzer_validation": validation,
        "position_tracking": position,
        "force_channels": force_channels,
        "safety": safety,
        "channel_verdict": channel_verdict,
        "prior_slave_final_force_clamp_status": classify_slave_final_clamp(safety),
    }
    result["verdict"] = verdict(validation, safety)
    (out / "metrics.json").write_text(json.dumps(result, indent=2, default=str))
    write_summary(out / "FULL4CH_TEST_SUMMARY.md", session_dir, trial_dir, result)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--session-dir", required=True)
    ap.add_argument("--trial-dir", required=True)
    args = ap.parse_args()
    result = analyze(Path(args.session_dir), Path(args.trial_dir))
    print(json.dumps({
        "verdict": result["verdict"],
        "analysis_dir": str(Path(args.session_dir) / "analysis"),
        "prior_slave_final_force_clamp_status": result["prior_slave_final_force_clamp_status"],
    }, indent=2))


if __name__ == "__main__":
    main()
