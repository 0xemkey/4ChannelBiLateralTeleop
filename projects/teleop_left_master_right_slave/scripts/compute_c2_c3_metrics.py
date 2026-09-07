#!/usr/bin/env python3
"""C2+C3 bilateral position-only diagnostic -- segmented numerical metrics.

Reads a log_phase2_scenario.py CSV captured with scenario
"c2_bilateral_position" (see msg/BridgeDiagnostics.msg for the source
fields) and reports, per phase (A/B/C/D/E) and per axis, the metrics the
task asked for: RMSE, MAE, peak absolute error, mean signed error,
Pearson correlation, estimated lag, and 95th-percentile absolute error --
for both C2 (master tracking, c2_position_error_*) and C3 (slave
tracking, c3_position_error_*) directions independently, NOT one blended
40s number.

Segment boundaries default to the scenario's own nominal schedule (A 0-5s,
B 5-15s, C 15-20s, D 20-30s, E 30-40s) but are overridable, since the task
explicitly says to infer/annotate true timing if the actual run differs
from the nominal schedule rather than assume it blindly.

No ROS dependency -- CSV + stdlib + optional numpy fallback to pure
Python if numpy is unavailable, matching plot_phase2_scenario.py's own
dependency-free convention (this script avoids even matplotlib).

USAGE:
    python3 compute_c2_c3_metrics.py capture.csv
    python3 compute_c2_c3_metrics.py capture.csv --out-json summary.json --out-txt summary.txt
    python3 compute_c2_c3_metrics.py capture.csv --segments 0,6,16,21,31,41
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

AXES = ("x", "y", "z")

# Nominal schedule from the c2_bilateral_position scenario description
# (log_phase2_scenario.py SCENARIOS dict): A 0-5, B 5-15, C 15-20, D 20-30,
# E 30-40. Expressed as boundaries between 6 edges for 5 segments.
DEFAULT_SEGMENT_BOUNDARIES = [0.0, 5.0, 15.0, 20.0, 30.0, 40.0]
SEGMENT_NAMES = ["A_baseline", "B_master_driven", "C_baseline2", "D_slave_driven", "E_mixed"]


def load_csv(path: Path) -> dict:
    cols: dict = {}
    with open(path) as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        raw: dict = {name: [] for name in fieldnames}
        for row in reader:
            for name in fieldnames:
                raw[name].append(row[name])
    for name, values in raw.items():
        try:
            cols[name] = [float(v) for v in values]
        except ValueError:
            cols[name] = values
    return cols


def _segment_indices(t: list, boundaries: list) -> list:
    """Returns a list of (name, [indices]) per segment, clipped to what's
    actually in the recording -- a run shorter than 40s just gets fewer/
    shorter segments rather than an out-of-range crash, and the txt report
    says so explicitly (task: infer/annotate true intervals, don't assume)."""
    out = []
    n_seg = len(boundaries) - 1
    names = SEGMENT_NAMES if n_seg == len(SEGMENT_NAMES) else [f"seg{i}" for i in range(n_seg)]
    for i in range(n_seg):
        lo, hi = boundaries[i], boundaries[i + 1]
        idxs = [j for j, tv in enumerate(t) if lo <= tv < hi]
        out.append((names[i], idxs))
    return out


def _mean(xs):
    return sum(xs) / len(xs) if xs else float("nan")


def _std(xs):
    if len(xs) < 2:
        return float("nan")
    m = _mean(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def _percentile(xs, p):
    if not xs:
        return float("nan")
    s = sorted(xs)
    k = (len(s) - 1) * (p / 100.0)
    f, c = math.floor(k), math.ceil(k)
    if f == c:
        return s[int(k)]
    return s[f] + (s[c] - s[f]) * (k - f)


def _pearson(a, b):
    if len(a) < 2 or len(b) < 2:
        return float("nan")
    ma, mb = _mean(a), _mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((y - mb) ** 2 for y in b))
    if da == 0 or db == 0:
        return float("nan")
    return num / (da * db)


def _estimate_lag_s(t, target, actual, max_lag_s=1.0):
    """Cross-correlation lag estimate: how much to shift `actual` forward in
    time to best match `target`. Positive = actual lags target. Coarse
    (uses the recording's own median dt as the search step) -- good enough
    to flag "no meaningful lag" vs "~100ms-class lag", not a precision tool.
    """
    n = len(t)
    if n < 4:
        return float("nan")
    dts = [t[i + 1] - t[i] for i in range(n - 1) if t[i + 1] > t[i]]
    if not dts:
        return float("nan")
    dt = _mean(dts)
    if dt <= 0:
        return float("nan")
    max_shift = max(1, int(round(max_lag_s / dt)))
    best_shift, best_corr = 0, -2.0
    for shift in range(0, max_shift + 1):
        if shift >= n:
            break
        a = target[shift:]
        b = actual[: n - shift]
        if len(a) < 2:
            continue
        c = _pearson(a, b)
        if not math.isnan(c) and c > best_corr:
            best_corr, best_shift = c, shift
    return best_shift * dt


def axis_metrics(t_seg, err_seg, target_seg=None, actual_seg=None) -> dict:
    abs_err = [abs(e) for e in err_seg]
    out = {
        "n_samples": len(err_seg),
        "rmse_m": math.sqrt(_mean([e * e for e in err_seg])) if err_seg else float("nan"),
        "mae_m": _mean(abs_err),
        "peak_abs_error_m": max(abs_err) if abs_err else float("nan"),
        "mean_signed_error_m": _mean(err_seg),
        "p95_abs_error_m": _percentile(abs_err, 95),
    }
    if target_seg is not None and actual_seg is not None and len(target_seg) >= 2:
        out["correlation"] = _pearson(target_seg, actual_seg)
        out["estimated_lag_s"] = _estimate_lag_s(t_seg, target_seg, actual_seg)
    else:
        out["correlation"] = float("nan")
        out["estimated_lag_s"] = float("nan")
    return out


def compute(cols: dict, boundaries: list) -> dict:
    t = cols["t_sync"]
    n = len(t)
    segments = _segment_indices(t, boundaries)

    result = {"segments": {}, "overall": {}}

    def channel_block(idxs, err_prefix, target_prefix, actual_prefix):
        block = {}
        for ax in AXES:
            err_col = f"{err_prefix}_{ax}"
            if err_col not in cols:
                continue
            err_seg = [cols[err_col][i] for i in idxs]
            t_seg = [t[i] for i in idxs]
            target_seg = actual_seg = None
            tcol, acol = f"{target_prefix}_{ax}", f"{actual_prefix}_{ax}"
            if tcol in cols and acol in cols:
                target_seg = [cols[tcol][i] for i in idxs]
                actual_seg = [cols[acol][i] for i in idxs]
            block[ax] = axis_metrics(t_seg, err_seg, target_seg, actual_seg)
        return block

    def alignment_block(idxs):
        if "alignment_error_norm" not in cols or not idxs:
            return {}
        vals = [cols["alignment_error_norm"][i] for i in idxs]
        return {"mean_m": _mean(vals), "max_m": max(vals), "std_m": _std(vals)}

    def flags_block(idxs):
        blk = {}
        for col, key in [
            ("master_travel_limit_flag", "master_travel_limit_count"),
            ("slave_travel_limit_flag", "slave_travel_limit_count"),
            ("master_force_saturation_flag", "master_saturation_count"),
            ("slave_force_saturation_flag", "slave_saturation_count"),
        ]:
            if col in cols and idxs:
                blk[key] = int(sum(1 for i in idxs if cols[col][i]))
                blk[key.replace("count", "pct")] = 100.0 * blk[key] / len(idxs)
        return blk

    def state_block(idxs):
        if "bridge_state" not in cols or not idxs:
            return {}
        states = [cols["bridge_state"][i] for i in idxs]
        n_active = sum(1 for s in states if s == "ACTIVE")
        n_fault = sum(1 for s in states if s == "FAULT")
        return {
            "active_pct": 100.0 * n_active / len(states),
            "fault_count": n_fault,
        }

    def coupling_block(idxs, err_prefix):
        """Cross-axis coupling: for each driven axis' error, ratio of the
        OTHER axes' error magnitude to that axis' own -- flags whether
        moving one axis bleeds into the others."""
        if not idxs:
            return {}
        mags = {}
        for ax in AXES:
            col = f"{err_prefix}_{ax}"
            if col in cols:
                mags[ax] = _mean([abs(cols[col][i]) for i in idxs])
        if len(mags) < 2:
            return {}
        out = {}
        for ax in AXES:
            if ax not in mags or mags[ax] == 0:
                continue
            others = _mean([mags[o] for o in AXES if o != ax and o in mags])
            out[f"{ax}_others_over_self_ratio"] = others / mags[ax] if mags[ax] else float("nan")
        return out

    def sample_rate_block(idxs):
        if len(idxs) < 2:
            return {}
        ts = [t[i] for i in idxs]
        dts = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        return {
            "sample_dt_mean_s": _mean(dts),
            "sample_dt_std_s": _std(dts),
            "max_sample_gap_s": max(dts) if dts else float("nan"),
        }

    for name, idxs in segments:
        result["segments"][name] = {
            "n_samples": len(idxs),
            "t_range_s": [t[idxs[0]], t[idxs[-1]]] if idxs else None,
            "c2": channel_block(idxs, "c2_position_error",
                                 "master_target", "master"),
            "c3": channel_block(idxs, "c3_position_error",
                                 "slave_target", "slave"),
            "alignment_error_norm": alignment_block(idxs),
            "flags": flags_block(idxs),
            "bridge_state": state_block(idxs),
            "c2_cross_axis_coupling": coupling_block(idxs, "c2_position_error"),
            "c3_cross_axis_coupling": coupling_block(idxs, "c3_position_error"),
            "sample_rate": sample_rate_block(idxs),
        }

    all_idxs = list(range(n))
    result["overall"] = {
        "n_samples": n,
        "t_range_s": [t[0], t[-1]] if n else None,
        "c2": channel_block(all_idxs, "c2_position_error", "master_target", "master"),
        "c3": channel_block(all_idxs, "c3_position_error", "slave_target", "slave"),
        "alignment_error_norm": alignment_block(all_idxs),
        "flags": flags_block(all_idxs),
        "bridge_state": state_block(all_idxs),
        "sample_rate": sample_rate_block(all_idxs),
    }
    return result


def format_txt(result: dict, csv_name: str) -> str:
    lines = [f"C2+C3 bilateral position-only diagnostic -- metrics for {csv_name}", ""]

    def fmt_axis(ax, m):
        return (f"      {ax}: RMSE={m['rmse_m']*1000:.3f}mm  MAE={m['mae_m']*1000:.3f}mm  "
                f"peak={m['peak_abs_error_m']*1000:.3f}mm  mean_signed={m['mean_signed_error_m']*1000:.3f}mm  "
                f"p95={m['p95_abs_error_m']*1000:.3f}mm  corr={m['correlation']:.3f}  "
                f"lag={m['estimated_lag_s']*1000:.1f}ms" if m['n_samples'] else f"      {ax}: no samples")

    for scope in ["overall"] + list(result["segments"].keys()):
        blk = result["overall"] if scope == "overall" else result["segments"][scope]
        lines.append(f"== {scope} ==  n={blk['n_samples']}  t_range={blk['t_range_s']}")
        if blk["n_samples"] == 0:
            lines.append("  (no samples in this segment -- schedule may not match actual run)")
            lines.append("")
            continue
        lines.append("  C2 (master tracking, slave->master):")
        for ax in AXES:
            if ax in blk["c2"]:
                lines.append(fmt_axis(ax, blk["c2"][ax]))
        lines.append("  C3 (slave tracking, master->slave):")
        for ax in AXES:
            if ax in blk["c3"]:
                lines.append(fmt_axis(ax, blk["c3"][ax]))
        ae = blk.get("alignment_error_norm", {})
        if ae:
            lines.append(f"  alignment_error_norm: mean={ae['mean_m']*1000:.3f}mm "
                          f"max={ae['max_m']*1000:.3f}mm std={ae['std_m']*1000:.3f}mm")
        fl = blk.get("flags", {})
        if fl:
            lines.append(f"  flags: {fl}")
        bs = blk.get("bridge_state", {})
        if bs:
            lines.append(f"  bridge_state: ACTIVE={bs['active_pct']:.1f}%  FAULT_count={bs['fault_count']}")
        sr = blk.get("sample_rate", {})
        if sr:
            lines.append(f"  sample_rate: mean_dt={sr['sample_dt_mean_s']*1000:.1f}ms "
                          f"std_dt={sr['sample_dt_std_s']*1000:.1f}ms "
                          f"max_gap={sr['max_sample_gap_s']*1000:.1f}ms")
        c2c = blk.get("c2_cross_axis_coupling", {})
        c3c = blk.get("c3_cross_axis_coupling", {})
        if c2c:
            lines.append(f"  c2_cross_axis_coupling: {c2c}")
        if c3c:
            lines.append(f"  c3_cross_axis_coupling: {c3c}")
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv_file", type=Path)
    p.add_argument("--segments", type=str, default=None,
                    help="Comma-separated segment boundaries in seconds, e.g. "
                         "0,5,15,20,30,40 (default). Override if the actual run's "
                         "phase timing differed from the nominal schedule.")
    p.add_argument("--out-json", type=Path, default=None)
    p.add_argument("--out-txt", type=Path, default=None)
    args = p.parse_args()

    boundaries = DEFAULT_SEGMENT_BOUNDARIES
    if args.segments:
        boundaries = [float(x) for x in args.segments.split(",")]

    cols = load_csv(args.csv_file)
    if not cols.get("t_sync"):
        print(f"[compute_c2_c3_metrics] {args.csv_file.name}: no t_sync column / empty file", file=sys.stderr)
        sys.exit(1)

    result = compute(cols, boundaries)
    txt = format_txt(result, args.csv_file.name)
    print(txt)

    out_json = args.out_json or args.csv_file.with_suffix(".metrics.json")
    out_txt = args.out_txt or args.csv_file.with_suffix(".metrics.txt")
    out_json.write_text(json.dumps(result, indent=2))
    out_txt.write_text(txt)
    print(f"[compute_c2_c3_metrics] wrote {out_json} and {out_txt}")


if __name__ == "__main__":
    main()
