#!/usr/bin/env python3
"""H3/H4 manual-motion analysis (2026-09-10). Pure post-hoc: reads the CSVs
+ manifest.json written by record_manual_motion.py, computes a merged
dataset, detected events, metrics, and a validation verdict, and writes
plots. Publishes/commands nothing, requires no ROS import at all -- every
function below operates on plain lists of dicts, so it is directly unit-
testable without rclpy or a live sim (see tests/test_manual_motion_analysis.py).

Anchor-relative Y convention (section 7): "master_y" means the MAPPED,
anchor-relative master Y (mapped_rel_y from raw_master.csv, i.e.
FROZEN.axis_map/axis_scale already applied once by the recorder);
    "slave_y" means the slave's own NATIVE anchor-relative Y (rel_y from
raw_slave.csv). The analyzer is mode-aware: C3/C4 use master as source and
slave as follower; C1/C2 use slave as source and master as follower.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def _read_csv(path: Path) -> list:
    if not path.exists():
        return []
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        row = {}
        for k, v in r.items():
            if v == "" or v is None:
                row[k] = None
            elif v in ("True", "False"):
                row[k] = (v == "True")
            else:
                try:
                    row[k] = float(v)
                except ValueError:
                    row[k] = v
        out.append(row)
    return out


# ---------------------------------------------------------------------------
# Section 6: nearest-neighbor join. Documented rule: join master (backbone)
# to the nearest slave/bridge/bilateral row within JOIN_TOLERANCE_S of
# monotonic receive time; a stream with no row inside that window at a
# given master sample gets None for every one of its columns at that row
# ("no measurement", never a fabricated zero -- section 6's explicit
# requirement).
# ---------------------------------------------------------------------------

JOIN_TOLERANCE_S = 0.05  # ~5 control ticks at 100Hz; no stricter rule is
                          # specified by TELEMETRY_CONTRACT.md, so this is
                          # an analysis parameter, not a frozen value.

AXES = ("x", "y", "z")
DIRECTIONS = {
    "c1": {"source_role": "slave", "follower_role": "master"},
    "c2": {"source_role": "slave", "follower_role": "master"},
    "c3": {"source_role": "master", "follower_role": "slave"},
    "c4": {"source_role": "master", "follower_role": "slave"},
}


def normalize_analysis_mode(manifest: dict) -> str:
    """Return the validation mode from recorder metadata.

    Older H3/H4 manifests predate explicit mode names; their expected channel
    configuration is authoritative when available.
    """
    stage = str(manifest.get("expected_stage", "")).strip().lower().replace("_", "-")
    if stage in ("c1-only", "c2-only", "c3-only", "c4-only", "c2c3", "full4ch"):
        return stage
    if stage in ("h3", "h4"):
        return "c3-only"
    expected = manifest.get("expected_channel_configuration", {})
    c2 = bool(expected.get("c2", False))
    c3 = bool(expected.get("c3", False))
    if c2 and c3:
        return "c2c3"
    if c2:
        return "c2-only"
    if bool(expected.get("c1", False)):
        return "c1-only"
    if bool(expected.get("c4", False)):
        return "c4-only"
    return "c3-only"


def primary_direction_for_mode(mode: str) -> str:
    if mode in ("c1-only", "c2-only"):
        return "c1" if mode == "c1-only" else "c2"
    if mode == "c4-only":
        return "c4"
    return "c3"


def _direction_keys(direction: str, axis: str = "y") -> dict:
    if direction not in DIRECTIONS:
        raise ValueError(f"unknown direction {direction!r}; expected one of {sorted(DIRECTIONS)}")
    if axis not in AXES:
        raise ValueError(f"unknown axis {axis!r}; expected one of {AXES}")
    spec = dict(DIRECTIONS[direction])
    spec["axis"] = axis
    spec["source_key"] = f"{spec['source_role']}_{axis}"
    spec["follower_key"] = f"{spec['follower_role']}_{axis}"
    return spec


def _nearest(rows: list, t: float, tol: float) -> Optional[dict]:
    if not rows:
        return None
    best, best_dt = None, None
    for r in rows:
        dt = abs(r["t_recv_monotonic"] - t)
        if best_dt is None or dt < best_dt:
            best, best_dt = r, dt
    if best is not None and best_dt <= tol:
        return best
    return None


def merge_streams(master_rows: list, slave_rows: list, bridge_rows: list,
                  bilateral_rows: list, tol: float = JOIN_TOLERANCE_S) -> list:
    merged = []
    for m in master_rows:
        t = m["t_recv_monotonic"]
        s = _nearest(slave_rows, t, tol)
        b = _nearest(bridge_rows, t, tol)
        bi = _nearest(bilateral_rows, t, tol)
        row = {"t": t, "t_ros_master": m.get("t_ros")}
        row["master_y"] = m.get("mapped_rel_y")
        row["master_x"] = m.get("mapped_rel_x")
        row["master_z"] = m.get("mapped_rel_z")
        row["master_applied_force_n_y"] = m.get("applied_force_n_y")
        row["master_enabled"] = m.get("enabled")
        row["master_deadman_ok"] = m.get("deadman_ok")
        row["master_final_force_clamped"] = m.get("final_force_clamped")
        row["slave_y"] = s.get("rel_y") if s is not None else None
        row["slave_x"] = s.get("rel_x") if s is not None else None
        row["slave_z"] = s.get("rel_z") if s is not None else None
        row["slave_applied_force_n_y"] = s.get("applied_force_n_y") if s is not None else None
        row["slave_final_force_clamped"] = s.get("final_force_clamped") if s is not None else None
        row["slave_available"] = s is not None
        row["bridge_state"] = b.get("bridge_state") if b is not None else None
        row["c1_enabled"] = b.get("c1_enabled") if b is not None else None
        row["c2_enabled"] = b.get("c2_enabled") if b is not None else None
        row["c3_enabled"] = b.get("c3_enabled") if b is not None else None
        row["c4_enabled"] = b.get("c4_enabled") if b is not None else None
        row["bridge_available"] = b is not None
        for i in range(4):
            row[f"channel_active_{i}"] = bi.get(f"channel_active_{i}") if bi is not None else None
        row["safety_state"] = bi.get("safety_state") if bi is not None else None
        row["bilateral_available"] = bi is not None
        merged.append(row)
    return merged


# ---------------------------------------------------------------------------
# Section 8/9: event detection. Pure state/threshold logic over merged rows'
# master_y series -- no fixed/guessed timestamps.
# ---------------------------------------------------------------------------

NOISE_BAND_M = 0.0005     # 0.5mm -- below this is "at rest", an analysis
                          # parameter (no frozen/hardware-validated noise
                          # floor exists yet -- see OFFSITE_REMAINING_WORK.md
                          # item 5's Touch-device-noise caveat).
MOTION_THRESHOLD_M = 0.001  # 1mm sustained excursion counts as motion start
SUSTAIN_SAMPLES = 3        # consecutive samples above/below threshold
RETURN_DROP_FRACTION = 0.3  # peak must drop by this fraction of its own
                            # value before RETURN_START is declared (avoids
                            # calling ordinary hold-phase jitter a return)
MIN_EXCITATION_M = 0.001   # below this peak excursion, gain/lag are N/A


def detect_events(merged: list, source_key: str = "master_y",
                  source_label: str = "master", axis: str = "y") -> dict:
    """Returns {"events": [{"name", "index", "t"}...], "insufficient_excitation": bool,
    "wrong_direction": bool}. Never classifies pure noise as motion (SUSTAIN_SAMPLES
    consecutive samples required, not a single-sample crossing)."""
    ys = [r.get(source_key) for r in merged]
    ts = [r["t"] for r in merged]
    n = len(ys)
    events = []

    if n == 0:
        return {"events": [], "insufficient_excitation": True,
               "insufficient_excitation_reason": "no samples", "wrong_direction": False,
               "source_key": source_key, "source_label": source_label, "axis": axis}

    # BASELINE: first index, by definition (start of recording).
    events.append({"name": "BASELINE", "index": 0, "t": ts[0]})

    def _sustained_above(i, thresh):
        if i + SUSTAIN_SAMPLES > n:
            return False
        return all(ys[j] is not None and abs(ys[j]) > thresh for j in range(i, i + SUSTAIN_SAMPLES))

    motion_start_idx = None
    for i in range(n):
        if ys[i] is None:
            continue
        if _sustained_above(i, MOTION_THRESHOLD_M):
            motion_start_idx = i
            break

    peak_abs = max((abs(y) for y in ys if y is not None), default=0.0)
    if motion_start_idx is None or peak_abs < MIN_EXCITATION_M:
        return {"events": events, "insufficient_excitation": True,
               "insufficient_excitation_reason":
                   f"peak |{source_key}|={peak_abs*1000:.4f}mm < {MIN_EXCITATION_M*1000:.1f}mm threshold, "
                   "or no sustained motion detected", "wrong_direction": False,
               "source_key": source_key, "source_label": source_label, "axis": axis}

    events.append({"name": "MOTION_START", "index": motion_start_idx, "t": ts[motion_start_idx]})

    # Direction: sign of the peak (max abs value) after motion start.
    valid = [(i, ys[i]) for i in range(motion_start_idx, n) if ys[i] is not None]
    peak_idx, peak_val = max(valid, key=lambda p: abs(p[1]))
    wrong_direction = peak_val < 0  # H3's intended excitation is +Y

    events.append({"name": "POSITIVE_PEAK" if not wrong_direction else "NEGATIVE_PEAK",
                   "index": peak_idx, "t": ts[peak_idx], "value_m": peak_val})

    # HOLD: contiguous region around the peak where |y| stays within
    # HOLD_FRACTION of the peak's own magnitude -- a VALUE-based definition
    # (not a derivative/velocity threshold): a smooth, slow, finely-sampled
    # manual motion has near-zero per-sample velocity everywhere near a
    # raised-cosine ramp's own start/end (by construction, velocity->0 at
    # both ends of a C1-continuous ramp), so a velocity-threshold approach
    # cannot distinguish "genuinely holding at the target" from "passing
    # slowly through an inflection on the way there" -- confirmed by an
    # earlier version of this function misclassifying an entire outward
    # ramp as HOLD for exactly that reason. Proximity to the peak VALUE has
    # no such ambiguity.
    HOLD_FRACTION = 0.85
    hold_thresh = abs(peak_val) * HOLD_FRACTION
    hold_lo, hold_hi = peak_idx, peak_idx
    for i in range(peak_idx, motion_start_idx, -1):
        if ys[i - 1] is None or abs(ys[i - 1]) < hold_thresh:
            break
        hold_lo = i - 1
    for i in range(peak_idx, n - 1):
        if ys[i + 1] is None or abs(ys[i + 1]) < hold_thresh:
            break
        hold_hi = i + 1
    events.append({"name": "HOLD", "index_start": hold_lo, "index_end": hold_hi,
                   "t_start": ts[hold_lo], "t_end": ts[hold_hi]})

    # RETURN_START: first index after the peak where |y| has dropped by
    # RETURN_DROP_FRACTION of the peak's own magnitude.
    return_start_idx = None
    drop_target = abs(peak_val) * (1.0 - RETURN_DROP_FRACTION)
    for i in range(hold_hi, n):
        if ys[i] is None:
            continue
        if abs(ys[i]) < drop_target:
            return_start_idx = i
            break
    if return_start_idx is not None:
        events.append({"name": "RETURN_START", "index": return_start_idx, "t": ts[return_start_idx]})

        return_complete_idx = None
        for i in range(return_start_idx, n):
            if _sustained_above(i, NOISE_BAND_M):
                continue
            if i + SUSTAIN_SAMPLES <= n and all(
                ys[j] is not None and abs(ys[j]) <= NOISE_BAND_M
                for j in range(i, min(i + SUSTAIN_SAMPLES, n))
            ):
                return_complete_idx = i
                break
        if return_complete_idx is not None:
            events.append({"name": "RETURN_COMPLETE", "index": return_complete_idx,
                           "t": ts[return_complete_idx]})

    return {"events": events, "insufficient_excitation": False, "wrong_direction": wrong_direction,
            "source_key": source_key, "source_label": source_label, "axis": axis}


# ---------------------------------------------------------------------------
# Section 9/10/11: metrics
# ---------------------------------------------------------------------------

def _find_event(events: list, name: str) -> Optional[dict]:
    for e in events:
        if e["name"] == name:
            return e
    return None


def compute_metrics(merged: list, event_result: dict, direction: str = "c3",
                    axis: str = "y") -> dict:
    events = event_result["events"]
    metrics: dict = {}
    spec = _direction_keys(direction, axis)
    source_key = spec["source_key"]
    follower_key = spec["follower_key"]
    source_role = spec["source_role"]
    follower_role = spec["follower_role"]
    metrics["analysis_direction"] = direction
    metrics["source_role"] = source_role
    metrics["follower_role"] = follower_role
    metrics["axis"] = axis

    if event_result["insufficient_excitation"]:
        metrics["result"] = "NOT_EVALUABLE"
        metrics["reason"] = event_result.get("insufficient_excitation_reason", "insufficient excitation")
        return metrics

    motion_start = _find_event(events, "MOTION_START")
    peak = _find_event(events, "POSITIVE_PEAK") or _find_event(events, "NEGATIVE_PEAK")
    ret_start = _find_event(events, "RETURN_START")
    ret_complete = _find_event(events, "RETURN_COMPLETE")

    ms_idx, pk_idx = motion_start["index"], peak["index"]
    outward = [r for r in merged[ms_idx:pk_idx + 1]
               if r.get(source_key) is not None and r.get(follower_key) is not None]

    source_peak_y = peak["value_m"]
    follower_peak_y = None
    if outward:
        follower_vals = [r[follower_key] for r in outward]
        follower_peak_y = max(follower_vals, key=abs)
    metrics["source_peak_y_m"] = source_peak_y
    metrics["follower_peak_y_m"] = follower_peak_y
    metrics["master_peak_y_m"] = max((r["master_y"] for r in outward
                                      if r.get("master_y") is not None), key=abs, default=None)
    metrics["slave_peak_y_m"] = max((r["slave_y"] for r in outward
                                     if r.get("slave_y") is not None), key=abs, default=None)

    # Positive-motion gain.
    if follower_peak_y is not None and abs(source_peak_y) > 1e-9:
        metrics["positive_motion_gain"] = follower_peak_y / source_peak_y
    else:
        metrics["positive_motion_gain"] = None
        metrics["positive_motion_gain_na_reason"] = (
            f"no synchronized {follower_role} samples during outward phase")

    # Tracking error over the whole excited region (motion_start..end).
    tail = merged[ms_idx:]
    errs = [r[source_key] - r[follower_key] for r in tail
            if r.get(source_key) is not None and r.get(follower_key) is not None]
    if errs:
        metrics["tracking_error_peak_m"] = max(abs(e) for e in errs)
        metrics["tracking_error_rmse_m"] = math.sqrt(statistics.fmean([e * e for e in errs]))
    else:
        metrics["tracking_error_peak_m"] = None
        metrics["tracking_error_rmse_m"] = None

    # Lag (cross-correlation), outward phase only -- needs enough samples.
    def _lag(rows):
        rows = [r for r in rows if r.get(source_key) is not None and r.get(follower_key) is not None]
        if len(rows) < 6:
            return None, "insufficient synchronized samples for a lag estimate"
        ts_ = [r["t"] for r in rows]
        dt_est = (ts_[-1] - ts_[0]) / max(1, len(rows) - 1)
        if dt_est <= 0:
            return None, "non-monotonic or degenerate timestamps"
        m_ = [r[source_key] for r in rows]
        s_ = [r[follower_key] for r in rows]
        m_mean, s_mean = statistics.fmean(m_), statistics.fmean(s_)
        m_c = [v - m_mean for v in m_]
        s_c = [v - s_mean for v in s_]
        max_lag = min(len(rows) // 2, int(1.0 / dt_est))
        if max_lag < 1:
            return None, "window too short for a lag search"
        best_lag, best_corr = 0, -1e18
        for lag in range(0, max_lag):
            num = sum(m_c[i] * s_c[i + lag] for i in range(len(rows) - lag))
            if num > best_corr:
                best_corr, best_lag = num, lag
        return best_lag * dt_est, None

    lag_out, lag_out_na = _lag(outward)
    metrics["lag_positive_motion_s"] = lag_out
    if lag_out is None:
        metrics["lag_positive_motion_na_reason"] = lag_out_na

    if ret_start is not None:
        end_idx = ret_complete["index"] if ret_complete is not None else len(merged) - 1
        ret_rows = merged[ret_start["index"]:end_idx + 1]
        lag_ret, lag_ret_na = _lag(ret_rows)
        metrics["lag_return_s"] = lag_ret
        if lag_ret is None:
            metrics["lag_return_na_reason"] = lag_ret_na

        ret_valid = [r for r in ret_rows if r.get(follower_key) is not None]
        if ret_valid:
            s_ret_peak = max(ret_valid, key=lambda r: abs(r[follower_key]))[follower_key]
            if follower_peak_y is not None and abs(follower_peak_y) > 1e-9:
                metrics["return_motion_gain"] = 1.0 - (abs(s_ret_peak) / abs(follower_peak_y))
            metrics["master_return_residual_m"] = merged[end_idx]["master_y"]
            metrics["slave_return_residual_m"] = merged[end_idx]["slave_y"]
            metrics["source_return_residual_m"] = merged[end_idx].get(source_key)
            metrics["follower_return_residual_m"] = merged[end_idx].get(follower_key)
        if follower_peak_y is not None and follower_peak_y != 0:
            overshoot_vals = [r[follower_key] / follower_peak_y for r in ret_rows
                              if r.get(follower_key) is not None]
            metrics["overshoot_ratio"] = max(overshoot_vals) - 1.0 if overshoot_vals else None
        if ret_complete is not None:
            metrics["settling_time_s"] = ret_complete["t"] - ret_start["t"]
        else:
            metrics["settling_time_s"] = None
            metrics["settling_time_na_reason"] = "RETURN_COMPLETE never detected within the recording window"
    else:
        metrics["lag_return_s"] = None
        metrics["lag_return_na_reason"] = "no RETURN_START detected"
        metrics["return_motion_gain"] = None
        metrics["settling_time_s"] = None
        metrics["settling_time_na_reason"] = "no RETURN_START detected"

    # Section 10: cross-axis coupling.
    excited = [r for r in merged[ms_idx:] if r.get(source_key) is not None]
    m_peak_x = max((abs(r["master_x"]) for r in excited if r["master_x"] is not None), default=None)
    m_peak_z = max((abs(r["master_z"]) for r in excited if r["master_z"] is not None), default=None)
    s_peak_x = max((abs(r["slave_x"]) for r in excited if r["slave_x"] is not None), default=None)
    s_peak_z = max((abs(r["slave_z"]) for r in excited if r["slave_z"] is not None), default=None)
    denom = abs(source_peak_y) if source_peak_y else None
    source_peak_x = max((abs(r[f"{source_role}_x"]) for r in excited
                         if r.get(f"{source_role}_x") is not None), default=None)
    source_peak_z = max((abs(r[f"{source_role}_z"]) for r in excited
                         if r.get(f"{source_role}_z") is not None), default=None)
    metrics["cross_axis"] = {
        "master_peak_x_m": m_peak_x, "master_peak_z_m": m_peak_z,
        "slave_peak_x_m": s_peak_x, "slave_peak_z_m": s_peak_z,
        "source_x_over_y_ratio": (source_peak_x / denom) if (source_peak_x is not None and denom) else None,
        "source_z_over_y_ratio": (source_peak_z / denom) if (source_peak_z is not None and denom) else None,
        "master_x_over_y_ratio": (m_peak_x / denom) if (m_peak_x is not None and denom) else None,
        "master_z_over_y_ratio": (m_peak_z / denom) if (m_peak_z is not None and denom) else None,
        "slave_x_over_y_ratio": (s_peak_x / abs(metrics["slave_peak_y_m"]))
        if (s_peak_x is not None and metrics["slave_peak_y_m"]) else None,
        "slave_z_over_y_ratio": (s_peak_z / abs(metrics["slave_peak_y_m"]))
        if (s_peak_z is not None and metrics["slave_peak_y_m"]) else None,
    }

    metrics["result"] = "EVALUATED"
    return metrics


# ---------------------------------------------------------------------------
# Section 10b (2026-09-10, full4ch pre-tuning closure Task 3): actual
# pre-final-clamp force headroom -- a GROUND-TRUTH diagnostic, deliberately
# separate from bridge_master_slave.py's own `*_accepted_force_budget_
# margin_n` (published on /bridge_diagnostics).
#
# Traced from source (coupling_force_budget.py, `grep -n "relay"` returns
# ZERO matches): that margin is `(max_force_n - force_reserve_n) -
# |predict_force_vector(...)|`, where predict_force_vector sums ONLY
# spring + damping + auxiliary -- it is a POSITION-CHANNEL-ONLY prediction
# computed BEFORE the C1/C4 relay force (added later, inside
# touch_slave.cpp's own Cartesian controller) is even known. It cannot see
# C1/C4 at all. This is why the 2026-09-10 full4ch baseline showed
# slave_accepted_force_budget_margin_n =~ +0.85N (a healthy POSITION-ONLY
# prediction) at the exact session where force_before_final_clamp_n
# reached 1.719N against the real 1.5N ceiling -- the two numbers answer
# different questions. See CURRENT_FULL4CH_AUDIT.md Part 9.
#
# This function computes the ACTUAL, ground-truth pre-final-clamp
# headroom directly from the device-write force path
# (force_before_final_clamp_n, the value AFTER spring+damping+aux+relay
# are all combined but BEFORE the final hard clamp) against the
# configured max_force_n for that arm. Conceptually this is the
# "position_path_force_margin" (the planner's own metric) vs.
# "actual_preclamp_force_headroom_n" (this one) distinction the audit
# requires -- two different, both real, quantities. Does NOT reconstruct
# applied_force_n (the actual clamped device-write value remains
# force_before_final_clamp_n before clamping / applied_force_n after);
# this function only subtracts a known scalar ceiling from an already-
# real per-sample vector already present in the raw telemetry.
def actual_preclamp_force_headroom(rows: list, prefix: str, max_force_n: Optional[float]) -> dict:
    """`rows` = raw_master.csv or raw_slave.csv rows (as dicts). `prefix`
    is 'master' or 'slave' (for labeling only). `max_force_n` is the
    configured ceiling for this arm for the WHOLE session (a launch-time
    constant, not itself recorded per-sample by record_manual_motion.py
    -- must be supplied from the session manifest.env, e.g.
    MASTER_MAX_FORCE_N / the slave's own LIMITS field)."""
    if max_force_n is None:
        return {"result": "NOT_EVALUABLE", "reason": "max_force_n not supplied (not recorded per-sample)"}
    norms = []
    min_headroom = None
    min_headroom_row = None
    for r in rows:
        fx = _num(r.get("force_before_final_clamp_n_x"))
        fy = _num(r.get("force_before_final_clamp_n_y"))
        fz = _num(r.get("force_before_final_clamp_n_z"))
        if fx is None or fy is None or fz is None:
            continue
        norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        norms.append(norm)
        headroom = max_force_n - norm
        if min_headroom is None or headroom < min_headroom:
            min_headroom = headroom
            min_headroom_row = r
    if not norms:
        return {"result": "NOT_EVALUABLE", "reason": "no force_before_final_clamp_n samples available"}
    return {
        "result": "EVALUATED",
        "side": prefix,
        "max_force_n": max_force_n,
        "peak_preclamp_force_norm_n": max(norms),
        "min_actual_preclamp_headroom_n": min_headroom,
        "min_headroom_final_force_clamped": _bool(min_headroom_row.get("final_force_clamped"))
        if min_headroom_row is not None else None,
        "note": "ground truth (force_before_final_clamp_n vs configured max_force_n) -- "
                "NOT the planner's position-only predicted margin",
    }


def _num(v) -> Optional[float]:
    if v is None or v == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def _bool(v) -> Optional[bool]:
    if v is None or v == "":
        return None
    return str(v).strip().lower() in ("true", "1")


# ---------------------------------------------------------------------------
# Section 11 (force/DOB) + 12 (channel isolation) + 13 (safety/invalid)
# ---------------------------------------------------------------------------

def force_dob_analysis(merged: list, event_result: dict) -> dict:
    out: dict = {}
    if event_result["insufficient_excitation"]:
        out["result"] = "NOT_EVALUABLE"
        out["reason"] = "no sufficient position excitation on the excited axis to attribute force behavior to"
        return out
    rows = [r for r in merged if r["master_applied_force_n_y"] is not None]
    if not rows:
        out["result"] = "NOT_EVALUABLE"
        out["reason"] = "no master applied-force samples available"
        return out
    out["result"] = "EVALUATED"
    out["master_peak_applied_force_n_y"] = max(abs(r["master_applied_force_n_y"]) for r in rows)
    srows = [r for r in merged if r["slave_applied_force_n_y"] is not None]
    out["slave_peak_applied_force_n_y"] = (max(abs(r["slave_applied_force_n_y"]) for r in srows)
                                           if srows else None)
    out["master_final_force_clamp_seen"] = any(bool(r["master_final_force_clamped"]) for r in merged
                                                if r["master_final_force_clamped"] is not None)
    out["slave_final_force_clamp_seen"] = any(bool(r["slave_final_force_clamped"]) for r in merged
                                              if r["slave_final_force_clamped"] is not None)
    return out


# 2026-09-10 fix (full4ch pre-tuning closure, Task 4): C2/C3's TRUE
# runtime owner is bridge_master_slave.py (its own c2_enabled/c3_enabled
# fields in BridgeDiagnostics) -- bilateral_4ch.py's C2/C3 joint-space
# weights are deliberately held at 0 in the supervised full4ch
# architecture (see ARCHITECTURE_FREEZE_4CH.md), so bilateral_status's
# channel_active_1/channel_active_2 will ALWAYS read inactive for C2/C3
# in this architecture regardless of whether the bridge's real C2/C3
# position coupling is working. Confirmed live on the
# 20260910T155604Z_full4ch_current_setup_baseline session: real
# bidirectional position tracking (C2 gain 1.08, C3 gain 0.92) alongside
# a validator report of "missing_expected_activation: C2, C3" -- a
# validator bug, not a masking/architecture defect (see
# CURRENT_FULL4CH_AUDIT.md Part 8). C1/C4 remain correctly owned by
# bilateral_status (that IS where they are actually implemented).
CHANNEL_ACTIVITY_SOURCE = {"c1": "bilateral_status", "c2": "bridge_diagnostics",
                          "c3": "bridge_diagnostics", "c4": "bilateral_status"}
CHANNEL_BILATERAL_INDEX = {"c1": 0, "c2": 1, "c3": 2, "c4": 3}


def channel_isolation_check(merged: list, expected: dict) -> dict:
    """Section 12: reports CONFIGURED gate state (from bridge rows) and LIVE
    channel_active state SEPARATELY -- never conflated.

    Two asymmetric questions, answered from the correct source for each:

    - "is an EXPECTED channel really active?" (missing_expected_activation)
      is answered ONLY by that channel's true owner (bridge_diagnostics for
      C2/C3, bilateral_status for C1/C4) -- asking the non-owner would
      always read "inactive" for C2/C3 in this architecture even when the
      bridge's real position coupling is working (the exact bug this fix
      addresses; see CURRENT_FULL4CH_AUDIT.md Part 8).
    - "is an UNEXPECTED channel accidentally active?" (unexpected_activation)
      is a SAFETY check and must look at BOTH possible sources for C2/C3:
      the bridge's own configured state AND bilateral_4ch's channel_active,
      because bilateral_4ch.py also HAS a (normally-zeroed) C2/C3
      implementation -- if it ever reported active, that would be a real
      duplicate-position-authority danger the bridge-only check alone
      would miss. C1/C4 only ever have the one (bilateral_status) source.

    `live_channel_active_available` stays specifically about bilateral_
    status presence (used by validate_trial's MISSING_BILATERAL_STATUS
    check, which is a C1/C4-specific requirement) -- it is NOT redefined
    to mean "any channel's source is available", which would silently
    defeat that check whenever bridge rows happen to be present.
    """
    bridge_rows = [r for r in merged if r["bridge_available"]]
    result = {
        "configured": None, "expected": expected,
        "configured_matches_expected": None,
        "live_channel_active_available": False,
        "unexpected_activation": None,
    }
    configured = None
    if bridge_rows:
        last = bridge_rows[-1]
        configured = {"c1": bool(last["c1_enabled"]), "c2": bool(last["c2_enabled"]),
                     "c3": bool(last["c3_enabled"]), "c4": bool(last["c4_enabled"])}
        result["configured"] = configured
        result["configured_matches_expected"] = (configured == {k: bool(v) for k, v in expected.items()})

    bilateral_rows = [r for r in merged if r["bilateral_available"]]
    result["live_channel_active_available"] = bool(bilateral_rows)

    chan_names = ["c1", "c2", "c3", "c4"]
    unexpected = []
    missing_expected = []

    def _bilateral_ever_active(name: str) -> bool:
        i = CHANNEL_BILATERAL_INDEX[name]
        return any(r.get(f"channel_active_{i}") for r in bilateral_rows)

    for name in chan_names:
        owner = CHANNEL_ACTIVITY_SOURCE[name]
        owner_active = (configured.get(name, False) if (owner == "bridge_diagnostics" and configured is not None)
                        else (_bilateral_ever_active(name) if (owner == "bilateral_status" and bilateral_rows)
                              else None))

        # Positive confirmation: owner-only.
        if expected.get(name, False) and owner_active is False:
            missing_expected.append({"channel": name})

        # Negative safety check: any KNOWN source reporting active while
        # not expected is unexpected, regardless of which source it is.
        if not expected.get(name, False):
            if owner_active:
                unexpected.append({"channel": name, "owner": owner})
            if owner != "bilateral_status" and bilateral_rows and _bilateral_ever_active(name):
                unexpected.append({"channel": name, "owner": "bilateral_status (non-owner, duplicate authority)"})

    result["unexpected_activation"] = unexpected if unexpected else None
    result["missing_expected_activation"] = missing_expected if missing_expected else None
    return result


TELEMETRY_STALE_THRESHOLD_S = 0.2  # FROZEN.force_budget_telemetry_stale_s -- the
                                   # one frozen/documented freshness threshold that
                                   # generalizes to "how old is too old" here.


def validate_trial(merged: list, event_result: dict, isolation: dict, stream_stats: dict) -> dict:
    flags = []
    thresholds_not_defined = []

    bridge_rows = [r for r in merged if r["bridge_available"]]
    if any(r["bridge_state"] == "FAULT" for r in bridge_rows):
        flags.append({"code": "BRIDGE_FAULT", "severity": "INVALID",
                      "detail": "bridge_state==FAULT observed during the trial"})

    if any(r["master_final_force_clamped"] for r in merged if r["master_final_force_clamped"] is not None) or \
       any(r["slave_final_force_clamped"] for r in merged if r["slave_final_force_clamped"] is not None):
        flags.append({"code": "FINAL_FORCE_CLAMP", "severity": "FLAG",
                      "detail": "final_force_clamped True on at least one arm during the trial"})

    for name, stats in stream_stats.items():
        if not stats.get("available"):
            continue
        max_gap = stats.get("max_gap_s")
        if max_gap is not None and max_gap > TELEMETRY_STALE_THRESHOLD_S:
            flags.append({"code": "TELEMETRY_STALE", "severity": "FLAG",
                          "detail": f"{name} stream max gap {max_gap:.3f}s > "
                                    f"{TELEMETRY_STALE_THRESHOLD_S}s threshold"})

    if isolation.get("unexpected_activation"):
        flags.append({"code": "UNEXPECTED_CHANNEL_ACTIVATION", "severity": "INVALID",
                      "detail": isolation["unexpected_activation"]})
    force_channels_expected = bool(isolation.get("expected", {}).get("c1")) or \
        bool(isolation.get("expected", {}).get("c4"))
    if force_channels_expected and not isolation.get("live_channel_active_available"):
        flags.append({"code": "MISSING_BILATERAL_STATUS", "severity": "INVALID",
                      "detail": "C1/C4 expected but no bilateral_status channel-active samples were recorded"})
    missing_force_activation = [
        item for item in (isolation.get("missing_expected_activation") or [])
        if item.get("channel") in ("c1", "c4")
    ]
    if missing_force_activation:
        flags.append({"code": "EXPECTED_CHANNEL_NOT_ACTIVE", "severity": "INVALID",
                      "detail": missing_force_activation})
    if isolation.get("configured_matches_expected") is False:
        flags.append({"code": "CONFIGURED_CHANNEL_MISMATCH", "severity": "INVALID",
                      "detail": f"configured={isolation['configured']} expected={isolation['expected']}"})

    if event_result.get("wrong_direction"):
        flags.append({"code": "WRONG_DIRECTION_MOTION", "severity": "INVALID",
                      "detail": "peak excursion was in the negative-Y direction, expected +Y"})

    events = event_result.get("events", [])
    if (not event_result["insufficient_excitation"] and _find_event(events, "MOTION_START") is not None
            and _find_event(events, "RETURN_START") is None):
        flags.append({"code": "FAILURE_TO_RETURN", "severity": "INVALID",
                      "detail": "motion was detected but the master never dropped back toward baseline"})

    thresholds_not_defined.append("excessive_signal_dropout_count (no frozen threshold exists)")
    thresholds_not_defined.append("excessive_cross_axis_motion_ratio (no frozen threshold exists)")
    thresholds_not_defined.append("master_deadman_loss_during_force_stage (deadman_ok tracked per-sample "
                                  "in raw_master.csv; no frozen minimum-hold-fraction threshold exists)")

    if event_result["insufficient_excitation"]:
        overall = "INSUFFICIENT_EXCITATION"
    elif any(f["severity"] == "INVALID" for f in flags):
        overall = "INVALID_TRIAL"
    else:
        overall = "VALID"

    return {
        "overall_result": overall,
        "flags": flags,
        "channel_isolation": isolation,
        "thresholds_used": {"telemetry_stale_s": TELEMETRY_STALE_THRESHOLD_S,
                            "noise_band_m": NOISE_BAND_M,
                            "motion_threshold_m": MOTION_THRESHOLD_M,
                            "min_excitation_m": MIN_EXCITATION_M},
        "thresholds_not_defined": thresholds_not_defined,
    }


# ---------------------------------------------------------------------------
# Plots (section 16)
# ---------------------------------------------------------------------------

def make_plots(merged: list, event_result: dict, plot_dir: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plot_dir.mkdir(parents=True, exist_ok=True)
    if not merged:
        return
    ts = [r["t"] - merged[0]["t"] for r in merged]

    def _mark_events(ax):
        for e in event_result.get("events", []):
            if "t" in e:
                ax.axvline(e["t"] - merged[0]["t"], color="gray", linestyle=":", linewidth=0.8)
                ax.annotate(e["name"], (e["t"] - merged[0]["t"], ax.get_ylim()[1]),
                           rotation=90, fontsize=6, va="top", color="gray")

    def _series(key):
        return [r[key] if r[key] is not None else float("nan") for r in merged]

    # 1. master Y vs slave Y vs time
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, _series("master_y"), label="master Y (mapped, anchor-rel)")
    ax.plot(ts, _series("slave_y"), label="slave Y (native, anchor-rel)")
    ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H3/H4: master vs slave Y")
    _mark_events(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "01_master_vs_slave_y.png", dpi=150); plt.close(fig)

    # 2. tracking error Y
    err = [(m - s) if (m is not None and s is not None) else float("nan")
          for m, s in zip(_series("master_y"), _series("slave_y"))]
    fig, ax = plt.subplots(figsize=(10, 3))
    ax.plot(ts, err); ax.set_ylabel("master_y - slave_y (m)"); ax.set_xlabel("t (s)")
    ax.set_title("H3/H4: Y tracking error"); _mark_events(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "02_tracking_error_y.png", dpi=150); plt.close(fig)

    # 3. master XYZ
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, _series("master_x"), label="x"); ax.plot(ts, _series("master_y"), label="y")
    ax.plot(ts, _series("master_z"), label="z")
    ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H3/H4: master XYZ (anchor-rel, mapped)")
    _mark_events(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "03_master_xyz.png", dpi=150); plt.close(fig)

    # 4. slave XYZ
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, _series("slave_x"), label="x"); ax.plot(ts, _series("slave_y"), label="y")
    ax.plot(ts, _series("slave_z"), label="z")
    ax.set_ylabel("m"); ax.set_xlabel("t (s)"); ax.legend(); ax.set_title("H3/H4: slave XYZ (anchor-rel, native)")
    _mark_events(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "04_slave_xyz.png", dpi=150); plt.close(fig)

    # 5. applied vs DOB predicted force -- no per-axis DOB-predicted-force
    # field exists on CartesianControllerTelemetry (only relay_force_n, the
    # ALREADY-composed relay contribution) -- plot applied vs relay instead
    # and label it accurately rather than claim a DOB prediction that was
    # never recorded (section 11: "do not invent a new interpretation").
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, _series("master_applied_force_n_y"), label="master applied_y")
    ax.plot(ts, _series("slave_applied_force_n_y"), label="slave applied_y")
    ax.set_ylabel("N"); ax.set_xlabel("t (s)"); ax.legend()
    ax.set_title("H3/H4: applied force Y (no separate DOB-prediction field exists in "
                "CartesianControllerTelemetry -- see relay_force_n for the C1/C4 "
                "contribution already summed into applied_force_n)")
    _mark_events(ax)
    fig.tight_layout(); fig.savefig(plot_dir / "05_applied_force_y.png", dpi=150); plt.close(fig)

    # 6-10 are best-effort from what merge_streams currently carries; full
    # per-axis spring/damping/aux/relay decomposition and the deadman/
    # dropout timelines are written directly from the raw per-stream rows
    # by write_outputs() below (it has access to the raw rows, this
    # function only sees the merged/backbone view).


def make_extra_plots(master_rows: list, bridge_rows: list, bilateral_rows: list, plot_dir: Path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if master_rows:
        t0 = master_rows[0]["t_recv_monotonic"]
        ts = [r["t_recv_monotonic"] - t0 for r in master_rows]
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(ts, [r["spring_force_n_y"] for r in master_rows], label="spring_y")
        ax.plot(ts, [r["damping_force_n_y"] for r in master_rows], label="damping_y")
        ax.plot(ts, [r["auxiliary_force_n_y"] for r in master_rows], label="auxiliary_y")
        ax.plot(ts, [r["relay_force_n_y"] for r in master_rows], label="relay_y")
        ax.plot(ts, [r["applied_force_n_y"] for r in master_rows], label="applied_y (final)")
        ax.set_ylabel("N"); ax.set_xlabel("t (s)"); ax.legend(fontsize=7)
        ax.set_title("H3/H4: master force decomposition (Y)")
        fig.tight_layout(); fig.savefig(plot_dir / "06_force_decomposition_master.png", dpi=150); plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(ts, [1 if r["enabled"] else 0 for r in master_rows], label="enabled", drawstyle="steps-post")
        ax.plot(ts, [1 if r["deadman_ok"] else 0 for r in master_rows], label="deadman_ok", drawstyle="steps-post")
        ax.set_ylim(-0.1, 1.1); ax.set_xlabel("t (s)"); ax.legend()
        ax.set_title("H3/H4: master enable/deadman timeline")
        fig.tight_layout(); fig.savefig(plot_dir / "09_deadman_enable_timeline.png", dpi=150); plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(ts, [r["dropout_gap_s"] for r in master_rows])
        ax.set_ylabel("gap since previous sample (s)"); ax.set_xlabel("t (s)")
        ax.set_title("H3/H4: master signal-age / dropout timeline")
        fig.tight_layout(); fig.savefig(plot_dir / "10_dropout_timeline.png", dpi=150); plt.close(fig)

    if bilateral_rows:
        t0 = bilateral_rows[0]["t_recv_monotonic"]
        ts = [r["t_recv_monotonic"] - t0 for r in bilateral_rows]
        fig, ax = plt.subplots(figsize=(10, 3))
        for i, name in enumerate(["C1", "C2", "C3", "C4"]):
            ax.plot(ts, [1 if r[f"channel_active_{i}"] else 0 for r in bilateral_rows],
                   label=name, drawstyle="steps-post")
        ax.set_ylim(-0.1, 1.1); ax.set_xlabel("t (s)"); ax.legend()
        ax.set_title("H3/H4: channel-active timeline")
        fig.tight_layout(); fig.savefig(plot_dir / "07_channel_active_timeline.png", dpi=150); plt.close(fig)
    else:
        print("[analyze] no bilateral_status samples -- skipping channel-active timeline "
              "(C1/C4 may not be running, which is expected for a C3-only H3 trial)")

    if bridge_rows:
        t0 = bridge_rows[0]["t_recv_monotonic"]
        ts = [r["t_recv_monotonic"] - t0 for r in bridge_rows]
        states = [r["bridge_state"] for r in bridge_rows]
        uniq = sorted(set(states))
        idx = {s: i for i, s in enumerate(uniq)}
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.step(ts, [idx[s] for s in states], where="post")
        ax.set_yticks(range(len(uniq))); ax.set_yticklabels(uniq, fontsize=7)
        ax.set_xlabel("t (s)"); ax.set_title("H3/H4: bridge/safety-state timeline")
        fig.tight_layout(); fig.savefig(plot_dir / "08_bridge_state_timeline.png", dpi=150); plt.close(fig)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def analyze(out_dir: Path) -> dict:
    manifest = json.loads((out_dir / "manifest.json").read_text()) if (out_dir / "manifest.json").exists() else {}
    master_rows = _read_csv(out_dir / "raw_master.csv")
    slave_rows = _read_csv(out_dir / "raw_slave.csv")
    bridge_rows = _read_csv(out_dir / "raw_bridge.csv")
    bilateral_rows = _read_csv(out_dir / "raw_bilateral.csv")

    merged = merge_streams(master_rows, slave_rows, bridge_rows, bilateral_rows)
    analysis_mode = normalize_analysis_mode(manifest)
    primary_direction = primary_direction_for_mode(analysis_mode)
    primary_spec = _direction_keys(primary_direction)
    event_result = detect_events(
        merged, source_key=primary_spec["source_key"],
        source_label=primary_spec["source_role"], axis=primary_spec["axis"])
    metrics = compute_metrics(merged, event_result, direction=primary_direction)
    metrics["analysis_mode"] = analysis_mode
    metrics["directional"] = {}
    for direction in ("c1", "c2", "c3", "c4"):
        spec = _direction_keys(direction)
        ev = detect_events(
            merged, source_key=spec["source_key"],
            source_label=spec["source_role"], axis=spec["axis"])
        direction_metrics = compute_metrics(merged, ev, direction=direction)
        metrics["directional"][direction] = {
            "event_result": ev,
            "metrics": direction_metrics,
        }
    force = force_dob_analysis(merged, event_result)
    metrics["force_dob"] = force
    expected = manifest.get("expected_channel_configuration",
                            {"c1": False, "c2": False, "c3": True, "c4": False})
    isolation = channel_isolation_check(merged, expected)
    stream_stats = manifest.get("stream_stats", {})
    validation = validate_trial(merged, event_result, isolation, stream_stats)

    with open(out_dir / "merged.csv", "w", newline="") as f:
        if merged:
            import csv as _csv
            w = _csv.DictWriter(f, fieldnames=list(merged[0].keys()))
            w.writeheader()
            w.writerows(merged)
    (out_dir / "events.json").write_text(json.dumps(event_result, indent=2, default=str))
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2, default=str))
    (out_dir / "validation.json").write_text(json.dumps(validation, indent=2, default=str))

    plot_dir = out_dir / "plots"
    make_plots(merged, event_result, plot_dir)
    make_extra_plots(master_rows, bridge_rows, bilateral_rows, plot_dir)

    return {"event_result": event_result, "metrics": metrics, "validation": validation}


def main():
    out_dir = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    result = analyze(out_dir)
    overall = result["validation"]["overall_result"]
    print(json.dumps({"overall_result": overall,
                      "metrics_summary": {k: v for k, v in result["metrics"].items()
                                         if not isinstance(v, dict)}}, indent=2, default=str))
    sys.exit(0 if overall == "VALID" else 1)


if __name__ == "__main__":
    main()
