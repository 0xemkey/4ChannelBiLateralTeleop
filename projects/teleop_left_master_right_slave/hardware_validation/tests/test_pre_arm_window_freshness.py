"""2026-09-10 real-hardware finding regression tests: evaluate_pre_arm_samples()'s
window_start_t / decision_time_t mechanism.

Real-hardware evidence (session 20260910T102922Z_h2): PRE_ARM_FAIL with
left/right max_gap ~0.96s, bridge max_gap ~0.01s, 389/388/20 samples
respectively. Root-caused via direct sim measurement (not assumed): the
bridge-startup wait loop (`bridge_proc.stdout.readline()`, blocking, no
rclpy.spin_once()) starved telemetry callbacks for the ENTIRE bridge
startup duration (measured 1.566s in one sim run, matching the real
gap's magnitude), and node.rows[...] accumulated from node-creation time
onward with no reset before being handed to evaluate_pre_arm_samples --
so this old startup-phase gap (even after the readline fix, in principle
any other pre-window event) could poison a decision that should only be
about "is telemetry fresh right now, in a bounded recent window."

Two independent fixes, tested here at the pure-function level:
  1. window_start_t: any row predating it is flagged as evidence the
     caller's window reset did not happen (defense in depth -- the
     primary fix is part1_gravity_settle_capture.py's own
     node.rows[...].clear() immediately after the bridge reports ready).
  2. decision_time_t: a LAST-SAMPLE-AGE check, distinct from max
     inter-sample gap -- a window can have uniformly small gaps
     throughout yet still be stale if nothing arrived between the last
     sample and the decision moment.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pre_arm_check import evaluate_pre_arm_samples  # noqa: E402

FORCE_TOL = 0.05
STALE_S = 0.2


def _telem_row(t, enabled=False, applied=0.0, clamped=False):
    return {"t": t, "enabled": enabled,
           "applied_force_n_x": 0.0, "applied_force_n_y": applied, "applied_force_n_z": 0.0,
           "position_error_clamped": clamped, "primary_force_clamped": False, "final_force_clamped": False}


def _bridge_row(t, state="READY", coupling=0.0, c1=False, c2=False, c3=False, c4=False):
    return {"t": t, "bridge_state": state, "coupling_ramp_alpha": coupling,
           "c1_enabled": c1, "c2_enabled": c2, "c3_enabled": c3, "c4_enabled": c4}


# ---------------------------------------------------------------------------
# Item 1: a historical gap (e.g. bridge startup) BEFORE window_start_t,
# followed by a clean fresh window -- must PASS. This is the exact
# real-hardware scenario, reproduced with synthetic timestamps.
# ---------------------------------------------------------------------------

def test_historical_startup_gap_before_window_then_clean_window_passes():
    # Simulate: 5 samples right after node creation (t=0..0.02), THEN a
    # ~1.5s startup gap (bridge-ready wait), THEN window_start_t, THEN a
    # clean 20-sample window with normal 10ms cadence.
    pre_window = [_telem_row(i * 0.005) for i in range(5)]  # t=0, .005, .01, .015, .02
    window_start_t = 1.55  # after the ~1.53s startup gap
    fresh_window = [_telem_row(window_start_t + i * 0.01) for i in range(20)]  # t=1.55..1.74
    left = pre_window + fresh_window
    right = pre_window + fresh_window
    bridge = [_bridge_row(window_start_t + i * 0.01) for i in range(20)]
    decision_time_t = fresh_window[-1]["t"]  # evaluated right as the window ends

    # Sanity: this data genuinely DOES contain a >0.2s historical gap if
    # naively evaluated whole -- confirms the test reproduces the real bug
    # shape before checking the fix.
    naive = evaluate_pre_arm_samples(left, right, bridge, FORCE_TOL, STALE_S, decision_time_t=decision_time_t)
    assert naive["status"] == "PRE_ARM_FAIL", "test setup did not reproduce the historical-gap bug shape"

    # The FIX: only hand the function rows >= window_start_t (mirrors
    # part1_gravity_settle_capture.py's node.rows[...].clear() approach)
    # AND pass window_start_t for the defense-in-depth check.
    windowed_left = [r for r in left if r["t"] >= window_start_t]
    windowed_right = [r for r in right if r["t"] >= window_start_t]
    windowed_bridge = [r for r in bridge if r["t"] >= window_start_t]
    result = evaluate_pre_arm_samples(windowed_left, windowed_right, windowed_bridge,
                                      FORCE_TOL, STALE_S, window_start_t=window_start_t,
                                      decision_time_t=decision_time_t)
    assert result["status"] == "PRE_ARM_PASS", result["reasons"]


def test_rows_predating_window_start_are_flagged_defense_in_depth():
    """If the CALLER forgets to actually clear/slice (a real regression
    this defense-in-depth check would catch even if the primary fix were
    ever reverted), rows with t < window_start_t must be flagged."""
    window_start_t = 1.0
    left = [_telem_row(0.5)] + [_telem_row(window_start_t + i * 0.01) for i in range(20)]
    right = [_telem_row(window_start_t + i * 0.01) for i in range(20)]
    bridge = [_bridge_row(window_start_t + i * 0.01) for i in range(20)]
    decision_time_t = left[-1]["t"]
    result = evaluate_pre_arm_samples(left, right, bridge, FORCE_TOL, STALE_S,
                                      window_start_t=window_start_t, decision_time_t=decision_time_t)
    assert result["status"] == "PRE_ARM_FAIL"
    assert any("predate the pre-arm window start" in r for r in result["reasons"])


# ---------------------------------------------------------------------------
# Item 2: a >0.2s gap INSIDE the dedicated window must still FAIL --
# proves the fix does not weaken genuine in-window staleness detection.
# ---------------------------------------------------------------------------

def test_gap_inside_dedicated_window_still_fails():
    window_start_t = 1.0
    rows = [_telem_row(window_start_t + i * 0.01) for i in range(10)]
    rows += [_telem_row(rows[-1]["t"] + 0.5)]  # 0.5s gap, inside the window
    rows += [_telem_row(rows[-1]["t"] + i * 0.01) for i in range(1, 10)]
    bridge = [_bridge_row(window_start_t + i * 0.01) for i in range(20)]
    decision_time_t = rows[-1]["t"]
    result = evaluate_pre_arm_samples(rows, rows, bridge, FORCE_TOL, STALE_S,
                                      window_start_t=window_start_t, decision_time_t=decision_time_t)
    assert result["status"] == "PRE_ARM_FAIL"
    assert any("telemetry gap" in r and "inside the evaluated window" in r for r in result["reasons"])


# ---------------------------------------------------------------------------
# Item 3: last sample older than stale_s at DECISION time must fail even
# if every historical inter-sample gap was small -- proves max-gap and
# last-sample-age are genuinely independent checks.
# ---------------------------------------------------------------------------

def test_stale_last_sample_at_decision_time_fails_despite_small_historical_gaps():
    window_start_t = 1.0
    rows = [_telem_row(window_start_t + i * 0.01) for i in range(20)]  # last at t=1.19, uniform 10ms cadence
    bridge = [_bridge_row(window_start_t + i * 0.01) for i in range(20)]
    decision_time_t = rows[-1]["t"] + 0.5  # decision made 0.5s AFTER the last sample arrived
    result = evaluate_pre_arm_samples(rows, rows, bridge, FORCE_TOL, STALE_S,
                                      window_start_t=window_start_t, decision_time_t=decision_time_t)
    assert result["status"] == "PRE_ARM_FAIL"
    assert any("final sample age" in r for r in result["reasons"])
    # Confirm max_gap itself stayed small -- this is genuinely a DIFFERENT
    # failure mode than test_gap_inside_dedicated_window_still_fails above.
    assert result["left_max_gap_s"] < STALE_S


# ---------------------------------------------------------------------------
# Item 4: a genuinely fresh, normal-cadence dedicated window (left, right,
# AND bridge) passes cleanly.
# ---------------------------------------------------------------------------

def test_fresh_normal_cadence_window_all_three_streams_passes():
    window_start_t = 2.0
    left = [_telem_row(window_start_t + i * 0.005) for i in range(20)]   # 200Hz-like
    right = [_telem_row(window_start_t + i * 0.005) for i in range(20)]
    bridge = [_bridge_row(window_start_t + i * 0.01) for i in range(20)]  # 100Hz-like
    decision_time_t = max(left[-1]["t"], right[-1]["t"], bridge[-1]["t"])
    result = evaluate_pre_arm_samples(left, right, bridge, FORCE_TOL, STALE_S,
                                      window_start_t=window_start_t, decision_time_t=decision_time_t)
    assert result["status"] == "PRE_ARM_PASS", result["reasons"]
    assert result["pre_arm_window_duration_s"] is not None
    assert result["left_final_sample_age_s"] < STALE_S
    assert result["right_final_sample_age_s"] < STALE_S
    assert result["bridge_final_sample_age_s"] < STALE_S


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
