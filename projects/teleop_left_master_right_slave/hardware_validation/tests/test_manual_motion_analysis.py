"""Pure-Python tests for analyze_manual_motion.py (2026-09-10, H3/H4
recorder tooling task, section 17: simulation/synthetic validation of the
ANALYZER half -- no ROS, no rclpy, no sim process required; these test the
event-detection/metrics/validation logic directly against constructed
row data, matching the exact field names merge_streams()/detect_events()/
etc. read from the recorder's raw_*.csv output.

Covers task-spec scenarios (section 17): A (nominal), B (no-motion), C
(wrong-direction), D (cross-axis), E (dropout), F (unexpected C2), H
(bridge FAULT), I (final-force clamp). G (C1/C4 unexpected activation) is
representative of the same UNEXPECTED_CHANNEL_ACTIVATION code path as F,
covered by parametrizing F rather than a separate test. J (repeated
cycles) is NOT covered -- detect_events() is a first-crossing state
machine (BASELINE->MOTION_START->PEAK->RETURN_START->RETURN_COMPLETE) and
does not attempt multi-cycle segmentation; this is a documented limitation
(see FINAL_REPORT_2026-09-10_H3_H4_RECORDER.md), not silently claimed as
tested.
"""
from __future__ import annotations

import math
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_manual_motion import (  # noqa: E402
    merge_streams, detect_events, compute_metrics, force_dob_analysis,
    channel_isolation_check, validate_trial, MIN_EXCITATION_M, analyze,
    normalize_analysis_mode, primary_direction_for_mode,
)

DT = 0.01  # 100Hz, matches FROZEN.rate_hz


def _master_row(t, y, x=0.0, z=0.0, applied_y=0.0, enabled=True, deadman_ok=True,
                final_clamped=False):
    return {"t_recv_monotonic": t, "t_ros": t,
           "dropout_gap_s": 0.0,
           "mapped_rel_y": y, "mapped_rel_x": x, "mapped_rel_z": z,
           "applied_force_n_y": applied_y, "spring_force_n_y": applied_y * 0.5,
           "damping_force_n_y": 0.0, "auxiliary_force_n_y": 0.0, "relay_force_n_y": 0.0,
           "enabled": enabled, "deadman_ok": deadman_ok, "final_force_clamped": final_clamped}


def _slave_row(t, y, x=0.0, z=0.0, applied_y=0.0, final_clamped=False):
    return {"t_recv_monotonic": t, "t_ros": t,
           "rel_y": y, "rel_x": x, "rel_z": z,
           "applied_force_n_y": applied_y, "final_force_clamped": final_clamped}


def _bridge_row(t, state="ACTIVE", c1=False, c2=False, c3=True, c4=False):
    return {"t_recv_monotonic": t, "t_ros": t, "bridge_state": state,
           "c1_enabled": c1, "c2_enabled": c2, "c3_enabled": c3, "c4_enabled": c4}


def _bilateral_row(t, active=(False, False, False, False), safety_state=0.0):
    row = {"t_recv_monotonic": t, "t_ros": t, "safety_state": safety_state}
    for i, a in enumerate(active):
        row[f"channel_active_{i}"] = a
    return row


def _nominal_master_slave_rows(n=400, dt=DT, amp=0.005, gain=0.8, lag_samples=2,
                               x_ratio=0.0, z_ratio=0.0, wrong_direction=False,
                               dropout_gap_at=None, dropout_gap_s=0.5):
    """Outward (0..0.3), hold (0.3..0.5), return (0.5..0.8) raised-cosine
    profile over n samples -- a smooth, non-step, no-random-noise synthetic
    trial matching what a real manual +5mm excursion should look like."""
    sign = -1.0 if wrong_direction else 1.0
    master_rows, slave_rows = [], []
    for i in range(n):
        t = i * dt
        u = i / (n - 1)
        if u < 0.3:
            y = sign * amp * (0.5 - 0.5 * math.cos(math.pi * (u / 0.3)))
        elif u < 0.5:
            y = sign * amp
        else:
            v = (u - 0.5) / 0.5
            y = sign * amp * (1.0 - (0.5 - 0.5 * math.cos(math.pi * v)))
        mrow = _master_row(t, y, x=y * x_ratio, z=y * z_ratio, applied_y=y * 40)
        if dropout_gap_at is not None and i == dropout_gap_at:
            mrow["dropout_gap_s"] = dropout_gap_s
        master_rows.append(mrow)
    for i in range(n):
        t = i * dt
        src_i = max(0, i - lag_samples)
        sy = gain * master_rows[src_i]["mapped_rel_y"]
        slave_rows.append(_slave_row(t, sy, applied_y=sy * 40))
    return master_rows, slave_rows


def _nominal_slave_master_rows(n=400, dt=DT, amp=0.006, gain=0.6, lag_samples=3):
    slave_rows, master_rows = [], []
    for i in range(n):
        t = i * dt
        u = i / (n - 1)
        if u < 0.3:
            y = amp * (0.5 - 0.5 * math.cos(math.pi * (u / 0.3)))
        elif u < 0.5:
            y = amp
        else:
            v = (u - 0.5) / 0.5
            y = amp * (1.0 - (0.5 - 0.5 * math.cos(math.pi * v)))
        slave_rows.append(_slave_row(t, y, applied_y=y * 30))
    for i in range(n):
        t = i * dt
        src_i = max(0, i - lag_samples)
        my = gain * slave_rows[src_i]["rel_y"]
        master_rows.append(_master_row(t, my, applied_y=my * 30))
    return master_rows, slave_rows


def _write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def _flat_rows(n=200, dt=DT, noise=0.0):
    master_rows = [_master_row(i * dt, 0.0 + (noise if i % 2 == 0 else -noise)) for i in range(n)]
    slave_rows = [_slave_row(i * dt, 0.0) for i in range(n)]
    return master_rows, slave_rows


def _bridge_rows_for(n, dt=DT, state="ACTIVE", **kw):
    return [_bridge_row(i * dt, state=state, **kw) for i in range(n)]


# ---------------------------------------------------------------------------
# A. nominal
# ---------------------------------------------------------------------------

def test_scenario_a_nominal_c3_only_valid():
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    assert not ev["insufficient_excitation"]
    assert not ev["wrong_direction"]
    names = [e["name"] for e in ev["events"]]
    assert "MOTION_START" in names and "POSITIVE_PEAK" in names
    assert "RETURN_START" in names and "RETURN_COMPLETE" in names

    metrics = compute_metrics(merged, ev)
    assert metrics["result"] == "EVALUATED"
    assert metrics["master_peak_y_m"] > 0.004
    assert 0.5 < metrics["positive_motion_gain"] < 1.1  # ~0.8 configured
    assert metrics["lag_positive_motion_s"] is not None
    assert metrics["settling_time_s"] is not None

    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    assert iso["configured_matches_expected"] is True
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "VALID", val


def test_c2_only_uses_slave_as_source_and_master_as_follower(tmp_path):
    m, s = _nominal_slave_master_rows(gain=0.6)
    b = _bridge_rows_for(len(m), c1=False, c2=True, c3=False, c4=False)
    bi = [_bilateral_row(i * DT, active=(False, False, False, False)) for i in range(len(m))]
    out = tmp_path / "trial"
    out.mkdir()
    _write_csv(out / "raw_master.csv", m)
    _write_csv(out / "raw_slave.csv", s)
    _write_csv(out / "raw_bridge.csv", b)
    _write_csv(out / "raw_bilateral.csv", bi)
    (out / "manifest.json").write_text(json.dumps({
        "expected_stage": "c2-only",
        "expected_channel_configuration": {"c1": False, "c2": True, "c3": False, "c4": False},
        "stream_stats": {},
    }))

    result = analyze(out)
    metrics = result["metrics"]
    assert metrics["analysis_mode"] == "c2-only"
    assert metrics["analysis_direction"] == "c2"
    assert metrics["source_role"] == "slave"
    assert metrics["follower_role"] == "master"
    assert 0.45 < metrics["positive_motion_gain"] < 0.75
    assert result["validation"]["overall_result"] == "VALID"


def test_c2c3_reports_both_causal_directions():
    m, s = _nominal_master_slave_rows(gain=0.8)
    b = _bridge_rows_for(len(m), c1=False, c2=True, c3=True, c4=False)
    merged = merge_streams(m, s, b, [])
    ev_c3 = detect_events(merged, source_key="master_y", source_label="master")
    ev_c2 = detect_events(merged, source_key="slave_y", source_label="slave")
    m_c3 = compute_metrics(merged, ev_c3, direction="c3")
    m_c2 = compute_metrics(merged, ev_c2, direction="c2")
    assert m_c3["source_role"] == "master"
    assert m_c3["follower_role"] == "slave"
    assert m_c2["source_role"] == "slave"
    assert m_c2["follower_role"] == "master"


def test_force_modes_select_correct_physical_source():
    assert normalize_analysis_mode({"expected_stage": "c1-only"}) == "c1-only"
    assert primary_direction_for_mode("c1-only") == "c1"
    assert primary_direction_for_mode("c4-only") == "c4"

    m, s = _nominal_slave_master_rows(gain=0.6)
    b = _bridge_rows_for(len(m), c1=True, c2=False, c3=False, c4=False)
    bi = [_bilateral_row(i * DT, active=(True, False, False, False)) for i in range(len(m))]
    merged = merge_streams(m, s, b, bi)
    spec_ev = detect_events(merged, source_key="slave_y", source_label="slave")
    metrics = compute_metrics(merged, spec_ev, direction="c1")
    assert metrics["source_role"] == "slave"
    assert metrics["follower_role"] == "master"


def test_expected_force_channel_requires_bilateral_status():
    m, s = _nominal_slave_master_rows(gain=0.6)
    b = _bridge_rows_for(len(m), c1=True, c2=False, c3=False, c4=False)
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged, source_key="slave_y", source_label="slave")
    iso = channel_isolation_check(merged, {"c1": True, "c2": False, "c3": False, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INVALID_TRIAL"
    assert "MISSING_BILATERAL_STATUS" in [f["code"] for f in val["flags"]]


def test_expected_force_channel_must_be_active():
    m, s = _nominal_slave_master_rows(gain=0.6)
    b = _bridge_rows_for(len(m), c1=True, c2=False, c3=False, c4=False)
    bi = [_bilateral_row(i * DT, active=(False, False, False, False)) for i in range(len(m))]
    merged = merge_streams(m, s, b, bi)
    ev = detect_events(merged, source_key="slave_y", source_label="slave")
    iso = channel_isolation_check(merged, {"c1": True, "c2": False, "c3": False, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INVALID_TRIAL"
    assert "EXPECTED_CHANNEL_NOT_ACTIVE" in [f["code"] for f in val["flags"]]


# ---------------------------------------------------------------------------
# B. no-motion / insufficient excitation
# ---------------------------------------------------------------------------

def test_scenario_b_no_motion_insufficient_excitation():
    m, s = _flat_rows(noise=0.00005)  # far below MOTION_THRESHOLD_M/MIN_EXCITATION_M
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    assert ev["insufficient_excitation"] is True

    metrics = compute_metrics(merged, ev)
    assert metrics["result"] == "NOT_EVALUABLE"

    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INSUFFICIENT_EXCITATION"


def test_noise_is_never_classified_as_motion():
    """A slightly larger, but still sub-threshold, single-sample spike must
    not trip MOTION_START -- only a SUSTAINED excursion should."""
    m, s = _flat_rows(n=100)
    m[50]["mapped_rel_y"] = 0.02  # one single-sample huge spike, immediately reverting
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    assert ev["insufficient_excitation"] is True, "a single-sample spike must not count as sustained motion"


# ---------------------------------------------------------------------------
# C. wrong direction
# ---------------------------------------------------------------------------

def test_scenario_c_wrong_direction_invalid():
    m, s = _nominal_master_slave_rows(wrong_direction=True)
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    assert ev["wrong_direction"] is True

    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INVALID_TRIAL"
    codes = [f["code"] for f in val["flags"]]
    assert "WRONG_DIRECTION_MOTION" in codes


# ---------------------------------------------------------------------------
# D. cross-axis contamination
# ---------------------------------------------------------------------------

def test_scenario_d_cross_axis_contamination_reported():
    m, s = _nominal_master_slave_rows(x_ratio=0.4)  # 40% of Y excursion leaking into X
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    metrics = compute_metrics(merged, ev)
    ratio = metrics["cross_axis"]["master_x_over_y_ratio"]
    assert ratio is not None
    assert 0.3 < ratio < 0.5, f"expected ~0.4 cross-axis ratio, got {ratio}"


def test_cross_axis_clean_trial_reports_near_zero_ratio():
    m, s = _nominal_master_slave_rows(x_ratio=0.0, z_ratio=0.0)
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    metrics = compute_metrics(merged, ev)
    assert abs(metrics["cross_axis"]["master_x_over_y_ratio"]) < 0.01


# ---------------------------------------------------------------------------
# E. telemetry dropout / stale interval
# ---------------------------------------------------------------------------

def test_scenario_e_dropout_flagged_via_stream_stats():
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    stream_stats = {"master": {"available": True, "max_gap_s": 0.6, "n": len(m), "effective_hz": 100.0}}
    val = validate_trial(merged, ev, iso, stream_stats)
    codes = [f["code"] for f in val["flags"]]
    assert "TELEMETRY_STALE" in codes


def test_no_dropout_does_not_flag_stale():
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    stream_stats = {"master": {"available": True, "max_gap_s": 0.01, "n": len(m), "effective_hz": 100.0}}
    val = validate_trial(merged, ev, iso, stream_stats)
    codes = [f["code"] for f in val["flags"]]
    assert "TELEMETRY_STALE" not in codes


# ---------------------------------------------------------------------------
# F/G. unexpected channel activation (C2 here; same code path covers C1/C4)
# ---------------------------------------------------------------------------

def test_scenario_f_unexpected_c2_activation_invalid():
    m, s = _nominal_master_slave_rows(n=100)
    b = _bridge_rows_for(len(m))
    bi = [_bilateral_row(i * DT, active=(False, True, True, False)) for i in range(len(m))]  # C2 unexpectedly active
    merged = merge_streams(m, s, b, bi)
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    assert iso["unexpected_activation"], "C2 active while expected False must be reported"
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INVALID_TRIAL"
    codes = [f["code"] for f in val["flags"]]
    assert "UNEXPECTED_CHANNEL_ACTIVATION" in codes


def test_no_bilateral_status_does_not_falsely_flag_unexpected_activation():
    """H3 is a C3-only stage -- bilateral_4ch.py is not expected to even be
    running, so an EMPTY bilateral stream must not be misread as evidence
    of an unexpected activation."""
    m, s = _nominal_master_slave_rows(n=100)
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    assert iso["live_channel_active_available"] is False
    assert iso["unexpected_activation"] is None
    val = validate_trial(merged, ev, iso, {})
    codes = [f["code"] for f in val["flags"]]
    assert "UNEXPECTED_CHANNEL_ACTIVATION" not in codes


# ---------------------------------------------------------------------------
# H. bridge FAULT during trial
# ---------------------------------------------------------------------------

def test_scenario_h_bridge_fault_invalid():
    m, s = _nominal_master_slave_rows(n=100)
    b = _bridge_rows_for(len(m))
    b[60]["bridge_state"] = "FAULT"
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    assert val["overall_result"] == "INVALID_TRIAL"
    codes = [f["code"] for f in val["flags"]]
    assert "BRIDGE_FAULT" in codes


# ---------------------------------------------------------------------------
# I. final-force clamp during trial
# ---------------------------------------------------------------------------

def test_scenario_i_final_force_clamp_flagged_not_necessarily_invalid():
    m, s = _nominal_master_slave_rows(n=100)
    m[50]["final_force_clamped"] = True
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    codes = [f["code"] for f in val["flags"]]
    assert "FINAL_FORCE_CLAMP" in codes
    # A clamp alone (no FAULT/wrong-direction/unexpected-activation/failure-
    # to-return) is a FLAG, not by itself grounds for INVALID_TRIAL.
    assert val["overall_result"] == "VALID"


# ---------------------------------------------------------------------------
# failure to return
# ---------------------------------------------------------------------------

def test_failure_to_return_invalid():
    n = 200
    master_rows = []
    for i in range(n):
        t = i * DT
        u = min(1.0, i / (n * 0.5))
        y = 0.005 * (0.5 - 0.5 * math.cos(math.pi * u))  # ramps up, never returns
        master_rows.append(_master_row(t, y))
    slave_rows = [_slave_row(i * DT, 0.8 * master_rows[i]["mapped_rel_y"]) for i in range(n)]
    b = _bridge_rows_for(n)
    merged = merge_streams(master_rows, slave_rows, b, [])
    ev = detect_events(merged)
    assert not ev["insufficient_excitation"]
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    val = validate_trial(merged, ev, iso, {})
    codes = [f["code"] for f in val["flags"]]
    assert "FAILURE_TO_RETURN" in codes
    assert val["overall_result"] == "INVALID_TRIAL"


# ---------------------------------------------------------------------------
# force/DOB analysis: insufficient excitation must not claim PASS
# ---------------------------------------------------------------------------

def test_force_dob_not_evaluable_without_excitation():
    m, s = _flat_rows()
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    force = force_dob_analysis(merged, ev)
    assert force["result"] == "NOT_EVALUABLE"


def test_force_dob_evaluated_with_excitation():
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m))
    merged = merge_streams(m, s, b, [])
    ev = detect_events(merged)
    force = force_dob_analysis(merged, ev)
    assert force["result"] == "EVALUATED"
    assert force["master_peak_applied_force_n_y"] > 0


# ---------------------------------------------------------------------------
# H. C2/C3 channel-state semantics fix (2026-09-10, full4ch pre-tuning
# closure Task 4/8) -- reproduces the exact confirmed bug from the
# 20260910T155604Z_full4ch_current_setup_baseline session: real
# bidirectional C2+C3 position tracking, bilateral_4ch.py running (for
# C1/C4) with its OWN C2/C3 weights correctly held at 0 (the supervised
# architecture's design, not a defect), and the OLD channel_isolation_
# check() reporting missing_expected_activation=[C2,C3] purely because it
# asked bilateral_status to prove channels it does not own.
# ---------------------------------------------------------------------------

def test_c2_c3_active_via_bridge_not_flagged_missing_when_bilateral_reports_inactive():
    """The exact full4ch bug: bridge says c2/c3 configured+active (real
    position coupling is working), bilateral_4ch.py is ALSO running (for
    C1/C4) but correctly reports channel_active_1/2=False (its own C2/C3
    weights are held at 0 by design) -- this must NOT be reported as
    missing_expected_activation for C2/C3."""
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m), c1=True, c2=True, c3=True, c4=True)
    bi = [_bilateral_row(i * DT, active=(True, False, False, True)) for i in range(len(m))]
    merged = merge_streams(m, s, b, bi)
    iso = channel_isolation_check(merged, {"c1": True, "c2": True, "c3": True, "c4": True})
    assert iso["missing_expected_activation"] is None, iso["missing_expected_activation"]
    assert iso["unexpected_activation"] is None, iso["unexpected_activation"]


def test_c2_c3_active_via_bridge_with_no_bilateral_status_at_all():
    """A plain C2/C3-only session (no bilateral_4ch.py running at all,
    no relay/force channels involved) must not require bilateral_status
    to confirm C2/C3 -- the bridge alone is authoritative for them."""
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m), c1=False, c2=True, c3=True, c4=False)
    merged = merge_streams(m, s, b, [])
    iso = channel_isolation_check(merged, {"c1": False, "c2": True, "c3": True, "c4": False})
    assert iso["missing_expected_activation"] is None, iso["missing_expected_activation"]
    assert iso["live_channel_active_available"] is False  # no bilateral_status at all, correctly reported


def test_c2_active_via_bilateral_status_alone_is_flagged_as_duplicate_authority():
    """Safety check preserved: if bilateral_4ch.py's OWN C2 weight were
    ever nonzero (bridge itself reports c2=False, i.e. NOT expected/
    configured), that is a real duplicate-position-authority danger and
    must still be caught, even though bilateral_status is not C2's
    normal owner."""
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m), c1=False, c2=False, c3=True, c4=False)
    bi = [_bilateral_row(i * DT, active=(False, True, False, False)) for i in range(len(m))]  # C2 active here too
    merged = merge_streams(m, s, b, bi)
    iso = channel_isolation_check(merged, {"c1": False, "c2": False, "c3": True, "c4": False})
    assert iso["unexpected_activation"], "bilateral_4ch reporting C2 active must still be flagged"


def test_c1_c4_still_require_bilateral_status_as_before():
    """C1/C4's ownership (bilateral_status) is unchanged by this fix --
    bridge-side c1/c4_enabled=true alone must not be treated as
    sufficient confirmation for C1/C4."""
    m, s = _nominal_master_slave_rows()
    b = _bridge_rows_for(len(m), c1=True, c2=False, c3=False, c4=True)
    bi = [_bilateral_row(i * DT, active=(False, False, False, False)) for i in range(len(m))]
    merged = merge_streams(m, s, b, bi)
    iso = channel_isolation_check(merged, {"c1": True, "c2": False, "c3": False, "c4": True})
    assert iso["missing_expected_activation"] == [{"channel": "c1"}, {"channel": "c4"}]


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
