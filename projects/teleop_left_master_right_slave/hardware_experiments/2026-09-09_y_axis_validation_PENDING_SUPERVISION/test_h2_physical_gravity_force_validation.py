"""2026-09-10 real-hardware finding regression tests: physical gravity-
force validation in analyze_h2_gravity_settle.py.

Real-hardware evidence: session 20260910T104521Z_h2 passed every settle-
gate criterion (gravity_ramp_alpha=1, coupling_ramp_alpha=0, C1-C4 off,
quiet-hold gate genuinely satisfied) while the slave's `enabled` flag was
False for ALL 2002 steady-settle samples -- confirmed via direct CSV
inspection that auxiliary_force_n, spring_force_n, damping_force_n, every
clamp stage, and applied_force_n were EXACTLY [0,0,0] throughout, and
traced to touch_slave.cpp:842 (`if (command.enabled && deadman_ok)`,
which gates ALL force including gravity-support auxiliary force). The
analyzer previously reported an unqualified "PASS" from the settle-gate
criteria alone, with no check that the gravity-support force this whole
stage exists to validate was ever actually applied. Separately,
`slave_aux_y_expected_n` was always null due to a missing metadata-
plumbing bug (frozen_thresholds never included slave_gravity_comp_n) --
fixed at the source (part1_gravity_settle_capture.py), not hardcoded here.
"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from analyze_h2_gravity_settle import analyze  # noqa: E402

DT = 0.01


def _telem_row(t, y=-0.1, vy=0.0, aux_y=0.0, applied_y=0.0, enabled=True, deadman_ok=True):
    row = {"t": t, "phase": "GRAVITY_SETTLE"}
    for f in ["position_m", "velocity_mps", "target_position_m", "position_error_raw_m",
             "spring_force_n", "damping_force_n", "auxiliary_force_n", "force_before_primary_clamp_n",
             "force_after_primary_clamp_n", "relay_torque_received_nm", "relay_force_n",
             "force_before_final_clamp_n", "applied_force_n"]:
        row[f"{f}_x"] = 0.0
        row[f"{f}_y"] = 0.0
        row[f"{f}_z"] = 0.0
    row["position_m_y"] = y
    row["velocity_mps_y"] = vy
    row["auxiliary_force_n_y"] = aux_y
    row["applied_force_n_y"] = applied_y
    for f in ["enabled", "deadman_ok", "target_valid", "target_fresh", "force_valid", "force_fresh",
             "relay_valid", "relay_fresh", "relay_applied", "tracking_valid", "position_error_clamped",
             "primary_force_clamped", "final_force_clamped", "joint_path_commanded",
             "cartesian_write_active", "near_singular"]:
        row[f] = False
    row["enabled"] = enabled
    row["deadman_ok"] = deadman_ok
    row["dob_waist"] = 0.05
    row["dob_shoulder"] = -0.08
    row["dob_elbow"] = 0.03
    row["dob_configured"] = True
    return row


def _bridge_row(t, state="GRAVITY_SETTLE", accum=0.15):
    return {
        "t": t, "bridge_state": state, "gravity_ramp_alpha": 1.0, "coupling_ramp_alpha": 0.0,
        "ramp_alpha": 0.0, "c1_enabled": False, "c2_enabled": False, "c3_enabled": False, "c4_enabled": False,
        "master_travel_limit_flag": False, "slave_travel_limit_flag": False,
        "planner_parameter_match_left": True, "planner_parameter_match_right": True,
        "force_budget_telemetry_fresh_left": True, "force_budget_telemetry_fresh_right": True,
        "gravity_settle_master_speed_mps": 0.0001, "gravity_settle_slave_speed_mps": 0.0001,
        "gravity_settle_master_pose_drift_m": 0.00001, "gravity_settle_slave_pose_drift_m": 0.00001,
        "gravity_settle_quiet_window_accumulated_s": accum,
        "gravity_settle_accepted_master_pose_drift_m": 0.00001,
        "gravity_settle_accepted_slave_pose_drift_m": 0.00001,
    }


def _write_session(out_dir: Path, master_rows, slave_rows, bridge_rows, meta_extra=None,
                   include_gravity_metadata=True):
    def write_csv(fn, rows):
        with open(out_dir / fn, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
    write_csv("part1_gravity_settle_master.csv", master_rows)
    write_csv("part1_gravity_settle_slave.csv", slave_rows)
    write_csv("part1_bridge_diag.csv", bridge_rows)

    gate_pass_t = bridge_rows[0]["t"]
    frozen_thresholds = {
        "gravity_settle_velocity_threshold_mps": 0.005, "gravity_settle_hold_s": 0.15,
        "gravity_settle_min_time_s": 0.1, "gravity_settle_timeout_s": 10.0,
        "force_budget_telemetry_stale_s": 0.2, "unexpected_spring_damping_norm_n": 0.05,
    }
    if include_gravity_metadata:
        frozen_thresholds.update({"slave_gravity_comp_n": 0.55, "slave_gravity_axis": 1,
                                  "left_gravity_comp_n": 0.0, "left_gravity_axis": 1})
    meta = {
        "session_id": "synthtest", "config_fingerprint": "deadbeef", "git_head": "x", "git_dirty": False,
        "git_branch": "x", "bridge_command": ["fake"], "aborted": False, "abort_reason": None, "abort_t": None,
        "final_phase": "GRAVITY_SETTLE", "disarm_confirmed": True, "arm_t": 0.0,
        "gate_pass_t": gate_pass_t, "gate_pass_accumulated_s": 0.15, "observation_window_s": 10.0,
        "frozen_thresholds": frozen_thresholds, "dob_baseline": None,
    }
    if meta_extra:
        meta.update(meta_extra)
    (out_dir / "part1_meta.json").write_text(json.dumps(meta, indent=2))


# ---------------------------------------------------------------------------
# Item 1: gate passes but applied force stays zero (enabled=False
# throughout) -> physical-force validation must FAIL. This is the EXACT
# real-hardware scenario.
# ---------------------------------------------------------------------------

def test_gate_passes_but_force_never_enabled_fails_physical_validation():
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        n = 200
        master_rows = [_telem_row(i * DT, enabled=False, applied_y=0.0) for i in range(n)]
        slave_rows = [_telem_row(i * DT, y=-0.21, enabled=False, aux_y=0.0, applied_y=0.0) for i in range(n)]
        bridge_rows = [_bridge_row(i * DT) for i in range(n)]
        _write_session(out_dir, master_rows, slave_rows, bridge_rows)

        result = analyze(out_dir)
        assert result["validation"]["overall_result"] == "FAIL"
        codes = [f["code"] for f in result["validation"]["flags"]]
        assert "GRAVITY_FORCE_NEVER_ENABLED" in codes
        assert result["metrics"]["physical_gravity_force_validated"] is False
        assert result["metrics"]["physical_gravity_force_enabled_sample_count"] == 0


# ---------------------------------------------------------------------------
# Item 2: intended gravity force genuinely present in applied_force_n ->
# physical-force validation PASSES (and overall PASS, matching the
# existing settle-gate criteria also passing on nominal data).
# ---------------------------------------------------------------------------

def test_gravity_force_present_in_applied_force_passes():
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        n = 200
        master_rows = [_telem_row(i * DT, enabled=True, applied_y=0.0) for i in range(n)]
        slave_rows = [_telem_row(i * DT, y=-0.21, enabled=True, aux_y=0.55, applied_y=0.55) for i in range(n)]
        bridge_rows = [_bridge_row(i * DT) for i in range(n)]
        _write_session(out_dir, master_rows, slave_rows, bridge_rows)

        result = analyze(out_dir)
        assert result["validation"]["overall_result"] == "PASS", result["validation"]["flags"]
        assert result["metrics"]["physical_gravity_force_validated"] is True
        assert abs(result["metrics"]["physical_gravity_force_mean_applied_n"] - 0.55) < 1e-6


def test_gravity_force_present_but_wrong_magnitude_fails():
    """A genuinely enabled run whose applied force does NOT match the
    configured gravity-comp value (e.g. half-applied, or some other bug)
    must fail -- distinct failure mode from 'never enabled'."""
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        n = 200
        master_rows = [_telem_row(i * DT, enabled=True, applied_y=0.0) for i in range(n)]
        slave_rows = [_telem_row(i * DT, y=-0.21, enabled=True, aux_y=0.2, applied_y=0.2) for i in range(n)]
        bridge_rows = [_bridge_row(i * DT) for i in range(n)]
        _write_session(out_dir, master_rows, slave_rows, bridge_rows)

        result = analyze(out_dir)
        assert result["validation"]["overall_result"] == "FAIL"
        codes = [f["code"] for f in result["validation"]["flags"]]
        assert "GRAVITY_FORCE_MISMATCH" in codes
        assert result["metrics"]["physical_gravity_force_validated"] is False


# ---------------------------------------------------------------------------
# Item 3: expected gravity metadata missing -> analyzer must not silently
# PASS (the exact plumbing bug found on the real run).
# ---------------------------------------------------------------------------

def test_missing_expected_gravity_metadata_does_not_silently_pass():
    with tempfile.TemporaryDirectory() as td:
        out_dir = Path(td)
        n = 200
        master_rows = [_telem_row(i * DT, enabled=True, applied_y=0.0) for i in range(n)]
        slave_rows = [_telem_row(i * DT, y=-0.21, enabled=True, aux_y=0.55, applied_y=0.55) for i in range(n)]
        bridge_rows = [_bridge_row(i * DT) for i in range(n)]
        _write_session(out_dir, master_rows, slave_rows, bridge_rows, include_gravity_metadata=False)

        result = analyze(out_dir)
        assert result["validation"]["overall_result"] == "FAIL"
        codes = [f["code"] for f in result["validation"]["flags"]]
        assert "GRAVITY_FORCE_EXPECTED_VALUE_MISSING" in codes
        assert result["metrics"]["slave_aux_y_expected_n"] is None
        assert result["metrics"]["physical_gravity_force_validated"] is False


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
