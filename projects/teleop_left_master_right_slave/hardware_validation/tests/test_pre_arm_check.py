"""Pure-Python tests for pre_arm_check.py's evaluate_pre_arm_samples()
(2026-09-10, H2 pre-arm safety-hardening task). No ROS/rclpy needed --
check_enable_publisher_ownership() (rclpy-dependent) is not covered here,
same convention as this project's other pure-vs-live test split.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pre_arm_check import evaluate_pre_arm_samples, VALID_PRE_ARM_BRIDGE_STATES  # noqa: E402

FORCE_TOL = 0.05
STALE_S = 0.2


def _telem_row(t, enabled=False, applied=0.0, clamped=False):
    return {"t": t, "enabled": enabled,
           "applied_force_n_x": 0.0, "applied_force_n_y": applied, "applied_force_n_z": 0.0,
           "position_error_clamped": clamped, "primary_force_clamped": False, "final_force_clamped": False}


def _bridge_row(t, state="READY", coupling=0.0, c1=False, c2=False, c3=False, c4=False):
    return {"t": t, "bridge_state": state, "coupling_ramp_alpha": coupling,
           "c1_enabled": c1, "c2_enabled": c2, "c3_enabled": c3, "c4_enabled": c4}


def _nominal(n=20, dt=0.01):
    left = [_telem_row(i * dt) for i in range(n)]
    right = [_telem_row(i * dt) for i in range(n)]
    bridge = [_bridge_row(i * dt) for i in range(n)]
    return left, right, bridge


def _eval(left, right, bridge, **kw):
    """Wrapper for tests that are not specifically about the
    window_start_t/decision_time_t freshness-at-decision-time mechanism
    (added 2026-09-10, see test_pre_arm_window_freshness.py for those
    dedicated tests): defaults decision_time_t to the latest timestamp
    actually present in the synthetic data, so "final sample age" reads
    ~0 instead of comparing against a real wall-clock time.time() the
    synthetic `t` values (small, near-zero numbers) were never meant to
    be measured against."""
    kw.setdefault("decision_time_t", max(
        (left[-1]["t"] if left else 0.0), (right[-1]["t"] if right else 0.0),
        (bridge[-1]["t"] if bridge else 0.0)))
    return evaluate_pre_arm_samples(left, right, bridge, FORCE_TOL, STALE_S, **kw)


def test_nominal_pre_arm_state_passes():
    left, right, bridge = _nominal()
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_PASS", r["reasons"]


def test_too_few_samples_fails():
    left, right, bridge = _nominal(n=3)
    r = _eval(left, right, bridge, min_samples=5)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("only 3 sample" in reason for reason in r["reasons"])


def test_stale_enabled_latch_fails():
    """The exact scenario this task is about: a stale enabled=True on the
    master, discovered by a prior read-only check."""
    left, right, bridge = _nominal()
    left[5]["enabled"] = True
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("left: enabled=True" in reason for reason in r["reasons"])


def test_enabled_true_allowed_when_require_enabled_false_is_false():
    """2026-09-10 production-arming finding: with no deadman gate
    (production's normal require_master_deadman=False), the bridge
    publishes enable=True unconditionally from startup, regardless of arm
    state -- confirmed on real hardware. require_enabled_false=False must
    let this through without flagging it."""
    left, right, bridge = _nominal()
    for r in left:
        r["enabled"] = True
    for r in right:
        r["enabled"] = True
    result = _eval(left, right, bridge, require_enabled_false=False)
    assert result["status"] == "PRE_ARM_PASS", result["reasons"]


def test_enabled_true_still_fails_by_default():
    """The default (require_enabled_false=True) must remain H2's existing,
    stricter behavior -- production opting out must be explicit."""
    left, right, bridge = _nominal()
    for r in left:
        r["enabled"] = True
    result = _eval(left, right, bridge)
    assert result["status"] == "PRE_ARM_FAIL"


def test_nonzero_force_fails():
    left, right, bridge = _nominal()
    right[10]["applied_force_n_y"] = 0.5
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("right: applied_force_norm" in reason for reason in r["reasons"])


def test_clamp_flag_fails():
    left, right, bridge = _nominal()
    left[3]["position_error_clamped"] = True
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("clamp flag set" in reason for reason in r["reasons"])


def test_invalid_bridge_state_fails():
    left, right, bridge = _nominal()
    bridge[7]["bridge_state"] = "ACTIVE"
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("not a valid pre-arm state" in reason for reason in r["reasons"])


def test_fault_state_fails():
    left, right, bridge = _nominal()
    bridge[7]["bridge_state"] = "FAULT"
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("FAULT observed" in reason for reason in r["reasons"])


def test_nonzero_coupling_alpha_fails():
    left, right, bridge = _nominal()
    bridge[9]["coupling_ramp_alpha"] = 0.3
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("coupling_ramp_alpha=0.3" in reason for reason in r["reasons"])


def test_channel_configured_on_fails():
    left, right, bridge = _nominal()
    bridge[2]["c3_enabled"] = True
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("channel is configured differently than expected" in reason for reason in r["reasons"])


def test_expected_channels_c3_true_passes_for_production_arming():
    """2026-09-10 production-arming reuse: c3=True is the CORRECT
    configured state for production C3-only teleoperation -- must not be
    flagged as an error when explicitly expected."""
    left, right, bridge = _nominal()
    for r in bridge:
        r["c3_enabled"] = True
    result = _eval(left, right, bridge, expected_channels={"c1": False, "c2": False, "c3": True, "c4": False})
    assert result["status"] == "PRE_ARM_PASS", result["reasons"]


def test_expected_channels_mismatch_still_fails():
    left, right, bridge = _nominal()
    for r in bridge:
        r["c3_enabled"] = True
        r["c2_enabled"] = True  # unexpected -- only c3 should be on
    result = _eval(left, right, bridge, expected_channels={"c1": False, "c2": False, "c3": True, "c4": False})
    assert result["status"] == "PRE_ARM_FAIL"
    assert any("channel is configured differently than expected" in reason for reason in result["reasons"])


def test_telemetry_gap_fails():
    left, right, bridge = _nominal()
    left[10]["t"] = left[9]["t"] + 1.0  # 1s gap >> 0.2s stale threshold
    r = _eval(left, right, bridge)
    assert r["status"] == "PRE_ARM_FAIL"
    assert any("telemetry gap" in reason for reason in r["reasons"])


def test_every_valid_pre_arm_state_individually_passes():
    for state in VALID_PRE_ARM_BRIDGE_STATES:
        left, right, bridge = _nominal(n=6)
        for r in bridge:
            r["bridge_state"] = state
        result = _eval(left, right, bridge, min_samples=5)
        assert result["status"] == "PRE_ARM_PASS", (state, result["reasons"])


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
