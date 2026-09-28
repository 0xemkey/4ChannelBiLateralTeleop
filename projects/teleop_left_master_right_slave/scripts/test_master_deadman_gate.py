#!/usr/bin/env python3
"""Part C (2026-09-10, real-hardware safety completion): pure unit tests
for deadman_gate_state() in bridge_master_slave.py -- no ROS/rclpy.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import deadman_gate_state  # noqa: E402

MASK = 1
STALE_S = 0.2


def test_gate_inactive_by_default():
    held, latched, reason = deadman_gate_state(
        require=False, latched=False, buttons_t=None, now=100.0,
        stale_s=STALE_S, buttons_value=0, mask=MASK)
    assert held is True and latched is False and reason is None
    print("[1] require=False -> always held, never latched. PASS")


def test_press_holds():
    held, latched, reason = deadman_gate_state(
        require=True, latched=False, buttons_t=100.0, now=100.05,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held is True and latched is False
    print("[2] fresh telemetry, button held -> held=True. PASS")


def test_release_disables_not_latched():
    held, latched, reason = deadman_gate_state(
        require=True, latched=False, buttons_t=100.0, now=100.05,
        stale_s=STALE_S, buttons_value=0, mask=MASK)
    assert held is False and latched is False, "an ordinary release must NOT latch"
    print("[3] fresh telemetry, button released -> held=False, NOT latched. PASS")


def test_release_then_repress_resumes_immediately():
    # Release.
    held, latched, _ = deadman_gate_state(
        require=True, latched=False, buttons_t=100.0, now=100.05,
        stale_s=STALE_S, buttons_value=0, mask=MASK)
    assert held is False and latched is False
    # Re-press, telemetry still fresh -- resumes without any re-arm.
    held2, latched2, _ = deadman_gate_state(
        require=True, latched=latched, buttons_t=100.10, now=100.11,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held2 is True and latched2 is False
    print("[4] release then re-press (telemetry fresh throughout) resumes immediately. PASS")


def test_never_received_is_stale_and_latches():
    held, latched, reason = deadman_gate_state(
        require=True, latched=False, buttons_t=None, now=100.0,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held is False and latched is True
    assert "ever received" in reason
    print("[5] no button message ever received -> held=False, LATCHED. PASS")


def test_stale_telemetry_latches():
    held, latched, reason = deadman_gate_state(
        require=True, latched=False, buttons_t=100.0, now=100.0 + STALE_S + 0.5,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held is False and latched is True
    assert "old" in reason
    print("[6] stale button telemetry -> held=False, LATCHED (even though bit is set). PASS")


def test_latch_persists_across_ticks_until_externally_cleared():
    # Simulate the CALLER's own state: latch set once, then telemetry
    # resumes (buttons_t becomes fresh again) on a LATER tick, but the
    # caller has not cleared `latched` (no explicit re-arm yet).
    _, latched, _ = deadman_gate_state(
        require=True, latched=False, buttons_t=None, now=100.0,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert latched is True
    # Telemetry "recovers" (fresh buttons_t, button held) -- but latched is
    # still True (as tracked by the caller) and was NOT cleared here.
    held2, latched2, _ = deadman_gate_state(
        require=True, latched=latched, buttons_t=105.0, now=105.01,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held2 is False, "must NOT auto-resume just because telemetry recovered"
    assert latched2 is True, "stays latched until the caller explicitly clears it"
    print("[7] latch persists across ticks even after telemetry recovers -- "
          "no automatic re-arm. PASS")


def test_latch_clears_only_when_caller_passes_latched_false():
    # This models bridge_master_slave.py's own _on_param: a successful
    # arm_teleop re-arm (with fresh telemetry) clears the latch BEFORE the
    # next deadman_gate_state() call.
    held, latched, _ = deadman_gate_state(
        require=True, latched=False, buttons_t=200.0, now=200.01,
        stale_s=STALE_S, buttons_value=MASK, mask=MASK)
    assert held is True and latched is False
    print("[8] fresh re-arm (latched explicitly cleared by caller) resumes normal operation. PASS")


if __name__ == "__main__":
    test_gate_inactive_by_default()
    test_press_holds()
    test_release_disables_not_latched()
    test_release_then_repress_resumes_immediately()
    test_never_received_is_stale_and_latches()
    test_stale_telemetry_latches()
    test_latch_persists_across_ticks_until_externally_cleared()
    test_latch_clears_only_when_caller_passes_latched_false()
    print("ALL TESTS PASSED")
