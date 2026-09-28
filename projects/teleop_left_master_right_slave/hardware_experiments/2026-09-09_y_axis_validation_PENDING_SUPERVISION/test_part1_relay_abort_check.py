"""2026-09-10 real-hardware finding regression test: GravitySettleCapture.
check_abort() must NOT abort on relay_valid alone.

Real-hardware evidence (first two H2 attempts, 2026-09-10): both false-
aborted within <1ms of arming with relay_valid=True but relay_fresh=False,
relay_applied=False, and relay_torque_received_nm/relay_force_n exactly
[0,0,0] on every recorded sample of both runs. Root cause: touch_slave.cpp
sets command_.joint_torque_ext_valid=true ONCE in the joint_torque_ext_cmd
callback and never resets it (see touch_slave.cpp:750) -- a permanent,
process-lifetime latch, not a live-state flag. The project's real driver
processes have been running continuously since 2026-09-09 across many
sessions, including legitimate earlier C1/C4 bilateral_4ch.py runs, so
relay_valid reads True on effectively every real H2 attempt regardless of
current safety state.

Requires the ROS 2 workspace sourced (constructs a real rclpy Node, same
requirement as every other test in this directory) but does not spin or
touch any live topic -- purely exercises check_abort()'s pure decision
logic against constructed message objects.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from types import SimpleNamespace

import rclpy

sys.path.insert(0, str(Path(__file__).parent))
from part1_gravity_settle_capture import GravitySettleCapture  # noqa: E402


def _telem(relay_valid=False, relay_fresh=False, relay_applied=False,
          position_error_clamped=False, primary_force_clamped=False, final_force_clamped=False,
          spring=(0.0, 0.0, 0.0), damping=(0.0, 0.0, 0.0),
          enabled=False, deadman_ok=True):
    return SimpleNamespace(
        relay_valid=relay_valid, relay_fresh=relay_fresh, relay_applied=relay_applied,
        position_error_clamped=position_error_clamped, primary_force_clamped=primary_force_clamped,
        final_force_clamped=final_force_clamped,
        spring_force_n=list(spring), damping_force_n=list(damping),
        enabled=enabled, deadman_ok=deadman_ok,
    )


def _make_node():
    if not rclpy.ok():
        rclpy.init()
    node = GravitySettleCapture()
    return node


def test_stale_relay_valid_latch_alone_does_not_abort():
    """The exact real-hardware scenario: relay_valid=True (permanent latch),
    relay_fresh=False, relay_applied=False, zero relay force -- must NOT
    trigger an abort."""
    node = _make_node()
    try:
        now = time.time()
        node.latest["left"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False))
        node.latest["right"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False))
        assert node.check_abort(now) is False, node.abort_reason
    finally:
        node.destroy_node()


def test_relay_fresh_true_does_abort():
    """A GENUINE live relay stream (relay_fresh=True) must still abort --
    this is the real danger signal, not weakened by the fix above."""
    node = _make_node()
    try:
        now = time.time()
        node.latest["left"] = (now, _telem(relay_valid=True, relay_fresh=True, relay_applied=False))
        node.latest["right"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False))
        assert node.check_abort(now) is True
        assert "relay_fresh" in node.abort_reason
    finally:
        node.destroy_node()


def test_relay_applied_true_does_abort():
    """Proof a nonzero relayed force was actually summed must abort,
    regardless of relay_valid/relay_fresh."""
    node = _make_node()
    try:
        now = time.time()
        node.latest["left"] = (now, _telem(relay_valid=True, relay_fresh=True, relay_applied=True))
        node.latest["right"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False))
        assert node.check_abort(now) is True
        assert "relay_applied" in node.abort_reason
    finally:
        node.destroy_node()


def test_nominal_gravity_only_state_does_not_abort():
    """Sanity check: a genuinely clean gravity-only state (matching real
    H2 telemetry once the relay-latch false-positive is fixed) must pass
    check_abort() cleanly."""
    node = _make_node()
    try:
        now = time.time()
        node.latest["left"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False,
                                           enabled=True, deadman_ok=True))
        node.latest["right"] = (now, _telem(relay_valid=True, relay_fresh=False, relay_applied=False,
                                            enabled=True, deadman_ok=True))
        assert node.check_abort(now) is False, node.abort_reason
    finally:
        node.destroy_node()


if __name__ == "__main__":
    import pytest
    raise SystemExit(pytest.main([__file__, "-v"]))
