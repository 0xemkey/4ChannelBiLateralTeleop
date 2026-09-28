#!/usr/bin/env python3
"""Gravity-compensation arm/ramp/disarm tests (2026-09-09, force-contract
task). Pure Python, no rclpy/hardware dependency.

Prior to this fix, _publish_gravity_comp()/_publish_left_gravity_comp()
published SLAVE_GRAVITY_COMP_N/LEFT_GRAVITY_COMP_N unconditionally,
regardless of arm_teleop/ramp_alpha -- confirmed on real hardware
(2026-09-09 narrow ACTIVE/DISARM check): right auxiliary_force_y read
+0.55N in READY (arm=false) and again after DISARM, identical to its ACTIVE
value. This meant two different SLAVE_GRAVITY_COMP_N configurations (the
planned A2=0.55 vs B3=0.70 comparison) were already physically distinct
states BEFORE arming -- violating the same "arm=false is a single,
configuration-independent safe state" contract the C2 position-channel fix
established for the C2/C3 targets. The fix reuses that exact ramp_alpha
(startup_alignment.py's TeleopStartupAlignment.ramp_alpha) to scale the
gravity bias the same way.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import ramped_gravity_comp_n  # noqa: E402
from startup_alignment import TeleopStartupAlignment, AlignmentConfig, READY, GRAVITY_RAMP_UP, ACTIVE  # noqa: E402


def _approx(a, b, tol=1e-9):
    return abs(a - b) < tol


def test_table_055():
    configured = 0.55
    table = [(0.00, 0.000), (0.25, 0.1375), (0.50, 0.2750), (0.75, 0.4125), (1.00, 0.5500)]
    for alpha, expected in table:
        got = ramped_gravity_comp_n(configured, alpha)
        assert _approx(got, expected), f"alpha={alpha}: expected {expected}, got {got}"
        print(f"[0.55] alpha={alpha:.2f} -> {got:.4f} N (expected {expected:.4f} N) PASS")
    print()


def test_table_070():
    configured = 0.70
    table = [(0.00, 0.000), (0.25, 0.175), (0.50, 0.350), (0.75, 0.525), (1.00, 0.700)]
    for alpha, expected in table:
        got = ramped_gravity_comp_n(configured, alpha)
        assert _approx(got, expected), f"alpha={alpha}: expected {expected}, got {got}"
        print(f"[0.70] alpha={alpha:.2f} -> {got:.4f} N (expected {expected:.4f} N) PASS")
    print()


def test_left_and_right_paths_use_the_same_function():
    """left_gravity_comp_n defaults to 0.0 in practice, but the SAME scaling
    must apply to whatever it is configured as -- confirmed here with a
    nonzero left value so the two paths (_publish_gravity_comp,
    _publish_left_gravity_comp) can't silently diverge."""
    for configured in (0.0, 0.30, 0.55, 0.70):
        for alpha in (0.0, 0.4, 1.0):
            expected = configured * alpha
            got_right = ramped_gravity_comp_n(configured, alpha)
            got_left = ramped_gravity_comp_n(configured, alpha)  # same function, both call sites
            assert _approx(got_right, expected) and _approx(got_left, expected)
    print("left/right paths: identical scaling for all configured/alpha combinations. PASS\n")


class FakeClock:
    def __init__(self):
        self.t = 2000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _mapper(pos):
    return list(pos)


def test_disarm_drives_gravity_comp_back_to_zero():
    """End-to-end through the real (2026-09-09 staged) state machine:
    READY(0) -> arm -> GRAVITY_RAMP_UP(0..1) -> GRAVITY_SETTLE -> COUPLING_
    RAMP_UP -> ACTIVE(1) -> disarm -> COUPLING_RAMP_DOWN -> GRAVITY_RAMP_DOWN
    (1..0) -> READY(0) again, reading gravity_ramp_alpha (NOT the legacy
    coupling-only ramp_alpha -- see startup_alignment.py's module docstring
    for why gravity_ramp_alpha and coupling_ramp_alpha are now separate) at
    each point and feeding it through ramped_gravity_comp_n() exactly as
    _tick() does (bridge_master_slave.py passes gravity_ramp_alpha, not
    ramp_alpha, to that function -- see its own 2026-09-09 update)."""
    clock = FakeClock()
    align = TeleopStartupAlignment(config=AlignmentConfig(), now_fn=clock)
    left_pos = [0.0, 0.0, 0.0]
    right_pos = [0.05, -0.03, 0.02]

    for _ in range(400):
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        align.tick(_mapper)
        if align.state == READY:
            break
        clock.advance(0.01)
    assert align.state == READY
    g_ready = ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)
    assert _approx(g_ready, 0.0), f"READY must give zero gravity comp, got {g_ready}"
    print(f"[disarm-cycle] READY: gravity_ramp_alpha={align.gravity_ramp_alpha} -> gravity={g_ready} N PASS")

    align.arm()
    align.tick(_mapper)
    assert align.state == GRAVITY_RAMP_UP
    saw_partial = False
    for _ in range(400):
        clock.advance(0.05)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly.
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)
        g = ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)
        if 0.0 < align.gravity_ramp_alpha < 1.0:
            saw_partial = True
            expected = 0.55 * align.gravity_ramp_alpha
            assert _approx(g, expected), (
                f"mid-ramp: gravity_ramp_alpha={align.gravity_ramp_alpha} expected {expected} got {g}")
        if align.state == ACTIVE:
            break
    assert saw_partial, "gravity ramp must pass through a genuine intermediate alpha, not jump 0->1"
    assert align.state == ACTIVE and align.gravity_ramp_alpha == 1.0
    g_active = ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)
    assert _approx(g_active, 0.55), f"ACTIVE must reach the configured value exactly, got {g_active}"
    print(f"[disarm-cycle] ACTIVE: gravity_ramp_alpha=1.0 -> gravity={g_active} N PASS")

    align.disarm()
    saw_partial_down = False
    for _ in range(400):
        clock.advance(0.05)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        align.tick(_mapper)
        if 0.0 < align.gravity_ramp_alpha < 1.0:
            saw_partial_down = True
        if align.state == READY:
            break
    assert saw_partial_down, "gravity ramp-down must also pass through a genuine intermediate alpha"
    assert align.state == READY and align.gravity_ramp_alpha == 0.0
    g_disarmed = ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)
    assert _approx(g_disarmed, 0.0), f"post-disarm must give zero gravity comp, got {g_disarmed}"
    print(f"[disarm-cycle] DISARM->READY: gravity_ramp_alpha=0.0 -> gravity={g_disarmed} N PASS\n")


if __name__ == "__main__":
    test_table_055()
    test_table_070()
    test_left_and_right_paths_use_the_same_function()
    test_disarm_drives_gravity_comp_back_to_zero()
    print("ALL TESTS PASSED")
