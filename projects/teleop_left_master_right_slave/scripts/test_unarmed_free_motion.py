#!/usr/bin/env python3
"""Unarmed-free-motion tests (2026-09-09), Section 6 A-G of the follow-up
force-contract task. Pure Python, no rclpy/hardware dependency.

Prior finding: while unarmed (READY), blend_target()/blend_left_target()
returned a point FROZEN at anchor capture (or, briefly, at disarm). That is
zero-force-safe only as long as the device never moves from that frozen
point. Real hardware showed it does move (the slave settling once its
gravity-comp aux force disappeared at disarm), regenerating a real spring
force (~0.078N) minutes after the operator considered the system "off".

Fix: blend_target()/blend_left_target() now track the device's LIVE pose
continuously whenever unarmed (READY/FAULT), and RAMPING blends from a
fresh pose snapshot taken at the READY->RAMPING transition (_ramp_origin_*),
not the stale startup anchor.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import compute_c2_target, compute_c3_target, apply_axis_map  # noqa: E402
from startup_alignment import TeleopStartupAlignment, AlignmentConfig, READY, GRAVITY_RAMP_UP, ACTIVE  # noqa: E402

IDENTITY_AXIS_MAP = [(0, 1.0), (1, 1.0), (2, 1.0)]
AXIS_SCALE = [1.0, 1.0, 1.0]
MAX_OFFSET = [0.12, 0.19, 0.15]
LEFT_MAX_OFFSET_M = 0.08
DEADBAND_M = 0.0002
ALL_ABSOLUTE = [0, 1, 2]
ZERO_TRIM = [0.0, 0.0, 0.0]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


class FakeClock:
    def __init__(self):
        self.t = 5000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _mapper(pos):
    return apply_axis_map(IDENTITY_AXIS_MAP, [pos[i] * AXIS_SCALE[i] for i in range(3)])


def _drive_to_ready(left_pos, right_pos, clock, cfg=None):
    align = TeleopStartupAlignment(config=cfg or AlignmentConfig(), now_fn=clock)
    for _ in range(400):
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        align.tick(_mapper)
        if align.state == READY:
            return align
        clock.advance(0.01)
    raise AssertionError(f"never reached READY, stuck at {align.state}")


def _published(align, master_pos, slave_pos, absolute_axes=ALL_ABSOLUTE):
    c3_full, _ = compute_c3_target(
        master_pos, align.left_anchor, align.right_anchor, IDENTITY_AXIS_MAP, AXIS_SCALE,
        DEADBAND_M, MAX_OFFSET, "axis", absolute_axes=absolute_axes, absolute_trim=ZERO_TRIM,
    )
    c2_full, _ = compute_c2_target(
        slave_pos, align.right_anchor, align.left_anchor, IDENTITY_AXIS_MAP, AXIS_SCALE,
        LEFT_MAX_OFFSET_M, absolute_axes=absolute_axes, absolute_trim=ZERO_TRIM,
    )
    c3_pub = align.blend_target(c3_full)
    c2_pub = align.blend_left_target(c2_full)
    return c2_pub, c3_pub


def test_a_ready_master_motion_target_follows():
    """Move the master 50mm while unarmed (READY). Target must FOLLOW the
    live master pose (not stay pinned at the stale anchor), so C2's own
    error (target - master_actual) stays ~0."""
    clock = FakeClock()
    left0 = [0.0, 0.0, 0.0]
    right0 = [0.05, -0.03, 0.02]
    align = _drive_to_ready(left0, right0, clock)

    moved_left = [0.05, 0.0, 0.0]  # +50mm on x
    align.update_master_pose(moved_left, t=clock.t)
    align.update_slave_pose(right0, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    c2_pub, _ = _published(align, moved_left, right0)
    err = norm([c2_pub[i] - moved_left[i] for i in range(3)])
    print(f"[A] master moved to {moved_left}, C2 published={c2_pub}, err={err*1000:.6f}mm")
    assert err < 1e-6, f"C2 target must follow the moved master pose, err={err*1000:.4f}mm"
    print("PASS\n")


def test_b_ready_slave_motion_target_follows():
    """Symmetric to A: move the slave while unarmed, C3's target must
    follow it (this is exactly the real-hardware Trial-1 finding -- the
    slave settling once gravity-comp dropped at disarm)."""
    clock = FakeClock()
    left0 = [0.0, 0.0, 0.0]
    right0 = [0.05, -0.03, 0.02]
    align = _drive_to_ready(left0, right0, clock)

    moved_right = [right0[0], right0[1] - 0.01, right0[2]]  # 10mm droop on y
    align.update_master_pose(left0, t=clock.t)
    align.update_slave_pose(moved_right, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    _, c3_pub = _published(align, left0, moved_right)
    err = norm([c3_pub[i] - moved_right[i] for i in range(3)])
    print(f"[B] slave moved to {moved_right}, C3 published={c3_pub}, err={err*1000:.6f}mm")
    assert err < 1e-6, f"C3 target must follow the moved slave pose, err={err*1000:.4f}mm"
    print("PASS\n")


def test_c_large_anchor_mismatch_after_motion_no_preload():
    """Move BOTH devices after READY, with a large (>50mm) startup anchor
    mismatch already on record. No spring preload may appear on either
    side -- confirms the fix does not depend on the anchors being close."""
    clock = FakeClock()
    # >50mm per the protocol, but under AlignmentConfig's default 0.15m
    # max_alignment_error_m sanity bound (a genuinely oversized gap is
    # SUPPOSED to FAULT -- that gate is correct, separate behavior, not
    # what this test is checking).
    left0 = [0.08, -0.04, 0.02]
    right0 = [0.02, 0.03, -0.04]
    gap = norm([left0[i] - right0[i] for i in range(3)])
    assert 0.05 < gap < 0.15, "test fixture should use a >50mm but sub-FAULT-threshold gap"
    align = _drive_to_ready(left0, right0, clock)

    moved_left = [0.15, -0.02, 0.05]
    moved_right = [0.05, 0.08, -0.03]
    align.update_master_pose(moved_left, t=clock.t)
    align.update_slave_pose(moved_right, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    c2_pub, c3_pub = _published(align, moved_left, moved_right)
    c2_err = norm([c2_pub[i] - moved_left[i] for i in range(3)])
    c3_err = norm([c3_pub[i] - moved_right[i] for i in range(3)])
    print(f"[C] large anchor gap ({gap*1000:.0f}mm) + post-READY motion: "
          f"C2 err={c2_err*1000:.6f}mm C3 err={c3_err*1000:.6f}mm")
    assert c2_err < 1e-6 and c3_err < 1e-6, "no spring preload may appear regardless of anchor mismatch"
    print("PASS\n")


def test_d_arm_transition_captures_current_pose_no_discontinuity():
    """Arm AFTER the operator has moved the device away from the startup
    anchor. The ramp must start from the CURRENT pose (fresh snapshot), not
    jump back toward the stale anchor first."""
    clock = FakeClock()
    left0 = [0.0, 0.0, 0.0]
    right0 = [0.05, -0.03, 0.02]
    align = _drive_to_ready(left0, right0, clock)

    moved_left = [0.08, 0.02, -0.01]
    moved_right = [0.12, -0.10, 0.06]
    align.update_master_pose(moved_left, t=clock.t)
    align.update_slave_pose(moved_right, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    c2_before, c3_before = _published(align, moved_left, moved_right)

    align.arm()
    align.tick(_mapper)
    # 2026-09-09 staged-transition rewrite: arm() now enters GRAVITY_RAMP_UP
    # first (coupling_ramp_alpha, i.e. legacy ramp_alpha, stays 0 through
    # the whole gravity phase) -- the discontinuity question this test is
    # about is unaffected, since blend_target/blend_left_target still use
    # coupling_ramp_alpha==0 here, giving target==live pose exactly.
    assert align.state == GRAVITY_RAMP_UP
    assert align.ramp_alpha == 0.0
    c2_after, c3_after = _published(align, moved_left, moved_right)

    jump_c2 = norm([c2_after[i] - c2_before[i] for i in range(3)])
    jump_c3 = norm([c3_after[i] - c3_before[i] for i in range(3)])
    print(f"[D] arm at moved pose: pre-arm target={c2_before}/{c3_before}, "
          f"post-arm (alpha=0) target={c2_after}/{c3_after}, jump={jump_c2*1000:.6f}/{jump_c3*1000:.6f}mm")
    assert jump_c2 < 1e-6 and jump_c3 < 1e-6, "ramp must start from the current pose, no discontinuity"
    print("PASS\n")


def test_e_active_absolute_coupling_unchanged():
    """ACTIVE semantics must be byte-for-byte unchanged: C2 target ==
    live slave pose, C3 target == live master pose, for absolute axes."""
    clock = FakeClock()
    left0 = [0.02, -0.03, 0.01]
    right0 = [0.04, 0.01, -0.02]
    align = _drive_to_ready(left0, right0, clock)
    align.arm()
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly (fake-clock unit
        # test, not a plant-dynamics test).
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)

    c2_pub, c3_pub = _published(align, left0, right0)
    c3_err = norm([c3_pub[i] - left0[i] for i in range(3)])
    c2_err = norm([c2_pub[i] - right0[i] for i in range(3)])
    print(f"[E] ACTIVE: C3 err={c3_err*1000:.6f}mm C2 err={c2_err*1000:.6f}mm")
    assert c3_err < 1e-9 and c2_err < 1e-9
    print("PASS\n")


def test_f_disarm_then_motion_target_follows_no_spring_regeneration():
    """The exact real-hardware scenario: disarm, then the device moves
    (e.g. settles under gravity). Target must follow, so spring_force does
    NOT regenerate -- this is the direct regression test for the ~0.078N
    post-disarm force observed on real hardware."""
    clock = FakeClock()
    left0 = [0.02, -0.03, 0.01]
    right0 = [0.04, 0.01, -0.02]
    align = _drive_to_ready(left0, right0, clock)
    align.arm()
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly (fake-clock unit
        # test, not a plant-dynamics test).
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)

    align.disarm()
    # 2026-09-09 staged-transition rewrite: disarm is staged (COUPLING_RAMP_
    # DOWN -> GRAVITY_RAMP_DOWN -> READY), not a single tick -- drive it to
    # completion. The devices are held steady (left0/right0) through this,
    # matching "slave settles AFTER disarm completes" below.
    steps = 0
    while align.state != READY:
        clock.advance(0.05)
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        align.tick(_mapper)
        steps += 1
        assert steps < 1000, "disarm sequence never reached READY"
    assert align.ramp_alpha == 0.0

    # Slave settles under gravity now that aux force is gone -- moves 8mm on y.
    settled_right = [right0[0], right0[1] - 0.008, right0[2]]
    align.update_master_pose(left0, t=clock.t)
    align.update_slave_pose(settled_right, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    _, c3_pub = _published(align, left0, settled_right)
    err = norm([c3_pub[i] - settled_right[i] for i in range(3)])
    print(f"[F] disarmed, slave settled to {settled_right}: C3 published={c3_pub}, err={err*1000:.6f}mm")
    assert err < 1e-6, (
        f"target must follow the settled slave pose post-disarm, err={err*1000:.4f}mm -- "
        "otherwise spring_force regenerates, reproducing the real-hardware ~0.078N finding"
    )
    print("PASS\n")


def test_g_gravity_ramp_unaffected():
    """Gravity-comp ramp contract (from the previous task) must be
    untouched by this fix: 0 in READY, alpha-scaled in RAMPING, configured
    value in ACTIVE, 0 after disarm."""
    sys.path.insert(0, str(Path(__file__).parent))
    from bridge_master_slave import ramped_gravity_comp_n

    clock = FakeClock()
    left0 = [0.0, 0.0, 0.0]
    right0 = [0.05, -0.03, 0.02]
    align = _drive_to_ready(left0, right0, clock)
    assert ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha) == 0.0
    print(f"[G] READY: gravity={ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)} PASS")

    align.arm()
    align.tick(_mapper)
    # 2026-09-09 staged-transition rewrite: gravity now ramps in its OWN
    # phase (GRAVITY_RAMP_UP), using gravity_ramp_alpha, before coupling
    # (legacy ramp_alpha) ever moves -- see startup_alignment.py's docstring.
    assert align.state == GRAVITY_RAMP_UP
    saw_partial = False
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly (fake-clock unit
        # test, not a plant-dynamics test).
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)
        if 0.0 < align.gravity_ramp_alpha < 1.0:
            saw_partial = True
            g = ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)
            assert abs(g - 0.55 * align.gravity_ramp_alpha) < 1e-9
    assert saw_partial
    assert ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha) == 0.55
    print(f"[G] ACTIVE: gravity={ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)} PASS")

    align.disarm()
    steps = 0
    while align.state != READY:
        clock.advance(0.05)
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        align.tick(_mapper)
        steps += 1
        assert steps < 1000, "disarm sequence never reached READY"
    assert align.gravity_ramp_alpha == 0.0
    assert ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha) == 0.0
    print(f"[G] DISARM: gravity={ramped_gravity_comp_n(0.55, align.gravity_ramp_alpha)} PASS\n")


if __name__ == "__main__":
    test_a_ready_master_motion_target_follows()
    test_b_ready_slave_motion_target_follows()
    test_c_large_anchor_mismatch_after_motion_no_preload()
    test_d_arm_transition_captures_current_pose_no_discontinuity()
    test_e_active_absolute_coupling_unchanged()
    test_f_disarm_then_motion_target_follows_no_spring_regeneration()
    test_g_gravity_ramp_unaffected()
    print("ALL TESTS PASSED")
