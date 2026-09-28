#!/usr/bin/env python3
"""Arm/disarm safety-semantics tests (2026-09-09, C2 arm-gate task).

Pure Python, no rclpy/hardware dependency -- drives TeleopStartupAlignment's
real state machine (startup_alignment.py) through WAIT_FOR_POSES -> ... ->
READY -> RAMPING -> ACTIVE -> (disarm) -> READY using a synthetic clock, and
combines it with the real compute_c2_target()/compute_c3_target() pure
functions (bridge_master_slave.py) the same way _tick()/_update_c2() do:

    full_target = compute_c{2,3}_target(...)
    published    = alignment.blend_{left_,}target(full_target)

This is deliberately the SAME composition the bridge uses, not a
reimplementation of it, so a change to either module cannot silently drift
out of sync with what this test asserts.

TEST A -- UNARMED: at ramp_alpha=0 (READY, not yet armed), both C2 and C3
    published targets must equal their OWN anchor exactly (zero relative
    motion -> zero spring error for a device sitting at its anchor),
    regardless of how large the anchor mismatch between master/slave is.
TEST B -- ARM TRANSITION: ramp_alpha must move monotonically and
    continuously from 0 to 1 over ramp_duration_s; no published-target
    jump exceeds what a continuous blend over that duration allows.
TEST C -- FULL ACTIVE: at ramp_alpha=1, absolute-axes C2/C3 targets equal
    the live slave/master position exactly (the frame fix, still intact).
TEST D -- DISARM: arm true -> false returns to READY with ramp_alpha=0 and
    both published targets equal to wherever the device now sits (not the
    stale startup anchor) -- no discontinuous pull.
TEST E -- MIXED AXES: x,z absolute / y relative, checked through the same
    arm/disarm cycle as A/C -- gating must not depend on which axes are
    absolute.
TEST F -- LARGE ANCHOR MISMATCH (~90mm, B2-trial scale): TEST A's zero-
    force guarantee must hold even at this size of mismatch -- this is
    exactly the scale of anchor gap that, pre-fix, produced the 91mm/100%-
    clamp C2 spring-force bug this whole investigation started from.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import compute_c2_target, compute_c3_target, apply_axis_map  # noqa: E402
from startup_alignment import (  # noqa: E402
    TeleopStartupAlignment, AlignmentConfig, READY, GRAVITY_RAMP_UP, ACTIVE, _ALL_RAMP_STATES,
)

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
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _mapper(pos):
    return apply_axis_map(IDENTITY_AXIS_MAP, [pos[i] * AXIS_SCALE[i] for i in range(3)])


def _drive_to_ready(left_pos, right_pos, clock, cfg=None):
    """Feed identical (still) poses until the state machine reaches READY.
    Mirrors what the bridge's own _left_cb/_right_cb + _tick() do every
    cycle; the poses are held constant, which is exactly HOLD_STILL's
    required condition, so this reaches READY deterministically."""
    align = TeleopStartupAlignment(config=cfg or AlignmentConfig(), now_fn=clock)
    for _ in range(400):
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        align.tick(_mapper)
        if align.state == READY:
            return align
        clock.advance(0.01)
    raise AssertionError(f"never reached READY, stuck at {align.state}")


def _published(align, master_pos, slave_pos, absolute_axes):
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


def test_a_unarmed_zero_force_safe():
    clock = FakeClock()
    left_pos = [0.0, 0.0, 0.0]
    right_pos = [0.05, -0.03, 0.02]  # deliberate mismatch, see test F for the >50mm case
    align = _drive_to_ready(left_pos, right_pos, clock)
    assert align.state == READY
    assert align.ramp_alpha == 0.0

    c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=ALL_ABSOLUTE)
    c2_err = norm([c2_pub[i] - align.left_anchor[i] for i in range(3)])
    c3_err = norm([c3_pub[i] - align.right_anchor[i] for i in range(3)])
    print(f"[A] READY, ramp_alpha=0: C2 published={c2_pub} left_anchor={align.left_anchor} err={c2_err*1000:.6f}mm")
    print(f"[A] READY, ramp_alpha=0: C3 published={c3_pub} right_anchor={align.right_anchor} err={c3_err*1000:.6f}mm")
    assert c2_err < 1e-9, f"C2 must publish exactly its own anchor while unarmed, err={c2_err*1000:.4f}mm"
    assert c3_err < 1e-9, f"C3 must publish exactly its own anchor while unarmed, err={c3_err*1000:.4f}mm"
    print("PASS\n")


def test_b_arm_transition_is_continuous():
    clock = FakeClock()
    left_pos = [0.02, 0.01, -0.01]
    right_pos = [0.06, -0.02, 0.03]
    align = _drive_to_ready(left_pos, right_pos, clock)
    align.arm()
    align.tick(_mapper)
    assert align.state == GRAVITY_RAMP_UP, f"expected GRAVITY_RAMP_UP right after arm(), got {align.state}"

    prev_c2, prev_c3 = _published(align, left_pos, right_pos, absolute_axes=ALL_ABSOLUTE)
    max_step_allowed = None
    steps = 0
    # 2026-09-09 staged-transition rewrite: the arm sequence now spans
    # GRAVITY_RAMP_UP -> GRAVITY_SETTLE -> COUPLING_RAMP_UP before ACTIVE.
    # align.ramp_alpha (== coupling_ramp_alpha) only starts moving in the
    # last of those, but stays non-decreasing across the whole sequence
    # (0 through the gravity phases), so this loop still spans exactly the
    # same "is a discontinuity ever introduced before ACTIVE" question.
    while align.state != ACTIVE:
        clock.advance(0.02)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        prev_alpha = align.ramp_alpha
        align.tick(_mapper)
        assert align.ramp_alpha >= prev_alpha - 1e-12, "ramp_alpha must be monotonically non-decreasing"
        c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=ALL_ABSOLUTE)
        step = max(norm([c2_pub[i] - prev_c2[i] for i in range(3)]),
                   norm([c3_pub[i] - prev_c3[i] for i in range(3)]))
        # A single 0.02s tick can move ramp_alpha by at most 0.02/ramp_duration_s
        # of the total travel; bound the published-target step by that fraction
        # of the largest possible travel (anchor-to-full-target distance).
        alpha_step = align.ramp_alpha - prev_alpha
        max_travel = max(
            norm([compute_c3_target(left_pos, align.left_anchor, align.right_anchor, IDENTITY_AXIS_MAP,
                                     AXIS_SCALE, DEADBAND_M, MAX_OFFSET, "axis",
                                     absolute_axes=ALL_ABSOLUTE, absolute_trim=ZERO_TRIM)[0][i]
                  - align.right_anchor[i] for i in range(3)]),
            norm([compute_c2_target(right_pos, align.right_anchor, align.left_anchor, IDENTITY_AXIS_MAP,
                                     AXIS_SCALE, LEFT_MAX_OFFSET_M,
                                     absolute_axes=ALL_ABSOLUTE, absolute_trim=ZERO_TRIM)[0][i]
                  - align.left_anchor[i] for i in range(3)]),
        )
        allowed = alpha_step * max_travel + 1e-9
        assert step <= allowed, (
            f"discontinuous jump during ramp: step={step*1000:.4f}mm > allowed={allowed*1000:.4f}mm "
            f"(alpha {prev_alpha:.3f}->{align.ramp_alpha:.3f})"
        )
        prev_c2, prev_c3 = c2_pub, c3_pub
        steps += 1
        assert steps < 1000, "ramp never completed"
    assert align.state == ACTIVE
    assert align.ramp_alpha == 1.0
    print(f"[B] ramp completed over {steps} ticks with no discontinuous jump. PASS\n")


def test_c_full_active_matches_absolute_frame_fix():
    clock = FakeClock()
    left_pos = [0.02, -0.03, 0.01]
    right_pos = [0.04, 0.01, -0.02]
    align = _drive_to_ready(left_pos, right_pos, clock)
    align.arm()
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly.
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)
    assert align.ramp_alpha == 1.0

    c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=ALL_ABSOLUTE)
    c3_err = norm([c3_pub[i] - left_pos[i] for i in range(3)])
    c2_err = norm([c2_pub[i] - right_pos[i] for i in range(3)])
    print(f"[C] ACTIVE: C3 published={c3_pub} master_pos={left_pos} err={c3_err*1000:.6f}mm")
    print(f"[C] ACTIVE: C2 published={c2_pub} slave_pos={right_pos} err={c2_err*1000:.6f}mm")
    assert c3_err < 1e-9, "ACTIVE C3 must equal live master position exactly (frame fix)"
    assert c2_err < 1e-9, "ACTIVE C2 must equal live slave position exactly (frame fix)"
    print("PASS\n")


def test_d_disarm_returns_to_current_pose_no_stale_pull():
    clock = FakeClock()
    left_pos = [0.02, -0.03, 0.01]
    right_pos = [0.04, 0.01, -0.02]
    align = _drive_to_ready(left_pos, right_pos, clock)
    align.arm()
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly.
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)

    # Device has physically moved during ACTIVE, away from the startup anchor.
    moved_left_pos = [0.20, 0.15, -0.10]
    moved_right_pos = [0.22, 0.17, -0.08]
    align.update_master_pose(moved_left_pos, t=clock.t)
    align.update_slave_pose(moved_right_pos, t=clock.t)
    align.tick(_mapper)

    align.disarm()
    # 2026-09-09 staged-transition rewrite: disarm is no longer a single-tick
    # transition -- it stages COUPLING_RAMP_DOWN -> GRAVITY_RAMP_DOWN -> READY
    # (see startup_alignment.py's module docstring). Drive it to completion.
    steps = 0
    while align.state != READY:
        clock.advance(0.05)
        align.update_master_pose(moved_left_pos, t=clock.t)
        align.update_slave_pose(moved_right_pos, t=clock.t)
        align.tick(_mapper)
        steps += 1
        assert steps < 1000, "disarm sequence never reached READY"
    assert align.ramp_alpha == 0.0

    c2_pub, c3_pub = _published(align, moved_left_pos, moved_right_pos, absolute_axes=ALL_ABSOLUTE)
    # Must hold at the CURRENT (post-motion) pose, not the stale original anchor.
    # C2 publishes to the master, held at left_anchor; C3 to the slave, held at right_anchor.
    c2_err = norm([c2_pub[i] - moved_left_pos[i] for i in range(3)])
    c3_err = norm([c3_pub[i] - moved_right_pos[i] for i in range(3)])
    stale_c2_err = norm([c2_pub[i] - left_pos[i] for i in range(3)])
    print(f"[D] disarmed at moved pose: C2 (->master) published={c2_pub} current_master_pos={moved_left_pos} err={c2_err*1000:.6f}mm")
    print(f"[D] disarmed at moved pose: C3 (->slave) published={c3_pub} current_slave_pos={moved_right_pos} err={c3_err*1000:.6f}mm")
    assert c2_err < 1e-9, "disarm must re-anchor to the CURRENT master pose, not pull toward the stale startup anchor"
    assert c3_err < 1e-9, "disarm must re-anchor to the CURRENT slave pose, not pull toward the stale startup anchor"
    assert stale_c2_err > 0.05, "test fixture should have moved the device meaningfully away from the old anchor"
    print("PASS\n")


def test_e_mixed_axes_gating_still_works():
    absolute_axes = [0, 2]  # x, z absolute; y relative
    clock = FakeClock()
    left_pos = [0.02, -0.03, 0.01]
    right_pos = [0.10, 0.01, -0.02]  # deliberately large y gap to also exercise the C2 y clamp
    align = _drive_to_ready(left_pos, right_pos, clock)

    # Unarmed: zero-force safe on both channels regardless of which axes are absolute.
    c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=absolute_axes)
    assert norm([c2_pub[i] - align.left_anchor[i] for i in range(3)]) < 1e-9
    assert norm([c3_pub[i] - align.right_anchor[i] for i in range(3)]) < 1e-9
    print("[E] unarmed, mixed axes: both channels zero-force-safe. PASS")

    align.arm()
    while align.state != ACTIVE:
        clock.advance(0.05)
        align.update_master_pose(left_pos, t=clock.t)
        align.update_slave_pose(right_pos, t=clock.t)
        # Poses held fixed -> zero velocity, satisfying GRAVITY_SETTLE's
        # 2026-09-09 velocity-gated quiet-hold promptly.
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)

    c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=absolute_axes)
    assert abs(c3_pub[0] - left_pos[0]) < 1e-9 and abs(c3_pub[2] - left_pos[2]) < 1e-9
    assert abs(c2_pub[0] - right_pos[0]) < 1e-9 and abs(c2_pub[2] - right_pos[2]) < 1e-9
    print(f"[E] ACTIVE, mixed axes: C3={c3_pub} C2={c2_pub} (x,z absolute; y relative+clamped). PASS\n")


def test_f_large_anchor_mismatch_still_zero_force_unarmed():
    """~90mm mismatch, matching the real B2 trial's scale -- the exact
    situation that, pre-fix, produced a 91mm/100%-clamp C2 spring force."""
    clock = FakeClock()
    left_pos = [0.03659441350635732, -0.0290291903897336, -0.02598308753967285]
    right_pos = [-0.0015765916532591738, -0.11049050431502506, -0.04055201796481485]
    gap = norm([left_pos[i] - right_pos[i] for i in range(3)])
    assert gap > 0.085, f"test fixture should reproduce the ~90mm B2-scale gap, got {gap*1000:.1f}mm"

    align = _drive_to_ready(left_pos, right_pos, clock)
    assert align.state == READY
    c2_pub, c3_pub = _published(align, left_pos, right_pos, absolute_axes=ALL_ABSOLUTE)
    c2_err = norm([c2_pub[i] - align.left_anchor[i] for i in range(3)])
    c3_err = norm([c3_pub[i] - align.right_anchor[i] for i in range(3)])
    print(f"[F] {gap*1000:.1f}mm anchor gap, unarmed: C2 err={c2_err*1000:.6f}mm C3 err={c3_err*1000:.6f}mm")
    assert c2_err < 1e-9, "large anchor mismatch must NOT create a pre-arm C2 spring preload"
    assert c3_err < 1e-9, "large anchor mismatch must NOT create a pre-arm C3 spring preload"
    print("PASS\n")


if __name__ == "__main__":
    test_a_unarmed_zero_force_safe()
    test_b_arm_transition_is_continuous()
    test_c_full_active_matches_absolute_frame_fix()
    test_d_disarm_returns_to_current_pose_no_stale_pull()
    test_e_mixed_axes_gating_still_works()
    test_f_large_anchor_mismatch_still_zero_force_unarmed()
    print("ALL TESTS PASSED")
