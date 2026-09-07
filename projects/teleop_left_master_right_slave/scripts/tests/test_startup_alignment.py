"""Pure-Python tests for startup_alignment.py -- no ROS/rclpy, no hardware,
matching this project's established "framework-free where the logic itself
has no ROS dependency" convention (see sim/tests/test_oscillation_guard.py
for the same pattern applied to bilateral_4ch.py's analyze_window()).

Uses a FakeClock so every test is deterministic and instant to run -- no
real time.sleep() anywhere.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from startup_alignment import (  # noqa: E402
    ACTIVE, CAPTURE_ANCHORS, FAULT, HOLD_STILL, READY, RAMPING,
    VALIDATE_ALIGNMENT, WAIT_FOR_POSES,
    AlignmentConfig, TeleopStartupAlignment,
)


class FakeClock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


IDENTITY_MAP = lambda pos: list(pos)  # noqa: E731


def _fast_config(**overrides) -> AlignmentConfig:
    cfg = AlignmentConfig(
        pose_stale_s=0.2, hold_still_motion_m=0.003, hold_still_duration_s=0.5,
        max_alignment_error_m=0.15, ramp_duration_s=1.0,
    )
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def _settle_to_ready(align: TeleopStartupAlignment, clock: FakeClock,
                     master_pos=(0.0, 0.0, 0.0), slave_pos=(0.0, 0.0, 0.0),
                     step_s: float = 0.05) -> None:
    """Feeds stable poses until READY, ticking in step_s increments."""
    total = 0.0
    while align.state != READY and align.state != FAULT and total < 5.0:
        align.update_master_pose(list(master_pos), t=clock.t)
        align.update_slave_pose(list(slave_pos), t=clock.t)
        align.tick(IDENTITY_MAP)
        clock.advance(step_s)
        total += step_s


# ---------------------------------------------------------------------------
# 1. No anchor capture on first pose message.
# ---------------------------------------------------------------------------
def test_no_anchor_capture_on_first_pose_message():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    align.update_master_pose([0.0, 0.0, 0.0], t=clock.t)
    align.update_slave_pose([0.0, 0.0, 0.0], t=clock.t)
    align.tick(IDENTITY_MAP)
    assert align.state == HOLD_STILL, (
        "a single pose pair must move to HOLD_STILL, not capture an anchor "
        "immediately -- capture only happens after CAPTURE_ANCHORS, which "
        "requires a full stable hold window first"
    )
    assert align.left_anchor is None
    assert align.right_anchor is None


# ---------------------------------------------------------------------------
# 2. Moving master prevents capture.
# ---------------------------------------------------------------------------
def test_moving_master_prevents_capture():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    total = 0.0
    x = 0.0
    while total < 2.0:
        x += 0.01  # 10mm/tick -- far above hold_still_motion_m (3mm)
        align.update_master_pose([x, 0.0, 0.0], t=clock.t)
        align.update_slave_pose([0.0, 0.0, 0.0], t=clock.t)
        align.tick(IDENTITY_MAP)
        clock.advance(0.05)
        total += 0.05
        assert align.state != CAPTURE_ANCHORS, "moving master must never reach CAPTURE_ANCHORS"
        assert align.state != READY
    assert align.state == HOLD_STILL


# ---------------------------------------------------------------------------
# 3. Moving slave prevents capture.
# ---------------------------------------------------------------------------
def test_moving_slave_prevents_capture():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    total = 0.0
    y = 0.0
    while total < 2.0:
        y += 0.01
        align.update_master_pose([0.0, 0.0, 0.0], t=clock.t)
        align.update_slave_pose([0.0, y, 0.0], t=clock.t)
        align.tick(IDENTITY_MAP)
        clock.advance(0.05)
        total += 0.05
        assert align.state != CAPTURE_ANCHORS
        assert align.state != READY
    assert align.state == HOLD_STILL


# ---------------------------------------------------------------------------
# 4. Stable poses produce anchors.
# ---------------------------------------------------------------------------
def test_stable_poses_produce_anchors():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    _settle_to_ready(align, clock, master_pos=(0.01, 0.02, 0.03), slave_pos=(0.05, 0.06, 0.07))
    assert align.state == READY, f"expected READY, got {align.state}"
    assert align.left_anchor is not None
    assert align.right_anchor is not None
    for i, expect in enumerate((0.01, 0.02, 0.03)):
        assert abs(align.left_anchor[i] - expect) < 1e-6
    for i, expect in enumerate((0.05, 0.06, 0.07)):
        assert abs(align.right_anchor[i] - expect) < 1e-6


# ---------------------------------------------------------------------------
# 5. Stale samples prevent READY.
# ---------------------------------------------------------------------------
def test_stale_samples_prevent_ready():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    _settle_to_ready(align, clock)
    assert align.state == READY

    # Now go silent -- no more update_*_pose calls, just let time pass and
    # keep ticking. The next tick must detect staleness and fault, not
    # silently keep reporting READY on data that's no longer fresh.
    clock.advance(1.0)  # far beyond pose_stale_s=0.2
    align.tick(IDENTITY_MAP)
    assert align.state == FAULT, "stale poses must fault, not remain READY"


# ---------------------------------------------------------------------------
# 6. Failed validation prevents arm.
# ---------------------------------------------------------------------------
def test_failed_validation_prevents_arm():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(), now_fn=clock)
    # Slave anchor 0.5m away from mapped master anchor -- comfortably above
    # the 0.15m sanity bound, simulating a gross misconfiguration.
    _settle_to_ready(align, clock, master_pos=(0.0, 0.0, 0.0), slave_pos=(0.5, 0.0, 0.0))
    assert align.state == FAULT, f"expected FAULT on gross misalignment, got {align.state}"
    assert align.initial_alignment_error_norm is not None
    assert align.initial_alignment_error_norm > 0.15

    # arm() must be a no-op from FAULT -- no auto-recovery.
    align.arm()
    align.tick(IDENTITY_MAP)
    assert align.state == FAULT, "arm() must not clear a FAULT; only reset() may"

    align.reset()
    assert align.state == WAIT_FOR_POSES


# ---------------------------------------------------------------------------
# 7 & 8. Ramp starts at zero and reaches one monotonically.
# ---------------------------------------------------------------------------
def test_ramp_starts_at_zero_and_reaches_one_monotonically():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(ramp_duration_s=1.0), now_fn=clock)
    _settle_to_ready(align, clock)
    assert align.state == READY
    assert align.ramp_alpha == 0.0

    align.arm()
    align.tick(IDENTITY_MAP)  # consumes the arm request, enters RAMPING
    assert align.state == RAMPING
    assert align.ramp_alpha == 0.0, "ramp must start at exactly 0, not jump"

    prev_alpha = 0.0
    samples = []
    total = 0.0
    while align.state == RAMPING and total < 2.0:
        clock.advance(0.05)
        align.update_master_pose([0.0, 0.0, 0.0], t=clock.t)
        align.update_slave_pose([0.0, 0.0, 0.0], t=clock.t)
        align.tick(IDENTITY_MAP)
        assert align.ramp_alpha >= prev_alpha - 1e-9, "ramp_alpha must be monotonically non-decreasing"
        prev_alpha = align.ramp_alpha
        samples.append(align.ramp_alpha)
        total += 0.05

    assert align.state == ACTIVE, f"ramp must complete into ACTIVE, ended in {align.state}"
    assert align.ramp_alpha == 1.0
    assert len(samples) > 1, "ramp should take multiple ticks, not jump in one"


# ---------------------------------------------------------------------------
# 9. No target discontinuity at READY -> RAMPING.
# ---------------------------------------------------------------------------
def test_no_target_discontinuity_at_ready_to_ramping():
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(ramp_duration_s=1.0), now_fn=clock)
    _settle_to_ready(align, clock, master_pos=(0.0, 0.0, 0.0), slave_pos=(0.03, 0.02, 0.01))
    assert align.state == READY

    full_target = [0.5, 0.3, 0.1]  # a large hypothetical mapped target, far from the anchor

    # Blended target the instant BEFORE arming (alpha still 0 from READY).
    target_before = align.blend_target(full_target)
    assert target_before == align.right_anchor, (
        "before arming, the blended target must equal right_anchor exactly "
        "(zero commanded relative motion)"
    )

    align.arm()
    align.tick(IDENTITY_MAP)  # -> RAMPING, alpha still 0.0 on the very first tick
    assert align.state == RAMPING
    target_at_ramp_start = align.blend_target(full_target)

    for i in range(3):
        assert abs(target_at_ramp_start[i] - target_before[i]) < 1e-9, (
            f"axis {i}: target jumped from {target_before[i]} to "
            f"{target_at_ramp_start[i]} at the READY->RAMPING transition"
        )


# ---------------------------------------------------------------------------
# 10. ACTIVE uses the same existing anchor-relative mapping as before.
# ---------------------------------------------------------------------------
def test_active_uses_existing_anchor_relative_mapping():
    """At alpha=1 (ACTIVE), blend_target must return the pre-existing
    formula's output UNCHANGED -- this module must not alter steady-state
    behavior, only how the anchors were captured and how quickly authority
    ramps in."""
    clock = FakeClock()
    align = TeleopStartupAlignment(_fast_config(ramp_duration_s=0.2), now_fn=clock)
    _settle_to_ready(align, clock, master_pos=(0.0, 0.0, 0.0), slave_pos=(0.03, 0.02, 0.01))
    align.arm()
    total = 0.0
    while align.state != ACTIVE and total < 2.0:
        clock.advance(0.05)
        align.update_master_pose([0.0, 0.0, 0.0], t=clock.t)
        align.update_slave_pose([0.03, 0.02, 0.01], t=clock.t)
        align.tick(IDENTITY_MAP)
        total += 0.05
    assert align.state == ACTIVE

    # Simulate the bridge's own formula: target = right_anchor + mapped(delta)
    delta = [0.03, -0.01, 0.02]
    mapped_delta = IDENTITY_MAP(delta)  # identity axis_map/scale in this test
    full_target = [align.right_anchor[i] + mapped_delta[i] for i in range(3)]

    blended = align.blend_target(full_target)
    assert blended == full_target, (
        "ACTIVE (alpha=1) must reproduce the existing anchor-relative "
        "formula exactly, not a modified version of it"
    )


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
