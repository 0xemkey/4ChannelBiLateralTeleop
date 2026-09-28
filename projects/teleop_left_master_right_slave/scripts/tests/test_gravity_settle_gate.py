"""Part 4 of the 2026-09-09 validation-infrastructure follow-up task:
dedicated pure tests for GRAVITY_SETTLE's velocity-gated quiet-hold logic
(see startup_alignment.py's GRAVITY_SETTLE tick() branch and its module
docstring for the root-cause finding this replaces a fixed 0.2s clock with).

Pure Python, FakeClock-driven, no ROS/rclpy -- matches test_startup_alignment.py's
own convention in this same directory.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from startup_alignment import (  # noqa: E402
    ACTIVE, FAULT, GRAVITY_RAMP_UP, GRAVITY_SETTLE, COUPLING_RAMP_UP, READY,
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

# Small, fast-test-friendly settle params -- deliberately DIFFERENT numbers
# from the production defaults (0.005 m/s / 0.15s / 0.1s / 10.0s) so these
# tests are exercising the GATE LOGIC itself, not memorized production
# constants.
VEL_THRESHOLD = 0.01     # 10 mm/s
HOLD_S = 0.10            # must stay quiet this long, continuously
MIN_TIME_S = 0.03        # GRAVITY_SETTLE floor, even if quiet immediately
TIMEOUT_S = 0.5          # backstop
DT = 0.01


def _settle_config(**overrides) -> AlignmentConfig:
    cfg = AlignmentConfig(
        pose_stale_s=0.5, hold_still_motion_m=0.003, hold_still_duration_s=0.05,
        max_alignment_error_m=0.15, ramp_duration_s=1.0,
        gravity_ramp_duration_s=0.02,  # fast, not what's under test here
        gravity_settle_velocity_threshold_mps=VEL_THRESHOLD,
        gravity_settle_hold_s=HOLD_S,
        gravity_settle_min_time_s=MIN_TIME_S,
        gravity_settle_timeout_s=TIMEOUT_S,
        coupling_ramp_down_duration_s=1.0, self_follow_settle_duration_s=0.05,
        gravity_ramp_down_duration_s=0.05,
    )
    for k, v in overrides.items():
        setattr(cfg, k, v)
    return cfg


def _drive_to_gravity_settle(clock: FakeClock, cfg=None) -> TeleopStartupAlignment:
    """Common setup: settle to READY, arm, tick through GRAVITY_RAMP_UP
    (fed quiet velocity throughout -- that phase is not under test here)
    until GRAVITY_SETTLE is reached. Poses held fixed at the origin the
    whole time."""
    align = TeleopStartupAlignment(config=cfg or _settle_config(), now_fn=clock)
    pos = [0.0, 0.0, 0.0]
    for _ in range(200):
        align.update_master_pose(pos, t=clock.t)
        align.update_slave_pose(pos, t=clock.t)
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(IDENTITY_MAP)
        if align.state == READY:
            break
        clock.advance(DT)
    assert align.state == READY

    align.arm()
    while align.state == READY or align.state == GRAVITY_RAMP_UP:
        clock.advance(DT)
        align.update_master_pose(pos, t=clock.t)
        align.update_slave_pose(pos, t=clock.t)
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(IDENTITY_MAP)
    assert align.state == GRAVITY_SETTLE, f"setup should reach GRAVITY_SETTLE, got {align.state}"
    return align


def _feed(align: TeleopStartupAlignment, clock: FakeClock, quiet: bool,
         pos=(0.0, 0.0, 0.0)) -> None:
    """One tick of GRAVITY_SETTLE with either quiet (well under threshold)
    or noisy (well over threshold) velocity fed to both arms."""
    v = [0.0, 0.0, 0.0] if quiet else [0.0, VEL_THRESHOLD * 5.0, 0.0]
    clock.advance(DT)
    align.update_master_pose(list(pos), t=clock.t)
    align.update_slave_pose(list(pos), t=clock.t)
    align.update_master_velocity(v)
    align.update_slave_velocity(v)
    align.tick(IDENTITY_MAP)


# ---------------------------------------------------------------------------
# 1. One instantaneous zero-velocity sample does NOT pass settle.
# ---------------------------------------------------------------------------
def test_single_instantaneous_quiet_sample_does_not_pass():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    _feed(align, clock, quiet=True)
    assert align.state == GRAVITY_SETTLE, (
        "a single quiet tick must not be enough to pass the gate -- "
        f"hold_s={HOLD_S}s requires sustained quiet, got state={align.state}"
    )
    print("[1] single instantaneous quiet sample correctly does NOT pass. PASS")


# ---------------------------------------------------------------------------
# 2. Velocity below threshold but for LESS than settle_hold_s does NOT pass.
# ---------------------------------------------------------------------------
def test_quiet_shorter_than_hold_time_does_not_pass():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    n_ticks = int(HOLD_S / DT) - 2  # deliberately short of the hold window
    assert n_ticks > 0
    for _ in range(n_ticks):
        _feed(align, clock, quiet=True)
        assert align.state == GRAVITY_SETTLE, (
            f"gate passed too early, after only {n_ticks * DT:.3f}s < hold_s={HOLD_S}s"
        )
    print(f"[2] quiet for {n_ticks * DT:.3f}s (< hold_s={HOLD_S}s) correctly does NOT pass. PASS")


# ---------------------------------------------------------------------------
# 3. Both arms quiet continuously for settle_hold_s (+ min_time_s) DOES pass.
# ---------------------------------------------------------------------------
def test_continuous_quiet_for_hold_time_passes():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    # Enough ticks to comfortably clear both hold_s and min_time_s.
    n_ticks = int((max(HOLD_S, MIN_TIME_S) + 3 * DT) / DT) + 2
    reached = False
    for _ in range(n_ticks):
        _feed(align, clock, quiet=True)
        if align.state == COUPLING_RAMP_UP:
            reached = True
            break
    assert reached, f"gate never passed after {n_ticks * DT:.3f}s of continuous quiet"
    print(f"[3] continuous quiet passes the gate -> COUPLING_RAMP_UP. PASS")


# ---------------------------------------------------------------------------
# 4. Either arm becoming noisy resets the quiet timer.
# ---------------------------------------------------------------------------
def test_either_arm_noisy_resets_quiet_timer():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    # Quiet for most of the hold window...
    almost_full = int(HOLD_S / DT) - 1
    for _ in range(almost_full):
        _feed(align, clock, quiet=True)
    assert align.state == GRAVITY_SETTLE, "setup: must not have passed yet"

    # ...then ONE noisy tick right before it would have passed.
    _feed(align, clock, quiet=False)
    assert align.state == GRAVITY_SETTLE, "noisy tick must not itself pass the gate"

    # Immediately after, even a single quiet tick must NOT be enough --
    # the timer must have been reset to zero, not merely paused.
    _feed(align, clock, quiet=True)
    assert align.state == GRAVITY_SETTLE, (
        "quiet timer was not reset by the noisy tick -- gate passed on only "
        "1 quiet tick after the reset, which would mean the noise was ignored"
    )

    # But a FULL fresh hold window from here on DOES pass.
    reached = False
    for _ in range(int(HOLD_S / DT) + 2):
        _feed(align, clock, quiet=True)
        if align.state == COUPLING_RAMP_UP:
            reached = True
            break
    assert reached, "gate should pass after a full fresh quiet window post-reset"
    print("[4] noisy tick resets the quiet timer; a full fresh quiet window still passes. PASS")


# ---------------------------------------------------------------------------
# 5. minimum_settle_time is respected (even if quiet from tick 1).
# ---------------------------------------------------------------------------
def test_minimum_settle_time_respected():
    # min_time_s > hold_s here so the min-time floor is the actual binding
    # constraint, isolating it from the hold-duration requirement (test 3
    # already covers the hold_s requirement on its own).
    cfg = _settle_config(gravity_settle_min_time_s=0.20, gravity_settle_hold_s=0.02)
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock, cfg=cfg)
    settle_entry_t = align._phase_start_t
    passed_t = None
    for _ in range(200):
        _feed(align, clock, quiet=True)
        if align.state == COUPLING_RAMP_UP:
            passed_t = clock.t
            break
    assert passed_t is not None, "gate never passed"
    elapsed = passed_t - settle_entry_t
    assert elapsed >= cfg.gravity_settle_min_time_s - 1e-9, (
        f"gate passed after only {elapsed:.4f}s, before the "
        f"{cfg.gravity_settle_min_time_s:.4f}s minimum settle-time floor"
    )
    print(f"[5] minimum_settle_time ({cfg.gravity_settle_min_time_s}s) respected "
          f"(passed at {elapsed:.4f}s). PASS")


# ---------------------------------------------------------------------------
# 6. max timeout triggers GRAVITY_SETTLE_TIMEOUT.
# ---------------------------------------------------------------------------
def test_max_timeout_triggers_gravity_settle_timeout_fault():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    for _ in range(int(TIMEOUT_S / DT) + 5):
        _feed(align, clock, quiet=False)  # velocity NEVER settles
        if align.state == FAULT:
            break
    assert align.state == FAULT, f"expected FAULT after timeout, got {align.state}"
    assert "GRAVITY_SETTLE_TIMEOUT" in (align._fault_reason or ""), (
        f"wrong fault reason: {align._fault_reason!r}"
    )
    # FAULT is immediate force-safe -- both ramp scalars must snap to 0.
    assert align.gravity_ramp_alpha == 0.0
    assert align.coupling_ramp_alpha == 0.0
    print(f"[6] sustained noise -> GRAVITY_SETTLE_TIMEOUT fault, force-safe. PASS")


# ---------------------------------------------------------------------------
# 7. Fresh coupling origins / bilateral pull are captured/enabled only AFTER
#    the gate passes -- at the pure-module level: coupling_ramp_alpha stays
#    exactly 0.0 throughout GRAVITY_SETTLE regardless of noise, and state
#    only ever equals COUPLING_RAMP_UP starting on the exact tick the gate
#    passes (never earlier). The bridge-side origin SNAPSHOT/live-pose
#    capture itself (coupling_force_budget.AdaptiveCouplingRamp.
#    start_bilateral()) is exercised end-to-end by
#    test_force_budget_117mm_live.py's live COUPLING_RAMP_UP log line,
#    which fires only once bridge_state first reads COUPLING_RAMP_UP -- i.e.
#    structurally after this same gate.
# ---------------------------------------------------------------------------
def test_coupling_alpha_and_state_only_advance_after_gate_passes():
    # Generous timeout here -- this test is about the alpha/state invariant,
    # not the timeout behavior (test 6 covers that), so give the
    # alternating-then-quiet pattern below comfortable room to finish before
    # gravity_settle_timeout_s would otherwise trip it.
    cfg = _settle_config(gravity_settle_timeout_s=5.0)
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock, cfg=cfg)
    saw_coupling_ramp_up = False
    for i in range(40):
        # Alternate noisy/quiet to exercise resets too, without ever
        # settling into a full hold window until near the very end.
        quiet = (i % 5 == 4) if i < 20 else True
        _feed(align, clock, quiet=quiet)
        assert align.coupling_ramp_alpha == 0.0 or align.state != GRAVITY_SETTLE, (
            "coupling_ramp_alpha must be exactly 0 for the entire time state==GRAVITY_SETTLE"
        )
        if align.state == COUPLING_RAMP_UP:
            saw_coupling_ramp_up = True
            break
    assert saw_coupling_ramp_up, "gate never passed within the test budget"
    print("[7] coupling_ramp_alpha stayed exactly 0 throughout GRAVITY_SETTLE; "
          "COUPLING_RAMP_UP entered only once the gate passed. PASS")


# ---------------------------------------------------------------------------
# 8. GRAVITY_SETTLE hold mode (2026-09-10, real-hardware Y-axis validation
#    Part 1): once gravity_settle_hold_mode is enabled and the quiet gate
#    passes, the state machine must remain in GRAVITY_SETTLE indefinitely --
#    gravity_ramp_alpha=1, coupling_ramp_alpha=0, no auto-transition to
#    COUPLING_RAMP_UP, no re-triggered GRAVITY_SETTLE_TIMEOUT -- for an
#    ARBITRARILY long hold (tested well beyond gravity_settle_timeout_s,
#    which would otherwise FAULT the ordinary/non-hold path).
# ---------------------------------------------------------------------------
def test_hold_mode_stays_in_gravity_settle_indefinitely():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    align.gravity_settle_hold_mode = True

    # Reach the gate first (same as test 3).
    n_ticks = int((max(HOLD_S, MIN_TIME_S) + 3 * DT) / DT) + 2
    reached_hold = False
    for _ in range(n_ticks):
        _feed(align, clock, quiet=True)
        if align.state == GRAVITY_SETTLE and align._gravity_settle_gate_passed:
            reached_hold = True
            break
    assert reached_hold, "gate should have passed (hold mode does not change the gate itself)"
    assert align.state == GRAVITY_SETTLE, "must NOT have auto-transitioned to COUPLING_RAMP_UP"
    assert align.coupling_ramp_alpha == 0.0
    assert align.gravity_ramp_alpha == 1.0

    # Continue for far longer than gravity_settle_timeout_s (0.5s in this
    # test's config) -- the ordinary (non-hold) path would FAULT with
    # GRAVITY_SETTLE_TIMEOUT well before this; hold mode must not.
    n_long = int((TIMEOUT_S * 20) / DT)
    for _ in range(n_long):
        _feed(align, clock, quiet=True)
        assert align.state == GRAVITY_SETTLE, (
            f"hold mode must stay in GRAVITY_SETTLE indefinitely, got {align.state} "
            f"(possible spurious timeout or transition)"
        )
        assert align.coupling_ramp_alpha == 0.0, "coupling_ramp_alpha must stay exactly 0 throughout the hold"
        assert align.gravity_ramp_alpha == 1.0, "gravity_ramp_alpha must stay exactly 1.0 throughout the hold"
    print(f"[8] hold mode held GRAVITY_SETTLE for {n_long * DT:.2f}s "
          f"(>{TIMEOUT_S * 20:.2f}s, {TIMEOUT_S/DT:.0f}x the ordinary timeout) -- "
          f"coupling_ramp_alpha and gravity_ramp_alpha exact throughout. PASS")


def test_hold_mode_blend_target_shows_zero_bilateral_pull_during_hold():
    """coupling_ramp_alpha==0 makes blend_target()/blend_left_target()
    return the live pose EXACTLY (see those methods' own docstrings) -- this
    confirms that structural guarantee holds specifically during an
    extended hold-mode window, not just momentarily at alpha=0."""
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    align.gravity_settle_hold_mode = True
    for _ in range(int((max(HOLD_S, MIN_TIME_S) + 3 * DT) / DT) + 2):
        _feed(align, clock, quiet=True)
        if align.state == GRAVITY_SETTLE and align._gravity_settle_gate_passed:
            break
    assert align._gravity_settle_gate_passed

    full_target = [0.5, -0.3, 0.2]  # arbitrary, far from live pose -- if any
                                    # bilateral pull leaked through, this
                                    # would show up as a nonzero blend.
    for _ in range(50):
        _feed(align, clock, quiet=True)
        live = [0.0, 0.0, 0.0]  # matches _feed's fixed pos=(0,0,0)
        published = align.blend_target(full_target)
        assert published == live, (
            f"blend_target() must equal the live pose exactly (zero bilateral "
            f"spring pull) throughout the hold, got {published} vs live {live}"
        )
        published_left = align.blend_left_target(full_target)
        assert published_left == live, (
            f"blend_left_target() must equal the live pose exactly during the hold, "
            f"got {published_left} vs live {live}"
        )
    print("[8b] blend_target()/blend_left_target() == live pose exactly, "
          "every tick, for the whole hold window -- zero bilateral spring pull confirmed. PASS")


def test_hold_mode_exits_only_via_explicit_disarm():
    clock = FakeClock()
    align = _drive_to_gravity_settle(clock)
    align.gravity_settle_hold_mode = True
    for _ in range(int((max(HOLD_S, MIN_TIME_S) + 3 * DT) / DT) + 2):
        _feed(align, clock, quiet=True)
        if align.state == GRAVITY_SETTLE and align._gravity_settle_gate_passed:
            break
    assert align._gravity_settle_gate_passed

    # Hold for a while first, confirming no spontaneous exit.
    for _ in range(200):
        _feed(align, clock, quiet=True)
        assert align.state == GRAVITY_SETTLE
    assert align.state == GRAVITY_SETTLE

    align.disarm()
    _feed(align, clock, quiet=True)
    assert align.state == "GRAVITY_RAMP_DOWN", f"disarm must exit the hold into GRAVITY_RAMP_DOWN, got {align.state}"
    print("[8c] hold mode exits ONLY via explicit disarm(), into GRAVITY_RAMP_DOWN. PASS")


if __name__ == "__main__":
    test_single_instantaneous_quiet_sample_does_not_pass()
    test_quiet_shorter_than_hold_time_does_not_pass()
    test_continuous_quiet_for_hold_time_passes()
    test_either_arm_noisy_resets_quiet_timer()
    test_minimum_settle_time_respected()
    test_max_timeout_triggers_gravity_settle_timeout_fault()
    test_coupling_alpha_and_state_only_advance_after_gate_passes()
    test_hold_mode_stays_in_gravity_settle_indefinitely()
    test_hold_mode_blend_target_shows_zero_bilateral_pull_during_hold()
    test_hold_mode_exits_only_via_explicit_disarm()
    print("ALL TESTS PASSED")
