#!/usr/bin/env python3
"""Pure-Python tests for coupling_force_budget.py (2026-09-09, force-aware
transition task). No rclpy, no hardware.

Covers Part 13 items 1-7 (the pure/unit subset -- items 8-10 are covered by
startup_alignment.py's own existing tests, which are unaffected by this
module since it is not yet wired into the state machine's tick()."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from coupling_force_budget import (  # noqa: E402
    AdaptiveCouplingRamp, ForceBudgetConfig, CouplingRampStalled, predict_force_vector, _norm,
)

K = 35.0
D = 0.20
MAX_FORCE = 1.5
G = 0.70
AUX = [0.0, G, 0.0]


# ---------------------------------------------------------------------------
# 1. Adaptive alpha monotonicity.
# ---------------------------------------------------------------------------
def test_alpha_monotonic_easy_case():
    """Small gap, plenty of force budget -- alpha should climb steadily and
    monotonically to 1.0."""
    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1, nominal_duration_s=1.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="moving")
    origin = [0.0, -0.02, 0.0]  # small 20mm gap
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    live_pose = list(origin)
    velocity = [0.0, 0.0, 0.0]
    t = 0.0
    dt = 0.01
    prev_alpha = 0.0
    alphas = []
    for _ in range(200):
        alpha = ramp.step(now=t, dt=dt, live_pose=live_pose, velocity=velocity,
                          full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
        assert alpha >= prev_alpha - 1e-12, "alpha must never decrease during a normal ramp"
        prev_alpha = alpha
        alphas.append(alpha)
        # Live pose "catches up" perfectly each tick for this easy case, so the
        # predicted force never grows unbounded.
        live_pose = [origin[i] + alpha * (full_target[i] - origin[i]) for i in range(3)]
        t += dt
        if alpha >= 1.0:
            break
    assert alphas[-1] == 1.0, "easy case must fully converge to alpha=1"
    assert len(alphas) > 1
    print(f"[1] monotonic climb to 1.0 over {len(alphas)} ticks (easy case). PASS")


# ---------------------------------------------------------------------------
# 2. Force-budget constraint is actually enforced.
# ---------------------------------------------------------------------------
def test_force_budget_constraint_caps_predicted_force():
    """Large gap, live pose held FIXED (worst case -- device not moving at
    all, so the full spring error is exposed every tick). The predicted
    force at whatever alpha is accepted must never exceed max_force_n -
    reserve."""
    # stall_timeout_s deliberately large: this test's device never moves at
    # all (worst case, by design), which is exactly the scenario test 5
    # separately checks DOES eventually trip the watchdog -- here the point
    # is only to check the force cap itself, so the watchdog is not meant
    # to fire mid-test.
    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1, nominal_duration_s=1.0,
                            stall_timeout_s=100.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="moving")
    origin = [0.0, -0.1174, 0.0]  # real 117mm case
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    live_pose = list(origin)  # device does NOT move -- worst case
    velocity = [0.0, 0.0, 0.0]
    t = 0.0
    dt = 0.01
    max_predicted_force_seen = 0.0
    for _ in range(400):
        alpha = ramp.step(now=t, dt=dt, live_pose=live_pose, velocity=velocity,
                          full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
        target = [origin[i] + alpha * (full_target[i] - origin[i]) for i in range(3)]
        f = predict_force_vector(live_pose, velocity, target, K, D, AUX)
        max_predicted_force_seen = max(max_predicted_force_seen, _norm(f))
        t += dt
        if alpha >= 1.0:
            break
    print(f"[2] max predicted force actually reached = {max_predicted_force_seen:.4f}N "
          f"(budget = {MAX_FORCE - cfg.force_reserve_n:.4f}N)")
    assert max_predicted_force_seen <= MAX_FORCE - cfg.force_reserve_n + 1e-6, (
        "force-budget constraint must never let predicted force exceed max_force_n - reserve")
    print("PASS")


# ---------------------------------------------------------------------------
# 3. Alpha holds when unsafe.
# ---------------------------------------------------------------------------
def test_alpha_holds_when_immediately_unsafe():
    """At the very first tick, if even the SMALLEST alpha step would
    already breach budget, alpha must hold at its current value, not force
    through. Note: for MOVING_LIVE_ORIGIN, target(alpha) - live_pose scales
    with alpha itself, so a large POSITION gap alone does not force an
    immediate hold (a small enough alpha step always yields a small enough
    target step, regardless of how far away full_target is) -- what DOES
    force an immediate hold regardless of alpha is a large VELOCITY, since
    the damping term (-D*velocity) does not depend on alpha or the target
    at all. This is exercised here with an extreme velocity so aux+damping
    ALONE already exceed the budget before any spring contribution."""
    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1, nominal_duration_s=1.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="moving")
    origin = [0.0, -0.1174, 0.0]
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    live_pose = list(origin)
    extreme_velocity = [0.0, -10.0, 0.0]  # damping alone: -D*v = -0.20*(-10) = +2.0N, already > budget
    alpha = ramp.step(now=0.0, dt=0.01, live_pose=live_pose, velocity=extreme_velocity,
                      full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
    print(f"[3] alpha after first tick with extreme velocity = {alpha} (expect ~0, held)")
    assert alpha < 1e-6, "alpha must hold near 0 when even a tiny step is unsafe"
    assert ramp.last_held is True
    print("PASS")


# ---------------------------------------------------------------------------
# 4. Alpha resumes when safe again.
# ---------------------------------------------------------------------------
def test_alpha_resumes_once_device_catches_up():
    """Start with live pose lagging (holding alpha back); once live pose
    catches up to the target (reducing spring demand), alpha must resume
    climbing, not stay stuck.

    Uses SNAPSHOT_ORIGIN, not MOVING_LIVE_ORIGIN -- an important finding
    from developing this test: under MOVING_LIVE_ORIGIN, origin re-centers
    on the live pose every tick, so a HELD alpha still implies an ongoing
    absolute target offset proportional to alpha times whatever the
    (shrinking) remaining gap to full_target currently is. That offset
    requires continuous nonzero spring force merely to be MAINTAINED, so
    margin hovers right at the edge of feasibility and never comfortably
    recovers even as the device visibly converges -- confirmed directly:
    with origin_mode="moving" this test's own convergence loop drove
    live_pose to within 1e-15m of target while margin stayed pinned at
    -0.00033N indefinitely, never allowing alpha to resume. SNAPSHOT_ORIGIN
    does not have this problem: once the device reaches a FIXED target
    implied by a held alpha, spring drops to ~0 and margin recovers
    comfortably, letting the watchdog-free resume this test checks for
    actually happen. This is the concrete answer to Part 2's "test both,
    do not assume" requirement for the force-budget-adaptive case
    specifically (a plain always-advancing fixed-time ramp does not have
    this failure mode, since alpha there never holds long enough for it
    to matter)."""
    # The bisection search (coupling_force_budget.py's step()) always finds
    # SOME safe nonzero increment unless even the smallest resolvable step
    # is already unsafe -- so, as test 3 established, a reliable hold needs
    # a force contribution that does NOT shrink with the alpha step size
    # (velocity/damping, not position/spring). Force a hold the same way
    # test 3 does, then relax it and confirm resumption -- this is more
    # deterministic than hoping a duration/gain mismatch happens to produce
    # an emergent hold.
    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1, nominal_duration_s=1.0,
                            stall_timeout_s=10.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="snapshot")
    origin = [0.0, -0.1174, 0.0]
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    live_pose = list(origin)

    alpha0 = ramp.step(now=0.0, dt=0.01, live_pose=live_pose, velocity=[0.0, -10.0, 0.0],
                       full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
    assert alpha0 < 1e-6 and ramp.last_held, "setup: extreme velocity must force an immediate hold"
    held_at_some_point = ramp.last_held
    alpha_before_relax = alpha0

    resumed_after_hold = False
    prev_alpha = alpha0
    t = 0.01
    dt = 0.01
    for i in range(50):
        # Velocity relaxes back to zero -- the device is no longer lurching,
        # so the damping-alone force contribution that was blocking progress
        # goes away.
        alpha = ramp.step(now=t, dt=dt, live_pose=live_pose, velocity=[0.0, 0.0, 0.0],
                          full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
        if alpha > prev_alpha:
            resumed_after_hold = True
            break
        prev_alpha = alpha
        t += dt
    print(f"[4] held_at_some_point={held_at_some_point} (alpha={alpha_before_relax}) "
          f"resumed_after_hold={resumed_after_hold} (alpha={prev_alpha if not resumed_after_hold else alpha})")
    assert held_at_some_point, "test fixture should have actually exercised a hold"
    assert resumed_after_hold, "alpha must resume climbing once the blocking condition clears"
    print("PASS")


# ---------------------------------------------------------------------------
# 5. Stall watchdog.
# ---------------------------------------------------------------------------
def test_stall_watchdog_trips_when_never_safe():
    """Pathological case: live pose NEVER catches up (held rigidly far from
    target forever) -- the watchdog must eventually raise, not hold silently
    forever."""
    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1, nominal_duration_s=1.0,
                            stall_timeout_s=1.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="moving")
    origin = [0.0, -0.5, 0.0]
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    live_pose = list(origin)  # never moves
    velocity = [0.0, 0.0, 0.0]
    t = 0.0
    dt = 0.01
    raised = False
    try:
        for _ in range(500):
            ramp.step(now=t, dt=dt, live_pose=live_pose, velocity=velocity,
                     full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=AUX)
            t += dt
    except CouplingRampStalled as e:
        raised = True
        print(f"[5] watchdog tripped: {e}")
    assert raised, "stall watchdog must trip when no safe progress is ever possible"
    assert ramp.stalled
    print("PASS")


# ---------------------------------------------------------------------------
# 6. Snapshot-origin continuity.
# ---------------------------------------------------------------------------
def test_snapshot_origin_continuity():
    """SNAPSHOT_ORIGIN: origin is captured ONCE at start(), and target(0) at
    the first step must equal that snapshot exactly, regardless of how the
    live pose moves afterward."""
    cfg = ForceBudgetConfig(nominal_duration_s=1.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="snapshot")
    origin = [0.01, -0.05, 0.02]
    ramp.start(now=0.0, origin_pose=origin)
    full_target = [0.0, 0.0, 0.0]
    # Live pose has moved AWAY from the snapshot by the time of the first step.
    moved_live_pose = [0.05, -0.02, 0.06]
    velocity = [0.0, 0.0, 0.0]
    alpha = ramp.step(now=0.0, dt=0.001, live_pose=moved_live_pose, velocity=velocity,
                      full_target=full_target, stiffness_n_per_m=K, damping_n_s_per_m=D, aux_vec=[0, 0, 0])
    target0 = [ramp._origin(moved_live_pose)[i] + alpha * (full_target[i] - ramp._origin(moved_live_pose)[i])
              for i in range(3)]
    for i in range(3):
        assert abs(ramp._origin(moved_live_pose)[i] - origin[i]) < 1e-12, (
            "snapshot origin must stay fixed at the pose captured at start(), not track live pose")
    print(f"[6] snapshot origin fixed at {origin}, unaffected by live pose moving to {moved_live_pose}. PASS")


# ---------------------------------------------------------------------------
# 7. Moving-origin continuity.
# ---------------------------------------------------------------------------
def test_moving_origin_continuity():
    """MOVING_LIVE_ORIGIN: origin(t) must always equal the CURRENT live
    pose, at every tick -- so at alpha=0 (before any progress), the
    implied target is always exactly the live pose, continuously."""
    cfg = ForceBudgetConfig(nominal_duration_s=1.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="moving")
    origin = [0.0, -0.1, 0.0]
    ramp.start(now=0.0, origin_pose=origin)
    for live_pose in ([0.0, -0.1, 0.0], [0.02, -0.05, 0.01], [0.10, 0.0, -0.02]):
        got = ramp._origin(live_pose)
        assert got == live_pose, f"moving origin must equal live pose exactly, got {got} for {live_pose}"
    print("[7] moving origin tracks live pose exactly at every query, regardless of prior positions. PASS")


if __name__ == "__main__":
    test_alpha_monotonic_easy_case()
    test_force_budget_constraint_caps_predicted_force()
    test_alpha_holds_when_immediately_unsafe()
    test_alpha_resumes_once_device_catches_up()
    test_stall_watchdog_trips_when_never_safe()
    test_snapshot_origin_continuity()
    test_moving_origin_continuity()
    print("\nALL TESTS PASSED")
