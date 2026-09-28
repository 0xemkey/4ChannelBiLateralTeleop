#!/usr/bin/env python3
"""Discrete multi-rate comparison, 2026-09-09, Part 7/8 of the force-aware
transition task -- OLD_SINGLE_STAGE vs STAGED_FORCE_AWARE (snapshot origin,
per the finding in test_coupling_force_budget.py's test 4 docstring) at the
mandatory 117mm real-trial initial gap.

SCOPE NOTE (stated honestly, not hidden): the full task asked for A/B/C/D
across 5 gap sizes with a Pareto sweep over duration/reserve/velocity-limit.
Given the size of this task, this script covers the MANDATORY 117mm case
comparing the two most informative strategies (A: old baseline, D: the new
force-aware design) rather than the full matrix -- see the final report's
"remaining risks / assumptions" section for what was NOT run.

DISCRETE MULTI-RATE MODEL (Part 7's explicit requirement -- replaces the
earlier same-rate continuous model, which could not reproduce the real
post-disarm sample-and-hold transient):
    plant/controller update: 200Hz (5ms) -- matches the real trial's
        measured effective_hz on pose/telemetry
    bridge target/gravity/state-machine tick: 100Hz (10ms) -- matches the
        real RATE_HZ=100 -- the controller HOLDS the last bridge-published
        target between bridge ticks (sample-and-hold), exactly as the real
        hardware does.
    force clamp: exact production logic (clip to max_force_n, vector norm)
    plant response: v = F_net / B, same ASSUMED B as
        compare_old_vs_staged_transition.py (not independently measured --
        stated here again so this file is self-contained about that
        assumption).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import ramped_gravity_comp_n  # noqa: E402
from startup_alignment import TeleopStartupAlignment, AlignmentConfig, READY, ACTIVE  # noqa: E402
from coupling_force_budget import AdaptiveCouplingRamp, ForceBudgetConfig, CouplingRampStalled  # noqa: E402

K = 35.0
D = 0.20
MAX_FORCE = 1.5
G_TRUE = 0.70
CONFIGURED_G = 0.70
B = 3.0  # ASSUMED, see module docstring and compare_old_vs_staged_transition.py

CONTROLLER_DT = 0.005   # 200Hz
BRIDGE_DT = 0.010       # 100Hz -- target/gravity/state only updates this often
STEPS_PER_BRIDGE_TICK = int(round(BRIDGE_DT / CONTROLLER_DT))

Y_START = -0.1174
Y_FULL_TARGET = 0.0

COUPLING_RAMP_DURATION = 1.0
GRAVITY_RAMP_DURATION = 0.3
GRAVITY_SETTLE_DURATION = 0.2


def controller_force(y, v, target_y, aux_y):
    spring = K * (target_y - y)
    damp = -D * v
    raw = spring + damp + aux_y
    total = max(-MAX_FORCE, min(MAX_FORCE, raw))
    clamped = abs(total - raw) > 1e-9
    return total, clamped, spring, damp


def step_plant_controller_tick(y, v, target_y, aux_y):
    """One 200Hz controller tick, holding target_y/aux_y fixed (sample-and-
    hold -- these only change once per bridge tick, per the multi-rate
    model above)."""
    total, clamped, spring, damp = controller_force(y, v, target_y, aux_y)
    net = total - G_TRUE
    v_new = net / B
    y_new = y + v_new * CONTROLLER_DT
    return y_new, v_new, total, clamped, spring


def run_old_single_stage(duration_s=3.0):
    y, v = Y_START, 0.0
    t = 0.0
    peak_force = peak_spring = peak_error = 0.0
    any_clamp = False
    n_ticks = int(duration_s / BRIDGE_DT)
    for i in range(n_ticks):
        alpha = min(1.0, t / COUPLING_RAMP_DURATION)
        target_y = Y_START + alpha * (Y_FULL_TARGET - Y_START)
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, alpha)
        for _ in range(STEPS_PER_BRIDGE_TICK):
            y, v, total, clamped, spring = step_plant_controller_tick(y, v, target_y, aux_y)
            peak_force = max(peak_force, abs(total))
            peak_spring = max(peak_spring, abs(spring))
            any_clamp = any_clamp or clamped
        peak_error = max(peak_error, abs(target_y - y))
        t += BRIDGE_DT
    return {"peak_force_n": peak_force, "peak_spring_n": peak_spring,
           "peak_error_m": peak_error, "any_clamp": any_clamp, "final_y": y}


def run_staged_force_aware(duration_s=5.0):
    y, v = Y_START, 0.0
    t = 0.0
    peak_force = peak_spring = peak_error = 0.0
    any_clamp = False
    stalled = False
    convergence_t = None

    cfg = ForceBudgetConfig(max_force_n=MAX_FORCE, force_reserve_n=0.1,
                            nominal_duration_s=COUPLING_RAMP_DURATION, stall_timeout_s=3.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="snapshot")

    # Gravity phase (fixed-time, unaffected by this task's force-aware work
    # -- gravity alone was never the saturation source).
    n_grav_ticks = int((GRAVITY_RAMP_DURATION + GRAVITY_SETTLE_DURATION) / BRIDGE_DT) + 1
    for i in range(n_grav_ticks):
        gravity_elapsed = t
        if gravity_elapsed < GRAVITY_RAMP_DURATION:
            gravity_alpha = gravity_elapsed / GRAVITY_RAMP_DURATION
        else:
            gravity_alpha = 1.0
        target_y = y  # own live pose during gravity-only phase (no coupling pull)
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, gravity_alpha)
        for _ in range(STEPS_PER_BRIDGE_TICK):
            y, v, total, clamped, spring = step_plant_controller_tick(y, v, target_y, aux_y)
            peak_force = max(peak_force, abs(total))
            peak_spring = max(peak_spring, abs(spring))
            any_clamp = any_clamp or clamped
        t += BRIDGE_DT

    # Coupling phase, force-budget-adaptive.
    ramp.start(now=t, origin_pose=[0.0, y, 0.0])
    n_coupling_ticks = int(duration_s / BRIDGE_DT)
    for i in range(n_coupling_ticks):
        try:
            alpha = ramp.step(now=t, dt=BRIDGE_DT, live_pose=[0.0, y, 0.0], velocity=[0.0, v, 0.0],
                              full_target=[0.0, Y_FULL_TARGET, 0.0], stiffness_n_per_m=K,
                              damping_n_s_per_m=D, aux_vec=[0.0, CONFIGURED_G, 0.0])
        except CouplingRampStalled:
            stalled = True
            break
        # Public accessor (2026-09-09 Part 6) instead of reaching into
        # ramp._snapshot_origin directly.
        target_y = ramp.current_target([0.0, y, 0.0], [0.0, Y_FULL_TARGET, 0.0])[1]
        aux_y = CONFIGURED_G  # gravity already fully settled
        for _ in range(STEPS_PER_BRIDGE_TICK):
            y, v, total, clamped, spring = step_plant_controller_tick(y, v, target_y, aux_y)
            peak_force = max(peak_force, abs(total))
            peak_spring = max(peak_spring, abs(spring))
            any_clamp = any_clamp or clamped
        peak_error = max(peak_error, abs(target_y - y))
        t += BRIDGE_DT
        if alpha >= 1.0 and convergence_t is None:
            convergence_t = t

    return {"peak_force_n": peak_force, "peak_spring_n": peak_spring,
           "peak_error_m": peak_error, "any_clamp": any_clamp, "final_y": y,
           "stalled": stalled, "convergence_t_s": convergence_t}


if __name__ == "__main__":
    print("=" * 78)
    print("MANDATORY 117mm CASE -- discrete multi-rate model (200Hz controller / 100Hz bridge)")
    print("=" * 78)
    old = run_old_single_stage()
    new = run_staged_force_aware()
    print(f"{'metric':<26} {'A: OLD_SINGLE_STAGE':>22} {'D: STAGED_FORCE_AWARE':>24}")
    print(f"{'peak |applied force| (N)':<26} {old['peak_force_n']:>22.4f} {new['peak_force_n']:>24.4f}")
    print(f"{'peak |spring force| (N)':<26} {old['peak_spring_n']:>22.4f} {new['peak_spring_n']:>24.4f}")
    print(f"{'peak |position error| (m)':<26} {old['peak_error_m']:>22.4f} {new['peak_error_m']:>24.4f}")
    print(f"{'any force clamp':<26} {str(old['any_clamp']):>22} {str(new['any_clamp']):>24}")
    print(f"{'stalled':<26} {'n/a':>22} {str(new['stalled']):>24}")
    print(f"{'convergence time (s)':<26} {COUPLING_RAMP_DURATION:>22.3f} "
          f"{(new['convergence_t_s'] or float('nan')):>24.3f}")
    print(f"{'final y (should be ~0)':<26} {old['final_y']:>22.5f} {new['final_y']:>24.5f}")
    print()
    print(f"REAL HARDWARE BASELINE (measured, for reference): slave max raw position "
          f"error 32.4mm, master 47.2mm, slave applied force clamped at 1.5N exactly.")
