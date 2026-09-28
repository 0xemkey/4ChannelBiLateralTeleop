#!/usr/bin/env python3
"""Old (single-stage RAMPING) vs new (staged GRAVITY_RAMP_UP/.../ACTIVE)
transition architecture comparison, 2026-09-09, Part 6/8 of the staged-
transition redesign task.

Drives the SAME 1-DOF (Y-axis, the axis that dominated the real trial's
transient) physical plant through both architectures' real target/gravity
generation code, from the same deliberately-difficult initial condition
(slave ~117mm from the master's held reference, matching the real 0.70N
Trial 1's measured sweep distance).

PLANT MODEL -- stated explicitly, not hidden: an overdamped (first-order,
mass-less) point model, v = F_net / B, chosen in the same style as the
sample-hold model already validated against real hardware data in
test_ramp_disarm_root_cause.py (order-of-magnitude match, not exact). B is
an ASSUMED effective mechanical coefficient (not independently measured for
this device) -- picked so a ~1N net force produces a plausible ~0.2-0.3m/s
response, consistent with the device's own 0.5 m/s max_speed_mps ceiling.
This is an illustrative model for COMPARING the two architectures under
identical assumptions, not a validated absolute-force predictor -- the
comparison (old vs new, same B, same everything else) is the point, not
either column's absolute number in isolation.

Controller law modeled (matches touch_slave.cpp's real force_ros formula,
see its own worst-sample decomposition confirmed against real telemetry in
this session's root-cause investigation):
    F_spring = K * (target - y)
    F_damp   = -D * v
    F_total  = clip(F_spring + F_damp + aux, -max_force, max_force)
    net force on the plant = F_total - G_true   (G_true = the constant real
        weight the gravity-comp channel exists to cancel; uncompensated
        whenever aux < G_true)
    v = F_total_net / B   (see plant model note above)
    y += v * dt

OLD ARCHITECTURE model: reproduces the single-stage RAMPING this session's
root-cause investigation found on real hardware -- ONE alpha(t), ramping
0->1 over ramp_duration_s, used for BOTH gravity (aux = G*alpha) and
coupling (target = origin + alpha*(full_target-origin)) simultaneously.
Implemented directly here (not via startup_alignment.py, which no longer
has a single-stage RAMPING state at all -- that removal IS the fix under
test) as a small, explicit, auditable function.

NEW ARCHITECTURE: the real TeleopStartupAlignment class, imported
unmodified, driving the plant through its actual GRAVITY_RAMP_UP ->
GRAVITY_SETTLE -> COUPLING_RAMP_UP -> ACTIVE -> COUPLING_RAMP_DOWN ->
GRAVITY_RAMP_DOWN -> READY sequence.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import ramped_gravity_comp_n  # noqa: E402
from startup_alignment import TeleopStartupAlignment, AlignmentConfig, READY, ACTIVE  # noqa: E402

# --- Shared plant/controller parameters (slave's real config) ---
K = 35.0            # stiffness_n_per_m (run_slave_right.sh default)
D = 0.20            # damping_n_s_per_m
MAX_FORCE = 1.5     # max_force_n
G_TRUE = 0.70       # the real static weight the 0.70N gravity-comp setting is tuned to cancel
CONFIGURED_G = 0.70
B = 3.0             # ASSUMED effective mechanical coefficient (N*s/m) -- see module docstring
DT = 0.005          # 200Hz plant/controller update, matching the real trial's measured effective_hz

GRAVITY_RAMP_DURATION = 0.3
GRAVITY_SETTLE_DURATION = 0.2
COUPLING_RAMP_DURATION = 1.0

Y_START = -0.1174   # slave's gravity-sagged rest pose, y-only (matches the real ~117mm gap)
Y_FULL_TARGET = 0.0  # master's held reference position, y-only


def step_plant(y, v, target_y, aux_y):
    spring = K * (target_y - y)
    damp = -D * v
    raw = spring + damp + aux_y
    total = max(-MAX_FORCE, min(MAX_FORCE, raw))
    clamped = abs(total - raw) > 1e-9
    net = total - G_TRUE
    v_new = net / B
    y_new = y + v_new * DT
    # |spring| alone isolates the position-tracking-induced component,
    # separate from the constant aux baseline -- this is what the real
    # hardware root-cause report's own worst-sample decomposition used
    # (spring_force_n as the diagnostic quantity), and what distinguishes a
    # genuine transient from a steady operating force.
    return y_new, v_new, total, clamped, spring, damp


def simulate_old_architecture(duration_s=3.0):
    """Single alpha(t) drives gravity and coupling simultaneously -- the
    real, previously-shipped behavior this session found on real hardware."""
    y, v = Y_START, 0.0
    t = 0.0
    peak_force = 0.0
    peak_spring = 0.0
    peak_error = 0.0
    any_clamp = False
    rows = []
    while t < duration_s:
        alpha = min(1.0, t / COUPLING_RAMP_DURATION)
        target_y = Y_START + alpha * (Y_FULL_TARGET - Y_START)
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, alpha)
        y, v, total, clamped, spring, damp = step_plant(y, v, target_y, aux_y)
        peak_force = max(peak_force, abs(total))
        peak_spring = max(peak_spring, abs(spring))
        peak_error = max(peak_error, abs(target_y - y))
        any_clamp = any_clamp or clamped
        rows.append((t, y, target_y, total, aux_y, alpha))
        t += DT
    return {
        "peak_force_n": peak_force, "peak_spring_n": peak_spring,
        "peak_error_m": peak_error, "any_clamp": any_clamp,
        "final_y": y, "rows": rows,
    }


def simulate_new_architecture(duration_s=3.0):
    """Real TeleopStartupAlignment, driving the plant through the actual
    staged state sequence."""
    clock_t = [0.0]

    def now():
        return clock_t[0]

    cfg = AlignmentConfig(
        gravity_ramp_duration_s=GRAVITY_RAMP_DURATION,
        gravity_settle_duration_s=GRAVITY_SETTLE_DURATION,
        ramp_duration_s=COUPLING_RAMP_DURATION,
    )
    align = TeleopStartupAlignment(config=cfg, now_fn=now)

    y, v = Y_START, 0.0
    peak_force = 0.0
    peak_spring = 0.0
    peak_error = 0.0
    any_clamp = False
    rows = []

    # Settle to READY first (identical master/slave poses, held still).
    align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
    align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
    for _ in range(400):
        align.tick(lambda p: p)
        if align.state == READY:
            break
        clock_t[0] += 0.01
        align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
        align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
    assert align.state == READY
    align.arm()

    t = 0.0
    while t < duration_s:
        align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
        align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
        align.tick(lambda p: p)

        full_target_y = Y_FULL_TARGET  # C3 absolute-axis target == master's live y
        target_y = align.blend_target([0.0, full_target_y, 0.0])[1]
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, align.gravity_ramp_alpha)

        y, v, total, clamped, spring, damp = step_plant(y, v, target_y, aux_y)
        peak_force = max(peak_force, abs(total))
        peak_spring = max(peak_spring, abs(spring))
        peak_error = max(peak_error, abs(target_y - y))
        any_clamp = any_clamp or clamped
        rows.append((t, y, target_y, total, aux_y, align.state))

        clock_t[0] += DT
        t += DT

    return {
        "peak_force_n": peak_force, "peak_spring_n": peak_spring,
        "peak_error_m": peak_error, "any_clamp": any_clamp,
        "final_y": y, "rows": rows, "final_state": align.state,
    }


def simulate_new_architecture_disarm(duration_s=3.0):
    """Continues from ACTIVE (steady-state, y already at Y_FULL_TARGET,
    fully gravity-supported) through a full disarm sequence, measuring the
    post-disarm peak transient -- the NEW architecture's counterpart to the
    real ~0.195-0.203N measured post-disarm spike."""
    clock_t = [0.0]

    def now():
        return clock_t[0]

    cfg = AlignmentConfig(
        gravity_ramp_duration_s=GRAVITY_RAMP_DURATION,
        gravity_settle_duration_s=GRAVITY_SETTLE_DURATION,
        ramp_duration_s=COUPLING_RAMP_DURATION,
        coupling_ramp_down_duration_s=COUPLING_RAMP_DURATION,
        self_follow_settle_duration_s=0.2,
        gravity_ramp_down_duration_s=GRAVITY_RAMP_DURATION,
    )
    align = TeleopStartupAlignment(config=cfg, now_fn=now)
    y, v = Y_FULL_TARGET, 0.0  # start already converged, at steady state
    align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
    align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
    for _ in range(400):
        align.tick(lambda p: p)
        if align.state == READY:
            break
        clock_t[0] += 0.01
        align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
        align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
    align.arm()
    while align.state != ACTIVE:
        clock_t[0] += 0.01
        align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
        align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
        align.tick(lambda p: p)
        target_y = align.blend_target([0.0, Y_FULL_TARGET, 0.0])[1]
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, align.gravity_ramp_alpha)
        y, v, *_ = step_plant(y, v, target_y, aux_y)

    align.disarm()
    peak_force_post = 0.0
    t = 0.0
    while t < duration_s:
        align.update_master_pose([0.0, Y_FULL_TARGET, 0.0], t=clock_t[0])
        align.update_slave_pose([0.0, y, 0.0], t=clock_t[0])
        align.tick(lambda p: p)
        target_y = align.blend_target([0.0, Y_FULL_TARGET, 0.0])[1]
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, align.gravity_ramp_alpha)
        y, v, total, clamped, spring, damp = step_plant(y, v, target_y, aux_y)
        peak_force_post = max(peak_force_post, abs(total))
        clock_t[0] += DT
        t += DT
    return {"post_disarm_peak_force_n": peak_force_post, "final_state": align.state}


def simulate_old_architecture_disarm(duration_s=3.0):
    """OLD single-stage disarm: gravity and coupling both go to 0
    simultaneously, single alpha(t): 1->0 over ramp_duration_s."""
    y, v = Y_FULL_TARGET, 0.0
    peak_force_post = 0.0
    t = 0.0
    while t < duration_s:
        alpha = max(0.0, 1.0 - t / COUPLING_RAMP_DURATION)
        target_y = Y_START + alpha * (Y_FULL_TARGET - Y_START)  # blends back toward READY's own anchor
        # Old architecture's actual disarm target: own live pose at alpha=0
        # (post the unarmed-free-motion fix) -- target = y (live) once
        # alpha hits 0, blended from full_target at alpha=1. Approximate
        # with target = y + alpha*(Y_FULL_TARGET - y) since origin tracks y.
        target_y = y + alpha * (Y_FULL_TARGET - y) if alpha < 1.0 else Y_FULL_TARGET
        aux_y = ramped_gravity_comp_n(CONFIGURED_G, alpha)
        y, v, total, clamped, spring, damp = step_plant(y, v, target_y, aux_y)
        peak_force_post = max(peak_force_post, abs(total))
        t += DT
    return {"post_disarm_peak_force_n": peak_force_post}


if __name__ == "__main__":
    print("=" * 70)
    print("ARM-SIDE COMPARISON (same ~117mm initial gap, same plant/controller)")
    print("=" * 70)
    old = simulate_old_architecture()
    new = simulate_new_architecture()
    print(f"{'metric':<28} {'OLD (single-stage)':>20} {'NEW (staged)':>20}")
    print(f"{'peak |applied force| (N)':<28} {old['peak_force_n']:>20.4f} {new['peak_force_n']:>20.4f}")
    print(f"{'peak |position error| (m)':<28} {old['peak_error_m']:>20.4f} {new['peak_error_m']:>20.4f}")
    print(f"{'any force clamp':<28} {str(old['any_clamp']):>20} {str(new['any_clamp']):>20}")
    print(f"{'final y (should be ~0)':<28} {old['final_y']:>20.5f} {new['final_y']:>20.5f}")

    print()
    print("=" * 70)
    print("DISARM-SIDE COMPARISON (from steady-state ACTIVE)")
    print("=" * 70)
    old_d = simulate_old_architecture_disarm()
    new_d = simulate_new_architecture_disarm()
    print(f"{'metric':<28} {'OLD (single-stage)':>20} {'NEW (staged)':>20}")
    print(f"{'post-disarm peak force (N)':<28} {old_d['post_disarm_peak_force_n']:>20.4f} "
          f"{new_d['post_disarm_peak_force_n']:>20.4f}")

    print()
    print(f"NOTE: plant coefficient B={B} N*s/m is an ASSUMED illustrative value "
          f"(not independently measured for this device) -- see module docstring. "
          f"The comparison (same B for both columns) is the evidence, not either "
          f"column's absolute number alone.")
