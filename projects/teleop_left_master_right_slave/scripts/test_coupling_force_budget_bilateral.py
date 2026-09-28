#!/usr/bin/env python3
"""Bilateral (BOTH arms) force-budget test, 2026-09-09 Part 1 of the
bilateral-force-budget follow-up task. Pure Python, no rclpy.

Reproduces the mandatory 117mm real-trial case with a genuine two-plant
(master + slave) discrete simulation: C2 target = slave's live position,
C3 target = master's live position (absolute-axis semantics, matching the
real bridge), driven by the SAME shared coupling_ramp_alpha via
step_bilateral(), with the corrected predictor (per-axis position-error
clamp applied per Part 3's controller audit) and REAL configured per-arm
parameters:

    master: stiffness=15 N/m, damping=0.10 N*s/m, max_force=2.0N,
            max_position_error_m=0.03 (run_master_left.sh defaults)
    slave:  stiffness=35 N/m, damping=0.20 N*s/m, max_force=1.5N,
            max_position_error_m=0.04 (run_slave_right.sh defaults)

Uses the discrete multi-rate model (200Hz controller / 100Hz bridge
sample-and-hold), matching compare_arm_strategies_discrete.py.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from coupling_force_budget import (  # noqa: E402
    AdaptiveCouplingRamp, ArmState, ForceBudgetConfig, CouplingRampStalled, predict_force_vector, _norm,
)

CONTROLLER_DT = 0.005
BRIDGE_DT = 0.010
STEPS_PER_BRIDGE_TICK = int(round(BRIDGE_DT / CONTROLLER_DT))
G_TRUE = 0.70
B = 3.0  # ASSUMED plant coefficient, see compare_old_vs_staged_transition.py

MASTER = dict(stiffness=15.0, damping=0.10, max_force=2.0, max_pos_err=0.03)
SLAVE = dict(stiffness=35.0, damping=0.20, max_force=1.5, max_pos_err=0.04)


def controller_tick(y, v, target_y, aux_y, stiffness, damping, max_force, g_true=0.0):
    spring = stiffness * (target_y - y)
    damp = -damping * v
    raw = spring + damp + aux_y
    total = max(-max_force, min(max_force, raw))
    clamped = abs(total - raw) > 1e-9
    net = total - g_true
    v_new = net / B
    y_new = y + v_new * CONTROLLER_DT
    return y_new, v_new, total, clamped, spring


def test_bilateral_117mm_case_hard_criteria():
    master_y, master_v = 0.0, 0.0             # master held at its reference pose
    slave_y, slave_v = -0.1174, 0.0            # slave sagged, real 117mm gap

    cfg = ForceBudgetConfig(force_reserve_n=0.1, nominal_duration_s=1.0, stall_timeout_s=3.0)
    ramp = AdaptiveCouplingRamp(cfg, origin_mode="snapshot")

    t = 0.0
    ramp.start_bilateral(now=t, master_origin_pose=[0.0, master_y, 0.0],
                         slave_origin_pose=[0.0, slave_y, 0.0])

    peak_master_force = peak_slave_force = 0.0
    peak_master_err = peak_slave_err = 0.0
    any_master_clamp = any_slave_clamp = False
    any_master_pos_over = any_slave_pos_over = False
    stalled = False
    prev_alpha = 0.0
    alphas = []
    convergence_t = None

    n_ticks = int(6.0 / BRIDGE_DT)
    for i in range(n_ticks):
        master = ArmState(
            live_pose=[0.0, master_y, 0.0], velocity=[0.0, master_v, 0.0],
            full_target=[0.0, slave_y, 0.0],  # C2: master target = slave's CURRENT live position
            stiffness_n_per_m=MASTER["stiffness"], damping_n_s_per_m=MASTER["damping"],
            aux_vec=[0.0, 0.0, 0.0], max_force_n=MASTER["max_force"], max_position_error_m=MASTER["max_pos_err"],
        )
        slave = ArmState(
            live_pose=[0.0, slave_y, 0.0], velocity=[0.0, slave_v, 0.0],
            full_target=[0.0, master_y, 0.0],  # C3: slave target = master's CURRENT live position
            stiffness_n_per_m=SLAVE["stiffness"], damping_n_s_per_m=SLAVE["damping"],
            aux_vec=[0.0, G_TRUE, 0.0], max_force_n=SLAVE["max_force"], max_position_error_m=SLAVE["max_pos_err"],
        )
        try:
            alpha = ramp.step_bilateral(now=t, dt=BRIDGE_DT, master=master, slave=slave)
        except CouplingRampStalled:
            stalled = True
            break
        assert alpha >= prev_alpha - 1e-12, "alpha must be monotonically non-decreasing"
        prev_alpha = alpha
        alphas.append(alpha)

        master_target_y = ramp.current_target_bilateral("master", [0.0, master_y, 0.0], [0.0, slave_y, 0.0])[1]
        slave_target_y = ramp.current_target_bilateral("slave", [0.0, slave_y, 0.0], [0.0, master_y, 0.0])[1]

        for _ in range(STEPS_PER_BRIDGE_TICK):
            master_y, master_v, m_total, m_clamped, m_spring = controller_tick(
                master_y, master_v, master_target_y, 0.0, MASTER["stiffness"], MASTER["damping"], MASTER["max_force"])
            slave_y, slave_v, s_total, s_clamped, s_spring = controller_tick(
                slave_y, slave_v, slave_target_y, G_TRUE, SLAVE["stiffness"], SLAVE["damping"], SLAVE["max_force"],
                g_true=G_TRUE)
            peak_master_force = max(peak_master_force, abs(m_total))
            peak_slave_force = max(peak_slave_force, abs(s_total))
            any_master_clamp = any_master_clamp or m_clamped
            any_slave_clamp = any_slave_clamp or s_clamped
            m_err = abs(master_target_y - master_y)
            s_err = abs(slave_target_y - slave_y)
            peak_master_err = max(peak_master_err, m_err)
            peak_slave_err = max(peak_slave_err, s_err)
            any_master_pos_over = any_master_pos_over or m_err > MASTER["max_pos_err"]
            any_slave_pos_over = any_slave_pos_over or s_err > SLAVE["max_pos_err"]

        t += BRIDGE_DT
        if alpha >= 1.0 and convergence_t is None:
            convergence_t = t

    print(f"peak_master_force={peak_master_force:.4f}N (limit {MASTER['max_force']}N) "
          f"any_clamp={any_master_clamp}")
    print(f"peak_slave_force={peak_slave_force:.4f}N (limit {SLAVE['max_force']}N) "
          f"any_clamp={any_slave_clamp}")
    print(f"peak_master_err={peak_master_err*1000:.2f}mm (real baseline 47.2mm) "
          f"any_over_configured_clamp={any_master_pos_over}")
    print(f"peak_slave_err={peak_slave_err*1000:.2f}mm (real baseline 32.4mm) "
          f"any_over_configured_clamp={any_slave_pos_over}")
    print(f"stalled={stalled} convergence_t={convergence_t} alpha_final={prev_alpha}")

    assert not stalled, "bilateral 117mm case must not stall"
    assert convergence_t is not None, "must reach alpha=1.0 within the test window"
    assert not any_master_clamp, "master must never hit its primary force clamp"
    assert not any_slave_clamp, "slave must never hit its primary force clamp"
    assert not any_master_pos_over, "master must never exceed its configured position-error limit"
    assert not any_slave_pos_over, "slave must never exceed its configured position-error limit"
    assert peak_master_err < 0.0472, "master peak error should be no worse than the real (contaminated) baseline"
    assert peak_slave_err < 0.0324, "slave peak error should be no worse than the real (contaminated) baseline"
    print("HARD CRITERIA: no clamps, no stall, converged, alpha monotonic. PASS")


if __name__ == "__main__":
    test_bilateral_117mm_case_hard_criteria()
    print("\nALL TESTS PASSED")
