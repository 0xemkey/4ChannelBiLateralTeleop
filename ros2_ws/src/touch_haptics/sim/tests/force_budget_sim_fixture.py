"""Deterministic initial-condition fixture for the force-aware coupling
validation (2026-09-09, validation-infrastructure task).

WHY THIS EXISTS: the earlier live stress test tried to create a large
master/slave separation by publishing a force to the slave's force_cmd and
then releasing it. That was non-deterministic and did not survive -- see the
diagnosis below -- and is explicitly not acceptable as the fixture for the
mandatory large-gap validation.

DIAGNOSIS THAT LED TO THIS DESIGN (all measured, not assumed):

  1. hd_sim_api.cpp ALREADY supports deterministic initial pose seeding via
     the TOUCH_SIM_INIT_POS_X/Y/Z environment variables (they set
     ArmParams::has_init_pose_m/init_pose_m, which ArmModel::reset() feeds
     to solveIkToPositionMultiStart). sim_common.SimNode ALREADY forwards
     per-process env via its `sim_env` argument. No new injection mechanism
     needed -- it existed and was simply not being used.

  2. Seeding alone is NOT enough, and the reason is physical, not a bug: an
     arm with no controller force holding it SAGS under the simulator's own
     gravity to a hanging equilibrium. Measured directly: seeded at
     y=-0.100, the tip is at y=-0.126 by the first telemetry sample (1ms)
     and then sags monotonically to y=-0.2107 by ~2s, every run. That
     equilibrium (y ~= -0.2108) is also, independently, the minimum Y the
     joint limits allow (analytically [-0.2112, +0.2507] for
     y = L1*sin(q2) - L2*cos(q3) over q2 in [-0.6,1.6], q3 in [-0.3,2.6]).
     This also explains the earlier "both devices drift to a shared
     attractor" observation -- they were both simply sagging.

     This is NOT an IK bug (checked: the IK does place the tip near the
     requested pose at t=0) and NOT a simulator defect. It is the sim
     faithfully reproducing the exact real-world condition the failed
     hardware trial was about: an unheld, un-gravity-compensated slave sags.

  3. Therefore the physically faithful way to create a persistent gap is the
     same way the real trial created one: HOLD THE MASTER and let the SLAVE
     SAG. hd_sim's Hand model is exactly that -- an operator's hand, and it
     already belongs to the master only (hd_sim_api.cpp forces k_hand=0 for
     right/"Default" devices). Setting TOUCH_SIM_HAND=stiff (k=400 N/m) plus
     TOUCH_SIM_HAND_MOTION=hold makes the master hold at its seeded pose,
     while the slave sags to its equilibrium.

RESULT: a deterministic, repeatable, physically-meaningful Y-dominant gap,
tunable by choosing where the master is held. Measured (3s settle):

    master seed y=-0.100 -> gap_y=108.75mm
    master seed y=-0.060 -> gap_y=148.82mm
    master seed y=-0.020 -> gap_y=189.24mm
    master seed y=+0.020 -> gap_y=229.78mm

i.e. gap_y ~= (master_held_y - SLAVE_SAG_EQUILIBRIUM_Y), with a small,
constant ~2mm static droop of the master against its own hand stiffness.

This fixture is SIMULATION-ONLY: it drives nothing but environment
variables read by hd_sim_api.cpp, which is only ever linked into
touch_slave_sim (real hardware links libHD). It changes no control
semantics, no gains, no limits, and nothing in the production bridge.
"""
from __future__ import annotations

from typing import Dict, Tuple

# Measured constants of this simulator (see the module docstring). Both are
# properties of the sim's own arm model, not tunables of the system under
# test -- they are used only to CHOOSE a seed, and every test that uses this
# fixture MEASURES the achieved gap rather than assuming it.
SLAVE_SAG_EQUILIBRIUM_Y_M = -0.2108
MASTER_HAND_DROOP_M = 0.0020

# The X/Z the fixture anchors both arms near. Chosen inside the reachable
# region so the IK seed is feasible (see the analytic workspace check in the
# module docstring).
ANCHOR_X_M = -0.098
ANCHOR_Z_M = 0.030


def master_seed_y_for_gap(gap_m: float) -> float:
    """Seed Y that yields the requested master-vs-slave Y gap once the master
    has settled against its hand and the slave has sagged."""
    return SLAVE_SAG_EQUILIBRIUM_Y_M + gap_m + MASTER_HAND_DROOP_M


def master_sim_env(gap_m: float) -> Dict[str, str]:
    """Env for the MASTER sim process: seeded at the pose that produces
    `gap_m`, and held there by a stiff hand (the operator's grip)."""
    return {
        "TOUCH_SIM_INIT_POS_X": str(ANCHOR_X_M),
        "TOUCH_SIM_INIT_POS_Y": str(master_seed_y_for_gap(gap_m)),
        "TOUCH_SIM_INIT_POS_Z": str(ANCHOR_Z_M),
        "TOUCH_SIM_HAND": "stiff",
        "TOUCH_SIM_HAND_MOTION": "hold",
    }


def slave_sim_env() -> Dict[str, str]:
    """Env for the SLAVE sim process: seeded at the same anchor, but with no
    hand (hd_sim forces k_hand=0 for right devices anyway), so it sags to its
    own gravity equilibrium exactly as the real un-compensated slave does."""
    return {
        "TOUCH_SIM_INIT_POS_X": str(ANCHOR_X_M),
        "TOUCH_SIM_INIT_POS_Y": str(SLAVE_SAG_EQUILIBRIUM_Y_M),
        "TOUCH_SIM_INIT_POS_Z": str(ANCHOR_Z_M),
    }


# Controller parameters the real rig runs, reused by every test that needs
# both arms configured consistently with the bridge's planner args.
MASTER_PARAMS: Dict[str, str] = {
    "stiffness_n_per_m": "15.0", "damping_n_s_per_m": "0.10",
    "max_position_error_m": "0.03", "max_force_n": "2.0",
    "require_deadman": "false",
}
SLAVE_PARAMS: Dict[str, str] = {
    "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
    "max_position_error_m": "0.04", "max_force_n": "1.5",
    "require_deadman": "false",
}

SETTLE_S = 3.0  # measured: both arms flat well before this (see docstring)


def measured_gap(master_pos, slave_pos) -> Tuple[float, float]:
    """(signed Y gap, full 3D gap norm), both in metres."""
    dy = master_pos[1] - slave_pos[1]
    norm = sum((master_pos[i] - slave_pos[i]) ** 2 for i in range(3)) ** 0.5
    return dy, norm
