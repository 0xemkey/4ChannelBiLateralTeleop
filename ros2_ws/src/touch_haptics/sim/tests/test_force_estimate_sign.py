"""The single most important test in this suite: does tau_ext_hat = -tau_dis_hat
actually point the same way as the sim's ground-truth contact torque?

Everything else in this project -- deadbands, presets, the energy tank -- is
worthless if this sign is wrong, because a force channel with the wrong sign
pushes AWAY from a wall instead of resisting it, which is worse than no force
channel at all (see touch_dob.hpp's header note on this exact failure mode).

Drives the RIGHT device (hd_sim's is_right split gives it the wall
environment, never a hand) into its default wall at y=-0.05 using the
EXISTING Cartesian target_pose path -- this does not depend on 0.5's
joint-space wiring, so it isolates the sign question from that newer code.
hd_sim logs ground truth (tau_ext_true = J^T(q_true) * F_env_true) to
TOUCH_SIM_TRUTH_CSV independently of anything the node estimates.
"""
from __future__ import annotations

import glob
import sys
import time

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from sim_common import Collector, SimNode, read_truth_csv, unique_ns  # noqa: E402

from touch_haptics.msg import DobDiagnostics

WALL_Y = -0.05
TARGET_Y = -0.18  # well past the wall, so the spring holds firm contact


def test_force_estimate_sign_matches_ground_truth():
    ns = unique_ns("fesign")
    truth_prefix = f"/tmp/{ns}_truth"

    with SimNode(
        ns, device="Default Device",
        params={"require_deadman": "false", "stiffness_n_per_m": "35.0",
                "damping_n_s_per_m": "0.5", "max_force_n": "1.0"},
        sim_env={"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_TRUTH_CSV": truth_prefix},
    ) as sim:
        import rclpy
        from rclpy.node import Node as RclpyNode

        if not rclpy.ok():
            rclpy.init()
        driver = RclpyNode(f"driver_{ns}")
        target_pub = driver.create_publisher(PoseStamped, sim.topic("target_pose"), 10)
        enable_pub = driver.create_publisher(Bool, sim.topic("enable"), 10)

        diag_c = Collector(sim.topic("dob_diagnostics"), DobDiagnostics)
        try:
            # Drive the spring into the wall for a few seconds, republishing
            # every tick like bridge_master_slave.py does -- the node zeroes
            # force on commands older than stale_timeout_s.
            deadline = time.time() + 4.0
            while time.time() < deadline:
                enable_pub.publish(Bool(data=True))
                pose = PoseStamped()
                pose.pose.position.y = TARGET_Y
                pose.pose.orientation.w = 1.0
                target_pub.publish(pose)
                rclpy.spin_once(driver, timeout_sec=0.02)

            diag_c.spin_for(0.5)
            tail = diag_c.messages[-200:]
            assert len(tail) > 20, "not enough dob_diagnostics samples collected"
        finally:
            diag_c.close()
            driver.destroy_node()

    # SimNode.__exit__ has already sent SIGINT and waited for clean shutdown,
    # which is what flushes and closes the truth CSV (see hdDisableDevice).
    matches = glob.glob(truth_prefix + "*.csv")
    assert matches, f"no truth CSV written at {truth_prefix}*.csv"
    rows = read_truth_csv(matches[0])
    assert len(rows) > 50, f"truth CSV too short: {len(rows)} rows"

    truth_tail = rows[-200:]
    tau_ext_true = np.array([
        [r["tau_ext_true1"], r["tau_ext_true2"], r["tau_ext_true3"]] for r in truth_tail
    ])
    mean_true = tau_ext_true.mean(axis=0)
    assert np.linalg.norm(mean_true) > 1e-4, (
        "ground truth shows no contact torque -- the target never reached the "
        "wall; check TARGET_Y / WALL_Y / stiffness_n_per_m"
    )

    tau_dis_hat = np.array([m.tau_dis_hat_nm for m in tail])
    tau_ext_hat = -tau_dis_hat  # msg/ForceEstimate.msg's documented sign
    mean_hat = tau_ext_hat.mean(axis=0)

    dot = float(np.dot(mean_hat, mean_true))
    assert dot > 0.0, (
        f"SIGN MISMATCH: mean tau_ext_hat={mean_hat}, mean tau_ext_true={mean_true}, "
        f"dot={dot:.6f}. This is the failure mode that pushes AWAY from a wall."
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
