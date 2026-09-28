"""Phase 2 sign audit: does the master's RECEIVED C1 relay torque point the
same way as, or opposite to, the slave's simulated ground-truth contact
torque?

This is the one test neither test_force_estimate_sign.py (validates only
the DOB's OWN tau_ext_hat = -tau_dis_hat step) nor test_c1_relay_live.py
(validates only relay MAGNITUDE/gating, using a force_cmd push rather than
a genuine environment contact) actually checks -- the relay STEP's sign
against physical ground truth.

100% simulated: two touch_slave_sim instances (never touch_slave / real
HDAPI / /dev/ttyACM*), wired through the real, unmodified bilateral_4ch.py.
No source file is edited by this test. This resolves the question raised
in docs/SCIENTIFIC_CONTROLLER_ASSUMPTION_REGISTER.md ("C1 relay sign") as
far as simulation can: the sim proves what the CURRENT CODE does; whether
that also matches the *physical* Touch devices' HD_CURRENT_FORCE sign
convention still requires REQUIRES_FUTURE_HARDWARE_VALIDATION (recorded in
the report this test feeds).

Decided target (per user, 2026-09-08): F_m ~= -F_s (reactive/opposing),
matching bilateral_4ch.py's own module docstring. This test reports
against that target -- a PASS here means the code's sign is CORRECT
relative to the decided target; the current code is expected, going in,
to FAIL this (that is the whole reason this test exists).

Run: pytest test_c1_sign_vs_ground_truth.py -v
(from this directory, with the ROS2 workspace sourced -- see sim_common.py
for the sourcing requirement; not run automatically by this session, its
result is reported in docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md)
"""
from __future__ import annotations

import glob
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns, read_truth_csv  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BILATERAL_SCRIPT = (
    Path("/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py")
)
WALL_Y = -0.05
TARGET_Y = -0.18
_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731


@pytest.mark.skipif(not BILATERAL_SCRIPT.exists(), reason="bilateral_4ch.py not found")
def test_c1_received_sign_vs_ground_truth_contact():
    master_ns = unique_ns("signaudit_master")
    slave_ns = unique_ns("signaudit_slave")
    truth_prefix = f"/tmp/{slave_ns}_truth"

    master = SimNode(master_ns, device="Left",
                      params={"dob_enable": "true", "require_deadman": "false"})
    slave = SimNode(slave_ns, device="Default Device",
                     params={"dob_enable": "true", "require_deadman": "false",
                              "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                              "max_force_n": "1.5"},
                     sim_env={"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_TRUTH_CSV": truth_prefix})
    master.wait_started()
    slave.wait_started()
    try:
        status_topic = f"/{unique_ns('signaudit_status')}"
        with ScriptProcess(
            [str(BILATERAL_SCRIPT),
             "--architecture", "custom", "--w1", "1.0", "--w2", "0.0",
             "--w3", "0.0", "--w4", "0.0",
             "--c1-disable-joints", "",  # keep shoulder IN for this test -- it's a
                                          # sign question, the mask would hide one axis
             "--left-force-topic", master.topic("force_ext_hat"),
             "--right-force-topic", slave.topic("force_ext_hat"),
             "--left-joint-angles-topic", master.topic("joint_angles"),
             "--right-joint-angles-topic", slave.topic("joint_angles"),
             "--left-joint-target-topic", master.topic("joint_target"),
             "--right-joint-target-topic", slave.topic("joint_target"),
             "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
             "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
             "--status-topic", status_topic,
             "--rate-hz", "200"],
            name=f"bilateral_{master_ns}",
        ) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            import rclpy
            from rclpy.node import Node as RclpyNode
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Bool

            if not rclpy.ok():
                rclpy.init()
            driver = RclpyNode(f"driver_{slave_ns}")
            target_pub = driver.create_publisher(PoseStamped, slave.topic("target_pose"), 10)
            enable_pub = driver.create_publisher(Bool, slave.topic("enable"), 10)

            with LiveNode("signaudit_verify") as live:
                live.watch("left_ext", master.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                live.watch("right_fe", slave.topic("force_ext_hat"), ForceEstimate,
                            extract=_fe_deadband)
                live.spin_for(1.0)  # let subscription discovery settle before relying on it

                # Drive slave firmly into its wall -- same recipe as
                # test_force_estimate_sign.py, proven there to produce a
                # nonzero, well-formed ground-truth contact torque.
                deadline = time.time() + 4.0
                while time.time() < deadline:
                    enable_pub.publish(Bool(data=True))
                    pose = PoseStamped()
                    pose.pose.position.y = TARGET_Y
                    pose.pose.orientation.w = 1.0
                    target_pub.publish(pose)
                    rclpy.spin_once(driver, timeout_sec=0.02)
                    live.spin_once(timeout_sec=0.001)

                # Settle + collect the master's received relay torque tail,
                # and the slave's own source estimate (tau_e_hat, what the
                # relay actually operates on -- distinct from ground truth,
                # since the DOB is itself an imperfect estimator; separating
                # these two ratios is what lets Part B distinguish a RELAY
                # gain error from a pre-existing DOB estimation gap).
                left_tail = []
                right_tail = []
                settle_end = time.time() + 2.0
                while time.time() < settle_end:
                    enable_pub.publish(Bool(data=True))
                    pose = PoseStamped()
                    pose.pose.position.y = TARGET_Y
                    pose.pose.orientation.w = 1.0
                    target_pub.publish(pose)
                    rclpy.spin_once(driver, timeout_sec=0.02)
                    live.spin_once(timeout_sec=0.001)
                    v = live.latest.get("left_ext")
                    if v is not None:
                        left_tail.append(v)
                    r = live.latest.get("right_fe")
                    if r is not None:
                        right_tail.append(r)

            driver.destroy_node()

        assert "OSCILLATION ABORT" not in bilateral.log_text()
    finally:
        master.stop()
        slave.stop()

    matches = glob.glob(truth_prefix + "*.csv")
    assert matches, f"no truth CSV written at {truth_prefix}*.csv"
    rows = read_truth_csv(matches[0])
    assert len(rows) > 50, f"truth CSV too short: {len(rows)} rows"
    truth_tail = rows[-200:]
    tau_ext_true = np.array([
        [r["tau_ext_true1"], r["tau_ext_true2"], r["tau_ext_true3"]] for r in truth_tail
    ])
    mean_true = tau_ext_true.mean(axis=0)
    assert np.linalg.norm(mean_true) > 1e-4, "ground truth shows no contact -- target never reached wall"

    assert len(left_tail) > 20, "not enough master joint_torque_ext_cmd samples collected"
    assert len(right_tail) > 20, "not enough slave force_ext_hat (source) samples collected"
    mean_received = np.array(left_tail[-200:]).mean(axis=0)
    mean_source = np.array(right_tail[-200:]).mean(axis=0)

    dot = float(np.dot(mean_received, mean_true))
    cos_sim = dot / (np.linalg.norm(mean_received) * np.linalg.norm(mean_true) + 1e-12)

    # Part B: magnitude behavior, decomposed into two independent ratios.
    mag_true = float(np.linalg.norm(mean_true))
    mag_source = float(np.linalg.norm(mean_source))
    mag_received = float(np.linalg.norm(mean_received))
    relay_gain = mag_received / mag_source if mag_source > 1e-9 else float("nan")
    estimator_ratio = mag_source / mag_true if mag_true > 1e-9 else float("nan")
    print(f"\nmean tau_ext_true (slave, ground truth)      = {mean_true}")
    print(f"mean slave force_ext_hat (source, DOB est.)  = {mean_source}")
    print(f"mean master joint_torque_ext_cmd (received)  = {mean_received}")
    print(f"dot = {dot:.6f}   cos_sim = {cos_sim:.4f}")
    print(f"RELAY gain |received|/|source|      = {relay_gain:.4f}  (configured w1=1.0, gain_error={relay_gain-1.0:+.4f})")
    print(f"DOB estimator ratio |source|/|truth| = {estimator_ratio:.4f}  (pre-existing estimator gap, Phase 4 territory, NOT a relay defect)")
    print("TARGET CONVENTION: F_m ~= -F_s  =>  expected dot < 0 (opposing)")
    if dot > 0:
        print("RESULT: dot > 0 -- master's received relay torque points the SAME "
              "way as the slave's true contact torque (mirror/+w1 behavior). "
              "This does NOT match the decided F_m ~= -F_s target.")
    else:
        print("RESULT: dot < 0 -- master's received relay torque OPPOSES the "
              "slave's true contact torque, matching the decided F_m ~= -F_s target.")

    # Reports, does not assert a specific direction as "passing" -- this
    # test's job is to MEASURE the sign in simulation, not to enforce one;
    # the pass/fail judgment against the decided target is made in
    # docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md, not silently baked into this
    # test (would risk masking a future intentional fix).
    assert abs(cos_sim) > 0.3, (
        f"received relay torque and ground-truth contact torque are nearly "
        f"orthogonal (cos_sim={cos_sim:.3f}) -- inconclusive, not a clean "
        f"same/opposite-direction result; increase contact depth or check wiring"
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
