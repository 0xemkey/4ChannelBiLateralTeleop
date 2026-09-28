#!/usr/bin/env python3
"""Phase 6: C1 gain (w1) characterization -- contact fidelity vs
free-motion false-force tradeoff.

Per the task's own rules: C1=0.5 is a BASELINE point, not assumed
correct. This sweeps w1 across a small, bounded grid (0.3 = C4's current
value, 0.5 = C1's current baseline, 0.7, 1.0 = upper bound, since w>1
would amplify beyond the source estimate) and measures TWO separate,
never-collapsed metrics per the task's Pareto-first rule:

  1. CONTACT GAIN: wall-contact condition (reuses the Phase 1/2/4B
     ground-truth recipe), steady-state |received relay torque| /
     |sim ground-truth contact torque|, tail-averaged (same
     clock-alignment-avoiding method as Phase 4B).
  2. FREE-MOTION FALSE FORCE: no contact, p99 |received relay torque| on
     the master during ordinary free settling (the same 3.0s settle +
     3.0s window methodology validated in Phase 4A/4B, reused here) --
     this is what an operator would feel as unwanted noise in free
     motion, and is a direct function of w1 * (DOB's own free-space
     floor, already characterized in Phase 4A).

Both measured in JOINT-TORQUE space (not Cartesian), same reasoning as
the original sign-audit test: T_sm (a master<->slave Cartesian frame
transform) is not established anywhere in this codebase (see
docs/SCIENTIFIC_SIGN_FRAME_DEFINITIONS.md), so comparing in joint space
sidesteps inventing one. Shoulder stays masked (the shipped default,
--c1-disable-joints=shoulder) -- this sweep characterizes the DEPLOYED
mask configuration, not the hypothetical unmasked one (already covered
separately in the prior root-cause investigation).

100% simulation, same hardware-independent infrastructure as every
other script in this directory.
"""
from __future__ import annotations

import glob
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import Collector, LiveNode, ScriptProcess, SimNode, unique_ns, read_truth_csv  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BILATERAL_SCRIPT = Path(
    "/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py")

W1_VALUES = [0.3, 0.5, 0.7, 1.0]
N_REPEATS = 5
WALL_Y = -0.05
TARGET_Y = -0.18
DRIVE_S = 4.0
TAIL_S = 1.0
SETTLE_S = 3.0  # Phase 4A/4B-validated settle duration
WINDOW_S = 3.0
_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731

OUT_JSON = Path(__file__).parent / "phase6_c1_gain_raw.json"


def _launch_pair(w1: float, ns_prefix: str):
    master_ns = unique_ns(f"{ns_prefix}_m")
    slave_ns = unique_ns(f"{ns_prefix}_s")
    master = SimNode(master_ns, device="Left",
                      params={"dob_enable": "true", "require_deadman": "false"})
    slave = SimNode(slave_ns, device="Default Device",
                     params={"dob_enable": "true", "require_deadman": "false",
                              "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                              "max_force_n": "1.5"})
    master.wait_started()
    slave.wait_started()
    return master, slave


def _launch_bilateral(master, slave, w1: float, status_ns: str, extra_slave_env=None):
    return ScriptProcess(
        [str(BILATERAL_SCRIPT),
         "--architecture", "custom", "--w1", str(w1), "--w2", "0.0",
         "--w3", "0.0", "--w4", "0.0",
         "--left-force-topic", master.topic("force_ext_hat"),
         "--right-force-topic", slave.topic("force_ext_hat"),
         "--left-joint-angles-topic", master.topic("joint_angles"),
         "--right-joint-angles-topic", slave.topic("joint_angles"),
         "--left-joint-target-topic", master.topic("joint_target"),
         "--right-joint-target-topic", slave.topic("joint_target"),
         "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
         "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
         "--status-topic", f"/{status_ns}",
         "--rate-hz", "200"],
        name=f"bilateral_{status_ns}",
    )


def contact_run(w1: float, run_idx: int) -> dict:
    w1_tag = str(w1).replace(".", "p")
    ns_prefix = unique_ns(f"p6c_w{w1_tag}")
    truth_prefix = f"/tmp/{ns_prefix}_truth"
    record = {"w1": w1, "run_idx": run_idx, "condition": "contact"}
    master, slave = None, None
    try:
        master_ns = unique_ns(f"{ns_prefix}_m")
        slave_ns = unique_ns(f"{ns_prefix}_s")
        master = SimNode(master_ns, device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(slave_ns, device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false",
                                  "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                                  "max_force_n": "1.5"},
                         sim_env={"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_TRUTH_CSV": truth_prefix})
        master.wait_started()
        slave.wait_started()

        with _launch_bilateral(master, slave, w1, unique_ns("p6c_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            import rclpy
            from rclpy.node import Node as RclpyNode
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Bool
            if not rclpy.ok():
                rclpy.init()
            driver = RclpyNode(f"driver_{ns_prefix}")
            target_pub = driver.create_publisher(PoseStamped, slave.topic("target_pose"), 10)
            enable_pub = driver.create_publisher(Bool, slave.topic("enable"), 10)
            with LiveNode("p6c_verify") as live:
                live.watch("left_ext", master.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                left_tail = []
                deadline = time.time() + DRIVE_S
                while time.time() < deadline:
                    enable_pub.publish(Bool(data=True))
                    pose = PoseStamped()
                    pose.pose.position.y = TARGET_Y
                    pose.pose.orientation.w = 1.0
                    target_pub.publish(pose)
                    rclpy.spin_once(driver, timeout_sec=0.02)
                    live.spin_once(timeout_sec=0.001)
                    if (time.time() - deadline) > -TAIL_S:
                        v = live.latest.get("left_ext")
                        if v is not None:
                            left_tail.append(v)
            driver.destroy_node()
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    matches = glob.glob(truth_prefix + "*.csv")
    if not matches:
        record["error"] = "no truth csv"
        return record
    rows = read_truth_csv(matches[0])
    if len(rows) < 50:
        record["error"] = f"truth csv too short {len(rows)}"
        return record
    n_tail = max(10, int(len(rows) * TAIL_S / DRIVE_S))
    truth_tail = rows[-n_tail:]
    tau_true = np.array([[r["tau_ext_true1"], r["tau_ext_true2"], r["tau_ext_true3"]] for r in truth_tail])
    mean_true_mag = float(np.mean(np.linalg.norm(tau_true, axis=1)))

    if len(left_tail) < 10:
        record["error"] = f"too few received samples {len(left_tail)}"
        return record
    recv = np.array(left_tail)
    mean_recv_mag = float(np.mean(np.linalg.norm(recv, axis=1)))
    mean_true_vec = tau_true.mean(axis=0)
    mean_recv_vec = recv.mean(axis=0)
    cos_sim = float(np.dot(mean_recv_vec, mean_true_vec) /
                     (np.linalg.norm(mean_recv_vec) * np.linalg.norm(mean_true_vec) + 1e-12))

    record.update(mean_true_mag=mean_true_mag, mean_recv_mag=mean_recv_mag,
                   gain=mean_recv_mag / mean_true_mag if mean_true_mag > 1e-6 else float("nan"),
                   cos_sim=cos_sim)
    return record


def freemotion_run(w1: float, run_idx: int) -> dict:
    w1_tag = str(w1).replace(".", "p")
    ns_prefix = unique_ns(f"p6f_w{w1_tag}")
    record = {"w1": w1, "run_idx": run_idx, "condition": "free_motion"}
    master, slave = None, None
    try:
        master_ns = unique_ns(f"{ns_prefix}_m")
        slave_ns = unique_ns(f"{ns_prefix}_s")
        master = SimNode(master_ns, device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(slave_ns, device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false",
                                  "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                                  "max_force_n": "1.5"})
        master.wait_started()
        slave.wait_started()

        with _launch_bilateral(master, slave, w1, unique_ns("p6f_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            with LiveNode("p6f_verify") as live:
                left_tail = []
                live.watch("left_ext", master.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                live.spin_for(SETTLE_S)
                settle_end = time.time() + WINDOW_S
                while time.time() < settle_end:
                    live.spin_once(timeout_sec=0.01)
                    v = live.latest.get("left_ext")
                    if v is not None:
                        left_tail.append(v)
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    if len(left_tail) < 10:
        record["error"] = f"too few samples {len(left_tail)}"
        return record
    recv = np.array(left_tail)
    mags = np.linalg.norm(recv, axis=1)
    record.update(p99_mag=float(np.percentile(mags, 99)), mean_mag=float(np.mean(mags)),
                   rms_mag=float(np.sqrt(np.mean(mags ** 2))))
    return record


def main() -> None:
    results = []
    for w1 in W1_VALUES:
        for r in range(N_REPEATS):
            rec = contact_run(w1, r)
            print(f"[contact] w1={w1} run {r}: gain={rec.get('gain', float('nan')):.4f} "
                  f"cos_sim={rec.get('cos_sim', float('nan')):.4f} {rec.get('error', '')}")
            results.append(rec)
        for r in range(N_REPEATS):
            rec = freemotion_run(w1, r)
            print(f"[free]    w1={w1} run {r}: p99={rec.get('p99_mag', float('nan')):.5f} "
                  f"{rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
