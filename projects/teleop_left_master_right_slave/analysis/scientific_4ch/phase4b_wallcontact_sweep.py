#!/usr/bin/env python3
"""Phase 4B redesign: g_dis characterization under ACTIVE WALL CONTACT
(user-selected option, 2026-09-08), reusing the Phase 1/2 sign-test's
proven wall-contact + ground-truth-CSV recipe instead of Phase 4B's
original passive free-space settle (which turned out g_dis-invariant by
construction -- see docs/SCIENTIFIC_DOB_VALIDATION.md).

100% simulation (touch_slave_sim/hd_sim, "Default Device" -> gets the
wall per hd_sim_api.cpp's is_right split). Same 3 candidate g_dis values
as before (none invented): 2*pi*5, 10, 5 rad/s, via the
dob_g_dis_override_rad_s ROS param added this session.

Primary metric: STEADY-STATE GAIN = mean(|tau_dis_hat| tail) /
mean(|tau_ext_true| tail), tail-averaged over the last 1.0s of a 4.0s
contact hold -- robust to any clock-alignment error between the two
streams (DobDiagnostics uses ROS/wall clock, the truth CSV uses the
sim's own t_s), since it only needs each stream's OWN steady tail, not
cross-stream time alignment.

Onset delay is explicitly NOT measured this pass -- the two streams use
different clocks (ROS wall-clock timestamps vs. the truth CSV's internal
t_s) and no reliable common time base was set up; reporting a delay
number from a shaky alignment would be worse than not reporting one.
INSUFFICIENT_DATA for onset/release delay under this design; a future
pass could add a shared time reference if delay becomes the operative
question (e.g. for Phase 6's C1/C4 characterization).
"""
from __future__ import annotations

import glob
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import Collector, SimNode, unique_ns, read_truth_csv  # noqa: E402

from touch_haptics.msg import DobDiagnostics  # noqa: E402

CANDIDATES = {
    "g_dis_2pi5": 2.0 * np.pi * 5.0,
    "g_dis_10": 10.0,
    "g_dis_5": 5.0,
}
N_REPEATS = 10
WALL_Y = -0.05
TARGET_Y = -0.18
DRIVE_S = 4.0
TAIL_S = 1.0
JOINT_INDEX = {"waist": 0, "shoulder": 1, "elbow": 2}
OUT_JSON = Path(__file__).parent / "phase4b_wallcontact_raw.json"


def one_run(condition: str, g_dis: float, run_idx: int) -> dict:
    ns = unique_ns(f"p4bwc_{condition}_r{run_idx}")
    truth_prefix = f"/tmp/{ns}_truth"
    record = {"condition": condition, "g_dis": g_dis, "run_idx": run_idx}
    with SimNode(ns, device="Default Device",
                 params={"require_deadman": "false", "stiffness_n_per_m": "35.0",
                          "damping_n_s_per_m": "0.5", "max_force_n": "1.0",
                          "dob_g_dis_override_rad_s": str(g_dis)},
                 sim_env={"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_TRUTH_CSV": truth_prefix}) as sim:
        if "DOB bank configured" not in sim.log_text():
            record["error"] = "DOB never configured"
            return record

        import rclpy
        from rclpy.node import Node as RclpyNode
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool

        if not rclpy.ok():
            rclpy.init()
        driver = RclpyNode(f"driver_{ns}")
        target_pub = driver.create_publisher(PoseStamped, sim.topic("target_pose"), 10)
        enable_pub = driver.create_publisher(Bool, sim.topic("enable"), 10)
        diag_c = Collector(sim.topic("dob_diagnostics"), DobDiagnostics)
        try:
            deadline = time.time() + DRIVE_S
            while time.time() < deadline:
                enable_pub.publish(Bool(data=True))
                pose = PoseStamped()
                pose.pose.position.y = TARGET_Y
                pose.pose.orientation.w = 1.0
                target_pub.publish(pose)
                rclpy.spin_once(driver, timeout_sec=0.02)
            diag_c.spin_for(0.5)
        finally:
            diag_c.close()
            driver.destroy_node()

    matches = glob.glob(truth_prefix + "*.csv")
    if not matches:
        record["error"] = "no truth csv"
        return record
    rows = read_truth_csv(matches[0])
    if len(rows) < 50:
        record["error"] = f"truth csv too short: {len(rows)}"
        return record

    n_tail_truth = max(10, int(len(rows) * TAIL_S / DRIVE_S))
    truth_tail = rows[-n_tail_truth:]
    tau_true = np.array([[r["tau_ext_true1"], r["tau_ext_true2"], r["tau_ext_true3"]] for r in truth_tail])
    mean_true_mag = float(np.mean(np.linalg.norm(tau_true, axis=1)))

    tail_msgs = diag_c.messages[-int(len(diag_c.messages) * TAIL_S / (DRIVE_S + 0.5)):] if diag_c.messages else []
    if len(tail_msgs) < 10:
        record["error"] = f"too few diag samples: {len(tail_msgs)}"
        return record
    tau_hat = np.array([m.tau_dis_hat_nm for m in tail_msgs])
    mean_hat_mag = float(np.mean(np.linalg.norm(tau_hat, axis=1)))

    record["n_truth_tail"] = len(truth_tail)
    record["n_diag_tail"] = len(tail_msgs)
    record["mean_true_mag"] = mean_true_mag
    record["mean_hat_mag"] = mean_hat_mag
    record["gain"] = mean_hat_mag / mean_true_mag if mean_true_mag > 1e-6 else float("nan")
    return record


def main() -> None:
    results = []
    for cond, g_dis in CANDIDATES.items():
        for r in range(N_REPEATS):
            t0 = time.time()
            rec = one_run(cond, g_dis, r)
            dur = time.time() - t0
            print(f"{cond:12s} run {r:2d}: gain={rec.get('gain', float('nan')):.4f} "
                  f"true_mag={rec.get('mean_true_mag', float('nan')):.5f} "
                  f"hat_mag={rec.get('mean_hat_mag', float('nan')):.5f} "
                  f"dur={dur:.2f}s {rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
