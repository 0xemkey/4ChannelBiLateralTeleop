#!/usr/bin/env python3
"""Phase 6, Next Step A: C1 active-motion false-force/noise test.

Goal: does increasing w1 amplify estimator contamination transmitted to
the master during genuine slave MOTION, with NO true external contact?
The prior free-motion (settled, at-rest) test measured exactly zero
false force at every gain -- too quiet a condition to reveal a
contact-fidelity-vs-noise tradeoff. This drives the slave through a
sustained sinusoidal Cartesian trajectory (target_pose, no wall/hand
model) instead, producing genuine velocity/acceleration content for the
DOB to (potentially) misread as external disturbance.

Same shoulder-only isolation as the contact-gain sweep
(--c1-disable-joints waist,elbow), for direct comparability and to keep
the SHOULDER-only claim-scope intact (do not generalize to elbow/waist,
per the standing claim limits).

100% simulation, hardware-independent (touch_slave_sim/hd_sim +
bilateral_4ch.py). No wall, no hand, no contact ground truth needed --
by definition, ANY relayed signal here is false (there is no true
external disturbance), so no ground-truth CSV is required.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BILATERAL_SCRIPT = Path(
    "/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py")

W1_VALUES = [0.3, 0.5, 0.7, 1.0]
N_REPEATS = 5
MOTION_S = 6.0
SETTLE_S = 2.0
AMP_M = 0.08
FREQ_HZ = 3.0
SHOULDER_IDX = 1
_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731

OUT_JSON = Path(__file__).parent / "phase6_c1_active_motion_raw.json"


def _launch_bilateral(master, slave, w1: float, status_ns: str):
    return ScriptProcess(
        [str(BILATERAL_SCRIPT),
         "--architecture", "custom", "--w1", str(w1), "--w2", "0.0",
         "--w3", "0.0", "--w4", "0.0",
         "--c1-disable-joints", "waist,elbow",
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


def active_motion_run(w1: float, run_idx: int) -> dict:
    w1_tag = str(w1).replace(".", "p")
    ns_prefix = unique_ns(f"p6am_w{w1_tag}")
    record = {"w1": w1, "run_idx": run_idx, "condition": "active_motion"}
    master, slave, bl_log_text = None, None, ""
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false",
                                  "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                                  "max_force_n": "1.5"})
        master.wait_started()
        slave.wait_started()

        with _launch_bilateral(master, slave, w1, unique_ns("p6am_status")) as bilateral:
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
            with LiveNode("p6am_verify") as live:
                live.watch("left_ext", master.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                # settle first (no motion commanded, target = start pose implicitly
                # via first published pose below)
                t0 = time.time()
                base_y = None
                while time.time() < t0 + SETTLE_S:
                    rclpy.spin_once(driver, timeout_sec=0.02)
                    live.spin_once(timeout_sec=0.001)
                tail = []
                t1 = time.time()
                while time.time() < t1 + MOTION_S:
                    t = time.time() - t1
                    y = -0.10 + AMP_M * math.sin(2.0 * math.pi * FREQ_HZ * t)
                    enable_pub.publish(Bool(data=True))
                    pose = PoseStamped()
                    pose.pose.position.y = y
                    pose.pose.orientation.w = 1.0
                    target_pub.publish(pose)
                    rclpy.spin_once(driver, timeout_sec=0.02)
                    live.spin_once(timeout_sec=0.001)
                    v = live.latest.get("left_ext")
                    if v is not None:
                        tail.append(v)
            driver.destroy_node()
            bl_log_text = bilateral.log_text()
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    record["oscillation_abort"] = "OSCILLATION ABORT" in bl_log_text
    if len(tail) < 10:
        record["error"] = f"too few samples {len(tail)}"
        return record
    recv = np.array(tail)
    shoulder = recv[:, SHOULDER_IDX]
    mag = np.abs(shoulder)
    record.update(
        rms_nm=float(np.sqrt(np.mean(mag ** 2))),
        p95_nm=float(np.percentile(mag, 95)),
        p99_nm=float(np.percentile(mag, 99)),
        peak_nm=float(np.max(mag)),
        mean_signed_nm=float(np.mean(shoulder)),
        frac_nonzero=float(np.mean(mag > 1e-9)),
    )
    return record


def main() -> None:
    results = []
    for w1 in W1_VALUES:
        for r in range(N_REPEATS):
            rec = active_motion_run(w1, r)
            print(f"w1={w1} run {r}: rms={rec.get('rms_nm', float('nan')):.5f} "
                  f"p99={rec.get('p99_nm', float('nan')):.5f} peak={rec.get('peak_nm', float('nan')):.5f} "
                  f"frac_nonzero={rec.get('frac_nonzero', float('nan')):.3f} "
                  f"osc={rec.get('oscillation_abort')} {rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
