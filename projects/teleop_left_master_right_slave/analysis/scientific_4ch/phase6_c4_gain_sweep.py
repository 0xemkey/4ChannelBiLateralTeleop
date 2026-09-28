#!/usr/bin/env python3
"""Phase 6, Next Step B: full C4 gain sweep.

Ground truth: hd_sim's hand model on the MASTER (verified independent of
the elbow C1 issue -- separate physical scenario, separate code path,
hd_sim_api.cpp:369-370). C4 has NO per-joint mask in bilateral_4ch.py
(confirmed by source read), so this is necessarily a WHOLE-VECTOR
characterization, unlike C1's shoulder-isolated treatment -- reported as
such, not silently compared apples-to-apples with C1's per-joint numbers.

Candidate set: w4 in {0.3, 0.5, 0.7, 1.0} -- 0.3 and 1.0 are EXISTING
preset values (bilateral_4ch.py's PRESETS: "4ch_safe" uses w4=0.3, "4ch"
uses w4=1.0); 0.5 and 0.7 mirror C1's own grid for direct
cross-channel comparability, not invented independently.

Three conditions, N=5 each:
  - contact: master hand-model ground truth (TOUCH_SIM_HAND=stiff,
    TOUCH_SIM_HAND_MOTION=step), slave's received relay vs ground truth.
  - free_motion: master at rest, no hand, no motion -- false force
    transmitted to slave in ordinary settling.
  - active_motion: master driven via its own target_pose sinusoid (no
    hand model), same amplitude/frequency validated for C1's active-
    motion test (0.08m, 3.0Hz) -- false force during motion without
    true contact.

100% simulation, hardware-independent throughout.
"""
from __future__ import annotations

import glob
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns, read_truth_csv  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BILATERAL_SCRIPT = Path(
    "/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py")

W4_VALUES = [0.3, 0.5, 0.7, 1.0]
N_REPEATS = 5
DRIVE_S = 4.0
TAIL_S = 1.0
SETTLE_S = 3.0
WINDOW_S = 3.0
MOTION_S = 6.0
AMP_M = 0.08
FREQ_HZ = 3.0
_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731

OUT_JSON = Path(__file__).parent / "phase6_c4_gain_raw.json"


def _launch_bilateral(master, slave, w4: float, status_ns: str):
    return ScriptProcess(
        [str(BILATERAL_SCRIPT),
         "--architecture", "custom", "--w1", "0.0", "--w2", "0.0",
         "--w3", "0.0", "--w4", str(w4),
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


def contact_run(w4: float, run_idx: int) -> dict:
    w4_tag = str(w4).replace(".", "p")
    ns_prefix = unique_ns(f"p6c4c_w{w4_tag}")
    truth_prefix = f"/tmp/{ns_prefix}_truth"
    record = {"w4": w4, "run_idx": run_idx, "condition": "contact"}
    master, slave, bl_log = None, None, ""
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"},
                          sim_env={"TOUCH_SIM_HAND": "stiff", "TOUCH_SIM_HAND_MOTION": "step",
                                    "TOUCH_SIM_TRUTH_CSV": truth_prefix})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false"})
        master.wait_started()
        slave.wait_started()
        with _launch_bilateral(master, slave, w4, unique_ns("p6c4c_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            with LiveNode("p6c4c_verify") as live:
                live.watch("right_ext", slave.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                tail = []
                deadline = time.time() + DRIVE_S
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    if (time.time() - deadline) > -TAIL_S:
                        v = live.latest.get("right_ext")
                        if v is not None:
                            tail.append(v)
            bl_log = bilateral.log_text()
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    record["oscillation_abort"] = "OSCILLATION ABORT" in bl_log
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
    mean_true_vec = tau_true.mean(axis=0)
    mean_true_mag = float(np.linalg.norm(mean_true_vec))

    if len(tail) < 10:
        record["error"] = f"too few received samples {len(tail)}"
        return record
    recv = np.array(tail)
    mean_recv_vec = recv.mean(axis=0)
    mean_recv_mag = float(np.linalg.norm(mean_recv_vec))
    cos_sim = float(np.dot(mean_recv_vec, mean_true_vec) / (mean_recv_mag * mean_true_mag + 1e-12))

    record.update(
        true_vec=mean_true_vec.tolist(), recv_vec=mean_recv_vec.tolist(),
        gain=mean_recv_mag / mean_true_mag if mean_true_mag > 1e-6 else float("nan"),
        cos_sim=cos_sim,
    )
    return record


def freemotion_run(w4: float, run_idx: int) -> dict:
    w4_tag = str(w4).replace(".", "p")
    ns_prefix = unique_ns(f"p6c4f_w{w4_tag}")
    record = {"w4": w4, "run_idx": run_idx, "condition": "free_motion"}
    master, slave = None, None
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false"})
        master.wait_started()
        slave.wait_started()
        with _launch_bilateral(master, slave, w4, unique_ns("p6c4f_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            with LiveNode("p6c4f_verify") as live:
                tail = []
                live.watch("right_ext", slave.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                live.spin_for(SETTLE_S)
                settle_end = time.time() + WINDOW_S
                while time.time() < settle_end:
                    live.spin_once(timeout_sec=0.01)
                    v = live.latest.get("right_ext")
                    if v is not None:
                        tail.append(v)
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    if len(tail) < 10:
        record["error"] = f"too few samples {len(tail)}"
        return record
    recv = np.array(tail)
    mag = np.linalg.norm(recv, axis=1)
    record.update(rms_nm=float(np.sqrt(np.mean(mag ** 2))), p99_nm=float(np.percentile(mag, 99)),
                   peak_nm=float(np.max(mag)))
    return record


def active_motion_run(w4: float, run_idx: int) -> dict:
    w4_tag = str(w4).replace(".", "p")
    ns_prefix = unique_ns(f"p6c4a_w{w4_tag}")
    record = {"w4": w4, "run_idx": run_idx, "condition": "active_motion"}
    master, slave, bl_log = None, None, ""
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false"})
        master.wait_started()
        slave.wait_started()
        with _launch_bilateral(master, slave, w4, unique_ns("p6c4a_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            import rclpy
            from rclpy.node import Node as RclpyNode
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Bool
            if not rclpy.ok():
                rclpy.init()
            driver = RclpyNode(f"driver_{ns_prefix}")
            target_pub = driver.create_publisher(PoseStamped, master.topic("target_pose"), 10)
            enable_pub = driver.create_publisher(Bool, master.topic("enable"), 10)
            with LiveNode("p6c4a_verify") as live:
                live.watch("right_ext", slave.topic("joint_torque_ext_cmd"), ForceEstimate,
                            extract=_fe_deadband)
                t0 = time.time()
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
                    v = live.latest.get("right_ext")
                    if v is not None:
                        tail.append(v)
            driver.destroy_node()
            bl_log = bilateral.log_text()
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    record["oscillation_abort"] = "OSCILLATION ABORT" in bl_log
    if len(tail) < 10:
        record["error"] = f"too few samples {len(tail)}"
        return record
    recv = np.array(tail)
    mag = np.linalg.norm(recv, axis=1)
    record.update(rms_nm=float(np.sqrt(np.mean(mag ** 2))), p99_nm=float(np.percentile(mag, 99)),
                   peak_nm=float(np.max(mag)), frac_nonzero=float(np.mean(mag > 1e-9)))
    return record


def main() -> None:
    results = []
    for w4 in W4_VALUES:
        for r in range(N_REPEATS):
            rec = contact_run(w4, r)
            print(f"[contact] w4={w4} run {r}: gain={rec.get('gain', float('nan')):.4f} "
                  f"cos_sim={rec.get('cos_sim', float('nan')):.4f} {rec.get('error', '')}")
            results.append(rec)
        for r in range(N_REPEATS):
            rec = freemotion_run(w4, r)
            print(f"[free]    w4={w4} run {r}: rms={rec.get('rms_nm', float('nan')):.6f} {rec.get('error', '')}")
            results.append(rec)
        for r in range(N_REPEATS):
            rec = active_motion_run(w4, r)
            print(f"[active]  w4={w4} run {r}: rms={rec.get('rms_nm', float('nan')):.6f} "
                  f"frac_nonzero={rec.get('frac_nonzero', float('nan')):.3f} {rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
