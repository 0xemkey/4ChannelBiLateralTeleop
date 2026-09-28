#!/usr/bin/env python3
"""Phase 6, restricted scope (per user instruction 2026-09-09): C1 gain
characterization using SHOULDER as the validated reference channel only.

Waist and elbow are masked OUT of the relay entirely
(--c1-disable-joints waist,elbow) so every received sample is
unambiguously attributable to shoulder -- avoids the elbow
below-threshold contamination found earlier (see
docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md's "Phase 6 caveat" section) by
construction, not by post-hoc filtering.

This does NOT imply the shoulder mask should be lifted in the deployed
default -- that stays a separate SAFETY_ONLY/LEGACY_COMMISSIONING_PRESET
decision, untouched here. This experiment characterizes what C1 gain
WOULD do on a clean channel, to inform gain selection independent of the
mask question.

Two conditions per w1 in {0.3, 0.5, 0.7, 1.0}, N=5 each:
  - contact: wall-contact ground truth (ties to shoulder component only)
  - free_motion: false-force transmitted to master in ordinary settling

100% simulation, same hardware-independent infrastructure as every prior
script in this directory.
"""
from __future__ import annotations

import glob
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns, read_truth_csv  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402

BILATERAL_SCRIPT = Path(
    "/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bilateral_4ch.py")

W1_VALUES = [0.3, 0.5, 0.7, 1.0]
N_REPEATS = 5
TARGET_Y = -0.18
DRIVE_S = 4.0
TAIL_S = 1.0
SETTLE_S = 3.0
WINDOW_S = 3.0
SHOULDER_IDX = 1
_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731

OUT_JSON = Path(__file__).parent / "phase6_c1_shoulder_gain_raw.json"


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


def contact_run(w1: float, run_idx: int) -> dict:
    w1_tag = str(w1).replace(".", "p")
    ns_prefix = unique_ns(f"p6sc_w{w1_tag}")
    truth_prefix = f"/tmp/{ns_prefix}_truth"
    record = {"w1": w1, "run_idx": run_idx, "condition": "contact"}
    master, slave, bl_log_text = None, None, ""
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false",
                                  "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                                  "max_force_n": "1.5"},
                         sim_env={"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_TRUTH_CSV": truth_prefix})
        master.wait_started()
        slave.wait_started()

        with _launch_bilateral(master, slave, w1, unique_ns("p6sc_status")) as bilateral:
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
            with LiveNode("p6sc_verify") as live:
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
            bl_log_text = bilateral.log_text()
    finally:
        if master:
            master.stop()
        if slave:
            slave.stop()

    record["oscillation_abort"] = "OSCILLATION ABORT" in bl_log_text

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
    shoulder_true = np.array([r["tau_ext_true2"] for r in truth_tail])  # index2 = 1-based col name for joint 1 (shoulder)
    mean_true = float(np.mean(shoulder_true))

    if len(left_tail) < 10:
        record["error"] = f"too few received samples {len(left_tail)}"
        return record
    recv = np.array(left_tail)
    shoulder_recv = recv[:, SHOULDER_IDX]
    mean_recv = float(np.mean(shoulder_recv))

    record.update(
        shoulder_true_nm=mean_true, shoulder_recv_nm=mean_recv,
        gain=abs(mean_recv) / abs(mean_true) if abs(mean_true) > 1e-6 else float("nan"),
        sign_agree=bool(np.sign(mean_recv) != np.sign(mean_true)) if abs(mean_true) > 1e-6 else None,
        # sign_agree=True means OPPOSING (matches F_m ~= -F_s target)
        waist_recv_diagnostic=float(np.mean(recv[:, 0])), elbow_recv_diagnostic=float(np.mean(recv[:, 2])),
    )
    return record


def freemotion_run(w1: float, run_idx: int) -> dict:
    w1_tag = str(w1).replace(".", "p")
    ns_prefix = unique_ns(f"p6sf_w{w1_tag}")
    record = {"w1": w1, "run_idx": run_idx, "condition": "free_motion"}
    master, slave = None, None
    try:
        master = SimNode(unique_ns(f"{ns_prefix}_m"), device="Left",
                          params={"dob_enable": "true", "require_deadman": "false"})
        slave = SimNode(unique_ns(f"{ns_prefix}_s"), device="Default Device",
                         params={"dob_enable": "true", "require_deadman": "false",
                                  "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                                  "max_force_n": "1.5"})
        master.wait_started()
        slave.wait_started()

        with _launch_bilateral(master, slave, w1, unique_ns("p6sf_status")) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)
            with LiveNode("p6sf_verify") as live:
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
    shoulder_mag = np.abs(recv[:, SHOULDER_IDX])
    record.update(
        shoulder_p99_nm=float(np.percentile(shoulder_mag, 99)),
        shoulder_rms_nm=float(np.sqrt(np.mean(shoulder_mag ** 2))),
        waist_p99_diagnostic=float(np.percentile(np.abs(recv[:, 0]), 99)),
        elbow_p99_diagnostic=float(np.percentile(np.abs(recv[:, 2]), 99)),
    )
    return record


def main() -> None:
    results = []
    for w1 in W1_VALUES:
        for r in range(N_REPEATS):
            rec = contact_run(w1, r)
            print(f"[contact] w1={w1} run {r}: gain={rec.get('gain', float('nan')):.4f} "
                  f"sign_opposing={rec.get('sign_agree')} {rec.get('error', '')}")
            results.append(rec)
        for r in range(N_REPEATS):
            rec = freemotion_run(w1, r)
            print(f"[free]    w1={w1} run {r}: shoulder_p99={rec.get('shoulder_p99_nm', float('nan')):.5f} "
                  f"{rec.get('error', '')}")
            results.append(rec)
    with open(OUT_JSON, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nwrote {OUT_JSON}")


if __name__ == "__main__":
    main()
