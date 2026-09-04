#!/usr/bin/env python3
"""Phase 2 (2026-09-04 fine-tuning task) -- synchronized position/force logger.

Subscribes to all 8 signals Phase 2 asks for (master + slave, symmetric),
synchronizes them to a COMMON timestamp AT CAPTURE TIME using
message_filters.ApproximateTimeSynchronizer -- not collected on separate
subscriptions and matched up after the fact, per the task's explicit 2.1
requirement -- and writes one CSV row per synchronized sample.

SIGNALS (all carry a std_msgs/Header, which is what makes synchronization
possible at all):
    /touch_left/pose                  master live pose      (C2/C3 baseline)
    /touch_left/target_pose            master's commanded target (C3 input)
    /touch_left/joint_angles           master q (waist/shoulder/elbow), for
                                        converting force_ext_hat's joint
                                        torque into a Cartesian force offline
    /touch_left/force_ext_hat          master's own DOB force estimate (C4 source)
    /touch_left/joint_torque_ext_cmd   what C1 delivered to the master (joint
                                        space, PRE-Cartesian-conversion --
                                        not the same thing as what the device
                                        actually renders, see applied_force)
    /touch_left/applied_force          the REAL rendered Cartesian force
                                        (Newtons) the master device actually
                                        writes -- this is the haptic force
                                        the operator actually feels
    /touch_right/pose                 slave live pose
    /touch_right/target_pose           slave's commanded target (C2 output... no,
                                        wait: C2 blends the MASTER's target from
                                        slave position -- target_pose here is
                                        each arm's OWN commanded target,
                                        regardless of which channel produced it)
    /touch_right/joint_angles          slave q, same purpose as master's
    /touch_right/force_ext_hat         slave's own DOB force estimate (C1 source)
    /touch_right/joint_torque_ext_cmd  what C4 delivered to the slave (joint space)
    /touch_right/applied_force         the REAL rendered Cartesian force at
                                        the slave

USAGE -- run on the real rig with terminals 1-4 already up (master, slave,
bridge, bilateral_4ch), *** REAL HARDWARE, RUN BY THE OPERATOR, NOT BY AN
AGENT THAT CANNOT TOUCH THE PHYSICAL DEVICES ***:

    python3 log_phase2_scenario.py --scenario single_axis_slow --axis x \\
        --repeat 1 --duration-s 15 --out-dir ~/dob_logs/phase2_captures

Prints an instruction for what to physically do (based on --scenario),
counts down, records for --duration-s seconds, then writes:
    <out-dir>/<scenario>_<axis>_rep<repeat>_<date>.csv

Run 3 times per scenario (--repeat 1/2/3) per the task's "avoid the n=1
problem" requirement -- this script does not loop that for you, since each
repeat needs the operator to actually redo the physical motion, not just
press a key.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
import message_filters

from touch_haptics.msg import ForceEstimate

SCENARIOS = {
    "single_axis_slow": (
        "SINGLE-AXIS, SLOW MOTION. Pick one axis on the MASTER and move it "
        "slowly and smoothly back and forth along that axis only, for the "
        "full duration. Purpose: verify C2/C3 baseline tracking accuracy."
    ),
    "single_axis_fast": (
        "SINGLE-AXIS, FAST/SHARP MOTION. Pick one axis on the MASTER and "
        "move it in QUICK, SHARP jabs along that axis only. Purpose: expose "
        "the DOB's bandwidth limit -- elbow's ~100ms lag should be visible."
    ),
    "push_hold_release": (
        "STEADY PUSH-HOLD-RELEASE. Hold the MASTER still, then push firmly "
        "against the SLAVE (or vice versa) for a couple seconds, hold, "
        "release, pause, repeat 2-3 times within the window. Purpose: "
        "observe deadband behavior."
    ),
    "multiaxis_push_release": (
        "FAST, MULTI-AXIS PUSH/RELEASE. Push and release across several "
        "different axes in quick succession. Purpose: observe the updated "
        "oscillation latch's behavior under exactly this pattern."
    ),
}


def build_writer(csv_path: Path):
    f = open(csv_path, "w", newline="")
    writer = csv.writer(f)
    writer.writerow([
        "t_sync",
        "master_x", "master_y", "master_z",
        "master_target_x", "master_target_y", "master_target_z",
        "master_q_waist", "master_q_shoulder", "master_q_elbow",
        "master_force_ext_hat_waist", "master_force_ext_hat_shoulder", "master_force_ext_hat_elbow",
        "master_joint_torque_ext_cmd_waist", "master_joint_torque_ext_cmd_shoulder", "master_joint_torque_ext_cmd_elbow",
        "master_applied_force_x", "master_applied_force_y", "master_applied_force_z",
        "slave_x", "slave_y", "slave_z",
        "slave_target_x", "slave_target_y", "slave_target_z",
        "slave_q_waist", "slave_q_shoulder", "slave_q_elbow",
        "slave_force_ext_hat_waist", "slave_force_ext_hat_shoulder", "slave_force_ext_hat_elbow",
        "slave_joint_torque_ext_cmd_waist", "slave_joint_torque_ext_cmd_shoulder", "slave_joint_torque_ext_cmd_elbow",
        "slave_applied_force_x", "slave_applied_force_y", "slave_applied_force_z",
    ])
    return f, writer


class Phase2Logger(Node):
    def __init__(self, args: argparse.Namespace, csv_writer, start_wall: float):
        super().__init__("phase2_scenario_logger")
        self.args = args
        self.csv_writer = csv_writer
        self.start_wall = start_wall
        self.n_samples = 0

        qos = QoSProfile(depth=30)
        subs = [
            message_filters.Subscriber(self, PoseStamped, "/touch_left/pose", qos_profile=qos),
            message_filters.Subscriber(self, PoseStamped, "/touch_left/target_pose", qos_profile=qos),
            message_filters.Subscriber(self, Vector3Stamped, "/touch_left/joint_angles", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_left/force_ext_hat", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_left/joint_torque_ext_cmd", qos_profile=qos),
            message_filters.Subscriber(self, WrenchStamped, "/touch_left/applied_force", qos_profile=qos),
            message_filters.Subscriber(self, PoseStamped, "/touch_right/pose", qos_profile=qos),
            message_filters.Subscriber(self, PoseStamped, "/touch_right/target_pose", qos_profile=qos),
            message_filters.Subscriber(self, Vector3Stamped, "/touch_right/joint_angles", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_right/force_ext_hat", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_right/joint_torque_ext_cmd", qos_profile=qos),
            message_filters.Subscriber(self, WrenchStamped, "/touch_right/applied_force", qos_profile=qos),
        ]
        # slop=0.02s: generous relative to the ~200Hz/100Hz publish rates in
        # play here (5-10ms native period), tight enough that no sample pairs
        # a stale value from a different servo tick as if it were simultaneous.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=30, slop=0.02)
        self.sync.registerCallback(self._on_synced)

    def _on_synced(self, m_pose, m_target, m_q, m_fe, m_ext_cmd, m_applied,
                   s_pose, s_target, s_q, s_fe, s_ext_cmd, s_applied) -> None:
        t_sync = time.time() - self.start_wall
        row = [
            f"{t_sync:.4f}",
            m_pose.pose.position.x, m_pose.pose.position.y, m_pose.pose.position.z,
            m_target.pose.position.x, m_target.pose.position.y, m_target.pose.position.z,
            m_q.vector.x, m_q.vector.y, m_q.vector.z,
            *m_fe.tau_ext_deadbanded_nm,
            *m_ext_cmd.tau_ext_deadbanded_nm,
            m_applied.wrench.force.x, m_applied.wrench.force.y, m_applied.wrench.force.z,
            s_pose.pose.position.x, s_pose.pose.position.y, s_pose.pose.position.z,
            s_target.pose.position.x, s_target.pose.position.y, s_target.pose.position.z,
            s_q.vector.x, s_q.vector.y, s_q.vector.z,
            *s_fe.tau_ext_deadbanded_nm,
            *s_ext_cmd.tau_ext_deadbanded_nm,
            s_applied.wrench.force.x, s_applied.wrench.force.y, s_applied.wrench.force.z,
        ]
        self.csv_writer.writerow(row)
        self.n_samples += 1


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", required=True, choices=sorted(SCENARIOS))
    p.add_argument("--axis", default="na",
                   help="Which axis this run targets (x/y/z/multi/na) -- only used for the output filename.")
    p.add_argument("--repeat", type=int, required=True,
                   help="Which repeat this is (1/2/3, per the >=3-repeats requirement). Not looped automatically.")
    p.add_argument("--duration-s", type=float, default=15.0)
    p.add_argument("--countdown-s", type=float, default=3.0)
    p.add_argument("--out-dir", type=Path, default=Path.home() / "dob_logs" / "phase2_captures")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    date_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.out_dir / f"{args.scenario}_{args.axis}_rep{args.repeat}_{date_str}.csv"

    print(f"[phase2_logger] scenario={args.scenario!r} axis={args.axis!r} repeat={args.repeat}")
    print(f"[phase2_logger] {SCENARIOS[args.scenario]}")
    print(f"[phase2_logger] output: {csv_path}")

    for remaining in range(int(args.countdown_s), 0, -1):
        print(f"[phase2_logger] starting in {remaining}...")
        time.sleep(1.0)

    f, writer = build_writer(csv_path)
    rclpy.init()
    start_wall = time.time()
    node = Phase2Logger(args, writer, start_wall)

    print(f"[phase2_logger] RECORDING for {args.duration_s:.1f}s -- go.")
    try:
        end_time = time.time() + args.duration_s
        while time.time() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("[phase2_logger] interrupted early")
    finally:
        f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"[phase2_logger] done. {node.n_samples} synchronized samples written to {csv_path}")
    if node.n_samples == 0:
        print("[phase2_logger] WARNING: zero samples captured -- check that all 8 topics are "
              "actually publishing (master, slave, bridge, bilateral_4ch all up?) and that "
              "message_filters' slop (0.02s) isn't too tight for the actual publish rates seen.")


if __name__ == "__main__":
    main()
