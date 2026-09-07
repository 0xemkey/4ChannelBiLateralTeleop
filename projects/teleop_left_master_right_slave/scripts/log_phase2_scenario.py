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
        --repeat 1 --out-dir ~/dob_logs/phase2_captures

Prints an instruction for what to physically do (based on --scenario),
counts down, then records as a SESSION -- keeps going for as long as you
move the devices, until you press Ctrl-C to stop. On stop it writes:
    <out-dir>/<scenario>_<axis>_rep<repeat>_<date>.csv
and IMMEDIATELY plots it (calls plot_phase2_scenario.py's own process_one()
directly, same PNG set described in that script's docstring) -- no separate
plot command needed. Pass --no-plot to skip that and plot manually later,
or --duration-s N for the old fixed-duration behavior instead of Ctrl-C.

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

sys.path.insert(0, str(Path(__file__).parent))
import plot_phase2_scenario  # noqa: E402 -- process_one() called at the end of a session

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
import message_filters

from touch_haptics.msg import BilateralStatus, BridgeDiagnostics, CartesianControllerTelemetry, ForceEstimate

# Fallback for joint_torque_ext_cmd when no bilateral_4ch.py is running (a
# C3-only "2ch" capture) -- tau_ext_deadbanded_nm defaults to all-zero
# already at construction, matching "nothing is being relayed."
_ZERO_FORCE_ESTIMATE = ForceEstimate()

# Fallback for /bridge_diagnostics when the bridge has never published one.
# bridge_state defaults to the empty string (not e.g. "ACTIVE") so this is
# never mistaken for a real, healthy reading.
_ZERO_BRIDGE_DIAG = BridgeDiagnostics()

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
    "c2_bilateral_position": (
        "C2+C3 BILATERAL POSITION-ONLY (C1/C4 must stay off). Structured "
        "~40s run: (A) 0-5s both stationary -- baseline/no-kick check; "
        "(B) 5-15s move MASTER gently, slave should follow (C3-dominant); "
        "(C) 15-20s both stationary again; (D) 20-30s move SLAVE gently, "
        "master should follow (C2-dominant); (E) 30-40s gentle mixed "
        "bilateral motion, both directions live. Keep amplitudes small -- "
        "avoid deliberately reaching travel limits. No force/contact."
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
        "c1_active", "c4_active",
        # Added 2026-09-08 for the C2+C3 position-only diagnostic. Sourced
        # from /bridge_diagnostics (see msg/BridgeDiagnostics.msg) and
        # /touch_{left,right}/cartesian_controller_telemetry. "Equivalent
        # existing field" mapping, per the task's own instruction not to
        # duplicate: master_actual_x/y/z == master_x/y/z above (already
        # captured); slave_actual_x/y/z == slave_x/y/z above.
        "master_relative_x", "master_relative_y", "master_relative_z",
        "slave_relative_x", "slave_relative_y", "slave_relative_z",
        "mapped_master_relative_x", "mapped_master_relative_y", "mapped_master_relative_z",
        "mapped_slave_relative_x", "mapped_slave_relative_y", "mapped_slave_relative_z",
        "c2_position_error_x", "c2_position_error_y", "c2_position_error_z",
        "c3_position_error_x", "c3_position_error_y", "c3_position_error_z",
        "alignment_error_x", "alignment_error_y", "alignment_error_z", "alignment_error_norm",
        "follow_error", "bridge_state", "ramp_alpha", "c2_enabled", "c3_enabled",
        "master_travel_limit_flag", "slave_travel_limit_flag",
        "master_force_saturation_flag", "slave_force_saturation_flag",
    ])
    return f, writer


class Phase2Logger(Node):
    def __init__(self, args: argparse.Namespace, csv_writer, start_wall: float):
        super().__init__("phase2_scenario_logger")
        self.args = args
        self.csv_writer = csv_writer
        self.start_wall = start_wall
        self.n_samples = 0

        # target_pose is NOT in the synchronizer below -- REAL BUG, fixed
        # 2026-09-07: /touch_left/target_pose only exists when C2 is armed
        # (bridge_master_slave.py's --enable-c2), and /touch_right/target_pose
        # only exists once the position bridge (C3) has captured its first
        # anchor. A 2-channel (C3-only, C2 off) capture -- exactly the "2ch"
        # side of a 2ch-vs-4ch comparison -- would NEVER have master's
        # target_pose published at all. message_filters.ApproximateTime
        # Synchronizer requires EVERY registered subscriber to have received
        # at least one message before it fires ONCE, ever -- so with target_
        # pose inside it, this logger silently produced exactly 0 samples for
        # the entire duration on any run where C2 wasn't armed, with no error,
        # only the generic "zero samples" warning at the end (which reads as
        # a topic-connectivity problem, not a synchronizer-membership one).
        # Confirmed live: a full master+slave+bridge+bilateral_4ch sim stack
        # produced 0 samples until target_pose was pulled out of the sync set.
        #
        # Fixed by tracking each side's target_pose as a plain "latest known
        # value" cache instead, read out (not synchronized on) whenever the
        # OTHER 10 always-published topics sync. Falls back to that side's
        # own live `pose` if no target_pose has ever arrived -- a target
        # equal to the current position is the correct representation of
        # "nothing is currently commanding this device toward anywhere,"
        # which is exactly true when C2/C3 aren't driving that side.
        self._left_target = None
        self._right_target = None
        self.create_subscription(
            PoseStamped, "/touch_left/target_pose",
            lambda m: setattr(self, "_left_target", m), qos_profile=QoSProfile(depth=10))
        self.create_subscription(
            PoseStamped, "/touch_right/target_pose",
            lambda m: setattr(self, "_right_target", m), qos_profile=QoSProfile(depth=10))

        # SAME bug, second instance: joint_torque_ext_cmd only exists at all
        # while bilateral_4ch.py is running (C1/C4). A pure C3-only ("2ch")
        # capture -- with no bilateral_4ch process at all -- would ALSO never
        # synchronize with this in the strict subscriber set. Confirmed live:
        # even after fixing target_pose above, a C3-only run (bridge running,
        # no bilateral_4ch) still produced 0 samples until this was pulled
        # out too. Same fix, same fallback reasoning: an empty/never-received
        # relay input is correctly represented as all-zero, not-valid, which
        # is exactly what "no force channel is relaying anything right now"
        # means.
        self._left_ext_cmd = None
        self._right_ext_cmd = None
        self.create_subscription(
            ForceEstimate, "/touch_left/joint_torque_ext_cmd",
            lambda m: setattr(self, "_left_ext_cmd", m), qos_profile=QoSProfile(depth=10))
        self.create_subscription(
            ForceEstimate, "/touch_right/joint_torque_ext_cmd",
            lambda m: setattr(self, "_right_ext_cmd", m), qos_profile=QoSProfile(depth=10))

        # 2ch/4ch phase marker for a SINGLE continuous session that starts
        # 2-channel (C1=C4=0) and has bilateral_4ch.py started partway
        # through (added 2026-09-07, so one recording can span both phases
        # instead of needing two separate runs -- see plot_comparison.py's
        # --single-csv mode, which auto-splits on this exact column).
        # channel_active[0]=C1, [3]=C4 -- the direct "is this REALLY active"
        # answer (weight>0 AND fresh AND valid), not just the target weight,
        # so a ramping-up or momentarily-stale channel isn't misclassified.
        # No publisher exists at all until bilateral_4ch.py actually starts,
        # so both default to False (correctly: "not 4ch yet") until then.
        self._c1_active = False
        self._c4_active = False
        self.create_subscription(
            BilateralStatus, "/bilateral_status",
            self._bilateral_status_cb, qos_profile=QoSProfile(depth=10))

        # Added 2026-09-08 for the C2+C3 position-only diagnostic. Same
        # "latest known, safe default" pattern as above -- /bridge_diagnostics
        # only exists once bridge_master_slave.py is running, and each
        # CartesianControllerTelemetry only exists once its own touch_slave
        # node is up (publish_telemetry:=true, default true in this project's
        # launch scripts).
        self._bridge_diag: BridgeDiagnostics | None = None
        self._master_telemetry: CartesianControllerTelemetry | None = None
        self._slave_telemetry: CartesianControllerTelemetry | None = None
        self.create_subscription(
            BridgeDiagnostics, "/bridge_diagnostics",
            lambda m: setattr(self, "_bridge_diag", m), qos_profile=QoSProfile(depth=10))
        self.create_subscription(
            CartesianControllerTelemetry, "/touch_left/cartesian_controller_telemetry",
            lambda m: setattr(self, "_master_telemetry", m), qos_profile=QoSProfile(depth=10))
        self.create_subscription(
            CartesianControllerTelemetry, "/touch_right/cartesian_controller_telemetry",
            lambda m: setattr(self, "_slave_telemetry", m), qos_profile=QoSProfile(depth=10))

        qos = QoSProfile(depth=30)
        subs = [
            message_filters.Subscriber(self, PoseStamped, "/touch_left/pose", qos_profile=qos),
            message_filters.Subscriber(self, Vector3Stamped, "/touch_left/joint_angles", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_left/force_ext_hat", qos_profile=qos),
            message_filters.Subscriber(self, WrenchStamped, "/touch_left/applied_force", qos_profile=qos),
            message_filters.Subscriber(self, PoseStamped, "/touch_right/pose", qos_profile=qos),
            message_filters.Subscriber(self, Vector3Stamped, "/touch_right/joint_angles", qos_profile=qos),
            message_filters.Subscriber(self, ForceEstimate, "/touch_right/force_ext_hat", qos_profile=qos),
            message_filters.Subscriber(self, WrenchStamped, "/touch_right/applied_force", qos_profile=qos),
        ]
        # slop=0.02s: generous relative to the ~200Hz/100Hz publish rates in
        # play here (5-10ms native period), tight enough that no sample pairs
        # a stale value from a different servo tick as if it were simultaneous.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            subs, queue_size=30, slop=0.02)
        self.sync.registerCallback(self._on_synced)

    def _bilateral_status_cb(self, msg: BilateralStatus) -> None:
        self._c1_active = bool(msg.channel_active[0])
        self._c4_active = bool(msg.channel_active[3])

    def _on_synced(self, m_pose, m_q, m_fe, m_applied,
                   s_pose, s_q, s_fe, s_applied) -> None:
        t_sync = time.time() - self.start_wall
        m_target = self._left_target if self._left_target is not None else m_pose
        s_target = self._right_target if self._right_target is not None else s_pose
        m_ext_cmd = self._left_ext_cmd if self._left_ext_cmd is not None else _ZERO_FORCE_ESTIMATE
        s_ext_cmd = self._right_ext_cmd if self._right_ext_cmd is not None else _ZERO_FORCE_ESTIMATE
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
            int(self._c1_active), int(self._c4_active),
        ]

        # Added 2026-09-08. bd defaults to a zero/inert BridgeDiagnostics
        # when the bridge has never published one (e.g. a capture taken
        # before the bridge started, or run without it at all) -- same
        # "safe default, not a crash" convention as _ZERO_FORCE_ESTIMATE.
        bd = self._bridge_diag if self._bridge_diag is not None else _ZERO_BRIDGE_DIAG
        mt = self._master_telemetry
        st = self._slave_telemetry
        row += [
            *bd.master_relative_m,
            *bd.slave_relative_m,
            *bd.mapped_master_relative_m,
            *bd.mapped_slave_relative_m,
            *bd.c2_position_error_m,
            *bd.c3_position_error_m,
            *bd.alignment_error_xyz_m, bd.alignment_error_norm_m,
            bd.follow_error_m, bd.bridge_state, bd.ramp_alpha,
            int(bd.c2_enabled), int(bd.c3_enabled),
            int(bd.master_travel_limit_flag), int(bd.slave_travel_limit_flag),
            int(mt.final_force_clamped) if mt is not None else 0,
            int(st.final_force_clamped) if st is not None else 0,
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
    p.add_argument("--duration-s", type=float, default=None,
                   help="If given, record for exactly this long and stop automatically. "
                        "If omitted (the default), this is a SESSION: recording runs until "
                        "you press Ctrl-C, for as long as you want to move the devices.")
    p.add_argument("--countdown-s", type=float, default=3.0)
    p.add_argument("--out-dir", type=Path, default=Path.home() / "dob_logs" / "phase2_captures")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip the automatic plotting step at the end (plot manually later "
                        "with plot_phase2_scenario.py if you pass this).")
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

    if args.duration_s is None:
        print("[phase2_logger] RECORDING as a SESSION -- move the devices, then press "
              "Ctrl-C when you're done to stop and plot.")
        try:
            while True:
                rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            print("[phase2_logger] stopped by Ctrl-C")
        finally:
            f.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    else:
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
        return

    if args.no_plot:
        print(f"[phase2_logger] --no-plot given; plot later with: "
              f"python3 plot_phase2_scenario.py {csv_path}")
        return

    print("[phase2_logger] plotting...")
    plot_phase2_scenario.process_one(csv_path, args.out_dir)


if __name__ == "__main__":
    main()
