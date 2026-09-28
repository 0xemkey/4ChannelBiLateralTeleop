#!/usr/bin/env python3
"""H3/H4 real-hardware manual-motion recorder (2026-09-10).

Instrument, not a controller. This script SUBSCRIBES to existing telemetry
topics and writes files. It never creates a command publisher of any kind
(no /enable, target_pose, force_cmd, torque, arm_teleop, calibration,
controller-parameter, channel-weight, or deadman-state publish -- see
RecorderNode.__init__ and the static assertion in
hardware_validation/tests/test_manual_motion_recorder.py, which fails the
build if this file is ever edited to call create_publisher()). The operator
physically moves the MASTER by hand; this script only observes and records
what happened, then hands off to analyze_manual_motion.py for metrics.

Designed for H3 (C3-only manual relative +Y motion) and H4 (return/
repeatability), but records generically -- any manual excursion on any
mapped axis works the same way; H3/H4 are simply the recommended axis/
direction, not something this script enforces.

Usage (simulation, safe to run now):
    ROS_DOMAIN_ID=42 python3 record_manual_motion.py --out-dir <dir> \
        --duration-s 20 --operator-mode single_operator \
        --left-telemetry-topic /some_sim_ns/cartesian_controller_telemetry \
        --right-telemetry-topic /some_other_sim_ns/cartesian_controller_telemetry

Usage (real hardware, once physically supervised -- NOT run by this task):
    python3 record_manual_motion.py --out-dir <session_dir>/manual_motion/trial1 \
        --duration-s 30 --operator-mode single_operator \
        --expected-c3 true --notes "H3 +5mm Y, operator moved master by hand"

Refuses to start (exit 4) if it detects it is attached to the live hardware
domain -- see _refuse_if_live_hardware_attached() -- since this task
authorizes NO real hardware actuation and no accidental live-domain
attachment should even collect data unexpectedly.
"""
from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node

sys.path.insert(0, str(Path(__file__).parent))
from frozen_config import FROZEN, config_fingerprint  # noqa: E402

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))
from bridge_master_slave import parse_axis_map, apply_axis_map  # noqa: E402

from touch_haptics.msg import (  # noqa: E402
    CartesianControllerTelemetry, BridgeDiagnostics, BilateralStatus, DobDiagnostics,
)

SCHEMA_VERSION = "1.0.0"
RECORDER_VERSION = "2026-09-10-h3h4-1"

# 2026-09-10 (section 18): the exact node names this project's real-hardware
# launch scripts use (run_master_left.sh: __node:=teleop_master_left;
# run_slave_right.sh: __node:=teleop_slave_right) -- if either is present in
# the live ROS graph, this recorder must refuse to start, unconditionally.
LIVE_HARDWARE_NODE_NAMES = ("teleop_master_left", "teleop_slave_right")

DEFAULT_LEFT_TELEM_TOPIC = "/touch_left/cartesian_controller_telemetry"
DEFAULT_RIGHT_TELEM_TOPIC = "/touch_right/cartesian_controller_telemetry"
DEFAULT_BRIDGE_DIAG_TOPIC = "/bridge_diagnostics"
DEFAULT_BILATERAL_STATUS_TOPIC = "/bilateral_status"

TELEM_FLOAT3_FIELDS = [
    "position_m", "velocity_mps", "target_position_m",
    "position_error_raw_m", "position_error_clamped_m",
    "spring_force_n", "damping_force_n", "auxiliary_force_n",
    "force_before_primary_clamp_n", "force_after_primary_clamp_n",
    "relay_torque_received_nm", "relay_force_n",
    "force_before_final_clamp_n", "applied_force_n",
]
TELEM_BOOL_FIELDS = [
    "enabled", "deadman_ok", "target_valid", "target_fresh", "force_valid",
    "force_fresh", "relay_valid", "relay_fresh", "relay_applied",
    "tracking_valid", "position_error_clamped", "primary_force_clamped",
    "final_force_clamped", "joint_path_commanded", "cartesian_write_active",
    "near_singular",
]

BRIDGE_SCALAR_FIELDS = [
    "bridge_state", "ramp_alpha", "gravity_ramp_alpha", "coupling_ramp_alpha",
    "c1_enabled", "c2_enabled", "c3_enabled", "c4_enabled",
    "master_travel_limit_flag", "slave_travel_limit_flag",
    "coupling_ramp_held", "force_budget_telemetry_fresh_left",
    "force_budget_telemetry_fresh_right", "planner_parameter_match_left",
    "planner_parameter_match_right", "follow_error_m",
    "master_accepted_force_budget_margin_n", "slave_accepted_force_budget_margin_n",
    "master_accepted_position_error_margin_m", "slave_accepted_position_error_margin_m",
    "master_candidate_force_budget_margin_n", "slave_candidate_force_budget_margin_n",
    "master_candidate_position_error_margin_m", "slave_candidate_position_error_margin_m",
    "coupling_ramp_stall_time_s", "alignment_error_norm_m",
    "gravity_settle_master_speed_mps", "gravity_settle_slave_speed_mps",
    "gravity_settle_master_pose_drift_m", "gravity_settle_slave_pose_drift_m",
    "gravity_settle_quiet_window_accumulated_s",
    "gravity_settle_accepted_master_pose_drift_m",
    "gravity_settle_accepted_slave_pose_drift_m",
]
BRIDGE_FLOAT3_FIELDS = [
    "master_relative_m", "slave_relative_m",
    "mapped_master_relative_m", "mapped_slave_relative_m",
    "c2_position_error_m", "c3_position_error_m",
    "alignment_error_xyz_m",
    "master_predicted_force_n", "slave_predicted_force_n",
]

BILATERAL_SCALAR_FIELDS = ["architecture", "safety_state", "clutch_active", "grip_detected"]
BILATERAL_FLOAT3_FIELDS = [
    "tau_e_hat_nm", "tau_h_hat_nm",
    "master_torque_cmd_nm", "slave_torque_cmd_nm",
    "effective_threshold_c1_nm", "effective_threshold_c4_nm",
    "c1_joint_mask",
]
BILATERAL_FLOAT_FIELDS = [
    "tau_e_age_s", "tau_h_age_s", "master_budget_used", "slave_budget_used",
    "passivity_energy_j", "damping_injected_nms_per_rad", "duty_cycle_nm",
    "c1_zero_crossing_hz",
]


def _refuse_if_live_hardware_attached(node: Node) -> Optional[str]:
    """Read-only ROS-graph check (section 17). Returns an error string if
    the recorder should refuse to start, else None. Does not subscribe or
    publish anything itself -- pure inspection of node_names."""
    deadline = time.time() + 1.5
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node_names = [n for n, _ in node.get_node_names_and_namespaces()]
    for bad in LIVE_HARDWARE_NODE_NAMES:
        if any(bad in n for n in node_names):
            return (f"live hardware node '{bad}' is present in the ROS graph "
                    f"(node_names={node_names}) -- refusing to attach. This "
                    "recorder is for simulation validation or a supervised "
                    "real-hardware session where the operator has already "
                    "confirmed physical presence per ONSITE_RUNBOOK.md; it "
                    "does not itself make that determination, so an "
                    "unexpected live-hardware graph is always treated as "
                    "unauthorized attachment.")
    return None


def _none3():
    return [None, None, None]


class RecorderNode(Node):
    """SUBSCRIBE-ONLY. See module docstring. This class must never call
    self.create_publisher() -- enforced by
    tests/test_manual_motion_recorder.py's static source-grep assertion."""

    def __init__(self, args):
        super().__init__("hardware_validation_manual_motion_recorder")
        self.args = args
        self.axis_map = parse_axis_map(FROZEN.axis_map)
        self.axis_scale = [float(x) for x in FROZEN.axis_scale.split(",")]

        self.master_anchor = None   # native XYZ, latched once
        self.slave_anchor = None

        self.rows_master = []
        self.rows_slave = []
        self.rows_bridge = []
        self.rows_bilateral = []

        self._last_master_recv = None
        self._last_slave_recv = None
        self._last_bridge_recv = None
        self._last_bilateral_recv = None

        self.create_subscription(CartesianControllerTelemetry, args.left_telemetry_topic,
                                 self._on_master, 100)
        self.create_subscription(CartesianControllerTelemetry, args.right_telemetry_topic,
                                 self._on_slave, 100)
        self.create_subscription(BridgeDiagnostics, args.bridge_diagnostics_topic,
                                 self._on_bridge, 50)
        self.create_subscription(BilateralStatus, args.bilateral_status_topic,
                                 self._on_bilateral, 50)
        # 2026-09-10 (C3 real-hardware baseline task): joint-space DOB
        # disturbance torque, both arms -- tau_dis_hat_nm is JOINT order
        # [waist, shoulder, elbow], NOT Cartesian XYZ (see DobDiagnostics.msg
        # and part1_gravity_settle_capture.py's own correction of an
        # earlier mislabeling in this project). Latched by side, merged
        # into each telemetry row at the row's own receive time (same
        # pattern as part1_gravity_settle_capture.py's _on_dob).
        self._dob_latest = {"left": None, "right": None}
        self.create_subscription(DobDiagnostics, args.left_dob_topic,
                                 lambda m: self._on_dob("left", m), 20)
        self.create_subscription(DobDiagnostics, args.right_dob_topic,
                                 lambda m: self._on_dob("right", m), 20)

    def _on_dob(self, side, m):
        j = list(m.tau_dis_hat_nm) if len(m.tau_dis_hat_nm) >= 3 else [None, None, None]
        self._dob_latest[side] = {"dob_waist_nm": j[0], "dob_shoulder_nm": j[1], "dob_elbow_nm": j[2],
                                  "dob_configured": bool(m.configured)}

    @staticmethod
    def _stamp(msg) -> float:
        h = msg.header.stamp
        return float(h.sec) + float(h.nanosec) * 1e-9

    def _mapped_relative(self, xyz, anchor, inverse: bool):
        """Anchor-relative XYZ run through the configured axis_map+scale,
        applied EXACTLY ONCE (section 7). `inverse=True` for the slave-side
        vector expressed back in master-mapped terms is NOT used here --
        both master and slave are reported in their OWN native anchor-
        relative frame plus the FORWARD map (master->slave direction,
        matching how C3 itself computes its target), since H3/H4's whole
        point is comparing "master's mapped motion" against "slave's own
        native motion" directly."""
        if anchor is None:
            return _none3()
        rel = [xyz[i] - anchor[i] for i in range(3)]
        return list(apply_axis_map(self.axis_map, rel))

    def _on_master(self, m):
        now = time.monotonic()
        if self.master_anchor is None and self.args.auto_anchor:
            self.master_anchor = list(m.position_m)
            self.get_logger().info(f"[recorder] master anchor latched: {self.master_anchor}")
        row = {
            "t_recv_monotonic": now,
            "t_ros": self._stamp(m),
            "dropout_gap_s": (now - self._last_master_recv) if self._last_master_recv else 0.0,
        }
        self._last_master_recv = now
        for f in TELEM_FLOAT3_FIELDS:
            v = list(getattr(m, f))
            row[f"{f}_x"], row[f"{f}_y"], row[f"{f}_z"] = v
        for f in TELEM_BOOL_FIELDS:
            row[f] = bool(getattr(m, f))
        mapped = self._mapped_relative(list(m.position_m), self.master_anchor, inverse=False)
        row["mapped_rel_x"], row["mapped_rel_y"], row["mapped_rel_z"] = mapped
        row.update(self._dob_latest["left"] or {"dob_waist_nm": None, "dob_shoulder_nm": None,
                                                 "dob_elbow_nm": None, "dob_configured": None})
        self.rows_master.append(row)

    def _on_slave(self, m):
        now = time.monotonic()
        if self.slave_anchor is None and self.args.auto_anchor:
            self.slave_anchor = list(m.position_m)
            self.get_logger().info(f"[recorder] slave anchor latched: {self.slave_anchor}")
        row = {
            "t_recv_monotonic": now,
            "t_ros": self._stamp(m),
            "dropout_gap_s": (now - self._last_slave_recv) if self._last_slave_recv else 0.0,
        }
        self._last_slave_recv = now
        for f in TELEM_FLOAT3_FIELDS:
            v = list(getattr(m, f))
            row[f"{f}_x"], row[f"{f}_y"], row[f"{f}_z"] = v
        for f in TELEM_BOOL_FIELDS:
            row[f] = bool(getattr(m, f))
        # slave reported in its OWN native anchor-relative frame, no map
        # applied (the slave's native axes ARE the frame C3 targets it in).
        rel = ([m.position_m[i] - self.slave_anchor[i] for i in range(3)]
               if self.slave_anchor is not None else _none3())
        row["rel_x"], row["rel_y"], row["rel_z"] = rel
        row.update(self._dob_latest["right"] or {"dob_waist_nm": None, "dob_shoulder_nm": None,
                                                  "dob_elbow_nm": None, "dob_configured": None})
        self.rows_slave.append(row)

    def _on_bridge(self, m):
        now = time.monotonic()
        row = {
            "t_recv_monotonic": now,
            "t_ros": self._stamp(m),
            "dropout_gap_s": (now - self._last_bridge_recv) if self._last_bridge_recv else 0.0,
        }
        self._last_bridge_recv = now
        for f in BRIDGE_SCALAR_FIELDS:
            row[f] = getattr(m, f)
        for f in BRIDGE_FLOAT3_FIELDS:
            v = list(getattr(m, f))
            row[f"{f}_x"], row[f"{f}_y"], row[f"{f}_z"] = v
        self.rows_bridge.append(row)

    def _on_bilateral(self, m):
        now = time.monotonic()
        row = {
            "t_recv_monotonic": now,
            "t_ros": self._stamp(m),
            "dropout_gap_s": (now - self._last_bilateral_recv) if self._last_bilateral_recv else 0.0,
        }
        self._last_bilateral_recv = now
        for f in BILATERAL_SCALAR_FIELDS:
            row[f] = getattr(m, f)
        for f in BILATERAL_FLOAT_FIELDS:
            row[f] = getattr(m, f)
        for f in BILATERAL_FLOAT3_FIELDS:
            v = list(getattr(m, f))
            row[f"{f}_waist"], row[f"{f}_shoulder"], row[f"{f}_elbow"] = v
        for i in range(4):
            row[f"channel_weight_{i}"] = m.channel_weight[i]
            row[f"channel_active_{i}"] = bool(m.channel_active[i])
            row[f"channel_reason_{i}"] = int(m.channel_reason[i])
        self.rows_bilateral.append(row)

    def record(self, duration_s: float, stop_file: Optional[Path] = None):
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if stop_file is not None and stop_file.exists():
                self.get_logger().info(f"[recorder] stop file observed: {stop_file}")
                break
            rclpy.spin_once(self, timeout_sec=0.02)

    def stream_stats(self):
        """Section 6: per-stream sample count / effective Hz / max age /
        dropout count. 'No measurement' is represented as an explicit
        n=0 entry, never silently folded into a zero-valued row."""
        def _stats(rows, expected_hz=None):
            if not rows:
                return {"n": 0, "effective_hz": None, "max_gap_s": None,
                       "dropout_count": None, "available": False}
            span = rows[-1]["t_recv_monotonic"] - rows[0]["t_recv_monotonic"]
            eff_hz = (len(rows) - 1) / span if span > 0 else None
            gaps = [r["dropout_gap_s"] for r in rows[1:]]
            max_gap = max(gaps) if gaps else 0.0
            dropout_thresh = (2.0 / eff_hz) if eff_hz else 0.5
            dropout_count = sum(1 for g in gaps if g > dropout_thresh)
            return {"n": len(rows), "effective_hz": eff_hz, "max_gap_s": max_gap,
                   "dropout_count": dropout_count, "available": True}
        return {
            "master": _stats(self.rows_master),
            "slave": _stats(self.rows_slave),
            "bridge": _stats(self.rows_bridge),
            "bilateral": _stats(self.rows_bilateral),
        }


def _write_csv(path: Path, rows: list):
    if not rows:
        path.write_text("")
        return
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def _git(*args, repo_root: Path):
    try:
        return subprocess.run(["git"] + list(args), capture_output=True, text=True,
                              cwd=repo_root, timeout=10).stdout.strip()
    except Exception as exc:
        return f"<git call failed: {exc}>"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--duration-s", type=float, default=30.0)
    ap.add_argument("--stop-file", default="",
                    help="optional file path; recording exits cleanly as soon as this file exists")
    ap.add_argument("--left-telemetry-topic", default=DEFAULT_LEFT_TELEM_TOPIC)
    ap.add_argument("--right-telemetry-topic", default=DEFAULT_RIGHT_TELEM_TOPIC)
    ap.add_argument("--bridge-diagnostics-topic", default=DEFAULT_BRIDGE_DIAG_TOPIC)
    ap.add_argument("--bilateral-status-topic", default=DEFAULT_BILATERAL_STATUS_TOPIC)
    ap.add_argument("--left-dob-topic", default="/touch_left/dob_diagnostics")
    ap.add_argument("--right-dob-topic", default="/touch_right/dob_diagnostics")
    ap.add_argument("--auto-anchor", action="store_true", default=True,
                    help="latch the anchor from the first sample of each stream "
                         "(default True) -- see section 7")
    ap.add_argument("--no-auto-anchor", dest="auto_anchor", action="store_false")
    ap.add_argument("--operator-mode", default="single_operator",
                    choices=["single_operator", "assisted_deadman", "unspecified"],
                    help="section 14: metadata only -- does NOT change controller "
                         "deadman policy, purely documents what happened")
    ap.add_argument("--expected-stage", default="H3")
    ap.add_argument("--expected-c1", default="false")
    ap.add_argument("--expected-c2", default="false")
    ap.add_argument("--expected-c3", default="true")
    ap.add_argument("--expected-c4", default="false")
    ap.add_argument("--notes", default="")
    ap.add_argument("--allow-live-domain-CHECK-DISABLED", dest="allow_live",
                    action="store_true",
                    help="2026-09-10 update: real-hardware recording is now explicitly "
                         "authorized (operator physically present, C3 already proven "
                         "ACTIVE on real hardware) -- pass this to record against the "
                         "live production bridge. Still subscribe-only; still creates "
                         "zero publishers (see the static/runtime tests) -- this flag "
                         "only bypasses the domain-attachment REFUSAL, it does not "
                         "change what the recorder does. Do not pass it for any sim/"
                         "isolated-domain run, where the refusal should still apply.")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = RecorderNode(args)
    try:
        if not args.allow_live:
            reason = _refuse_if_live_hardware_attached(node)
            if reason is not None:
                print(f"[recorder][REFUSED] {reason}", flush=True)
                (out_dir / "REFUSED.json").write_text(json.dumps({"reason": reason}, indent=2))
                sys.exit(4)

        stop_file = Path(args.stop_file) if args.stop_file else None
        if stop_file:
            print(f"[recorder] recording for up to {args.duration_s}s; stop-file={stop_file}", flush=True)
        else:
            print(f"[recorder] recording for {args.duration_s}s -- move the master by hand now.", flush=True)
        record_started = time.monotonic()
        node.record(args.duration_s, stop_file=stop_file)
        actual_elapsed_s = time.monotonic() - record_started
        stats = node.stream_stats()
        print(f"[recorder] stream stats: {json.dumps(stats, indent=2)}", flush=True)

        _write_csv(out_dir / "raw_master.csv", node.rows_master)
        _write_csv(out_dir / "raw_slave.csv", node.rows_slave)
        _write_csv(out_dir / "raw_bridge.csv", node.rows_bridge)
        _write_csv(out_dir / "raw_bilateral.csv", node.rows_bilateral)

        repo_root = Path(__file__).resolve().parents[3]
        manifest = {
            "schema_version": SCHEMA_VERSION,
            "recorder_version": RECORDER_VERSION,
            "session_out_dir": str(out_dir),
            "utc_time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "local_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "git_head": _git("rev-parse", "HEAD", repo_root=repo_root),
            "git_dirty": bool(_git("status", "--porcelain", repo_root=repo_root)),
            "git_branch": _git("branch", "--show-current", repo_root=repo_root),
            "config_fingerprint": config_fingerprint(),
            "ros_domain_id": __import__("os").environ.get("ROS_DOMAIN_ID"),
            "topics": {
                "left_telemetry": args.left_telemetry_topic,
                "right_telemetry": args.right_telemetry_topic,
                "bridge_diagnostics": args.bridge_diagnostics_topic,
                "bilateral_status": args.bilateral_status_topic,
            },
            "expected_stage": args.expected_stage,
            "expected_channel_configuration": {
                "c1": args.expected_c1.lower() == "true",
                "c2": args.expected_c2.lower() == "true",
                "c3": args.expected_c3.lower() == "true",
                "c4": args.expected_c4.lower() == "true",
            },
            "operator_mode": args.operator_mode,
            "notes": args.notes,
            "master_anchor_native_m": node.master_anchor,
            "slave_anchor_native_m": node.slave_anchor,
            "axis_map": FROZEN.axis_map,
            "axis_scale": FROZEN.axis_scale,
            "stream_stats": stats,
            "duration_s": args.duration_s,
            "actual_elapsed_s": actual_elapsed_s,
            "stop_file": str(stop_file) if stop_file is not None else None,
            "stop_file_observed": bool(stop_file is not None and stop_file.exists()),
        }
        (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))
        print(f"[recorder] wrote manifest.json + 4 raw CSVs to {out_dir}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
