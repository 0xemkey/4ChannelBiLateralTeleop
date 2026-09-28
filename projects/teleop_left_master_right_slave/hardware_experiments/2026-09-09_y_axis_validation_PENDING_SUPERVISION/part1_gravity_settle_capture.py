#!/usr/bin/env python3
"""H2 / Part 1: GRAVITY_SETTLE-only capture, bilateral coupling OFF.

2026-09-10 REVISION (H2 real-hardware task): uses the source-level
gravity-settle-only/HOLD MODE (startup_alignment.py's
gravity_settle_hold_mode, wired to bridge_master_slave.py's
--gravity-settle-hold-mode) instead of racing to disarm on
COUPLING_RAMP_UP. Once armed with hold mode on, the state machine remains
in GRAVITY_SETTLE indefinitely once its quiet-hold gate passes --
gravity_ramp_alpha=1, coupling_ramp_alpha=0, structurally forever, exiting
ONLY via this script's own explicit disarm() call at the end of the
observation window. COUPLING_RAMP_UP is NEVER entered during this test --
this is enforced by startup_alignment.py itself (see
test_hold_mode_stays_in_gravity_settle_indefinitely and its two sibling
tests in scripts/tests/test_gravity_settle_gate.py), not by this script
racing a state transition.

GATE-PASS DETECTION (2026-09-10 correction): the previous version of this
script treated `gravity_ramp_alpha >= 1.0` as "the settle gate passed" --
that is WRONG: gravity_ramp_alpha reaching 1.0 only means the gravity
RAMP finished; the QUIET-HOLD gate (both arms' speed below
gravity_settle_velocity_threshold_mps, continuously, for
gravity_settle_hold_s, evaluated ONLY once inside GRAVITY_SETTLE) is a
SEPARATE, later condition. The correct, evidence-based signal is
BridgeDiagnostics.gravity_settle_quiet_window_accumulated_s crossing
FROZEN.gravity_settle_hold_s while bridge_state=="GRAVITY_SETTLE" -- see
startup_alignment.py's GRAVITY_SETTLE tick() branch: this field
accumulates (now - quiet_since_t) while both arms' speed stays under
threshold, RESETS TO 0 the instant either arm exceeds it, and the tick
that finally satisfies `accumulated_s >= hold_s AND elapsed >=
min_time_s` is the exact tick the gate passes (after which, in hold mode,
tick() returns early and this field FREEZES at its final value -- so
"the value stopped changing" is a secondary confirmation, not the primary
signal). This script now detects the crossing directly instead of
inferring gate-pass from a proxy that fires far too early. This ensures
H2 cannot false-pass merely because a single instantaneous velocity
sample happened to read zero.

Uses frozen_config.py for every parameter -- one canonical source, no
hand-duplicated numbers that can drift out of sync between scripts.

Does NOT command any motion -- the only actuation is gravity-compensation
AUXILIARY force (frozen_config.FROZEN.slave_gravity_comp_n=0.55N,
controlled-sweep-validated; left_gravity_comp_n=0.0, i.e. the master gets
NO added compensation).

DOB NOTE (2026-09-10 correction): DobDiagnostics.tau_dis_hat_nm is a
JOINT-order array [waist, shoulder, elbow] (J1,J2,J3) -- NOT a Cartesian
XYZ array. An earlier script in this project (passive_baseline_capture.py)
read tau_dis_hat_nm[1] and labeled it "tau_y", which is a JOINT-to-
CARTESIAN conflation: index 1 is the SHOULDER joint's disturbance
estimate, not a Cartesian-Y disturbance. This script records the full
3-joint array under its correct joint labels (dob_waist/dob_shoulder/
dob_elbow) and does NOT repeat the "_y" mislabeling. This matches the
still-open "joint-to-Cartesian interpretation confirmation" item in
OFFSITE_REMAINING_WORK.md -- flagged, not silently fixed elsewhere, since
fixing the OLDER script is out of this task's scope.

ABORT LOGIC:
  - EXPECTED: slave auxiliary_force_n ~= slave_gravity_comp_n (0.55N) once
    settled. This is NOT an abort trigger by itself.
  - UNEXPECTED (abort immediately): spring_force_n or damping_force_n norm
    (either arm) exceeding a small numerical tolerance -- during a true
    gravity-only hold, target==live pose exactly (hold mode's own
    structural guarantee), so spring+damping should read ~0; any
    departure means something is pulling the target away from live pose,
    which should be architecturally impossible here.
  - coupling_ramp_alpha != 0 (either arm) -- must never happen, structural
    invariant of hold mode; checking it anyway, defense in depth.
  - any of c1_enabled/c2_enabled/c3_enabled/c4_enabled True in
    BridgeDiagnostics -- H2 requires all four off; this is a configured-
    state check, independent of the CLI args actually passed.
  - relay_fresh/relay_applied True on either arm's telemetry -- would mean
    a C1/C4 contribution is live or being summed despite
    c1_enabled=c4_enabled=false. Deliberately NOT relay_valid: confirmed via
    real hardware (2026-09-10, first two H2 attempts) that touch_slave.cpp
    latches joint_torque_ext_valid true PERMANENTLY, once, in the
    joint_torque_ext_cmd subscription callback, and never resets it -- so
    relay_valid reads True for the remaining life of any driver process
    that has ever received one such message (true of this project's
    long-running real driver processes from earlier C1/C4 sessions),
    regardless of current safety state. relay_fresh is the real, time-gated
    "is a relay message currently arriving" signal; relay_applied is proof
    a nonzero relayed force was actually summed. Both remain hard abort
    triggers; relay_valid alone does not.
  - position_error_clamped / primary_force_clamped / final_force_clamped.
  - telemetry stale (> force_budget_telemetry_stale_s).
  - deadman_ok False on the master's own telemetry while enabled is True
    (deadman lost during a force-producing stage).
  - planner_parameter_match_left/right False (declared-vs-actual config
    mismatch -- the bridge itself would already FAULT on this, checked
    here too as defense in depth).
  - bridge_state == "FAULT".
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from touch_haptics.msg import CartesianControllerTelemetry, DobDiagnostics, BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from frozen_config import FROZEN, bridge_cli_args, config_fingerprint  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "hardware_validation"))
import pre_arm_check  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[2] / "scripts" / "bridge_master_slave.py"
)

MAX_WAIT_S = 30.0
OBSERVATION_WINDOW_S = 10.0  # how long to hold in GRAVITY_SETTLE once reached

UNEXPECTED_SPRING_DAMPING_NORM_N = 0.05
COUPLING_ALPHA_TOLERANCE = 1e-9
QUIET_WINDOW_TOLERANCE_S = 1e-3  # float-compare slack for the hold_s crossing

# 2026-09-10 safety-hardening: pre-arm sample-collection window, same
# target/minimum convention as hardware_validation/verify_disarmed.py's
# post-disarm check (preferring 20 samples, never proceeding with fewer
# than 5, on each of left/right/bridge).
PRE_ARM_WINDOW_S = 3.0
PRE_ARM_TARGET_SAMPLES = 20
PRE_ARM_MIN_SAMPLES = 5

# 2026-09-10 real-hardware finding: the arm_teleop parameter request MUST
# NOT be sent via a blocking `subprocess.run(["ros2", "param", "set", ...])`
# -- that spawns a brand-new `ros2` CLI process (rclpy init, DDS discovery,
# service call, process exit) which measured 0.8-1.3s wall time on this
# machine, during which THIS process's own rclpy.spin_once() is never
# called, so every subscription's `self.latest[...]` timestamp freezes for
# the full duration -- the first two real-hardware H2 attempts both
# false-aborted on "telemetry stale" within milliseconds of the arm call
# returning, for exactly this reason (measured directly: see
# test_async_param_set_does_not_starve_telemetry_callbacks in
# test_part1_relay_abort_check.py). Fixed by using rcl_interfaces/srv/
# SetParameters directly as an ASYNC service client on the SAME node,
# spinning that node while waiting for the future -- this keeps every
# other subscription's callbacks serviced throughout (measured: <10ms max
# callback gap during the wait, vs 800-1300ms with the CLI subprocess).
ARM_PARAM_TIMEOUT_S = 5.0
POST_ARM_FRESHNESS_WINDOW_S = 2.0  # bounded wait for fresh left/right/diag
                                   # samples immediately after arming,
                                   # before declaring the armed monitoring
                                   # phase established (Task 1's explicit
                                   # "require fresh POST-ARM telemetry"
                                   # requirement)


def _set_bool_param_async(node: "GravitySettleCapture", client, name: str, value: bool,
                          timeout_s: float = ARM_PARAM_TIMEOUT_S):
    """Sets one bool parameter on the bridge node via its standard
    rcl_interfaces SetParameters service, asynchronously -- spins `node`
    (servicing every OTHER subscription callback too) while waiting for
    the response, instead of blocking the whole process the way a `ros2`
    CLI subprocess call does. Returns (ok: bool, reason: str)."""
    if not client.service_is_ready():
        if not client.wait_for_service(timeout_sec=timeout_s):
            return False, f"set_parameters service not available within {timeout_s}s"
    req = SetParameters.Request()
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    req.parameters = [p]
    future = client.call_async(req)
    deadline = time.time() + timeout_s
    while not future.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
    if not future.done():
        return False, f"set_parameters call timed out after {timeout_s}s"
    result = future.result()
    if result is None or not result.results:
        return False, f"set_parameters returned no result: {result}"
    r0 = result.results[0]
    return bool(r0.successful), r0.reason


TELEM_FLOAT3 = [
    "position_m", "velocity_mps", "target_position_m", "position_error_raw_m",
    "spring_force_n", "damping_force_n", "auxiliary_force_n",
    "force_before_primary_clamp_n", "force_after_primary_clamp_n",
    "relay_torque_received_nm", "relay_force_n", "force_before_final_clamp_n",
    "applied_force_n",
]
TELEM_BOOL = [
    "enabled", "deadman_ok", "target_valid", "target_fresh", "force_valid",
    "force_fresh", "relay_valid", "relay_fresh", "relay_applied",
    "tracking_valid", "position_error_clamped", "primary_force_clamped",
    "final_force_clamped", "joint_path_commanded", "cartesian_write_active",
    "near_singular",
]

BRIDGE_FIELDS = [
    "bridge_state", "gravity_ramp_alpha", "coupling_ramp_alpha", "ramp_alpha",
    "c1_enabled", "c2_enabled", "c3_enabled", "c4_enabled",
    "master_travel_limit_flag", "slave_travel_limit_flag",
    "planner_parameter_match_left", "planner_parameter_match_right",
    "force_budget_telemetry_fresh_left", "force_budget_telemetry_fresh_right",
    "gravity_settle_master_speed_mps", "gravity_settle_slave_speed_mps",
    "gravity_settle_master_pose_drift_m", "gravity_settle_slave_pose_drift_m",
    "gravity_settle_quiet_window_accumulated_s",
    "gravity_settle_accepted_master_pose_drift_m",
    "gravity_settle_accepted_slave_pose_drift_m",
]


class GravitySettleCapture(Node):
    def __init__(self):
        super().__init__("part1_gravity_settle_capture")
        self.rows = {"left": [], "right": [], "bridge": []}
        self.latest = {}
        self.create_subscription(CartesianControllerTelemetry, "/touch_left/cartesian_controller_telemetry",
                                  lambda m: self._on_telem("left", m), 50)
        self.create_subscription(CartesianControllerTelemetry, "/touch_right/cartesian_controller_telemetry",
                                  lambda m: self._on_telem("right", m), 50)
        self.create_subscription(DobDiagnostics, "/touch_left/dob_diagnostics",
                                  lambda m: self._on_dob("left", m), 20)
        self.create_subscription(DobDiagnostics, "/touch_right/dob_diagnostics",
                                  lambda m: self._on_dob("right", m), 20)
        self.create_subscription(BridgeDiagnostics, "/bridge_diagnostics", self._on_diag, 50)
        self._dob_latest = {"left": None, "right": None}
        self.phase = "PRE_ARM"
        self.abort_reason = None
        self.abort_t = None
        self.gate_pass_t = None
        self.gate_pass_accumulated_s = None
        self._last_accumulated_s = None
        self._param_client = self.create_client(SetParameters, "/touch_master_slave_bridge/set_parameters")

    def set_arm_teleop(self, value: bool, timeout_s: float = ARM_PARAM_TIMEOUT_S):
        """Async arm/disarm -- see _set_bool_param_async()'s docstring for
        why this replaces the previous blocking `ros2 param set` CLI call."""
        return _set_bool_param_async(self, self._param_client, "arm_teleop", value, timeout_s)

    def _on_dob(self, side, m):
        j = list(m.tau_dis_hat_nm) if len(m.tau_dis_hat_nm) >= 3 else [None, None, None]
        self._dob_latest[side] = {"dob_waist": j[0], "dob_shoulder": j[1], "dob_elbow": j[2],
                                  "dob_configured": bool(m.configured)}

    def _on_diag(self, m):
        t = time.time()
        if m.bridge_state != self.phase:
            print(f"[part1] bridge_state -> {m.bridge_state} "
                  f"(gravity_ramp_alpha={m.gravity_ramp_alpha:.3f} coupling_ramp_alpha={m.coupling_ramp_alpha:.4f})",
                  flush=True)
        self.phase = m.bridge_state
        row = {"t": t}
        for f in BRIDGE_FIELDS:
            row[f] = getattr(m, f)
        self.rows["bridge"].append(row)
        self.latest["diag"] = (t, m)

        # Gate-pass detection: the tick where the quiet-hold window first
        # reaches the configured hold_s, while genuinely inside
        # GRAVITY_SETTLE -- see module docstring. Recorded once, first
        # crossing only.
        if (self.gate_pass_t is None and m.bridge_state == "GRAVITY_SETTLE"
                and m.gravity_settle_quiet_window_accumulated_s >= FROZEN.gravity_settle_hold_s - QUIET_WINDOW_TOLERANCE_S):
            self.gate_pass_t = t
            self.gate_pass_accumulated_s = m.gravity_settle_quiet_window_accumulated_s
            print(f"[part1] GATE PASSED at t={t:.3f}: quiet_window_accumulated_s="
                  f"{m.gravity_settle_quiet_window_accumulated_s:.4f}s >= hold_s={FROZEN.gravity_settle_hold_s}s "
                  f"(master_speed={m.gravity_settle_master_speed_mps*1000:.3f}mm/s "
                  f"slave_speed={m.gravity_settle_slave_speed_mps*1000:.3f}mm/s "
                  f"master_drift={m.gravity_settle_master_pose_drift_m*1000:.4f}mm "
                  f"slave_drift={m.gravity_settle_slave_pose_drift_m*1000:.4f}mm)", flush=True)

    def _on_telem(self, side, m):
        t = time.time()
        self.latest[side] = (t, m)
        row = {"t": t, "phase": self.phase}
        for f in TELEM_FLOAT3:
            v = list(getattr(m, f))
            row[f"{f}_x"], row[f"{f}_y"], row[f"{f}_z"] = v
        for f in TELEM_BOOL:
            row[f] = bool(getattr(m, f))
        dob = self._dob_latest[side]
        if dob is not None:
            row.update(dob)
        else:
            row.update({"dob_waist": None, "dob_shoulder": None, "dob_elbow": None, "dob_configured": None})
        self.rows[side].append(row)

    def check_abort(self, now):
        for side in ("left", "right"):
            if side not in self.latest:
                continue
            t_recv, m = self.latest[side]
            if now - t_recv > FROZEN.force_budget_telemetry_stale_s:
                self.abort_reason = f"{side} telemetry stale ({now - t_recv:.3f}s)"
                return True
            if m.position_error_clamped or m.primary_force_clamped or m.final_force_clamped:
                self.abort_reason = f"{side} clamp engaged"
                return True
            # 2026-09-10 real-hardware finding: relay_valid must NOT be used
            # here -- touch_slave.cpp sets command_.joint_torque_ext_valid=
            # true ONCE, in the joint_torque_ext_cmd subscription callback,
            # and never resets it anywhere in the file. It is a permanent,
            # one-way, process-lifetime latch ("has this process EVER
            # received a structurally valid relay message since it
            # started"), not a live-state flag. Since the master/slave
            # driver processes on this project's real rig run continuously
            # across many sessions (including legitimate earlier C1/C4
            # bilateral_4ch.py runs), relay_valid reads True on effectively
            # every real H2 attempt regardless of current safety state --
            # confirmed directly: the first two real-hardware H2 runs both
            # false-aborted on this line within <1ms of arming, with
            # relay_valid=True but relay_fresh=False, relay_applied=False,
            # and relay_torque_received_nm/relay_force_n exactly [0,0,0] on
            # EVERY recorded sample of both runs -- i.e. no relay
            # contribution ever occurred; only this check's own
            # interpretation of the field was wrong. relay_fresh (a real,
            # time-gated freshness check against last_joint_torque_ext_time)
            # and relay_applied (proof a nonzero relayed force was actually
            # summed into force_device) are the genuine live-danger signals
            # and remain hard abort triggers below.
            if m.relay_fresh or m.relay_applied:
                self.abort_reason = (f"{side} relay_fresh={m.relay_fresh} relay_applied={m.relay_applied} "
                                     "-- C1/C4 contribution present despite H2 requiring all channels off")
                return True
            if m.enabled and not m.deadman_ok and side == "left":
                self.abort_reason = "master deadman lost while enabled=True"
                return True
            unexpected_norm = math.sqrt(
                sum(v * v for v in m.spring_force_n) + sum(v * v for v in m.damping_force_n))
            if unexpected_norm > UNEXPECTED_SPRING_DAMPING_NORM_N:
                self.abort_reason = (
                    f"{side} unexpected spring+damping force {unexpected_norm:.4f}N "
                    f"(> {UNEXPECTED_SPRING_DAMPING_NORM_N}N tolerance) during gravity-only hold "
                    f"-- target should equal live pose exactly, this implies it does not"
                )
                return True
        if "diag" in self.latest:
            _, d = self.latest["diag"]
            if d.bridge_state == "FAULT":
                self.abort_reason = "bridge FAULT"
                return True
            if abs(d.coupling_ramp_alpha) > COUPLING_ALPHA_TOLERANCE:
                self.abort_reason = f"coupling_ramp_alpha={d.coupling_ramp_alpha} (must be exactly 0 during hold)"
                return True
            if d.c1_enabled or d.c2_enabled or d.c3_enabled or d.c4_enabled:
                self.abort_reason = (f"unexpected channel enabled: c1={d.c1_enabled} c2={d.c2_enabled} "
                                     f"c3={d.c3_enabled} c4={d.c4_enabled} (H2 requires all four off)")
                return True
            if not d.planner_parameter_match_left or not d.planner_parameter_match_right:
                self.abort_reason = (f"planner_parameter_match_left={d.planner_parameter_match_left} "
                                     f"planner_parameter_match_right={d.planner_parameter_match_right}")
                return True
        return False


def _disarm(node: "GravitySettleCapture" = None):
    """Disarm. Prefers the async, non-blocking-to-callbacks path
    (node.set_arm_teleop) when a live node is available; falls back to the
    blocking CLI subprocess ONLY for the outer teardown `finally:` block,
    where no monitoring is happening any more and bridge_proc.terminate()
    runs immediately afterward regardless (so blocking briefly there does
    not reintroduce the callback-starvation problem this fix addresses)."""
    if node is not None:
        ok, reason = node.set_arm_teleop(False)
        if not ok:
            print(f"[part1][WARNING] async disarm did not confirm success: {reason!r} "
                  "-- falling back to a CLI disarm as a last resort.", flush=True)
            subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                           capture_output=True, text=True, timeout=10)
        return
    subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                   capture_output=True, text=True, timeout=10)


TEARDOWN_VERIFY_TIMEOUT_S = 8.0


def _verified_teardown(node, bridge_proc, expected_node_name: str = pre_arm_check.EXPECTED_BRIDGE_NODE_NAME,
                       timeout_s: float = TEARDOWN_VERIFY_TIMEOUT_S) -> dict:
    """2026-09-10 fix (Task 3): disarm, terminate the bridge PROCESS, then
    do NOT report clean until the bridge's ROS-graph presence has actually
    disappeared -- process exit and DDS-graph disappearance are not the
    same moment (DDS discovery has its own propagation latency). Polls
    check_single_bridge_node_instance()/check_enable_publisher_ownership()
    (reusing the same live `node`, if one exists) until BOTH report zero,
    or until `timeout_s` elapses, in which case this returns
    status=SAFETY_INCOMPLETE rather than silently returning control to the
    caller with a potentially-still-live bridge instance -- exactly the
    scenario that caused the real-hardware READY<->GRAVITY_RAMP_DOWN
    oscillation this task investigates. Always runs from the single,
    unified `finally:` block in main(), regardless of which return path
    got there."""
    result = {"bridge_process_terminated": None, "graph_clear": None, "status": "UNKNOWN"}

    if node is not None:
        try:
            _disarm(node)
        except Exception as exc:
            print(f"[part1][WARNING] disarm during teardown raised {exc!r}", flush=True)

    if bridge_proc is not None:
        bridge_proc.terminate()
        try:
            bridge_proc.wait(timeout=5)
            result["bridge_process_terminated"] = True
        except subprocess.TimeoutExpired:
            bridge_proc.kill()
            try:
                bridge_proc.wait(timeout=5)
                result["bridge_process_terminated"] = True
            except subprocess.TimeoutExpired:
                result["bridge_process_terminated"] = False
    else:
        result["bridge_process_terminated"] = True  # nothing was ever launched this run

    if node is not None and rclpy.ok():
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            ni = pre_arm_check.check_single_bridge_node_instance(node, expected_node_name)
            own = pre_arm_check.check_enable_publisher_ownership(node, expected_node_name)
            endpoint_counts = {t: len(v) for t, v in own.items() if isinstance(v, list)}
            all_gone = ni["count"] == 0 and all(n == 0 for n in endpoint_counts.values())
            if all_gone:
                result["graph_clear"] = True
                result["final_instance_count"] = ni["count"]
                result["final_endpoint_counts"] = endpoint_counts
                break
        if result["graph_clear"] is None:
            result["graph_clear"] = False
            result["final_instance_count"] = ni["count"]
            result["final_endpoint_counts"] = endpoint_counts
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    else:
        result["graph_clear"] = None  # not checked -- no live node to check with

    if result["bridge_process_terminated"] and result["graph_clear"]:
        result["status"] = "CLEAN"
    elif result["graph_clear"] is None and result["bridge_process_terminated"]:
        result["status"] = "UNKNOWN"
    else:
        result["status"] = "SAFETY_INCOMPLETE"
    return result


def _git(*args, repo_root):
    try:
        return subprocess.run(["git"] + list(args), capture_output=True, text=True,
                              cwd=repo_root, timeout=10).stdout.strip()
    except Exception as exc:
        return f"<git call failed: {exc}>"


def _write_pre_arm_fail_meta(out_dir: Path, args, repo_root: Path, bargs: list,
                             pre_arm: dict, ownership: dict, final_phase: str,
                             extra_abort_reason: str = None, arm_t: float = None,
                             node_instance: dict = None):
    """2026-09-10 fix (Task 4): every exit path -- including a PRE_ARM_FAIL,
    which previously left NO part1_meta.json at all -- must write a
    complete, machine-readable artifact so analyze_h2_gravity_settle.py can
    report a normal structured FAIL instead of crashing with
    FileNotFoundError. Mirrors the real meta.json schema's field set
    (same keys, so downstream tooling does not need two schemas), with
    obviously-empty run fields (gate_pass_t=None, etc.) since arming never
    happened or never produced a monitored run."""
    abort_reason = extra_abort_reason
    if abort_reason is None:
        if pre_arm.get("status") != "PRE_ARM_PASS":
            abort_reason = f"PRE_ARM_FAILED: {'; '.join(pre_arm.get('reasons', []))}"
        elif not ownership.get("ok"):
            abort_reason = f"PRE_ARM_FAILED: publisher ownership check failed: {ownership}"
        elif node_instance is not None and not node_instance.get("ok"):
            abort_reason = (f"PRE_ARM_FAILED: single-bridge-instance check failed -- found "
                            f"{node_instance.get('count')} instance(s), expected exactly 1: {node_instance}")
        else:
            abort_reason = "PRE_ARM_FAILED (unspecified reason)"
    meta = {
        "session_id": args.session_id,
        "config_fingerprint": config_fingerprint(),
        "git_head": _git("rev-parse", "HEAD", repo_root=repo_root),
        "git_dirty": bool(_git("status", "--porcelain", repo_root=repo_root)),
        "git_branch": _git("branch", "--show-current", repo_root=repo_root),
        "bridge_command": bargs,
        "aborted": True,
        "abort_reason": abort_reason,
        "abort_t": time.time(),
        "final_phase": final_phase,
        "disarm_confirmed": None,
        "arm_sent": arm_t is not None,
        "arm_t": arm_t,
        "gate_pass_t": None,
        "gate_pass_accumulated_s": None,
        "observation_window_s": args.observation_window_s,
        "frozen_thresholds": {
            "gravity_settle_velocity_threshold_mps": FROZEN.gravity_settle_velocity_threshold_mps,
            "gravity_settle_hold_s": FROZEN.gravity_settle_hold_s,
            "gravity_settle_min_time_s": FROZEN.gravity_settle_min_time_s,
            "gravity_settle_timeout_s": FROZEN.gravity_settle_timeout_s,
            "force_budget_telemetry_stale_s": FROZEN.force_budget_telemetry_stale_s,
            "unexpected_spring_damping_norm_n": UNEXPECTED_SPRING_DAMPING_NORM_N,
            # 2026-09-10 real-hardware finding, fix (Task 5): these were
            # missing entirely -- analyze_h2_gravity_settle.py's
            # slave_aux_y_expected_n read th.get("slave_gravity_comp_n")
            # against a dict that never had that key, always returning
            # None/null regardless of the actual resolved config. Sourced
            # from FROZEN (the single canonical config), not hardcoded.
            "slave_gravity_comp_n": FROZEN.slave_gravity_comp_n,
            "slave_gravity_axis": FROZEN.slave_gravity_axis,
            "left_gravity_comp_n": FROZEN.left_gravity_comp_n,
            "left_gravity_axis": FROZEN.left_gravity_axis,
        },
        "dob_baseline": None,
        "pre_arm_evaluation": pre_arm,
        "publisher_ownership": ownership,
        "single_bridge_instance": node_instance,
    }
    with open(out_dir / "part1_meta.json", "w") as f:
        json.dump(meta, f, indent=2, default=str)
    # Empty-but-present CSVs too, so the analyzer's file-existence checks
    # find a well-formed (if headerless/empty) file rather than nothing.
    for fname in ("part1_gravity_settle_master.csv", "part1_gravity_settle_slave.csv", "part1_bridge_diag.csv"):
        p = out_dir / fname
        if not p.exists():
            p.write_text("")
    print(f"[part1] wrote part1_meta.json (PRE_ARM_FAILED artifact) to {out_dir}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--session-id", default=None)
    ap.add_argument("--observation-window-s", type=float, default=OBSERVATION_WINDOW_S)
    ap.add_argument("--baseline-json", default=None,
                    help="path to an H0/H1 passive_baseline.json for the DOB baseline-relative "
                         "delta -- if omitted, the most recent experiment_runs/teleop_setup/*_h0h1/"
                         "passive_baseline.json is used automatically")
    ap.add_argument("--confirmed-supervision", action="store_true",
                    help="required -- must be passed explicitly to actually run the capture loop")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    out_dir = Path(args.out_dir)

    calib_path = str(out_dir / "calibration.json")
    bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(
        calib_path, enable_c2=False, gravity_settle_hold_mode=True,
        c3_enable=False, require_master_deadman=True, c1_enable=False, c4_enable=False)

    print("================ H2: resolved configuration ================", flush=True)
    print(f"session_id: {args.session_id}", flush=True)
    print(f"config_fingerprint: {config_fingerprint()}", flush=True)
    print(f"c1_enable=False c2_enable(enable_c2)=False c3_enable=False c4_enable=False "
          f"require_master_deadman=True gravity_settle_hold_mode=True", flush=True)
    print(f"slave_gravity_comp_n={FROZEN.slave_gravity_comp_n}N "
          f"left_gravity_comp_n={FROZEN.left_gravity_comp_n}N (master UNCOMPENSATED)", flush=True)
    print(f"gravity_settle_velocity_threshold_mps={FROZEN.gravity_settle_velocity_threshold_mps} "
          f"(SIM-derived, NOT yet hardware-validated -- used as configured, not tuned by this script)", flush=True)
    print(f"gravity_settle_hold_s={FROZEN.gravity_settle_hold_s} "
          f"gravity_settle_min_time_s={FROZEN.gravity_settle_min_time_s} "
          f"gravity_settle_timeout_s={FROZEN.gravity_settle_timeout_s}", flush=True)
    print("[part1] bridge command:", " ".join(bargs), flush=True)
    print("===============================================================", flush=True)

    if not args.confirmed_supervision:
        print("[part1] --confirmed-supervision not passed -- refusing to launch the bridge "
              "or arm anything. Exiting without action.", flush=True)
        sys.exit(3)

    # Baseline DOB lookup (read-only, no actuation implication).
    baseline_path = args.baseline_json
    if baseline_path is None:
        candidates = sorted((repo_root / "experiment_runs" / "teleop_setup").glob("*_h0h1"),
                            key=lambda p: p.stat().st_mtime, reverse=True)
        for c in candidates:
            if (c / "passive_baseline.json").exists():
                baseline_path = str(c / "passive_baseline.json")
                break
    baseline = None
    if baseline_path and Path(baseline_path).exists():
        bdata = json.loads(Path(baseline_path).read_text())
        import statistics
        lv = [r["tau_y"] for r in bdata.get("left_dob", []) if r.get("tau_y") is not None]
        rv = [r["tau_y"] for r in bdata.get("right_dob", []) if r.get("tau_y") is not None]
        baseline = {
            "source_file": baseline_path,
            "note": ("baseline dob values below come from the EXISTING passive_baseline_capture.py "
                    "schema, which stores tau_dis_hat_nm[1] under the key 'tau_y' -- index 1 is the "
                    "SHOULDER joint (see DobDiagnostics.msg), NOT a Cartesian Y disturbance; kept under "
                    "this project's existing (mislabeled) key name here ONLY so the delta below is "
                    "computed against the exact same quantity, index-for-index, as the H2 capture's "
                    "dob_shoulder field"),
            "master_shoulder_dob_mean_nm": statistics.fmean(lv) if lv else None,
            "slave_shoulder_dob_mean_nm": statistics.fmean(rv) if rv else None,
        }
        print(f"[part1] DOB baseline loaded from {baseline_path}: "
              f"master_shoulder={baseline['master_shoulder_dob_mean_nm']:.5f}Nm "
              f"slave_shoulder={baseline['slave_shoulder_dob_mean_nm']:.5f}Nm", flush=True)
    else:
        print("[part1][WARNING] no H0/H1 baseline JSON found -- delta_tau_dis will be omitted "
              "from metrics; absolute DOB values will still be recorded and reported.", flush=True)

    out_dir.mkdir(parents=True, exist_ok=True)

    # 2026-09-10 fix (Task 3): rclpy/node creation moved BEFORE the bridge
    # subprocess is launched, so this run can prove the ROS graph is clean
    # (no leftover touch_master_slave_bridge instance from a previous run
    # still alive) BEFORE ever spawning its own bridge. `node` and
    # `bridge_proc` are tracked from here so the single, unified
    # `_verified_teardown()` call in `finally:` below can always find them,
    # regardless of which return path this function takes.
    rclpy.init()
    node = GravitySettleCapture()
    bridge_proc = None
    aborted = False
    arm_t = None
    try:
        # ---- Task 3: pre-launch single-instance check (fail closed) ----
        # Real-hardware finding, 2026-09-10: two H2 runs started ~3s apart
        # caused a previous run's still-shutting-down bridge process to
        # race a newly-launched one, both briefly alive as
        # touch_master_slave_bridge simultaneously, observed as a
        # READY<->GRAVITY_RAMP_DOWN diagnostics oscillation. This check
        # refuses to launch a second bridge if the ROS graph already shows
        # one -- run_h2.sh's flock single-instance lock is the primary
        # defense; this is the independent, in-process defense-in-depth
        # check for anyone invoking this script directly.
        deadline = time.time() + 1.5
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        pre_launch = pre_arm_check.check_single_bridge_node_instance(node)
        if pre_launch["count"] != 0:
            reason = (f"PRE_LAUNCH_FAILED: {pre_launch['count']} existing instance(s) of "
                     f"{pre_launch['expected_node_name']!r} already present in the ROS graph "
                     f"before this run launched its own bridge -- refusing to launch a second "
                     f"one. details: {pre_launch}")
            print(f"[part1][FAIL CLOSED] {reason}", flush=True)
            _write_pre_arm_fail_meta(
                out_dir, args, repo_root, bargs,
                pre_arm={"status": "PRE_ARM_FAIL", "reasons": [reason]},
                ownership={"ok": False, "note": "not checked -- refused before this run's bridge was launched"},
                final_phase="UNKNOWN (this run never launched a bridge)",
                extra_abort_reason=reason, node_instance=pre_launch,
            )
            return

        # 2026-09-10 real-hardware finding, fix: this loop used to call
        # bridge_proc.stdout.readline() in BLOCKING text mode with no
        # rclpy.spin_once() anywhere in the loop -- since Task 3's fix
        # moved `node` creation to BEFORE this point (so the pre-launch
        # single-instance check could run first), that node's own live
        # telemetry subscriptions were starved for the ENTIRE bridge-
        # startup duration. Measured directly (sim harness): bridge
        # startup took 1.566s, during which the master telemetry callback
        # gap was 1.566s -- effectively the whole wait, one single frozen
        # gap, closely matching the magnitude of the real-hardware 0.96s
        # gap this bug produced. Fixed by reading the subprocess's stdout
        # in NON-BLOCKING raw-byte mode and interleaving rclpy.spin_once()
        # calls, so telemetry keeps draining throughout this wait exactly
        # as it does everywhere else in this script.
        bridge_proc = subprocess.Popen(bargs, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        os.set_blocking(bridge_proc.stdout.fileno(), False)

        deadline = time.time() + MAX_WAIT_S
        ready = False
        stdout_buf = b""
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                chunk = bridge_proc.stdout.read(65536)
            except BlockingIOError:
                chunk = None
            if chunk:
                stdout_buf += chunk
                while b"\n" in stdout_buf:
                    raw_line, stdout_buf = stdout_buf.split(b"\n", 1)
                    text_line = raw_line.decode("utf-8", errors="replace")
                    print("[bridge]", text_line, flush=True)
                    if "teleoperation live" in text_line:
                        ready = True
                        break
                if ready:
                    break
            elif bridge_proc.poll() is not None:
                break
        if not ready:
            print("[part1][ERROR] bridge never reached ready -- aborting, no arm sent.", flush=True)
            return

        # 2026-09-10 real-hardware finding, fix (Task 2's "Preferred
        # design"): `node.rows[...]` has been accumulating since node
        # creation -- BEFORE this run's own bridge even launched, let
        # alone reported ready. Evaluating pre-arm freshness against that
        # full accumulated history means an old startup-phase gap (even a
        # genuinely harmless one, or one from the fix above before it
        # existed) could poison a decision that should only be about
        # "is telemetry fresh RIGHT NOW, in a bounded recent window."
        # Explicitly reset here, immediately after the bridge is confirmed
        # ready, so the dedicated PRE-ARM window below starts from a
        # provably clean slate.
        node.rows["left"].clear()
        node.rows["right"].clear()
        node.rows["bridge"].clear()
        pre_arm_window_start_t = time.time()

        # ---- PRE-ARM verification (2026-09-10 safety-hardening change) ----
        # Collect fresh samples over a bounded window (preferring
        # PRE_ARM_TARGET_SAMPLES, never fewer than PRE_ARM_MIN_SAMPLES)
        # AFTER the bridge has started publishing but BEFORE arm_teleop is
        # ever sent -- this is what proves any stale enabled=True latch
        # (found by a read-only live check before this task) was actually
        # cleared by the bridge's own unconditional enable=False publish,
        # not merely assumed. See pre_arm_check.py's module docstring for
        # the PRE-ARM vs POST-DISARM distinction.
        deadline = time.time() + PRE_ARM_WINDOW_S
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if (len(node.rows["left"]) >= PRE_ARM_TARGET_SAMPLES
                    and len(node.rows["right"]) >= PRE_ARM_TARGET_SAMPLES
                    and len(node.rows["bridge"]) >= PRE_ARM_TARGET_SAMPLES):
                break

        ownership = pre_arm_check.check_enable_publisher_ownership(node)
        node_instance = pre_arm_check.check_single_bridge_node_instance(node)
        pre_arm = pre_arm_check.evaluate_pre_arm_samples(
            node.rows["left"], node.rows["right"], node.rows["bridge"],
            force_tolerance_n=UNEXPECTED_SPRING_DAMPING_NORM_N,
            telemetry_stale_s=FROZEN.force_budget_telemetry_stale_s,
            min_samples=PRE_ARM_MIN_SAMPLES,
            window_start_t=pre_arm_window_start_t,
        )
        print(f"[part1] pre-arm publisher ownership: {ownership}", flush=True)
        print(f"[part1] pre-arm single-bridge-instance check: {node_instance}", flush=True)
        print(f"[part1] pre-arm sample evaluation: status={pre_arm['status']} "
              f"left_n={pre_arm.get('left_n_samples')} right_n={pre_arm.get('right_n_samples')} "
              f"bridge_n={pre_arm.get('bridge_n_samples')} "
              f"window_duration_s={pre_arm.get('pre_arm_window_duration_s')} "
              f"left_final_age_s={pre_arm.get('left_final_sample_age_s')} "
              f"right_final_age_s={pre_arm.get('right_final_sample_age_s')}", flush=True)
        if pre_arm["status"] != "PRE_ARM_PASS":
            for reason in pre_arm["reasons"]:
                print(f"[part1][PRE_ARM_FAIL] {reason}", flush=True)
        if not ownership["ok"]:
            print(f"[part1][PRE_ARM_FAIL] enable/diagnostics publisher ownership check failed: {ownership}", flush=True)
        if not node_instance["ok"]:
            print(f"[part1][PRE_ARM_FAIL] single-bridge-instance check failed "
                  f"(found {node_instance['count']} instance(s) of {node_instance['expected_node_name']!r}, "
                  f"expected exactly 1): {node_instance}", flush=True)

        if pre_arm["status"] != "PRE_ARM_PASS" or not ownership["ok"] or not node_instance["ok"]:
            print("[part1][ABORT] pre-arm verification FAILED -- arm_teleop will NEVER be sent this run.",
                  flush=True)
            with open(out_dir / "part1_pre_arm_result.json", "w") as f:
                json.dump({"pre_arm_evaluation": pre_arm, "publisher_ownership": ownership,
                          "single_bridge_instance": node_instance}, f, indent=2, default=str)
            # 2026-09-10 fix (Task 4): every exit path must leave a
            # complete, machine-readable artifact -- a PRE_ARM_FAIL used to
            # leave no part1_meta.json at all, which crashed
            # analyze_h2_gravity_settle.py with FileNotFoundError instead
            # of reporting a structured FAIL. current_phase reflects
            # whatever bridge_state was last observed during the pre-arm
            # window (e.g. GRAVITY_RAMP_DOWN if a previous run's bridge
            # instance was still shutting down -- see Task 2's finding).
            _write_pre_arm_fail_meta(out_dir, args, repo_root, bargs, pre_arm, ownership, node.phase,
                                     node_instance=node_instance)
            return

        print("[part1] pre-arm verification PASSED -- arming now.", flush=True)
        with open(out_dir / "part1_pre_arm_result.json", "w") as f:
            json.dump({"pre_arm_evaluation": pre_arm, "publisher_ownership": ownership}, f, indent=2, default=str)

        arm_ok, arm_reason = node.set_arm_teleop(True)
        if not arm_ok:
            print(f"[part1][ERROR] could not arm: {arm_reason!r}", flush=True)
            _write_pre_arm_fail_meta(out_dir, args, repo_root, bargs, pre_arm, ownership, node.phase,
                                     extra_abort_reason=f"arm request failed: {arm_reason!r}")
            return
        arm_t = time.time()

        # 2026-09-10 fix (Task 1): require FRESH post-arm telemetry from
        # left, right, AND bridge diagnostics before declaring the armed
        # monitoring phase established -- confirms spin_once() is actually
        # draining the callback queue right after arming (the same
        # mechanism that was starved by the old blocking CLI call), rather
        # than assuming it.
        fresh_deadline = time.time() + POST_ARM_FRESHNESS_WINDOW_S
        post_arm_fresh = {"left": False, "right": False, "diag": False}
        while time.time() < fresh_deadline and not all(post_arm_fresh.values()):
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.time()
            for key in ("left", "right", "diag"):
                if key in node.latest and now - node.latest[key][0] <= FROZEN.force_budget_telemetry_stale_s:
                    post_arm_fresh[key] = True
        if not all(post_arm_fresh.values()):
            reason = f"post-arm telemetry freshness not established within {POST_ARM_FRESHNESS_WINDOW_S}s: {post_arm_fresh}"
            print(f"[part1][ABORT] {reason} -- disarming immediately.", flush=True)
            _disarm(node)  # prompt disarm here, not deferred to the unified teardown below --
                           # arming DID succeed on this path, so minimizing time-to-disarm matters.
            _write_pre_arm_fail_meta(out_dir, args, repo_root, bargs, pre_arm, ownership, node.phase,
                                     extra_abort_reason=reason, arm_t=arm_t)
            return
        print(f"[part1] armed (hold mode) at t={arm_t:.3f}, post-arm telemetry freshness confirmed "
              f"(left/right/diag). Waiting for the quiet-hold gate to pass, "
              f"then observing for {args.observation_window_s:.1f}s...", flush=True)

        deadline = time.time() + MAX_WAIT_S
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.time()
            if node.check_abort(now):
                node.abort_t = now
                print(f"[part1][ABORT] t={now:.3f} {node.abort_reason} -- disarming.", flush=True)
                _disarm(node)
                aborted = True
                break
            if node.gate_pass_t is not None and now - node.gate_pass_t >= args.observation_window_s:
                print("[part1] observation window complete.", flush=True)
                break
            if node.phase == "FAULT":
                aborted = True
                node.abort_reason = node.abort_reason or "bridge_state==FAULT (observed via phase, no other abort matched)"
                node.abort_t = now
                break
        else:
            if node.gate_pass_t is None:
                aborted = True
                node.abort_reason = f"quiet-hold gate never passed within MAX_WAIT_S={MAX_WAIT_S}s"
                node.abort_t = time.time()
                print(f"[part1][ABORT] {node.abort_reason}", flush=True)

        _disarm(node)
        deadline = time.time() + 10.0
        while time.time() < deadline and node.phase not in ("READY", "GRAVITY_RAMP_DOWN"):
            rclpy.spin_once(node, timeout_sec=0.1)
        disarm_confirmed = node.phase in ("READY", "GRAVITY_RAMP_DOWN")

        for side, fname in (("left", "part1_gravity_settle_master.csv"), ("right", "part1_gravity_settle_slave.csv")):
            with open(out_dir / fname, "w", newline="") as f:
                if node.rows[side]:
                    w = csv.DictWriter(f, fieldnames=list(node.rows[side][0].keys()))
                    w.writeheader(); w.writerows(node.rows[side])
        with open(out_dir / "part1_bridge_diag.csv", "w", newline="") as f:
            if node.rows["bridge"]:
                w = csv.DictWriter(f, fieldnames=list(node.rows["bridge"][0].keys()))
                w.writeheader(); w.writerows(node.rows["bridge"])

        meta = {
            "session_id": args.session_id,
            "config_fingerprint": config_fingerprint(),
            "git_head": _git("rev-parse", "HEAD", repo_root=repo_root),
            "git_dirty": bool(_git("status", "--porcelain", repo_root=repo_root)),
            "git_branch": _git("branch", "--show-current", repo_root=repo_root),
            "bridge_command": bargs,
            "aborted": aborted,
            "abort_reason": node.abort_reason,
            "abort_t": node.abort_t,
            "final_phase": node.phase,
            "disarm_confirmed": disarm_confirmed,
            "arm_t": arm_t,
            "gate_pass_t": node.gate_pass_t,
            "gate_pass_accumulated_s": node.gate_pass_accumulated_s,
            "observation_window_s": args.observation_window_s,
            "frozen_thresholds": {
                "gravity_settle_velocity_threshold_mps": FROZEN.gravity_settle_velocity_threshold_mps,
                "gravity_settle_hold_s": FROZEN.gravity_settle_hold_s,
                "gravity_settle_min_time_s": FROZEN.gravity_settle_min_time_s,
                "gravity_settle_timeout_s": FROZEN.gravity_settle_timeout_s,
                "force_budget_telemetry_stale_s": FROZEN.force_budget_telemetry_stale_s,
                "unexpected_spring_damping_norm_n": UNEXPECTED_SPRING_DAMPING_NORM_N,
                # 2026-09-10 real-hardware finding, fix (Task 5): see the
                # matching comment in _write_pre_arm_fail_meta() above --
                # this key was missing entirely, so
                # analyze_h2_gravity_settle.py's slave_aux_y_expected_n
                # always read null regardless of the actual resolved
                # config.
                "slave_gravity_comp_n": FROZEN.slave_gravity_comp_n,
                "slave_gravity_axis": FROZEN.slave_gravity_axis,
                "left_gravity_comp_n": FROZEN.left_gravity_comp_n,
                "left_gravity_axis": FROZEN.left_gravity_axis,
            },
            "dob_baseline": baseline,
        }
        with open(out_dir / "part1_meta.json", "w") as f:
            json.dump(meta, f, indent=2, default=str)
        print(f"[part1] wrote CSVs + part1_meta.json to {out_dir} (aborted={aborted}, "
              f"gate_pass_t={node.gate_pass_t}, disarm_confirmed={disarm_confirmed})", flush=True)
    finally:
        # 2026-09-10 fix (Task 3): ONE unified, verified teardown for every
        # return path (normal completion, any abort, any pre-arm/arm
        # failure, or an unhandled exception) -- disarms, terminates the
        # bridge PROCESS, and then does NOT return control to the caller
        # until the bridge's ROS graph presence (node instance + its
        # publishers on all three watched topics) has ACTUALLY disappeared,
        # not merely assumed from the OS process having exited (DDS
        # discovery has its own latency, separate from process exit).
        teardown = _verified_teardown(node, bridge_proc)
        print(f"[part1] teardown verification: {teardown}", flush=True)
        if teardown["status"] != "CLEAN":
            print(f"[part1][SAFETY_INCOMPLETE] teardown did not verify clean within the bounded "
                  f"timeout -- a subsequent H2 launch may be unsafe until this is investigated "
                  f"manually. status={teardown['status']}", flush=True)
        try:
            with open(out_dir / "part1_teardown.json", "w") as f:
                json.dump(teardown, f, indent=2, default=str)
        except Exception as exc:
            print(f"[part1][WARNING] could not write part1_teardown.json: {exc!r}", flush=True)


if __name__ == "__main__":
    main()
