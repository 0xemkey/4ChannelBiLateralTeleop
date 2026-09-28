#!/usr/bin/env python3
"""Single-trial recorder for the A2(0.55N)/B3(0.70N) scientific gravity-comp
comparison (2026-09-09, v3 -- rewritten twice after real-hardware/protocol
findings, see below). Framework-free (plain rclpy, no rosbag2_py), matching
this project's established testing convention.

Assumes bridge_master_slave.py is ALREADY RUNNING, unarmed (arm_teleop=false),
with the frozen A2/B3 configuration and the SLAVE_GRAVITY_COMP_N for this
trial's condition. This script does not start/stop the bridge or set
arm_teleop -- the operator does that via `ros2 param set`. This script's job
is ONLY to record.

v1 -> v2: v1 polled `ros2 param get arm_teleop` with a fixed 60s timeout,
which expired mid-reference-matching and killed the recorder before the
operator armed -- trial_0.70_1 was classified
PROTOCOL_DEVIATION_RECORDER_STARTED_POST_ACTIVE as a result. v2 replaced the
poll with continuous /bridge_diagnostics observation and a 600s (0=infinite)
arm-wait timeout.

v2 -> v3 (this version): v2 still only started recording (`node.start()`,
`_rec()`'s `if not self.recording: return` gate) AFTER confirmed ACTIVE --
so ramp_start_t/arm_transition_t were real EVENT timestamps, but the actual
RAMPING TELEMETRY (gravity 0->configured ramp, C2/C3 engaging, any force
spike/clamp/discontinuity) was still discarded, exactly the category of
missing evidence that invalidated trial_0.70_1 in the first place. v3
separates two concepts:

  EPISODE CAPTURE: starts once required topics are confirmed live and the
  bridge is confirmed READY/unarmed, BEFORE the operator arms. Continues,
  uninterrupted, through READY(pre-arm) -> RAMPING -> ACTIVE -> READY
  (post-disarm) for at least a settling window. Every sample from every
  subscribed topic is kept, tagged with wall-clock time, for the whole
  episode -- nothing before ACTIVE_START is thrown away anymore.

  ACTIVE METRIC WINDOW: unchanged in definition -- exactly [active_start_t,
  active_start_t + duration_s), latched in the SAME /bridge_diagnostics
  callback that captures every other transition (not invented later in the
  main polling loop). The primary 20s performance metrics are computed only
  from this sub-window of the episode; the ramp and post-disarm segments get
  their own separate validation metrics (Sections 7-8 below).

  CONTINUOUS ACTIVE-WINDOW INTEGRITY CHECK: while inside the metric window,
  every bridge_diagnostics sample is checked against bridge_state==ACTIVE
  and ramp_alpha>=0.999. Any exit is latched as active_window_violation and
  the trial is NOT silently treated as a clean scientific replicate.

Usage:
    python3 run_gravcomp_trial.py --session-dir SESSION_DIR --condition 0.55 \\
        --trial-index 1 --duration-s 20 [--arm-wait-timeout-s 600] \\
        [--disarm-wait-timeout-s 120] [--settle-window-s 2.0]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics, DobDiagnostics


def _now():
    return time.time()


# 2026-09-09 Part 5 (validation-infrastructure follow-up): the full set of
# per-tick BridgeDiagnostics fields this recorder now captures -- not just
# the 4-field (bridge_state, ramp_alpha, follow_error_m, gravity_ramp_alpha)
# tuple v3 shipped with. bridge_state is recorded VERBATIM (GRAVITY_RAMP_UP/
# GRAVITY_SETTLE/COUPLING_RAMP_UP/ACTIVE/COUPLING_RAMP_DOWN/GRAVITY_RAMP_DOWN/
# READY/FAULT) -- see _phase_label() below for how PRE_ARM/POST_DISARM_READY/
# SELF_FOLLOW_SETTLE are derived from it, instead of the old single generic
# "RAMPING" bucket. Order matters -- BRIDGE_DIAG_IDX below indexes into it.
BRIDGE_DIAG_FIELDS = (
    "bridge_state", "gravity_ramp_alpha", "coupling_ramp_alpha", "ramp_alpha", "follow_error_m",
    # ACCEPTED margins (published-target, safety-relevant -- see
    # coupling_force_budget.py's 2026-09-09 Part 1 accepted/candidate split).
    "master_accepted_force_margin_n", "slave_accepted_force_margin_n",
    "master_accepted_position_margin_m", "slave_accepted_position_margin_m",
    # CANDIDATE margins (last bisection trial, diagnostic/explanatory only --
    # CAN be legitimately negative on a held tick).
    "master_candidate_force_margin_n", "slave_candidate_force_margin_n",
    "master_candidate_position_margin_m", "slave_candidate_position_margin_m",
    "master_predicted_force_n", "slave_predicted_force_n",  # each a 3-tuple
    "telemetry_fresh_left", "telemetry_fresh_right",
    "planner_parameter_match_left", "planner_parameter_match_right",
    "coupling_ramp_held", "coupling_ramp_stall_time_s",
    # GRAVITY_SETTLE observability (Part 3).
    "settle_master_speed_mps", "settle_slave_speed_mps",
    "settle_master_pose_drift_m", "settle_slave_pose_drift_m",
    "settle_quiet_window_accumulated_s",
)
BRIDGE_DIAG_IDX = {name: i for i, name in enumerate(BRIDGE_DIAG_FIELDS)}


def _bridge_diag_row(m) -> tuple:
    return tuple(
        (bool(getattr(m, f)) if f in ("telemetry_fresh_left", "telemetry_fresh_right",
                                       "planner_parameter_match_left", "planner_parameter_match_right",
                                       "coupling_ramp_held")
         else tuple(getattr(m, f)) if f in ("master_predicted_force_n", "slave_predicted_force_n")
         else getattr(m, f))
        for f in (
            "bridge_state", "gravity_ramp_alpha", "coupling_ramp_alpha", "ramp_alpha", "follow_error_m",
            "master_accepted_force_budget_margin_n", "slave_accepted_force_budget_margin_n",
            "master_accepted_position_error_margin_m", "slave_accepted_position_error_margin_m",
            "master_candidate_force_budget_margin_n", "slave_candidate_force_budget_margin_n",
            "master_candidate_position_error_margin_m", "slave_candidate_position_error_margin_m",
            "master_predicted_force_n", "slave_predicted_force_n",
            "force_budget_telemetry_fresh_left", "force_budget_telemetry_fresh_right",
            "planner_parameter_match_left", "planner_parameter_match_right",
            "coupling_ramp_held", "coupling_ramp_stall_time_s",
            "gravity_settle_master_speed_mps", "gravity_settle_slave_speed_mps",
            "gravity_settle_master_pose_drift_m", "gravity_settle_slave_pose_drift_m",
            "gravity_settle_quiet_window_accumulated_s",
        )
    )


# States that mean the ARM/DISARM sequence is actively transitioning --
# any of these get their bridge_state used VERBATIM as the phase label.
# Only bridge_state=="READY" needs disambiguating (pre-arm vs post-disarm),
# and only GRAVITY_RAMP_DOWN needs a further sub-phase split
# (SELF_FOLLOW_SETTLE, see startup_alignment.py's own comment on why that
# is represented as a sub-phase of GRAVITY_RAMP_DOWN rather than a distinct
# bridge_state).
_VERBATIM_PHASE_STATES = frozenset((
    "GRAVITY_RAMP_UP", "GRAVITY_SETTLE", "COUPLING_RAMP_UP", "ACTIVE",
    "COUPLING_RAMP_DOWN", "FAULT",
))


def _phase_label(bridge_state: str, gravity_ramp_alpha: float, ever_armed: bool) -> str:
    """2026-09-09 Part 5: real per-sample phase label, derived from the
    ACTUAL bridge_state the sample was recorded under -- replaces the old
    time-window-reconstructed 6-bucket PRE_ARM/RAMPING/ACTIVE/
    ACTIVE_TAIL_PRE_DISARM/POST_DISARM/UNKNOWN scheme, which called every
    sample between arm and ACTIVE "RAMPING" regardless of which of the
    (now 3 separate) arm-sequence states it was actually in."""
    if bridge_state == "READY":
        return "POST_DISARM_READY" if ever_armed else "PRE_ARM"
    if bridge_state == "GRAVITY_RAMP_DOWN":
        # gravity_ramp_alpha stays pinned at 1.0 through the SELF_FOLLOW_
        # SETTLE sub-phase (see startup_alignment.py's GRAVITY_RAMP_DOWN
        # tick() branch) -- once it starts actually decreasing below 1.0,
        # gravity is really ramping down.
        return "SELF_FOLLOW_SETTLE" if gravity_ramp_alpha >= 1.0 - 1e-9 else "GRAVITY_RAMP_DOWN"
    if bridge_state in _VERBATIM_PHASE_STATES:
        return bridge_state
    return bridge_state or "UNKNOWN"


# Topics whose absence is a hard abort -- the trial cannot be scientifically
# meaningful without these.
REQUIRED_TOPICS = ("left_pose", "right_pose", "left_force", "right_force",
                   "left_telem", "right_telem", "bridge_diag",
                   "left_target", "right_target")
# DOB is observation-only and may legitimately be unconfigured/inert (see
# DobDiagnostics.msg's own header) -- its absence is recorded explicitly
# (DOB_AVAILABLE=false) rather than either silently ignored or hard-aborted.
OPTIONAL_TOPICS = ("left_dob", "right_dob")


DEFAULT_TOPICS = {
    "left_pose": "/touch_left/pose", "right_pose": "/touch_right/pose",
    "left_target": "/touch_left/target_pose", "right_target": "/touch_right/target_pose",
    "left_force": "/touch_left/applied_force", "right_force": "/touch_right/applied_force",
    "left_telem": "/touch_left/cartesian_controller_telemetry",
    "right_telem": "/touch_right/cartesian_controller_telemetry",
    "left_dob": "/touch_left/dob_diagnostics", "right_dob": "/touch_right/dob_diagnostics",
    "bridge_diag": "/bridge_diagnostics",
}


class TrialRecorder(Node):
    def __init__(self, duration_s, topics=None):
        super().__init__("gravcomp_trial_recorder")
        self.duration_s = duration_s
        # Overridable only for sim/integration testing (test_gravcomp_trial_recorder_live.py)
        # so that test doesn't have to run on the real hardware's own live
        # /touch_left, /touch_right topics -- defaults are the real names,
        # unchanged from every prior version, for actual trials.
        topics = {**DEFAULT_TOPICS, **(topics or {})}
        self.topics = topics
        all_keys = REQUIRED_TOPICS + OPTIONAL_TOPICS
        # Whole-episode capture: every (wall_time, value) sample, from
        # episode start (READY, pre-arm) through the post-disarm settling
        # window. No "if not recording" gate -- see module docstring.
        self.episode = {k: [] for k in all_keys}
        self.last_seen = {k: None for k in all_keys}
        self.episode_recording = False
        self.prev_bridge_state = None
        # 2026-09-09 Part 5: True from the first non-READY bridge_state ever
        # observed -- lets _phase_label() distinguish PRE_ARM READY from
        # POST_DISARM_READY (both are bridge_state=="READY").
        self.ever_armed = False

        self.arm_transition_t = None
        self.ramp_start_t = None
        self.active_start_t = None
        self.active_end_t = None
        self.disarm_transition_t = None
        self.active_window_violations = []  # list of (wall_time, bridge_state, ramp_alpha)

        self.create_subscription(PoseStamped, topics["left_pose"],
                                  lambda m: self._rec("left_pose", (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 20)
        self.create_subscription(PoseStamped, topics["right_pose"],
                                  lambda m: self._rec("right_pose", (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 20)
        self.create_subscription(PoseStamped, topics["left_target"],
                                  lambda m: self._rec("left_target", (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 20)
        self.create_subscription(PoseStamped, topics["right_target"],
                                  lambda m: self._rec("right_target", (m.pose.position.x, m.pose.position.y, m.pose.position.z)), 20)
        self.create_subscription(WrenchStamped, topics["left_force"],
                                  lambda m: self._rec("left_force", (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)), 20)
        self.create_subscription(WrenchStamped, topics["right_force"],
                                  lambda m: self._rec("right_force", (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z)), 20)
        self.create_subscription(CartesianControllerTelemetry, topics["left_telem"],
                                  lambda m: self._rec("left_telem", _telem_row(m)), 20)
        self.create_subscription(CartesianControllerTelemetry, topics["right_telem"],
                                  lambda m: self._rec("right_telem", _telem_row(m)), 20)
        try:
            self.create_subscription(DobDiagnostics, topics["left_dob"],
                                      lambda m: self._rec("left_dob", tuple(m.tau_dis_hat_nm)), 5)
            self.create_subscription(DobDiagnostics, topics["right_dob"],
                                      lambda m: self._rec("right_dob", tuple(m.tau_dis_hat_nm)), 5)
        except Exception as e:
            self.get_logger().warn(f"dob_diagnostics subscription skipped: {e}")
        self.create_subscription(BridgeDiagnostics, topics["bridge_diag"],
                                  self._on_bridge_diag, 20)

    def _rec(self, key, value):
        self.last_seen[key] = value
        if not self.episode_recording:
            return
        self.episode[key].append((_now(), value))

    def start_episode(self):
        self.episode_recording = True

    def _on_bridge_diag(self, m):
        """The single place every state-transition timestamp is latched --
        callback-latched per the task's explicit requirement, never invented
        later from a main-loop poll.

        2026-09-09 staged-transition update: the single "RAMPING" state was
        replaced by startup_alignment.py with an explicit sequence (READY ->
        GRAVITY_RAMP_UP -> GRAVITY_SETTLE -> COUPLING_RAMP_UP -> ACTIVE ->
        COUPLING_RAMP_DOWN -> GRAVITY_RAMP_DOWN -> READY) so that gravity
        compensation and bilateral coupling are never ramped simultaneously
        -- see that module's docstring for the full root-cause rationale.
        The three event timestamps this recorder cares about map onto the
        NEW sequence as: arm_transition_t/ramp_start_t = the first moment
        the arm sequence begins (READY -> GRAVITY_RAMP_UP, still the
        earliest possible signal that arming started); active_start_t =
        unchanged (bridge_state=="ACTIVE" and ramp_alpha>=0.999 -- ramp_alpha
        is coupling_ramp_alpha's backward-compatible alias, so this is
        still exactly "bilateral coupling is fully engaged", unaffected by
        the staging); disarm_transition_t = the moment the disarm sequence
        FULLY completes and returns to READY (from either of the two
        disarm-side ramp-down states, not just directly from ACTIVE, since
        ACTIVE now always transitions through COUPLING_RAMP_DOWN/GRAVITY_
        RAMP_DOWN before reaching READY again)."""
        wall_t = _now()
        prev = self.prev_bridge_state
        if prev == "READY" and m.bridge_state != "READY" and self.ramp_start_t is None:
            self.ramp_start_t = wall_t
            self.arm_transition_t = wall_t
        if m.bridge_state != "READY":
            self.ever_armed = True
        if m.bridge_state == "ACTIVE" and m.ramp_alpha >= 0.999 and self.active_start_t is None:
            self.active_start_t = wall_t
        if (prev in ("ACTIVE", "COUPLING_RAMP_DOWN", "GRAVITY_RAMP_DOWN")
                and m.bridge_state == "READY" and self.disarm_transition_t is None):
            self.disarm_transition_t = wall_t

        # Continuous ACTIVE-window integrity check (Section 3): only while
        # we are inside the still-open metric window.
        if (self.active_start_t is not None and self.active_end_t is None
                and wall_t < self.active_start_t + self.duration_s):
            if m.bridge_state != "ACTIVE" or m.ramp_alpha < 0.999:
                self.active_window_violations.append((wall_t, m.bridge_state, m.ramp_alpha))

        self.prev_bridge_state = m.bridge_state
        self._rec("bridge_diag", _bridge_diag_row(m))

    def is_ready(self):
        bd = self.last_seen.get("bridge_diag")
        return bd is not None and bd[0] == "READY"


TELEM_IDX_SPRING, TELEM_IDX_DAMP, TELEM_IDX_AUX, TELEM_IDX_APPLIED = 0, 1, 2, 3
TELEM_IDX_POS_CLAMPED, TELEM_IDX_PRIMARY_FORCE_CLAMPED = 4, 5
TELEM_IDX_FINAL_FORCE_CLAMPED, TELEM_IDX_NEAR_SINGULAR = 6, 7
TELEM_IDX_POS_ERROR_RAW, TELEM_IDX_POS_ERROR_CLAMPED_VEC = 8, 9


def _telem_row(m):
    return (
        tuple(m.spring_force_n), tuple(m.damping_force_n), tuple(m.auxiliary_force_n),
        tuple(m.applied_force_n), bool(m.position_error_clamped), bool(m.primary_force_clamped),
        bool(m.final_force_clamped), bool(m.near_singular),
        tuple(m.position_error_raw_m), tuple(m.position_error_clamped_m),
    )


def _write_csv(path, rows, header, phase_fn):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["wall_t", "phase"] + header)
        for t, v in rows:
            w.writerow([t, phase_fn(t)] + list(_flatten(v)))


def _flatten(v):
    out = []
    for x in v:
        if isinstance(x, (tuple, list)):
            out.extend(_flatten(x))
        else:
            out.append(x)
    return out


def _delta_p(left, right):
    return [left[i] - right[i] for i in range(3)]


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _phase_fn(bridge_diag_rows):
    """2026-09-09 Part 5 replacement: phase(t) now looks up the REAL
    bridge_state (and gravity_ramp_alpha, for the SELF_FOLLOW_SETTLE
    sub-phase split) from the nearest-in-time recorded bridge_diag sample,
    via _phase_label() -- not a synthetic reconstruction from 3-4 latched
    timestamps. Every transition sample gets its actual state
    (GRAVITY_RAMP_UP/GRAVITY_SETTLE/COUPLING_RAMP_UP/ACTIVE/
    COUPLING_RAMP_DOWN/SELF_FOLLOW_SETTLE/GRAVITY_RAMP_DOWN/PRE_ARM/
    POST_DISARM_READY/FAULT), not a single generic "RAMPING" bucket."""
    if not bridge_diag_rows:
        return lambda t: "UNKNOWN"
    ever_armed_at = []
    seen_armed = False
    for t, row in bridge_diag_rows:
        if row[BRIDGE_DIAG_IDX["bridge_state"]] != "READY":
            seen_armed = True
        ever_armed_at.append(seen_armed)

    def phase(t):
        # nearest-in-time bridge_diag sample (matches compute_metrics'
        # own nearest() pairing convention elsewhere in this file).
        best_i = min(range(len(bridge_diag_rows)),
                    key=lambda i: abs(bridge_diag_rows[i][0] - t))
        row = bridge_diag_rows[best_i][1]
        return _phase_label(row[BRIDGE_DIAG_IDX["bridge_state"]],
                            row[BRIDGE_DIAG_IDX["gravity_ramp_alpha"]],
                            ever_armed_at[best_i])
    return phase


def _segment(rows, lo, hi):
    if lo is None or hi is None:
        return []
    return [(t, v) for t, v in rows if lo <= t < hi]


def _effective_hz(rows):
    if len(rows) < 2:
        return None
    span = rows[-1][0] - rows[0][0]
    return (len(rows) - 1) / span if span > 0 else None


def compute_metrics(episode, events, duration_s, condition_n, settle_window_s):
    """Pure function over the whole-episode (wall_time, value) lists plus
    the latched event timestamps -- kept separate from ROS glue so it is
    independently testable without rclpy."""
    ramp_start_t = events["ramp_start_t"]
    active_start_t = events["active_start_t"]
    disarm_t = events["disarm_transition_t"]

    left_pose = episode["left_pose"]
    right_pose = episode["right_pose"]
    if active_start_t is None:
        return {"error": "trial never reached ACTIVE_START", "event_timestamps": events}

    # The scientific metric window is DEFINED as exactly [active_start_t,
    # active_start_t + duration_s) -- not the wall-clock time the Python
    # loop happened to notice the spin had elapsed (events["active_end_t"],
    # kept separately below as active_capture_stop_observed_t). Using the
    # observed loop-exit time here would let scheduling jitter silently
    # expand or shrink the scientific window; this guarantees exactly
    # duration_s regardless of that jitter.
    metric_active_end_t = active_start_t + duration_s
    events = dict(events)  # do not mutate the caller's dict
    events["metric_active_end_t"] = metric_active_end_t
    events["active_capture_stop_observed_t"] = events.pop("active_end_t")
    events["exact_active_duration_valid"] = abs(
        metric_active_end_t - active_start_t - duration_s) < 1e-6

    active_left_pose = _segment(left_pose, active_start_t, metric_active_end_t)
    active_right_pose = _segment(right_pose, active_start_t, metric_active_end_t)
    if not active_left_pose or not active_right_pose:
        return {"error": "no pose samples inside the ACTIVE window", "event_timestamps": events}

    def nearest(t, series):
        best = min(series, key=lambda s: abs(s[0] - t))
        return best[1], abs(best[0] - t)

    dp_series = []
    pair_dts = []
    for t, lp in active_left_pose:
        rp, dt = nearest(t, active_right_pose)
        dp_series.append((t, _delta_p(lp, rp)))
        pair_dts.append(dt)

    pair_dts_sorted = sorted(pair_dts)
    n = len(pair_dts_sorted)
    p95_idx = min(n - 1, int(math.ceil(0.95 * n)) - 1) if n else 0
    pairing_audit = {
        "mean_abs_dt_s": statistics.fmean(pair_dts) if pair_dts else None,
        "p95_abs_dt_s": pair_dts_sorted[p95_idx] if n else None,
        "max_abs_dt_s": max(pair_dts) if pair_dts else None,
        "n_pairs": n,
    }

    initial = dp_series[0][1]
    final = dp_series[-1][1]
    norms = [_norm(dp) for _, dp in dp_series]

    def window(series, last_s):
        cutoff = series[-1][0] - last_s
        return [v for t, v in series if t >= cutoff]

    def stats(vals):
        if not vals:
            return {"mean": None, "std": None}
        return {"mean": statistics.fmean(vals), "std": statistics.pstdev(vals) if len(vals) > 1 else 0.0}

    dp_3s = window(dp_series, 3.0)
    dp_5s = window(dp_series, 5.0)
    norms_3s = [_norm(v) for v in dp_3s]
    norms_5s = [_norm(v) for v in dp_5s]

    rms_tracking_error = math.sqrt(statistics.fmean([n2 * n2 for n2 in norms])) if norms else None
    ptp_steady = (max(norms_5s) - min(norms_5s)) if norms_5s else None

    def clamp_pct(telem_rows, idx):
        if not telem_rows:
            return None
        flags = [row[1][idx] for row in telem_rows]
        return 100.0 * sum(1 for f in flags if f) / len(flags)

    active_left_telem = _segment(episode["left_telem"], active_start_t, metric_active_end_t)
    active_right_telem = _segment(episode["right_telem"], active_start_t, metric_active_end_t)

    metrics = {
        "condition_n": condition_n,
        "duration_s": duration_s,
        "event_timestamps": events,
        "active_window_violation_count": len(events.get("active_window_violations", [])),
        "n_samples_left_pose_active": len(active_left_pose),
        "n_samples_right_pose_active": len(active_right_pose),
        "pairing_audit": pairing_audit,
        "initial_dxyz": initial, "final_dxyz": final,
        "initial_norm": _norm(initial), "final_norm": _norm(final),
        "min_norm": min(norms), "max_norm": max(norms),
        "final_3s": {
            "mean_dx": stats([v[0] for v in dp_3s])["mean"], "std_dx": stats([v[0] for v in dp_3s])["std"],
            "mean_dy": stats([v[1] for v in dp_3s])["mean"], "std_dy": stats([v[1] for v in dp_3s])["std"],
            "mean_dz": stats([v[2] for v in dp_3s])["mean"], "std_dz": stats([v[2] for v in dp_3s])["std"],
            "mean_norm": stats(norms_3s)["mean"], "std_norm": stats(norms_3s)["std"],
        },
        "final_5s": {
            "mean_dx": stats([v[0] for v in dp_5s])["mean"], "std_dx": stats([v[0] for v in dp_5s])["std"],
            "mean_dy": stats([v[1] for v in dp_5s])["mean"], "std_dy": stats([v[1] for v in dp_5s])["std"],
            "mean_dz": stats([v[2] for v in dp_5s])["mean"], "std_dz": stats([v[2] for v in dp_5s])["std"],
            "mean_norm": stats(norms_5s)["mean"], "std_norm": stats(norms_5s)["std"],
        },
        "rms_tracking_error_m": rms_tracking_error,
        "peak_to_peak_final5s_m": ptp_steady,
        "clamp_pct": {
            "left_position_error_clamped": clamp_pct(active_left_telem, 4),
            "right_position_error_clamped": clamp_pct(active_right_telem, 4),
            "left_primary_force_clamped": clamp_pct(active_left_telem, 5),
            "right_primary_force_clamped": clamp_pct(active_right_telem, 5),
            "left_final_force_clamped": clamp_pct(active_left_telem, 6),
            "right_final_force_clamped": clamp_pct(active_right_telem, 6),
            "left_near_singular": clamp_pct(active_left_telem, 7),
            "right_near_singular": clamp_pct(active_right_telem, 7),
        },
        "n_telem_left_active": len(active_left_telem),
        "n_telem_right_active": len(active_right_telem),
        "effective_hz": {
            "left_pose": _effective_hz(active_left_pose), "right_pose": _effective_hz(active_right_pose),
            "left_telem": _effective_hz(active_left_telem), "right_telem": _effective_hz(active_right_telem),
        },
    }
    if active_right_telem:
        aux_y = [row[1][2][1] for row in active_right_telem]
        spring_y = [row[1][0][1] for row in active_right_telem]
        applied_y = [row[1][3][1] for row in active_right_telem]
        metrics["slave_y_force"] = {
            "aux_y_mean": statistics.fmean(aux_y), "aux_y_final": aux_y[-1],
            "spring_y_mean": statistics.fmean(spring_y),
            "applied_y_mean": statistics.fmean(applied_y),
        }

    # --- Section 7: ramp validation (RAMPING segment) ---
    ramp_left_telem = _segment(episode["left_telem"], ramp_start_t, active_start_t) if ramp_start_t else []
    ramp_right_telem = _segment(episode["right_telem"], ramp_start_t, active_start_t) if ramp_start_t else []
    ramp_bridge_diag = _segment(episode["bridge_diag"], ramp_start_t, active_start_t) if ramp_start_t else []
    ramp_left_target = _segment(episode["left_target"], ramp_start_t, active_start_t) if ramp_start_t else []
    ramp_right_target = _segment(episode["right_target"], ramp_start_t, active_start_t) if ramp_start_t else []

    configured_n = float(condition_n)
    ramp_metrics = {"n_ramp_samples_left_telem": len(ramp_left_telem), "n_ramp_samples_right_telem": len(ramp_right_telem)}
    if ramp_start_t is not None and active_start_t is not None:
        ramp_metrics["ramp_duration_measured_s"] = active_start_t - ramp_start_t
    _I = BRIDGE_DIAG_IDX
    if ramp_bridge_diag:
        # ramp_alpha (index _I["ramp_alpha"]) is the legacy coupling-ramp
        # alias -- see BRIDGE_DIAG_FIELDS' own comment for why this, not
        # gravity_ramp_alpha, is what "ramp_alpha_monotonic" means here.
        alphas = [v[_I["ramp_alpha"]] for _, v in ramp_bridge_diag]
        ramp_metrics["ramp_alpha_min"] = min(alphas)
        ramp_metrics["ramp_alpha_max"] = max(alphas)
        violations = sum(1 for i in range(1, len(alphas)) if alphas[i] < alphas[i - 1] - 1e-9)
        ramp_metrics["ramp_alpha_monotonicity_violations"] = violations
    if ramp_right_telem and ramp_bridge_diag:
        # Pair each right_telem sample with the nearest-in-time bridge_diag
        # sample for its GRAVITY ramp alpha -- NOT the legacy ramp_alpha/
        # coupling alias: since the 2026-09-09 staged transition, gravity
        # and coupling ramp independently, on different schedules, so aux_y
        # tracks configured*gravity_ramp_alpha, not configured*coupling_ramp_alpha.
        def nearest_gravity_alpha(t):
            best = min(ramp_bridge_diag, key=lambda s: abs(s[0] - t))
            return best[1][_I["gravity_ramp_alpha"]]
        residuals = []
        for t, row in ramp_right_telem:
            alpha = nearest_gravity_alpha(t)
            aux_y = row[2][1]
            residuals.append(abs(aux_y - configured_n * alpha))
        ramp_metrics["gravity_ramp_residual_mean_abs_n"] = statistics.fmean(residuals) if residuals else None
        ramp_metrics["gravity_ramp_residual_max_abs_n"] = max(residuals) if residuals else None
    if ramp_left_telem:
        applied_norms = [_norm(row[TELEM_IDX_APPLIED]) for _, row in ramp_left_telem]
        ramp_metrics["ramp_max_abs_applied_force_left_n"] = max(applied_norms)
        ramp_metrics["ramp_any_position_error_clamped_left"] = any(row[TELEM_IDX_POS_CLAMPED] for _, row in ramp_left_telem)
        ramp_metrics["ramp_any_primary_force_clamped_left"] = any(row[TELEM_IDX_PRIMARY_FORCE_CLAMPED] for _, row in ramp_left_telem)
        ramp_metrics["ramp_any_final_force_clamped_left"] = any(row[TELEM_IDX_FINAL_FORCE_CLAMPED] for _, row in ramp_left_telem)
        ramp_metrics["ramp_any_near_singular_left"] = any(row[TELEM_IDX_NEAR_SINGULAR] for _, row in ramp_left_telem)
        # Section 1: ramp position-error characterization -- proves a smooth
        # target ramp did not still create a large servo error.
        pos_err_raw_norms = [_norm(row[TELEM_IDX_POS_ERROR_RAW]) for _, row in ramp_left_telem]
        pos_err_clamped_norms = [_norm(row[TELEM_IDX_POS_ERROR_CLAMPED_VEC]) for _, row in ramp_left_telem]
        ramp_metrics["ramp_max_position_error_raw_left_m"] = max(pos_err_raw_norms)
        ramp_metrics["ramp_rms_position_error_raw_left_m"] = math.sqrt(statistics.fmean([n * n for n in pos_err_raw_norms]))
        ramp_metrics["ramp_max_position_error_clamped_left_m"] = max(pos_err_clamped_norms)
    if ramp_right_telem:
        applied_norms = [_norm(row[TELEM_IDX_APPLIED]) for _, row in ramp_right_telem]
        ramp_metrics["ramp_max_abs_applied_force_right_n"] = max(applied_norms)
        ramp_metrics["ramp_any_position_error_clamped_right"] = any(row[TELEM_IDX_POS_CLAMPED] for _, row in ramp_right_telem)
        ramp_metrics["ramp_any_primary_force_clamped_right"] = any(row[TELEM_IDX_PRIMARY_FORCE_CLAMPED] for _, row in ramp_right_telem)
        ramp_metrics["ramp_any_final_force_clamped_right"] = any(row[TELEM_IDX_FINAL_FORCE_CLAMPED] for _, row in ramp_right_telem)
        ramp_metrics["ramp_any_near_singular_right"] = any(row[TELEM_IDX_NEAR_SINGULAR] for _, row in ramp_right_telem)
        pos_err_raw_norms = [_norm(row[TELEM_IDX_POS_ERROR_RAW]) for _, row in ramp_right_telem]
        pos_err_clamped_norms = [_norm(row[TELEM_IDX_POS_ERROR_CLAMPED_VEC]) for _, row in ramp_right_telem]
        ramp_metrics["ramp_max_position_error_raw_right_m"] = max(pos_err_raw_norms)
        ramp_metrics["ramp_rms_position_error_raw_right_m"] = math.sqrt(statistics.fmean([n * n for n in pos_err_raw_norms]))
        ramp_metrics["ramp_max_position_error_clamped_right_m"] = max(pos_err_clamped_norms)

    # Section 2: target-discontinuity metric must cross the state boundary --
    # intra-RAMPING steps alone can miss the biggest jump (the boundary
    # itself). Compute all three categories explicitly.
    pre_arm_left_target = ([(t, v) for t, v in episode["left_target"] if t < ramp_start_t]
                          if ramp_start_t is not None else [])
    pre_arm_right_target = ([(t, v) for t, v in episode["right_target"] if t < ramp_start_t]
                           if ramp_start_t is not None else [])
    active_left_target = _segment(episode["left_target"], active_start_t, metric_active_end_t)
    active_right_target = _segment(episode["right_target"], active_start_t, metric_active_end_t)

    def boundary_step(before_series, after_series):
        if not before_series or not after_series:
            return None
        last_before = before_series[-1][1]
        first_after = after_series[0][1]
        return _norm([first_after[i] - last_before[i] for i in range(3)])

    def intra_max_step(series):
        if len(series) < 2:
            return None
        steps = [_norm([series[i][1][j] - series[i - 1][1][j] for j in range(3)])
                 for i in range(1, len(series))]
        return max(steps)

    ramp_metrics["prearm_to_ramp_target_step_left_m"] = boundary_step(pre_arm_left_target, ramp_left_target)
    ramp_metrics["prearm_to_ramp_target_step_right_m"] = boundary_step(pre_arm_right_target, ramp_right_target)
    ramp_metrics["max_target_step_left_m"] = intra_max_step(ramp_left_target)
    ramp_metrics["max_target_step_right_m"] = intra_max_step(ramp_right_target)
    ramp_metrics["ramp_to_active_target_step_left_m"] = boundary_step(ramp_left_target, active_left_target)
    ramp_metrics["ramp_to_active_target_step_right_m"] = boundary_step(ramp_right_target, active_right_target)

    metrics["ramp_validation"] = ramp_metrics

    # --- Section 8: post-disarm settling verification ---
    post_disarm_metrics = {}
    if disarm_t is not None:
        settle_left_force = _segment(episode["left_force"], disarm_t, disarm_t + settle_window_s)
        settle_right_force = _segment(episode["right_force"], disarm_t, disarm_t + settle_window_s)
        settle_left_telem = _segment(episode["left_telem"], disarm_t, disarm_t + settle_window_s)
        settle_right_telem = _segment(episode["right_telem"], disarm_t, disarm_t + settle_window_s)
        post_disarm_metrics["n_settle_samples_left_force"] = len(settle_left_force)
        post_disarm_metrics["n_settle_samples_right_force"] = len(settle_right_force)
        if settle_left_force:
            post_disarm_metrics["max_abs_applied_force_left_n"] = max(_norm(v) for _, v in settle_left_force)
        if settle_right_force:
            post_disarm_metrics["max_abs_applied_force_right_n"] = max(_norm(v) for _, v in settle_right_force)
        if settle_left_telem:
            post_disarm_metrics["max_abs_spring_force_left_n"] = max(_norm(row[0]) for _, row in settle_left_telem)
            post_disarm_metrics["max_abs_auxiliary_force_left_n"] = max(_norm(row[2]) for _, row in settle_left_telem)
        if settle_right_telem:
            post_disarm_metrics["max_abs_spring_force_right_n"] = max(_norm(row[0]) for _, row in settle_right_telem)
            post_disarm_metrics["max_abs_auxiliary_force_right_n"] = max(_norm(row[2]) for _, row in settle_right_telem)
    metrics["post_disarm_settling"] = post_disarm_metrics

    # --- Section 7: automatic recorder ACQUISITION validity block. This is
    # ONLY about whether the recording itself is usable evidence -- it does
    # NOT infer the scientific 0.55-vs-0.70 conclusion. ---
    pv = {}
    pv["required_topics_complete"] = events.get("required_topics_complete", True)
    pv["target_topics_complete"] = events.get("target_topics_complete", True)
    pv["ramp_samples_present"] = ramp_metrics.get("n_ramp_samples_left_telem", 0) > 0 and \
        ramp_metrics.get("n_ramp_samples_right_telem", 0) > 0
    pv["active_samples_present"] = len(active_left_pose) > 0 and len(active_right_pose) > 0
    settle_left_n = post_disarm_metrics.get("n_settle_samples_left_force", 0)
    settle_right_n = post_disarm_metrics.get("n_settle_samples_right_force", 0)
    pv["post_disarm_samples_present"] = disarm_t is not None and settle_left_n > 0 and settle_right_n > 0
    pv["active_window_violation_count"] = len(events.get("active_window_violations", []))
    pv["event_order_valid"] = (
        ramp_start_t is not None and active_start_t is not None and disarm_t is not None
        and ramp_start_t <= active_start_t <= metric_active_end_t <= disarm_t + 1.0
        # +1.0s slack: disarm_t is latched on the callback AFTER the metric
        # window closes, at whatever cadence the bridge happens to publish,
        # not synchronized to metric_active_end_t to the microsecond.
    )
    pv["exact_active_duration_valid"] = events["exact_active_duration_valid"]
    pv["ramp_alpha_monotonic"] = ramp_metrics.get("ramp_alpha_monotonicity_violations", 0) == 0
    grr = ramp_metrics.get("gravity_ramp_residual_max_abs_n")
    pv["gravity_ramp_valid"] = grr is not None and grr < 0.05
    pv["clamp_free_ramp"] = not any(ramp_metrics.get(k, False) for k in (
        "ramp_any_position_error_clamped_left", "ramp_any_position_error_clamped_right",
        "ramp_any_primary_force_clamped_left", "ramp_any_primary_force_clamped_right",
        "ramp_any_final_force_clamped_left", "ramp_any_final_force_clamped_right",
    ))
    clean_post_disarm = True
    reasons = []
    for key, label in (
        ("max_abs_applied_force_left_n", "left applied_force"),
        ("max_abs_applied_force_right_n", "right applied_force"),
        ("max_abs_spring_force_left_n", "left spring_force"),
        ("max_abs_spring_force_right_n", "right spring_force"),
        ("max_abs_auxiliary_force_left_n", "left auxiliary_force"),
        ("max_abs_auxiliary_force_right_n", "right auxiliary_force"),
    ):
        v = post_disarm_metrics.get(key)
        if v is not None and v > 0.05:
            clean_post_disarm = False
            reasons.append(f"{label}={v:.4f}N > 0.05N during post-disarm settling")
    pv["clean_post_disarm"] = clean_post_disarm

    hard_criteria = ("required_topics_complete", "target_topics_complete", "ramp_samples_present",
                     "active_samples_present", "post_disarm_samples_present", "event_order_valid",
                     "exact_active_duration_valid", "ramp_alpha_monotonic", "gravity_ramp_valid",
                     "clamp_free_ramp", "clean_post_disarm")
    failing = [k for k in hard_criteria if not pv.get(k, False)]
    if pv["active_window_violation_count"] > 0:
        failing.append("active_window_violation_count>0")
    pv["all_hard_criteria_pass"] = len(failing) == 0
    metrics["protocol_validity"] = pv
    if failing:
        metrics["aggregate_include"] = False
        metrics["aggregate_exclude_reason"] = f"recorder acquisition validity failed: {failing}" + (
            f" ({'; '.join(reasons)})" if reasons else "")
    else:
        # Acquisition validity does NOT imply scientific acceptance -- this
        # only means the recording itself is trustworthy evidence. The
        # operator/user makes the final scientific inclusion call.
        metrics["aggregate_include"] = None
        metrics["aggregate_exclude_reason"] = None

    return metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session-dir", required=True)
    ap.add_argument("--condition", required=True)
    ap.add_argument("--trial-index", required=True, type=int)
    ap.add_argument("--duration-s", type=float, default=20.0)
    ap.add_argument("--arm-wait-timeout-s", type=float, default=600.0)
    ap.add_argument("--disarm-wait-timeout-s", type=float, default=120.0)
    ap.add_argument("--settle-window-s", type=float, default=2.0)
    ap.add_argument("--topic-prefix-left", default=None,
                     help="Override, e.g. /c2_master_ab12 instead of /touch_left "
                          "-- sim/integration testing only, real trials use the default.")
    ap.add_argument("--topic-prefix-right", default=None)
    ap.add_argument("--bridge-diag-topic", default=None)
    args = ap.parse_args()

    out_dir = Path(args.session_dir) / "analysis" / f"trial_{args.condition}_{args.trial_index}"
    out_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    topics_override = {}
    if args.topic_prefix_left:
        topics_override.update({
            "left_pose": f"{args.topic_prefix_left}/pose", "left_target": f"{args.topic_prefix_left}/target_pose",
            "left_force": f"{args.topic_prefix_left}/applied_force",
            "left_telem": f"{args.topic_prefix_left}/cartesian_controller_telemetry",
            "left_dob": f"{args.topic_prefix_left}/dob_diagnostics",
        })
    if args.topic_prefix_right:
        topics_override.update({
            "right_pose": f"{args.topic_prefix_right}/pose", "right_target": f"{args.topic_prefix_right}/target_pose",
            "right_force": f"{args.topic_prefix_right}/applied_force",
            "right_telem": f"{args.topic_prefix_right}/cartesian_controller_telemetry",
            "right_dob": f"{args.topic_prefix_right}/dob_diagnostics",
        })
    if args.bridge_diag_topic:
        topics_override["bridge_diag"] = args.bridge_diag_topic
    node = TrialRecorder(args.duration_s, topics=topics_override)
    print("[trial] subscribed, spinning 1s to confirm topics are live...", flush=True)
    end = _now() + 1.0
    while _now() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    missing = [k for k in REQUIRED_TOPICS if node.last_seen[k] is None]
    if missing:
        print(f"[trial][ERROR] no data seen yet on required topics: {missing} -- aborting.", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    dob_available = node.last_seen["left_dob"] is not None and node.last_seen["right_dob"] is not None
    print(f"[trial] all required topics confirmed live. DOB_AVAILABLE={dob_available}", flush=True)

    bd0 = node.last_seen.get("bridge_diag")
    if bd0 is None or bd0[0] != "READY":
        print(f"[trial][ERROR] bridge_diag not READY at recorder start (got {bd0}), aborting.", flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    node.prev_bridge_state = "READY"
    node.start_episode()
    print(f"[trial] EPISODE CAPTURE started (READY, pre-arm) "
          f"condition={args.condition} trial={args.trial_index} "
          f"t={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}", flush=True)
    print(f"[trial] READY_FOR_ARM. Arm whenever ready "
          f"(waiting up to {'indefinitely' if args.arm_wait_timeout_s <= 0 else f'{args.arm_wait_timeout_s:.0f}s'}).",
          flush=True)

    arm_deadline = None if args.arm_wait_timeout_s <= 0 else _now() + args.arm_wait_timeout_s
    while node.ramp_start_t is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if arm_deadline is not None and _now() > arm_deadline:
            print(f"[trial][ERROR] no READY->RAMPING transition within {args.arm_wait_timeout_s:.0f}s, aborting.",
                  flush=True)
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
    print(f"[trial] RAMPING detected (episode capture already running -- ramp telemetry preserved) "
          f"t={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(node.ramp_start_t))}", flush=True)

    active_timeout = _now() + 10.0
    while node.active_start_t is None and _now() < active_timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.active_start_t is None:
        bd = node.last_seen.get("bridge_diag")
        print(f"[trial][ERROR] never confirmed ACTIVE within 10s of RAMPING (last bridge_diag={bd}), aborting.",
              flush=True)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print(f"[trial] ACTIVE_START (callback-latched) t={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(node.active_start_t))}",
          flush=True)

    end = node.active_start_t + args.duration_s
    while _now() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.active_end_t = _now()
    if node.active_window_violations:
        print(f"[trial][WARN] {len(node.active_window_violations)} ACTIVE-window integrity violation(s) "
              f"detected during the metric window -- e.g. {node.active_window_violations[0]}", flush=True)
    print(f"[trial] ACTIVE_END. Disarm now (ros2 param set /touch_master_slave_bridge arm_teleop false).",
          flush=True)

    disarm_deadline = _now() + args.disarm_wait_timeout_s
    while node.disarm_transition_t is None and _now() < disarm_deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.disarm_transition_t is None:
        print(f"[trial][WARN] no ACTIVE->READY transition observed within {args.disarm_wait_timeout_s:.0f}s "
              f"of ACTIVE_END -- disarm_transition_t will be null.", flush=True)
    else:
        print(f"[trial] DISARM confirmed t={time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime(node.disarm_transition_t))}. "
              f"Continuing episode capture for a {args.settle_window_s:.1f}s post-disarm settling window.", flush=True)
        settle_end = node.disarm_transition_t + args.settle_window_s
        while _now() < settle_end:
            rclpy.spin_once(node, timeout_sec=0.05)

    events = {
        "arm_transition_t": node.arm_transition_t, "ramp_start_t": node.ramp_start_t,
        "active_start_t": node.active_start_t, "active_end_t": node.active_end_t,
        "disarm_transition_t": node.disarm_transition_t,
        "active_window_violations": node.active_window_violations,
        # Trivially True here -- the recorder hard-aborts before EPISODE
        # CAPTURE even starts if any REQUIRED_TOPICS entry (which now
        # includes left_target/right_target, per Section 3) was never seen.
        # Recorded explicitly so protocol_validity documents the invariant
        # rather than assuming it silently.
        "required_topics_complete": True,
        "target_topics_complete": True,
    }

    phase_fn = _phase_fn(node.episode["bridge_diag"])
    headers = {
        "left_pose": ["x", "y", "z"], "right_pose": ["x", "y", "z"],
        "left_target": ["x", "y", "z"], "right_target": ["x", "y", "z"],
        "left_force": ["fx", "fy", "fz"], "right_force": ["fx", "fy", "fz"],
        "left_telem": ["spring_x", "spring_y", "spring_z", "damp_x", "damp_y", "damp_z",
                       "aux_x", "aux_y", "aux_z", "applied_x", "applied_y", "applied_z",
                       "pos_clamped", "primary_force_clamped", "final_force_clamped", "near_singular",
                       "pos_error_raw_x", "pos_error_raw_y", "pos_error_raw_z",
                       "pos_error_clamped_x", "pos_error_clamped_y", "pos_error_clamped_z"],
        "right_telem": ["spring_x", "spring_y", "spring_z", "damp_x", "damp_y", "damp_z",
                        "aux_x", "aux_y", "aux_z", "applied_x", "applied_y", "applied_z",
                        "pos_clamped", "primary_force_clamped", "final_force_clamped", "near_singular",
                        "pos_error_raw_x", "pos_error_raw_y", "pos_error_raw_z",
                        "pos_error_clamped_x", "pos_error_clamped_y", "pos_error_clamped_z"],
        "bridge_diag": [
            "bridge_state", "gravity_ramp_alpha", "coupling_ramp_alpha", "ramp_alpha", "follow_error_m",
            "master_accepted_force_margin_n", "slave_accepted_force_margin_n",
            "master_accepted_position_margin_m", "slave_accepted_position_margin_m",
            "master_candidate_force_margin_n", "slave_candidate_force_margin_n",
            "master_candidate_position_margin_m", "slave_candidate_position_margin_m",
            "master_predicted_force_x_n", "master_predicted_force_y_n", "master_predicted_force_z_n",
            "slave_predicted_force_x_n", "slave_predicted_force_y_n", "slave_predicted_force_z_n",
            "telemetry_fresh_left", "telemetry_fresh_right",
            "planner_parameter_match_left", "planner_parameter_match_right",
            "coupling_ramp_held", "coupling_ramp_stall_time_s",
            "settle_master_speed_mps", "settle_slave_speed_mps",
            "settle_master_pose_drift_m", "settle_slave_pose_drift_m",
            "settle_quiet_window_accumulated_s",
        ],
        "left_dob": ["tau_dis_hat_j1_nm", "tau_dis_hat_j2_nm", "tau_dis_hat_j3_nm"],
        "right_dob": ["tau_dis_hat_j1_nm", "tau_dis_hat_j2_nm", "tau_dis_hat_j3_nm"],
    }
    for key, hdr in headers.items():
        if node.episode[key]:
            _write_csv(out_dir / f"{key}.csv", node.episode[key], hdr, phase_fn)

    metrics = compute_metrics(node.episode, events, args.duration_s, args.condition, args.settle_window_s)
    metrics["trial_index"] = args.trial_index
    metrics["dob_available"] = dob_available
    metrics["timestamp_handling"] = (
        "left/right topics are NOT hardware-synchronized; compute_metrics() pairs each "
        "left_pose sample with the nearest-in-time right_pose sample -- see pairing_audit "
        "(mean/p95/max |dt|, computed for this trial) and effective_hz (measured, not assumed)."
    )
    with open(out_dir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2, default=str)
    print(f"[trial] wrote {out_dir}", flush=True)
    print(json.dumps(metrics, indent=2, default=str))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
