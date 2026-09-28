#!/usr/bin/env python3
"""Demonstrates run_gravcomp_trial.py's v4 state-transition and metrics
logic (2026-09-09, third protocol-gap follow-up) WITHOUT waiting real
wall-clock time and WITHOUT a full rclpy/sim bridge process -- exercises
TrialRecorder._on_bridge_diag()/_rec() and compute_metrics() directly (the
exact code the real node uses, imported unmodified) with a monkeypatched
clock, driving a full synthetic episode through:

    READY (sustained > 60s of simulated time, proving no premature death)
    -> RAMPING (ramp telemetry preserved, incl. position_error_raw_m --
                v3 added ramp telemetry capture but not this field; v4 adds it)
    -> ACTIVE, ramp_alpha climbing 0->1 (active_start_t latched INSIDE the
                callback only at >=0.999)
    -> exactly 20.000s ACTIVE metric window (defined as active_start_t +
                duration_s, NOT the observed loop-exit wall time)
    -> READY again (disarm_transition_t captured; post-disarm settling
                samples continue to be captured)

and then runs the real compute_metrics() over that synthetic episode,
asserting:
  - ramp position-error stats are computed (max/RMS raw, max clamped)
  - the target step crosses all three boundaries (PRE_ARM->RAMPING,
    intra-RAMPING, RAMPING->ACTIVE), not just intra-RAMPING
  - the ACTIVE metric window duration is exactly requested duration_s
  - the automatic protocol_validity acquisition block passes cleanly in
    this deliberately-clean synthetic case
"""
from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import run_gravcomp_trial as rgt  # noqa: E402


class FakeDiag:
    """2026-09-09 Part 5 update: expanded to carry every BridgeDiagnostics
    field _bridge_diag_row() now reads (see run_gravcomp_trial.py's
    BRIDGE_DIAG_FIELDS) -- defaults are safe/neutral production-realistic
    values (comfortably positive margins, fresh telemetry, matching
    params) so existing call sites (FakeDiag("READY", 0.0), etc.) keep
    working unchanged; only tests that specifically exercise a margin/
    fail-safe field need to override it."""
    def __init__(self, bridge_state, ramp_alpha, follow_error_m=0.0, gravity_ramp_alpha=None,
                coupling_ramp_alpha=None,
                master_accepted_force_budget_margin_n=1.0, slave_accepted_force_budget_margin_n=1.0,
                master_accepted_position_error_margin_m=0.02, slave_accepted_position_error_margin_m=0.02,
                master_candidate_force_budget_margin_n=1.0, slave_candidate_force_budget_margin_n=1.0,
                master_candidate_position_error_margin_m=0.02, slave_candidate_position_error_margin_m=0.02,
                master_predicted_force_n=(0.0, 0.0, 0.0), slave_predicted_force_n=(0.0, 0.0, 0.0),
                force_budget_telemetry_fresh_left=True, force_budget_telemetry_fresh_right=True,
                planner_parameter_match_left=True, planner_parameter_match_right=True,
                coupling_ramp_held=False, coupling_ramp_stall_time_s=0.0,
                gravity_settle_master_speed_mps=0.0, gravity_settle_slave_speed_mps=0.0,
                gravity_settle_master_pose_drift_m=0.0, gravity_settle_slave_pose_drift_m=0.0,
                gravity_settle_quiet_window_accumulated_s=0.0):
        self.bridge_state = bridge_state
        self.ramp_alpha = ramp_alpha
        self.follow_error_m = follow_error_m
        # gravity_ramp_alpha defaults to ramp_alpha itself -- most existing
        # call sites use a single alpha to mean "how far through arming",
        # which was true pre-staging; ACTIVE always implies both are 1.0.
        self.gravity_ramp_alpha = ramp_alpha if gravity_ramp_alpha is None else gravity_ramp_alpha
        self.coupling_ramp_alpha = ramp_alpha if coupling_ramp_alpha is None else coupling_ramp_alpha
        self.master_accepted_force_budget_margin_n = master_accepted_force_budget_margin_n
        self.slave_accepted_force_budget_margin_n = slave_accepted_force_budget_margin_n
        self.master_accepted_position_error_margin_m = master_accepted_position_error_margin_m
        self.slave_accepted_position_error_margin_m = slave_accepted_position_error_margin_m
        self.master_candidate_force_budget_margin_n = master_candidate_force_budget_margin_n
        self.slave_candidate_force_budget_margin_n = slave_candidate_force_budget_margin_n
        self.master_candidate_position_error_margin_m = master_candidate_position_error_margin_m
        self.slave_candidate_position_error_margin_m = slave_candidate_position_error_margin_m
        self.master_predicted_force_n = master_predicted_force_n
        self.slave_predicted_force_n = slave_predicted_force_n
        self.force_budget_telemetry_fresh_left = force_budget_telemetry_fresh_left
        self.force_budget_telemetry_fresh_right = force_budget_telemetry_fresh_right
        self.planner_parameter_match_left = planner_parameter_match_left
        self.planner_parameter_match_right = planner_parameter_match_right
        self.coupling_ramp_held = coupling_ramp_held
        self.coupling_ramp_stall_time_s = coupling_ramp_stall_time_s
        self.gravity_settle_master_speed_mps = gravity_settle_master_speed_mps
        self.gravity_settle_slave_speed_mps = gravity_settle_slave_speed_mps
        self.gravity_settle_master_pose_drift_m = gravity_settle_master_pose_drift_m
        self.gravity_settle_slave_pose_drift_m = gravity_settle_slave_pose_drift_m
        self.gravity_settle_quiet_window_accumulated_s = gravity_settle_quiet_window_accumulated_s


def make_recorder(duration_s=20.0):
    node = rgt.TrialRecorder.__new__(rgt.TrialRecorder)
    node.duration_s = duration_s
    all_keys = rgt.REQUIRED_TOPICS + rgt.OPTIONAL_TOPICS
    node.episode = {k: [] for k in all_keys}
    node.last_seen = {k: None for k in all_keys}
    node.episode_recording = False
    node.prev_bridge_state = None
    node.ever_armed = False
    node.arm_transition_t = None
    node.ramp_start_t = None
    node.active_start_t = None
    node.active_end_t = None
    node.disarm_transition_t = None
    node.active_window_violations = []
    return node


def _telem(spring=(0, 0, 0), aux=(0, 0, 0), applied=(0, 0, 0),
          pos_clamped=False, primary_clamped=False, final_clamped=False, near_sing=False,
          pos_err_raw=(0, 0, 0), pos_err_clamped=(0, 0, 0)):
    return (spring, (0.0, 0.0, 0.0), aux, applied, pos_clamped, primary_clamped,
            final_clamped, near_sing, pos_err_raw, pos_err_clamped)


def test_full_cycle_with_ramp_position_error_and_boundary_steps():
    t = [1000.0]

    def fake_now():
        return t[0]

    with mock.patch.object(rgt, "_now", fake_now):
        node = make_recorder(duration_s=20.0)

        node._on_bridge_diag(FakeDiag("READY", 0.0))
        node.prev_bridge_state = "READY"
        node.episode_recording = True

        # PRE_ARM: target sits at a fixed pose, 80s simulated (>60s, proving
        # no premature recorder death), then one final sample right before arm.
        for i in range(80):
            t[0] += 1.0
            node._on_bridge_diag(FakeDiag("READY", 0.0))
            node._rec("left_target", (0.10, 0.10, 0.10))
            node._rec("right_target", (0.20, 0.20, 0.20))
        assert t[0] - 1000.0 > 60.0

        # ARM -> RAMPING. Advance the clock a hair first so this transition
        # does not share an exact timestamp with the last PRE_ARM sample --
        # _segment()'s [lo, hi) convention would otherwise classify a
        # same-instant sample as RAMPING (correct, consistent behavior; two
        # ordinary ROS callbacks essentially never collide to the float,
        # this is purely to keep the synthetic fixture unambiguous).
        t[0] += 0.01
        node._on_bridge_diag(FakeDiag("RAMPING", 0.0))
        first_ramp_t = node.ramp_start_t
        assert first_ramp_t == t[0]

        # RAMPING: target STEPS to a new value at the very first ramp sample
        # (the boundary this fix is specifically about), then holds there.
        # Also record right_telem with a nonzero, decaying position error
        # (a plausible physical ramp-in transient) and matching aux_y ramp.
        ramp_targets_left = [0.13, 0.13, 0.13]
        ramp_targets_right = [0.24, 0.24, 0.24]
        pos_err_seq = [0.010, 0.007, 0.004, 0.002, 0.001, 0.0005]
        for alpha, err in zip((0.1, 0.3, 0.5, 0.7, 0.9, 0.998), pos_err_seq):
            t[0] += 0.15
            node._on_bridge_diag(FakeDiag("RAMPING", alpha))
            node._rec("left_target", tuple(ramp_targets_left))
            node._rec("right_target", tuple(ramp_targets_right))
            node._rec("right_telem", _telem(
                aux=(0.0, 0.55 * alpha, 0.0), applied=(0.0, 0.55 * alpha, 0.0),
                pos_err_raw=(0.0, err, 0.0), pos_err_clamped=(0.0, err, 0.0)))
            node._rec("left_telem", _telem(pos_err_raw=(0.0, err * 0.5, 0.0)))

        t[0] += 0.02
        node._on_bridge_diag(FakeDiag("ACTIVE", 1.0))
        active_start_t = node.active_start_t
        assert active_start_t == t[0]

        # ACTIVE: target steps AGAIN at the RAMPING->ACTIVE boundary (the
        # third category), then holds through the whole 20.0s metric window.
        active_target_left = [0.20, 0.20, 0.20]
        active_target_right = [0.31, 0.31, 0.31]
        for i in range(200):
            t[0] += 0.1
            node._on_bridge_diag(FakeDiag("ACTIVE", 1.0))
            node._rec("left_pose", (0.01, 0.02, 0.0))
            node._rec("right_pose", (0.01, 0.02, 0.0))
            node._rec("left_target", tuple(active_target_left))
            node._rec("right_target", tuple(active_target_right))
            node._rec("right_telem", _telem(aux=(0.0, 0.55, 0.0), applied=(0.0, 0.55, 0.0)))
            node._rec("left_telem", _telem())
        node.active_end_t = t[0]  # observed loop-exit time -- deliberately later than active_start_t+20 by a hair
        assert abs((node.active_end_t - active_start_t) - 20.0) < 0.05

        # DISARM -> READY, then post-disarm settling samples.
        t[0] += 0.3
        node._on_bridge_diag(FakeDiag("READY", 0.0))
        disarm_t = node.disarm_transition_t
        for i in range(20):
            t[0] += 0.1
            node._on_bridge_diag(FakeDiag("READY", 0.0))
            node._rec("left_force", (0.0, 0.0, 0.0))
            node._rec("right_force", (0.0, 0.0, 0.0))
            node._rec("left_telem", _telem())
            node._rec("right_telem", _telem())

        events = {
            "arm_transition_t": node.arm_transition_t, "ramp_start_t": node.ramp_start_t,
            "active_start_t": node.active_start_t, "active_end_t": node.active_end_t,
            "disarm_transition_t": node.disarm_transition_t,
            "active_window_violations": node.active_window_violations,
            "required_topics_complete": True, "target_topics_complete": True,
        }
        metrics = rgt.compute_metrics(node.episode, events, 20.0, "0.55", 2.0)
        assert "error" not in metrics, metrics.get("error")

        # --- Section 6: exact 20.000s metric window, independent of the
        # observed (slightly-later) loop-exit wall time. ---
        ev = metrics["event_timestamps"]
        assert abs(ev["metric_active_end_t"] - active_start_t - 20.0) < 1e-9
        assert ev["exact_active_duration_valid"] is True
        assert ev["active_capture_stop_observed_t"] == node.active_end_t
        print(f"[demo] metric window exactly 20.000s (active_capture_stop_observed_t="
              f"{ev['active_capture_stop_observed_t']:.3f}, metric_active_end_t="
              f"{ev['metric_active_end_t']:.3f}, these may legitimately differ slightly). PASS")

        # --- Section 1: ramp position-error stats present and correct. ---
        rv = metrics["ramp_validation"]
        assert abs(rv["ramp_max_position_error_raw_right_m"] - max(pos_err_seq)) < 1e-9
        assert rv["ramp_rms_position_error_raw_right_m"] > 0
        assert rv["ramp_max_position_error_clamped_right_m"] > 0
        print(f"[demo] ramp position-error captured: max_raw_right={rv['ramp_max_position_error_raw_right_m']*1000:.2f}mm "
              f"rms_raw_right={rv['ramp_rms_position_error_raw_right_m']*1000:.2f}mm. PASS")

        # --- Section 2: three-category target-discontinuity metric. ---
        assert rv["prearm_to_ramp_target_step_left_m"] is not None and rv["prearm_to_ramp_target_step_left_m"] > 0.02
        assert rv["prearm_to_ramp_target_step_right_m"] is not None and rv["prearm_to_ramp_target_step_right_m"] > 0.02
        assert rv["max_target_step_left_m"] is None or rv["max_target_step_left_m"] < 1e-9  # target held constant intra-ramp
        assert rv["ramp_to_active_target_step_left_m"] is not None and rv["ramp_to_active_target_step_left_m"] > 0.05
        assert rv["ramp_to_active_target_step_right_m"] is not None and rv["ramp_to_active_target_step_right_m"] > 0.05
        print(f"[demo] three boundary-step categories measured: "
              f"prearm->ramp={rv['prearm_to_ramp_target_step_left_m']*1000:.1f}mm, "
              f"intra-ramp={rv['max_target_step_left_m']}, "
              f"ramp->active={rv['ramp_to_active_target_step_left_m']*1000:.1f}mm. PASS")

        # --- Section 7: automatic protocol_validity acquisition block,
        # clean synthetic case must pass all hard criteria. ---
        pv = metrics["protocol_validity"]
        assert pv["ramp_samples_present"] is True
        assert pv["active_samples_present"] is True
        assert pv["post_disarm_samples_present"] is True
        assert pv["active_window_violation_count"] == 0
        assert pv["event_order_valid"] is True
        assert pv["exact_active_duration_valid"] is True
        assert pv["ramp_alpha_monotonic"] is True
        assert pv["gravity_ramp_valid"] is True
        assert pv["clamp_free_ramp"] is True
        assert pv["clean_post_disarm"] is True
        assert pv["all_hard_criteria_pass"] is True
        assert metrics["aggregate_include"] is None, (
            "acquisition-valid trials get aggregate_include=None (acquisition OK, "
            "scientific inclusion is a separate human call), not auto-True")
        print("[demo] protocol_validity: all hard acquisition criteria pass on the clean synthetic case. PASS\n")


def test_active_window_violation_is_flagged_and_fails_protocol_validity():
    t = [2000.0]

    def fake_now():
        return t[0]

    with mock.patch.object(rgt, "_now", fake_now):
        node = make_recorder(duration_s=5.0)
        node._on_bridge_diag(FakeDiag("READY", 0.0))
        node.prev_bridge_state = "READY"
        node.episode_recording = True
        node._on_bridge_diag(FakeDiag("RAMPING", 0.0))
        t[0] += 1.0
        node._on_bridge_diag(FakeDiag("ACTIVE", 1.0))
        assert node.active_start_t is not None

        t[0] += 1.0
        node._on_bridge_diag(FakeDiag("ACTIVE", 1.0))
        assert node.active_window_violations == []

        t[0] += 1.0
        node._on_bridge_diag(FakeDiag("FAULT", 0.0))
        assert len(node.active_window_violations) == 1
        assert node.active_window_violations[0][1] == "FAULT"
        print(f"[demo] mid-window state exit correctly flagged: {node.active_window_violations[0]}. PASS")

        # A violated window must fail protocol_validity's hard-criteria gate.
        # (Still within the 5s window at this point -- an immediate READY
        # right after FAULT, with no further time elapsed, is ALSO correctly
        # flagged as a second violation: it is still not ACTIVE. Assert
        # >=1, not an exact count, since that count legitimately depends on
        # how many non-ACTIVE diag messages land before the window closes.)
        node._on_bridge_diag(FakeDiag("READY", 0.0))
        events = {
            "arm_transition_t": node.arm_transition_t, "ramp_start_t": node.ramp_start_t,
            "active_start_t": node.active_start_t, "active_end_t": t[0],
            "disarm_transition_t": node.disarm_transition_t,
            "active_window_violations": node.active_window_violations,
            "required_topics_complete": True, "target_topics_complete": True,
        }
        # Minimal telem/target so compute_metrics doesn't early-return on missing pose data.
        node.episode["left_pose"].append((node.active_start_t + 0.1, (0, 0, 0)))
        node.episode["right_pose"].append((node.active_start_t + 0.1, (0, 0, 0)))
        metrics = rgt.compute_metrics(node.episode, events, 5.0, "0.55", 2.0)
        assert metrics["protocol_validity"]["active_window_violation_count"] >= 1
        assert metrics["protocol_validity"]["all_hard_criteria_pass"] is False
        assert metrics["aggregate_include"] is False
        print(f"[demo] a violated ACTIVE window ({metrics['protocol_validity']['active_window_violation_count']} "
              f"violation(s)) forces aggregate_include=False. PASS\n")


if __name__ == "__main__":
    test_full_cycle_with_ramp_position_error_and_boundary_steps()
    test_active_window_violation_is_flagged_and_fails_protocol_validity()
    print("ALL TESTS PASSED")
