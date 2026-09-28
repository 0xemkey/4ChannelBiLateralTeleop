"""Deterministic 117mm Y-axis live stress test for the force-aware coupling
ramp (2026-09-09, Parts 5+6 of the validation-infrastructure task).

This is a REPRODUCTION OF THE REAL Y-DOMINANT FAILURE CONDITION, not an
axis substitution: the initial condition is a held master and a
gravity-sagged slave separated by ~117mm in Y -- the same physical
configuration the real 0.70N Trial 1 had -- created deterministically by
force_budget_sim_fixture (measured repeatable to ~0.01mm across independent
launches by test_sim_initial_condition_repeatability.py).

Runs the REAL bridge_master_slave.py, REAL touch_slave_sim controllers, the
REAL staged state machine and the REAL bilateral force-budget planner, and
MEASURES rather than merely asserting (Part 6).
"""
from __future__ import annotations

import math
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402
from force_budget_sim_fixture import (  # noqa: E402
    MASTER_PARAMS, SLAVE_PARAMS, SETTLE_S, master_sim_env, slave_sim_env, measured_gap,
)

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)
sys.path.insert(0, str(BRIDGE_SCRIPT.parent))
from coupling_force_budget import predict_force_vector, _norm  # noqa: E402

GRAVITY_N = 0.70
REQUESTED_GAP_M = 0.117


def bridge_args(master, slave, calib_path):
    return [str(BRIDGE_SCRIPT),
            "--left-pose-topic", master.topic("pose"),
            "--right-pose-topic", slave.topic("pose"),
            "--right-target-topic", slave.topic("target_pose"),
            "--right-enable-topic", slave.topic("enable"),
            "--left-target-topic", master.topic("target_pose"),
            "--left-enable-topic", master.topic("enable"),
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--slave-force-topic", slave.topic("force_cmd"),
            "--enable-c2", "true",
            "--left-max-offset-m", "0.08",
            "--calibration-file", str(calib_path),
            "--axis-map", "x,y,z",
            "--absolute-axes", "x,y,z", "--absolute-trim-m", "0.0",
            "--slave-gravity-comp-n", str(GRAVITY_N), "--slave-gravity-axis", "1",
            "--master-stiffness-n-per-m", "15.0", "--master-damping-n-s-per-m", "0.10",
            "--master-max-force-n", "2.0", "--master-max-position-error-m", "0.03",
            "--stiffness-n-per-m", "35.0", "--slave-damping-n-s-per-m", "0.20",
            "--slave-max-force-n", "1.5", "--slave-max-position-error-m", "0.04",
            "--force-budget-reserve-n", "0.10",
            "--rate-hz", "100", "--wait-timeout-s", "15",
            ]


class ArmStats:
    """Accumulates the Part 6 per-arm metrics from telemetry samples."""

    def __init__(self, label):
        self.label = label
        self.peak_raw_err = 0.0
        self.sq_err_sum = 0.0
        self.n_err = 0
        self.peak_spring = 0.0
        self.peak_damping = 0.0
        self.peak_pre_primary = 0.0
        self.peak_applied = 0.0
        self.pos_clamp_samples = 0
        self.force_clamp_samples = 0
        self.final_clamp_samples = 0
        self.n = 0
        self.pred_errors = []

    def add(self, t, coupling_phase: bool):
        self.n += 1
        if t.position_error_clamped:
            self.pos_clamp_samples += 1
        if t.primary_force_clamped:
            self.force_clamp_samples += 1
        if t.final_force_clamped:
            self.final_clamp_samples += 1
        # Predictor equivalence: recompute from the controller's OWN reported
        # inputs and compare to its own pre-clamp force.
        pred = predict_force_vector(
            list(t.position_m), list(t.velocity_mps), list(t.target_position_m),
            t.stiffness_n_per_m, t.damping_n_s_per_m, list(t.auxiliary_force_n),
            max_position_error_m=t.max_position_error_m)
        self.pred_errors.append(_norm([pred[i] - t.force_before_primary_clamp_n[i] for i in range(3)]))
        if coupling_phase:
            e = _norm(list(t.position_error_raw_m))
            self.peak_raw_err = max(self.peak_raw_err, e)
            self.sq_err_sum += e * e
            self.n_err += 1
            self.peak_spring = max(self.peak_spring, _norm(list(t.spring_force_n)))
            self.peak_damping = max(self.peak_damping, _norm(list(t.damping_force_n)))
            self.peak_pre_primary = max(self.peak_pre_primary, _norm(list(t.force_before_primary_clamp_n)))
            self.peak_applied = max(self.peak_applied, _norm(list(t.applied_force_n)))

    @property
    def rms_err(self):
        return math.sqrt(self.sq_err_sum / self.n_err) if self.n_err else 0.0

    def pred_stats(self):
        if not self.pred_errors:
            return None
        s = sorted(self.pred_errors)
        n = len(s)
        return {"mean": sum(s) / n, "rms": math.sqrt(sum(e * e for e in s) / n),
                "p95": s[min(n - 1, int(math.ceil(0.95 * n)) - 1)], "max": s[-1], "n": n}

    def report(self):
        p = self.pred_stats()
        print(f"  {self.label}:")
        print(f"    peak raw position error   = {self.peak_raw_err*1000:8.3f} mm")
        print(f"    RMS transition error      = {self.rms_err*1000:8.3f} mm")
        print(f"    peak spring force         = {self.peak_spring:8.4f} N")
        print(f"    peak damping force        = {self.peak_damping:8.4f} N")
        print(f"    peak pre-primary force    = {self.peak_pre_primary:8.4f} N")
        print(f"    peak applied force        = {self.peak_applied:8.4f} N")
        print(f"    position-clamp samples    = {self.pos_clamp_samples}")
        print(f"    primary-force-clamp smpls = {self.force_clamp_samples}")
        print(f"    final-force-clamp samples = {self.final_clamp_samples}")
        print(f"    predictor err mean/rms/p95/max = {p['mean']:.3e}/{p['rms']:.3e}/"
              f"{p['p95']:.3e}/{p['max']:.3e} N  (n={p['n']})")


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_deterministic_117mm_y_stress():
    mn, sn = unique_ns("s117_m"), unique_ns("s117_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS,
                sim_env=master_sim_env(REQUESTED_GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS,
                sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        # Let the deterministic initial condition establish itself (held
        # master, sagged slave) BEFORE the bridge starts, so the bridge
        # captures anchors at the intended separation.
        with LiveNode(f"s117_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"),
                     CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"),
                     CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)
            mp0 = list(pre.latest["m"].position_m)
            sp0 = list(pre.latest["s"].position_m)
        gap_y0, gap_norm0 = measured_gap(mp0, sp0)

        with ScriptProcess(bridge_args(master, slave, calib_path), name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"s117_v_{mn}") as live:
                live.watch("m", master.topic("cartesian_controller_telemetry"),
                          CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("s", slave.topic("cartesian_controller_telemetry"),
                          CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

                # Bounded retry: `ros2 param set` runs as its own process and
                # does its own DDS discovery, which can transiently miss a
                # node that is definitely up (wait_for_marker already
                # confirmed the bridge is live). This retries a discovery
                # race; it does not mask a real failure -- exhausting the
                # retries still fails the test.
                out = ""
                for _ in range(10):
                    r = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    out = r.stdout + r.stderr
                    if "successful" in r.stdout:
                        break
                    live.spin_for(0.5)
                assert "successful" in out, f"could not arm the bridge: {out!r}"

                m_stats, s_stats = ArmStats("MASTER"), ArmStats("SLAVE")
                states = []
                alphas = []
                holds = 0
                hold_ticks = 0
                # 2026-09-09 Part 1 audit fix: track ACCEPTED (published-
                # target, safety-relevant) and CANDIDATE (last-bisection-
                # trial, explanatory-only) margins separately -- see
                # coupling_force_budget.py's AdaptiveCouplingRamp docstring.
                # Only the accepted ones are asserted non-negative below.
                min_m_fmargin_acc = min_s_fmargin_acc = float("inf")
                min_m_pmargin_acc = min_s_pmargin_acc = float("inf")
                min_m_fmargin_cand = min_s_fmargin_cand = float("inf")
                min_m_pmargin_cand = min_s_pmargin_cand = float("inf")
                accepted_violations = []
                coupling_start = coupling_end = None
                fault_reason_seen = False
                reached_active = False
                last_diag_alpha = None

                deadline = time.time() + 20.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    coupling_phase = False
                    if d is not None:
                        if not states or states[-1] != d.bridge_state:
                            states.append(d.bridge_state)
                            if d.bridge_state == "COUPLING_RAMP_UP":
                                coupling_start = time.time()
                        if d.bridge_state == "FAULT":
                            fault_reason_seen = True
                            break
                        if d.bridge_state == "COUPLING_RAMP_UP":
                            coupling_phase = True
                            if last_diag_alpha is None or d.coupling_ramp_alpha != last_diag_alpha:
                                alphas.append(d.coupling_ramp_alpha)
                                last_diag_alpha = d.coupling_ramp_alpha
                            if d.coupling_ramp_held:
                                holds += 1
                                hold_ticks += 1
                            min_m_fmargin_acc = min(min_m_fmargin_acc, d.master_accepted_force_budget_margin_n)
                            min_s_fmargin_acc = min(min_s_fmargin_acc, d.slave_accepted_force_budget_margin_n)
                            min_m_pmargin_acc = min(min_m_pmargin_acc, d.master_accepted_position_error_margin_m)
                            min_s_pmargin_acc = min(min_s_pmargin_acc, d.slave_accepted_position_error_margin_m)
                            min_m_fmargin_cand = min(min_m_fmargin_cand, d.master_candidate_force_budget_margin_n)
                            min_s_fmargin_cand = min(min_s_fmargin_cand, d.slave_candidate_force_budget_margin_n)
                            min_m_pmargin_cand = min(min_m_pmargin_cand, d.master_candidate_position_error_margin_m)
                            min_s_pmargin_cand = min(min_s_pmargin_cand, d.slave_candidate_position_error_margin_m)
                            # Part 1's explicit hard requirement: the ACCEPTED
                            # (published) target must never violate the real
                            # controller's own position-error limit, on EVERY
                            # COUPLING_RAMP_UP tick -- not just at the end.
                            if d.master_accepted_position_error_margin_m < 0.0:
                                accepted_violations.append(("master_pos", d.master_accepted_position_error_margin_m))
                            if d.slave_accepted_position_error_margin_m < 0.0:
                                accepted_violations.append(("slave_pos", d.slave_accepted_position_error_margin_m))
                            if d.master_accepted_force_budget_margin_n < 0.0:
                                accepted_violations.append(("master_force", d.master_accepted_force_budget_margin_n))
                            if d.slave_accepted_force_budget_margin_n < 0.0:
                                accepted_violations.append(("slave_force", d.slave_accepted_force_budget_margin_n))
                            assert d.force_budget_telemetry_fresh_left, "telemetry went stale (left)"
                            assert d.force_budget_telemetry_fresh_right, "telemetry went stale (right)"
                            assert d.planner_parameter_match_left, "planner/controller param mismatch (left)"
                            assert d.planner_parameter_match_right, "planner/controller param mismatch (right)"
                        if d.bridge_state == "ACTIVE" and not reached_active:
                            reached_active = True
                            coupling_end = time.time()
                            break
                    if live.latest.get("m") is not None:
                        m_stats.add(live.latest["m"], coupling_phase)
                    if live.latest.get("s") is not None:
                        s_stats.add(live.latest["s"], coupling_phase)

                # ---------------- Part 6: measure, don't just assert -------------
                print("\n================ DETERMINISTIC 117mm Y STRESS ================")
                print("Initial condition:")
                print(f"  requested gap             = {REQUESTED_GAP_M*1000:8.3f} mm (Y)")
                print(f"  measured gap_y            = {gap_y0*1000:8.3f} mm")
                print(f"  measured |gap|            = {gap_norm0*1000:8.3f} mm")
                print(f"  master xyz                = ({mp0[0]:+.5f}, {mp0[1]:+.5f}, {mp0[2]:+.5f})")
                print(f"  slave  xyz                = ({sp0[0]:+.5f}, {sp0[1]:+.5f}, {sp0[2]:+.5f})")
                print(f"  xyz separation            = ({(mp0[0]-sp0[0])*1000:+.2f}, "
                      f"{(mp0[1]-sp0[1])*1000:+.2f}, {(mp0[2]-sp0[2])*1000:+.2f}) mm")
                print("Transition:")
                print(f"  state sequence            = {states}")
                dur = (coupling_end - coupling_start) if (coupling_start and coupling_end) else float('nan')
                print(f"  coupling duration         = {dur:8.3f} s")
                print(f"  alpha updates observed    = {len(alphas)}")
                print(f"  alpha holds (ticks)       = {hold_ticks}")
                print(f"  min master ACCEPTED force margin   = {min_m_fmargin_acc:8.4f} N")
                print(f"  min slave  ACCEPTED force margin   = {min_s_fmargin_acc:8.4f} N")
                print(f"  min master ACCEPTED pos-err margin = {min_m_pmargin_acc*1000:8.3f} mm")
                print(f"  min slave  ACCEPTED pos-err margin = {min_s_pmargin_acc*1000:8.3f} mm")
                print(f"  min master candidate force margin   = {min_m_fmargin_cand:8.4f} N  (diagnostic only, CAN be negative)")
                print(f"  min slave  candidate force margin   = {min_s_fmargin_cand:8.4f} N  (diagnostic only, CAN be negative)")
                print(f"  min master candidate pos-err margin = {min_m_pmargin_cand*1000:8.3f} mm  (diagnostic only, CAN be negative)")
                print(f"  min slave  candidate pos-err margin = {min_s_pmargin_cand*1000:8.3f} mm  (diagnostic only, CAN be negative)")
                print("Per-arm:")
                m_stats.report()
                s_stats.report()
                total_clamps = (m_stats.pos_clamp_samples + m_stats.force_clamp_samples
                                + m_stats.final_clamp_samples + s_stats.pos_clamp_samples
                                + s_stats.force_clamp_samples + s_stats.final_clamp_samples)
                print(f"Clamps: total clamped samples across both arms = {total_clamps}")
                print("==============================================================\n")

                # ---------------- Part 5: hard criteria -------------------------
                assert not accepted_violations, (
                    f"ACCEPTED (published) target violated a production limit on "
                    f"{len(accepted_violations)} tick(s) -- this WOULD be a real planner "
                    f"bug (not the expected, benign negative CANDIDATE margin on held "
                    f"ticks): {accepted_violations[:10]}"
                )
                assert not fault_reason_seen, f"bridge FAULTed during the run, states={states}"
                assert reached_active, f"must reach ACTIVE, states={states}"
                for required in ("READY", "GRAVITY_RAMP_UP", "GRAVITY_SETTLE", "COUPLING_RAMP_UP"):
                    assert required in states, f"missing state {required} in {states}"
                assert m_stats.pos_clamp_samples == 0, "master position clamp engaged"
                assert m_stats.force_clamp_samples == 0, "master primary force clamp engaged"
                assert m_stats.final_clamp_samples == 0, "master final force clamp engaged"
                assert s_stats.pos_clamp_samples == 0, "slave position clamp engaged"
                assert s_stats.force_clamp_samples == 0, "slave primary force clamp engaged"
                assert s_stats.final_clamp_samples == 0, "slave final force clamp engaged"
                for i in range(1, len(alphas)):
                    assert alphas[i] >= alphas[i - 1] - 1e-9, "coupling alpha must be monotonic"
                assert m_stats.pred_stats()["max"] < 1e-6, "master predictor must match controller"
                assert s_stats.pred_stats()["max"] < 1e-6, "slave predictor must match controller"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
