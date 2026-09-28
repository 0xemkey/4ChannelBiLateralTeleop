"""End-to-end sim integration test for run_gravcomp_trial.py (2026-09-09,
Section 9 of the recorder-protocol follow-up task).

The fake-clock unit test (test_trial_recorder_state_machine.py, in
projects/teleop_left_master_right_slave/experiment_launch/) proves the
_on_bridge_diag() state-transition logic in isolation. This test proves the
ACTUAL recorder SCRIPT, run as its own subprocess against a REAL (sim)
bridge_master_slave.py process and two touch_slave_sim instances, produces a
complete, correctly-ordered trial: READY -> RAMPING (telemetry preserved) ->
ACTIVE (exact window) -> READY (post-disarm settling captured) -- driven
entirely by real ROS topics, not synthetic callbacks.

Uses --topic-prefix-left/--topic-prefix-right/a short --duration-s so this
runs on sim's own namespaced topics (never the real hardware's /touch_left,
/touch_right -- those may be live from an unrelated real-hardware session on
the same machine) and completes quickly.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import ScriptProcess, SimNode, unique_ns  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)
RECORDER_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "experiment_launch" / "run_gravcomp_trial.py"
)

GRAVITY_N = 0.55


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists() or not RECORDER_SCRIPT.exists(),
                    reason="bridge_master_slave.py or run_gravcomp_trial.py not found")
def test_recorder_captures_full_arm_cycle_against_real_bridge():
    master_ns = unique_ns("rec_master")
    slave_ns = unique_ns("rec_slave")

    with SimNode(master_ns, device="Left",
                # max_force_n/max_position_error_m overridden to match the
                # bridge's own --master-max-force-n/--master-max-position-
                # error-m defaults (2.0N/0.03m) -- otherwise the controller's
                # own telemetry (touch_slave_sim default 0.45N/0.02m) disagrees
                # with what the bridge's force-budget planner declares, and
                # the 2026-09-09 production-integration task's planner/
                # controller parameter-consistency check (COUPLING_RAMP_
                # PARAMETER_MISMATCH) correctly faults on that real
                # disagreement -- a pre-existing gap in this test's fixture,
                # unrelated to the GRAVITY_SETTLE velocity-gating change.
                params={"stiffness_n_per_m": "15.0", "damping_n_s_per_m": "0.10",
                       "max_force_n": "2.0", "max_position_error_m": "0.03",
                       "require_deadman": "false"}) as master, \
        SimNode(slave_ns, device="Default Device",
                # max_force_n overridden -- see test_gravity_comp_arm_gate_live.py's
                # matching comment: touch_slave.cpp's own default (0.45N) would
                # silently clamp GRAVITY_N=0.55N before this test could observe it.
                params={"stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                       "require_deadman": "false", "max_force_n": "1.5"}) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with ScriptProcess(
            [str(BRIDGE_SCRIPT),
            "--left-pose-topic", master.topic("pose"),
            "--right-pose-topic", slave.topic("pose"),
            "--right-target-topic", slave.topic("target_pose"),
            "--right-enable-topic", slave.topic("enable"),
            "--left-target-topic", master.topic("target_pose"),
            "--left-enable-topic", master.topic("enable"),
            # 2026-09-09 production-integration task: see test_c3_live.py's
            # matching comment.
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--slave-force-topic", slave.topic("force_cmd"),
            "--enable-c2", "true",
            "--left-max-offset-m", "0.08",
            "--calibration-file", str(calib_path),
            "--axis-map", "x,y,z",
            "--absolute-axes", "x,y,z", "--absolute-trim-m", "0.0",
            "--slave-gravity-comp-n", str(GRAVITY_N), "--slave-gravity-axis", "1",
            "--rate-hz", "100", "--wait-timeout-s", "15",
            ],
            name=f"bridge_{master_ns}",
        ) as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with tempfile.TemporaryDirectory() as session_dir:
                recorder = subprocess.Popen(
                    [sys.executable, str(RECORDER_SCRIPT),
                     "--session-dir", session_dir, "--condition", "0.55",
                     "--trial-index", "1", "--duration-s", "2.0",
                     "--arm-wait-timeout-s", "20", "--disarm-wait-timeout-s", "20",
                     "--settle-window-s", "1.0",
                     "--topic-prefix-left", f"/{master_ns}",
                     "--topic-prefix-right", f"/{slave_ns}"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                )

                # Wait for the recorder to confirm READY_FOR_ARM before arming --
                # arming earlier would race episode-capture startup.
                deadline = time.time() + 15.0
                saw_ready = False
                lines = []
                while time.time() < deadline:
                    line = recorder.stdout.readline()
                    if not line:
                        if recorder.poll() is not None:
                            break
                        continue
                    lines.append(line)
                    if "READY_FOR_ARM" in line:
                        saw_ready = True
                        break
                assert saw_ready, f"recorder never reported READY_FOR_ARM. Output so far:\n{''.join(lines)}"

                r = subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                    capture_output=True, text=True, timeout=10)
                assert "successful" in r.stdout

                # The recorder blocks (up to --disarm-wait-timeout-s) AFTER
                # ACTIVE_END waiting to OBSERVE the real disarm -- so the disarm
                # command must be sent WHILE the recorder is still running and
                # watching, not after it exits. Read its live output for the
                # ACTIVE_END marker, disarm immediately, then let it finish.
                active_end_deadline = time.time() + 15.0
                saw_active_end = False
                while time.time() < active_end_deadline:
                    line = recorder.stdout.readline()
                    if not line:
                        if recorder.poll() is not None:
                            break
                        continue
                    lines.append(line)
                    if "ACTIVE_END" in line:
                        saw_active_end = True
                        break
                assert saw_active_end, f"recorder never reported ACTIVE_END. Output so far:\n{''.join(lines)}"

                subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                    capture_output=True, text=True, timeout=10)

                remaining_out, _ = recorder.communicate(timeout=30)
                lines.append(remaining_out)
                full_output = "".join(lines)
                assert recorder.returncode == 0, f"recorder exited {recorder.returncode}:\n{full_output}"

                metrics_path = Path(session_dir) / "analysis" / "trial_0.55_1" / "metrics.json"
                assert metrics_path.exists(), f"metrics.json not written. Recorder output:\n{full_output}"
                with open(metrics_path) as f:
                    metrics = json.load(f)

                ev = metrics["event_timestamps"]
                assert ev["arm_transition_t"] is not None
                assert ev["ramp_start_t"] is not None
                assert ev["active_start_t"] is not None
                assert ev["metric_active_end_t"] is not None
                assert ev["active_capture_stop_observed_t"] is not None
                assert ev["disarm_transition_t"] is not None, (
                    "disarm_transition_t is null -- either the real disarm above didn't "
                    "land before disarm_wait_timeout, or the transition wasn't observed")
                assert (ev["ramp_start_t"] <= ev["active_start_t"] <= ev["metric_active_end_t"]
                        <= ev["disarm_transition_t"] + 1.0), f"event ordering violated: {ev}"
                assert ev["exact_active_duration_valid"] is True
                assert abs(ev["metric_active_end_t"] - ev["active_start_t"] - 2.0) < 1e-6, (
                    "metric window must be exactly the requested --duration-s=2.0")

                out_dir = metrics_path.parent
                required_csvs = ["left_pose.csv", "right_pose.csv", "left_telem.csv",
                                "right_telem.csv", "bridge_diag.csv",
                                "left_target.csv", "right_target.csv"]
                for fname in required_csvs:
                    assert (out_dir / fname).exists(), f"missing required output CSV: {fname}"

                if metrics["dob_available"]:
                    for fname in ("left_dob.csv", "right_dob.csv"):
                        p = out_dir / fname
                        assert p.exists(), f"dob_available=true but missing {fname}"
                        with open(p) as f:
                            dob_rows = f.readlines()
                        assert len(dob_rows) > 1, f"{fname} exists but has no data rows"
                else:
                    print("[note] dob_available=false for this sim run -- DOB CSV completeness not asserted "
                          "(touch_slave_sim's DOB bank may not be configured in this test fixture)")

                import csv as _csv
                with open(out_dir / "right_telem.csv") as f:
                    rows = list(_csv.DictReader(f))
                # 2026-09-09 Part 5 (validation-infrastructure follow-up): the
                # recorder no longer buckets every arm-sequence sample into a
                # single generic "RAMPING" phase -- it now records the REAL
                # per-tick bridge_state (GRAVITY_RAMP_UP/GRAVITY_SETTLE/
                # COUPLING_RAMP_UP), and "POST_DISARM" is now
                # "POST_DISARM_READY" (disambiguated from PRE_ARM, both of
                # which are bridge_state=="READY"). "ramp_rows" here means
                # the union of all three arm-sequence sub-phases, matching
                # what the old single "RAMPING" bucket covered.
                ramp_states = {"GRAVITY_RAMP_UP", "GRAVITY_SETTLE", "COUPLING_RAMP_UP"}
                ramp_rows = [r for r in rows if r["phase"] in ramp_states]
                active_rows = [r for r in rows if r["phase"] == "ACTIVE"]
                post_disarm_rows = [r for r in rows if r["phase"] == "POST_DISARM_READY"]
                assert len(ramp_rows) > 0, "no arm-sequence-phase telemetry captured (the v2 gap this test guards against)"
                assert len(active_rows) > 0, "no ACTIVE-phase telemetry captured"
                assert len(post_disarm_rows) > 0, "no POST_DISARM_READY-phase telemetry captured"
                # And confirm the phase labeling is now GRANULAR, not one
                # generic bucket -- this is the actual Part 5 improvement.
                seen_ramp_substates = {r["phase"] for r in ramp_rows}
                assert len(seen_ramp_substates) >= 2, (
                    f"expected to observe at least 2 distinct arm-sequence sub-phases "
                    f"(GRAVITY_RAMP_UP/GRAVITY_SETTLE/COUPLING_RAMP_UP), got only "
                    f"{seen_ramp_substates} -- phase labeling may have regressed to one generic bucket"
                )

                ramp_aux_y = [float(r["aux_y"]) for r in ramp_rows]
                assert min(ramp_aux_y) < GRAVITY_N - 0.05, (
                    f"ramp aux_y never showed a partial value, looks like a step not a ramp: {ramp_aux_y}")
                active_aux_y = [float(r["aux_y"]) for r in active_rows]
                assert all(abs(v - GRAVITY_N) < 0.02 for v in active_aux_y), (
                    f"ACTIVE aux_y should equal configured {GRAVITY_N}N throughout: {active_aux_y}")
                # Skip the first few POST_DISARM_READY samples: nearest-
                # in-time phase labeling right at a fast GRAVITY_RAMP_DOWN
                # ->READY boundary can attribute 1-2 still-decaying
                # telemetry samples (200Hz) to the READY label a bridge_diag
                # sample (its own, slower publish cadence) already reports --
                # a multi-rate/sample-hold boundary artifact of the same
                # kind test_ramp_disarm_root_cause.py already characterized
                # for this project, not a gravity-comp gate regression. The
                # steady-state (all but the first few samples) must be ~0.
                post_disarm_aux_y = [float(r["aux_y"]) for r in post_disarm_rows][3:]
                assert post_disarm_aux_y, "not enough POST_DISARM_READY samples to check steady state"
                assert all(abs(v) < 0.02 for v in post_disarm_aux_y), (
                    f"POST_DISARM_READY steady-state aux_y should be ~0: {post_disarm_aux_y}")

                assert metrics["active_window_violation_count"] == 0
                assert metrics["dob_available"] in (True, False)  # explicitly recorded either way

                # Section 5/9: target CSVs exist and have samples in every phase.
                with open(out_dir / "left_target.csv") as f:
                    lt_rows = list(_csv.DictReader(f))
                with open(out_dir / "right_target.csv") as f:
                    rt_rows = list(_csv.DictReader(f))
                for label, rows_ in (("left_target", lt_rows), ("right_target", rt_rows)):
                    phases_present = {r["phase"] for r in rows_}
                    for required_phase in ("PRE_ARM", "GRAVITY_RAMP_UP", "GRAVITY_SETTLE",
                                           "COUPLING_RAMP_UP", "ACTIVE"):
                        assert required_phase in phases_present, (
                            f"{label}.csv missing {required_phase}-phase samples "
                            f"(phases present: {phases_present})")

                # Section 2: three-category target-discontinuity metrics present.
                rv = metrics["ramp_validation"]
                for key in ("prearm_to_ramp_target_step_left_m", "prearm_to_ramp_target_step_right_m",
                           "ramp_to_active_target_step_left_m", "ramp_to_active_target_step_right_m"):
                    assert key in rv, f"missing boundary-step metric: {key}"

                # Section 1: ramp position-error stats present.
                for key in ("ramp_max_position_error_raw_left_m", "ramp_rms_position_error_raw_left_m",
                           "ramp_max_position_error_raw_right_m", "ramp_rms_position_error_raw_right_m"):
                    assert key in rv, f"missing ramp position-error metric: {key}"

                # Section 7: acquisition-validity block present and clean for this run.
                pv = metrics["protocol_validity"]
                assert pv["all_hard_criteria_pass"] is True, f"protocol_validity failed: {pv}"
                assert metrics["aggregate_include"] is None, (
                    "acquisition-clean trial should get aggregate_include=None "
                    "(acquisition OK; scientific inclusion is a separate call)")

                print(f"PASS: ramp={len(ramp_rows)} active={len(active_rows)} "
                      f"post_disarm={len(post_disarm_rows)} samples, "
                      f"ramp_aux_y range=[{min(ramp_aux_y):.3f},{max(ramp_aux_y):.3f}], "
                      f"active_aux_y~{GRAVITY_N}, post_disarm_aux_y~0, "
                      f"target boundary steps left(prearm->ramp)={rv['prearm_to_ramp_target_step_left_m']}, "
                      f"protocol_validity.all_hard_criteria_pass={pv['all_hard_criteria_pass']}, "
                      f"event_timestamps={ev}")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
