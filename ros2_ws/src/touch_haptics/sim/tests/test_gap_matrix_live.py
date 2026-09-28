"""Part 8 of the 2026-09-09 validation-infrastructure follow-up task: small
deterministic gap matrix (20/50/80/117/140mm), ONE frozen planner
configuration (identical to test_force_budget_117mm_live.py's bridge_args --
NOT tuned per gap, per Part 9's explicit "do not sweep parameters until the
frozen configuration passes the gap matrix"), same Y-dominant held-master/
sagged-slave fixture used throughout this validation task.

Only runs after Parts 1-7 pass (this file assumes that -- it does not
re-verify the settle gate, fail-safes, or disarm sequence, all covered by
their own dedicated test files).
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
GRAVITY_N = 0.70
GAPS_MM = (20, 50, 80, 117, 140)


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


# FROZEN configuration -- identical, verbatim, to test_force_budget_117mm_live.py's
# bridge_args -- the only thing that varies per case is the requested gap.
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
            "--enable-c2", "true", "--left-max-offset-m", "0.08",
            "--calibration-file", str(calib_path), "--axis-map", "x,y,z",
            "--absolute-axes", "x,y,z", "--absolute-trim-m", "0.0",
            "--slave-gravity-comp-n", str(GRAVITY_N), "--slave-gravity-axis", "1",
            "--master-stiffness-n-per-m", "15.0", "--master-damping-n-s-per-m", "0.10",
            "--master-max-force-n", "2.0", "--master-max-position-error-m", "0.03",
            "--stiffness-n-per-m", "35.0", "--slave-damping-n-s-per-m", "0.20",
            "--slave-max-force-n", "1.5", "--slave-max-position-error-m", "0.04",
            "--force-budget-reserve-n", "0.10",
            "--rate-hz", "100", "--wait-timeout-s", "15",
            ]


def _run_one_gap(gap_mm: float) -> dict:
    gap_m = gap_mm / 1000.0
    mn, sn = unique_ns(f"gm{gap_mm}_m"), unique_ns(f"gm{gap_mm}_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS, sim_env=master_sim_env(gap_m)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS, sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"gm_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)
            mp0 = list(pre.latest["m"].position_m)
            sp0 = list(pre.latest["s"].position_m)
        gap_y0, gap_norm0 = measured_gap(mp0, sp0)

        with ScriptProcess(bridge_args(master, slave, calib_path), name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"gm_v_{mn}") as live:
                live.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

                out = ""
                for _ in range(10):
                    r = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    out = r.stdout + r.stderr
                    if "successful" in r.stdout:
                        break
                    live.spin_for(0.5)
                assert "successful" in out, f"could not arm: {out!r}"

                states = []
                m_err, s_err = [], []
                m_applied, s_applied = [], []
                clamp_count = 0
                min_m_facc = min_s_facc = float("inf")
                min_m_pacc = min_s_pacc = float("inf")
                candidate_rejections = 0
                coupling_start = coupling_end = None
                settle_start = None
                fault_seen = False
                reached_active = False

                deadline = time.time() + 25.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    coupling_phase = False
                    if d is not None:
                        if not states or states[-1] != d.bridge_state:
                            states.append(d.bridge_state)
                            if d.bridge_state == "GRAVITY_SETTLE" and settle_start is None:
                                settle_start = time.time()
                            if d.bridge_state == "COUPLING_RAMP_UP":
                                coupling_start = time.time()
                        if d.bridge_state == "FAULT":
                            fault_seen = True
                            break
                        if d.bridge_state == "COUPLING_RAMP_UP":
                            coupling_phase = True
                            min_m_facc = min(min_m_facc, d.master_accepted_force_budget_margin_n)
                            min_s_facc = min(min_s_facc, d.slave_accepted_force_budget_margin_n)
                            min_m_pacc = min(min_m_pacc, d.master_accepted_position_error_margin_m)
                            min_s_pacc = min(min_s_pacc, d.slave_accepted_position_error_margin_m)
                            if d.coupling_ramp_held:
                                candidate_rejections += 1
                        if d.bridge_state == "ACTIVE" and not reached_active:
                            reached_active = True
                            coupling_end = time.time()
                            break
                    if coupling_phase:
                        m = live.latest.get("m")
                        s = live.latest.get("s")
                        if m is not None:
                            m_err.append(_norm(list(m.position_error_raw_m)))
                            m_applied.append(_norm(list(m.applied_force_n)))
                            clamp_count += int(m.position_error_clamped) + int(m.primary_force_clamped) + int(m.final_force_clamped)
                        if s is not None:
                            s_err.append(_norm(list(s.position_error_raw_m)))
                            s_applied.append(_norm(list(s.applied_force_n)))
                            clamp_count += int(s.position_error_clamped) + int(s.primary_force_clamped) + int(s.final_force_clamped)

    settle_time = (coupling_start - settle_start) if (settle_start and coupling_start) else float("nan")
    coupling_duration = (coupling_end - coupling_start) if (coupling_start and coupling_end) else float("nan")
    return {
        "gap_mm": gap_mm, "requested_gap_m": gap_m,
        "measured_gap_y_mm": gap_y0 * 1000, "measured_gap_norm_mm": gap_norm0 * 1000,
        "states": states, "fault_seen": fault_seen, "reached_active": reached_active,
        "settle_time_s": settle_time, "coupling_duration_s": coupling_duration,
        "candidate_rejection_ticks": candidate_rejections,
        "max_master_raw_err_mm": (max(m_err) * 1000) if m_err else None,
        "max_slave_raw_err_mm": (max(s_err) * 1000) if s_err else None,
        "rms_master_err_mm": (math.sqrt(statistics.fmean([e * e for e in m_err])) * 1000) if m_err else None,
        "rms_slave_err_mm": (math.sqrt(statistics.fmean([e * e for e in s_err])) * 1000) if s_err else None,
        "peak_master_applied_n": max(m_applied) if m_applied else None,
        "peak_slave_applied_n": max(s_applied) if s_applied else None,
        "min_accepted_master_force_margin_n": min_m_facc if min_m_facc != float("inf") else None,
        "min_accepted_slave_force_margin_n": min_s_facc if min_s_facc != float("inf") else None,
        "min_accepted_master_pos_margin_mm": (min_m_pacc * 1000) if min_m_pacc != float("inf") else None,
        "min_accepted_slave_pos_margin_mm": (min_s_pacc * 1000) if min_s_pacc != float("inf") else None,
        "clamp_count": clamp_count,
    }


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_gap_matrix_frozen_config():
    results = []
    for gap_mm in GAPS_MM:
        r = _run_one_gap(gap_mm)
        results.append(r)
        print(f"\n[gap={gap_mm}mm] requested={r['requested_gap_m']*1000:.1f}mm "
              f"measured_y={r['measured_gap_y_mm']:.2f}mm")
        print(f"  states={r['states']}")
        print(f"  settle_time={r['settle_time_s']:.3f}s coupling_duration={r['coupling_duration_s']:.3f}s "
              f"candidate_rejection_ticks={r['candidate_rejection_ticks']}")
        print(f"  max_raw_err master={r['max_master_raw_err_mm']}mm slave={r['max_slave_raw_err_mm']}mm")
        print(f"  rms_err master={r['rms_master_err_mm']}mm slave={r['rms_slave_err_mm']}mm")
        print(f"  peak_applied master={r['peak_master_applied_n']}N slave={r['peak_slave_applied_n']}N")
        print(f"  min_accepted_force_margin master={r['min_accepted_master_force_margin_n']}N "
              f"slave={r['min_accepted_slave_force_margin_n']}N")
        print(f"  min_accepted_pos_margin master={r['min_accepted_master_pos_margin_mm']}mm "
              f"slave={r['min_accepted_slave_pos_margin_mm']}mm")
        print(f"  clamp_count={r['clamp_count']} fault_seen={r['fault_seen']} reached_active={r['reached_active']}")

    print("\n================ GAP MATRIX SUMMARY (frozen config) ================")
    for r in results:
        print(f"  {r['gap_mm']:4d}mm: ACTIVE={r['reached_active']} FAULT={r['fault_seen']} "
              f"settle={r['settle_time_s']:.3f}s coupling={r['coupling_duration_s']:.3f}s "
              f"rejects={r['candidate_rejection_ticks']} "
              f"peak_applied(m/s)={r['peak_master_applied_n']:.3f}/{r['peak_slave_applied_n']:.3f}N "
              f"clamps={r['clamp_count']}")
    print("======================================================================\n")

    failures = []
    for r in results:
        if r["fault_seen"]:
            failures.append(f"{r['gap_mm']}mm: FAULTed, states={r['states']}")
        if not r["reached_active"]:
            failures.append(f"{r['gap_mm']}mm: never reached ACTIVE")
        if r["clamp_count"] > 0:
            failures.append(f"{r['gap_mm']}mm: {r['clamp_count']} clamp sample(s) engaged")
    assert not failures, f"gap matrix failures under the FROZEN config: {failures}"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
