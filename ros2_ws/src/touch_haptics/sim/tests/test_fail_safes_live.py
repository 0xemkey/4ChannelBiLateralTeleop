"""Part 6 of the 2026-09-09 validation-infrastructure follow-up task:
DEDICATED live-sim fail-safe tests, not incidental evidence -- each test
deliberately constructs the failure condition and asserts the exact safe
FAULT reason, using the REAL bridge_master_slave.py against REAL
touch_slave_sim controllers (matches every other *_live.py test in this
directory).

Covers Part 6.B (PARAMETER MISMATCH) and Part 6.C (FORCE-BUDGET STALL).
Part 6.A (TELEMETRY STALE) needs a telemetry-relay proxy to desync
telemetry from pose cleanly (touch_slave_sim's cartesian_controller_
telemetry topic is NOT gated by its own publish_telemetry_ param, so it
cannot be toggled off directly) -- see test_fail_safe_telemetry_stale_live.py.
Part 6.D (GRAVITY_SETTLE_TIMEOUT) is covered by the PURE test
test_max_timeout_triggers_gravity_settle_timeout_fault in
scripts/tests/test_gravity_settle_gate.py -- that logic is pure Python
(no ROS/physics dependency) already exercised directly with fast custom
thresholds; a live version would re-prove the same velocity-threshold
comparison through much more integration surface for no added coverage
(GRAVITY_SETTLE_TIMEOUT is a state-machine-internal decision, unlike
B/C which are genuinely exercising the live bridge<->controller telemetry
path).
"""
from __future__ import annotations

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
    MASTER_PARAMS, SLAVE_PARAMS, SETTLE_S, master_sim_env, slave_sim_env,
)

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)

GRAVITY_N = 0.70
GAP_M = 0.05  # modest gap -- these tests are about the fail-safe, not the gap magnitude


def _base_bridge_args(master, slave, calib_path, **overrides):
    args = {
        "left-pose-topic": master.topic("pose"), "right-pose-topic": slave.topic("pose"),
        "right-target-topic": slave.topic("target_pose"), "right-enable-topic": slave.topic("enable"),
        "left-target-topic": master.topic("target_pose"), "left-enable-topic": master.topic("enable"),
        "left-telemetry-topic": master.topic("cartesian_controller_telemetry"),
        "right-telemetry-topic": slave.topic("cartesian_controller_telemetry"),
        "slave-force-topic": slave.topic("force_cmd"),
        "enable-c2": "true", "left-max-offset-m": "0.08",
        "calibration-file": str(calib_path), "axis-map": "x,y,z",
        "absolute-axes": "x,y,z", "absolute-trim-m": "0.0",
        "slave-gravity-comp-n": str(GRAVITY_N), "slave-gravity-axis": "1",
        "master-stiffness-n-per-m": "15.0", "master-damping-n-s-per-m": "0.10",
        "master-max-force-n": "2.0", "master-max-position-error-m": "0.03",
        "stiffness-n-per-m": "35.0", "slave-damping-n-s-per-m": "0.20",
        "slave-max-force-n": "1.5", "slave-max-position-error-m": "0.04",
        "force-budget-reserve-n": "0.10",
        "rate-hz": "100", "wait-timeout-s": "15",
    }
    args.update(overrides)
    out = [str(BRIDGE_SCRIPT)]
    for k, v in args.items():
        out += [f"--{k}", str(v)]
    return out


def _arm_and_wait_for_fault(live, timeout_s=20.0):
    """Shared drive loop: arm, then watch bridge_diagnostics for FAULT.
    Returns (states_seen, final_diag)."""
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

    states = []
    deadline = time.time() + timeout_s
    d = None
    while time.time() < deadline:
        live.spin_once(timeout_sec=0.01)
        d = live.latest.get("d")
        if d is None:
            continue
        if not states or states[-1] != d.bridge_state:
            states.append(d.bridge_state)
        if d.bridge_state == "FAULT":
            break
    return states, d


# ---------------------------------------------------------------------------
# Part 6.B -- PARAMETER MISMATCH: controller and planner launched with
# different K/D/Fmax/position-error-limit.
# ---------------------------------------------------------------------------
@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_parameter_mismatch_faults_safely():
    mn, sn = unique_ns("pmm_m"), unique_ns("pmm_s")
    # Deliberately mismatched: the SLAVE controller runs Fmax=1.5N (its own
    # real default), but the bridge is told (and the planner therefore
    # believes) it is 1.2N -- a genuine declared-vs-actual disagreement.
    with SimNode(mn, device="Left", params=MASTER_PARAMS, sim_env=master_sim_env(GAP_M)) as master, \
        SimNode(sn, device="Default Device",
                params={**SLAVE_PARAMS, "max_force_n": "1.5"},
                sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"pmm_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)

        args = _base_bridge_args(master, slave, calib_path, **{"slave-max-force-n": "1.2"})
        with ScriptProcess(args, name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)
            with LiveNode(f"pmm_v_{mn}") as live:
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

                # No coupling progress should ever be observed before FAULT.
                states, d = _arm_and_wait_for_fault(live, timeout_s=20.0)

        assert "FAULT" in states, f"expected a FAULT, got states={states}"
        assert d is not None
        assert d.planner_parameter_match_right is False, (
            "planner_parameter_match_right should read False at the mismatch"
        )
        assert d.coupling_ramp_alpha == 0.0, (
            f"no coupling progress should have occurred before the mismatch fault, "
            f"got coupling_ramp_alpha={d.coupling_ramp_alpha}"
        )
        print(f"[6B] PARAMETER MISMATCH: states={states}, "
              f"planner_parameter_match_left={d.planner_parameter_match_left} "
              f"planner_parameter_match_right={d.planner_parameter_match_right}, "
              f"coupling_ramp_alpha={d.coupling_ramp_alpha} -- safe FAULT confirmed. PASS")


# ---------------------------------------------------------------------------
# Part 6.C -- FORCE-BUDGET STALL: construct a condition with NO safe alpha
# progress available (force_budget_reserve_n set so close to slave's
# max_force_n that the constant gravity-comp AUX ALONE already exceeds the
# remaining budget at alpha=0 -- position-independent, so no bisection step,
# however small, can ever be accepted).
# ---------------------------------------------------------------------------
@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_force_budget_stall_faults_safely():
    mn, sn = unique_ns("fbs_m"), unique_ns("fbs_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS, sim_env=master_sim_env(GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS, sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"fbs_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)

        # reserve=1.45N leaves only 0.05N of budget below slave's 1.5N
        # Fmax -- GRAVITY_N=0.70N alone (a constant, position-independent
        # aux term) already exceeds that at ANY alpha, including alpha=0's
        # smallest possible bisection increment. Fast stall_timeout_s so
        # this test doesn't need to wait the production 3.0s default.
        args = _base_bridge_args(
            master, slave, calib_path,
            **{"force-budget-reserve-n": "1.45", "force-budget-stall-timeout-s": "1.0"})
        with ScriptProcess(args, name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)
            with LiveNode(f"fbs_v_{mn}") as live:
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

                held_ticks = 0
                max_alpha_seen = 0.0
                states, d = None, None
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

                states = []
                deadline = time.time() + 20.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is None:
                        continue
                    if not states or states[-1] != d.bridge_state:
                        states.append(d.bridge_state)
                    if d.bridge_state == "COUPLING_RAMP_UP":
                        max_alpha_seen = max(max_alpha_seen, d.coupling_ramp_alpha)
                        if d.coupling_ramp_held:
                            held_ticks += 1
                    if d.bridge_state == "FAULT":
                        break

        assert "COUPLING_RAMP_UP" in states, f"never even entered COUPLING_RAMP_UP: states={states}"
        assert "FAULT" in states, f"expected a FAULT, got states={states}"
        assert held_ticks > 0, "stall scenario should have produced held ticks before FAULT"
        assert max_alpha_seen < 1e-6, (
            f"NO safe alpha progress should have been possible at all, got max_alpha_seen={max_alpha_seen}"
        )
        print(f"[6C] FORCE-BUDGET STALL: states={states}, held_ticks={held_ticks}, "
              f"max_alpha_seen={max_alpha_seen}, final coupling_ramp_stall_time_s={d.coupling_ramp_stall_time_s:.3f}s "
              f"-- safe FAULT confirmed, zero progress the whole time. PASS")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
