"""2026-09-10 real-hardware finding regression tests: the production
teleoperation arm sequence.

Confirmed live on real hardware: `experiment_launch/03_start_position_
bridge.sh --arm-position` started the bridge process and left C3
configured, `/touch_right/enable == true`, but NEVER set `arm_teleop=true`
on the bridge -- confirmed via `grep -rn arm_teleop experiment_launch/*.sh`
returning zero matches. `bridge_state` stayed READY forever;
`coupling_ramp_alpha` never left 0; moving the master produced zero slave
motion. Fixed with `hardware_validation/arm_production_bridge.py`, wired
into `03_start_position_bridge.sh` after `start_component bridge`.

These tests reproduce the OLD (buggy) behavior against a live sim bridge
first (must fail to arm), then prove the NEW arm_production_bridge.py
script actually reaches ACTIVE and C3 tracks master motion.

DOMAIN ISOLATION: same convention as this directory's other real-hardware-
adjacent tests -- ROS_DOMAIN_ID forced to an isolated value before any
rclpy-touching import/subprocess spawn.
"""
from __future__ import annotations

import os
os.environ["ROS_DOMAIN_ID"] = "49"

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
from sim_common import LiveNode, ScriptProcess, SimNode  # noqa: E402

HW_DIR = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_experiments"
    / "2026-09-09_y_axis_validation_PENDING_SUPERVISION"
)
sys.path.insert(0, str(HW_DIR))
from frozen_config import FROZEN, bridge_cli_args  # noqa: E402

HV_DIR = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_validation"
)
ARM_SCRIPT = HV_DIR / "arm_production_bridge.py"

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)

HAND_AMPLITUDE_M = 0.005
HAND_F0_HZ = 0.1


def _launch_production_bridge(ns_prefix):
    """Launches a bridge matching the PRODUCTION configuration exactly as
    03_start_position_bridge.sh's own `cmd` array resolves it:
    enable_c2=False, c3_enable=True (default), c1/c4=False (default),
    require_master_deadman=False (production never sets this).

    Deliberately uses the REAL topic names (/touch_left/..., /touch_right/...,
    /bridge_diagnostics) -- NOT retopic'd to a namespaced sim pair -- same
    technique as this session's earlier real-entrypoint H2 confirmation
    test: check_enable_publisher_ownership() (and arm_production_bridge.py,
    which is meant to run against production's real topics) hardcode those
    exact names, so retopicing enable/diagnostics here would make the
    ownership check fail for a reason that has nothing to do with the
    actual bug under test. Safe because ROS_DOMAIN_ID=49 (set at module
    import time, above) provides full network isolation regardless of
    topic name reuse -- this never touches the real hardware domain."""
    master_params = {"stiffness_n_per_m": str(FROZEN.master_stiffness_n_per_m),
                     "damping_n_s_per_m": str(FROZEN.master_damping_n_s_per_m),
                     "max_force_n": str(FROZEN.master_max_force_n),
                     "max_position_error_m": str(FROZEN.master_max_position_error_m),
                     "require_deadman": "false"}
    slave_params = {"stiffness_n_per_m": str(FROZEN.slave_stiffness_n_per_m),
                    "damping_n_s_per_m": str(FROZEN.slave_damping_n_s_per_m),
                    "max_force_n": str(FROZEN.slave_max_force_n),
                    "max_position_error_m": str(FROZEN.slave_max_position_error_m),
                    "require_deadman": "false"}
    master_sim_env = {
        "TOUCH_SIM_INIT_POS_X": "-0.10", "TOUCH_SIM_INIT_POS_Y": "-0.21", "TOUCH_SIM_INIT_POS_Z": "0.0",
        "TOUCH_SIM_HAND": "stiff", "TOUCH_SIM_HAND_MOTION": "sine",
        "TOUCH_SIM_HAND_AMP": str(HAND_AMPLITUDE_M), "TOUCH_SIM_HAND_F0": str(HAND_F0_HZ),
    }
    master = SimNode("touch_left", device="Left", params=master_params, sim_env=master_sim_env)
    slave = SimNode("touch_right", device="Default Device", params=slave_params)
    master.__enter__(); slave.__enter__()

    calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
    calib_path.unlink()
    bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False,
                                                    gravity_settle_hold_mode=False,
                                                    c3_enable=True, require_master_deadman=False)

    bridge = ScriptProcess(bargs, name=f"prodbridge_{ns_prefix}")
    bridge.wait_for_marker("teleoperation live", timeout=15.0)
    return master, slave, bridge


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_old_orchestration_never_arms():
    """Reproduces the exact confirmed bug: a bridge started and left alone
    (the OLD 03_start_position_bridge.sh behavior -- start the process,
    never call arm_teleop) never leaves READY, no matter how long you
    wait, and coupling_ramp_alpha never leaves 0. This is the regression
    baseline the fix must actually change."""
    master, slave, bridge = _launch_production_bridge("oldorch")
    try:
        with LiveNode("old_orch_watch") as live:
            live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
            deadline = time.time() + 5.0
            last_state = None
            max_coupling_alpha = 0.0
            while time.time() < deadline:
                live.spin_once(timeout_sec=0.02)
                d = live.latest.get("d")
                if d is not None:
                    last_state = d.bridge_state
                    max_coupling_alpha = max(max_coupling_alpha, d.coupling_ramp_alpha)
            assert last_state == "READY", (
                f"expected the OLD (unfixed) orchestration to leave the bridge stuck in READY "
                f"forever, got state={last_state} -- if this fails, something ELSE is now also "
                f"arming the bridge unexpectedly")
            assert max_coupling_alpha == 0.0
    finally:
        bridge.stop(); master.stop(); slave.stop()


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
@pytest.mark.skipif(not ARM_SCRIPT.exists(), reason=f"arm script not found at {ARM_SCRIPT}")
def test_new_arm_sequence_reaches_active_and_c3_tracks_master():
    """Proves the FIX: arm_production_bridge.py actually sets
    arm_teleop=true, the state machine progresses out of READY through to
    ACTIVE, coupling_ramp_alpha reaches ~1.0, c3_enabled stays true
    throughout, and -- with the sim's sine-driven 'hand' standing in for
    manual master motion -- the slave visibly tracks it (same technique as
    test_part3_c3_slave_tracking_sim.py)."""
    master, slave, bridge = _launch_production_bridge("newarm")
    try:
        arm_args = [
            str(ARM_SCRIPT),
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--bridge-diagnostics-topic", "/bridge_diagnostics",
            "--expected-c1", "false", "--expected-c2", "false",
            "--expected-c3", "true", "--expected-c4", "false",
            "--progression-timeout-s", "20",
        ]
        r = subprocess.run([sys.executable] + arm_args, capture_output=True, text=True, timeout=45)
        print(r.stdout[-4000:])
        if r.returncode != 0:
            print("STDERR:", r.stderr[-2000:])
        assert r.returncode == 0, "arm_production_bridge.py did not succeed"
        assert "PRODUCTION TELEOPERATION ACTIVE" in r.stdout
        assert "ACTIVE reached" in r.stdout

        # Independently verify live, from a fresh subscription (not just
        # trusting the arm script's own stdout).
        with LiveNode("new_arm_watch") as live:
            live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
            live.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry,
                      extract=lambda x: x)
            live.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry,
                      extract=lambda x: x)
            live.spin_for(0.5)
            d = live.latest.get("d")
            assert d is not None
            assert d.bridge_state == "ACTIVE"
            assert d.coupling_ramp_alpha > 0.999
            assert d.c3_enabled is True
            assert d.c1_enabled is False and d.c2_enabled is False and d.c4_enabled is False

            rows = []
            deadline = time.time() + 12.0
            while time.time() < deadline:
                live.spin_once(timeout_sec=0.01)
                m, s = live.latest.get("m"), live.latest.get("s")
                if m is not None and s is not None:
                    rows.append({"t": time.time(), "my": m.position_m[1], "sy": s.position_m[1]})
            assert rows, "no telemetry collected during the tracking observation window"
            my0 = rows[0]["my"]
            m_disp = [r["my"] - my0 for r in rows]
            s_disp = [r["sy"] - rows[0]["sy"] for r in rows]
            m_ptp = max(m_disp) - min(m_disp)
            s_ptp = max(s_disp) - min(s_disp)
            gain = s_ptp / m_ptp if m_ptp > 1e-9 else float("nan")
            print(f"[test] master Y ptp={m_ptp*1000:.3f}mm slave Y ptp={s_ptp*1000:.3f}mm gain={gain:.4f}")
            assert m_ptp > 0.001, "master (sine-hand) should have visibly moved"
            assert gain > 0.3, f"slave should visibly follow the master's oscillation via C3, gain={gain:.4f}"

        subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                       capture_output=True, text=True, timeout=10)
    finally:
        bridge.stop(); master.stop(); slave.stop()


@pytest.mark.skipif(not ARM_SCRIPT.exists(), reason=f"arm script not found at {ARM_SCRIPT}")
def test_arm_sequence_fails_closed_when_no_bridge_present():
    """No bridge running at all -- publisher-ownership check must fail
    closed (zero publishers, not the expected one), never hang, never
    falsely claim ACTIVE."""
    arm_args = [
        str(ARM_SCRIPT),
        "--left-telemetry-topic", "/test_no_bridge_left_telem",
        "--right-telemetry-topic", "/test_no_bridge_right_telem",
        "--bridge-diagnostics-topic", "/test_no_bridge_diag",
        "--ready-wait-timeout-s", "2", "--window-s", "1",
    ]
    r = subprocess.run([sys.executable] + arm_args, capture_output=True, text=True, timeout=30)
    assert r.returncode != 0
    assert "PRODUCTION TELEOPERATION ACTIVE" not in r.stdout
    assert "FAIL CLOSED" in r.stdout


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
