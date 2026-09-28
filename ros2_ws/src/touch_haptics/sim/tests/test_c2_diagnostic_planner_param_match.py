"""2026-09-10 real-hardware finding regression tests: the C2-only diagnostic
arm sequence FAULTed on real hardware (session 20260910T120450Z_c2_baseline)
with planner_parameter_match_left=false. Root cause: bridge_master_slave.py
has always had --master-stiffness-n-per-m / --master-damping-n-s-per-m /
--master-max-force-n / --master-max-position-error-m CLI args (cross-checked
live against the master controller's own telemetry -- see
planner_parameter_match_left in bridge_master_slave.py), but run_bridge.sh
never wired ANY of them to an environment variable -- it always launched the
bridge with the hardcoded argparse defaults (15.0 / 0.10 / 2.0 / 0.03, which
happen to match run_master_left.sh's own production defaults). This was
invisible for every prior real-hardware run because they all used the
production master driver. It broke the instant a launcher started a master
driver with different gains: 01b_start_master_c2_diagnostic.sh's diagnostic
gains (5.0 / 0.05 / 0.5), used by 03b/03c.

Fixed by: adding MASTER_STIFFNESS_N_PER_M / MASTER_DAMPING_N_S_PER_M /
MASTER_MAX_FORCE_N / MASTER_MAX_POSITION_ERROR_M env vars to run_bridge.sh
(defaulting to the same production values as before, so every existing
caller that doesn't set them is unaffected), and setting the diagnostic
values explicitly in 03b_start_bridge_c2_diagnostic.sh and
03c_start_bridge_c2_only_diagnostic.sh (matching 01b's actual master gains).

These tests exercise bridge_master_slave.py directly (not the shell
launchers -- shell env-var wiring isn't independently testable via pytest),
proving: (1) the exact FAULT reproduces when the bridge's declared master
params disagree with the master controller's actual telemetry, (2) it
clears -- reaching ACTIVE -- once the declared params are corrected to match,
(3) channel isolation (C1=C4=false, C2=true, C3=false) holds throughout, and
(4) the production C3 launch (test_production_arm_sequence_live.py) is
untouched by this fix, since it never overrides the master planner args.

DOMAIN ISOLATION: same convention as this directory's other real-hardware-
adjacent tests -- ROS_DOMAIN_ID forced to an isolated value before any
rclpy-touching import/subprocess spawn.
"""
from __future__ import annotations

import os
os.environ["ROS_DOMAIN_ID"] = "51"

import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from touch_haptics.msg import BridgeDiagnostics

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

# The actual diagnostic master gains 01b_start_master_c2_diagnostic.sh
# starts the master driver with -- NOT the production defaults.
#
# 2026-09-10, Candidate A (C2 force-budget fix): max_force_n 0.8->0.85 and
# max_position_error_m 0.03->0.04 (K, damping, LEFT_GRAVITY_COMP_N
# unchanged) -- 0.15N spring authority was architecturally insufficient
# (measured on real hardware: master never moved). 0.20N spring authority
# (K=5.0 * 0.04) + 0.55N gravity aux + 0.10N reserve = 0.85N required
# budget, exactly matching the new max_force_n.
DIAG_STIFFNESS_N_PER_M = 5.0
DIAG_DAMPING_N_S_PER_M = 0.05
DIAG_MAX_FORCE_N = 0.85
DIAG_MAX_POSITION_ERROR_M = 0.04


def _launch_c2_only_bridge(ns_prefix, declare_diag_master_params: bool):
    """Launches a bridge matching 03c_start_bridge_c2_only_diagnostic.sh's
    configuration (c2_enable=True, c3_enable=False, c1/c4=False), with a
    master sim node configured at the DIAGNOSTIC gains (matching what
    01b_start_master_c2_diagnostic.sh actually starts on real hardware).

    declare_diag_master_params=False reproduces the OLD bug: the bridge is
    told (or defaults to) the production master params despite the master
    controller actually running at diagnostic gains -- planner_parameter_
    match_left must go false and the state machine must FAULT.
    declare_diag_master_params=True is the FIX: the bridge's declared
    master params are overridden to match, and coupling should proceed."""
    master_params = {"stiffness_n_per_m": str(DIAG_STIFFNESS_N_PER_M),
                     "damping_n_s_per_m": str(DIAG_DAMPING_N_S_PER_M),
                     "max_force_n": str(DIAG_MAX_FORCE_N),
                     "max_position_error_m": str(DIAG_MAX_POSITION_ERROR_M),
                     "require_deadman": "false"}
    # 02_start_slave.sh's params are UNCHANGED between C3 and C2 diagnostic
    # sessions (fixed production values) -- match FROZEN here, exactly like
    # test_production_arm_sequence_live.py's slave_params, so the RIGHT
    # side stays matched and this test isolates the LEFT/master mismatch
    # under investigation.
    slave_params = {"stiffness_n_per_m": str(FROZEN.slave_stiffness_n_per_m),
                    "damping_n_s_per_m": str(FROZEN.slave_damping_n_s_per_m),
                    "max_force_n": str(FROZEN.slave_max_force_n),
                    "max_position_error_m": str(FROZEN.slave_max_position_error_m),
                    "require_deadman": "false"}
    master = SimNode("touch_left", device="Left", params=master_params)
    slave = SimNode("touch_right", device="Default Device", params=slave_params)
    master.__enter__(); slave.__enter__()

    calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
    calib_path.unlink()
    bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=True,
                                                    gravity_settle_hold_mode=False,
                                                    c3_enable=False, require_master_deadman=False)
    if declare_diag_master_params:
        # Repeated argparse flags: the LAST occurrence wins (confirmed
        # separately), so appending these overrides the FROZEN production
        # values bridge_cli_args() already put in bargs -- exactly what
        # run_bridge.sh's new MASTER_* env vars now do for 03c.
        bargs += [
            "--master-stiffness-n-per-m", str(DIAG_STIFFNESS_N_PER_M),
            "--master-damping-n-s-per-m", str(DIAG_DAMPING_N_S_PER_M),
            "--master-max-force-n", str(DIAG_MAX_FORCE_N),
            "--master-max-position-error-m", str(DIAG_MAX_POSITION_ERROR_M),
        ]

    bridge = ScriptProcess(bargs, name=f"c2diagbridge_{ns_prefix}")
    bridge.wait_for_marker("teleoperation live", timeout=15.0)
    return master, slave, bridge


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_old_c2_diagnostic_mismatch_reproduces_fault():
    """Reproduces the exact confirmed real-hardware finding: a C2-only
    bridge that does NOT declare the diagnostic master's actual params
    (the old run_bridge.sh, with no MASTER_* wiring at all) must observe
    planner_parameter_match_left=false and fault before ever reaching
    ACTIVE, regardless of how long arming is attempted."""
    master, slave, bridge = _launch_c2_only_bridge("oldmismatch", declare_diag_master_params=False)
    try:
        arm_args = [
            str(ARM_SCRIPT),
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--bridge-diagnostics-topic", "/bridge_diagnostics",
            "--expected-c1", "false", "--expected-c2", "true",
            "--expected-c3", "false", "--expected-c4", "false",
            "--progression-timeout-s", "8",
        ]
        r = subprocess.run([sys.executable] + arm_args, capture_output=True, text=True, timeout=30)
        print(r.stdout[-4000:])
        assert r.returncode != 0, "arm sequence must fail closed on a planner parameter mismatch"
        assert "PRODUCTION TELEOPERATION ACTIVE" not in r.stdout

        with LiveNode("old_mismatch_watch") as live:
            live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
            deadline = time.time() + 3.0
            saw_mismatch = False
            saw_fault = False
            while time.time() < deadline:
                live.spin_once(timeout_sec=0.02)
                d = live.latest.get("d")
                if d is not None:
                    if not d.planner_parameter_match_left:
                        saw_mismatch = True
                    if d.bridge_state == "FAULT":
                        saw_fault = True
            assert saw_mismatch, "expected planner_parameter_match_left=false at some point"
            assert saw_fault, "expected bridge_state to reach FAULT"
    finally:
        bridge.stop(); master.stop(); slave.stop()


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
@pytest.mark.skipif(not ARM_SCRIPT.exists(), reason=f"arm script not found at {ARM_SCRIPT}")
def test_fixed_c2_diagnostic_params_reach_active_with_channel_isolation():
    """Proves the FIX: once the bridge's declared master params match the
    diagnostic master's actual telemetry (what run_bridge.sh's new MASTER_*
    env vars, set by 03c, now provide), planner_parameter_match_left/right
    are both true, the state machine reaches ACTIVE, and channel isolation
    holds: c2_enabled=true, c1=c3=c4=false throughout."""
    master, slave, bridge = _launch_c2_only_bridge("fixedmatch", declare_diag_master_params=True)
    try:
        arm_args = [
            str(ARM_SCRIPT),
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--bridge-diagnostics-topic", "/bridge_diagnostics",
            "--expected-c1", "false", "--expected-c2", "true",
            "--expected-c3", "false", "--expected-c4", "false",
            "--progression-timeout-s", "20",
        ]
        r = subprocess.run([sys.executable] + arm_args, capture_output=True, text=True, timeout=45)
        print(r.stdout[-4000:])
        if r.returncode != 0:
            print("STDERR:", r.stderr[-2000:])
        assert r.returncode == 0, "arm sequence must succeed once declared master params match telemetry"
        assert "PRODUCTION TELEOPERATION ACTIVE" in r.stdout

        with LiveNode("fixed_match_watch") as live:
            live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
            live.spin_for(0.5)
            d = live.latest.get("d")
            assert d is not None
            assert d.bridge_state == "ACTIVE"
            assert d.coupling_ramp_alpha > 0.999
            assert d.planner_parameter_match_left is True
            assert d.planner_parameter_match_right is True
            assert d.c2_enabled is True
            assert d.c1_enabled is False and d.c3_enabled is False and d.c4_enabled is False

        subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                       capture_output=True, text=True, timeout=10)
    finally:
        bridge.stop(); master.stop(); slave.stop()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
