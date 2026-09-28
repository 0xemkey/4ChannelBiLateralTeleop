"""Regression test for the 2026-09-10 preflight.py node-name bug fix
(H2 pre-arm/post-disarm safety-hardening task): before the fix,
preflight.py's `bridge_present` check searched live node names for the
substring "bridge_master_slave", but bridge_master_slave.py's actual ROS
node name is "touch_master_slave_bridge" (different word order) -- the
substring never matched, so preflight.py could report IDLE_TOPOLOGY_PASS
even while the bridge was actively running. This test launches the REAL
bridge_master_slave.py (topics retargeted to a namespaced sim pair, exactly
like every other live-sim test in this directory -- its NODE NAME is not
affected by topic remaps, so this exercises the exact node-name matching
preflight.py performs) and confirms `preflight.py`'s own CLI output now
correctly reports the bridge as present.

DOMAIN ISOLATION: same convention as test_manual_motion_recorder_live.py --
this machine has real hardware driver nodes running persistently on the
default ROS domain, so ROS_DOMAIN_ID is forced to an isolated value BEFORE
any rclpy-touching import/subprocess spawn, keeping this test's real
(but sim-topic-retargeted) bridge process fully separate from them.
"""
from __future__ import annotations

import os
os.environ["ROS_DOMAIN_ID"] = "42"

import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import ScriptProcess, SimNode, unique_ns  # noqa: E402

HW_DIR = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_experiments"
    / "2026-09-09_y_axis_validation_PENDING_SUPERVISION"
)
sys.path.insert(0, str(HW_DIR))
from frozen_config import FROZEN, bridge_cli_args  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)
PREFLIGHT_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_validation" / "preflight.py"
)


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
@pytest.mark.skipif(not PREFLIGHT_SCRIPT.exists(), reason=f"preflight not found at {PREFLIGHT_SCRIPT}")
def test_preflight_detects_running_bridge_by_correct_node_name():
    mn, sn = unique_ns("pf_m"), unique_ns("pf_s")
    params = {"stiffness_n_per_m": str(FROZEN.master_stiffness_n_per_m),
             "damping_n_s_per_m": str(FROZEN.master_damping_n_s_per_m),
             "max_force_n": str(FROZEN.master_max_force_n),
             "max_position_error_m": str(FROZEN.master_max_position_error_m),
             "require_deadman": "false"}
    sparams = {"stiffness_n_per_m": str(FROZEN.slave_stiffness_n_per_m),
              "damping_n_s_per_m": str(FROZEN.slave_damping_n_s_per_m),
              "max_force_n": str(FROZEN.slave_max_force_n),
              "max_position_error_m": str(FROZEN.slave_max_position_error_m),
              "require_deadman": "false"}
    with SimNode(mn, device="Left", params=params) as master, \
        SimNode(sn, device="Default Device", params=sparams) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()
        bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False)

        def _retopic(args, flag, new_val):
            i = args.index(flag)
            args[i + 1] = new_val
        _retopic(bargs, "--left-pose-topic", master.topic("pose"))
        _retopic(bargs, "--right-pose-topic", slave.topic("pose"))
        _retopic(bargs, "--right-target-topic", slave.topic("target_pose"))
        _retopic(bargs, "--right-enable-topic", slave.topic("enable"))
        _retopic(bargs, "--left-target-topic", master.topic("target_pose"))
        _retopic(bargs, "--left-enable-topic", master.topic("enable"))
        _retopic(bargs, "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"))
        _retopic(bargs, "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"))
        _retopic(bargs, "--slave-force-topic", slave.topic("force_cmd"))

        with ScriptProcess(bargs, name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            r = subprocess.run([sys.executable, str(PREFLIGHT_SCRIPT)],
                               capture_output=True, text=True, timeout=15)
            print(r.stdout)
            assert "touch_master_slave_bridge present: True" in r.stdout, (
                "preflight.py failed to detect the running bridge by its correct node "
                f"name -- this is exactly the bug fixed 2026-09-10. stdout:\n{r.stdout}"
            )
            assert "PREFLIGHT MODE: STAGE_LAUNCHED_UNARMED" in r.stdout or \
                   "PREFLIGHT MODE: ARMED_TOPOLOGY_ELIGIBLE" in r.stdout, (
                f"expected a non-idle preflight mode while the bridge is running, got:\n{r.stdout}"
            )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
