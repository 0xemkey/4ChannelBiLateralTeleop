"""Live-sim end-to-end test for hardware_validation/record_manual_motion.py
+ analyze_manual_motion.py (2026-09-10, H3/H4 recorder tooling task).

Reuses test_part3_c3_slave_tracking_sim.py's sine-hand-motion technique
(TOUCH_SIM_HAND_MOTION=sine) as the "manual operator motion" stand-in --
the recorder itself never drives the master (subscribe-only, see
test_manual_motion_recorder.py's static assertion); this test's sim-side
Hand model is what moves it, exactly the same role a real operator's hand
plays on real hardware.

Runs record_manual_motion.py as a REAL subprocess (not just importing its
functions) against the live bridge + sim devices, confirming the actual
CLI entrypoint produces a valid, analyzable dataset end-to-end.

DOMAIN ISOLATION (section 17, required): this machine was found, while
writing this test, to have the REAL hardware driver nodes
(teleop_master_left / teleop_slave_right, touch_slave binaries, idle with
enabled_on_start=false) running persistently on the default ROS domain
(confirmed via `ps aux` -- these are long-running background processes
from an earlier session, not something this task started). The recorder's
own live-hardware refusal check (record_manual_motion.py's
_refuse_if_live_hardware_attached) correctly detected this and refused to
start the FIRST time this test was run without domain isolation -- proof
the safety check works, not a bug. ROS_DOMAIN_ID is therefore forced to a
dedicated value below, BEFORE any rclpy-touching import, so every
subprocess this test spawns (which all inherit os.environ) is fully
isolated from that live domain and never has a chance to observe or be
observed by the real hardware nodes.
"""
from __future__ import annotations

import os
os.environ["ROS_DOMAIN_ID"] = "42"  # must happen before any rclpy-touching
                                    # import/subprocess spawn -- see module
                                    # docstring above.

import json
import subprocess
import sys
import tempfile
import time
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

HV_DIR = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_validation"
)
sys.path.insert(0, str(HV_DIR))
from analyze_manual_motion import analyze  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)
RECORDER_SCRIPT = HV_DIR / "record_manual_motion.py"

HAND_AMPLITUDE_M = 0.005
HAND_F0_HZ = 0.1
RECORD_DURATION_S = 14.0  # > 1 full 10s cycle at 0.1Hz


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
@pytest.mark.skipif(not RECORDER_SCRIPT.exists(), reason=f"recorder not found at {RECORDER_SCRIPT}")
def test_manual_motion_recorder_end_to_end_c3_only():
    mn, sn = unique_ns("mm_m"), unique_ns("mm_s")
    master_params = {
        "stiffness_n_per_m": str(FROZEN.master_stiffness_n_per_m),
        "damping_n_s_per_m": str(FROZEN.master_damping_n_s_per_m),
        "max_force_n": str(FROZEN.master_max_force_n),
        "max_position_error_m": str(FROZEN.master_max_position_error_m),
        "require_deadman": "false",
    }
    slave_params = {
        "stiffness_n_per_m": str(FROZEN.slave_stiffness_n_per_m),
        "damping_n_s_per_m": str(FROZEN.slave_damping_n_s_per_m),
        "max_force_n": str(FROZEN.slave_max_force_n),
        "max_position_error_m": str(FROZEN.slave_max_position_error_m),
        "require_deadman": "false",
    }
    master_sim_env = {
        "TOUCH_SIM_INIT_POS_X": "-0.10", "TOUCH_SIM_INIT_POS_Y": "-0.21", "TOUCH_SIM_INIT_POS_Z": "0.0",
        "TOUCH_SIM_HAND": "stiff",
        "TOUCH_SIM_HAND_MOTION": "sine",
        "TOUCH_SIM_HAND_AMP": str(HAND_AMPLITUDE_M),
        "TOUCH_SIM_HAND_F0": str(HAND_F0_HZ),
    }

    with SimNode(mn, device="Left", params=master_params, sim_env=master_sim_env) as master, \
        SimNode(sn, device="Default Device", params=slave_params) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False,
                                                        gravity_settle_hold_mode=False,
                                                        c3_enable=True)

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

            out = ""
            deadline = time.time() + 10.0
            while time.time() < deadline:
                r = subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                    capture_output=True, text=True, timeout=10)
                out = r.stdout
                if "successful" in out:
                    break
                time.sleep(0.3)
            assert "successful" in out, f"could not arm: {out!r}"

            # Wait for ACTIVE via a `ros2 topic echo` subprocess (NOT an
            # in-process rclpy Node) -- every subprocess this test spawns
            # inherits os.environ (ROS_DOMAIN_ID=42, set at module import
            # time above), which keeps this poll on the isolated domain
            # regardless of whatever domain rclpy.init() in THIS process
            # was already bound to by an earlier test in the same pytest
            # session (rclpy's domain is fixed at first init() and cannot
            # be changed afterward within one process).
            reached_active = False
            deadline = time.time() + 20.0
            while time.time() < deadline:
                r = subprocess.run(
                    ["ros2", "topic", "echo", "--once", "/bridge_diagnostics"],
                    capture_output=True, text=True, timeout=5)
                if "bridge_state: ACTIVE" in r.stdout:
                    reached_active = True
                    break
                time.sleep(0.3)
            assert reached_active, "bridge never reached ACTIVE before starting the recorder"

            out_dir = Path(tempfile.mkdtemp(prefix="mm_test_"))
            rec_args = [
                str(RECORDER_SCRIPT),
                "--out-dir", str(out_dir),
                "--duration-s", str(RECORD_DURATION_S),
                "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
                "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
                "--bridge-diagnostics-topic", "/bridge_diagnostics",
                "--bilateral-status-topic", "/bilateral_status_unused_for_this_test",
                "--operator-mode", "single_operator",
                "--expected-stage", "H3",
                "--expected-c3", "true",
                "--notes", "live-sim end-to-end validation, sine-hand stand-in for manual motion",
            ]
            with ScriptProcess(rec_args, name=f"recorder_{mn}") as recorder:
                recorder.wait_for_marker("recording for", timeout=10.0)
                deadline = time.time() + RECORD_DURATION_S + 15.0
                while recorder.proc.poll() is None and time.time() < deadline:
                    time.sleep(0.2)
                assert recorder.proc.returncode == 0, (
                    f"recorder exited nonzero: {recorder.proc.returncode}\n{recorder.log_text()}"
                )

            subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                           capture_output=True, text=True, timeout=10)

    assert (out_dir / "manifest.json").exists()
    assert (out_dir / "raw_master.csv").exists()
    assert (out_dir / "raw_slave.csv").exists()
    manifest = json.loads((out_dir / "manifest.json").read_text())
    assert manifest["stream_stats"]["master"]["n"] > 100
    assert manifest["stream_stats"]["slave"]["n"] > 100
    assert manifest["master_anchor_native_m"] is not None
    assert manifest["slave_anchor_native_m"] is not None

    result = analyze(out_dir)
    print(f"[manual_motion e2e] validation={result['validation']['overall_result']}")
    print(f"[manual_motion e2e] metrics={json.dumps(result['metrics'], indent=2, default=str)}")
    assert not result["event_result"]["insufficient_excitation"], (
        "sine-hand motion at 5mm amplitude should have produced sufficient "
        "excitation for event detection"
    )
    assert result["metrics"]["result"] == "EVALUATED"
    assert (out_dir / "merged.csv").exists()
    assert (out_dir / "events.json").exists()
    assert (out_dir / "metrics.json").exists()
    assert (out_dir / "validation.json").exists()
    assert (out_dir / "plots" / "01_master_vs_slave_y.png").exists()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
