"""Part C (2026-09-10, real-hardware safety completion): live-sim test for
the bridge-level master deadman gate, against the REAL bridge_master_slave.py
and REAL touch_slave_sim controllers.

Covers: arm rejected with no button telemetry, arm succeeds once fresh
telemetry exists, press->enable=True, release->enable=False (not latched,
resumes on re-press), stale telemetry (process-loss stand-in: simply stop
publishing) -> latched disable that does NOT self-clear even once
telemetry resumes, and an explicit re-arm (fresh telemetry required)
clearing the latch.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from std_msgs.msg import Int32, Bool
from touch_haptics.msg import BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

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
DEADMAN_MASK = 1


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_master_deadman_gate_press_release_stale_rearm():
    mn, sn = unique_ns("dm_m"), unique_ns("dm_s")
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

        bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False,
                                                        gravity_settle_hold_mode=False)
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
        # DEDICATED test-only topic -- NOT the sim device's own native
        # /touch_left/buttons (which touch_slave_sim publishes continuously,
        # at ~200Hz, on its own, always reporting SOME value -- pointing the
        # bridge at that topic would mean telemetry can never actually go
        # stale for this test, AND would be exactly the kind of unintended
        # dual-publisher conflict Part F/item 18 warns about). This test
        # owns the only publisher on this topic.
        test_buttons_topic = master.topic("test_deadman_buttons")
        bargs += ["--left-buttons-topic", test_buttons_topic,
                 "--require-master-deadman", "true",
                 "--master-deadman-button-mask", str(DEADMAN_MASK),
                 "--master-deadman-stale-s", "1.0"]

        with ScriptProcess(bargs, name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"dm_v_{mn}") as live:
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                slave_enable_seen = {"v": None}
                live._node.create_subscription(
                    Bool, slave.topic("enable"), lambda m: slave_enable_seen.__setitem__("v", m.data), 10)
                buttons_pub = live._node.create_publisher(Int32, test_buttons_topic, 10)

                def _publish_buttons(value):
                    m = Int32(); m.data = value
                    buttons_pub.publish(m)

                live.spin_for(0.5)

                # ---- 1. arm REJECTED with no button telemetry ever sent ----
                r = subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                    capture_output=True, text=True, timeout=10)
                assert "successful" not in r.stdout, (
                    f"arm should have been REJECTED with no button telemetry, got: {r.stdout}")
                print(f"[1] arm rejected with no button telemetry: {r.stdout.strip()}. PASS")

                # ---- 2. start publishing PRESSED, arm should now succeed ----
                for _ in range(20):
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_for(0.02)
                r = ""
                full = ""
                for _ in range(10):
                    resp = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    r = resp.stdout
                    full = resp.stdout + resp.stderr
                    if "successful" in r:
                        break
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_for(0.1)
                assert "successful" in r, f"arm should succeed with fresh pressed telemetry: {full!r}"
                print("[2] arm succeeds once button telemetry is fresh and pressed. PASS")

                # ---- 3. keep holding, confirm slave enable reads True ----
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is True, (
                    f"slave enable should be True while deadman held, got {slave_enable_seen['v']}"
                )
                print("[3] slave enable=True while deadman continuously held. PASS")

                # ---- 4. release -> enable goes False (not latched) ----
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    _publish_buttons(0)  # released, but telemetry still flowing
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is False, "enable should be False on release"
                print("[4] release -> slave enable=False. PASS")

                # ---- 5. re-press (telemetry never went stale) -> resumes WITHOUT re-arm ----
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is True, (
                    "re-press with telemetry never stale should resume without re-arm"
                )
                print("[5] re-press (no stale gap) resumes immediately without re-arm. PASS")

                # ---- 6. STOP publishing entirely (process-loss stand-in) -> stale -> latched disable ----
                deadline = time.time() + 2.0  # > master-deadman-stale-s=1.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.02)
                assert slave_enable_seen["v"] is False, "stale button telemetry must disable"
                print("[6] button telemetry stopped (process-loss stand-in) -> enable=False. PASS")

                # ---- 7. telemetry RESUMES (pressed) but latch must NOT auto-clear ----
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is False, (
                    "must NOT auto-resume just because telemetry recovered -- requires explicit re-arm"
                )
                print("[7] telemetry recovered (pressed) but stays disabled -- no automatic re-arm. PASS")

                # ---- 8. explicit re-arm (disarm, then arm again with fresh telemetry) clears the latch ----
                subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                               capture_output=True, text=True, timeout=10)
                live.spin_for(0.3)
                r = ""
                for _ in range(10):
                    resp = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    r = resp.stdout
                    if "successful" in r:
                        break
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_for(0.1)
                assert "successful" in r, f"re-arm should succeed with fresh telemetry: {r}"
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is True, "explicit re-arm should clear the latch and resume"
                print("[8] explicit re-arm (fresh telemetry) clears the latch, resumes. PASS")

                # ---- 9. FAULT/disarm still overrides even with deadman held ----
                subprocess.run(["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                               capture_output=True, text=True, timeout=10)
                deadline = time.time() + 5.0
                disarmed = False
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is not None and d.bridge_state == "READY":
                        disarmed = True
                        break
                    time.sleep(0.01)
                assert disarmed, "disarm should complete to READY even with deadman held"
                print("[9] disarm completes to READY even while deadman is held. PASS")


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_master_deadman_gate_fault_overrides_held_deadman():
    """Distinct from ordinary disarm (test above, step 9): a bridge FAULT
    (here, a genuine parameter mismatch, same technique as
    test_fail_safes_live.py) must disable the slave regardless of the
    deadman being continuously held throughout."""
    mn, sn = unique_ns("dmf_m"), unique_ns("dmf_s")
    params = {"stiffness_n_per_m": str(FROZEN.master_stiffness_n_per_m),
             "damping_n_s_per_m": str(FROZEN.master_damping_n_s_per_m),
             "max_force_n": str(FROZEN.master_max_force_n),
             "max_position_error_m": str(FROZEN.master_max_position_error_m),
             "require_deadman": "false"}
    # Deliberately mismatched slave Fmax (declared vs actual), same
    # technique as test_fail_safes_live.py's PARAMETER MISMATCH case.
    sparams = {"stiffness_n_per_m": str(FROZEN.slave_stiffness_n_per_m),
              "damping_n_s_per_m": str(FROZEN.slave_damping_n_s_per_m),
              "max_force_n": "1.5",
              "max_position_error_m": str(FROZEN.slave_max_position_error_m),
              "require_deadman": "false"}
    with SimNode(mn, device="Left", params=params) as master, \
        SimNode(sn, device="Default Device", params=sparams) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False,
                                                        gravity_settle_hold_mode=False)
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
        # Declared mismatch: bridge told 1.2N, controller actually running 1.5N.
        _retopic(bargs, "--slave-max-force-n", "1.2")
        test_buttons_topic = master.topic("test_deadman_buttons")
        bargs += ["--left-buttons-topic", test_buttons_topic,
                 "--require-master-deadman", "true",
                 "--master-deadman-button-mask", str(DEADMAN_MASK),
                 "--master-deadman-stale-s", "1.0"]

        with ScriptProcess(bargs, name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"dmf_v_{mn}") as live:
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                slave_enable_seen = {"v": None}
                live._node.create_subscription(
                    Bool, slave.topic("enable"), lambda m: slave_enable_seen.__setitem__("v", m.data), 10)
                buttons_pub = live._node.create_publisher(Int32, test_buttons_topic, 10)

                def _publish_buttons(value):
                    m = Int32(); m.data = value
                    buttons_pub.publish(m)

                live.spin_for(0.5)
                for _ in range(20):
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_for(0.02)

                out = ""
                for _ in range(10):
                    r = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    out = r.stdout
                    if "successful" in out:
                        break
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_for(0.1)
                assert "successful" in out, f"could not arm: {out!r}"

                states = []
                reached_fault = False
                deadline = time.time() + 20.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)  # held CONTINUOUSLY throughout
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is not None:
                        if not states or states[-1] != d.bridge_state:
                            states.append(d.bridge_state)
                        if d.bridge_state == "FAULT":
                            reached_fault = True
                            break
                    time.sleep(0.005)
                assert reached_fault, f"expected a FAULT (parameter mismatch), states={states}"

                # Deadman still held -- confirm the slave is disabled anyway,
                # within a short settling window.
                deadline = time.time() + 1.0
                while time.time() < deadline:
                    _publish_buttons(DEADMAN_MASK)
                    live.spin_once(timeout_sec=0.01)
                    time.sleep(0.01)
                assert slave_enable_seen["v"] is False, (
                    f"slave must be disabled during FAULT even with deadman held, "
                    f"got enable={slave_enable_seen['v']}"
                )
                print(f"[FAULT] states={states}, slave enable=False during FAULT despite deadman held. PASS")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
