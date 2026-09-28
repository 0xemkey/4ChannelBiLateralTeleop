"""Gravity-compensation arm/ramp/disarm gate, end-to-end in sim (2026-09-09).

Confirms the fix to _publish_gravity_comp()/_publish_left_gravity_comp() (see
bridge_master_slave.py's ramped_gravity_comp_n() docstring) holds through a
real bridge process talking to real touch_slave_sim instances, not just the
pure-function/state-machine unit tests in test_gravity_comp_ramp.py:

    READY (unarmed):  right applied_force_y ~= 0
    ACTIVE:            right applied_force_y ~= SLAVE_GRAVITY_COMP_N
    DISARM:            right applied_force_y ~= 0 again

This is the sim-side regression guard for the real-hardware finding
(2026-09-09 narrow ACTIVE/DISARM check) that the gravity bias was present
identically in READY, ACTIVE, and after DISARM -- i.e. completely outside
the arm/ramp state machine -- before this fix.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped, WrenchStamped
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)

GRAVITY_N = 0.55


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(),
                    reason=f"bridge_master_slave.py not found at {BRIDGE_SCRIPT}")
def test_gravity_comp_gated_by_arm_state():
    master_ns = unique_ns("grav_master")
    slave_ns = unique_ns("grav_slave")

    with SimNode(master_ns, device="Left",
                # max_force_n/max_position_error_m overridden -- see
                # test_c2_live.py's matching comment.
                params={"stiffness_n_per_m": "15.0", "damping_n_s_per_m": "0.10",
                       "max_force_n": "2.0", "max_position_error_m": "0.03",
                       "require_deadman": "false"}) as master, \
        SimNode(slave_ns, device="Default Device",
                # max_force_n overridden to 1.5 (matches run_slave_right.sh's
                # real-hardware default) -- touch_slave.cpp's own default is
                # 0.45N, which silently clamps GRAVITY_N=0.55N before this
                # test can ever observe it, unrelated to the arm-gate fix
                # under test here.
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
            "--slave-force-topic", slave.topic("force_cmd"),
            # 2026-09-09 production-integration task: see test_c3_live.py's
            # matching comment.
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--calibration-file", str(calib_path),
            "--axis-map", "x,y,z",
            "--slave-gravity-comp-n", str(GRAVITY_N),
            "--slave-gravity-axis", "1",
            "--rate-hz", "100",
            "--wait-timeout-s", "15",
            ],
            name=f"bridge_{master_ns}",
        ) as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode("grav_verify") as live:
                # auxiliary_force_n isolates the gravity bias directly (spring
                # + damping are separate fields on the same message) -- avoids
                # needing to wait out C3's own spring convergence to read a
                # clean gravity-only number.
                live.watch("right_telem", slave.topic("cartesian_controller_telemetry"),
                          CartesianControllerTelemetry,
                          extract=lambda m: tuple(m.auxiliary_force_n))
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)

                # READY, unarmed: gravity bias must be ~0, not GRAVITY_N.
                live.spin_for(2.0)
                assert "right_telem" in live.latest, "no /cartesian_controller_telemetry from the slave stand-in"
                ready_y = live.latest["right_telem"][1]
                assert abs(ready_y) < 0.02, (
                    f"READY (unarmed): right applied_force_y={ready_y:.4f}N, expected ~0 -- "
                    f"gravity comp is not gated by arm_teleop (configured {GRAVITY_N}N)"
                )
                print(f"[READY] right applied_force_y={ready_y:.4f}N (expected ~0) PASS")

                r = subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                    capture_output=True, text=True, timeout=10)
                assert "successful" in r.stdout, f"arm_teleop param set failed: {r.stdout} {r.stderr}"

                # Mid-ramp: sample partway through gravity_ramp_duration_s
                # (0.3s default, 2026-09-09 staged arm/disarm transition --
                # gravity now ramps in its OWN short phase, separate from
                # and well before the 1.0s bilateral-coupling ramp; see
                # startup_alignment.py's module docstring).
                live.spin_for(0.12)
                mid_y = live.latest["right_telem"][1]
                print(f"[MID-RAMP ~0.12s] right applied_force_y={mid_y:.4f}N "
                      f"(expect strictly between 0 and {GRAVITY_N})")
                assert 0.0 < mid_y < GRAVITY_N - 0.02, (
                    f"mid-ramp gravity comp should be a partial value, got {mid_y:.4f}N -- "
                    "looks like an instantaneous step rather than a ramp"
                )

                # Let the full staged arm sequence (gravity ramp + settle +
                # coupling ramp) finish -- poll for ACTIVE rather than a
                # fixed spin_for(1.5): 2026-09-09's velocity-gated
                # GRAVITY_SETTLE (see startup_alignment.py) makes this
                # duration variable (waits for genuine mechanical
                # quiescence, not a fixed clock).
                deadline = time.time() + 15.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is not None and d.bridge_state == "ACTIVE":
                        break
                    if d is not None and d.bridge_state == "FAULT":
                        raise AssertionError(f"bridge FAULTed while waiting for ACTIVE: {d}")
                else:
                    raise AssertionError("bridge never reached ACTIVE within 15s")
                active_y = live.latest["right_telem"][1]
                assert abs(active_y - GRAVITY_N) < 0.02, (
                    f"ACTIVE: right applied_force_y={active_y:.4f}N, expected ~{GRAVITY_N}N"
                )
                print(f"[ACTIVE] right applied_force_y={active_y:.4f}N (expected ~{GRAVITY_N}N) PASS")

                r = subprocess.run(
                    ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                    capture_output=True, text=True, timeout=10)
                assert "successful" in r.stdout, f"disarm param set failed: {r.stdout} {r.stderr}"
                # 2026-09-09 staged transition: disarm now takes
                # coupling_ramp_down_duration_s (1.0s) + self_follow_settle_
                # duration_s (0.2s) + gravity_ramp_down_duration_s (0.3s)
                # before gravity actually reaches 0 -- coupling backs off
                # FIRST, with gravity deliberately held constant during that
                # phase (the whole point of the redesign; see startup_
                # alignment.py's module docstring), so a naive 1.0s wait
                # would catch gravity still at full value.
                live.spin_for(1.8)
                disarmed_y = live.latest["right_telem"][1]
                assert abs(disarmed_y) < 0.02, (
                    f"DISARM: right applied_force_y={disarmed_y:.4f}N, expected ~0"
                )
                print(f"[DISARM] right applied_force_y={disarmed_y:.4f}N (expected ~0) PASS")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
