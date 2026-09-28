"""Part 6.A of the 2026-09-09 validation-infrastructure follow-up task:
DEDICATED live-sim test for COUPLING_RAMP_TELEMETRY_STALE.

touch_slave_sim's cartesian_controller_telemetry topic is published
UNCONDITIONALLY every controller tick (not gated by its own
publish_telemetry_ ROS param, which only gates a handful of legacy
Vector3Stamped topics) -- so there is no clean way to stop JUST telemetry
on the real controller's own topic while leaving /pose alone. This test
instead points the bridge's --right-telemetry-topic at a RELAY topic that
this test owns: a small proxy node subscribes to the slave's REAL
cartesian_controller_telemetry topic and republishes it onto the relay
topic only while `relay_active` is True. /pose is untouched (still points
directly at the real SimNode), so this cleanly desyncs telemetry
freshness from pose freshness -- exactly the "stop one telemetry stream"
scenario Part 6.A specifies, without killing the whole SimNode (which
would also stall pose and trigger the state machine's own, DIFFERENT,
pose-staleness FAULT instead).
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
GAP_M = 0.05


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_telemetry_stale_during_coupling_faults_safely():
    mn, sn = unique_ns("tst_m"), unique_ns("tst_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS, sim_env=master_sim_env(GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS, sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        relay_topic = f"/{sn}/telem_relay"

        with LiveNode(f"tst_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)

        args = [str(BRIDGE_SCRIPT),
                "--left-pose-topic", master.topic("pose"),
                "--right-pose-topic", slave.topic("pose"),
                "--right-target-topic", slave.topic("target_pose"),
                "--right-enable-topic", slave.topic("enable"),
                "--left-target-topic", master.topic("target_pose"),
                "--left-enable-topic", master.topic("enable"),
                "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
                "--right-telemetry-topic", relay_topic,  # RELAY, not the real topic
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
                "--force-budget-telemetry-stale-s", "0.2",
                "--force-budget-stall-timeout-s", "1.0",  # fast, this test is not about the timeout magnitude
                "--rate-hz", "100", "--wait-timeout-s", "15",
                ]
        with ScriptProcess(args, name=f"bridge_{mn}") as bridge:
            with LiveNode(f"tst_v_{mn}") as live:
                relay_active = {"on": True}
                relay_pub = live._node.create_publisher(CartesianControllerTelemetry, relay_topic, 10)

                def _relay_cb(msg: CartesianControllerTelemetry) -> None:
                    if relay_active["on"]:
                        relay_pub.publish(msg)
                live._node.create_subscription(
                    CartesianControllerTelemetry, slave.topic("cartesian_controller_telemetry"),
                    _relay_cb, 20)
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)

                bridge.wait_for_marker("teleoperation live", timeout=15.0)
                live.spin_for(1.0)  # let the relay start flowing before arming

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
                cut_t = None
                alpha_at_cut = None
                saw_stale_flag = False
                alpha_at_first_stale = None
                max_alpha_after_stale_detected = 0.0
                deadline = time.time() + 20.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is None:
                        continue
                    if not states or states[-1] != d.bridge_state:
                        states.append(d.bridge_state)
                    if d.bridge_state == "COUPLING_RAMP_UP" and cut_t is None:
                        # Once coupling has genuinely started (alpha has moved
                        # at least once), CUT the relay -- proves this isn't
                        # just "telemetry was never fresh in the first place".
                        if d.coupling_ramp_alpha > 0.0:
                            cut_t = time.time()
                            alpha_at_cut = d.coupling_ramp_alpha
                            relay_active["on"] = False
                    if cut_t is not None and d.bridge_state == "COUPLING_RAMP_UP":
                        if not d.force_budget_telemetry_fresh_right:
                            # The freshness check tolerates up to
                            # --force-budget-telemetry-stale-s of lag by design
                            # (an in-flight message age, not a bug) -- alpha CAN
                            # still advance during that grace window. The hard
                            # requirement is: once staleness is ACTUALLY
                            # DETECTED (this flag first reads False), alpha
                            # must never advance again.
                            if not saw_stale_flag:
                                saw_stale_flag = True
                                alpha_at_first_stale = d.coupling_ramp_alpha
                            max_alpha_after_stale_detected = max(
                                max_alpha_after_stale_detected, d.coupling_ramp_alpha)
                    if d.bridge_state == "FAULT":
                        break

        assert cut_t is not None, f"never observed live coupling progress before cutting the relay: states={states}"
        assert "FAULT" in states, f"expected a FAULT after cutting telemetry, got states={states}"
        assert saw_stale_flag, "force_budget_telemetry_fresh_right should have read False before the FAULT"
        assert max_alpha_after_stale_detected <= alpha_at_first_stale + 1e-9, (
            f"alpha must not advance further once staleness was actually DETECTED: "
            f"alpha_at_first_stale={alpha_at_first_stale}, "
            f"max_alpha_after_stale_detected={max_alpha_after_stale_detected}"
        )
        print(f"[6A] TELEMETRY STALE: states={states}, alpha_at_cut={alpha_at_cut:.4f}, "
              f"alpha_at_first_stale_detected={alpha_at_first_stale:.4f}, "
              f"max_alpha_after_stale_detected={max_alpha_after_stale_detected:.4f} (held) "
              f"-- safe FAULT confirmed, no further alpha progress once staleness detected. PASS")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
