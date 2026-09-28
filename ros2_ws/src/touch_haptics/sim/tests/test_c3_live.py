"""C3 (master position -> slave target) end-to-end regression, in sim.

C3 is already confirmed on real hardware (5 live single-axis-motion tests,
2026-09-01, see docs/FOUR_CHANNEL_TODO.md) -- this is the sim-side
regression guard so a future change to bridge_master_slave.py or
touch_slave.cpp's Cartesian path can't silently break it again without a
test noticing, matching test_c2_live.py's shape (see that file's docstring
for why a continuous LiveNode is used instead of two timed snapshots).

Also confirms C3 keeps working with the default axis_map=x,y,z -- do not
reintroduce the old, never-validated x,z,-y override (see the
axis-mapping-unresolved-bug project memory).
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
from touch_haptics.msg import BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(),
                    reason=f"bridge_master_slave.py not found at {BRIDGE_SCRIPT}")
def test_c3_slave_target_tracks_master_position():
    master_ns = unique_ns("c3_master")
    slave_ns = unique_ns("c3_slave")

    # require_deadman defaults to true in touch_slave.cpp -- without this
    # override, force_cmd is silently gated off (see test_c2_live.py's
    # matching comment for how that showed up).
    with SimNode(master_ns, device="Left",
                # max_force_n/max_position_error_m overridden -- see
                # test_c2_live.py's matching comment.
                params={"stiffness_n_per_m": "15.0", "damping_n_s_per_m": "0.10",
                       "max_force_n": "2.0", "max_position_error_m": "0.03",
                       "require_deadman": "false"}) as master, \
        SimNode(slave_ns, device="Default Device",
                # max_force_n/max_position_error_m overridden to match the
                # bridge's own --slave-max-force-n/--slave-max-position-
                # error-m defaults (1.5N/0.04m) -- see test_c2_live.py's
                # matching master-side comment.
                params={"stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                       "max_force_n": "1.5", "max_position_error_m": "0.04",
                       "require_deadman": "false"}) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with ScriptProcess(
            [str(BRIDGE_SCRIPT),
            "--left-pose-topic", master.topic("pose"),
            "--right-pose-topic", slave.topic("pose"),
            "--right-target-topic", slave.topic("target_pose"),
            "--right-enable-topic", slave.topic("enable"),
            # 2026-09-09 production-integration task: the bilateral force-
            # budget planner (COUPLING_RAMP_UP) needs BOTH arms' telemetry
            # to avoid holding alpha / faulting on stale telemetry -- must
            # point at this test's own namespaced topics, not the (here,
            # nonexistent) real-hardware default.
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--calibration-file", str(calib_path),
            "--axis-map", "x,y,z",
            "--rate-hz", "100",
            "--wait-timeout-s", "15",
            ],
            name=f"bridge_{master_ns}",
        ) as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            # startup_alignment.py's READY->RAMPING->ACTIVE gate (added
            # 2026-09-08, after this test was first written) blends C3's
            # commanded target from right_anchor (frozen, zero relative
            # motion) toward the real target only once armed -- without
            # this, the target legitimately never moves regardless of how
            # far the master is pushed, which is exactly what a 2026-09-09
            # root-cause investigation found this test silently hitting
            # (right_target frozen bit-for-bit at right_anchor for the
            # entire test). This is the same one-line fix used throughout
            # that investigation's own scripts. See
            # docs/... (bridge/C2-C3 frame-mixing root-cause report) for
            # the full trace -- this is a STALE TEST ASSUMPTION fix, not a
            # bridge code change.
            r = subprocess.run(
                ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                capture_output=True, text=True, timeout=10)
            assert "successful" in r.stdout, f"arm_teleop param set failed: {r.stdout} {r.stderr}"

            with LiveNode("c3_verify") as live:
                live.watch("left_pose", master.topic("pose"), PoseStamped,
                          extract=lambda m: (m.pose.position.x, m.pose.position.y, m.pose.position.z))
                live.watch("right_target", slave.topic("target_pose"), PoseStamped,
                          extract=lambda m: (m.pose.position.x, m.pose.position.y, m.pose.position.z))
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)

                # Wait for ACTIVE (bridge_state), not a fixed spin_for(3.0) --
                # 2026-09-09's velocity-gated GRAVITY_SETTLE (see
                # startup_alignment.py) makes the total arm-sequence duration
                # variable (waits for genuine mechanical quiescence, not a
                # fixed clock), so the push-force comparison below must not
                # assume a fixed schedule has completed by a fixed wall time.
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
                assert "left_pose" in live.latest, "no /pose from the master stand-in"
                assert "right_target" in live.latest, "no C3 output on the slave's target_pose"
                baseline_left = live.latest["left_pose"]
                baseline_right_target = live.latest["right_target"]

                end_time = time.time() + 4.0
                while time.time() < end_time:
                    live.push_force(master.topic("force_cmd"), master.topic("enable"), 0.0, 0.35, 0.0)
                    live.spin_once(timeout_sec=0.01)

                # Settle before comparing, WITHOUT releasing force -- see
                # test_c2_live.py's matching comment. Releasing to zero here
                # lets the master drift back toward its own gravity/friction
                # rest pose (confirmed live: with no target commanded on the
                # master, nothing else holds it, so releasing undoes almost
                # the entire push before the final read).
                settle_end = time.time() + 4.0
                while time.time() < settle_end:
                    live.push_force(master.topic("force_cmd"), master.topic("enable"), 0.0, 0.35, 0.0)
                    live.spin_once(timeout_sec=0.01)

                after_left = live.latest["left_pose"]
                after_right_target = live.latest["right_target"]

                left_delta = [after_left[i] - baseline_left[i] for i in range(3)]
                right_delta = [after_right_target[i] - baseline_right_target[i] for i in range(3)]

                moved = sum(abs(v) for v in left_delta)
                assert moved > 0.005, (
                    f"master stand-in barely moved ({moved*1000:.2f}mm total) -- "
                    "push had no effect, cannot judge C3 tracking"
                )
                # Proportional, not bit-exact -- see test_c1_relay_live.py's
                # and test_c2_live.py's matching comments (real, load-
                # dependent residual gap between two independent processes,
                # not a bridge bug). Check the dominant (actually pushed)
                # axis, not every axis equally.
                dominant = max(range(3), key=lambda i: abs(left_delta[i]))
                assert abs(left_delta[dominant]) > 1e-3, "no dominant axis moved meaningfully"
                ratio = right_delta[dominant] / left_delta[dominant]
                assert 0.5 < ratio < 1.5, (
                    f"C3 mismatch on the dominant axis ({'xyz'[dominant]}): "
                    f"master moved {left_delta[dominant]*1000:+.2f}mm, slave's C3 "
                    f"target moved {right_delta[dominant]*1000:+.2f}mm, ratio={ratio:.2f} "
                    f"(left_delta={left_delta}, right_delta={right_delta})"
                )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
