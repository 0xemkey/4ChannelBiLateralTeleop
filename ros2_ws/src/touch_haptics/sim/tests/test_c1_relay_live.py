"""C1 (slave force -> master) relay, end-to-end in sim, plus regression
coverage for two real bugs found while building this test on 2026-09-02:

  1. bilateral_4ch.py's oscillation-abort "growing" check (3 consecutive
    rising 1-second peak-to-peak windows) false-triggered on the ordinary
    onset of ANY signal appearing from a near-zero baseline -- confirmed
    live with both an abrupt force step and a gentle 3-second ramp,
    tripping at the identical moment on a tiny pp=0.0165 N*m against a
    0.4 N*m threshold. Fixed by requiring the latest window to also cross
    a magnitude floor (25% of trip_pp_nm) before "growing" counts.
  2. `ros2 param set /bilateral_4ch manual_rearm true` -- the documented way
    to clear a trip -- crashed the node with AttributeError: 'bool' object
    has no attribute 'bool_value'. rclpy's parameter callback already hands
    back the unwrapped Python bool via Parameter.value, not a raw
    ParameterValue needing .bool_value. Fixed: p.value.bool_value -> p.value.

Two touch_slave_sim instances stand in for master/slave. No wall/hand model
is used -- a sustained force_cmd on the slave is itself indistinguishable
from an external disturbance to the observer (that is the whole basis of
the DOB), so this is simpler and pose-independent compared to relying on a
wall-plane contact, which turned out to be sensitive to wherever the sim's
random settle happened to land relative to the plane (see the 2026-09-02
session notes' "connected-channels test").
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import pytest
from touch_haptics.msg import ForceEstimate

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

BILATERAL_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)

_fe_deadband = lambda m: tuple(m.tau_ext_deadbanded_nm)  # noqa: E731


def _launch_pair():
    """Two touch_slave_sim instances (master + slave), DOB enabled on both --
    force_ext_hat is computed unconditionally whenever the bank is
    configured, regardless of joint_space_control, so neither node needs
    that flag just for this test to observe the estimate/relay topics.

    The slave gets a wall contact model (environment.hpp: contact requires
    y < TOUCH_SIM_WALL_Y, i.e. MORE negative -- confirmed live by direct
    measurement, not assumed; get this backwards and every push below
    exits contact instead of deepening it and force_ext_hat reads exactly
    zero throughout, which is what happened during this test's own
    development). A commanded force_cmd push alone is NOT a valid stand-in
    for contact (also confirmed live: peak_right stayed exactly 0.0 across
    4s of pushing with no wall): force_cmd is fully accounted for in
    tau_cmd before the observer ever sees it, so by definition it cannot
    appear as an unmodeled external disturbance -- only a genuine
    environment contact (or a hand model, see HandParams) can. The push
    below (negative Y) is only the DRIVER that deepens real wall contact,
    not the disturbance signal itself.

    The sim's natural random settle already lands within a few mm of the
    default wall plane (TOUCH_SIM_WALL_Y=-0.05) often enough to make this
    reliable without explicitly seeding a start pose -- an attempt to seed
    one directly (TOUCH_SIM_INIT_POS_Y=-0.09) during this test's own
    development landed outside this arm's IK-reachable region ("outside
    this model's reach at this pose" in the sim's own log) and made things
    worse, not better; left out deliberately.
    """
    master_ns = unique_ns("c1_master")
    slave_ns = unique_ns("c1_slave")
    # require_deadman defaults to true in touch_slave.cpp -- without this
    # override, the push below does nothing (see test_c2_live.py's comment).
    master = SimNode(master_ns, device="Left",
                    params={"dob_enable": "true", "require_deadman": "false"})
    slave = SimNode(slave_ns, device="Default Device",
                    params={"dob_enable": "true", "require_deadman": "false",
                           "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
                           "max_force_n": "1.5"},
                    sim_env={
                        "TOUCH_SIM_ENV": "wall",
                        "TOUCH_SIM_WALL_Y": "-0.05",
                        "TOUCH_SIM_WALL_K": "2000",
                    })
    master.wait_started()
    slave.wait_started()
    for sim in (master, slave):
        assert "DOB bank configured" in sim.log_text(), (
            f"{sim.namespace}: DOB never configured (missing "
            f"~/dob_logs/results/*.json?); see {sim.log_text()[-800:]}"
        )
    return master, slave


@pytest.mark.skipif(not BILATERAL_SCRIPT.exists(),
                    reason=f"bilateral_4ch.py not found at {BILATERAL_SCRIPT}")
def test_c1_relay_matches_slave_force_estimate():
    master, slave = _launch_pair()
    try:
        status_topic = f"/{unique_ns('c1_status')}"
        with ScriptProcess(
            [str(BILATERAL_SCRIPT),
            "--architecture", "custom", "--w1", "1.0", "--w2", "0.0",
            "--w3", "0.0", "--w4", "0.0",
            "--left-force-topic", master.topic("force_ext_hat"),
            "--right-force-topic", slave.topic("force_ext_hat"),
            "--left-joint-angles-topic", master.topic("joint_angles"),
            "--right-joint-angles-topic", slave.topic("joint_angles"),
            "--left-joint-target-topic", master.topic("joint_target"),
            "--right-joint-target-topic", slave.topic("joint_target"),
            "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
            "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
            "--status-topic", status_topic,
            "--rate-hz", "200",
            ],
            name=f"bilateral_{master.namespace}",
        ) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            with LiveNode("c1_verify") as live:
                live.watch("right_fe", slave.topic("force_ext_hat"), ForceEstimate, extract=_fe_deadband)
                live.watch("left_ext", master.topic("joint_torque_ext_cmd"), ForceEstimate, extract=_fe_deadband)

                live.spin_for(2.0)

                end_time = time.time() + 5.0
                peak_right = 0.0
                peak_left = 0.0
                while time.time() < end_time:
                    live.push_force(slave.topic("force_cmd"), slave.topic("enable"), 0.0, -0.5, 0.0)
                    live.spin_once(timeout_sec=0.01)
                    rf = live.latest.get("right_fe")
                    lf = live.latest.get("left_ext")
                    if rf is not None:
                        peak_right = max(peak_right, sum(v * v for v in rf) ** 0.5)
                    if lf is not None:
                        peak_left = max(peak_left, sum(v * v for v in lf) ** 0.5)

                assert "bilateral_4ch started" in bilateral.log_text()
                assert "OSCILLATION ABORT" not in bilateral.log_text(), (
                    f"unexpected oscillation-abort trip during an ordinary push; "
                    f"see {bilateral._log_path}"
                )
                assert peak_right > 0.002, (
                    f"slave's force_ext_hat never registered the push "
                    f"(peak={peak_right:.5f} N*m) -- DOB not producing a signal"
                )

                # Settle before the final comparison -- same reasoning as
                # test_c2_live.py's matching comment (independent publish
                # rates between the slave's observer and bilateral_4ch's own
                # relay tick can catch "latest" a sub-tick apart mid-motion).
                settle_end = time.time() + 3.0
                while time.time() < settle_end:
                    live.push_force(slave.topic("force_cmd"), slave.topic("enable"), 0.0, -0.5, 0.0)
                    live.spin_once(timeout_sec=0.01)

                final_right = live.latest["right_fe"]
                final_left = live.latest["left_ext"]
                # Proportional, not bit-exact: bilateral_4ch's C1 path has no
                # filtering of its own (tau_c1 = w1*cf*tau_e_hat, a direct
                # multiply -- checked in bilateral_4ch.py, not assumed), but
                # this is still two independent processes each subscribing to
                # the DOB's output on their own schedule, and under load
                # (confirmed: this test alone got a tight, near-exact match;
                # run alongside the other sim tests in the same session, with
                # more processes competing, the gap widened to ~18% on the
                # shoulder axis) that residual lag is real, not a relay bug.
                # What actually matters -- the relay is live and proportional
                # -- is what this checks.
                #
                # Shoulder (i=1) is handled separately, not skipped: this
                # script launches bilateral_4ch.py with --c1-disable-joints
                # left at its shipped default ("shoulder", 2026-09-03 --
                # shoulder's identification data was found unreliable, its
                # noise/friction floor meets or exceeds its own torque
                # budget). This push DOES load shoulder meaningfully (its own
                # force_ext_hat is well above the 1e-3 floor), so asserting
                # the relayed value stays near zero regardless proves the
                # mask actively works, not just that nothing crashed.
                for i, axis in enumerate("wse"):  # waist/shoulder/elbow
                    if axis == "s":
                        assert abs(final_left[i]) < 1e-3, (
                            f"C1 relay should exclude shoulder (bilateral_4ch.py's "
                            f"--c1-disable-joints default), but master's "
                            f"joint_torque_ext_cmd shoulder={final_left[i]:.5f} "
                            f"despite slave force_ext_hat shoulder={final_right[i]:.5f} "
                            f"-- the mask is not zeroing it"
                        )
                        continue
                    if abs(final_right[i]) < 1e-3:
                        continue  # axis not meaningfully loaded, skip
                    ratio = final_left[i] / final_right[i]
                    assert 0.5 < ratio < 1.5, (
                        f"C1 relay mismatch on {axis}: slave force_ext_hat="
                        f"{final_right}, master joint_torque_ext_cmd={final_left}, "
                        f"ratio={ratio:.2f} (expected roughly 1.0)"
                    )
    finally:
        master.stop()
        slave.stop()


@pytest.mark.skipif(not BILATERAL_SCRIPT.exists(),
                    reason=f"bilateral_4ch.py not found at {BILATERAL_SCRIPT}")
def test_c1_no_false_trip_on_gentle_ramp():
    """Regression for bug #1 above. A push that ramps up gently must not
    trip the oscillation-abort just because it is a NEW signal rising from
    zero -- only genuine growth toward a dangerous magnitude should."""
    master, slave = _launch_pair()
    try:
        status_topic = f"/{unique_ns('c1b_status')}"
        with ScriptProcess(
            [str(BILATERAL_SCRIPT),
            "--architecture", "custom", "--w1", "1.0", "--w2", "0.0",
            "--w3", "0.0", "--w4", "0.0",
            "--left-force-topic", master.topic("force_ext_hat"),
            "--right-force-topic", slave.topic("force_ext_hat"),
            "--left-joint-angles-topic", master.topic("joint_angles"),
            "--right-joint-angles-topic", slave.topic("joint_angles"),
            "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
            "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
            "--status-topic", status_topic,
            "--rate-hz", "200",
            # Default trip_pp_nm (0.4 N*m) -- deliberately NOT lowered here,
            # this test wants the real, shipped threshold.
            ],
            name=f"bilateral_{master.namespace}",
        ) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            with LiveNode("c1b_verify") as live:
                live.spin_for(1.0)
                # Ramp 0 -> 0.5 N over 3s, then hold 5s -- same shape that
                # false-tripped before the fix, at both this and a much
                # more abrupt step.
                ramp_start = time.time()
                ramp_dur = 3.0
                hold_end = ramp_start + ramp_dur + 5.0
                while time.time() < hold_end:
                    t = time.time() - ramp_start
                    fy = min(0.5, 0.5 * (t / ramp_dur))
                    live.push_force(slave.topic("force_cmd"), slave.topic("enable"), 0.0, -fy, 0.0)
                    live.spin_once(timeout_sec=0.01)

            assert "OSCILLATION ABORT" not in bilateral.log_text(), (
                f"false-tripped on an ordinary gentle ramp -- the "
                f"growing-magnitude-floor fix regressed; see {bilateral._log_path}\n"
                f"{bilateral.log_text()}"
            )
    finally:
        master.stop()
        slave.stop()


@pytest.mark.skipif(not BILATERAL_SCRIPT.exists(),
                    reason=f"bilateral_4ch.py not found at {BILATERAL_SCRIPT}")
def test_c1_manual_rearm_clears_a_real_trip():
    """Regression for bug #2 above. Deliberately induces a REAL trip (a very
    low --trip-pp-nm guarantees this on any nonzero push, sidestepping the
    "growing" heuristic entirely so this test is not coupled to bug #1's
    fix), then confirms manual_rearm clears it without crashing the node."""
    master, slave = _launch_pair()
    try:
        status_topic = f"/{unique_ns('c1c_status')}"
        with ScriptProcess(
            [str(BILATERAL_SCRIPT),
            "--architecture", "custom", "--w1", "1.0", "--w2", "0.0",
            "--w3", "0.0", "--w4", "0.0",
            "--left-force-topic", master.topic("force_ext_hat"),
            "--right-force-topic", slave.topic("force_ext_hat"),
            "--left-joint-angles-topic", master.topic("joint_angles"),
            "--right-joint-angles-topic", slave.topic("joint_angles"),
            "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
            "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
            "--status-topic", status_topic,
            "--rate-hz", "200",
            "--trip-pp-nm", "0.001",  # guarantees a real trip almost immediately
            ],
            name=f"bilateral_{master.namespace}",
        ) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            with LiveNode("c1c_verify") as live:
                live.spin_for(1.0)
                end_time = time.time() + 3.0
                while time.time() < end_time:
                    live.push_force(slave.topic("force_cmd"), slave.topic("enable"), 0.0, -0.3, 0.0)
                    live.spin_once(timeout_sec=0.01)

            bilateral.wait_for_marker("OSCILLATION ABORT", timeout=5.0)
            assert bilateral.proc.poll() is None, (
                "bilateral_4ch died on its own before manual_rearm was even tried"
            )

            result = subprocess.run(
                ["ros2", "param", "set", "/bilateral_4ch", "manual_rearm", "true"],
                capture_output=True, text=True, timeout=10.0,
            )
            assert result.returncode == 0, (
                f"ros2 param set failed: stdout={result.stdout!r} stderr={result.stderr!r}"
            )
            # Give the crashed-callback bug a moment to show up in the log if
            # it regresses -- rclpy's executor logs the traceback but does
            # not necessarily kill the process instantly.
            time.sleep(1.0)
            log = bilateral.log_text()
            assert "AttributeError" not in log, (
                f"manual_rearm crashed the callback again; see {bilateral._log_path}\n{log}"
            )
            assert bilateral.proc.poll() is None, (
                f"bilateral_4ch died after manual_rearm; see {bilateral._log_path}"
            )
            assert "oscillation trip cleared" in log, (
                f"manual_rearm did not report clearing the trip; see {bilateral._log_path}\n{log}"
            )
    finally:
        master.stop()
        slave.stop()


@pytest.mark.skipif(not BILATERAL_SCRIPT.exists(),
                    reason=f"bilateral_4ch.py not found at {BILATERAL_SCRIPT}")
def test_c1_oscillation_trip_repeated_push_baseline():
    """Not a currently-broken assertion -- a sim baseline for the
    real-hardware finding from 2026-09-03's first live C1 test: at
    --w1 0.5 (the value used live that day) with the default --trip-pp-nm
    (0.4 N*m, not overridden here), deliberate push-and-release motions
    retripped the oscillation-abort almost immediately after each
    manual_rearm, a real usability friction point for testing with big
    motions specifically. This establishes what that looks like in sim, so
    future weight/threshold tuning has something to check against instead
    of costing real hardware time.

    The push/release amplitude and cycle count below are themselves a
    judgment call with no single right answer from evidence gathered so
    far -- 4 cycles, ramped to a firm 0.6N push over 0.4s, held 0.3s,
    released over 0.4s. Treat this run's trip count as the recorded
    baseline, not a target to hit; the only hard requirements are that the
    node survives (no crash, matching bug #2's regression coverage above)
    and that manual_rearm actually clears every trip it produces.
    """
    master, slave = _launch_pair()
    try:
        status_topic = f"/{unique_ns('c1d_status')}"
        with ScriptProcess(
            [str(BILATERAL_SCRIPT),
            "--architecture", "custom", "--w1", "0.5", "--w2", "0.0",
            "--w3", "0.0", "--w4", "0.0",
            "--left-force-topic", master.topic("force_ext_hat"),
            "--right-force-topic", slave.topic("force_ext_hat"),
            "--left-joint-angles-topic", master.topic("joint_angles"),
            "--right-joint-angles-topic", slave.topic("joint_angles"),
            "--left-joint-torque-ext-topic", master.topic("joint_torque_ext_cmd"),
            "--right-joint-torque-ext-topic", slave.topic("joint_torque_ext_cmd"),
            "--status-topic", status_topic,
            "--rate-hz", "200",
            ],
            name=f"bilateral_{master.namespace}",
        ) as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            n_cycles = 4
            trip_count = 0
            with LiveNode("c1d_verify") as live:
                live.spin_for(1.0)
                for cycle in range(n_cycles):
                    ramp_start = time.time()
                    ramp_dur, hold_dur, release_dur = 0.4, 0.3, 0.4
                    cycle_end = ramp_start + ramp_dur + hold_dur + release_dur
                    while time.time() < cycle_end:
                        t = time.time() - ramp_start
                        if t < ramp_dur:
                            fy = 0.6 * (t / ramp_dur)
                        elif t < ramp_dur + hold_dur:
                            fy = 0.6
                        else:
                            rt = t - ramp_dur - hold_dur
                            fy = 0.6 * max(0.0, 1.0 - rt / release_dur)
                        live.push_force(
                            slave.topic("force_cmd"), slave.topic("enable"), 0.0, -fy, 0.0)
                        live.spin_once(timeout_sec=0.01)

                    if "OSCILLATION ABORT" in bilateral.log_text():
                        trip_count += 1
                        assert bilateral.proc.poll() is None, (
                            f"bilateral_4ch died after cycle {cycle}'s trip; "
                            f"see {bilateral._log_path}"
                        )
                        result = subprocess.run(
                            ["ros2", "param", "set", "/bilateral_4ch",
                             "manual_rearm", "true"],
                            capture_output=True, text=True, timeout=10.0,
                        )
                        assert result.returncode == 0, (
                            f"ros2 param set failed after cycle {cycle}: "
                            f"stdout={result.stdout!r} stderr={result.stderr!r}"
                        )
                        time.sleep(0.3)
                        log = bilateral.log_text()
                        assert "AttributeError" not in log, (
                            f"manual_rearm crashed the callback after cycle "
                            f"{cycle}; see {bilateral._log_path}\n{log}"
                        )
                        assert bilateral.proc.poll() is None, (
                            f"bilateral_4ch died after manual_rearm following "
                            f"cycle {cycle}; see {bilateral._log_path}"
                        )

            print(
                f"\n[baseline] w1=0.5, trip_pp_nm=default(0.4): "
                f"{trip_count}/{n_cycles} push-release cycles tripped "
                f"the oscillation-abort, all cleared cleanly by manual_rearm."
            )
    finally:
        master.stop()
        slave.stop()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
