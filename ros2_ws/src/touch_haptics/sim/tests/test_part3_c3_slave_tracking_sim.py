"""2026-09-10 item 4 of the real-hardware Y-axis validation revision:
FULL Part 3 simulation -- C2 disabled, C3 enabled, force reflection/C1/C4
disabled, a SIMULATED OPERATOR moves the master through a smooth Y
trajectory, and the slave must follow via C3.

TIMING NOTE (modeling choice, documented rather than hidden): hd_sim's
built-in Hand motion (TOUCH_SIM_HAND_MOTION) fires on a clock local to the
master SimNode's OWN process start, independent of when the bridge later
reaches ACTIVE (which itself takes a variable few seconds through GRAVITY_
RAMP_UP/GRAVITY_SETTLE/COUPLING_RAMP_UP). A one-shot "step" trajectory
therefore cannot be reliably synced to land inside the ACTIVE window without
either a new sim-side delay parameter (a C++ change to test-only sim code)
or fragile process-launch-order timing. Using TOUCH_SIM_HAND_MOTION=sine
instead sidesteps this entirely: the master's hand-driven position is
CONTINUOUSLY oscillating for the whole test, so C3 tracking can be observed
at any point after ACTIVE is reached, including well after this specific
run's own gravity-settle/coupling-ramp timing. Peak excursion is still
exactly the requested +/-5mm; this is a smooth, continuous, non-step
motion, matching "no step commands" in spirit (this is simulated-operator
modeling, not a command sent to real hardware).

Confirms (Part 6 of the earlier revision) that /touch_left/target_pose has
NO bridge publisher when --enable-c2 false (this test's own bridge args),
so the master's live pose is driven ONLY by the sim's Hand model -- nothing
publishes a target to the master at all in this test.
"""
from __future__ import annotations

import math
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped
from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics

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

HAND_AMPLITUDE_M = 0.005   # +/-5mm, matches item 4's "+5mm" request (peak excursion)
HAND_F0_HZ = 0.1           # slow, smooth -- 10s period
OBSERVE_S = 15.0           # >1 full cycle at 0.1Hz
SETTLE_S = 3.0


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_part3_master_drives_slave_via_c3_only():
    mn, sn = unique_ns("p3_m"), unique_ns("p3_s")
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
    # The bridge's own VALIDATE_ALIGNMENT sanity bound (max_alignment_error_m
    # =0.15m) rejects a large master/slave anchor mismatch. The slave's OWN
    # gravity-sag physics reach a Y equilibrium around -0.2108 within ~1s
    # regardless of its init position (confirmed via this test's own first
    # failed run: right_anchor=[-0.1006, -0.2108, -0.0029] even with NO
    # init-pos override at all) -- so only the MASTER's init position needs
    # to be set, close to that same natural slave resting point, rather than
    # an axis-mapping bug (there is none -- axis_map stays the validated
    # default x,y,z).
    master_sim_env = {
        "TOUCH_SIM_INIT_POS_X": "-0.10", "TOUCH_SIM_INIT_POS_Y": "-0.21", "TOUCH_SIM_INIT_POS_Z": "0.0",
        "TOUCH_SIM_HAND": "stiff",  # firm grip -- represents an attentive operator
        "TOUCH_SIM_HAND_MOTION": "sine",
        "TOUCH_SIM_HAND_AMP": str(HAND_AMPLITUDE_M),
        "TOUCH_SIM_HAND_F0": str(HAND_F0_HZ),
    }

    with SimNode(mn, device="Left", params=master_params, sim_env=master_sim_env) as master, \
        SimNode(sn, device="Default Device", params=slave_params) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"p3_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)  # let the slave reach its gravity-sag equilibrium

        bargs = [str(BRIDGE_SCRIPT)] + bridge_cli_args(str(calib_path), enable_c2=False,
                                                        gravity_settle_hold_mode=False)
        # Override topics to this test's namespaced SimNodes (frozen_config's
        # bridge_cli_args defaults to the real hardware topic names).
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

            with LiveNode(f"p3_v_{mn}") as live:
                live.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

                # Item 5: prove /touch_left/target_pose has no publisher at
                # all -- confirm the topic exists (subscribed) but NEVER
                # receives a message for the whole test (checked at the end).
                got_left_target = {"seen": False}
                live._node.create_subscription(
                    PoseStamped, master.topic("target_pose"),
                    lambda m: got_left_target.__setitem__("seen", True), 10)

                out = ""
                for _ in range(10):
                    r = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                        capture_output=True, text=True, timeout=10)
                    out = r.stdout + r.stderr
                    if "successful" in r.stdout:
                        break
                    live.spin_for(0.5)
                assert "successful" in out, f"could not arm: {out!r}"

                states = []
                reached_active = False
                fault_seen = False
                deadline = time.time() + 30.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is None:
                        continue
                    if not states or states[-1] != d.bridge_state:
                        states.append(d.bridge_state)
                    if d.bridge_state == "ACTIVE":
                        reached_active = True
                        break
                    if d.bridge_state == "FAULT":
                        fault_seen = True
                        break
                assert not fault_seen, f"bridge FAULTed before ACTIVE, states={states}"
                assert reached_active, f"never reached ACTIVE, states={states}"
                print(f"[part3] ACTIVE reached, states={states}", flush=True)

                rows = []
                deadline = time.time() + OBSERVE_S
                any_fault_during_observe = False
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    m, s, d = live.latest.get("m"), live.latest.get("s"), live.latest.get("d")
                    if m is None or s is None or d is None:
                        continue
                    if d.bridge_state == "FAULT":
                        any_fault_during_observe = True
                        break
                    rows.append({
                        "t": time.time(),
                        "my": m.position_m[1], "sy": s.position_m[1],
                        "mvy": m.velocity_mps[1], "svy": s.velocity_mps[1],
                        "m_applied": _norm(list(m.applied_force_n)), "s_applied": _norm(list(s.applied_force_n)),
                        "m_spring_y": m.spring_force_n[1], "s_spring_y": s.spring_force_n[1],
                        "m_damping_y": m.damping_force_n[1], "s_damping_y": s.damping_force_n[1],
                        "m_aux_y": m.auxiliary_force_n[1], "s_aux_y": s.auxiliary_force_n[1],
                        "m_pos_clamped": bool(m.position_error_clamped), "s_pos_clamped": bool(s.position_error_clamped),
                        "m_force_clamped": bool(m.primary_force_clamped or m.final_force_clamped),
                        "s_force_clamped": bool(s.primary_force_clamped or s.final_force_clamped),
                        "coupling_alpha": d.coupling_ramp_alpha,
                    })
                assert not any_fault_during_observe, "bridge FAULTed during the observation window"

                # Return both devices toward their origins and disarm (item
                # 4's explicit "return both devices to their live-latched
                # origins and disarm" -- the staged disarm sequence already
                # returns coupling_ramp_alpha to 0 and hands each side back
                # to its own live pose; the master's physical position is
                # governed by the (still-running) sine hand, not by us).
                out = ""
                for _ in range(10):
                    r = subprocess.run(
                        ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "false"],
                        capture_output=True, text=True, timeout=10)
                    out = r.stdout + r.stderr
                    if "successful" in r.stdout:
                        break
                    live.spin_for(0.5)
                assert "successful" in out, f"could not disarm: {out!r}"
                deadline = time.time() + 15.0
                disarmed = False
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is not None and d.bridge_state == "READY":
                        disarmed = True
                        break
                print(f"[part3] disarm confirmed={disarmed}", flush=True)

    assert rows, "no telemetry samples collected during the observation window"
    my0 = rows[0]["my"]
    sy_series = [r["sy"] for r in rows]
    my_series = [r["my"] for r in rows]

    m_disp = [r["my"] - my0 for r in rows]
    s_disp = [r["sy"] - sy_series[0] for r in rows]

    # Cross-correlation-based lag estimate (coarse, sample-index based --
    # sufficient for a "does the slave lag the master, by roughly how much"
    # diagnostic, not a publication-grade estimate).
    dt_est = (rows[-1]["t"] - rows[0]["t"]) / max(1, len(rows) - 1)
    max_lag_samples = int(2.0 / dt_est)  # search up to 2s of lag
    best_lag, best_corr = 0, -1e18
    m_mean = statistics.fmean(m_disp)
    s_mean = statistics.fmean(s_disp)
    m_c = [v - m_mean for v in m_disp]
    s_c = [v - s_mean for v in s_disp]
    for lag in range(0, min(max_lag_samples, len(rows) // 2)):
        num = sum(m_c[i] * s_c[i + lag] for i in range(len(rows) - lag))
        if num > best_corr:
            best_corr, best_lag = num, lag
    lag_s = best_lag * dt_est

    err = [r["my"] - r["sy"] for r in rows]  # NOT the controller's own tracking error --
                                              # this is master-vs-slave POSITION difference,
                                              # the actual bilateral-tracking quantity of interest
    rmse = math.sqrt(statistics.fmean([e * e for e in err]))
    peak_abs_err = max(abs(e) for e in err)

    # Gain: ratio of slave peak-to-peak displacement to master peak-to-peak
    # displacement over the observation window (amplitude tracking, not a
    # point ratio -- the master is continuously oscillating).
    m_ptp = max(m_disp) - min(m_disp)
    s_ptp = max(s_disp) - min(s_disp)
    gain = s_ptp / m_ptp if m_ptp > 1e-9 else float("nan")

    peak_m_vy = max(abs(r["mvy"]) for r in rows)
    peak_s_vy = max(abs(r["svy"]) for r in rows)
    peak_m_applied = max(r["m_applied"] for r in rows)
    peak_s_applied = max(r["s_applied"] for r in rows)
    any_clamp = any(r["m_pos_clamped"] or r["s_pos_clamped"] or r["m_force_clamped"] or r["s_force_clamped"]
                    for r in rows)
    min_coupling_alpha = min(r["coupling_alpha"] for r in rows)

    print("\n================ PART 3: C3 SLAVE-TRACKS-MASTER SIMULATION ================")
    print(f"observation window = {OBSERVE_S:.1f}s, n={len(rows)} samples")
    print(f"master Y peak-to-peak displacement = {m_ptp*1000:.3f}mm (commanded amplitude = "
          f"{HAND_AMPLITUDE_M*2000:.1f}mm peak-to-peak)")
    print(f"slave  Y peak-to-peak displacement = {s_ptp*1000:.3f}mm")
    print(f"gain (slave ptp / master ptp)      = {gain:.4f}")
    print(f"master-vs-slave Y RMSE             = {rmse*1000:.3f}mm")
    print(f"master-vs-slave Y peak abs error    = {peak_abs_err*1000:.3f}mm")
    print(f"estimated lag (cross-correlation)  = {lag_s*1000:.1f}ms")
    print(f"peak master vy / slave vy          = {peak_m_vy*1000:.2f} / {peak_s_vy*1000:.2f} mm/s")
    print(f"peak master applied / slave applied = {peak_m_applied:.4f}N / {peak_s_applied:.4f}N")
    print(f"any clamp                          = {any_clamp}")
    print(f"min coupling_ramp_alpha observed    = {min_coupling_alpha:.4f} (should be 1.0 throughout ACTIVE)")
    print(f"master target_pose ever published (should be False, C2 off) = {got_left_target['seen']}")
    print("=============================================================================\n")

    assert not any_clamp, "unexpected clamp during Part 3 slave-tracking observation"
    assert min_coupling_alpha > 0.999, "coupling_ramp_alpha should be exactly 1.0 throughout ACTIVE"
    assert not got_left_target["seen"], (
        "master received a target_pose message -- C2 should be off, meaning the bridge "
        "must never publish to /touch_left/target_pose"
    )
    assert gain > 0.3, f"slave should visibly follow the master's oscillation, gain={gain:.4f} is too small"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
