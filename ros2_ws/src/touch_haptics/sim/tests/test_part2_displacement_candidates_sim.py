"""2026-09-10 item 5 of the real-hardware Y-axis validation revision:
validate candidate Part 2 (master-only, no bridge) Y-displacement
magnitudes using the EXACT simulated control path Part 2 will actually
use on real hardware -- a bare touch_slave(_sim) instance driven directly
by target_pose commands, no bridge/planner involved (the sim binary runs
the IDENTICAL C++ controller code as the real touch_slave, only the HD API
backend differs).

Reuses part2_master_only_y_motion.py's own ramp-hold-ramp profile
(_smoothstep, RAMP_DURATION_S, HOLD_DURATION_S) so this is genuinely "the
same control path", not a re-derived approximation.

Sweeps 2mm / 5mm / 10mm with the FROZEN config (frozen_config.py) -- K/D/
Fmax/max_position_error_m unchanged, nothing tuned. Reports peak force,
tracking error, velocity, settling time, clamp status, and whether the
motion is distinguishable from the measured quiet-state noise floor
(investigate_quiet_state_noise_floor.py: velocity noise <= 1.85mm/s peak,
this session's sim measurement). Recommends a value from evidence -- does
NOT run on hardware.
"""
from __future__ import annotations

import math
import statistics
import sys
import time
from pathlib import Path

import pytest
from touch_haptics.msg import CartesianControllerTelemetry

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, SimNode, unique_ns  # noqa: E402

HW_DIR = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "hardware_experiments"
    / "2026-09-09_y_axis_validation_PENDING_SUPERVISION"
)
sys.path.insert(0, str(HW_DIR))
from part2_master_only_y_motion import _smoothstep, RAMP_DURATION_S, HOLD_DURATION_S  # noqa: E402
from frozen_config import FROZEN  # noqa: E402

CANDIDATES_MM = (2.0, 5.0, 10.0)
SETTLE_S = 3.0
PUBLISH_HZ = 100.0
SETTLE_HOLD_WINDOW_S = 0.2  # continuous-hold window required to call it "settled"
MIN_ACHIEVED_COMMANDED_RATIO = 0.80  # per the task's explicit 80% bar

# 2026-09-10 correction: the master ("Left") device defaults to hd_sim's
# "kFree" environment with HandParams preset "none" (k_hand=0) -- i.e. a
# genuinely UNHELD, free-floating mass with nothing providing the give a
# real operator's hand would. Every OTHER master-motion sim test this
# session used TOUCH_SIM_HAND=stiff/loose specifically to represent a hand
# holding the stylus; this test must too, or it validates a scenario (an
# untouched device swinging under its own inertia) that has nothing to do
# with Part 2's actual real-hardware scenario (an operator holding the
# stylus, required to have require_deadman:=true held the whole time).
# "loose" (k_hand=40 N/m, b_hand=1.5 Ns/m -- environment.hpp's own
# comment: "loose ~ fingertips") is the closest available preset to an
# attentive but light grip; it is a MODELING APPROXIMATION (a passive
# spring-damper, not an actively-compliant human), documented as such --
# see the printed caveat in the report below.
MASTER_HAND_PRESET = "loose"

# From this session's ALREADY-MEASURED real quiet-state POSITION drift
# (not a derived proxy): the 117mm live hardware-validation-infrastructure
# task measured master_settle_pose_drift_m = 0.070mm during the accepted
# GRAVITY_SETTLE quiet window (150ms hold, both arms confirmed quiescent).
# This is a genuine, already-validated position-noise reference, not
# invented for this test.
QUIET_STATE_POSITION_DRIFT_M = 0.00007


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _run_one_displacement(dy_mm: float) -> dict:
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool

    dy_m = dy_mm / 1000.0
    mn = unique_ns(f"p2v_{int(dy_mm)}mm")
    params = {
        "stiffness_n_per_m": str(FROZEN.master_stiffness_n_per_m),
        "damping_n_s_per_m": str(FROZEN.master_damping_n_s_per_m),
        "max_force_n": str(FROZEN.master_max_force_n),
        "max_position_error_m": str(FROZEN.master_max_position_error_m),
        "require_deadman": "false",
    }
    sim_env = {"TOUCH_SIM_HAND": MASTER_HAND_PRESET}
    with SimNode(mn, device="Left", params=params, sim_env=sim_env) as master:
        with LiveNode(f"p2v_v_{mn}") as live:
            live.watch("m", master.topic("cartesian_controller_telemetry"),
                      CartesianControllerTelemetry, extract=lambda x: x)
            live.spin_for(SETTLE_S)
            assert "m" in live.latest, "no telemetry from the sim master"
            m0 = live.latest["m"]
            x0, y0, z0 = m0.position_m[0], m0.position_m[1], m0.position_m[2]

            target_pub = live._node.create_publisher(PoseStamped, master.topic("target_pose"), 10)
            enable_pub = live._node.create_publisher(Bool, master.topic("enable"), 10)

            def _pub_target(xyz):
                msg = PoseStamped()
                msg.header.stamp = live._node.get_clock().now().to_msg()
                msg.header.frame_id = "touch_left_base"
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
                msg.pose.orientation.w = 1.0
                target_pub.publish(msg)

            def _pub_enable(on):
                msg = Bool(); msg.data = on
                enable_pub.publish(msg)

            _pub_enable(True)

            rows = []
            dt = 1.0 / PUBLISH_HZ
            phases = [
                ("RAMP_UP", RAMP_DURATION_S, lambda u: y0 + dy_m * _smoothstep(u)),
                ("HOLD", HOLD_DURATION_S, lambda u: y0 + dy_m),
                ("RAMP_DOWN", RAMP_DURATION_S, lambda u: y0 + dy_m * (1.0 - _smoothstep(u))),
            ]
            for phase_name, duration, y_fn in phases:
                t_start = time.time()
                while True:
                    now = time.time()
                    u = (now - t_start) / duration
                    live.spin_once(timeout_sec=0.0)
                    y_cmd = y_fn(min(1.0, u))
                    _pub_target((x0, y_cmd, z0))
                    m = live.latest.get("m")
                    if m is not None:
                        rows.append({
                            "t": now, "phase": phase_name,
                            "target_y": y_cmd,
                            "y": m.position_m[1], "vy": m.velocity_mps[1],
                            "err_y": m.position_error_raw_m[1],
                            "spring_y": m.spring_force_n[1], "damping_y": m.damping_force_n[1],
                            "applied": _norm(list(m.applied_force_n)),
                            "pos_clamped": bool(m.position_error_clamped),
                            "primary_clamped": bool(m.primary_force_clamped),
                            "final_clamped": bool(m.final_force_clamped),
                        })
                    if u >= 1.0:
                        break
                    time.sleep(dt)
            _pub_enable(False)

    # 2026-09-10 fix: the old settling-time check scanned err_y from t=0
    # (RAMP_UP start), where err_y is TRIVIALLY ~0 (target starts equal to
    # y0), so it always "settled" instantly and meaninglessly. Corrected
    # per the task's explicit spec: settling is measured ONLY AFTER
    # RAMP_UP completes (target has reached its final commanded value),
    # and must stay within tolerance for a CONTINUOUS hold window, not a
    # single sample.
    y0 = rows[0]["y"]
    peak_applied = max(r["applied"] for r in rows)
    any_clamp = any(r["pos_clamped"] or r["primary_clamped"] or r["final_clamped"] for r in rows)

    ramp_up_rows = [r for r in rows if r["phase"] == "RAMP_UP"]
    hold_rows = [r for r in rows if r["phase"] == "HOLD"]
    ramp_down_rows = [r for r in rows if r["phase"] == "RAMP_DOWN"]
    ramp_up_hold_rows = ramp_up_rows + hold_rows

    peak_err_m = max(abs(r["err_y"]) for r in ramp_up_hold_rows) if ramp_up_hold_rows else float("nan")
    rms_err_m = (math.sqrt(statistics.fmean([r["err_y"] ** 2 for r in ramp_up_hold_rows]))
                if ramp_up_hold_rows else float("nan"))

    # Steady-state: final 30% of the HOLD phase's samples (by count).
    if hold_rows:
        n_tail = max(1, int(0.3 * len(hold_rows)))
        steady_rows = hold_rows[-n_tail:]
        steady_state_y = statistics.fmean([r["y"] for r in steady_rows])
        steady_state_err_m = statistics.fmean([abs(r["err_y"]) for r in steady_rows])
    else:
        steady_state_y = rows[-1]["y"]
        steady_state_err_m = abs(rows[-1]["err_y"])

    achieved_displacement_m = steady_state_y - y0
    ratio = (achieved_displacement_m / dy_m) if dy_m != 0 else float("nan")

    # Rise time: RAMP_UP start until y FIRST reaches 90% of the actual
    # STEADY-STATE achieved displacement (not 90% of the commanded value --
    # with a persistent steady-state offset, using the commanded value
    # would make "rise time" undefined whenever the device never gets
    # there at all, which is a different, separately-reported fact).
    rise_time_s = None
    if ramp_up_rows and abs(achieved_displacement_m) > 1e-6:
        t_ramp_start = ramp_up_rows[0]["t"]
        target_90pct = y0 + 0.9 * achieved_displacement_m
        for r in ramp_up_rows:
            if (achieved_displacement_m > 0 and r["y"] >= target_90pct) or \
               (achieved_displacement_m < 0 and r["y"] <= target_90pct):
                rise_time_s = r["t"] - t_ramp_start
                break

    # Settling time -- AFTER RAMP_UP completes, first moment err_y enters
    # a tolerance band around the FINAL COMMANDED target (10% of dy_m or
    # 0.5mm, whichever is larger) and REMAINS there continuously for
    # SETTLE_HOLD_WINDOW_S. None if it never does (a real, reportable
    # outcome, not a bug, when a persistent steady-state offset exists).
    settle_thresh_m = max(0.1 * abs(dy_m), 0.0005)
    settling_time_s = None
    if hold_rows:
        t_ramp_up_end = hold_rows[0]["t"]
        for i, r in enumerate(hold_rows):
            if abs(r["err_y"]) >= settle_thresh_m:
                continue
            window_end_t = r["t"] + SETTLE_HOLD_WINDOW_S
            held = all(abs(r2["err_y"]) < settle_thresh_m for r2 in hold_rows[i:]
                      if r2["t"] <= window_end_t)
            # Also require the window actually had enough samples/duration
            # (not just "ran out of HOLD data") -- only accept if either the
            # window is fully covered by recorded HOLD samples, or HOLD
            # itself ends while still within tolerance (device stayed
            # settled through the end of the phase).
            covered = (hold_rows[-1]["t"] - r["t"]) >= SETTLE_HOLD_WINDOW_S or \
                all(abs(r2["err_y"]) < settle_thresh_m for r2 in hold_rows[i:])
            if held and covered:
                settling_time_s = r["t"] - t_ramp_up_end
                break

    return_to_origin_error_m = abs(ramp_down_rows[-1]["y"] - y0) if ramp_down_rows else abs(rows[-1]["y"] - y0)

    achieved_peak_y_excursion = max(abs(r["y"] - y0) for r in rows)
    noise_snr = achieved_peak_y_excursion / QUIET_STATE_POSITION_DRIFT_M

    return {
        "dy_mm": dy_mm, "peak_applied_n": peak_applied, "any_clamp": any_clamp,
        "achieved_displacement_mm": achieved_displacement_m * 1000,
        "achieved_commanded_ratio": ratio,
        "steady_state_err_mm": steady_state_err_m * 1000,
        "peak_err_mm": peak_err_m * 1000, "rms_err_mm": rms_err_m * 1000,
        "rise_time_s": rise_time_s, "settling_time_s": settling_time_s,
        "return_to_origin_error_mm": return_to_origin_error_m * 1000,
        "achieved_peak_y_excursion_mm": achieved_peak_y_excursion * 1000,
        "noise_snr_proxy": noise_snr,
    }


def test_part2_displacement_candidates():
    results = [_run_one_displacement(dy) for dy in CANDIDATES_MM]
    print("\n================ PART 2 DISPLACEMENT CANDIDATE VALIDATION (sim) ================")
    print(f"MASTER_HAND_PRESET={MASTER_HAND_PRESET!r} (passive spring-damper stand-in for an "
          f"operator's grip -- see module docstring)")
    for r in results:
        print(f"  DY={r['dy_mm']:5.1f}mm:")
        print(f"    achieved displacement      = {r['achieved_displacement_mm']:8.3f}mm "
              f"(ratio={r['achieved_commanded_ratio']*100:6.2f}% of commanded)")
        print(f"    steady-state error         = {r['steady_state_err_mm']:8.3f}mm (final 30% of HOLD)")
        print(f"    peak / RMS tracking error  = {r['peak_err_mm']:8.3f}mm / {r['rms_err_mm']:8.3f}mm "
              f"(RAMP_UP+HOLD)")
        print(f"    rise time (to 90% of ACHIEVED steady value) = {r['rise_time_s']}s")
        print(f"    settling time (post-RAMP_UP, vs COMMANDED target) = {r['settling_time_s']}s")
        print(f"    return-to-origin error (post-RAMP_DOWN) = {r['return_to_origin_error_mm']:8.3f}mm")
        print(f"    peak applied force         = {r['peak_applied_n']:.4f}N   any_clamp={r['any_clamp']}")
        print(f"    achieved-excursion vs noise floor (0.070mm) = {r['noise_snr_proxy']:.1f}x")
    print("==================================================================================\n")

    for r in results:
        assert not r["any_clamp"], f"DY={r['dy_mm']}mm engaged a clamp -- unexpected under frozen config"
        assert r["peak_applied_n"] < FROZEN.master_max_force_n, (
            f"DY={r['dy_mm']}mm peak applied {r['peak_applied_n']:.3f}N approached Fmax"
        )

    # Per the task's explicit bar: do NOT recommend a displacement unless
    # achieved/commanded >= 80% AND the achieved excursion is distinguishable
    # from the measured noise floor. "None" is an explicitly valid, expected
    # result -- not a failure of this test.
    recommended = None
    for r in sorted(results, key=lambda r: r["dy_mm"]):
        if (r["achieved_commanded_ratio"] >= MIN_ACHIEVED_COMMANDED_RATIO
                and r["noise_snr_proxy"] >= 10.0 and not r["any_clamp"]):
            recommended = r
            break
    print(f"RECOMMENDATION (from evidence, NOT a hardware GO, "
          f"requires >={MIN_ACHIEVED_COMMANDED_RATIO*100:.0f}% achieved/commanded ratio): "
          f"{recommended['dy_mm'] if recommended else 'NONE'}mm")
    if recommended is None:
        print("NONE of the candidates met the 80% achieved/commanded bar under this control "
              "path (self-actuated target_pose against a passive hand model) -- see the ratio "
              "column above. This is evidence that Part 2's self-actuated method, as currently "
              "designed, does not reliably move the master a controlled distance -- see item 3's "
              "Procedure A vs B split.")
    return results


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
