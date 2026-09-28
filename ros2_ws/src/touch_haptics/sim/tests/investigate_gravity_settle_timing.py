"""INVESTIGATION ONLY (2026-09-09) -- no code changes to the bridge or
startup_alignment.py. Purpose: quantify what the slave's telemetry
(position_m / velocity_mps) is actually doing across GRAVITY_RAMP_UP ->
GRAVITY_SETTLE -> COUPLING_RAMP_UP, to check the hypothesis that
gravity_settle_duration_s=0.2s is too short for the slave to mechanically
settle under the freshly-ramped 0.70N gravity load before COUPLING_RAMP_UP
snapshots its origin (SNAPSHOT_ORIGIN mode).

This is a read-only diagnostic run of the REAL bridge/controllers -- same
deterministic 117mm fixture as test_force_budget_117mm_live.py -- with a
per-tick log of bridge_state, slave position_m[1] (Y), slave velocity_mps[1]
(Y), and coupling_ramp_alpha, printed with elapsed-time-since-GRAVITY_SETTLE
markers so the settle window can be inspected directly.

Not a pytest test (no pass/fail assertions) -- run directly:
    python3 investigate_gravity_settle_timing.py
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import time
from pathlib import Path

from touch_haptics.msg import CartesianControllerTelemetry, BridgeDiagnostics

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402
from force_budget_sim_fixture import (  # noqa: E402
    MASTER_PARAMS, SLAVE_PARAMS, SETTLE_S, master_sim_env, slave_sim_env, measured_gap,
)

BRIDGE_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bridge_master_slave.py"
)

GRAVITY_N = 0.70
REQUESTED_GAP_M = 0.117


def bridge_args(master, slave, calib_path):
    return [str(BRIDGE_SCRIPT),
            "--left-pose-topic", master.topic("pose"),
            "--right-pose-topic", slave.topic("pose"),
            "--right-target-topic", slave.topic("target_pose"),
            "--right-enable-topic", slave.topic("enable"),
            "--left-target-topic", master.topic("target_pose"),
            "--left-enable-topic", master.topic("enable"),
            "--left-telemetry-topic", master.topic("cartesian_controller_telemetry"),
            "--right-telemetry-topic", slave.topic("cartesian_controller_telemetry"),
            "--slave-force-topic", slave.topic("force_cmd"),
            "--enable-c2", "true",
            "--left-max-offset-m", "0.08",
            "--calibration-file", str(calib_path),
            "--axis-map", "x,y,z",
            "--absolute-axes", "x,y,z", "--absolute-trim-m", "0.0",
            "--slave-gravity-comp-n", str(GRAVITY_N), "--slave-gravity-axis", "1",
            "--master-stiffness-n-per-m", "15.0", "--master-damping-n-s-per-m", "0.10",
            "--master-max-force-n", "2.0", "--master-max-position-error-m", "0.03",
            "--stiffness-n-per-m", "35.0", "--slave-damping-n-s-per-m", "0.20",
            "--slave-max-force-n", "1.5", "--slave-max-position-error-m", "0.04",
            "--force-budget-reserve-n", "0.10",
            "--rate-hz", "100", "--wait-timeout-s", "15",
            ]


def main():
    mn, sn = unique_ns("gset_m"), unique_ns("gset_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS,
                sim_env=master_sim_env(REQUESTED_GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS,
                sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"gset_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"),
                     CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"),
                     CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)
            mp0 = list(pre.latest["m"].position_m)
            sp0 = list(pre.latest["s"].position_m)
        gap_y0, gap_norm0 = measured_gap(mp0, sp0)
        print(f"pre-bridge fixture settle: master={mp0} slave={sp0} gap_y={gap_y0*1000:.3f}mm")

        with ScriptProcess(bridge_args(master, slave, calib_path), name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"gset_v_{mn}") as live:
                live.watch("m", master.topic("cartesian_controller_telemetry"),
                          CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("s", slave.topic("cartesian_controller_telemetry"),
                          CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("d", "/bridge_diagnostics", BridgeDiagnostics, extract=lambda x: x)
                live.spin_for(1.0)

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

                last_state = None
                state_t0 = {}
                log = []
                t_start = time.time()
                deadline = t_start + 15.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    s = live.latest.get("s")
                    if d is None or s is None:
                        continue
                    st = d.bridge_state
                    now = time.time()
                    if st != last_state:
                        state_t0[st] = now
                        print(f"\n>>> STATE TRANSITION -> {st} at t={now - t_start:8.3f}s "
                              f"slave_y={s.position_m[1]:+.5f} slave_vy={s.velocity_mps[1]:+.5f}")
                        last_state = st
                    since = now - state_t0[st]
                    log.append((now - t_start, st, since, s.position_m[1], s.velocity_mps[1],
                               d.coupling_ramp_alpha, d.slave_accepted_position_error_margin_m,
                               d.slave_accepted_force_budget_margin_n))
                    if st in ("GRAVITY_SETTLE", "COUPLING_RAMP_UP"):
                        print(f"  t={now - t_start:8.3f}s (+{since:6.3f}s in {st}) "
                              f"slave_y={s.position_m[1]:+.6f} slave_vy={s.velocity_mps[1]:+.6f} "
                              f"alpha={d.coupling_ramp_alpha:.4f} pos_margin={d.slave_accepted_position_error_margin_m*1000:+8.3f}mm "
                              f"force_margin={d.slave_accepted_force_budget_margin_n:+7.4f}N")
                    if st == "FAULT":
                        print("\n>>> FAULT reached, stopping.")
                        break

        print("\n\n=== SUMMARY: slave Y position/velocity across GRAVITY_SETTLE ===")
        gsettle = [row for row in log if row[1] == "GRAVITY_SETTLE"]
        if gsettle:
            print(f"  GRAVITY_SETTLE duration observed: {gsettle[-1][2]:.3f}s "
                  f"(entries: {len(gsettle)})")
            print(f"  slave_y at GRAVITY_SETTLE start = {gsettle[0][3]:+.6f}  "
                  f"vy={gsettle[0][4]:+.6f}")
            print(f"  slave_y at GRAVITY_SETTLE end   = {gsettle[-1][3]:+.6f}  "
                  f"vy={gsettle[-1][4]:+.6f}")
        cru = [row for row in log if row[1] == "COUPLING_RAMP_UP"]
        if cru:
            print(f"\n  COUPLING_RAMP_UP entries: {len(cru)}, duration observed: {cru[-1][2]:.3f}s")
            print(f"  slave_y at COUPLING_RAMP_UP start = {cru[0][3]:+.6f}  vy={cru[0][4]:+.6f}")
            print(f"  slave_y at COUPLING_RAMP_UP end   = {cru[-1][3]:+.6f}  vy={cru[-1][4]:+.6f}")
            print(f"  slave_y drift DURING COUPLING_RAMP_UP (alpha={cru[0][5]:.3f}->{cru[-1][5]:.3f}) "
                  f"= {(cru[-1][3]-cru[0][3])*1000:+.3f}mm")
            # Is the drift correlated with velocity staying nonzero (still
            # settling) vs alpha actually advancing (coupling pulling it)?
            still_moving_ticks = sum(1 for row in cru if abs(row[4]) > 0.001)
            print(f"  ticks in COUPLING_RAMP_UP with |vy|>1mm/s: {still_moving_ticks}/{len(cru)}")


if __name__ == "__main__":
    main()
