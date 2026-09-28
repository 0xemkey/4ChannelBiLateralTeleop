"""Part 2 of the validation-infrastructure task (2026-09-09): prove the
deterministic initial-condition fixture actually IS deterministic.

Launches the sim fixture from scratch 5 independent times at a requested
117mm master/slave Y separation, measures the achieved gap immediately
before a bridge would be started, and reports mean/std/max-deviation.

Target (the task's own bar): deterministic to within simulator numerical
precision, or at minimum < 0.5mm deviation across runs.
"""
from __future__ import annotations

import statistics
import sys
from pathlib import Path

import pytest
from touch_haptics.msg import CartesianControllerTelemetry

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, SimNode, unique_ns  # noqa: E402
from force_budget_sim_fixture import (  # noqa: E402
    MASTER_PARAMS, SLAVE_PARAMS, SETTLE_S, master_sim_env, slave_sim_env, measured_gap,
)

REQUESTED_GAP_M = 0.117
N_RUNS = 5
MAX_DEVIATION_M = 0.0005  # 0.5mm, the task's stated minimum bar


def _launch_and_measure(gap_m: float):
    """One completely independent launch: fresh sim processes, settle, measure."""
    mn, sn = unique_ns("rep_m"), unique_ns("rep_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS, sim_env=master_sim_env(gap_m)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS, sim_env=slave_sim_env()) as slave:
        with LiveNode(f"rep_v_{mn}") as live:
            live.watch("m", master.topic("cartesian_controller_telemetry"),
                      CartesianControllerTelemetry, extract=lambda x: x)
            live.watch("s", slave.topic("cartesian_controller_telemetry"),
                      CartesianControllerTelemetry, extract=lambda x: x)
            live.spin_for(SETTLE_S)
            assert "m" in live.latest and "s" in live.latest, "no telemetry from one of the sim arms"
            mp = list(live.latest["m"].position_m)
            sp = list(live.latest["s"].position_m)
            return mp, sp


def test_initial_condition_is_repeatable_across_independent_launches():
    gaps_y = []
    gaps_norm = []
    for run in range(N_RUNS):
        mp, sp = _launch_and_measure(REQUESTED_GAP_M)
        dy, norm = measured_gap(mp, sp)
        gaps_y.append(dy)
        gaps_norm.append(norm)
        print(f"[run {run+1}/{N_RUNS}] master=({mp[0]:+.5f},{mp[1]:+.5f},{mp[2]:+.5f}) "
              f"slave=({sp[0]:+.5f},{sp[1]:+.5f},{sp[2]:+.5f}) "
              f"gap_y={dy*1000:8.3f}mm |gap|={norm*1000:8.3f}mm")

    mean_y = statistics.fmean(gaps_y)
    std_y = statistics.pstdev(gaps_y) if len(gaps_y) > 1 else 0.0
    max_dev = max(abs(g - mean_y) for g in gaps_y)
    print(f"\nrequested gap      = {REQUESTED_GAP_M*1000:.3f} mm")
    print(f"measured gap_y mean= {mean_y*1000:.3f} mm")
    print(f"measured gap_y std = {std_y*1000:.4f} mm")
    print(f"max abs deviation  = {max_dev*1000:.4f} mm")
    print(f"per-run gap_y (mm) : {[round(g*1000, 3) for g in gaps_y]}")

    assert max_dev < MAX_DEVIATION_M, (
        f"initial condition must be repeatable to better than "
        f"{MAX_DEVIATION_M*1000:.2f}mm across independent launches, got "
        f"max deviation {max_dev*1000:.4f}mm")
    # The fixture aims at the requested gap but the honest contract is
    # REPEATABILITY (asserted above) plus a MEASURED gap that is actually in
    # the intended neighbourhood -- not a claim of hitting it exactly.
    assert abs(mean_y - REQUESTED_GAP_M) < 0.010, (
        f"achieved mean gap {mean_y*1000:.2f}mm should land within 10mm of the "
        f"requested {REQUESTED_GAP_M*1000:.1f}mm")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
