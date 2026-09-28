"""INVESTIGATION ONLY (2026-09-09), no bridge/state-machine code changes.

Purpose: measure the telemetry velocity_mps noise floor for BOTH arms in a
condition that is genuinely physically quiet, as the basis for choosing a
velocity threshold for a state-based (not fixed-duration) GRAVITY_SETTLE
gate -- per the user's explicit "do not choose thresholds arbitrarily; first
derive provisional thresholds from the measured quiet-state telemetry noise
floor" requirement.

MASTER: held by the sim's stiff Hand (TOUCH_SIM_HAND=stiff), which pins it
mechanically -- any residual velocity_mps is pure sensor/estimator noise,
not real motion.

SLAVE: the slave has no Hand (is_right forces k_hand=0 in hd_sim_api.cpp),
so it cannot be pinned the same way -- instead this measures velocity_mps
AFTER the fixture's own SETTLE_S=3.0s pre-bridge settle window, which
test_sim_initial_condition_repeatability.py already established converges
to <0.01mm position deviation across independent launches (i.e. genuinely
at rest under the sim's own native gravity, no bridge control active yet,
no bridge gravity-comp aux applied). velocity_mps there is this arm's own
noise floor in the same telemetry pipeline the GRAVITY_SETTLE gate will read.

Not a pytest test -- run directly:
    python3 investigate_quiet_state_noise_floor.py
"""
from __future__ import annotations

import math
import statistics
import sys
from pathlib import Path

from touch_haptics.msg import CartesianControllerTelemetry

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, SimNode, unique_ns  # noqa: E402
from force_budget_sim_fixture import (  # noqa: E402
    MASTER_PARAMS, SLAVE_PARAMS, SETTLE_S, master_sim_env, slave_sim_env,
)

REQUESTED_GAP_M = 0.117
SAMPLE_WINDOW_S = 2.0


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _stats(label, samples):
    s = sorted(samples)
    n = len(s)
    mean = statistics.fmean(s)
    rms = math.sqrt(sum(x * x for x in s) / n)
    p95 = s[min(n - 1, int(math.ceil(0.95 * n)) - 1)]
    mx = s[-1]
    print(f"  {label}: n={n} mean={mean*1000:8.4f} rms={rms*1000:8.4f} "
          f"p95={p95*1000:8.4f} max={mx*1000:8.4f}  (mm/s)")
    return {"mean": mean, "rms": rms, "p95": p95, "max": mx, "n": n}


def main():
    mn, sn = unique_ns("qnf_m"), unique_ns("qnf_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS,
                sim_env=master_sim_env(REQUESTED_GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS,
                sim_env=slave_sim_env()) as slave:
        with LiveNode(f"qnf_v_{mn}") as live:
            live.watch("m", master.topic("cartesian_controller_telemetry"),
                      CartesianControllerTelemetry, extract=lambda x: x)
            live.watch("s", slave.topic("cartesian_controller_telemetry"),
                      CartesianControllerTelemetry, extract=lambda x: x)

            # Let the fixture's own SETTLE_S apply first (matches the
            # pre-bridge condition every other test in this suite uses).
            live.spin_for(SETTLE_S)

            m_samples, s_samples = [], []
            t_end = SAMPLE_WINDOW_S
            import time
            deadline = time.time() + t_end
            while time.time() < deadline:
                live.spin_once(timeout_sec=0.01)
                m = live.latest.get("m")
                s = live.latest.get("s")
                if m is not None:
                    m_samples.append(_norm(list(m.velocity_mps)))
                if s is not None:
                    s_samples.append(_norm(list(s.velocity_mps)))

    print(f"\n=== QUIET-STATE VELOCITY NOISE FLOOR (post {SETTLE_S:.1f}s settle, "
          f"{SAMPLE_WINDOW_S:.1f}s sample window) ===")
    print("MASTER (mechanically pinned by stiff Hand):")
    m_stats = _stats("velocity norm", m_samples)
    print("SLAVE (sim-native gravity settle only, no bridge control active):")
    s_stats = _stats("velocity norm", s_samples)
    return m_stats, s_stats


if __name__ == "__main__":
    main()
