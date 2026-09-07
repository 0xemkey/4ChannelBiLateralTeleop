#!/usr/bin/env python3
"""Capture RAW (pre-any-deadband) per-joint disturbance-estimate signal from
a rich, repeated multi-push scenario -- sim-only, no bilateral_4ch.py, no
peer node. One capture per source node (C1's source = slave, C4's source =
master) is reused OFFLINE by diag_deadband_variants.py to reconstruct and
compare every candidate deadband architecture from the SAME underlying
signal, instead of launching a separate live run per variant (which would
also confound the comparison with sim's unseeded random-settle pose
varying run to run).

WHY tau_ext_nm SPECIFICALLY: ForceEstimate carries BOTH `tau_ext_nm` (raw,
`ext = -tau_dis_hat_nm`, no deadband applied -- see
touch_force_estimate.hpp's computeForceEstimate(), `r.tau_ext_nm[i] = ext;`
is set before the deadband subtraction a few lines later) and
`tau_ext_deadbanded_nm` (stage 1 already applied). Capturing the RAW field
means every downstream variant, including ones that skip or relocate stage
1, can be reconstructed from ONE recording.

PUSH PATTERNS, matching the user's explicit ask for "multiple amplitudes,
directions, timings, and repetitions, including the closest reproducible
version of the 2026-09-03 hardware pattern" -- three patterns, mirroring
FOUR_CHANNEL_TODO.md's own 2026-09-03 real-hardware findings:
  A "escalating_fast"  -- single axis, amplitudes ramp 0.2->0.6N over 5
                          cycles, NO pause between cycles. Matches the
                          real session's initial verification method,
                          itself noted as a plausible false-positive
                          trigger for the "growing" oscillation check.
  B "escalating_slow"  -- same 5 amplitudes, but with a 2.5s pause between
                          cycles. Matches the real session's "one axis, 2s
                          hold, 3-4s pause" pattern that did NOT re-trip.
  C "multiaxis_fast"   -- cycles through +x/+y/-y/+z/-z with no pauses,
                          matching the exact sequence that DID trip the
                          real session's latch.

SAFETY: sim-only, same touch_slave_sim binary every other sim/tests/*.py
uses; no OpenHaptics device opened.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Dict, List

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, SimNode, unique_ns  # noqa: E402

from touch_haptics.msg import ForceEstimate  # noqa: E402
from geometry_msgs.msg import Vector3Stamped  # noqa: E402

OUT_DIR = Path("/tmp/deadband_capture")
OUT_DIR.mkdir(exist_ok=True)


def _push_profile_escalating(pause_s: float, axis: int = 1):
    """5 cycles, amplitude 0.2->0.6N in 0.1N steps, ramp 0.3s/hold 1.2s/
    release 0.3s, `pause_s` of zero-force between cycles."""
    events = []
    t = 0.0
    for amp in (0.2, 0.3, 0.4, 0.5, 0.6):
        events.append((t, t + 0.3, "ramp", axis, amp, 0.3))
        t += 0.3
        events.append((t, t + 1.2, "hold", axis, amp, 0.0))
        t += 1.2
        events.append((t, t + 0.3, "release", axis, amp, 0.3))
        t += 0.3
        if pause_s > 0:
            events.append((t, t + pause_s, "pause", axis, 0.0, 0.0))
            t += pause_s
    return events, t


def _push_profile_multiaxis_fast():
    """+x/+y/-y/+z/-z, 0.5N each, ramp 0.2s/hold 0.5s/release 0.2s, NO
    pause -- matches the exact sequence that tripped the real 2026-09-03
    session."""
    axes_signs = [(0, 1), (1, 1), (1, -1), (2, 1), (2, -1)]
    events = []
    t = 0.0
    for axis, sign in axes_signs:
        amp = 0.5 * sign
        events.append((t, t + 0.2, "ramp", axis, amp, 0.2))
        t += 0.2
        events.append((t, t + 0.5, "hold", axis, amp, 0.0))
        t += 0.5
        events.append((t, t + 0.2, "release", axis, amp, 0.2))
        t += 0.2
    return events, t


def _force_at(events, t) -> List[float]:
    fvec = [0.0, 0.0, 0.0]
    for t0, t1, kind, axis, amp, dur in events:
        if t0 <= t < t1:
            if kind == "ramp":
                frac = (t - t0) / dur if dur > 0 else 1.0
                fvec[axis] = -amp * frac
            elif kind == "hold":
                fvec[axis] = -amp
            elif kind == "release":
                frac = (t - t0) / dur if dur > 0 else 0.0
                fvec[axis] = -amp * max(0.0, 1.0 - frac)
            # "pause": stays 0
            return fvec
    return fvec


def _capture_one(wall_side: str, pattern_name: str, events, total_s: float) -> Dict:
    ns = unique_ns("cap")
    wall_env = {"TOUCH_SIM_ENV": "wall", "TOUCH_SIM_WALL_Y": "-0.05", "TOUCH_SIM_WALL_K": "2000"}
    device = "Left" if wall_side == "master" else "Default Device"
    home = str(Path.home())
    params = {"dob_enable": "true", "require_deadman": "false",
              "dob_torque_f_max_n": "0.6"}
    if wall_side == "slave":
        # BUG FOUND 2026-09-04, fixed here: touch_slave.cpp's own
        # declare_parameter defaults for dob_inertia_json/dob_gravity_json
        # ALWAYS point at the left-arm fit (inertia_left.json/gravity_left_
        # minimal.json), and this capture script was previously launching
        # its "slave" (C1-source, right-arm role) SimNode without ever
        # overriding them -- so it was silently using the LEFT arm's
        # identified gravity/inertia coefficients on a node meant to
        # represent the RIGHT arm. The real run_slave_right.sh explicitly
        # overrides DOB_INERTIA_JSON/DOB_GRAVITY_JSON to the *_right.json
        # files (confirmed by reading that script directly) -- production
        # does NOT have this bug, only this sim capture did. Matters here
        # specifically because the C1/C4 shoulder-mask question this script
        # feeds into is about whether a per-arm identification quality
        # issue applies to one arm, both, or neither -- using the wrong
        # arm's coefficients would misrepresent exactly that.
        params["dob_inertia_json"] = f"{home}/dob_logs/results/inertia_right.json"
        params["dob_gravity_json"] = f"{home}/dob_logs/results/gravity_right_minimal.json"
    with SimNode(
        ns, device=device,
        params=params,
        sim_env=wall_env,
    ) as sim:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        with LiveNode(f"cap_{pattern_name}") as live:
            live.watch("fe", sim.topic("force_ext_hat"), ForceEstimate,
                      extract=lambda m: (list(m.tau_ext_nm), list(m.tau_ext_deadbanded_nm)))
            live.watch("q", sim.topic("joint_angles"), Vector3Stamped,
                      extract=lambda m: (m.vector.x, m.vector.y, m.vector.z))
            live.spin_for(1.0)
            settle_q = live.latest.get("q")

            t0 = time.time()
            samples = []
            while time.time() - t0 < total_s:
                t = time.time() - t0
                fvec = _force_at(events, t)
                live.push_force(sim.topic("force_cmd"), sim.topic("enable"), *fvec)
                live.spin_once(timeout_sec=0.004)
                fe = live.latest.get("fe")
                if fe is not None:
                    raw, deadbanded = fe
                    samples.append({"t": t, "raw": raw, "deadbanded": deadbanded})
            settle_q_end = live.latest.get("q")

    return {
        "wall_side": wall_side, "pattern": pattern_name,
        "settle_q_start": list(settle_q) if settle_q else [0, 0, 0],
        "settle_q_end": list(settle_q_end) if settle_q_end else [0, 0, 0],
        "samples": samples,
    }


def main() -> None:
    patterns = {}
    patterns["escalating_fast"] = _push_profile_escalating(pause_s=0.0)
    patterns["escalating_slow"] = _push_profile_escalating(pause_s=2.5)
    patterns["multiaxis_fast"] = _push_profile_multiaxis_fast()

    for wall_side in ("slave", "master"):  # slave=C1 source, master=C4 source
        for pattern_name, (events, total_s) in patterns.items():
            print(f"=== capturing wall_side={wall_side} pattern={pattern_name} "
                  f"({total_s:.1f}s) ===", flush=True)
            result = _capture_one(wall_side, pattern_name, events, total_s)
            out_path = OUT_DIR / f"{wall_side}_{pattern_name}.json"
            with open(out_path, "w") as f:
                json.dump(result, f)
            print(f"  {len(result['samples'])} samples -> {out_path}")


if __name__ == "__main__":
    main()
