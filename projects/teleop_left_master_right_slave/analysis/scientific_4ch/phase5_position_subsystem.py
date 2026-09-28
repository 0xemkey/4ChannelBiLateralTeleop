#!/usr/bin/env python3
"""Phase 5: C2/C3 position subsystem, analyzed independently of force
reflection (C1/C4 absent -- bridge_master_slave.py has no force channel
of its own; the SimNode's own Cartesian spring/damper is the only force
present, and it exists to track a position target, not to reflect an
external estimate).

100% simulation (touch_slave_sim/hd_sim + bridge_master_slave.py, the
REAL Cartesian-bridge implementation of C2/C3 -- not bilateral_4ch.py's
separate joint-space C2/C3, which this project's own docstring says is
NOT the working path; bridge_master_slave.py is). Confirmed
hardware-independent the same way as every other script here.

Three gain configurations, none invented -- all pulled directly from
existing shipped defaults (run_master_left.sh / run_slave_right.sh):
    baseline     : master=(15.0 N/m, 0.10 Ns/m)  slave=(35.0 N/m, 0.20 Ns/m)  [current, asymmetric]
    symmetric_soft : both=(15.0, 0.10)
    symmetric_stiff: both=(35.0, 0.20)

Driving signal: a constant force step on the master (0.4N, X axis) held
for 4.0s (analogous to a human pushing the master to a new position and
holding), then released (0.0N) for 3.0s -- gives a clean step response
for rise-time/overshoot/settling metrics, plus a release/recovery phase.
Both C2 (slave position -> master target) and C3 (master position ->
slave target) active throughout (--enable-c2 true, C3 is bridge_master_slave.py's default-on behavior).

N=5 repeats per configuration (matching this session's own established
practice of N-repeat runs after Phase 4A's screening lesson).
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path("/home/nural/HIL-RL/ros2_ws/src/touch_haptics/sim/tests")))
from sim_common import LiveNode, ScriptProcess, SimNode, unique_ns  # noqa: E402

from geometry_msgs.msg import PoseStamped  # noqa: E402

BRIDGE_SCRIPT = Path(
    "/home/nural/HIL-RL/projects/teleop_left_master_right_slave/scripts/bridge_master_slave.py")

CONFIGS = {
    "baseline_asymmetric": dict(master_k=15.0, master_d=0.10, slave_k=35.0, slave_d=0.20),
    "symmetric_soft":      dict(master_k=15.0, master_d=0.10, slave_k=15.0, slave_d=0.10),
    "symmetric_stiff":     dict(master_k=35.0, master_d=0.20, slave_k=35.0, slave_d=0.20),
}
N_REPEATS = 5
PUSH_S = 4.0
RELEASE_S = 3.0
PUSH_MAG = 0.4  # N, same magnitude used for the original X-axis run
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
OUT_JSON_TEMPLATE = str(Path(__file__).parent / "phase5_position_raw_{axis}.json")


def one_run(cond: str, params: dict, run_idx: int, axis: str) -> dict:
    master_ns = unique_ns(f"p5{axis}_{cond}_m{run_idx}")
    slave_ns = unique_ns(f"p5{axis}_{cond}_s{run_idx}")
    record = {"condition": cond, "run_idx": run_idx, "axis": axis, **params}

    with SimNode(master_ns, device="Left",
                 params={"stiffness_n_per_m": str(params["master_k"]),
                          "damping_n_s_per_m": str(params["master_d"]),
                          "require_deadman": "false"}) as master, \
         SimNode(slave_ns, device="Default Device",
                 params={"stiffness_n_per_m": str(params["slave_k"]),
                          "damping_n_s_per_m": str(params["slave_d"]),
                          "require_deadman": "false"}) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with ScriptProcess(
            [str(BRIDGE_SCRIPT),
             "--left-pose-topic", master.topic("pose"),
             "--right-pose-topic", slave.topic("pose"),
             "--right-target-topic", slave.topic("target_pose"),
             "--right-enable-topic", slave.topic("enable"),
             "--left-target-topic", master.topic("target_pose"),
             "--left-enable-topic", master.topic("enable"),
             "--enable-c2", "true",
             "--left-max-offset-m", "0.08",
             "--calibration-file", str(calib_path),
             "--axis-map", "x,y,z",
             "--rate-hz", "100",
             "--wait-timeout-s", "15",
             ],
            name=f"bridge_{master_ns}",
        ) as bridge:
            try:
                bridge.wait_for_marker("teleoperation live", timeout=15.0)
            except Exception as e:  # noqa: BLE001
                record["error"] = f"bridge did not start: {e}"
                return record

            # bridge_master_slave.py's startup_alignment.py state machine
            # requires an explicit arm before its READY->RAMPING->ACTIVE
            # transition (a deliberate operator-confirmation safety gate,
            # not a bug -- found by debugging this session's first attempt,
            # which silently sat at READY forever with target frozen at the
            # anchor). On real hardware this would be a human confirming
            # readiness; in sim, this is the equivalent explicit trigger.
            r = subprocess.run(
                ["ros2", "param", "set", "/touch_master_slave_bridge", "arm_teleop", "true"],
                capture_output=True, text=True, timeout=10)
            if "successful" not in r.stdout:
                record["error"] = f"arm_teleop param set failed: {r.stdout} {r.stderr}"
                return record

            with LiveNode("p5_verify") as live:
                live.watch("master_pose", master.topic("pose"), PoseStamped,
                            extract=lambda m: (m.pose.position.x, m.pose.position.y, m.pose.position.z))
                live.watch("slave_pose", slave.topic("pose"), PoseStamped,
                            extract=lambda m: (m.pose.position.x, m.pose.position.y, m.pose.position.z))
                live.spin_for(2.0)  # let alignment/calibration settle before the push

                series = {"t": [], "master": [], "slave": []}
                t0 = time.time()
                end_time = t0 + PUSH_S + RELEASE_S
                idx = AXIS_INDEX[axis]
                while time.time() < end_time:
                    mag = PUSH_MAG if (time.time() - t0) < PUSH_S else 0.0
                    f = [0.0, 0.0, 0.0]
                    f[idx] = mag
                    live.push_force(master.topic("force_cmd"), master.topic("enable"), *f)
                    live.spin_once(timeout_sec=0.01)
                    mp = live.latest.get("master_pose")
                    sp = live.latest.get("slave_pose")
                    if mp is not None and sp is not None:
                        series["t"].append(time.time() - t0)
                        series["master"].append(mp)
                        series["slave"].append(sp)

    record["series"] = series
    return record


def main(axis: str) -> None:
    results = []
    for cond, params in CONFIGS.items():
        for r in range(N_REPEATS):
            t0 = time.time()
            rec = one_run(cond, params, r, axis)
            dur = time.time() - t0
            n = len(rec.get("series", {}).get("t", []))
            print(f"[{axis}] {cond:20s} run {r}: n_samples={n} dur={dur:.2f}s {rec.get('error', '')}")
            results.append(rec)
    out_path = OUT_JSON_TEMPLATE.format(axis=axis)
    with open(out_path, "w") as f:
        json.dump(results, f)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    axis_arg = sys.argv[1] if len(sys.argv) > 1 else "x"
    main(axis_arg)
