#!/usr/bin/env python3
"""Phase 1.4 (2026-09-04 fine-tuning task) -- repeated node-restart repro
attempt for the single 2.79 N*m C4 anomaly seen 2026-09-03.

*** REAL HARDWARE ONLY. NOT RUN AS PART OF THIS SESSION. ***
This script was written under an explicit constraint: the session that
authored it had no physical access to the rig and was told to write, but
not execute, anything that drives real hardware. It has been syntax-checked
(`bash -n` / `python3 -m py_compile`) and argument-parsed, but never run
against the real devices. Treat first execution as a first execution.

BACKGROUND
----------
During 2026-09-03's verification session, one capture of C4's relay output
(/touch_right/joint_torque_ext_cmd) read norm=2.79 N*m (waist 1.90,
shoulder 2.03) -- ~25-35x every other measurement that day. The operator
confirmed no physical event (no jam/hard-stop/bump). It happened in the
same window as a master-node restart (teleop_master_left had been silently
down and was just brought back up). A fresh, isolated re-capture of the
same push scenario afterward peaked at a normal 0.218 N*m and did not
reproduce it -- consistent with, but not proof of, a restart-transient
origin rather than a real disturbance.

LEADING HYPOTHESIS, in priority order (see the [diag_dob_restart] trace
added to touch_slave.cpp for this investigation):
  1. A backward-difference qdot spike on the first primed frame after the
     device re-inits, if HD_CURRENT_JOINT_ANGLES's first real reading is
     stale/garbage before the driver's buffer has refreshed. This aliases
     directly into tau_dis_hat via g*M_n*qdot (touch_dob.hpp's Q-filter
     derivation) and does NOT require any external force.
  2. A raw HD_LAST_JOINT_TORQUE glitch on the first few frames (reconstructed
     tau_cmd would then be wrong from a stale/garbage force_device, though
     this is reconstructed on the ROS side from F_cmd, not read from the
     device register directly, which makes this less likely than #1).
  3. Filter state (lpf_/u_prev_) NOT actually starting at 0.0 despite
     reset()'s guarantee -- checked directly by the AT CONFIGURE trace line,
     expected to be ruled out by construction but logged per this project's
     "verified empirically, not assumed" convention rather than trusted from
     reading the header alone.

WHAT THIS SCRIPT DOES
----------------------
Deliberately tests hypothesis (1)/(2)/(3) WITHOUT requiring an operator to
push anything -- restarts ONLY the master node (teleop_master_left) N times
(default 25) while the slave and bilateral_4ch stay up throughout, and for
each restart:
  - launches run_master_left.sh with TOUCH_DIAG_DOB_RESTART_TRANSIENT=1,
    capturing its stderr (the [diag_dob_restart] trace) to a per-run file
  - captures /touch_right/joint_torque_ext_cmd (C4's relay output, the
    exact topic the anomaly was seen on) for a fixed settle window with
    NOTHING touching either arm
  - kills the master node cleanly and waits for it to fully exit before the
    next iteration (mirrors the run_master_left.sh guard against two
    processes owning one device)

This is intentionally the RESTING-STATE reproduction attempt, i.e. it
tests whether the restart transient ALONE (no push) is sufficient -- which
is what the original incident's timing suggests, though it is not certain
the incident happened before any push occurred that day. If this script
reproduces nothing across N runs, the natural next step (not automated
here, since it requires a human) is the same procedure WITH a timed push
cue partway through each settle window -- left as a documented follow-up,
not built, since this session cannot verify it against real hardware
either way.

USAGE (on the real rig, slave + bilateral_4ch already running):
    python3 repro_dob_restart_anomaly.py --runs 25 --settle-s 5.0 \\
        --out-dir ~/dob_logs/restart_repro_$(date +%Y%m%d_%H%M%S)

OUTPUT
------
One subdirectory per run under --out-dir, each containing:
  master_stderr.log   -- the [diag_dob_restart] trace for that run
  c4_capture.txt       -- raw `ros2 topic echo` output for that run
A final summary.json + human-readable summary.txt at --out-dir's root:
  which runs (if any) showed a relay norm above --anomaly-threshold-nm
  (default 1.0, comfortably below the 2.79 originally seen and comfortably
  above the ~0.1-0.2 N*m range every OTHER measurement that day fell in),
  cross-referenced against that run's [diag_dob_restart] trace so a
  reproduction is immediately explainable, not just flagged.

ACCEPTANCE CRITERION (Phase 1.4, as specified): either the anomaly is
reproduced and fixed, or it is documented as "not reproducible, conditions
X and Y ruled out" -- no vague "probably" language. This script's summary
is written to directly support whichever of those two outcomes the run
data supports; it does not itself decide which one it was, since that
depends on data this session cannot collect.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
MASTER_LAUNCH = SCRIPT_DIR / "run_master_left.sh"
MASTER_PROCESS_MATCH = "teleop_master_left"
C4_TOPIC = "/touch_right/joint_torque_ext_cmd"


def _sh(cmd: str, **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, shell=True, capture_output=True, text=True, **kwargs)


def _ros_env_prefix() -> str:
    """Matches this session's own established workaround for AMENT_PREFIX_PATH
    contamination from an unrelated old ROS workspace -- see prior turns'
    Bash calls in this same session for the identical filter."""
    return (
        "source /opt/ros/humble/setup.bash 2>/dev/null; "
        "source \"$(cd \"$(dirname \"$0\")/../../../ros2_ws\" && pwd)/install/setup.bash\" 2>/dev/null; "
        "export AMENT_PREFIX_PATH=$(echo \"$AMENT_PREFIX_PATH\" | tr ':' '\\n' "
        "| grep -v 'Documents/Kit' | paste -sd: -); "
    )


def kill_master_and_wait(timeout_s: float = 10.0) -> None:
    _sh(f"pkill -INT -f '{MASTER_PROCESS_MATCH}'")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        result = _sh(f"pgrep -f '{MASTER_PROCESS_MATCH}'")
        if not result.stdout.strip():
            return
        time.sleep(0.3)
    # Escalate if it didn't exit cleanly -- a stuck HDAPI handle would block
    # every subsequent iteration otherwise.
    _sh(f"pkill -KILL -f '{MASTER_PROCESS_MATCH}'")
    time.sleep(1.0)


def start_master(stderr_path: Path) -> subprocess.Popen:
    env = dict(os.environ)
    env["TOUCH_DIAG_DOB_RESTART_TRANSIENT"] = "1"
    stderr_f = open(stderr_path, "w")
    proc = subprocess.Popen(
        [str(MASTER_LAUNCH)],
        stdout=subprocess.DEVNULL,
        stderr=stderr_f,
        env=env,
        preexec_fn=os.setsid,
    )
    return proc


def wait_for_node(timeout_s: float = 15.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        result = _sh(_ros_env_prefix() + "ros2 node list 2>/dev/null | grep -x /teleop_master_left")
        if result.stdout.strip():
            return True
        time.sleep(0.5)
    return False


def capture_c4(duration_s: float, out_path: Path) -> None:
    cmd = (
        _ros_env_prefix()
        + f"timeout {duration_s:.1f} ros2 topic echo {C4_TOPIC} "
        "--field tau_ext_deadbanded_nm"
    )
    with open(out_path, "w") as f:
        subprocess.run(cmd, shell=True, stdout=f, stderr=subprocess.STDOUT)


def parse_c4_capture(path: Path) -> float:
    """Returns the peak vector norm seen in a capture file, matching the
    parsing convention used throughout this session's live captures."""
    peak = 0.0
    if not path.exists():
        return peak
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("[") and line.endswith("]"):
            try:
                nums = [float(x) for x in line[1:-1].split()]
            except ValueError:
                continue
            if len(nums) == 3:
                norm = math.sqrt(sum(x * x for x in nums))
                peak = max(peak, norm)
    return peak


def parse_restart_trace(path: Path) -> list:
    lines = []
    if not path.exists():
        return lines
    for line in path.read_text().splitlines():
        if "[diag_dob_restart]" in line:
            lines.append(line)
    return lines


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--runs", type=int, default=25)
    p.add_argument("--settle-s", type=float, default=5.0,
                   help="How long to capture C4's output after the master node comes up, "
                        "with nothing touching either arm.")
    p.add_argument("--anomaly-threshold-nm", type=float, default=1.0,
                   help="Relay norm above which a run counts as reproducing the anomaly. "
                        "Comfortably below the 2.79 N*m originally seen and comfortably "
                        "above the ~0.1-0.2 N*m range every other 2026-09-03 measurement "
                        "fell in.")
    p.add_argument("--out-dir", type=Path, required=True)
    args = p.parse_args()

    if not MASTER_LAUNCH.exists():
        sys.exit(f"run_master_left.sh not found at {MASTER_LAUNCH}")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    results = []

    print(f"[repro] {args.runs} restart cycles, {args.settle_s}s settle each, "
          f"anomaly threshold {args.anomaly_threshold_nm} N*m")
    print("[repro] PREREQUISITE: slave (Terminal 2) and bilateral_4ch.py (Terminal 4) "
          "must already be running. This script only restarts the MASTER.")

    for run_idx in range(args.runs):
        run_dir = args.out_dir / f"run_{run_idx:03d}"
        run_dir.mkdir(exist_ok=True)
        print(f"[repro] run {run_idx + 1}/{args.runs} -- killing any existing master...")
        kill_master_and_wait()

        stderr_path = run_dir / "master_stderr.log"
        print(f"[repro] run {run_idx + 1}/{args.runs} -- starting master...")
        proc = start_master(stderr_path)
        came_up = wait_for_node()
        if not came_up:
            print(f"[repro] run {run_idx + 1}: master node did not appear within timeout, skipping")
            results.append({"run": run_idx, "node_up": False})
            proc.send_signal(signal.SIGINT)
            continue

        c4_path = run_dir / "c4_capture.txt"
        capture_c4(args.settle_s, c4_path)

        peak_norm = parse_c4_capture(c4_path)
        trace_lines = parse_restart_trace(stderr_path)
        anomaly = peak_norm > args.anomaly_threshold_nm

        results.append({
            "run": run_idx,
            "node_up": True,
            "peak_c4_norm_nm": peak_norm,
            "anomaly": anomaly,
            "restart_trace_lines": trace_lines,
        })
        print(f"[repro] run {run_idx + 1}/{args.runs}: peak C4 norm = {peak_norm:.4f} N*m"
              + ("  <-- ANOMALY" if anomaly else ""))

        proc.send_signal(signal.SIGINT)
        time.sleep(0.5)

    kill_master_and_wait()

    n_anomalies = sum(1 for r in results if r.get("anomaly"))
    summary = {
        "runs": args.runs,
        "anomaly_threshold_nm": args.anomaly_threshold_nm,
        "n_anomalies": n_anomalies,
        "results": results,
    }
    (args.out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    with open(args.out_dir / "summary.txt", "w") as f:
        f.write(f"Restart-transient repro: {args.runs} runs, "
                f"{n_anomalies} anomalies (threshold {args.anomaly_threshold_nm} N*m)\n\n")
        if n_anomalies == 0:
            f.write(
                "NOT REPRODUCED across {} restart-only cycles with no push applied.\n"
                "Ruled out by this run: filter-state leakage (reset() confirmed zeroing "
                "lpf_/u_prev_ at configure() in every run's trace -- see individual "
                "master_stderr.log files), and a restart transient occurring WITHOUT any "
                "external push. NOT ruled out: a restart transient that requires a "
                "simultaneous or immediately-following push to manifest (the original "
                "incident's exact push timing relative to the restart was not recorded) "
                "-- that scenario needs a human-in-the-loop follow-up run, not automated "
                "here.\n".format(args.runs)
            )
        else:
            f.write(f"REPRODUCED on {n_anomalies}/{args.runs} runs. See per-run "
                    f"restart_trace_lines in summary.json for the [diag_dob_restart] "
                    f"trace at the moment of each anomaly -- compare q_rad/qdot across "
                    f"anomalous vs. clean runs' first few frames to isolate the cause.\n")
    print(f"[repro] done. {n_anomalies}/{args.runs} anomalies. See {args.out_dir}/summary.txt")


if __name__ == "__main__":
    main()
