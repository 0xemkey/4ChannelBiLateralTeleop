#!/usr/bin/env python3
"""Sim-only performance baseline for the ACTIVE C1/C4 Cartesian-relay path.

CONTEXT (2026-09-04): touch_slave.cpp's "C1/C4-onto-Cartesian relay" block
(added 2026-09-03) is what actually runs when joint_space_control:=false
(the default on every real-hardware launch script -- see run_master_left.sh /
run_slave_right.sh, neither overrides it). This is DIFFERENT from the older
joint-space PD path (joint_space_control:=true), which was evaluated for
production and rejected (see touch_slave.cpp's own comment at the relay
block: "the joint-space path's own C2/C3 position tracking was found live to
feel structurally worse... This block lets C1/C4 ride alongside the
Cartesian bridge's already-good position feel instead").

IMPORTANT CAVEAT, per RUNNING_SCRIPTS.txt's own C1 section as of this
writing: bilateral_4ch.py's C1/C4 relay is labelled "SIM-VERIFIED ONLY, not
run on real hardware" -- that section predates the 2026-09-03 Cartesian-relay
addition and still describes the OLD joint_space_control:=true requirement,
but the underlying fact (C1/C4 has never actually been driven on the real
rig, via either mechanism) still holds. This script is exactly the kind of
sim-only preparation FOUR_CHANNEL_TODO.md's own roadmap (item 10, "C4
first... scope the first test as sim-only... at a low ceiling before any
real-hardware torque write") calls for before that real-hardware step is
attempted -- it does not claim the path is production-proven, only that it
is the code path DESIGNATED to eventually be production, sim-characterized
here.

WHAT THIS MEASURES
-------------------
Two touch_slave_sim instances (master=Left, slave=Default Device), NO
bridge_master_slave.py (its own --reflect-enable force mechanism is a
SEPARATE, non-DOB force-feedback path -- see RUNNING_SCRIPTS.txt's "Feeling
the slave hit something" section -- and would confound isolating C1/C4's own
contribution, so it is deliberately excluded here). bilateral_4ch.py runs
with position weights (w2, w3) at 0 -- this harness characterizes the force
channels in isolation, not the position-tracking system.

For each scenario (C1: slave hits a wall, relayed to master; C4: master hits
a wall, relayed to slave) a controlled ramp-hold-release push profile is
applied via LiveNode.push_force into a wall contact (environment.hpp) -- a
commanded force_cmd push is fully accounted for in the SAME node's own
tau_cmd before its observer ever sees it (confirmed in test_c1_relay_live.py's
own docstring), so genuine wall contact is required to produce a real,
DOB-detectable external disturbance; a bare force_cmd push cannot substitute
for it.

Each case is run TWICE -- at the target relay weight, and at weight=0 (relay
disabled, everything else identical) -- and the difference isolates the
Cartesian relay's own contribution on the peer device's REAL applied_force
(the actual rendered Cartesian force, not the pre-conversion joint torque),
the same "on vs off" isolation test_c4_tracking_improvement.py already uses.

TOUCH_SIM_TRUTH_CSV is enabled on both nodes for full 1kHz ground truth
(q, qdot, tau_cmd, true F_ext/tau_ext, tip position) -- see hd_sim_api.cpp's
writeTruth(). This is what makes rise/settling time, overshoot, and torque
saturation fraction measurable precisely, rather than estimated from the
200 Hz ROS topic rate alone.

SAFETY: sim-only. Every process this script launches is touch_slave_sim
(linked against hd_sim, hd_sim_api.cpp -- see its own header: "linked against
hd_sim, never real hardware"), the same binary sim/tests/*.py already uses.
No OpenHaptics device is opened; no HD_* call in this dependency chain can
reach real hardware. See _assert_sim_only() below for a belt-and-suspenders
runtime check.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).parent))
from sim_common import LiveNode, ScriptProcess, SimNode, read_truth_csv, unique_ns  # noqa: E402

from geometry_msgs.msg import WrenchStamped  # noqa: E402

BILATERAL_SCRIPT = (
    Path(__file__).resolve().parents[5]
    / "projects" / "teleop_left_master_right_slave" / "scripts" / "bilateral_4ch.py"
)

# Production defaults, matching configureDob() in touch_slave.cpp and the
# real launch scripts (run_master_left.sh / run_slave_right.sh both default
# DOB_TORQUE_F_MAX_N=0.6 unless overridden).
DOB_TORQUE_F_MAX_N = "0.6"
MAX_TORQUE_NM = {"waist": 0.16200, "shoulder": 0.08100, "elbow": 0.08100}

TRUTH_DIR = Path("/tmp/c1c4_baseline_truth")
TRUTH_DIR.mkdir(exist_ok=True)


def _assert_sim_only() -> None:
    """Belt-and-suspenders check: this script must only ever spawn
    touch_slave_sim (see sim_common._resolve_touch_slave_sim_exe -- resolves
    via ament_index against the touch_haptics package's own lib dir, not
    PATH), never the real hardware touch_slave / touch_master binaries. Fails
    loudly before launching anything if that resolution would somehow point
    elsewhere.
    """
    from sim_common import _resolve_touch_slave_sim_exe
    exe = _resolve_touch_slave_sim_exe()
    assert "touch_slave_sim" in exe, f"refusing to launch non-sim binary: {exe}"
    assert os.path.isfile(exe), f"sim binary missing: {exe}"


# ---------------------------------------------------------------------------
# Push profile: ramp 0->peak over ramp_s, hold for hold_s, release over
# release_s, settle for settle_s. Same shape as test_c1_oscillation_trip_
# repeated_push_baseline's single-cycle profile, parametrized.
# ---------------------------------------------------------------------------
def _drive_push(live: LiveNode, force_topic: str, enable_topic: str,
                axis: int, peak_n: float, ramp_s: float, hold_s: float,
                release_s: float, settle_s: float,
                sample_cb=None, peer_enable_topic: Optional[str] = None) -> None:
    """peer_enable_topic: the RECEIVING node's own /enable topic. The
    Cartesian relay block in touch_slave.cpp is gated on `command.enabled`
    for the node it runs on (same safety gate the real spring/damper force
    requires -- a relayed signal must not move a device the operator has not
    enabled), which is normally driven by teleop's own enable flow, not by
    force_cmd at all. Without publishing this continuously, the peer's
    `command.enabled` stays false (enabled_on_start_ defaults false) and the
    relay is silently gated off regardless of weight -- confirmed live: an
    earlier version of this script omitted this and every case, including
    weight=1.0, measured exactly 0.0 N on the peer."""
    t0 = time.time()
    total = ramp_s + hold_s + release_s + settle_s
    zero = WrenchStamped()
    while True:
        t = time.time() - t0
        if t >= total:
            break
        if t < ramp_s:
            f = peak_n * (t / ramp_s)
        elif t < ramp_s + hold_s:
            f = peak_n
        elif t < ramp_s + hold_s + release_s:
            rt = t - ramp_s - hold_s
            f = peak_n * max(0.0, 1.0 - rt / release_s)
        else:
            f = 0.0
        fvec = [0.0, 0.0, 0.0]
        fvec[axis] = -f  # negative: matches test_c1_relay_live.py's push-into-wall sign
        live.push_force(force_topic, enable_topic, *fvec)
        if peer_enable_topic is not None:
            # Zero commanded force, enable=true only -- the peer's motion
            # (if any) should come ENTIRELY from the relayed Cartesian force,
            # not from a competing force_cmd.
            live.push_force(peer_enable_topic[0], peer_enable_topic[1], 0.0, 0.0, 0.0)
        live.spin_once(timeout_sec=0.005)
        if sample_cb is not None:
            sample_cb(t, f)


class TopicSampler:
    """Collects (t, value) samples from a watched LiveNode key, timestamped
    against a shared t0 so multiple sources line up on one time axis."""

    def __init__(self, t0: float):
        self.t0 = t0
        self.samples: List[Tuple[float, float]] = []

    def sample(self, value: Optional[float]) -> None:
        if value is None:
            return
        self.samples.append((time.time() - self.t0, value))


def _wrench_norm(msg: WrenchStamped) -> float:
    f = msg.wrench.force
    return math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)


def _wrench_axis(msg: WrenchStamped, axis: int) -> float:
    f = msg.wrench.force
    return (f.x, f.y, f.z)[axis]


def _launch_pair(wall_side: str, dob_torque_f_max_n: str = DOB_TORQUE_F_MAX_N):
    """wall_side: 'master' or 'slave' -- which node gets the wall contact
    model (the genuine-disturbance source for this scenario). The other node
    is free (no environment), matching whichever role it plays as the relay
    RECEIVER. Both nodes always run production-default joint_space_control
    (false, i.e. omitted -- SimNode does not set it, so touch_slave_sim's
    own declare_parameter default applies, same as the real hardware
    launch scripts)."""
    master_ns = unique_ns("m")
    slave_ns = unique_ns("s")

    wall_env = {
        "TOUCH_SIM_ENV": "wall",
        "TOUCH_SIM_WALL_Y": "-0.05",
        "TOUCH_SIM_WALL_K": "2000",
    }
    master_env = wall_env if wall_side == "master" else {}
    slave_env = wall_env if wall_side == "slave" else {}

    # hd_sim_api.cpp's writeTruth() appends ".{device_name}.csv" itself
    # (spaces -> underscores) whenever the env value has no "%s" -- so the
    # value handed here must NOT already end in .csv, or the real file ends
    # up double-suffixed (confirmed live: an earlier version passed
    # "{ns}.csv" and the actual file was "{ns}.csv.Left.csv", silently never
    # found by read_truth_csv, which returned {} for every case).
    master_truth_stem = str(TRUTH_DIR / master_ns)
    slave_truth_stem = str(TRUTH_DIR / slave_ns)
    master_truth = f"{master_truth_stem}.Left.csv"
    slave_truth = f"{slave_truth_stem}.Default_Device.csv"
    master_env = dict(master_env, TOUCH_SIM_TRUTH_CSV=master_truth_stem)
    slave_env = dict(slave_env, TOUCH_SIM_TRUTH_CSV=slave_truth_stem)

    master = SimNode(
        master_ns, device="Left",
        params={
            "dob_enable": "true", "require_deadman": "false",
            "dob_torque_f_max_n": dob_torque_f_max_n,
            "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
            "max_force_n": "1.5",
        },
        sim_env=master_env,
    )
    slave = SimNode(
        slave_ns, device="Default Device",
        params={
            "dob_enable": "true", "require_deadman": "false",
            "dob_torque_f_max_n": dob_torque_f_max_n,
            "stiffness_n_per_m": "35.0", "damping_n_s_per_m": "0.20",
            "max_force_n": "1.5",
        },
        sim_env=slave_env,
    )
    master.wait_started()
    slave.wait_started()
    for sim in (master, slave):
        assert "DOB bank configured" in sim.log_text(), (
            f"{sim.namespace}: DOB never configured; see {sim.log_text()[-800:]}"
        )
    return master, slave, master_truth, slave_truth


def _run_case(scenario: str, weight: float, axis: int = 1,
             peak_n: float = 0.6, c1_disable_joints: str = "shoulder",
             dob_torque_f_max_n: str = DOB_TORQUE_F_MAX_N) -> Dict:
    """scenario: 'c1' (slave wall -> master relay) or 'c4' (master wall ->
    slave relay). weight: the relay weight under test (w1 for c1, w4 for c4);
    the OTHER three weights are always 0 so this run isolates exactly one
    channel. Returns a dict of raw time series + derived metrics."""
    assert scenario in ("c1", "c4")
    wall_side = "slave" if scenario == "c1" else "master"
    push_side = wall_side  # push whichever side has the wall, into it
    peer_side = "master" if scenario == "c1" else "slave"

    master, slave, master_truth, slave_truth = _launch_pair(wall_side, dob_torque_f_max_n)
    push_node = slave if push_side == "slave" else master

    w1 = weight if scenario == "c1" else 0.0
    w4 = weight if scenario == "c4" else 0.0

    try:
        status_topic = f"/{unique_ns('status')}"
        args = [
            str(BILATERAL_SCRIPT),
            "--architecture", "custom",
            "--w1", str(w1), "--w2", "0.0", "--w3", "0.0", "--w4", str(w4),
            "--c1-disable-joints", c1_disable_joints,
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
        ]
        with ScriptProcess(args, name=f"bilateral_{master.namespace}") as bilateral:
            bilateral.wait_for_marker("bilateral_4ch started", timeout=10.0)

            with LiveNode("baseline") as live:
                # last_force, NOT applied_force -- see the module docstring's
                # "REAL BUG FOUND" note. applied_force is populated from
                # force_ros, captured BEFORE the C1/C4 relay's addition to
                # force_device (touch_slave.cpp ~line 1301 vs ~1244-1246), so
                # it silently omits the relay's contribution whenever C1/C4
                # is active. last_force reads HD_LAST_FORCE, the driver's own
                # readback AFTER the actual write (one frame lagged), which
                # does include it -- confirmed directly against the
                # [diag_relay_math] stderr trace during this harness's own
                # debugging (force_device_pre + f_ext_added matched last_force
                # one frame later; applied_force stayed at force_device_pre
                # forever).
                peer_topic = master.topic("last_force") if peer_side == "master" \
                    else slave.topic("last_force")
                src_topic = slave.topic("last_force") if push_side == "slave" \
                    else master.topic("last_force")
                live.watch("peer_force", peer_topic, WrenchStamped,
                          extract=lambda m: _wrench_axis(m, axis))
                live.watch("src_force", src_topic, WrenchStamped,
                          extract=lambda m: _wrench_axis(m, axis))

                live.spin_for(1.0)  # settle before driving

                t0 = time.time()
                peer_series = TopicSampler(t0)
                src_series = TopicSampler(t0)
                push_series = TopicSampler(t0)

                def _sample(t, f):
                    push_series.sample(f)
                    peer_series.sample(live.latest.get("peer_force"))
                    src_series.sample(live.latest.get("src_force"))

                force_topic = push_node.topic("force_cmd")
                enable_topic = push_node.topic("enable")
                peer_node = master if peer_side == "master" else slave
                _drive_push(
                    live, force_topic, enable_topic, axis=axis, peak_n=peak_n,
                    ramp_s=0.5, hold_s=2.0, release_s=0.5, settle_s=1.5,
                    sample_cb=_sample,
                    peer_enable_topic=(peer_node.topic("force_cmd"), peer_node.topic("enable")),
                )

                tripped = "OSCILLATION ABORT" in bilateral.log_text()
                bilateral_log = bilateral.log_text()

        return {
            "scenario": scenario, "weight": weight, "axis": axis,
            "peak_n": peak_n, "c1_disable_joints": c1_disable_joints,
            "dob_torque_f_max_n": dob_torque_f_max_n,
            "peer_series": peer_series.samples,
            "src_series": src_series.samples,
            "push_series": push_series.samples,
            "oscillation_tripped": tripped,
            "master_truth": master_truth, "slave_truth": slave_truth,
            "bilateral_log_tail": bilateral_log[-2000:],
        }
    finally:
        master.stop()
        slave.stop()


# ---------------------------------------------------------------------------
# Derived metrics
# ---------------------------------------------------------------------------
def _interp(series: List[Tuple[float, float]], t: float) -> float:
    if not series:
        return 0.0
    if t <= series[0][0]:
        return series[0][1]
    if t >= series[-1][0]:
        return series[-1][1]
    for i in range(1, len(series)):
        t0, v0 = series[i - 1]
        t1, v1 = series[i]
        if t0 <= t <= t1:
            if t1 == t0:
                return v1
            frac = (t - t0) / (t1 - t0)
            return v0 + frac * (v1 - v0)
    return series[-1][1]


def _rise_settling_overshoot(series: List[Tuple[float, float]],
                             ramp_s: float, hold_s: float,
                             target: float) -> Dict:
    """Rise time (10-90% of target, measured from ramp start), settling time
    (time to enter and stay within 5% of target, measured from ramp start),
    overshoot (max value beyond target during hold, as a fraction)."""
    if not series or abs(target) < 1e-9:
        return {"rise_s": None, "settling_s": None, "overshoot_frac": None}
    vals_after_ramp = [(t, v) for t, v in series if t <= ramp_s + hold_s]
    if not vals_after_ramp:
        return {"rise_s": None, "settling_s": None, "overshoot_frac": None}

    sign = 1.0 if target >= 0 else -1.0
    lo, hi = 0.1 * abs(target), 0.9 * abs(target)
    t_lo = t_hi = None
    for t, v in vals_after_ramp:
        av = sign * v
        if t_lo is None and av >= lo:
            t_lo = t
        if t_hi is None and av >= hi:
            t_hi = t
            break
    rise_s = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else None

    band = 0.05 * abs(target)
    settling_s = None
    for t, v in vals_after_ramp:
        if t < ramp_s:
            continue
        if abs(sign * v - abs(target)) <= band:
            rest = [vv for tt, vv in vals_after_ramp if tt >= t]
            if all(abs(sign * vv - abs(target)) <= band for vv in rest):
                settling_s = t
                break

    max_signed = max((sign * v for _, v in vals_after_ramp), default=0.0)
    overshoot_frac = max(0.0, (max_signed - abs(target)) / abs(target))

    return {"rise_s": rise_s, "settling_s": settling_s, "overshoot_frac": overshoot_frac}


def _steady_state_error(peer_series, src_series, ramp_s, hold_s) -> Optional[float]:
    """|peer - src| averaged over the last 0.5s of the hold window --
    'closure' in the user's terms: how much of the source's true applied
    force actually reached the peer device."""
    window = [(t, v) for t, v in peer_series if ramp_s + hold_s - 0.5 <= t <= ramp_s + hold_s]
    if not window:
        return None
    peer_avg = statistics.mean(v for _, v in window)
    src_avg = statistics.mean(
        v for t, v in src_series if ramp_s + hold_s - 0.5 <= t <= ramp_s + hold_s
    ) if src_series else 0.0
    return peer_avg, src_avg


def _torque_headroom_from_truth(truth_path: str, hold_s: float,
                                release_s: float, settle_s: float) -> Dict:
    """From the SOURCE node's own truth CSV (q, tau_cmd), compute per-joint
    max |tau_cmd| during the hold window and its fraction of MAX_TORQUE_NM
    (production ceiling at dob_torque_f_max_n=0.6N) -- the actual commanded
    joint torque this run produced, not a theoretical figure.

    The truth CSV's t_s is the SIM PROCESS's own internal clock (running
    since the node launched, seconds before this test's own wall-clock t0
    started driving), not aligned to the push profile's t=0 -- so the hold
    window is located relative to the END of the log instead (last_t minus
    the known release+settle tail), which is robust to that offset without
    needing to correlate wall-clock time against the sim's internal clock."""
    try:
        rows = read_truth_csv(truth_path)
    except FileNotFoundError:
        return {}
    if not rows:
        return {}
    last_t = rows[-1]["t_s"]
    hold_end = last_t - release_s - settle_s
    hold_start = hold_end - hold_s
    window = [r for r in rows if hold_start <= r["t_s"] <= hold_end]
    if not window:
        return {}
    out = {}
    for i, name in enumerate(("waist", "shoulder", "elbow")):
        vals = [abs(r[f"tau_cmd{i+1}"]) for r in window]
        peak = max(vals) if vals else 0.0
        ceiling = MAX_TORQUE_NM[name]
        out[name] = {
            "peak_tau_nm": peak,
            "ceiling_nm": ceiling,
            "budget_used_frac": peak / ceiling if ceiling > 0 else None,
            "frac_time_saturated": (
                sum(1 for v in vals if v >= 0.98 * ceiling) / len(vals) if vals else 0.0
            ),
        }
    qdots = [
        max(abs(r["qdot1"]), abs(r["qdot2"]), abs(r["qdot3"])) for r in window
    ]
    out["max_qdot_rad_s"] = max(qdots) if qdots else 0.0
    return out


def _noise_std(series: List[Tuple[float, float]], t_start: float, t_end: float) -> Optional[float]:
    vals = [v for t, v in series if t_start <= t <= t_end]
    if len(vals) < 3:
        return None
    return statistics.pstdev(vals)


def analyze_case(result: Dict) -> Dict:
    ramp_s, hold_s = 0.5, 2.0
    target_sign = -1.0  # push_series/peer_series follow the same negative-axis convention
    peer_kin = _rise_settling_overshoot(
        result["peer_series"], ramp_s, hold_s, target=target_sign * result["peak_n"])
    ss = _steady_state_error(result["peer_series"], result["src_series"], ramp_s, hold_s)
    peer_avg, src_avg = ss if ss else (0.0, 0.0)
    closure_frac = (peer_avg / src_avg) if src_avg not in (0.0, None) and abs(src_avg) > 1e-6 else None

    source_node_truth = result["master_truth"] if result["scenario"] == "c4" else result["slave_truth"]
    headroom = _torque_headroom_from_truth(source_node_truth, hold_s, release_s=0.5, settle_s=1.5)

    noise_hold = _noise_std(result["peer_series"], ramp_s + 0.5, ramp_s + hold_s)
    noise_settle = _noise_std(result["peer_series"], ramp_s + hold_s + 1.0, ramp_s + hold_s + 1.5)

    return {
        "scenario": result["scenario"],
        "weight": result["weight"],
        "peer_force_avg_hold_n": peer_avg,
        "src_force_avg_hold_n": src_avg,
        "closure_frac": closure_frac,
        "rise_s": peer_kin["rise_s"],
        "settling_s": peer_kin["settling_s"],
        "overshoot_frac": peer_kin["overshoot_frac"],
        "torque_headroom": headroom,
        "peer_force_noise_std_hold_n": noise_hold,
        "peer_force_noise_std_settle_n": noise_settle,
        "oscillation_tripped": result["oscillation_tripped"],
    }


def main() -> None:
    _assert_sim_only()
    p = argparse.ArgumentParser()
    p.add_argument("--peak-n", type=float, default=0.6)
    p.add_argument("--axis", type=int, default=1, help="0=x,1=y,2=z; y matches the wall's own axis")
    p.add_argument("--out", type=Path, default=Path("/tmp/c1c4_baseline_results.json"))
    args = p.parse_args()

    weight_cases = {
        "c1": [0.0, 0.5, 1.0],   # 0.0 = relay-off control, 0.5 = last real-hardware value, 1.0 = full "4ch"
        "c4": [0.0, 0.3, 1.0],   # 0.0 = control, 0.3 = "4ch_safe" preset value, 1.0 = full "4ch"
    }

    all_results = []
    for scenario, weights in weight_cases.items():
        for w in weights:
            print(f"\n=== running scenario={scenario} weight={w} ===", flush=True)
            raw = _run_case(scenario, w, axis=args.axis, peak_n=args.peak_n)
            metrics = analyze_case(raw)
            metrics["raw_peer_series"] = raw["peer_series"]
            metrics["raw_src_series"] = raw["src_series"]
            all_results.append(metrics)
            print(json.dumps({k: v for k, v in metrics.items() if not k.startswith("raw_")}, indent=2))

    with open(args.out, "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\nWrote {args.out}")


if __name__ == "__main__":
    main()
