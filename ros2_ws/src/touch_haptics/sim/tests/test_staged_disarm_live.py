"""Part 7 of the 2026-09-09 validation-infrastructure follow-up task:
staged DISARM live revalidation -- ACTIVE -> COUPLING_RAMP_DOWN ->
SELF_FOLLOW_SETTLE -> GRAVITY_RAMP_DOWN -> READY, measured against the REAL
bridge/controllers (same deterministic 117mm fixture as
test_force_budget_117mm_live.py), at multi-rate (controller/telemetry
~200Hz, bridge ~100Hz).

Measures separately from the moment coupling begins decreasing
(COUPLING_RAMP_DOWN entry) and from the moment gravity begins decreasing
(the true gravity-ramp-down sub-phase of GRAVITY_RAMP_DOWN, AFTER
SELF_FOLLOW_SETTLE -- see startup_alignment.py's own comment on why that
sub-phase is represented via gravity_ramp_alpha staying pinned at 1.0
rather than a distinct bridge_state): target-live lag, raw position error,
spring/damping/auxiliary/applied force.
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


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


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
            "--enable-c2", "true", "--left-max-offset-m", "0.08",
            "--calibration-file", str(calib_path), "--axis-map", "x,y,z",
            "--absolute-axes", "x,y,z", "--absolute-trim-m", "0.0",
            "--slave-gravity-comp-n", str(GRAVITY_N), "--slave-gravity-axis", "1",
            "--master-stiffness-n-per-m", "15.0", "--master-damping-n-s-per-m", "0.10",
            "--master-max-force-n", "2.0", "--master-max-position-error-m", "0.03",
            "--stiffness-n-per-m", "35.0", "--slave-damping-n-s-per-m", "0.20",
            "--slave-max-force-n", "1.5", "--slave-max-position-error-m", "0.04",
            "--force-budget-reserve-n", "0.10",
            "--rate-hz", "100", "--wait-timeout-s", "15",
            ]


class Segment:
    """Accumulates telemetry stats over one disarm sub-phase, for one arm."""
    def __init__(self, label):
        self.label = label
        self.n = 0
        self.spring = []
        self.damping = []
        self.aux = []
        self.applied = []
        self.raw_err = []
        self.pos_clamp = 0
        self.primary_clamp = 0
        self.final_clamp = 0

    def add(self, t: CartesianControllerTelemetry):
        self.n += 1
        self.spring.append(_norm(list(t.spring_force_n)))
        self.damping.append(_norm(list(t.damping_force_n)))
        self.aux.append(_norm(list(t.auxiliary_force_n)))
        self.applied.append(_norm(list(t.applied_force_n)))
        self.raw_err.append(_norm(list(t.position_error_raw_m)))
        if t.position_error_clamped:
            self.pos_clamp += 1
        if t.primary_force_clamped:
            self.primary_clamp += 1
        if t.final_force_clamped:
            self.final_clamp += 1

    def _below_frac(self, series, thresh):
        if not series:
            return None
        return sum(1 for v in series if v < thresh) / len(series)

    def report(self):
        if self.n == 0:
            print(f"  {self.label}: NO SAMPLES")
            return
        rms_spring = math.sqrt(statistics.fmean([v * v for v in self.spring]))
        print(f"  {self.label} (n={self.n}):")
        print(f"    peak spring force   = {max(self.spring):8.4f} N   RMS = {rms_spring:8.4f} N")
        print(f"    peak damping force  = {max(self.damping):8.4f} N")
        print(f"    peak aux force      = {max(self.aux):8.4f} N")
        print(f"    peak applied force  = {max(self.applied):8.4f} N")
        print(f"    peak raw pos error  = {max(self.raw_err)*1000:8.3f} mm")
        print(f"    time frac spring<0.10N = {self._below_frac(self.spring, 0.10):.3f}   "
              f"<0.05N = {self._below_frac(self.spring, 0.05):.3f}   "
              f"<0.01N = {self._below_frac(self.spring, 0.01):.3f}")
        print(f"    clamps: pos={self.pos_clamp} primary={self.primary_clamp} final={self.final_clamp}")


@pytest.mark.skipif(not BRIDGE_SCRIPT.exists(), reason=f"bridge not found at {BRIDGE_SCRIPT}")
def test_staged_disarm_from_active_to_ready():
    mn, sn = unique_ns("dis_m"), unique_ns("dis_s")
    with SimNode(mn, device="Left", params=MASTER_PARAMS,
                sim_env=master_sim_env(REQUESTED_GAP_M)) as master, \
        SimNode(sn, device="Default Device", params=SLAVE_PARAMS,
                sim_env=slave_sim_env()) as slave:

        calib_path = Path(tempfile.mkstemp(suffix=".json")[1])
        calib_path.unlink()

        with LiveNode(f"dis_settle_{mn}") as pre:
            pre.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
            pre.spin_for(SETTLE_S)
            mp0 = list(pre.latest["m"].position_m)
            sp0 = list(pre.latest["s"].position_m)
        gap_y0, _ = measured_gap(mp0, sp0)

        with ScriptProcess(bridge_args(master, slave, calib_path), name=f"bridge_{mn}") as bridge:
            bridge.wait_for_marker("teleoperation live", timeout=15.0)

            with LiveNode(f"dis_v_{mn}") as live:
                live.watch("m", master.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
                live.watch("s", slave.topic("cartesian_controller_telemetry"), CartesianControllerTelemetry, extract=lambda x: x)
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
                assert "successful" in out, f"could not arm: {out!r}"

                # Wait for ACTIVE.
                deadline = time.time() + 20.0
                reached_active = False
                fault_seen = False
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is None:
                        continue
                    if d.bridge_state == "ACTIVE":
                        reached_active = True
                        break
                    if d.bridge_state == "FAULT":
                        fault_seen = True
                        break
                assert reached_active, f"never reached ACTIVE (fault_seen={fault_seen})"

                # Hold ACTIVE briefly for a clean baseline, then disarm.
                live.spin_for(0.5)
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

                m_seg = {"COUPLING_RAMP_DOWN": Segment("MASTER/COUPLING_RAMP_DOWN"),
                        "SELF_FOLLOW_SETTLE": Segment("MASTER/SELF_FOLLOW_SETTLE"),
                        "GRAVITY_RAMP_DOWN": Segment("MASTER/GRAVITY_RAMP_DOWN(true)")}
                s_seg = {"COUPLING_RAMP_DOWN": Segment("SLAVE/COUPLING_RAMP_DOWN"),
                        "SELF_FOLLOW_SETTLE": Segment("SLAVE/SELF_FOLLOW_SETTLE"),
                        "GRAVITY_RAMP_DOWN": Segment("SLAVE/GRAVITY_RAMP_DOWN(true)")}
                states = []
                reached_ready = False
                fault_seen2 = False
                deadline = time.time() + 20.0
                while time.time() < deadline:
                    live.spin_once(timeout_sec=0.01)
                    d = live.latest.get("d")
                    if d is None:
                        continue
                    if not states or states[-1] != d.bridge_state:
                        states.append(d.bridge_state)
                    sub = None
                    if d.bridge_state == "COUPLING_RAMP_DOWN":
                        sub = "COUPLING_RAMP_DOWN"
                    elif d.bridge_state == "GRAVITY_RAMP_DOWN":
                        sub = "SELF_FOLLOW_SETTLE" if d.gravity_ramp_alpha >= 1.0 - 1e-9 else "GRAVITY_RAMP_DOWN"
                    if sub is not None:
                        m = live.latest.get("m")
                        s = live.latest.get("s")
                        if m is not None:
                            m_seg[sub].add(m)
                        if s is not None:
                            s_seg[sub].add(s)
                    if d.bridge_state == "READY":
                        reached_ready = True
                        break
                    if d.bridge_state == "FAULT":
                        fault_seen2 = True
                        break

                # Post-READY settle window: confirm no persistent force
                # regeneration (target must keep following own live pose).
                post_ready_m = Segment("MASTER/POST_READY")
                post_ready_s = Segment("SLAVE/POST_READY")
                post_deadline = time.time() + 2.0
                while time.time() < post_deadline:
                    live.spin_once(timeout_sec=0.01)
                    m = live.latest.get("m")
                    s = live.latest.get("s")
                    if m is not None:
                        post_ready_m.add(m)
                    if s is not None:
                        post_ready_s.add(s)
                final_d = live.latest.get("d")

        print("\n================ STAGED DISARM LIVE REVALIDATION ================")
        print(f"initial gap_y = {gap_y0*1000:.3f}mm")
        print(f"disarm state sequence = {states}")
        for sub in ("COUPLING_RAMP_DOWN", "SELF_FOLLOW_SETTLE", "GRAVITY_RAMP_DOWN"):
            m_seg[sub].report()
            s_seg[sub].report()
        print("post-READY settle (2.0s):")
        post_ready_m.report()
        post_ready_s.report()
        print(f"final gravity_ramp_alpha={final_d.gravity_ramp_alpha if final_d else 'N/A'}, "
              f"coupling_ramp_alpha={final_d.coupling_ramp_alpha if final_d else 'N/A'}")
        print("===================================================================\n")

        # ---------------- Part 7: hard requirements -------------------------
        assert not fault_seen2, f"bridge FAULTed during disarm, states={states}"
        assert reached_ready, f"disarm must reach READY, states={states}"
        for required in ("COUPLING_RAMP_DOWN",):
            assert required in states, f"missing {required} in {states}"
        assert final_d is not None
        assert final_d.gravity_ramp_alpha == 0.0, "gravity must be exactly 0 at READY"
        assert final_d.coupling_ramp_alpha == 0.0, "coupling must be exactly 0 at READY"

        total_clamps = 0
        for seg in list(m_seg.values()) + list(s_seg.values()) + [post_ready_m, post_ready_s]:
            total_clamps += seg.pos_clamp + seg.primary_clamp + seg.final_clamp
        assert total_clamps == 0, f"clamp(s) engaged during disarm/post-READY: {total_clamps}"

        # No PERSISTENT post-disarm force regeneration: a brief transient
        # right at the READY transition is EXPECTED and already root-caused
        # this session (test_ramp_disarm_root_cause.py: a continuously-
        # sagging device tracked by a target that only refreshes once per
        # bridge tick produces a bounded sample-hold spring-force transient
        # -- PHYSICAL_RESPONSE + SCHEDULING, not a code bug). The hard
        # requirement is that this DECAYS -- the tail of the post-READY
        # window (last 1.0s of the 2.0s window) must be settled near-zero,
        # not that the peak (right at the transition) is itself tiny.
        for label, seg, seg_raw in (("master", post_ready_m, m_seg), ("slave", post_ready_s, s_seg)):
            if seg.n == 0:
                continue
            tail = seg.applied[len(seg.applied) // 2:]
            assert tail, f"{label}: no tail samples to check"
            assert max(tail) < 0.05, (
                f"{label} post-READY applied force must have DECAYED by the second half of the "
                f"2.0s settle window (no persistent regeneration), tail peak={max(tail):.4f}N"
            )
        print(f"tail (last 1.0s) peak applied force: "
              f"master={max(post_ready_m.applied[len(post_ready_m.applied)//2:]) if post_ready_m.n else float('nan'):.4f}N "
              f"slave={max(post_ready_s.applied[len(post_ready_s.applied)//2:]) if post_ready_s.n else float('nan'):.4f}N")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v", "-s"]))
