#!/usr/bin/env python3
"""Root-cause regression tests (2026-09-09) for the real-hardware Trial 1
(0.70N) ramp/disarm transient investigation. Pure Python, no rclpy.

These do NOT test for a code bug -- the investigation found none in the
ramp-origin/target-publication path. They instead QUANTIFY the two
legitimate physical/scheduling effects that explain the observed transients,
using the real functions this project already ships (compute_c2_target,
compute_c3_target, blend_target/blend_left_target), so the explanation is
falsifiable code, not prose.

TEST 1 (ramp): reproduces the real trial's ~117mm PRE_ARM-target-to-ACTIVE-
target separation (slave's gravity-sagged rest pose vs. the master's held
reference pose) and confirms: (a) the ramp origin is exactly the live pose
at arm time (no stale-anchor contribution), (b) the full sweep distance over
the 1.0s ramp is large even though each individual published-target STEP is
tiny (matches the real trial's own boundary-step metrics, ~1-2mm).

TEST 2 (disarm): models the real trial's OWN measured post-disarm slave
velocity (extracted from right_pose.csv, ~0.05-0.22 m/s and accelerating,
i.e. genuine gravity sag once the 0.70N aux force is removed) against a
target that updates only once per bridge tick (10ms, RATE_HZ=100) while the
device itself moves continuously (~5ms, ~200Hz). Confirms the resulting
position-error-driven spring force is bounded by v * sample_hold_delay * K,
and is on the same order as the ~0.05-0.2N transient actually measured --
i.e. NOT a code bug, a real update-rate/physical-sag interaction.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from bridge_master_slave import compute_c2_target, compute_c3_target, apply_axis_map  # noqa: E402
from startup_alignment import TeleopStartupAlignment, AlignmentConfig, READY, GRAVITY_RAMP_UP, ACTIVE  # noqa: E402

IDENTITY_AXIS_MAP = [(0, 1.0), (1, 1.0), (2, 1.0)]
AXIS_SCALE = [1.0, 1.0, 1.0]
MAX_OFFSET = [0.12, 0.19, 0.15]
LEFT_MAX_OFFSET_M = 0.08
DEADBAND_M = 0.0002
ALL_ABSOLUTE = [0, 1, 2]
ZERO_TRIM = [0.0, 0.0, 0.0]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


class FakeClock:
    def __init__(self):
        self.t = 9000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def _mapper(pos):
    return apply_axis_map(IDENTITY_AXIS_MAP, [pos[i] * AXIS_SCALE[i] for i in range(3)])


def test_ramp_origin_fresh_with_real_trial_scale_separation():
    """Real trial: slave's PRE_ARM position (gravity-sagged, target-tracks-
    own-live-pose while unarmed) and the master's held reference position
    were ~117mm apart in absolute-Y. Confirm the ramp origin is exactly the
    live pose at arm time (not the stale startup anchor), and that the full
    sweep distance matches -- large cumulative sweep, tiny per-tick step."""
    clock = FakeClock()
    align = TeleopStartupAlignment(config=AlignmentConfig(), now_fn=clock)

    # Startup anchors, captured once at bridge boot -- deliberately different
    # from where the devices sit by the time of THIS arm (matches reality:
    # anchors are a boot-time snapshot, not re-captured per arm cycle).
    # Kept within AlignmentConfig's own max_alignment_error_m sanity bound
    # (0.15m) -- that check only fires once, at startup VALIDATE_ALIGNMENT,
    # and is unrelated to what this test is about (the ramp origin, captured
    # much later at arm time from wherever the devices are BY THEN, which is
    # free to differ from the anchors by more than that bound).
    left0 = [0.02, -0.03, 0.01]
    right0 = [-0.01, 0.02, -0.03]
    for _ in range(400):
        align.update_master_pose(left0, t=clock.t)
        align.update_slave_pose(right0, t=clock.t)
        align.tick(_mapper)
        if align.state == READY:
            break
        clock.advance(0.01)
    assert align.state == READY

    # By arm time, the slave has sagged (unpowered, gravity comp=0 while
    # unarmed) to a real rest pose ~117mm from the master's held reference
    # pose -- reproducing the real trial's own measured sweep distance.
    master_held = [0.00842704200744629, 0.0005558998584747314, 0.008186971664428711]
    slave_sagged = [master_held[0] + 0.02, master_held[1] - 0.1174, master_held[2] + 0.01]
    gap = norm([master_held[i] - slave_sagged[i] for i in range(3)])
    assert abs(gap - 0.1174) < 0.01, f"test fixture should reproduce the real ~117mm gap, got {gap*1000:.1f}mm"

    align.update_master_pose(master_held, t=clock.t)
    align.update_slave_pose(slave_sagged, t=clock.t)
    align.tick(_mapper)
    assert align.state == READY

    align.arm()
    align.tick(_mapper)
    # 2026-09-09 staged-transition rewrite: arm() now enters GRAVITY_RAMP_UP
    # first, with coupling_ramp_alpha (legacy ramp_alpha) held at exactly 0
    # through both gravity phases -- see startup_alignment.py's module
    # docstring for the full state sequence. There is no more separate
    # "_ramp_origin_right" snapshot to go stale: blend_target() now blends
    # continuously between the LIVE pose and the full target by
    # coupling_ramp_alpha, so at alpha=0 it is ALWAYS exactly the live pose,
    # at every tick, by construction -- a strictly stronger guarantee than a
    # one-time snapshot captured at ramp start (Part 3's "capture fresh
    # poses AGAIN at the start of this phase" is satisfied trivially and
    # continuously, not just once).
    assert align.state == GRAVITY_RAMP_UP
    assert align.coupling_ramp_alpha == 0.0
    c3_full, _ = compute_c3_target(master_held, align.left_anchor, align.right_anchor,
                                    IDENTITY_AXIS_MAP, AXIS_SCALE, DEADBAND_M, MAX_OFFSET, "axis",
                                    absolute_axes=ALL_ABSOLUTE, absolute_trim=ZERO_TRIM)
    published_at_alpha0 = align.blend_target(c3_full)
    origin_err = norm([published_at_alpha0[i] - slave_sagged[i] for i in range(3)])
    stale_anchor_err = norm([published_at_alpha0[i] - right0[i] for i in range(3)])
    print(f"[ramp-origin] |blend_target(alpha=0) - live_pose_at_arm| = {origin_err*1000:.6f}mm (must be ~0)")
    print(f"[ramp-origin] |blend_target(alpha=0) - stale_startup_anchor| = {stale_anchor_err*1000:.2f}mm "
          f"(large -- confirms origin is NOT the stale anchor)")
    assert origin_err < 1e-9, "at coupling_ramp_alpha=0, published target must equal the live pose exactly"
    assert stale_anchor_err > 0.05, "test fixture should make the stale-anchor alternative clearly wrong"
    print("[ramp-origin] CONFIRMED: no bilateral coupling pull exists during GRAVITY_RAMP_UP -- "
          "Part 2's critical invariant ('bilateral position error must NOT generate spring demand "
          "during the gravity-only phase') holds structurally.")

    # Sweep the ramp, recording published-target steps and full distance.
    prev_target = None
    max_step = 0.0
    total_path = 0.0
    while align.state != ACTIVE:
        clock.advance(0.01)
        align.update_master_pose(master_held, t=clock.t)
        align.update_slave_pose(slave_sagged, t=clock.t)  # slave held fixed for this test's purpose
        # Both poses held exactly fixed here -> zero velocity, so
        # GRAVITY_SETTLE's 2026-09-09 velocity-gated quiet-hold is satisfied
        # promptly (this test models a fake-clock instantaneous plant, not
        # the real settle transient -- that is covered separately by
        # investigate_gravity_settle_timing.py's live-sim measurement).
        align.update_master_velocity([0.0, 0.0, 0.0])
        align.update_slave_velocity([0.0, 0.0, 0.0])
        align.tick(_mapper)
        c3_full, _ = compute_c3_target(master_held, align.left_anchor, align.right_anchor,
                                        IDENTITY_AXIS_MAP, AXIS_SCALE, DEADBAND_M, MAX_OFFSET, "axis",
                                        absolute_axes=ALL_ABSOLUTE, absolute_trim=ZERO_TRIM)
        published = align.blend_target(c3_full)
        if prev_target is not None:
            step = norm([published[i] - prev_target[i] for i in range(3)])
            max_step = max(max_step, step)
            total_path += step
        prev_target = published

    final_err = norm([prev_target[i] - master_held[i] for i in range(3)])
    print(f"[ramp-sweep] max single-tick target step = {max_step*1000:.3f}mm (small, matches real trial's 1-2mm)")
    print(f"[ramp-sweep] total target path over the ramp = {total_path*1000:.1f}mm (large -- matches real "
          f"trial's 117mm measured sweep)")
    print(f"[ramp-sweep] final target reaches master_held to within {final_err*1000:.6f}mm")
    assert max_step < 0.01, "per-tick target step must stay small even for a large overall sweep"
    assert total_path > 0.09, "cumulative sweep should be on the order of the real ~117mm case"
    assert final_err < 1e-6
    print("[ramp] CONCLUSION: small per-tick steps + large cumulative sweep can coexist by construction -- "
          "this is not a contradiction, and the ramp-origin capture is proven fresh, not stale.\n")


def test_disarm_sample_hold_transient_matches_measured_order_of_magnitude():
    """Models the real trial's own measured post-disarm slave motion: the
    live pose updates continuously (~200Hz, matching cartesian_controller_
    telemetry's measured effective_hz) while the bridge only republishes a
    fresh target once per RATE_HZ=100 tick (10ms) -- so between bridge ticks,
    a continuously-moving (sagging) device is tracked by a target that is
    always up to one tick (10ms) stale relative to its current position.
    Quantifies the resulting position-error and compares to the real
    peak-order-of-magnitude measured 0.195-0.203N."""
    RATE_HZ = 100.0
    BRIDGE_TICK_S = 1.0 / RATE_HZ
    CONTROLLER_TICK_S = 0.005  # ~200Hz, matches the real trial's measured effective_hz on right_pose/right_telem
    STIFFNESS_N_PER_M = 35.0  # slave's real stiffness_n_per_m (run_slave_right.sh default, confirmed via telemetry)

    # Real measured slave sag velocity from right_pose.csv, steepest observed
    # segment (t=38.97ms -> t=94.09ms, dy=-0.011968m over 0.05512s): this is
    # what "unpowered under gravity, no longer compensated" actually looked
    # like on the real hardware, not an assumed number.
    measured_velocity_mps = 0.011968 / 0.05512

    n_bridge_ticks = 15
    live_pose_y = 0.0
    published_target_y = 0.0
    t = 0.0
    max_position_error = 0.0
    controller_ticks_per_bridge_tick = int(round(BRIDGE_TICK_S / CONTROLLER_TICK_S))
    for _ in range(n_bridge_ticks):
        # Bridge tick: republish target = live pose AT THIS INSTANT (the
        # actual blend_target() READY-branch behavior, confirmed correct
        # in test_unarmed_free_motion.py) -- but this snapshot immediately
        # goes stale as the device keeps moving until the NEXT bridge tick.
        published_target_y = live_pose_y
        for _ in range(controller_ticks_per_bridge_tick):
            live_pose_y -= measured_velocity_mps * CONTROLLER_TICK_S  # continues sagging
            position_error = published_target_y - live_pose_y
            max_position_error = max(max_position_error, abs(position_error))
        t += BRIDGE_TICK_S

    predicted_spring_force_n = STIFFNESS_N_PER_M * max_position_error
    print(f"[disarm-model] measured slave sag velocity = {measured_velocity_mps*1000:.1f} mm/s "
          f"(from the real trial's right_pose.csv)")
    print(f"[disarm-model] worst-case sample-hold position error = {max_position_error*1000:.3f}mm")
    print(f"[disarm-model] predicted spring force = K*error = {predicted_spring_force_n:.4f}N")
    print(f"[disarm-model] real trial measured peak post-disarm applied_force ~= 0.195-0.203N")
    assert predicted_spring_force_n > 0.02, (
        "the sample-hold mechanism alone should already predict a non-negligible force, "
        "same order of magnitude as measured, not zero")
    # Order-of-magnitude check, not exact match -- real behavior also has
    # damping and a second-order (underdamped) response the model above
    # doesn't capture, so this bounds it within roughly 5x either direction.
    assert 0.2 * 0.195 < predicted_spring_force_n < 5.0 * 0.203, (
        f"predicted {predicted_spring_force_n:.4f}N should be within an order of magnitude "
        f"of the measured ~0.2N transient")
    print("[disarm] CONCLUSION: a real, continuously-sagging device tracked by a target that "
          "refreshes at half the controller's own rate produces a bounded but non-zero transient "
          "force of the correct order of magnitude -- SCHEDULING/UPDATE_RATE_EFFECT + PHYSICAL_"
          "RESPONSE, not a stale-target code bug (the target itself is always the CORRECT live "
          "pose as of its last publish, per test_unarmed_free_motion.py).\n")


if __name__ == "__main__":
    test_ramp_origin_fresh_with_real_trial_scale_separation()
    test_disarm_sample_hold_transient_matches_measured_order_of_magnitude()
    print("ALL TESTS PASSED")
