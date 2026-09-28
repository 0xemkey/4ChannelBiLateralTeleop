#!/usr/bin/env python3
"""Direct left-master to right-slave teleoperation bridge.

Maps left Touch pose deltas onto right touch_slave target poses. The right node
renders a spring/damper toward that target, so the right arm follows the left.

Differences from the older scripts/bridge_touch_left_to_right_slave.py:

  clutching        hold the clutch button to reposition your hand without
                   moving the slave; the anchor is retaken on release
  axis map         one --axis-map string instead of separate invert flags,
                   matching the convention used elsewhere in this repo
  rate limit       caps how fast the target may travel, so a dropped pose
                   message cannot turn into a jump
  keep-alive       target and enable are republished every tick, because the
                   right node zeroes force on commands older than its
                   stale_timeout_s (0.05 s by default)
  telemetry        optional tracking-error readout, which is the only usable
                   proxy for external load on this hardware
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped, WrenchStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Int32

from startup_alignment import (
    ACTIVE,
    COUPLING_RAMP_DOWN,
    COUPLING_RAMP_UP,
    GRAVITY_RAMP_DOWN,
    GRAVITY_RAMP_UP,
    GRAVITY_SETTLE,
    AlignmentConfig,
    TeleopStartupAlignment,
)
from coupling_force_budget import (
    AdaptiveCouplingRamp, ArmState, ForceBudgetConfig, CouplingRampStalled, _norm,
)
from touch_haptics.msg import BridgeDiagnostics, CartesianControllerTelemetry, BilateralStatus

AXIS_INDEX = {"x": 0, "y": 1, "z": 2}


def _bool_arg(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_triple(spec: str, fallback: float, name: str) -> list[float]:
    """Parse '1.0,0.6,1.0' into three floats. A single value applies to all axes.

    Lets one axis be scaled or limited independently, which matters because the
    Omni's workspace is not a cube: it is appreciably shorter vertically than it
    is wide, so a 1:1 map that fits in X will run out of travel in Y.
    """
    text = str(spec or "").strip()
    if not text:
        return [fallback] * 3
    parts = [p.strip() for p in text.split(",") if p.strip()]
    try:
        values = [float(p) for p in parts]
    except ValueError as exc:
        raise ValueError(f"invalid --{name}={spec!r}: not a number") from exc
    if len(values) == 1:
        return values * 3
    if len(values) != 3:
        raise ValueError(f"invalid --{name}={spec!r}: give one value or three")
    return values


def parse_axis_map(spec: str) -> list[tuple[int, float]]:
    """Parse a map like '-x,z,y' into [(source_index, sign), ...] for slave xyz."""
    parsed: list[tuple[int, float]] = []
    for token in [part.strip().lower() for part in str(spec).split(",") if part.strip()]:
        sign = 1.0
        if token.startswith("+"):
            token = token[1:]
        elif token.startswith("-"):
            sign = -1.0
            token = token[1:]
        if token not in AXIS_INDEX:
            raise ValueError(f"invalid --axis-map={spec!r}: bad axis {token!r}")
        parsed.append((AXIS_INDEX[token], sign))
    if len(parsed) != 3:
        raise ValueError(f"invalid --axis-map={spec!r}: expected exactly three axes")
    seen = [src for src, _ in parsed]
    if sorted(seen) != [0, 1, 2]:
        raise ValueError(f"invalid --axis-map={spec!r}: each of x,y,z must appear exactly once")
    return parsed


def _params_match(telem, stiffness_n_per_m: float, damping_n_s_per_m: float,
                  max_force_n: float, max_position_error_m: float,
                  rel_tol: float = 1e-6, abs_tol: float = 1e-9) -> bool:
    """True when the bridge's DECLARED planner parameters agree with what the
    controller's own telemetry says it is actually running (2026-09-09,
    Part 4). Strict by design: these are configuration constants, not
    measured signals, so they should agree exactly -- any disagreement means
    the launch config and the running rig genuinely differ."""
    def close(a: float, b: float) -> bool:
        return math.isclose(a, b, rel_tol=rel_tol, abs_tol=abs_tol)
    return (close(telem.stiffness_n_per_m, stiffness_n_per_m)
            and close(telem.damping_n_s_per_m, damping_n_s_per_m)
            and close(telem.max_force_n, max_force_n)
            and close(telem.max_position_error_m, max_position_error_m))


def deadman_gate_state(require: bool, latched: bool, buttons_t, now: float,
                       stale_s: float, buttons_value: int, mask: int):
    """2026-09-10 Part C: pure deadman-gate decision, no ROS -- returns
    (held: bool, new_latched: bool, reason: Optional[str]).

    require=False -> (True, False, None) unconditionally: the gate is
    inactive, matching every existing caller's unchanged behavior.

    latched=True (a prior stale event, not yet cleared by an explicit
    re-arm) -> stays (False, True, None): once latched, ONLY the caller
    clearing `latched` externally (on a successful arm_teleop re-arm or
    reset_alignment) can lift it -- this function never self-clears it,
    by design ("no automatic re-arm after telemetry recovers").

    buttons_t is None or older than stale_s -> (False, True, reason):
    stale/missing telemetry disables AND latches.

    Otherwise -> (bool(buttons_value & mask), False, None): an ordinary
    release is NOT latched -- re-press resumes immediately, matching a
    real deadman switch's normal use (only a telemetry/connection loss is
    treated as severe enough to require a deliberate re-arm)."""
    if not require:
        return True, False, None
    if latched:
        return False, True, None
    if buttons_t is None:
        return False, True, "no button message ever received"
    age = now - buttons_t
    if age > stale_s:
        return False, True, f"button telemetry {age:.2f}s old (> {stale_s}s)"
    return bool(buttons_value & mask), False, None


def ramped_gravity_comp_n(configured_n: float, ramp_alpha: float) -> float:
    """Pure function (2026-09-09, arm/disarm force-contract task): the
    gravity-compensation bias must be zero while unarmed and reach exactly
    configured_n once ACTIVE, using the same ramp_alpha that already gates
    the C2/C3 position targets -- so a bridge configured for a different
    SLAVE_GRAVITY_COMP_N is not already physically distinguishable from
    another configuration before either one is armed."""
    return configured_n * ramp_alpha


def apply_axis_map(amap: list[tuple[int, float]], v: list[float]) -> list[float]:
    """master delta -> slave delta."""
    return [sign * v[src] for src, sign in amap]


def apply_axis_map_inverse(amap: list[tuple[int, float]], v: list[float]) -> list[float]:
    """slave-frame vector -> master frame.

    The forward map is slave[i] = sign_i * master[src_i]. Signs are +/-1, so the
    inverse is master[src_i] = sign_i * slave[i]. Needed to push the operator's
    hand back along the axis they are actually driving.
    """
    out = [0.0, 0.0, 0.0]
    for i, (src, sign) in enumerate(amap):
        out[src] = sign * v[i]
    return out


def compute_c2_target(slave_pos: list[float], right_anchor: list[float],
                       left_anchor: list[float], axis_map: list[tuple[int, float]],
                       axis_scale: list[float], left_max_offset_m: float,
                       absolute_axes: Optional[set[int]] = None,
                       absolute_trim: Optional[list[float]] = None) -> tuple[list[float], bool]:
    """C2 (slave position -> master target). Pure function, extracted from
    _update_c2() (2026-09-09, root-cause investigation of the READY->ACTIVE
    target discontinuity) so it is unit-testable without a live rclpy Node,
    and so its absolute-axes handling can be kept in sync with _tick()'s C3
    path by construction instead of by convention.

    BUG FIXED HERE (see docs/... root-cause note): the original _update_c2()
    had NO absolute_axes handling at all -- it unconditionally used the
    anchor-relative formula `left_anchor + (slave_pos - right_anchor)` even
    when the C3 direction (in _tick()) was overriding those same axes to the
    ABSOLUTE formula `target = master_pos`. Whenever left_anchor and
    right_anchor differ (captured at bridge startup, from whatever pose the
    two devices happened to be resting at -- NOT necessarily matched), this
    mismatch alone becomes a constant, uncancelled offset in C2's target,
    with NOTHING to do with the operator's actual reference-pose match at
    arm time. Measured on real hardware: a 91mm anchor mismatch produced a
    matching ~91mm master target error, pinning the master's position-error
    clamp at 100% for an entire 19s trial (session
    20260909T100330Z_dob_observe, trial h2abs_gravcomp_B2_070).

    Returns (left_target, master_travel_limit_flag).
    """
    if absolute_axes is None:
        absolute_axes = set()
    if absolute_trim is None:
        absolute_trim = [0.0, 0.0, 0.0]
    slave_disp = [slave_pos[i] - right_anchor[i] for i in range(3)]
    unscaled = [
        slave_disp[i] / axis_scale[i] if abs(axis_scale[i]) > 1e-9 else 0.0
        for i in range(3)
    ]
    master_disp = apply_axis_map_inverse(axis_map, unscaled)
    master_travel_limit_flag = False
    limit = float(left_max_offset_m)
    if limit > 0.0:
        for i in range(3):
            if i in absolute_axes:
                continue  # absolute axes are not anchor-offset-limited; see below
            if master_disp[i] > limit:
                master_disp[i] = limit
                master_travel_limit_flag = True
            elif master_disp[i] < -limit:
                master_disp[i] = -limit
                master_travel_limit_flag = True
    left_target = [left_anchor[i] + master_disp[i] for i in range(3)]
    # Absolute axes ignore the anchor entirely and command the slave's own
    # raw coordinate directly -- the exact mirror of _tick()'s C3 override
    # (bridge_master_slave.py's existing absolute_axes block), so C2 and C3
    # agree on which frame each axis lives in instead of silently disagreeing.
    for i in absolute_axes:
        left_target[i] = slave_pos[i] * axis_scale[i] + absolute_trim[i]
    return left_target, master_travel_limit_flag


def compute_c3_target(master_pos: list[float], left_anchor: list[float], right_anchor: list[float],
                       axis_map: list[tuple[int, float]], axis_scale: list[float],
                       deadband_m: float, max_offset: list[float], max_offset_mode: str,
                       absolute_axes: Optional[list[int]] = None,
                       absolute_trim: Optional[list[float]] = None) -> tuple[list[float], bool]:
    """C3 (master position -> slave target). Pure function, extracted from
    _tick() (2026-09-09, Phase 2 of the C2/C3 frame-mixing root-cause
    investigation) alongside compute_c2_target(), purely so the two can be
    unit-tested for symmetry -- this is a refactor for testability, the
    actual formula (including the absolute_axes override) is unchanged from
    the pre-existing _tick() code.

    Returns (target, slave_travel_limit_flag).
    """
    if absolute_axes is None:
        absolute_axes = []
    if absolute_trim is None:
        absolute_trim = [0.0, 0.0, 0.0]
    delta = [master_pos[i] - left_anchor[i] for i in range(3)]
    for i in range(3):
        if abs(delta[i]) < deadband_m:
            delta[i] = 0.0
    mapped = apply_axis_map(axis_map, delta)
    mapped = [mapped[i] * axis_scale[i] for i in range(3)]
    slave_travel_limit_flag = False
    if str(max_offset_mode) == "norm":
        norm_val = math.sqrt(sum(v * v for v in mapped))
        limit_norm = max_offset[0] if max_offset else 0.0
        if limit_norm > 0.0 and norm_val > limit_norm:
            shrink = limit_norm / norm_val
            mapped = [v * shrink for v in mapped]
            slave_travel_limit_flag = True
    else:
        for i in range(3):
            limit = max_offset[i] if i < len(max_offset) else 0.0
            if limit > 0.0:
                if mapped[i] > limit:
                    mapped[i] = limit
                    slave_travel_limit_flag = True
                elif mapped[i] < -limit:
                    mapped[i] = -limit
                    slave_travel_limit_flag = True
    target = [right_anchor[i] + mapped[i] for i in range(3)]
    if absolute_axes:
        abs_master = apply_axis_map(axis_map, master_pos)
        for i in absolute_axes:
            target[i] = abs_master[i] * axis_scale[i] + absolute_trim[i]
    return target, slave_travel_limit_flag


class MasterSlaveBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("touch_master_slave_bridge")
        self.args = args
        self.axis_map = parse_axis_map(args.axis_map)
        # Both are in SLAVE axis order, applied after the axis map, because the
        # thing being fitted is the slave's workspace.
        self.axis_scale = parse_triple(args.axis_scale, args.scale, "axis-scale")
        self.max_offset = parse_triple(args.max_offset_xyz, args.max_offset_m, "max-offset-xyz")
        self.absolute_axes = sorted({
            AXIS_INDEX[c] for c in str(args.absolute_axes or "").strip().lower()
            if c in AXIS_INDEX
        })
        # Per-axis trim (2026-09-04, Phase 3 fine-tuning): y and z showed a
        # roughly CONSTANT (not motion-dependent) gap between master and
        # slave in a Phase 2 capture -- ~0.010-0.013m on y, ~0.025-0.035m on
        # z, matching teleop_calibration.json's own right_minus_mapped_left
        # diagnostic almost exactly. That value was previously recorded but
        # never applied (see _write_calibration's "for diagnostics" note).
        # A single shared --absolute-trim-m cannot correct two axes with two
        # different offsets at once, so this adds a per-axis version of the
        # same --max-offset-xyz pattern already used elsewhere in this file.
        self.absolute_trim = parse_triple(
            args.absolute_trim_xyz, args.absolute_trim_m, "absolute-trim-xyz")

        # Startup/alignment state machine (2026-09-08, Phase 1 of the
        # diagnose-and-repair task) -- OWNS anchor capture now. See
        # startup_alignment.py's module docstring for why this does not
        # change the anchor-relative control law itself, only how/when the
        # anchors get captured and how quickly authority ramps in.
        self.alignment = TeleopStartupAlignment(AlignmentConfig(
            pose_stale_s=args.alignment_pose_stale_s,
            hold_still_motion_m=args.alignment_hold_still_motion_m,
            hold_still_duration_s=args.alignment_hold_still_duration_s,
            max_alignment_error_m=args.alignment_max_error_m,
            ramp_duration_s=args.alignment_ramp_s,
        ))
        # 2026-09-10 (real-hardware Y-axis validation, Part 1): optional
        # gravity-settle-only/hold mode -- default False preserves the
        # normal READY->...->ACTIVE production behavior unchanged. When
        # True, the state machine remains in GRAVITY_SETTLE indefinitely
        # once the quiet-hold gate passes (never auto-advances into
        # COUPLING_RAMP_UP), exiting only via explicit disarm -- see
        # startup_alignment.py's GRAVITY_SETTLE tick() branch.
        self.alignment.gravity_settle_hold_mode = args.gravity_settle_hold_mode
        # Bilateral force-budget-aware coupling ramp (2026-09-09,
        # production-integration task) -- see coupling_force_budget.py's
        # module docstring.
        #
        # 2026-09-09 Part 1 (validation-infrastructure follow-up) origin-mode
        # fix: this used to be origin_mode="snapshot", but
        # startup_alignment.py's blend_target()/blend_left_target() -- the
        # functions that compute what is ACTUALLY published to the real
        # controller -- always use the continuously-live pose as origin,
        # unconditionally, with no knowledge of this module's snapshot. That
        # meant the planner was certifying the safety of a DIFFERENT
        # trajectory than the one actually sent to the hardware -- a real
        # planner/publish mismatch, not just a diagnostics-labeling issue.
        # SNAPSHOT_ORIGIN was originally chosen because MOVING_LIVE_ORIGIN
        # deadlocked (see coupling_force_budget.py's test history) -- but
        # that deadlock was reproduced-and-confirmed-GONE against today's
        # code (a physics-integrated convergence check: both modes now reach
        # alpha=1.0 with no stall). Root cause of the old deadlock was the
        # missing per-axis position-error clamp in predict_force_vector(),
        # added later in this same session (the controller-predictor audit
        # correction) -- not something inherent to MOVING_LIVE_ORIGIN.
        # origin_mode="moving" now matches blend_target()/blend_left_target()
        # exactly, so the planner vets the actual published trajectory.
        self._coupling_ramp = AdaptiveCouplingRamp(
            ForceBudgetConfig(
                max_force_n=args.slave_max_force_n,  # per-arm max_force_n is passed per-call via ArmState;
                force_reserve_n=args.force_budget_reserve_n,  # this ForceBudgetConfig field is otherwise unused
                nominal_duration_s=args.alignment_ramp_s,
                stall_timeout_s=args.force_budget_stall_timeout_s,
            ), origin_mode="moving")
        self._prev_coupling_state = None
        self._telemetry_stale_since = None
        # Part 4 diagnostics: whether the DECLARED planner params agree with
        # the controller's own telemetry. True until proven otherwise (they
        # are only evaluated once COUPLING_RAMP_UP has live telemetry).
        self._planner_param_match_left = True
        self._planner_param_match_right = True
        self._calibration_written = False
        # Populated by _update_c2() when C2 is on; stay at these safe
        # defaults (zero delta, no limit hit) when it never runs.
        self._c2_mapped_slave_relative = [0.0, 0.0, 0.0]
        self._master_travel_limit_flag = False
        self.bridge_diag_pub = self.create_publisher(
            BridgeDiagnostics, "/bridge_diagnostics", 10)
        self.declare_parameter("arm_teleop", False)
        self.declare_parameter("reset_alignment", False)
        self.add_on_set_parameters_callback(self._on_param)

        self.latest_left: Optional[list[float]] = None
        self.latest_right: Optional[list[float]] = None
        self.left_buttons = 0
        # 2026-09-10 (Part C, real-hardware safety completion): bridge-level
        # master deadman gate. Off by default (require_master_deadman=False)
        # -- existing behavior (enable always True, the "keep-alive" this
        # file's own _tick() comment describes) is UNCHANGED unless
        # explicitly opted in, same convention as gravity_settle_hold_mode.
        self._left_buttons_t: Optional[float] = None
        # Latched (not just transient) disable -- set when button TELEMETRY
        # itself goes stale (the master node died/disconnected, a more
        # severe failure than an ordinary release), cleared ONLY by an
        # explicit re-arm (arm_teleop false->true, with fresh telemetry) or
        # reset_alignment. An ordinary button RELEASE (telemetry still
        # fresh, bit just not set) is NOT latched -- re-pressing resumes
        # immediately, matching a real deadman switch's normal UX. Losing
        # the CONNECTION is what requires a deliberate re-arm, not simply
        # letting go.
        self._deadman_stale_latched = False

        # 2026-09-10 (item 2/4): C1/C4 safety-state COMPOSITION, not a
        # second independent enable authority -- see COMMAND_AUTHORITY_ADR.md.
        # Only subscribed when this bridge is told a C1/C4 relay is expected
        # (--c1-enable/--c4-enable), so a normal C2/C3-only launch is
        # completely unaffected (no subscription, no dependency).
        self._bilateral_status = None
        self._bilateral_status_t: Optional[float] = None
        if args.c1_enable or args.c4_enable:
            self.create_subscription(
                BilateralStatus, args.bilateral_status_topic, self._bilateral_status_cb, 10)

        self.clutch_active = False
        self.last_target: Optional[list[float]] = None
        self.tracking_error: Optional[list[float]] = None
        self.last_report = 0.0
        self.start_time = time.time()
        self.last_wait_notice = 0.0
        self.ready_announced = False
        self.force_sub_warned = False

        # Force reflection state.
        self.reflect_engaged = False
        self.reflect_force = [0.0, 0.0, 0.0]
        self.reflect_excess = 0.0
        self.prev_target: Optional[list[float]] = None
        self.target_speed = 0.0
        self.master_enable_sent = False
        self.prev_left: Optional[list[float]] = None
        self.master_vel = [0.0, 0.0, 0.0]
        self.prev_right: Optional[list[float]] = None
        self.slave_speed = 0.0
        self.stalled = False
        self.limit_notice = [0.0, 0.0, 0.0]
        # Furthest each axis was asked to go, and furthest it actually got.
        # A gap between them is the slave falling short of your hand.
        self.peak_cmd = [0.0, 0.0, 0.0]
        self.peak_act = [0.0, 0.0, 0.0]

        self.create_subscription(PoseStamped, args.left_pose_topic, self._left_cb, 10)
        self.create_subscription(PoseStamped, args.right_pose_topic, self._right_cb, 10)
        # Diagnostics only (q_left_anchor/q_right_anchor) -- not consumed by
        # the control law, which is purely Cartesian. Optional: empty topic
        # string disables it entirely, matching this file's existing
        # "empty/0 disables the feature" convention (slave_gravity_comp_n,
        # left_gravity_comp_n).
        if args.left_joint_angles_topic:
            self.create_subscription(
                Vector3Stamped, args.left_joint_angles_topic,
                lambda m: self.alignment.update_master_q([m.vector.x, m.vector.y, m.vector.z]), 10)
        if args.right_joint_angles_topic:
            self.create_subscription(
                Vector3Stamped, args.right_joint_angles_topic,
                lambda m: self.alignment.update_slave_q([m.vector.x, m.vector.y, m.vector.z]), 10)
        # Force-budget planner input (2026-09-09, production-integration
        # task): the controller's OWN velocity/position/force telemetry --
        # NOT a bridge-side finite-difference estimate. touch_slave.cpp
        # computes damping_force from HD_CURRENT_VELOCITY (the device
        # driver's native readout), which a bridge-side pose-diff estimate
        # cannot reproduce exactly (see coupling_force_budget.py's module
        # docstring / the controller-predictor audit that motivated this).
        # Subscribing to the controller's own already-published
        # velocity_mps closes that gap to one message-passing tick instead
        # of leaving it unquantified.
        self.left_telemetry = None
        self.left_telemetry_t = None
        self.right_telemetry = None
        self.right_telemetry_t = None
        self.create_subscription(
            CartesianControllerTelemetry, args.left_telemetry_topic,
            self._left_telemetry_cb, 10)
        self.create_subscription(
            CartesianControllerTelemetry, args.right_telemetry_topic,
            self._right_telemetry_cb, 10)

        self.create_subscription(Int32, args.left_buttons_topic, self._buttons_cb, 10)
        if args.report or args.reflect_enable:
            self.create_subscription(
                Vector3Stamped, args.tracking_error_topic, self._tracking_cb, 10
            )

        self.target_pub = self.create_publisher(PoseStamped, args.right_target_topic, 10)
        self.enable_pub = self.create_publisher(Bool, args.right_enable_topic, 10)

        # C2 (slave position -> master): pulls the master's own spring toward
        # wherever the slave actually is, the reverse direction of C3. Off by
        # default -- this changes what force the operator's hand feels, so it
        # needs an explicit opt-in rather than coming along for free with the
        # rest of the bridge.
        self.left_target_pub = None
        self.last_left_target: Optional[list[float]] = None
        if args.enable_c2:
            self.left_target_pub = self.create_publisher(PoseStamped, args.left_target_topic, 10)

        # 2026-09-10 (command-authority ADR, "bridge as single final enable
        # authority"): the master's /enable publisher is now UNCONDITIONAL
        # and SINGULAR -- previously this bridge created UP TO THREE separate
        # publisher objects on the same default topic (/touch_left/enable):
        # left_enable_pub (C2-gated), master_enable_pub (reflect-gated), and
        # (in a SEPARATE process) bilateral_4ch.py's own left_enable_pub
        # (needed because neither of the above two covers the C1/C4-only
        # case -- see bilateral_4ch.py's own 2026-09-07 bug-fix comment: C1
        # silently did nothing without it). Consolidated into ONE publisher,
        # published exactly once per tick in _tick(), gated by the master
        # deadman -- see _master_deadman_held(). This closes the ORIGINAL
        # 2026-09-07 precondition gap (master enable now flows regardless of
        # C2/reflect/C1/C4 being on) WITHOUT bilateral_4ch.py needing its own
        # competing publisher -- see bilateral_4ch.py's matching
        # --publish-enable flag (now defaults False).
        self.left_enable_pub = self.create_publisher(Bool, args.left_enable_topic, 10)

        self.master_force_pub = None
        if args.reflect_enable:
            self.master_force_pub = self.create_publisher(WrenchStamped, args.master_force_topic, 10)

        # Gravity compensation for the slave. The Omni is not gravity
        # compensated, so on the vertical axis the tracking spring has to lift
        # the arm's own weight before it moves at all -- which is why that axis
        # runs out of range first. A constant bias on force_cmd is added to the
        # spring inside the node, freeing the spring to do the tracking.
        self.slave_force_pub = None
        if args.slave_gravity_comp_n != 0.0:
            self.slave_force_pub = self.create_publisher(
                WrenchStamped, args.slave_force_topic, 10)

        # Symmetric fix for C2, added 2026-09-03: the master has the exact
        # same unweighted-vertical-axis problem as the slave (confirmed
        # live -- C2's commanded target updated correctly on all 3 axes,
        # but the master's own spring could not visibly follow on Y even
        # after raising LEFT_STIFFNESS_N_PER_M from 15 to 28 N/m; Z and X
        # were fine at the same stiffness, isolating this to gravity, not
        # general spring weakness). Same mechanism as slave_force_pub above.
        self.left_force_pub = None
        if args.left_gravity_comp_n != 0.0:
            self.left_force_pub = self.create_publisher(
                WrenchStamped, args.left_force_topic, 10)

        self.timer = self.create_timer(1.0 / max(args.rate_hz, 1.0), self._tick)

        self.get_logger().warn(
            f"bridge started: {args.left_pose_topic} -> {args.right_target_topic} "
            f"axis_map={args.axis_map!r} axis_scale={self.axis_scale} "
            f"max_offset_xyz={[round(v, 4) for v in self.max_offset]} m "
            f"max_speed={args.max_speed_mps} m/s clutch_mask={args.clutch_button_mask}"
        )
        if args.slave_gravity_comp_n != 0.0:
            self.get_logger().warn(
                f"slave gravity compensation {args.slave_gravity_comp_n:+.3f} N on "
                f"{'xyz'[args.slave_gravity_axis]}, helping the vertical axis carry its own weight"
            )
        if args.left_gravity_comp_n != 0.0:
            self.get_logger().warn(
                f"master (left) gravity compensation {args.left_gravity_comp_n:+.3f} N on "
                f"{'xyz'[args.left_gravity_axis]}, helping C2's vertical tracking carry the "
                "master's own weight"
            )
        if args.enable_c2:
            self.get_logger().warn(
                f"C2 ON: slave position -> {args.left_target_topic}, "
                f"left_max_offset={args.left_max_offset_m} m -- the master will now feel a "
                "spring pulling toward the slave's actual position, not just render whatever "
                "force_cmd/reflection publishes"
            )
        if args.reflect_enable:
            self.get_logger().warn(
                f"force reflection ON: gain={args.reflect_gain} N/m "
                f"deadband={args.reflect_deadband_m} m release_ratio={args.reflect_release_ratio} "
                f"max={args.reflect_max_n} N alpha={args.reflect_alpha} "
                f"velocity_comp={args.reflect_velocity_comp} s -> {args.master_force_topic}"
            )
            # The master node zeroes force on commands older than its
            # stale_timeout_s (0.05 s by default). Publishing below ~40 Hz makes
            # the reflected force stutter instead of holding.
            if args.rate_hz < 40.0:
                self.get_logger().error(
                    f"rate_hz={args.rate_hz} is too low for force reflection; "
                    "the master will drop force between ticks. Use 100 Hz or more."
                )

    # ---- subscriptions -------------------------------------------------

    def _on_param(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "arm_teleop" and p.value:
                # 2026-09-10 Part C: "arming is rejected unless button
                # telemetry is fresh" -- only enforced when the deadman gate
                # is actually active; a caller that never opted in sees no
                # change. Checks TELEMETRY FRESHNESS only (not whether the
                # button is currently held) -- an operator is not expected
                # to already be gripping the deadman before arming.
                if self.args.require_master_deadman:
                    now = time.time()
                    fresh = (self._left_buttons_t is not None
                             and now - self._left_buttons_t <= self.args.master_deadman_stale_s)
                    if not fresh:
                        self.get_logger().error(
                            "arm_teleop REJECTED: master deadman button telemetry is not fresh "
                            f"({'never received' if self._left_buttons_t is None else f'{now - self._left_buttons_t:.2f}s old'}) "
                            "-- cannot arm without a live button feed"
                        )
                        return SetParametersResult(
                            successful=False,
                            reason="master deadman button telemetry not fresh")
                    # Fresh telemetry confirmed -- this IS the explicit
                    # re-arm that clears a stale-triggered latch.
                    self._deadman_stale_latched = False
                self.alignment.arm()
                self.get_logger().warn("arm_teleop: alignment armed, ramping if READY")
            if p.name == "arm_teleop" and not p.value:
                self.alignment.disarm()
                self.get_logger().warn(
                    "arm_teleop: alignment disarmed, re-anchored to current pose, "
                    "back to READY (no-op if not RAMPING/ACTIVE)"
                )
            if p.name == "reset_alignment" and p.value:
                self.alignment.reset()
                self._calibration_written = False
                self._deadman_stale_latched = False
                self.get_logger().warn("reset_alignment: alignment state machine reset")
        return SetParametersResult(successful=True)

    def _map_axis(self, pos: list[float]) -> list[float]:
        """The bridge's own axis_map + axis_scale composition, exposed for
        startup_alignment.py's VALIDATE_ALIGNMENT step -- passed in rather
        than duplicated there, so there is exactly one copy of this math."""
        mapped = apply_axis_map(self.axis_map, pos)
        return [mapped[i] * self.axis_scale[i] for i in range(3)]

    def _left_telemetry_cb(self, msg: CartesianControllerTelemetry) -> None:
        self.left_telemetry = msg
        self.left_telemetry_t = time.time()
        # 2026-09-09 GRAVITY_SETTLE revision: feed the controller's own
        # velocity_mps (the same field the force-budget predictor already
        # uses) into the alignment state machine so GRAVITY_SETTLE can gate
        # on actual mechanical quiescence instead of a fixed clock -- see
        # startup_alignment.py's module docstring / GRAVITY_SETTLE comments.
        self.alignment.update_master_velocity(list(msg.velocity_mps))

    def _right_telemetry_cb(self, msg: CartesianControllerTelemetry) -> None:
        self.right_telemetry = msg
        self.right_telemetry_t = time.time()
        self.alignment.update_slave_velocity(list(msg.velocity_mps))

    def _left_cb(self, msg: PoseStamped) -> None:
        self.latest_left = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # time.time(), not the ROS clock -- matches startup_alignment.py's
        # own now_fn=time.time default (used for staleness/hold-window
        # timing) and every other timestamp already used elsewhere in this
        # file (self.start_time, self.last_wait_notice, ...). Mixing clock
        # sources here would silently break staleness detection if this
        # node ever ran under simulated time.
        self.alignment.update_master_pose(self.latest_left, t=time.time())

    def _right_cb(self, msg: PoseStamped) -> None:
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        # Slave speed, smoothed. This is what separates "lagging behind" from
        # "physically stopped", which is the difference between free motion and
        # an obstacle.
        if self.prev_right is not None:
            step = math.sqrt(sum((pos[i] - self.prev_right[i]) ** 2 for i in range(3)))
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            self.slave_speed = (1.0 - sa) * self.slave_speed + sa * (step * 200.0)
        self.prev_right = list(pos)
        self.latest_right = pos
        self.alignment.update_slave_pose(pos, t=time.time())

    def _write_calibration(self) -> None:
        """Record the anchor pair to a shared file, so anything launched after
        the bridge (a sim twin, a diagnostic script) can read the exact
        starting reference the bridge is using, instead of re-sampling the
        live topics itself and risking a stale/racy read.

        Real hardware is never moved by this -- it is a log of wherever each
        device already was when its anchor was taken, written for diagnostics
        and for seeding sim twins (see run_slave_right_sim.sh /
        run_master_left_sim.sh). Two devices have independent physical
        origins, so there is no absolute pose they should "match"; what this
        adds is a single, well-defined snapshot of the pair, captured at a
        known moment (guaranteed fresh by the master -> bridge -> slave
        launch order), rather than each downstream script sampling the live
        topics again later and possibly getting a different, inconsistent
        reading.
        """
        path = os.path.expanduser(self.args.calibration_file)
        left_anchor = self.alignment.left_anchor
        right_anchor = self.alignment.right_anchor
        record: dict = {
            "timestamp": time.time(),
            "axis_map": self.args.axis_map,
            "axis_scale": self.axis_scale,
            "left_anchor": left_anchor,
            "right_anchor": right_anchor,
            # 2026-09-08: anchors now come from startup_alignment.py's
            # validated hold-still capture, not "whichever pose arrived
            # first" -- see that module's docstring for what
            # right_minus_mapped_left (below) does and does not mean
            # geometrically.
            "q_left_anchor": self.alignment.q_left_anchor,
            "q_right_anchor": self.alignment.q_right_anchor,
            "startup_state": self.alignment.state,
        }
        if left_anchor is not None and right_anchor is not None:
            # Diagnostic (and, since 2026-09-08, an actual FAULT gate --
            # see startup_alignment.py's VALIDATE_ALIGNMENT): what the right
            # anchor "would be" if the two devices shared one origin and the
            # current axis_map/scale were correct. Reuses the SAME value the
            # state machine already computed (self.alignment.initial_
            # alignment_error_xyz/_norm) rather than recomputing it here,
            # so this file can never silently disagree with what actually
            # gated arming.
            mapped_left = self._map_axis(left_anchor)
            record["mapped_left_anchor"] = mapped_left
            record["right_minus_mapped_left"] = self.alignment.initial_alignment_error_xyz or [
                right_anchor[i] - mapped_left[i] for i in range(3)
            ]
            record["initial_alignment_error_norm"] = self.alignment.initial_alignment_error_norm
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(
                dir=os.path.dirname(path) or ".", prefix=".calib_", suffix=".tmp")
            with os.fdopen(fd, "w") as f:
                json.dump(record, f, indent=2)
            os.replace(tmp_path, path)
            self.get_logger().info(f"calibration written to {path}")
        except OSError as exc:
            self.get_logger().warn(f"could not write calibration file {path}: {exc}")

    def _buttons_cb(self, msg: Int32) -> None:
        self.left_buttons = int(msg.data)
        self._left_buttons_t = time.time()
        engaged = bool(self.left_buttons & int(self.args.clutch_button_mask))
        if engaged and not self.clutch_active:
            self.clutch_active = True
            self.get_logger().info("clutch engaged: slave frozen, move your hand freely")
        elif not engaged and self.clutch_active:
            self.clutch_active = False
            # Re-anchor both sides so the slave does not jump on re-engage.
            # Direct mutation of the alignment module's anchors, bypassing
            # its HOLD_STILL/VALIDATE_ALIGNMENT gate on purpose: clutch is a
            # MID-SESSION re-anchor (the operator is already holding the
            # clutch button, which is itself a form of "hold still" intent),
            # not a fresh startup -- unchanged behavior from before this
            # module existed, just pointed at the new anchor storage.
            if self.latest_left is not None:
                self.alignment.left_anchor = list(self.latest_left)
            if self.last_target is not None:
                self.alignment.right_anchor = list(self.last_target)
            self.get_logger().info("clutch released: re-anchored")

    def _bilateral_status_cb(self, msg: BilateralStatus) -> None:
        self._bilateral_status = msg
        self._bilateral_status_t = time.time()

    def _bilateral_relay_safe(self) -> bool:
        """2026-09-10 (item 2/4): True (no dependency) when neither
        --c1-enable nor --c4-enable is set. Otherwise composes
        bilateral_4ch.py's own safety_state (its energy-tank/oscillation-
        trip logic) into THIS bridge's single enable decision -- stale/
        missing status is treated as unsafe (fail closed), matching the
        deadman gate's own convention."""
        if not (self.args.c1_enable or self.args.c4_enable):
            return True
        if self._bilateral_status is None or self._bilateral_status_t is None:
            return False
        if time.time() - self._bilateral_status_t > self.args.bilateral_status_stale_s:
            return False
        return int(self._bilateral_status.safety_state) != 2  # SAFETY_TRIPPED

    def _master_deadman_held(self) -> bool:
        """2026-09-10 Part C: bridge-level master deadman gate -- thin
        wrapper around the pure deadman_gate_state() (see below) so the
        decision logic itself is unit-testable without rclpy. Returns True
        (gate inactive) when --require-master-deadman is not set, so this
        never changes behavior for a caller that never opts in."""
        held, new_latched, reason = deadman_gate_state(
            require=self.args.require_master_deadman,
            latched=self._deadman_stale_latched,
            buttons_t=self._left_buttons_t,
            now=time.time(),
            stale_s=self.args.master_deadman_stale_s,
            buttons_value=self.left_buttons,
            mask=int(self.args.master_deadman_button_mask),
        )
        if new_latched and not self._deadman_stale_latched:
            self.get_logger().error(f"MASTER_DEADMAN_TELEMETRY_STALE: {reason} -- "
                                    "latched disabled, requires explicit re-arm")
        self._deadman_stale_latched = new_latched
        return held

    def _tracking_cb(self, msg: Vector3Stamped) -> None:
        self.tracking_error = [msg.vector.x, msg.vector.y, msg.vector.z]

    # ---- main loop -----------------------------------------------------

    def _tick(self) -> None:
        # Keep-alive: the right node drops force on stale commands.
        # 2026-09-10 Part C: gated by the master deadman when
        # --require-master-deadman is set (returns True unconditionally,
        # i.e. unchanged behavior, otherwise) -- touch_slave.cpp's own
        # command.enabled&&deadman_ok gate (touch_slave.cpp:842) means this
        # single flag already covers ALL force output on the slave,
        # including gravity-comp auxiliary force (force_cmd_n is only
        # summed in inside that same enabled block, touch_slave.cpp:863),
        # not just the position spring.
        enable = Bool()
        # "bridge FAULT/disarm overrides everything" (Part C spec): the
        # position/gravity ramps already zero themselves out during FAULT
        # (both alphas snap to 0, see startup_alignment.py's _fault()), so
        # force stays safe regardless -- but the enable FLAG itself is a
        # second, independent layer, and should say so plainly rather than
        # relying solely on target==live-pose to keep force at zero.
        armed_enable_state = self.alignment.state in (
            GRAVITY_RAMP_UP, GRAVITY_SETTLE, COUPLING_RAMP_UP,
            ACTIVE, COUPLING_RAMP_DOWN, GRAVITY_RAMP_DOWN,
        )
        # READY is pre-arm: C2/C3 targets are still blended to zero there,
        # but C1/C4 relay torque is independent of target blending. Keep both
        # devices disabled until arm_teleop has explicitly entered the armed
        # ramp/active state sequence, so relay torque cannot render force
        # during pre-arm verification.
        enable.data = (armed_enable_state
                      and self._master_deadman_held()
                      and self.alignment.state != "FAULT"
                      and self._bilateral_relay_safe())
        self.enable_pub.publish(enable)
        # 2026-09-10: the MASTER's own /enable, published unconditionally
        # and exactly once here (see left_enable_pub's construction-site
        # comment for the full consolidation history) -- SAME value as the
        # slave's, since the deadman/FAULT gate is not per-arm. This is what
        # lets C1 (rendered on the master) work without bilateral_4ch.py
        # needing its own competing publisher, and closes the original
        # 2026-09-07 precondition gap for every C1/C4-only launch, not just
        # C2/reflect ones.
        left_enable = Bool()
        left_enable.data = enable.data
        self.left_enable_pub.publish(left_enable)
        self._publish_gravity_comp()
        self._publish_left_gravity_comp()

        self.alignment.tick(self._map_axis)
        if not self._calibration_written and self.alignment.left_anchor is not None \
                and self.alignment.right_anchor is not None:
            self._calibration_written = True
            self.get_logger().info(
                f"anchors captured: left={self.alignment.left_anchor} "
                f"right={self.alignment.right_anchor} state={self.alignment.state}"
            )
            self._write_calibration()
        if self.alignment.state == "FAULT" and not getattr(self, "_fault_announced", False):
            self._fault_announced = True
            self.get_logger().error(
                f"startup alignment FAULT: {self.alignment.diagnostics().get('fault_reason')} "
                "-- clear with 'ros2 param set /touch_master_slave_bridge reset_alignment true'"
            )
        elif self.alignment.state != "FAULT":
            self._fault_announced = False

        if self.alignment.left_anchor is None or self.alignment.right_anchor is None \
                or self.latest_left is None:
            self._report_waiting()
            return

        self._update_force_budget_coupling()

        if not self.ready_announced:
            self.ready_announced = True
            self.get_logger().info("both devices are publishing, teleoperation live")
            if self.args.reflect_enable and self.count_subscribers(self.args.master_force_topic) == 0:
                self.get_logger().warn(
                    f"nothing subscribed to {self.args.master_force_topic}; the master will not "
                    "render force. Is run_master_left.sh running?"
                )

        if self.clutch_active:
            # Hold the last target so the slave stays put while you reposition.
            # Reflected force is released too: the hand is off task during a
            # clutch, and a live push would fight the repositioning.
            if self.last_target is not None:
                self._publish_target(self.last_target)
            self._decay_and_publish_force()
            if self.args.enable_c2 and self.left_target_pub is not None and self.last_left_target is not None:
                self._publish_left_target(self.last_left_target)
            return

        # Master velocity, smoothed, for the damping term. Same derivative-noise
        # caution as target_speed: this is differentiated encoder data.
        if self.prev_left is not None:
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            for i in range(3):
                raw_v = (self.latest_left[i] - self.prev_left[i]) * self.args.rate_hz
                self.master_vel[i] = (1.0 - sa) * self.master_vel[i] + sa * raw_v
        self.prev_left = list(self.latest_left)

        delta = [self.latest_left[i] - self.alignment.left_anchor[i] for i in range(3)]
        if abs(delta[0]) < self.args.deadband_m:
            delta[0] = 0.0
        if abs(delta[1]) < self.args.deadband_m:
            delta[1] = 0.0
        if abs(delta[2]) < self.args.deadband_m:
            delta[2] = 0.0

        # Diagnostic only (pre-clamp, anchor-relative delta) -- matches the
        # pre-existing mapped_unclamped convention regardless of
        # absolute_axes, same reasoning as C2's _c2_mapped_slave_relative.
        mapped_unclamped = apply_axis_map(self.axis_map, delta)
        mapped_unclamped = [mapped_unclamped[i] * self.axis_scale[i] for i in range(3)]

        target, slave_travel_limit_flag = compute_c3_target(
            self.latest_left, self.alignment.left_anchor, self.alignment.right_anchor,
            self.axis_map, self.axis_scale, self.args.deadband_m, self.max_offset,
            self.args.max_offset_mode, absolute_axes=self.absolute_axes,
            absolute_trim=self.absolute_trim,
        )
        if slave_travel_limit_flag and str(self.args.max_offset_mode) != "norm":
            for i in range(3):
                limit = self.max_offset[i]
                if limit > 0.0 and abs(mapped_unclamped[i]) > limit + 1e-9:
                    self._note_limit(i)

        # Startup ramp, applied at the safest mathematically consistent
        # location (see startup_alignment.py's module docstring): blends
        # the FINAL commanded setpoint from right_anchor (alpha=0, zero
        # relative motion -- the slave holds exactly where its anchor was
        # captured) to the full target above (alpha=1, the pre-existing
        # formula byte-for-byte). Before RAMPING/ACTIVE (i.e. during
        # WAIT_FOR_POSES..READY) ramp_alpha is exactly 0.0, so this
        # continuously commands "stay at your anchor" -- actuator authority
        # stays gated without needing a separate branch that skips
        # publishing entirely (which would let the slave's force go stale
        # and drop to zero instead of holding).
        target = self.alignment.blend_target(target)

        # 2026-09-10 (item 4, channel gating): C3 was architecturally
        # ALWAYS active -- there was no way to reach ACTIVE (coupling_ramp_
        # alpha=1) without C3 engaging, which is exactly what made the
        # previous runbook's "H5 C2-only" not actually C2-only (C3's spring
        # was live the whole time). --c3-enable (default True, preserving
        # existing behavior) lets this be genuinely disabled: when False,
        # the published target is forced to the slave's OWN current live
        # pose (zero relative motion, zero spring demand) regardless of
        # coupling_ramp_alpha -- the ramp/state-machine/force-budget-planner
        # machinery still progresses normally underneath (so C2/deadman/
        # gravity are unaffected), only the PUBLISHED target is overridden.
        if not self.args.c3_enable:
            if self.latest_right is not None:
                target = list(self.latest_right)
            elif self.alignment.right_anchor is not None:
                target = list(self.alignment.right_anchor)

        target = self._rate_limit(target)

        # Commanded target speed, used to excuse the servo lag that free motion
        # always produces. Without this the operator feels drag in empty space.
        if self.prev_target is not None:
            step = math.sqrt(sum((target[i] - self.prev_target[i]) ** 2 for i in range(3)))
            raw_speed = step * self.args.rate_hz
            # Low-pass it. This is a position derivative at rate_hz, so encoder
            # quantisation lands straight in it. Unfiltered, the noise rides
            # through the threshold into the force and buzzes at loop rate --
            # and gets worse as reflect_gain rises.
            sa = min(max(self.args.reflect_speed_alpha, 0.0), 1.0)
            self.target_speed = (1.0 - sa) * self.target_speed + sa * raw_speed
        self.prev_target = list(target)

        for i in range(3):
            self.peak_cmd[i] = max(self.peak_cmd[i], abs(mapped_unclamped[i]))
            if self.latest_right is not None and self.alignment.right_anchor is not None:
                self.peak_act[i] = max(
                    self.peak_act[i], abs(self.latest_right[i] - self.alignment.right_anchor[i]))

        self.last_target = target
        self._publish_target(target)
        self._update_force_reflection(target)
        if self.args.enable_c2 and self.left_target_pub is not None:
            self._update_c2()
        self._publish_bridge_diagnostics(delta, mapped_unclamped, target, slave_travel_limit_flag)
        self._maybe_report(target)

    def _publish_bridge_diagnostics(self, master_relative: list[float],
                                    mapped_master_relative: list[float],
                                    c3_target: list[float],
                                    slave_travel_limit_flag: bool) -> None:
        """See msg/BridgeDiagnostics.msg's own header comment for what each
        field means and why it exists -- added 2026-09-08 for the C2+C3
        position-only diagnostic task. Read-only telemetry; does not feed
        back into any control decision."""
        slave_relative = [0.0, 0.0, 0.0]
        c3_err = [0.0, 0.0, 0.0]
        if self.latest_right is not None and self.alignment.right_anchor is not None:
            slave_relative = [self.latest_right[i] - self.alignment.right_anchor[i] for i in range(3)]
            c3_err = [c3_target[i] - self.latest_right[i] for i in range(3)]
        c2_err = [0.0, 0.0, 0.0]
        if self.args.enable_c2 and self.last_left_target is not None and self.latest_left is not None:
            c2_err = [self.last_left_target[i] - self.latest_left[i] for i in range(3)]

        msg = BridgeDiagnostics()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.master_relative_m = list(master_relative)
        msg.slave_relative_m = slave_relative
        msg.mapped_master_relative_m = list(mapped_master_relative)
        msg.mapped_slave_relative_m = list(self._c2_mapped_slave_relative)
        msg.c2_position_error_m = c2_err
        msg.c3_position_error_m = c3_err
        err_xyz = self.alignment.initial_alignment_error_xyz
        msg.alignment_error_xyz_m = list(err_xyz) if err_xyz is not None else [0.0, 0.0, 0.0]
        msg.alignment_error_norm_m = float(self.alignment.initial_alignment_error_norm or 0.0)
        msg.follow_error_m = math.sqrt(sum(v * v for v in c3_err))
        msg.bridge_state = self.alignment.state
        msg.ramp_alpha = float(self.alignment.ramp_alpha)
        msg.gravity_ramp_alpha = float(self.alignment.gravity_ramp_alpha)
        msg.coupling_ramp_alpha = float(self.alignment.coupling_ramp_alpha)
        msg.c2_enabled = bool(self.args.enable_c2)
        msg.c3_enabled = bool(self.args.c3_enable)
        msg.c1_enabled = bool(self.args.c1_enable)
        msg.c4_enabled = bool(self.args.c4_enable)
        msg.master_travel_limit_flag = bool(self._master_travel_limit_flag)
        msg.slave_travel_limit_flag = bool(slave_travel_limit_flag)
        cr = self._coupling_ramp
        # 2026-09-09 Part 1 audit fix: publish accepted (published-target,
        # safety-relevant) and candidate (last-bisection-trial, explanatory
        # only) margins as separate fields -- see coupling_force_budget.py's
        # AdaptiveCouplingRamp docstring and BridgeDiagnostics.msg's matching
        # comment for why these must never be conflated.
        def _f(attr):
            v = getattr(cr, attr, None)
            return float(v) if v is not None else 0.0
        msg.master_accepted_force_budget_margin_n = _f("last_master_accepted_force_margin_n")
        msg.slave_accepted_force_budget_margin_n = _f("last_slave_accepted_force_margin_n")
        msg.master_accepted_position_error_margin_m = _f("last_master_accepted_position_margin_m")
        msg.slave_accepted_position_error_margin_m = _f("last_slave_accepted_position_margin_m")
        msg.master_candidate_force_budget_margin_n = _f("last_master_candidate_force_margin_n")
        msg.slave_candidate_force_budget_margin_n = _f("last_slave_candidate_force_margin_n")
        msg.master_candidate_position_error_margin_m = _f("last_master_candidate_position_margin_m")
        msg.slave_candidate_position_error_margin_m = _f("last_slave_candidate_position_margin_m")
        msg.coupling_ramp_held = bool(cr.last_held)
        msg.coupling_ramp_stall_time_s = float(cr.stall_elapsed_s(time.time()))
        now = time.time()
        msg.force_budget_telemetry_fresh_left = (
            self.left_telemetry_t is not None
            and now - self.left_telemetry_t <= self.args.force_budget_telemetry_stale_s)
        msg.force_budget_telemetry_fresh_right = (
            self.right_telemetry_t is not None
            and now - self.right_telemetry_t <= self.args.force_budget_telemetry_stale_s)
        msg.master_predicted_force_n = [float(v) for v in (cr.last_master_predicted_force_n or [0.0, 0.0, 0.0])]
        msg.slave_predicted_force_n = [float(v) for v in (cr.last_slave_predicted_force_n or [0.0, 0.0, 0.0])]
        msg.planner_parameter_match_left = bool(self._planner_param_match_left)
        msg.planner_parameter_match_right = bool(self._planner_param_match_right)
        # 2026-09-09 Part 3/5 (validation-infrastructure follow-up):
        # GRAVITY_SETTLE observability, published so the recorder and live
        # tests can see it, not just the pure alignment.diagnostics() dict.
        al = self.alignment
        def _o(attr):
            v = getattr(al, attr, None)
            return float(v) if v is not None else 0.0
        msg.gravity_settle_master_speed_mps = _o("master_settle_velocity_norm_mps")
        msg.gravity_settle_slave_speed_mps = _o("slave_settle_velocity_norm_mps")
        msg.gravity_settle_master_pose_drift_m = _o("master_settle_pose_drift_m")
        msg.gravity_settle_slave_pose_drift_m = _o("slave_settle_pose_drift_m")
        msg.gravity_settle_quiet_window_accumulated_s = _o("settle_quiet_window_accumulated_s")
        msg.gravity_settle_accepted_master_pose_drift_m = _o("last_accepted_settle_master_pose_drift_m")
        msg.gravity_settle_accepted_slave_pose_drift_m = _o("last_accepted_settle_slave_pose_drift_m")
        self.bridge_diag_pub.publish(msg)

    def _update_force_budget_coupling(self) -> None:
        """Drives coupling_ramp_alpha during COUPLING_RAMP_UP via the
        bilateral force-budget planner (2026-09-09 production-integration
        task) instead of the old fixed-time ramp. No-op in every other
        state -- gravity phases, ACTIVE, and the disarm-side ramp-down are
        all untouched by this method (Part 4 scope: arm-side only)."""
        state = self.alignment.state
        if state != "COUPLING_RAMP_UP":
            self._prev_coupling_state = state
            self._telemetry_stale_since = None
            return

        if self._prev_coupling_state != "COUPLING_RAMP_UP":
            # Fresh entry into COUPLING_RAMP_UP this tick -- capture SNAPSHOT
            # origins now, from wherever the devices actually are (Part 4:
            # "at entry: capture fresh master/slave snapshot origin").
            self.alignment.external_coupling_alpha_control = True
            self._coupling_ramp.start_bilateral(
                now=time.time(), master_origin_pose=list(self.latest_left),
                slave_origin_pose=list(self.latest_right) if self.latest_right is not None else list(self.latest_left))
            self._telemetry_stale_since = None
            m_speed = (_norm(list(self.left_telemetry.velocity_mps))
                      if self.left_telemetry is not None else float("nan"))
            s_speed = (_norm(list(self.right_telemetry.velocity_mps))
                      if self.right_telemetry is not None else float("nan"))
            m_drift = self.alignment.last_accepted_settle_master_pose_drift_m
            s_drift = self.alignment.last_accepted_settle_slave_pose_drift_m
            self.get_logger().warn(
                f"COUPLING_RAMP_UP: force-budget planner armed (origin_mode=moving, matches "
                f"blend_target()/blend_left_target() exactly; entered after GRAVITY_SETTLE's "
                f"velocity-gated quiet-hold was satisfied), "
                f"master_pose={self.latest_left} master_speed={m_speed*1000:.3f}mm/s "
                f"master_settle_pose_drift={(m_drift*1000 if m_drift is not None else float('nan')):.4f}mm "
                f"slave_pose={self.latest_right} slave_speed={s_speed*1000:.3f}mm/s "
                f"slave_settle_pose_drift={(s_drift*1000 if s_drift is not None else float('nan')):.4f}mm")
        self._prev_coupling_state = state

        if self.latest_right is None:
            return  # nothing to couple toward yet

        now = time.time()
        left_fresh = (self.left_telemetry_t is not None
                      and now - self.left_telemetry_t <= self.args.force_budget_telemetry_stale_s)
        right_fresh = (self.right_telemetry_t is not None
                       and now - self.right_telemetry_t <= self.args.force_budget_telemetry_stale_s)
        if not (left_fresh and right_fresh):
            # Part 1: HOLD alpha (do not call step_bilateral at all this
            # tick -- an untrusted velocity estimate must never be used,
            # not even as a fallback) and track how long telemetry has been
            # stale, faulting explicitly rather than holding forever.
            if self._telemetry_stale_since is None:
                self._telemetry_stale_since = now
            elif now - self._telemetry_stale_since > self.args.force_budget_stall_timeout_s:
                self.get_logger().error(
                    f"COUPLING_RAMP_TELEMETRY_STALE: left_fresh={left_fresh} right_fresh={right_fresh} "
                    f"for {now - self._telemetry_stale_since:.2f}s -- forcing FAULT")
                self.alignment.trigger_fault("COUPLING_RAMP_TELEMETRY_STALE")
            return
        self._telemetry_stale_since = None

        master_full_target, _ = compute_c2_target(
            self.latest_right, self.alignment.right_anchor, self.alignment.left_anchor,
            self.axis_map, self.axis_scale, float(self.args.left_max_offset_m),
            absolute_axes=self.absolute_axes, absolute_trim=self.absolute_trim,
        )
        slave_full_target, _ = compute_c3_target(
            self.latest_left, self.alignment.left_anchor, self.alignment.right_anchor,
            self.axis_map, self.axis_scale, self.args.deadband_m, self.max_offset,
            self.args.max_offset_mode, absolute_axes=self.absolute_axes, absolute_trim=self.absolute_trim,
        )

        left_aux = [0.0, 0.0, 0.0]
        left_aux[self.args.left_gravity_axis] = ramped_gravity_comp_n(
            float(self.args.left_gravity_comp_n), self.alignment.gravity_ramp_alpha)
        right_aux = [0.0, 0.0, 0.0]
        right_aux[self.args.slave_gravity_axis] = ramped_gravity_comp_n(
            float(self.args.slave_gravity_comp_n), self.alignment.gravity_ramp_alpha)

        # PARAMETER SOURCE (2026-09-09, Part 4 of the validation-infrastructure
        # task): the CONTROLLER'S OWN telemetry is authoritative, not the
        # bridge's CLI args. touch_slave.cpp publishes the exact
        # stiffness/damping/max_position_error/max_force it is actually
        # running with, so the planner uses those directly and cannot
        # silently predict against values the controller isn't using. The CLI
        # args are kept ONLY as a declared expectation, cross-checked below --
        # a disagreement means the launch config (and therefore the session
        # manifest) is wrong about the rig, which is a provenance defect worth
        # faulting on rather than quietly overriding.
        lt, rt = self.left_telemetry, self.right_telemetry
        self._planner_param_match_left = _params_match(
            lt, self.args.master_stiffness_n_per_m, self.args.master_damping_n_s_per_m,
            self.args.master_max_force_n, self.args.master_max_position_error_m)
        self._planner_param_match_right = _params_match(
            rt, self.args.stiffness_n_per_m, self.args.slave_damping_n_s_per_m,
            self.args.slave_max_force_n, self.args.slave_max_position_error_m)
        if not (self._planner_param_match_left and self._planner_param_match_right):
            self.get_logger().error(
                f"COUPLING_RAMP_PARAMETER_MISMATCH: declared planner params disagree with the "
                f"controller's own telemetry. left_match={self._planner_param_match_left} "
                f"right_match={self._planner_param_match_right}. "
                f"master telemetry K={lt.stiffness_n_per_m} D={lt.damping_n_s_per_m} "
                f"Fmax={lt.max_force_n} errmax={lt.max_position_error_m} vs declared "
                f"K={self.args.master_stiffness_n_per_m} D={self.args.master_damping_n_s_per_m} "
                f"Fmax={self.args.master_max_force_n} errmax={self.args.master_max_position_error_m}; "
                f"slave telemetry K={rt.stiffness_n_per_m} D={rt.damping_n_s_per_m} "
                f"Fmax={rt.max_force_n} errmax={rt.max_position_error_m} vs declared "
                f"K={self.args.stiffness_n_per_m} D={self.args.slave_damping_n_s_per_m} "
                f"Fmax={self.args.slave_max_force_n} errmax={self.args.slave_max_position_error_m} "
                f"-- forcing FAULT")
            self.alignment.trigger_fault("COUPLING_RAMP_PARAMETER_MISMATCH")
            return

        master_arm = ArmState(
            live_pose=list(lt.position_m), velocity=list(lt.velocity_mps),
            full_target=master_full_target, stiffness_n_per_m=lt.stiffness_n_per_m,
            damping_n_s_per_m=lt.damping_n_s_per_m, aux_vec=left_aux,
            max_force_n=lt.max_force_n, max_position_error_m=lt.max_position_error_m,
        )
        slave_arm = ArmState(
            live_pose=list(rt.position_m), velocity=list(rt.velocity_mps),
            full_target=slave_full_target, stiffness_n_per_m=rt.stiffness_n_per_m,
            damping_n_s_per_m=rt.damping_n_s_per_m, aux_vec=right_aux,
            max_force_n=rt.max_force_n, max_position_error_m=rt.max_position_error_m,
        )
        try:
            accepted_alpha = self._coupling_ramp.step_bilateral(now=now, dt=1.0 / self.args.rate_hz,
                                                                 master=master_arm, slave=slave_arm)
        except CouplingRampStalled as e:
            self.get_logger().error(f"{e} -- forcing FAULT")
            self.alignment.trigger_fault("COUPLING_RAMP_STALLED_FORCE_BUDGET")
            return
        self.alignment.set_coupling_alpha(accepted_alpha)

    def _update_c2(self) -> None:
        """C2: slave position -> master target, the reverse direction of C3.

        Undoes axis_map/axis_scale to express the slave's own displacement
        (from its anchor) in the master's frame, then adds that to the
        master's anchor -- so the master's spring pulls toward wherever the
        slave actually is, not just wherever it was last commanded to go.
        This is what lets a stalled/obstructed slave be felt on the master
        without needing a real force sensor (that is what --reflect-enable
        already does via tracking error; this is the position-channel
        counterpart the channel table calls C2).
        """
        if self.latest_right is None or self.alignment.right_anchor is None or self.alignment.left_anchor is None:
            return
        # Diagnostic only (BridgeDiagnostics' mapped_slave_relative_m), kept
        # in the anchor-relative frame regardless of absolute_axes so it
        # still reads as "how far from its own anchor is the slave", matching
        # C3's own mapped_unclamped diagnostic convention. NOT the target --
        # see compute_c2_target() for the actual (frame-correct) target.
        slave_disp = [self.latest_right[i] - self.alignment.right_anchor[i] for i in range(3)]
        unscaled = [
            slave_disp[i] / self.axis_scale[i] if abs(self.axis_scale[i]) > 1e-9 else 0.0
            for i in range(3)
        ]
        self._c2_mapped_slave_relative = apply_axis_map_inverse(self.axis_map, unscaled)

        left_target, self._master_travel_limit_flag = compute_c2_target(
            self.latest_right, self.alignment.right_anchor, self.alignment.left_anchor,
            self.axis_map, self.axis_scale, float(self.args.left_max_offset_m),
            absolute_axes=self.absolute_axes, absolute_trim=self.absolute_trim,
        )
        # Same arm_teleop gate as C3's blend_target() (see _tick()'s matching
        # comment) -- before this, left_target was published unconditionally
        # regardless of READY/RAMPING/ACTIVE state, so the master rendered
        # real force even while unarmed (found 2026-09-09, see
        # blend_left_target()'s docstring in startup_alignment.py).
        left_target = self.alignment.blend_left_target(left_target)

        self.last_left_target = left_target
        self._publish_left_target(left_target)

    def _publish_left_target(self, target: list[float]) -> None:
        # enable is now published centrally, once per tick, from _tick() --
        # see the 2026-09-10 command-authority consolidation comment at
        # left_enable_pub's construction site.
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.left_target_pub.publish(msg)

    # ---- force reflection ----------------------------------------------

    def _slave_error(self, target: list[float]) -> Optional[list[float]]:
        """target - actual, in slave frame.

        Prefers the node's own tracking_error topic, which is computed inside the
        200 Hz servo loop against the exact target the servo saw. Falls back to
        differencing our commanded target with the reported pose.
        """
        if self.tracking_error is not None:
            return list(self.tracking_error)
        if self.latest_right is not None:
            return [target[i] - self.latest_right[i] for i in range(3)]
        return None

    def _update_force_reflection(self, target: list[float]) -> None:
        if not self.args.reflect_enable or self.master_force_pub is None:
            return

        error = self._slave_error(target)
        if error is None or self.clutch_active:
            self._decay_and_publish_force()
            return

        err_norm = math.sqrt(sum(v * v for v in error))

        # TRIGGER. The slave always lags a little while it accelerates, so the
        # threshold is the free-motion lag we expect at this speed plus a fixed
        # floor. Only error beyond that counts as an obstruction.
        threshold = self.args.reflect_deadband_m + self.args.reflect_velocity_comp * self.target_speed

        # Stall gate. An obstacle means the slave was told to move and did not.
        # Free motion carries error too -- the servo always lags -- but the slave
        # is still travelling, so error alone cannot tell the two apart. Requiring
        # the slave to be stopped is what makes free space genuinely force-free.
        if self.args.reflect_require_stall:
            keeping_up = max(
                float(self.args.reflect_stall_floor_mps),
                float(self.args.reflect_stall_ratio) * self.target_speed,
            )
            self.stalled = self.slave_speed < keeping_up
        else:
            self.stalled = True

        # Hysteresis: engage at the threshold, release lower down, so error
        # hovering at the boundary does not chatter the force on and off.
        if self.reflect_engaged:
            engaged = err_norm > threshold * self.args.reflect_release_ratio
        else:
            engaged = err_norm > threshold
        engaged = engaged and self.stalled

        if engaged != self.reflect_engaged:
            self.reflect_engaged = engaged
            if engaged:
                self.get_logger().info(
                    f"resistance detected: error={err_norm:.4f} m threshold={threshold:.4f} m"
                )
            else:
                self.get_logger().info("resistance cleared")

        if not engaged:
            self.reflect_excess = 0.0
            self._decay_and_publish_force()
            return

        excess = err_norm - threshold * self.args.reflect_release_ratio
        self.reflect_excess = max(excess, 0.0)

        # Direction: the error points where the slave still needs to go, which is
        # into the obstruction. Map it back to the master frame and negate, so the
        # push opposes the hand that is driving into the constraint.
        direction = [v / err_norm for v in error] if err_norm > 1e-9 else [0.0, 0.0, 0.0]
        magnitude = min(self.args.reflect_gain * self.reflect_excess, self.args.reflect_max_n)
        slave_frame = [-direction[i] * magnitude for i in range(3)]
        raw = apply_axis_map_inverse(self.axis_map, slave_frame)

        # Viscous damping on the master's own motion. OFF by default (0.0), and
        # that default is deliberate.
        #
        # Damping is the textbook cure for a ringing teleoperation loop, but it
        # only dissipates energy when the velocity it opposes is current. Here
        # master_vel is a low-passed derivative of encoder position, and the
        # force it produces arrives after another ~30 ms of round trip. Late
        # enough, "damping" is in phase with the motion and pumps the
        # oscillation instead. A closed-loop simulation of this exact code showed
        # peak-to-peak force rising from 0.007 N to 0.65 N as damping went from
        # 0 to 4.0 N s/m.
        #
        # Left available because the simulation's hand model is a guess and real
        # hardware may disagree. If you try it, start at 0.5 and stop
        # immediately if the vibration grows.
        damping = float(self.args.reflect_master_damping)
        if damping > 0.0:
            raw = [raw[i] - damping * self.master_vel[i] for i in range(3)]

        alpha = min(max(self.args.reflect_alpha, 0.0), 1.0)
        smoothed = [
            (1.0 - alpha) * self.reflect_force[i] + alpha * raw[i] for i in range(3)
        ]
        self.reflect_force = self._slew_limit(smoothed)
        self._publish_master_force(self.reflect_force)

    def _slew_limit(self, target_force: list[float]) -> list[float]:
        """Cap how fast the rendered force may change.

        The master is a haptic device: force moves your hand, your hand moves the
        master, that moves the slave target, which changes the error and the
        force. That loop closes through the hardware and is what oscillates. A
        slew cap bounds the loop gain at high frequency without softening the
        steady-state wall, which a lower gain would.
        """
        limit = float(self.args.reflect_slew_n_per_s) / max(self.args.rate_hz, 1.0)
        if limit <= 0.0:
            return target_force
        delta = [target_force[i] - self.reflect_force[i] for i in range(3)]
        mag = math.sqrt(sum(v * v for v in delta))
        if mag <= limit or mag < 1e-12:
            return target_force
        shrink = limit / mag
        return [self.reflect_force[i] + delta[i] * shrink for i in range(3)]

    def _decay_and_publish_force(self) -> None:
        """Ramp down rather than cutting to zero, so releasing a wall is smooth."""
        alpha = min(max(self.args.reflect_alpha, 0.0), 1.0)
        self.reflect_force = [(1.0 - alpha) * v for v in self.reflect_force]
        if all(abs(v) < 1e-4 for v in self.reflect_force):
            self.reflect_force = [0.0, 0.0, 0.0]
        self._publish_master_force(self.reflect_force)

    def _publish_master_force(self, force: list[float]) -> None:
        if self.master_force_pub is None:
            return
        # enable is now published centrally, once per tick, from _tick() --
        # see the 2026-09-10 command-authority consolidation comment.
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        self.master_force_pub.publish(msg)

    def _note_limit(self, axis: int) -> None:
        """Warn, at most once per axis per 5 s, when travel is being clipped."""
        now = time.time()
        if now - self.limit_notice[axis] < 5.0:
            return
        self.limit_notice[axis] = now
        name = "xyz"[axis]
        self.get_logger().warn(
            f"slave {name} travel clipped at {self.max_offset[axis]*1000:.0f} mm. "
            f"The master is asking for more range than this axis is allowed. "
            f"Lower its AXIS_SCALE or raise its MAX_OFFSET_XYZ entry."
        )

    def _missing_topics(self) -> list[str]:
        # latest_left/latest_right are set on every message unconditionally
        # (see _left_cb/_right_cb), unlike alignment.left_anchor/right_anchor
        # which only become non-None once the full hold-still capture
        # completes -- using the anchors here would misreport "waiting for
        # topic" for the entire alignment sequence even once both topics are
        # genuinely publishing.
        missing = []
        if self.latest_left is None:
            missing.append(self.args.left_pose_topic)
        if self.latest_right is None:
            missing.append(self.args.right_pose_topic)
        if self.latest_left is not None and self.latest_right is not None \
                and self.alignment.left_anchor is None:
            missing.append(f"alignment ({self.alignment.state})")
        return missing

    def _report_waiting(self) -> None:
        """Say what is missing, once a second, and give up after the timeout."""
        now = time.time()
        waited = now - self.start_time
        missing = self._missing_topics()

        if now - self.last_wait_notice >= 1.0:
            self.last_wait_notice = now
            self.get_logger().warn(
                f"waiting {waited:.0f}s for {', '.join(missing)}"
            )

        timeout = float(self.args.wait_timeout_s)
        if timeout > 0.0 and waited > timeout:
            self.get_logger().error(
                f"no publisher on {', '.join(missing)} after {timeout:.0f}s. "
                "Start run_master_left.sh and run_slave_right.sh first, or raise WAIT_TIMEOUT_S."
            )
            raise SystemExit(1)

    def _rate_limit(self, target: list[float]) -> list[float]:
        if self.last_target is None or self.args.max_speed_mps <= 0.0:
            return target
        step_limit = self.args.max_speed_mps / max(self.args.rate_hz, 1.0)
        move = [target[i] - self.last_target[i] for i in range(3)]
        dist = math.sqrt(sum(v * v for v in move))
        if dist <= step_limit or dist < 1e-12:
            return target
        shrink = step_limit / dist
        return [self.last_target[i] + move[i] * shrink for i in range(3)]

    def _publish_gravity_comp(self) -> None:
        """Upward bias, republished every tick so it cannot go stale. Scaled
        by ramp_alpha (added 2026-09-09, arm/disarm force-contract task):
        this used to be a constant regardless of arm state, so the slave
        carried the full configured bias even in READY/unarmed -- meaning
        two different SLAVE_GRAVITY_COMP_N configurations (e.g. the planned
        A2=0.55 vs B3=0.70 comparison) were already physically distinct
        BEFORE arming, violating the same "arm=false must be a single,
        configuration-independent safe state" contract the C2 position fix
        established. ramp_alpha is 0 in READY, 0->1 across RAMPING, and 1 in
        ACTIVE/immediately after a fresh arm -- same ramp already used for
        the C2/C3 targets, not a new one."""
        if self.slave_force_pub is None:
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.slave_frame
        force = [0.0, 0.0, 0.0]
        force[self.args.slave_gravity_axis] = ramped_gravity_comp_n(
            float(self.args.slave_gravity_comp_n), self.alignment.gravity_ramp_alpha)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        self.slave_force_pub.publish(msg)

    def _publish_left_gravity_comp(self) -> None:
        """Master-side counterpart to _publish_gravity_comp(), for C2. Same
        mechanism, same reasoning: a bias on the master's own force_cmd, so
        C2's spring is not spending its whole budget holding the master's
        own weight up before it can track the slave. Same ramp_alpha scaling
        as _publish_gravity_comp() -- see that method's docstring."""
        if self.left_force_pub is None:
            return
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.master_frame
        force = [0.0, 0.0, 0.0]
        force[self.args.left_gravity_axis] = ramped_gravity_comp_n(
            float(self.args.left_gravity_comp_n), self.alignment.gravity_ramp_alpha)
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
        self.left_force_pub.publish(msg)

    def _publish_target(self, target: list[float]) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.right_frame
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def _maybe_report(self, target: list[float]) -> None:
        if not self.args.report:
            return
        now = time.time()
        if now - self.last_report < self.args.report_interval_s:
            return
        self.last_report = now
        d = self.alignment.diagnostics()
        self.get_logger().info(
            f"[align] state={d['startup_state']} ramp_alpha={d['ramp_alpha']:.2f} "
            f"master_age={d['master_pose_age_s']:.3f}s slave_age={d['slave_pose_age_s']:.3f}s "
            f"alignment_error_norm={d['initial_alignment_error_norm']}"
        )
        line = f"[bridge] target=({target[0]:+.4f},{target[1]:+.4f},{target[2]:+.4f})"
        if self.latest_right is not None:
            lag = [target[i] - self.latest_right[i] for i in range(3)]
            lag_norm = math.sqrt(sum(v * v for v in lag))
            line += f" follow_error={lag_norm:.4f} m"
            if self.args.stiffness_n_per_m > 0.0:
                # Reaction force the spring is exerting to hold that error.
                # This is an estimate from a commanded spring, not a measurement.
                line += f" est_load={lag_norm * self.args.stiffness_n_per_m:.3f} N"
        if self.tracking_error is not None:
            te = self.tracking_error
            line += f" node_tracking_error=({te[0]:+.4f},{te[1]:+.4f},{te[2]:+.4f})"
        if self.args.reflect_enable:
            fn = math.sqrt(sum(v * v for v in self.reflect_force))
            state = "ENGAGED" if self.reflect_engaged else "free"
            thr = self.args.reflect_deadband_m + self.args.reflect_velocity_comp * self.target_speed
            line += (
                f" | reflect={state} threshold={thr:.4f}m"
                f" excess={self.reflect_excess:.4f}m force={fn:.3f}N"
                f" slave_v={self.slave_speed:.3f} cmd_v={self.target_speed:.3f}"
                f" {'STALLED' if self.stalled else 'moving '}"
            )
        self.get_logger().info(line)

        # Reach summary. Says plainly whether an axis stopped short, and why.
        parts = []
        for i, name in enumerate("xyz"):
            if self.peak_cmd[i] < 0.005:
                continue
            reached = self.peak_act[i] / self.peak_cmd[i] if self.peak_cmd[i] > 1e-6 else 0.0
            note = ""
            if self.peak_cmd[i] >= self.max_offset[i] * 0.98:
                note = "@LIMIT"          # our clamp is the ceiling
            elif reached < 0.75:
                note = "SHORT"           # commanded fine, slave could not get there
            parts.append(
                f"{name}: asked {self.peak_cmd[i]*1000:5.1f} got {self.peak_act[i]*1000:5.1f} "
                f"({reached:4.2f}){note}"
            )
        if parts:
            self.get_logger().info("  reach mm  " + "  |  ".join(parts))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--left-pose-topic", default="/touch_left/pose")
    parser.add_argument("--left-buttons-topic", default="/touch_left/buttons")
    parser.add_argument("--right-pose-topic", default="/touch_right/pose")
    parser.add_argument("--left-telemetry-topic", default="/touch_left/cartesian_controller_telemetry")
    parser.add_argument("--right-telemetry-topic", default="/touch_right/cartesian_controller_telemetry")
    fb = parser.add_argument_group(
        "force-budget-aware coupling ramp (2026-09-09)",
        "See coupling_force_budget.py's module docstring for the full "
        "root-cause rationale and controller-predictor audit. These are "
        "the planner's OWN belief about each arm's controller parameters "
        "-- the bridge cannot query touch_slave.cpp/touch_slave_sim's "
        "actual configured values at runtime, so a mismatch between these "
        "and what the controller node was ACTUALLY launched with is a "
        "real, silent risk (Part 5's 'parameter source audit' -- not "
        "closed, only made visible: published in /bridge_diagnostics).")
    fb.add_argument("--master-stiffness-n-per-m", type=float, default=15.0,
                    help="must match run_master_left.sh's actual stiffness_n_per_m")
    fb.add_argument("--master-damping-n-s-per-m", type=float, default=0.10,
                    help="must match run_master_left.sh's actual damping_n_s_per_m")
    fb.add_argument("--master-max-force-n", type=float, default=2.0,
                    help="must match run_master_left.sh's actual MAX_FORCE_N")
    fb.add_argument("--master-max-position-error-m", type=float, default=0.03,
                    help="must match touch_slave.cpp's actual max_position_error_m for the master")
    fb.add_argument("--slave-damping-n-s-per-m", type=float, default=0.20,
                    help="must match run_slave_right.sh's actual DAMPING_N_S_PER_M")
    fb.add_argument("--slave-max-force-n", type=float, default=1.5,
                    help="must match run_slave_right.sh's actual MAX_FORCE_N")
    fb.add_argument("--slave-max-position-error-m", type=float, default=0.04,
                    help="must match touch_slave.cpp's actual max_position_error_m for the slave")
    fb.add_argument("--force-budget-reserve-n", type=float, default=0.10,
                    help="safety margin below max_force_n the planner targets -- experimental, "
                         "not asserted optimal")
    fb.add_argument("--force-budget-telemetry-stale-s", type=float, default=0.2,
                    help="if either arm's cartesian_controller_telemetry is older than this "
                         "during COUPLING_RAMP_UP, coupling alpha HOLDS rather than using an "
                         "untrusted velocity estimate (Part 1's explicit requirement)")
    fb.add_argument("--force-budget-stall-timeout-s", type=float, default=3.0)
    align = parser.add_argument_group(
        "startup alignment (2026-09-08 Phase 1)",
        "See startup_alignment.py for the full state machine and the "
        "worked-out geometric meaning of the alignment-error metric.")
    align.add_argument("--alignment-pose-stale-s", type=float, default=0.2)
    align.add_argument("--alignment-hold-still-motion-m", type=float, default=0.003,
                       help="peak-to-peak envelope, per axis, both robots must "
                            "stay within during the hold window")
    align.add_argument("--alignment-hold-still-duration-s", type=float, default=0.75)
    align.add_argument("--alignment-max-error-m", type=float, default=0.15,
                       help="sanity bound on right_minus_mapped_left's norm -- "
                            "NOT a 'must be zero' precision gate, see the module "
                            "docstring for why")
    align.add_argument("--alignment-ramp-s", type=float, default=1.0,
                       help="duration of the RAMPING state's authority blend")
    align.add_argument("--gravity-settle-hold-mode", type=_bool_arg, default=False,
                       help="2026-09-10: if true, remain in GRAVITY_SETTLE indefinitely "
                            "once the quiet-hold gate passes (never auto-advance into "
                            "COUPLING_RAMP_UP) -- exit only via explicit disarm. Default "
                            "false preserves normal production behavior unchanged.")
    align.add_argument("--left-joint-angles-topic", default="",
                       help="optional, for q_left_anchor diagnostics only -- "
                            "not consumed by the control law. Empty disables it.")
    align.add_argument("--right-joint-angles-topic", default="",
                       help="optional, for q_right_anchor diagnostics only.")
    parser.add_argument("--right-target-topic", default="/touch_right/target_pose")
    parser.add_argument("--right-enable-topic", default="/touch_right/enable")
    parser.add_argument("--enable-c2", type=_bool_arg, default=False,
                         help="publish a slave-position-derived target to --left-target-topic, "
                         "so the master's own spring pulls toward the slave's actual position "
                         "(C2 in the channel table). OFF by default: this changes what force "
                         "the operator's hand feels, so it must be turned on deliberately, and "
                         "run_master_left.sh must also be started with stiffness/damping set "
                         "for its own spring to have anything to render against.")
    # 2026-09-10 (item 4, explicit per-channel gating, safe default all
    # False except where changing the pre-existing default would itself be
    # a behavior change unrelated to this task -- --c3-enable keeps
    # defaulting True to match the PRE-2026-09-10 always-on behavior for
    # every existing caller; NEW isolated-channel launches must pass
    # --c3-enable=false explicitly, e.g. for a genuine C2-only stage).
    parser.add_argument("--c3-enable", type=_bool_arg, default=True,
                        help="C3 (master position -> slave). Default True matches this "
                             "project's pre-2026-09-10 always-on behavior. Set false for a "
                             "genuinely isolated non-C3 stage (e.g. real C2-only) -- when "
                             "false, the published slave target is forced to the slave's own "
                             "live pose (zero relative motion) regardless of ramp state.")
    parser.add_argument("--c1-enable", type=_bool_arg, default=False,
                        help="Declares that a C1 relay (bilateral_4ch.py, master<-slave force) "
                             "is expected to be co-running. Does NOT itself compute C1 -- "
                             "that stays bilateral_4ch.py's job -- but makes this bridge "
                             "SUBSCRIBE to --bilateral-status-topic and factor its "
                             "safety_state into the bridge's own single enable decision "
                             "(composition, not a second independent enable publisher -- see "
                             "COMMAND_AUTHORITY_ADR.md). Safe default False.")
    parser.add_argument("--c4-enable", type=_bool_arg, default=False,
                        help="Declares that a C4 relay (bilateral_4ch.py, master->slave force) "
                             "is expected to be co-running. Same composition mechanism as "
                             "--c1-enable (both flags share the one bilateral_status "
                             "subscription). Safe default False.")
    parser.add_argument("--bilateral-status-topic", default="/bilateral_status",
                        help="Only subscribed when --c1-enable or --c4-enable is set.")
    parser.add_argument("--bilateral-status-stale-s", type=float, default=0.5,
                        help="bilateral_4ch.py publishes at --rate-hz there (default 200Hz) -- "
                             "0.5s is generous, matching this project's other freshness "
                             "thresholds' spirit while tolerating startup-order races.")
    parser.add_argument("--left-target-topic", default="/touch_left/target_pose")
    parser.add_argument("--left-enable-topic", default="/touch_left/enable")
    parser.add_argument("--left-max-offset-m", type=float, default=0.08,
                         help="per-axis clamp on how far C2 may pull the master's target from "
                         "its own anchor, independent of the slave-side max-offset clamps")
    parser.add_argument("--tracking-error-topic", default="/touch_right/tracking_error")
    parser.add_argument("--right-frame", default="touch_right_base")
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--axis-map", default="x,y,z", help="master delta -> slave xyz, e.g. '-x,z,y'")
    parser.add_argument("--axis-scale", default="",
                        help="Per-slave-axis scale, e.g. '1.0,0.6,1.0' to shrink vertical travel. "
                             "Empty uses --scale for all three.")
    parser.add_argument("--max-offset-m", type=float, default=0.12)
    parser.add_argument("--max-offset-xyz", default="",
                        help="Per-slave-axis travel limit in metres, e.g. '0.12,0.06,0.12'. "
                             "Empty uses --max-offset-m for all three.")
    parser.add_argument("--absolute-axes", default="",
                        help="Axes that should match the master's ABSOLUTE coordinate rather than "
                             "its displacement, e.g. 'y' to make the slave hold the same height. "
                             "Empty keeps the normal anchor-relative behaviour on every axis.")
    parser.add_argument("--absolute-trim-m", type=float, default=0.0,
                        help="Constant offset added to ALL absolute axes uniformly, to correct a "
                             "known mounting or base-height difference between the two devices. "
                             "Use --absolute-trim-xyz instead if different axes need different "
                             "trims (the common case, since the anchor gap this corrects is rarely "
                             "the same on every axis).")
    parser.add_argument("--absolute-trim-xyz", default="",
                        help="Per-axis version of --absolute-trim-m, e.g. '0,0.012,0.030'. Empty "
                             "uses --absolute-trim-m for all three, same fallback pattern as "
                             "--max-offset-xyz/--max-offset-m.")
    parser.add_argument("--max-offset-mode", choices=("axis", "norm"), default="axis",
                        help="'axis' limits each axis independently. 'norm' restores the original "
                             "vector-length clamp, which bounds total reach more tightly.")
    parser.add_argument("--max-speed-mps", type=float, default=0.5)
    parser.add_argument("--deadband-m", type=float, default=0.0002)
    parser.add_argument("--clutch-button-mask", type=int, default=2)
    # 2026-09-10 (Part C, real-hardware safety completion): bridge-level
    # master deadman gate. Default False preserves the existing "enable
    # always True" keep-alive behavior unchanged for every caller that
    # never opts in.
    parser.add_argument("--require-master-deadman", type=_bool_arg, default=False,
                        help="if true, slave enable/force output require the master's "
                             "configured deadman button to be continuously held, with fresh "
                             "telemetry -- see bridge_master_slave.py's _master_deadman_held()")
    parser.add_argument("--master-deadman-button-mask", type=int, default=1,
                        help="bitmask into /touch_left/buttons, default 1 (HD_DEVICE_BUTTON_1 "
                             "convention, matching touch_slave.cpp's own require_deadman default)")
    parser.add_argument("--master-deadman-stale-s", type=float, default=0.2,
                        help="button telemetry older than this is treated as missing -- "
                             "matches this project's other freshness thresholds (pose_stale_s, "
                             "force_budget_telemetry_stale_s)")
    parser.add_argument("--stiffness-n-per-m", type=float, default=35.0,
                        help="Right node spring gain, used only to estimate load from follow error.")
    parser.add_argument("--report", action="store_true")
    parser.add_argument("--report-interval-s", type=float, default=0.5)
    parser.add_argument("--slave-force-topic", default="/touch_right/force_cmd")
    parser.add_argument("--slave-frame", default="touch_right_base")
    parser.add_argument("--slave-gravity-comp-n", type=float, default=0.0,
                        help="Constant bias force on the slave, N. Offsets the arm's own weight so "
                             "the tracking spring does not spend its authority holding altitude. "
                             "Start around 0.15 and raise until the arm stops sagging.")
    parser.add_argument("--slave-gravity-axis", type=int, default=1, choices=(0, 1, 2),
                        help="Which slave axis is up. 1 (y) for a standard Touch.")
    parser.add_argument("--left-force-topic", default="/touch_left/force_cmd")
    parser.add_argument("--left-gravity-comp-n", type=float, default=0.0,
                        help="C2's counterpart to --slave-gravity-comp-n: constant bias force "
                        "on the master, N, so its own spring is not spending its authority "
                        "holding the master's weight up before it can track the slave "
                        "(--enable-c2). Start around 0.15 and raise until the arm stops sagging.")
    parser.add_argument("--left-gravity-axis", type=int, default=1, choices=(0, 1, 2),
                        help="Which master axis is up. 1 (y) for a standard Touch.")
    parser.add_argument("--calibration-file", default="~/dob_logs/teleop_calibration.json",
                         help="where to record the left/right anchor pair as they are "
                         "captured, for sim-twin seeding and axis-mapping diagnostics "
                         "(see _write_calibration)")
    parser.add_argument("--wait-timeout-s", type=float, default=30.0,
                        help="Give up if both device poses have not arrived in this long. 0 waits forever.")

    reflect = parser.add_argument_group("force reflection (master feels slave resistance)")
    reflect.add_argument("--reflect-enable", action="store_true",
                         help="Push back on the master when the slave cannot reach its target.")
    reflect.add_argument("--master-force-topic", default="/touch_left/force_cmd")
    reflect.add_argument("--master-enable-topic", default="/touch_left/enable")
    reflect.add_argument("--master-frame", default="touch_left_base")
    reflect.add_argument("--reflect-gain", type=float, default=25.0,
                         help="N per meter of excess tracking error. This is the felt stiffness "
                              "of the master. Approaching the slave spring gain makes the "
                              "master/slave loop oscillate; back off if it growls.")
    reflect.add_argument("--reflect-deadband-m", type=float, default=0.004,
                         help="Fixed part of the trigger threshold. Below this, no force.")
    reflect.add_argument("--reflect-velocity-comp", type=float, default=0.05,
                         help="Seconds. Threshold grows by this times the commanded target speed, "
                              "excusing the servo lag that free motion always produces.")
    reflect.add_argument("--reflect-release-ratio", type=float, default=0.6,
                         help="Hysteresis. Once engaged, release below threshold*ratio.")
    reflect.add_argument("--reflect-max-n", type=float, default=1.5,
                         help="Clamp on reflected force in N. Only bites if it is <= the master's "
                              "own max_force_n, so raise both together.")
    reflect.add_argument("--reflect-alpha", type=float, default=0.15,
                         help="Low-pass on the reflected force. Lower is smoother and more stable.")
    reflect.add_argument("--reflect-speed-alpha", type=float, default=0.15,
                         help="Low-pass on the commanded target speed that sets the trigger "
                              "threshold. Unfiltered it injects derivative noise into the force.")
    reflect.add_argument("--reflect-master-damping", type=float, default=0.0,
                         help="N s/m opposing the master's velocity while engaged. OFF by default: "
                              "the velocity estimate lags, so this can pump the oscillation rather "
                              "than damp it. If you try it, start at 0.5.")
    reflect.add_argument("--reflect-require-stall", type=_bool_arg, default=True,
                         help="Only render force when the slave is commanded to move but is not "
                              "moving. This is what keeps free space force-free.")
    reflect.add_argument("--reflect-stall-ratio", type=float, default=0.5,
                         help="Slave counts as stalled below this fraction of commanded speed.")
    reflect.add_argument("--reflect-stall-floor-mps", type=float, default=0.004,
                         help="Speed below which the slave counts as stopped regardless of command.")
    reflect.add_argument("--reflect-slew-n-per-s", type=float, default=8.0,
                         help="Max rate of change of rendered force, N/s. Bounds the hardware "
                              "loop at high frequency. 0 disables.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = MasterSlaveBridge(args)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Raised when the process is signalled (SIGTERM, or Ctrl-C under some
        # RMW builds). A clean exit, not a crash worth a traceback.
        pass
    except SystemExit as exc:
        exit_code = exc.code if isinstance(exc.code, int) else 1
    except Exception as exc:  # noqa: BLE001
        # Real bug found 2026-09-08: SIGTERM (what
        # experiment_launch/99_stop_setup.sh actually sends, vs. the
        # KeyboardInterrupt/ExternalShutdownException this already handled)
        # can invalidate rclpy's context WHILE spin()'s loop is mid-
        # spin_once(), raising a bare rclpy._rclpy_pybind11.RCLError instead
        # of the higher-level ExternalShutdownException this file expected.
        # Confirmed live: a real-hardware C2 diagnostic session, stopped via
        # 99_stop_setup.sh, produced exactly this traceback -- the session's
        # own data up to that point was healthy (no FAULT, normal tracking),
        # this was purely a noisy, otherwise-harmless shutdown-path gap, not
        # a live control fault. Logged, not silently swallowed -- if this
        # ever fires for a DIFFERENT underlying reason, it should still be
        # visible, just not as an unhandled traceback.
        print(f"[bridge] shutdown: {type(exc).__name__}: {exc}", file=sys.stderr)
    finally:
        try:
            # Zero the master force before dropping enable, so the stylus does
            # not hold a push on the way out.
            if node.master_force_pub is not None:
                node._publish_master_force([0.0, 0.0, 0.0])
                time.sleep(0.05)
            disable = Bool()
            disable.data = False
            node.enable_pub.publish(disable)
            node.left_enable_pub.publish(disable)
            time.sleep(0.1)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
