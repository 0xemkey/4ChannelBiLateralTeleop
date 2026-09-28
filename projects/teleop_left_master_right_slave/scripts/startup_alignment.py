"""Deterministic startup/alignment state machine for the master<->slave
Cartesian teleop bridge (2026-09-08 Phase 1; staged arm/disarm transition
added 2026-09-09 after a real-hardware root-cause investigation, see below).

WHAT THIS DOES NOT DO, stated up front per the task's own correction to
its initial framing: it does NOT change the control law. bridge_master_
slave.py's existing formula

    target = right_anchor + mapped(master_pos - left_anchor)

is already anchor-relative (dx_m = x_m - x_m0 in the task's own notation)
and is mathematically INVARIANT to the absolute values of left_anchor/
right_anchor -- any constant shift in either anchor is exactly cancelled
by the subtraction/addition around it. This module does not touch that
formula. What it adds is making the CAPTURE of those two anchors
deterministic, validated, and safely armed, instead of "whichever pose
message happens to arrive first," and (as of 2026-09-09) staging HOW
authority is handed to that formula so that gravity compensation and
bilateral position coupling are never ramped simultaneously.

GEOMETRIC MEANING OF right_minus_mapped_left, worked out precisely because
the task explicitly warns against a threshold chosen without understanding
this: since the control law is anchor-invariant, a nonzero right_minus_
mapped_left does NOT, by itself, mean the control law will behave
incorrectly -- the relative-motion mapping is identical regardless of what
the anchors are. What it DOES measure is a snapshot fact: "at the moment
the anchors were captured, how far was the slave's actual position from
where the (axis-mapped) master position would place it, if the two poses
were meant to be numerically coincident under the mapping." That is only
meaningful as:
  (a) a check that the operator physically placed both arms in
      corresponding postures before capture (a real, useful thing to
      verify -- a large gap here means the two arms LOOK unaligned even
      though the relative-motion math is fine), or
  (b) a sanity bound against a GROSS misconfiguration (wrong axis_map,
      swapped left/right devices, wrong workspace assumption) -- those
      produce alignment errors far larger than any plausible manual
      placement error.
It is NOT evidence of a real base-frame calibration defect merely for
being nonzero: the 2026-09-04 finding already on record (a small, roughly
constant 10-35mm gap on y/z) is the EXPECTED size of (a)-type error from
ordinary manual placement, not a bug. This module therefore gates on a
generous sanity bound (catches (b)) rather than trying to drive this
value to exactly zero (which would be gating on something the control law
never actually required).

STATE MACHINE (rewritten 2026-09-09, staged arm/disarm)
    WAIT_FOR_POSES -> HOLD_STILL -> CAPTURE_ANCHORS -> VALIDATE_ALIGNMENT
        -> READY
    ARM:
        READY -> GRAVITY_RAMP_UP -> GRAVITY_SETTLE -> COUPLING_RAMP_UP -> ACTIVE
    DISARM:
        ACTIVE -> COUPLING_RAMP_DOWN -> GRAVITY_RAMP_DOWN -> READY
    Any state -> FAULT (stale pose, excessive motion during hold,
        impossible/oversized alignment error). FAULT requires an explicit
        reset() -- no auto-recovery, per the task's explicit requirement.
        FAULT is an IMMEDIATE force-safe transition -- gravity_ramp_alpha
        and coupling_ramp_alpha both snap to 0 in the same tick, unlike a
        NORMAL disarm which stages down deliberately (see _fault() and
        Part 4 of the 2026-09-09 staged-transition task).

WHY STAGED (2026-09-09 root-cause investigation): the previous single-stage
RAMPING blended gravity 0->G and bilateral coupling own-pose->target
SIMULTANEOUSLY over one ramp_duration_s. On real hardware this let the
slave's C3 target sweep ~117mm (its own gravity-sagged rest pose -> the
master's held position) while gravity compensation was ALSO still ramping
in, meaning the arm was being asked to travel a large distance while not
yet fully weight-supported -- producing large (30-47mm) tracking error and
driving the slave's force to its 1.5N ceiling (primary_force_clamped).
Symmetrically, on disarm, gravity vanished at the same instant coupling
did, so the arm's own gravity sag immediately competed with the bridge's
target-follows-own-live-pose logic through a 100Hz-target/200Hz-controller
sample-and-hold gap, producing a real but bounded post-disarm spring-force
transient (~0.2N). Neither was a code bug (see test_ramp_disarm_root_cause.py
and this session's real-hardware CSV analysis) -- both were the physical/
scheduling consequence of doing two different physical processes (weight
support, position coupling) at once. Staging them (gravity fully settles
BEFORE coupling engages; coupling fully backs off BEFORE gravity is
removed) removes the interaction, not just masks its symptom.

INTEGRATION: bridge_master_slave.py feeds this module pose samples via
update_master_pose()/update_slave_pose() (called from its existing
_left_cb/_right_cb) and drives it forward via tick() (called once per
_tick()). It then reads .state, .left_anchor, .right_anchor,
.gravity_ramp_alpha, .coupling_ramp_alpha, .ramp_alpha (legacy alias, see
below), .ready, and .diagnostics(). Gravity commands use
.gravity_ramp_alpha (via ramped_gravity_comp_n() in bridge_master_slave.py);
C2/C3 targets are blended via blend_target()/blend_left_target(), which now
use .coupling_ramp_alpha exclusively -- gravity_ramp_alpha never enters the
position-target formula, and coupling_ramp_alpha never enters the gravity
formula, per Part 1's explicit "do not overload one ramp_alpha for
unrelated physical processes" requirement.

`ramp_alpha` is kept as a backward-compatible ALIAS, always exactly equal
to `coupling_ramp_alpha`: this is the one unambiguous meaning that survives
the split, because .state only ever equals ACTIVE once BOTH gravity_ramp_
alpha==1.0 AND coupling_ramp_alpha==1.0 (gravity always finishes ramping
before coupling starts), so external code that already gates on
`state=="ACTIVE" and ramp_alpha>=0.999` (e.g. the scientific recorder's
is_confirmed_active()) continues to work unchanged and continues to mean
exactly what it always meant: "bilateral coupling is fully engaged."
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# 2026-09-09 Part 2 (validation-infrastructure follow-up) explicit labeling:
# this threshold was derived from SIM quiet-state telemetry noise floor
# (investigate_quiet_state_noise_floor.py: master max ~1.85mm/s, slave max
# ~1.08mm/s, both under the sim's own touch_slave_sim controller/estimator
# pipeline). It is NOT validated against real hardware -- the real Touch/
# Phantom Omni devices' own velocity estimation noise floor is unmeasured
# and may differ substantially (different encoder resolution, different
# differentiation/filtering in the real HD API vs the sim's estimator).
# Frozen at this sim-derived value for THIS validation task; a separate
# quiet-state measurement from the real devices is required before this
# gate can be trusted on real hardware -- see the torque-sensing/hardware-
# interaction-hold project memories for the standing "no real hardware in
# this task" constraint this also respects.
SIM_GRAVITY_SETTLE_VELOCITY_THRESHOLD_MPS = 0.005

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
WAIT_FOR_POSES = "WAIT_FOR_POSES"
HOLD_STILL = "HOLD_STILL"
CAPTURE_ANCHORS = "CAPTURE_ANCHORS"
VALIDATE_ALIGNMENT = "VALIDATE_ALIGNMENT"
READY = "READY"
GRAVITY_RAMP_UP = "GRAVITY_RAMP_UP"
GRAVITY_SETTLE = "GRAVITY_SETTLE"
COUPLING_RAMP_UP = "COUPLING_RAMP_UP"
ACTIVE = "ACTIVE"
COUPLING_RAMP_DOWN = "COUPLING_RAMP_DOWN"
GRAVITY_RAMP_DOWN = "GRAVITY_RAMP_DOWN"
FAULT = "FAULT"

# Legacy alias: some callers/tests still reference RAMPING as "any state
# where authority is transitioning." Not a real state -- see _ALL_RAMP_STATES.
_ALL_RAMP_STATES = (GRAVITY_RAMP_UP, GRAVITY_SETTLE, COUPLING_RAMP_UP,
                    COUPLING_RAMP_DOWN, GRAVITY_RAMP_DOWN)

_ALL_STATES = (WAIT_FOR_POSES, HOLD_STILL, CAPTURE_ANCHORS, VALIDATE_ALIGNMENT,
              READY, GRAVITY_RAMP_UP, GRAVITY_SETTLE, COUPLING_RAMP_UP, ACTIVE,
              COUPLING_RAMP_DOWN, GRAVITY_RAMP_DOWN, FAULT)


@dataclass
class AlignmentConfig:
    # WAIT_FOR_POSES: a pose older than this is not "fresh" -- generous
    # against the ~200Hz publish rate (5ms native period) so ordinary DDS
    # jitter never blocks progress, tight enough to catch a genuinely dead
    # topic quickly.
    pose_stale_s: float = 0.2

    # HOLD_STILL: both robots must stay within this peak-to-peak envelope,
    # per axis, for hold_still_duration_s continuously. 3mm is comfortably
    # above ordinary hand tremor/encoder noise (the bridge's own steady-
    # state deadband_m default is 0.0002m, two orders of magnitude tighter,
    # but that is a MOTION deadband during active teleop, not a "did a
    # human hold still" bound) and comfortably below any deliberate
    # repositioning motion.
    hold_still_motion_m: float = 0.003
    hold_still_duration_s: float = 0.75

    # VALIDATE_ALIGNMENT: sanity bound on right_minus_mapped_left's norm,
    # NOT a "must equal zero" precision gate -- see the module docstring.
    # 0.15m is chosen to sit above the largest plausible manual-placement
    # gap (documented precedent: 10-35mm on y/z) with real margin, while
    # still catching a genuinely wrong axis_map/swapped-device case, which
    # produces errors comparable to the device's own ~150-190mm workspace
    # radius, not a few cm.
    max_alignment_error_m: float = 0.15

    # COUPLING_RAMP_UP: commanded target blends from own-live-pose (zero
    # relative motion) to the full C2/C3 target over this duration. 1.0s,
    # unchanged from the pre-staging default (this is the SAME quantity the
    # old single-stage RAMPING used, kept under its original name/CLI flag
    # for backward compatibility -- see bridge_master_slave.py's
    # --alignment-ramp-s).
    ramp_duration_s: float = 1.0

    # GRAVITY_RAMP_UP: gravity compensation ramps 0->configured over this
    # duration BEFORE any bilateral coupling begins -- see the module
    # docstring's root-cause rationale. Deliberately shorter than the
    # coupling ramp: gravity is a single scalar bias per axis, not a
    # potentially-large position sweep, so it does not need as much time.
    gravity_ramp_duration_s: float = 0.3

    # GRAVITY_SETTLE (2026-09-09, REVISED after a real-hardware/live-sim
    # finding: the original fixed gravity_settle_duration_s=0.2s snapshotted
    # the COUPLING_RAMP_UP origin mid-transient -- measured slave vy was
    # STILL INCREASING (+0.032 -> +0.122 m/s) at the end of that 0.2s window,
    # nowhere near settled. A fixed clock cannot know how long the real
    # plant's step response to newly-applied gravity actually takes, so this
    # is now a STATE-BASED gate: advance to COUPLING_RAMP_UP only once
    # ||velocity_mps|| for BOTH arms has stayed under a noise-floor-derived
    # threshold continuously for gravity_settle_hold_s, with a minimum time
    # floor and a maximum timeout (see investigate_gravity_settle_timing.py
    # and investigate_quiet_state_noise_floor.py for the measurements this
    # was derived from).
    #
    # Threshold derivation (investigate_quiet_state_noise_floor.py, 2026-09-09):
    # measured quiet-state (mechanically-at-rest) velocity_mps norm noise
    # floor over an 801-sample/2.0s window, post the fixture's own 3.0s
    # native-physics settle:
    #   master (pinned by the sim's stiff Hand): mean=0.578 rms=0.830
    #       p95=1.845 max=1.852 mm/s
    #   slave  (native-gravity settle, no bridge control active): mean=1.027
    #       rms=1.028 p95=1.075 max=1.081 mm/s
    # Both arms' observed noise ceiling sits under 2mm/s. 5mm/s gives
    # >2.7x margin over master's max and >4.6x over slave's max, so ordinary
    # sensor/estimator noise at rest never falsely reads as "still moving",
    # while remaining two orders of magnitude below the genuine transient
    # velocities observed during an actual gravity-settle overshoot
    # (order 100-150mm/s in the 117mm live run that motivated this change).
    #
    # 2026-09-09 Part 2 explicit labeling: this is SIM_GRAVITY_SETTLE_
    # VELOCITY_THRESHOLD_MPS (see the module-level constant of that name) --
    # a SIM-derived value, NOT validated against real hardware. Do not treat
    # this default as applicable to a real-hardware run without a separate
    # quiet-state noise-floor measurement from the real devices first.
    gravity_settle_velocity_threshold_mps: float = SIM_GRAVITY_SETTLE_VELOCITY_THRESHOLD_MPS

    # Velocity must stay under the threshold CONTINUOUSLY for this long
    # before GRAVITY_SETTLE is considered complete -- rejects a single lucky
    # noise dip through zero mid-oscillation from being mistaken for having
    # actually settled. ~0.15s covers many samples at this project's typical
    # 100-200Hz telemetry/tick rates.
    gravity_settle_hold_s: float = 0.15

    # Minimum time GRAVITY_SETTLE must remain active even if velocity reads
    # quiet immediately -- guards against a single fortunate quiet tick right
    # as GRAVITY_RAMP_UP completes (before any real transient has even had a
    # chance to develop) short-circuiting the gate.
    gravity_settle_min_time_s: float = 0.1

    # Backstop: if the plant has not satisfied the quiet-hold condition
    # within this long, GRAVITY_SETTLE_TIMEOUT faults (immediate force-safe)
    # rather than snapshotting a mid-transient origin -- the real 117mm live
    # run showed velocity still large (no decay below the noise floor) after
    # 3.3s of continuous observation, so 10s gives real margin above that
    # while still being a bounded, reportable failure rather than an
    # unbounded hang.
    gravity_settle_timeout_s: float = 10.0

    # Legacy name, kept ONLY as the attribute existing callers/tests may
    # still reference for the old fixed-duration behavior's approximate
    # scale -- NOT read by tick() any more (see GRAVITY_SETTLE's velocity-
    # gated logic above). Retained rather than deleted so an old test that
    # merely constructs AlignmentConfig(gravity_settle_duration_s=...) does
    # not hard-error; it simply has no effect.
    gravity_settle_duration_s: float = 0.2

    # COUPLING_RAMP_DOWN: symmetric to COUPLING_RAMP_UP, coupling 1->0 with
    # gravity held constant. Same duration by default (a symmetric ramp is
    # the conservative choice absent evidence either direction needs to
    # move faster/slower).
    coupling_ramp_down_duration_s: float = 1.0

    # SELF_FOLLOW settle window between coupling reaching 0 and gravity
    # beginning to ramp down -- lets the "own target follows own live pose,
    # spring force near zero" state (Part 3's SELF_FOLLOW_CONFIRMED) actually
    # be confirmed by elapsed time before gravity is touched, rather than
    # removing gravity in the very same tick coupling reaches zero.
    self_follow_settle_duration_s: float = 0.2

    # GRAVITY_RAMP_DOWN: gravity ramps configured->0 over this duration,
    # ONLY after coupling has already reached zero and settled -- this is
    # what removes the disarm-side interaction the root-cause investigation
    # found (gravity disappearing while coupling was still live/just-removed).
    gravity_ramp_down_duration_s: float = 0.3


@dataclass
class _PoseSample:
    t: float
    pos: List[float]


class TeleopStartupAlignment:
    """Pure Python, no ROS dependency -- same "framework-free, unit-testable
    without rclpy" convention this project already uses for bilateral_4ch.py's
    analyze_window() (see sim/tests/test_oscillation_guard.py). The ROS glue
    (subscriptions calling update_*_pose(), a timer calling tick(), a
    parameter callback calling arm()) lives in bridge_master_slave.py.
    """

    def __init__(self, config: Optional[AlignmentConfig] = None,
                now_fn=time.time) -> None:
        self.cfg = config or AlignmentConfig()
        self._now = now_fn

        self.state = WAIT_FOR_POSES
        self._fault_reason: Optional[str] = None

        self._latest_master: Optional[_PoseSample] = None
        self._latest_slave: Optional[_PoseSample] = None
        # 2026-09-09 GRAVITY_SETTLE revision: velocity is optional input
        # (None until update_master_velocity/update_slave_velocity is
        # called at least once) -- None is treated as "not known quiet",
        # never as "assumed zero", so a caller that forgets to wire velocity
        # in cannot accidentally short-circuit the settle gate.
        self._latest_master_vel: Optional[List[float]] = None
        self._latest_slave_vel: Optional[List[float]] = None
        # Tracks the start time of the CURRENT unbroken run of "both arms
        # under the velocity threshold" -- None means not currently quiet.
        self._quiet_since_t: Optional[float] = None
        # 2026-09-09 Part 3 observability (see GRAVITY_SETTLE's tick() branch
        # for how these are populated).
        self._quiet_window_master_pos0: Optional[List[float]] = None
        self._quiet_window_slave_pos0: Optional[List[float]] = None
        self.master_settle_velocity_norm_mps: Optional[float] = None
        self.slave_settle_velocity_norm_mps: Optional[float] = None
        self.master_settle_pose_drift_m: Optional[float] = None
        self.slave_settle_pose_drift_m: Optional[float] = None
        self.settle_quiet_window_accumulated_s: float = 0.0
        self.last_accepted_settle_master_pose_drift_m: Optional[float] = None
        self.last_accepted_settle_slave_pose_drift_m: Optional[float] = None

        self._hold_window_start: Optional[float] = None
        self._hold_master_samples: List[_PoseSample] = []
        self._hold_slave_samples: List[_PoseSample] = []

        self.left_anchor: Optional[List[float]] = None
        self.right_anchor: Optional[List[float]] = None
        self.capture_timestamp: Optional[float] = None
        # Joint angles, captured alongside the Cartesian anchors when the
        # caller supplies them (optional -- see update_master_q/update_slave_q).
        # Not consumed by the control law (which is purely Cartesian); kept
        # for diagnostics only, per the task's explicit CAPTURE_ANCHORS spec.
        self._latest_master_q: Optional[List[float]] = None
        self._latest_slave_q: Optional[List[float]] = None
        self.q_left_anchor: Optional[List[float]] = None
        self.q_right_anchor: Optional[List[float]] = None

        self.initial_alignment_error_xyz: Optional[List[float]] = None
        self.initial_alignment_error_norm: Optional[float] = None

        self._armed_requested = False
        self._disarmed_requested = False

        # Two independent ramp scalars (2026-09-09 staged-transition
        # rewrite) -- see the module docstring for why these are kept
        # separate rather than one overloaded ramp_alpha.
        self.gravity_ramp_alpha: float = 0.0
        self.coupling_ramp_alpha: float = 0.0
        self._phase_start_t: Optional[float] = None
        # The alpha value coupling/gravity ramps started FROM this phase
        # (usually 0.0 or 1.0, but can be partial if disarm/re-arm interrupts
        # a ramp already in progress -- see _ramp_value()).
        self._phase_start_alpha: float = 0.0

        # 2026-09-09 production-integration task: when True, tick()'s
        # COUPLING_RAMP_UP branch does not compute coupling_ramp_alpha
        # itself -- the caller (bridge_master_slave.py, which owns an
        # AdaptiveCouplingRamp with real controller telemetry) drives it via
        # set_coupling_alpha() before each tick() call. False (default)
        # preserves the original fixed-time behavior unchanged, so every
        # existing caller/test that never touches this flag is unaffected.
        self.external_coupling_alpha_control: bool = False

        # 2026-09-10 (real-hardware Y-axis validation, Part 1): optional
        # gravity-settle-only/hold mode -- False (default) preserves the
        # original fixed READY->...->ACTIVE behavior unchanged for every
        # existing caller/test that never touches this flag. When True, the
        # state machine remains in GRAVITY_SETTLE indefinitely once the
        # quiet-hold gate has passed (gravity_ramp_alpha=1,
        # coupling_ramp_alpha=0, exit only via explicit disarm()) instead of
        # auto-advancing into COUPLING_RAMP_UP. Same convention as
        # external_coupling_alpha_control above -- a plain public attribute
        # the caller (bridge or a standalone diagnostic script) sets
        # directly, not a constructor-only config field, so it can be
        # toggled per-run without a new AlignmentConfig instance.
        self.gravity_settle_hold_mode: bool = False
        self._gravity_settle_gate_passed: bool = False

        self._transitions: List[Tuple[float, str, str]] = []  # (t, from, to)

    # ---- inputs, called from ROS callbacks --------------------------------

    def update_master_pose(self, pos: List[float], t: Optional[float] = None) -> None:
        self._latest_master = _PoseSample(t if t is not None else self._now(), list(pos))

    def update_slave_pose(self, pos: List[float], t: Optional[float] = None) -> None:
        self._latest_slave = _PoseSample(t if t is not None else self._now(), list(pos))

    def update_master_velocity(self, vel: List[float]) -> None:
        """2026-09-09 GRAVITY_SETTLE revision: velocity_mps from the
        controller's own telemetry (the same quantity HD_CURRENT_VELOCITY-
        derived and already used by coupling_force_budget.py's predictor),
        fed in by the caller so this module can gate GRAVITY_SETTLE on
        actual mechanical quiescence rather than a fixed clock. Optional --
        a caller that never calls this simply never satisfies the quiet
        condition and relies on gravity_settle_timeout_s (see GRAVITY_SETTLE
        below), rather than silently mis-measuring zero velocity."""
        self._latest_master_vel = list(vel)

    def update_slave_velocity(self, vel: List[float]) -> None:
        self._latest_slave_vel = list(vel)

    def update_master_q(self, q: List[float]) -> None:
        self._latest_master_q = list(q)

    def update_slave_q(self, q: List[float]) -> None:
        self._latest_slave_q = list(q)

    def arm(self) -> None:
        """Operator-explicit arm request. Only takes effect from READY --
        harmless (ignored) from any other state, matching bilateral_4ch.py's
        own manual_rearm convention of being a no-op unless in the state it
        actually applies to."""
        self._armed_requested = True

    def disarm(self) -> None:
        """Operator-explicit NORMAL disarm request -- staged coupling-down
        then gravity-down (see the module docstring's DISARM sequence and
        Part 4's explicit NORMAL-vs-EMERGENCY distinction: this is the
        graceful path, not the immediate-force-safe FAULT path). Only sets a
        flag, exactly like arm() -- tick() (the ONLY place state actually
        transitions) interprets it per-state below, so an external call can
        never mutate state outside the tick cycle or race a
        staleness/FAULT check. Takes effect from ACTIVE, or from anywhere in
        the arm sequence (GRAVITY_RAMP_UP/GRAVITY_SETTLE/COUPLING_RAMP_UP) --
        disarming mid-arm gracefully unwinds whatever has already been
        engaged rather than first completing the arm sequence. No-op from
        READY or already-disarming states (tick() simply never checks the
        flag there)."""
        self._disarmed_requested = True

    def set_coupling_alpha(self, value: float) -> None:
        """External-planner entrypoint (2026-09-09 production-integration
        task): the caller sets this to the force-budget-accepted alpha for
        the CURRENT tick, before calling tick() -- only takes effect while
        external_coupling_alpha_control is True and state is
        COUPLING_RAMP_UP (silently ignored otherwise, matching arm()/
        disarm()'s own "no-op outside the state it applies to" convention,
        so a stray call from the wrong phase cannot corrupt state). Enforces
        monotonic non-decrease here too, as defense in depth against a
        buggy caller -- coupling_force_budget.py's own AdaptiveCouplingRamp
        already guarantees this, but this is the single authoritative
        coupling_ramp_alpha value in the whole system (Part 4's explicit
        'there must be exactly one authoritative coupling_ramp_alpha'
        requirement), so it re-asserts the invariant rather than trusting
        the caller silently."""
        if not self.external_coupling_alpha_control or self.state != COUPLING_RAMP_UP:
            return
        assert value >= self.coupling_ramp_alpha - 1e-9, (
            f"coupling_ramp_alpha must be monotonically non-decreasing, "
            f"got {value} after {self.coupling_ramp_alpha}")
        self.coupling_ramp_alpha = min(1.0, max(self.coupling_ramp_alpha, value))
        self._armed_requested = False

    def reset(self) -> None:
        """Explicit re-arm after FAULT. No auto-recovery -- the task's own
        requirement -- so this must be called deliberately (e.g. a ROS
        parameter, same trigger style as bilateral_4ch.py's manual_rearm)."""
        self._transition(WAIT_FOR_POSES)
        self._fault_reason = None
        self._hold_window_start = None
        self._hold_master_samples = []
        self._hold_slave_samples = []
        self.left_anchor = None
        self.right_anchor = None
        self.q_left_anchor = None
        self.q_right_anchor = None
        self.capture_timestamp = None
        self.initial_alignment_error_xyz = None
        self.initial_alignment_error_norm = None
        self._armed_requested = False
        self._disarmed_requested = False
        self.gravity_ramp_alpha = 0.0
        self.coupling_ramp_alpha = 0.0
        self._phase_start_t = None
        self._phase_start_alpha = 0.0
        self._quiet_since_t = None
        self._quiet_window_master_pos0 = None
        self._quiet_window_slave_pos0 = None
        self.master_settle_velocity_norm_mps = None
        self.slave_settle_velocity_norm_mps = None
        self.master_settle_pose_drift_m = None
        self.slave_settle_pose_drift_m = None
        self.settle_quiet_window_accumulated_s = 0.0
        self.last_accepted_settle_master_pose_drift_m = None
        self.last_accepted_settle_slave_pose_drift_m = None
        self._gravity_settle_gate_passed = False
        # gravity_settle_hold_mode itself is NOT reset here -- it is a
        # caller-controlled mode selection (like external_coupling_alpha_
        # control), not per-cycle state; reset() clears the STATE MACHINE's
        # progress, not the caller's mode choice.

    # ---- mapping, supplied by the caller (bridge owns axis_map/scale) -----

    def compute_alignment_error(self, left_anchor: List[float], right_anchor: List[float],
                                mapped_left_anchor: List[float]) -> Tuple[List[float], float]:
        err = [right_anchor[i] - mapped_left_anchor[i] for i in range(3)]
        norm = sum(v * v for v in err) ** 0.5
        return err, norm

    # ---- state machine ------------------------------------------------

    def _transition(self, new_state: str) -> None:
        if new_state == self.state:
            return
        self._transitions.append((self._now(), self.state, new_state))
        self.state = new_state

    def _begin_phase(self, new_state: str) -> None:
        """Enter a ramp phase, recording the CURRENT alpha (gravity's or
        coupling's, whichever this phase drives) as the ramp's starting
        point -- so disarming mid-ramp-up reverses smoothly from wherever
        authority actually is, not from an assumed 0 or 1 (Part 1's
        "preserve existing hard safety behavior" plus ordinary continuity)."""
        self._phase_start_t = self._now()
        if new_state in (GRAVITY_RAMP_UP, GRAVITY_RAMP_DOWN):
            self._phase_start_alpha = self.gravity_ramp_alpha
        elif new_state in (COUPLING_RAMP_UP, COUPLING_RAMP_DOWN):
            self._phase_start_alpha = self.coupling_ramp_alpha
        if new_state == GRAVITY_SETTLE:
            self._quiet_since_t = None
            self._quiet_window_master_pos0 = None
            self._quiet_window_slave_pos0 = None
            self.master_settle_pose_drift_m = None
            self.slave_settle_pose_drift_m = None
            self.settle_quiet_window_accumulated_s = 0.0
            self._gravity_settle_gate_passed = False
        self._transition(new_state)

    def _fault(self, reason: str) -> None:
        """IMMEDIATE force-safe transition (Part 4): unlike a normal disarm,
        FAULT does not stage anything down -- both ramp scalars snap to 0 in
        this same tick, so blend_target()/blend_left_target() and the
        gravity command are already zero-coupling/zero-gravity on the very
        next publish, regardless of what phase the arm/disarm sequence was
        in when the fault occurred."""
        self._fault_reason = reason
        self._transition(FAULT)
        self.gravity_ramp_alpha = 0.0
        self.coupling_ramp_alpha = 0.0

    def trigger_fault(self, reason: str) -> None:
        """Public entrypoint for an EXTERNAL safety abort (2026-09-09
        production-integration task) -- e.g. bridge_master_slave.py's force-
        budget planner reporting COUPLING_RAMP_STALLED_FORCE_BUDGET or
        COUPLING_RAMP_TELEMETRY_STALE. Identical semantics to the internal
        _fault() path (immediate force-safe, no staged unwind) -- exposed
        as a named public method rather than requiring callers to reach
        into the private _fault()."""
        self._fault(reason)

    @property
    def ready(self) -> bool:
        return self.state in (READY, GRAVITY_RAMP_UP, GRAVITY_SETTLE, COUPLING_RAMP_UP,
                              ACTIVE, COUPLING_RAMP_DOWN, GRAVITY_RAMP_DOWN)

    @property
    def ramp_alpha(self) -> float:
        """Legacy backward-compatible alias -- see the module docstring's
        explanation of why this equals coupling_ramp_alpha exactly and
        remains unambiguous for existing callers gating on
        state=="ACTIVE" and ramp_alpha>=0.999."""
        return self.coupling_ramp_alpha

    def _ramp_value(self, elapsed: float, duration: float, direction: int) -> float:
        """direction=+1: ramps from self._phase_start_alpha toward 1.0.
        direction=-1: ramps from self._phase_start_alpha toward 0.0.
        Proportional to the FULL nominal duration, so resuming/reversing a
        partially-completed ramp takes proportionally less time (a smooth,
        bounded reversal, not a jump) -- exercised by
        test_partial_ramp_reversal_is_smooth."""
        span = 1.0 - self._phase_start_alpha if direction > 0 else self._phase_start_alpha
        frac = min(1.0, max(0.0, elapsed / max(duration, 1e-6)))
        if direction > 0:
            return min(1.0, self._phase_start_alpha + frac * span)
        return max(0.0, self._phase_start_alpha - frac * span)

    def tick(self, mapper_fn) -> None:
        """mapper_fn(pos_xyz) -> mapped_xyz: the bridge's own apply_axis_map
        + axis_scale composition, passed in rather than duplicated here, so
        this module never has its own copy of that math to drift out of
        sync with the real one."""
        now = self._now()

        if self.state == FAULT:
            return  # no auto-recovery; caller must reset()

        # Staleness is checked in every state except WAIT_FOR_POSES itself
        # (nothing to be stale yet) and FAULT (already handled above).
        if self.state != WAIT_FOR_POSES:
            if self._latest_master is None or now - self._latest_master.t > self.cfg.pose_stale_s:
                self._fault(f"master pose stale (state={self.state})")
                return
            if self._latest_slave is None or now - self._latest_slave.t > self.cfg.pose_stale_s:
                self._fault(f"slave pose stale (state={self.state})")
                return

        if self.state == WAIT_FOR_POSES:
            if (self._latest_master is not None and self._latest_slave is not None
                    and now - self._latest_master.t <= self.cfg.pose_stale_s
                    and now - self._latest_slave.t <= self.cfg.pose_stale_s):
                self._transition(HOLD_STILL)
                self._hold_window_start = now
                self._hold_master_samples = [self._latest_master]
                self._hold_slave_samples = [self._latest_slave]
            return

        if self.state == HOLD_STILL:
            self._hold_master_samples.append(self._latest_master)
            self._hold_slave_samples.append(self._latest_slave)
            m_motion = _peak_to_peak_norm(self._hold_master_samples)
            s_motion = _peak_to_peak_norm(self._hold_slave_samples)
            if m_motion > self.cfg.hold_still_motion_m:
                self._hold_window_start = now
                self._hold_master_samples = [self._latest_master]
                self._hold_slave_samples = [self._latest_slave]
                return
            if s_motion > self.cfg.hold_still_motion_m:
                self._hold_window_start = now
                self._hold_master_samples = [self._latest_master]
                self._hold_slave_samples = [self._latest_slave]
                return
            if now - self._hold_window_start >= self.cfg.hold_still_duration_s:
                self._transition(CAPTURE_ANCHORS)
            return

        if self.state == CAPTURE_ANCHORS:
            self.left_anchor = _mean_pos(self._hold_master_samples)
            self.right_anchor = _mean_pos(self._hold_slave_samples)
            self.capture_timestamp = now
            if self._latest_master_q is not None:
                self.q_left_anchor = list(self._latest_master_q)
            if self._latest_slave_q is not None:
                self.q_right_anchor = list(self._latest_slave_q)
            self._transition(VALIDATE_ALIGNMENT)
            return

        if self.state == VALIDATE_ALIGNMENT:
            mapped_left = mapper_fn(self.left_anchor)
            err, norm = self.compute_alignment_error(self.left_anchor, self.right_anchor, mapped_left)
            self.initial_alignment_error_xyz = err
            self.initial_alignment_error_norm = norm
            if norm > self.cfg.max_alignment_error_m:
                self._fault(
                    f"alignment error {norm:.4f}m exceeds sanity bound "
                    f"{self.cfg.max_alignment_error_m:.4f}m -- likely wrong "
                    f"axis_map, swapped devices, or a genuine mapping defect, "
                    f"not ordinary manual-placement offset"
                )
                return
            self._transition(READY)
            return

        if self.state == READY:
            if self._armed_requested:
                self._armed_requested = False
                self._begin_phase(GRAVITY_RAMP_UP)
            return

        # ---- ARM sequence: GRAVITY_RAMP_UP -> GRAVITY_SETTLE ->
        #      COUPLING_RAMP_UP -> ACTIVE. Coupling stays exactly 0 through
        #      both gravity phases (Part 2's critical invariant: "bilateral
        #      position error must NOT generate spring demand during the
        #      gravity-only phase" -- enforced structurally here, since
        #      coupling_ramp_alpha is simply never touched by these branches). ----

        if self.state == GRAVITY_RAMP_UP:
            if self._disarmed_requested:
                self._disarmed_requested = False
                self._begin_phase(GRAVITY_RAMP_DOWN)
                return
            elapsed = now - self._phase_start_t
            self.gravity_ramp_alpha = self._ramp_value(elapsed, self.cfg.gravity_ramp_duration_s, +1)
            if self.gravity_ramp_alpha >= 1.0:
                self._begin_phase(GRAVITY_SETTLE)
            return

        if self.state == GRAVITY_SETTLE:
            if self._disarmed_requested:
                self._disarmed_requested = False
                self._begin_phase(GRAVITY_RAMP_DOWN)
                return
            self.gravity_ramp_alpha = 1.0

            if self.gravity_settle_hold_mode and self._gravity_settle_gate_passed:
                # 2026-09-10 (real-hardware Y-axis validation, Part 1):
                # gravity-settle-only/hold mode. The quiet gate already
                # passed once (see below) -- HOLD here indefinitely rather
                # than transitioning to COUPLING_RAMP_UP. gravity_ramp_alpha
                # stays 1.0, coupling_ramp_alpha stays exactly 0 (never
                # touched by this branch -- same structural invariant the
                # module docstring already documents for the gravity-only
                # phases), and no timeout re-check applies once the gate has
                # already been satisfied. Exit ONLY via explicit disarm
                # (handled above, before this check).
                return

            elapsed = now - self._phase_start_t
            if elapsed >= self.cfg.gravity_settle_timeout_s:
                self._fault(
                    f"GRAVITY_SETTLE_TIMEOUT: mechanical settle not confirmed "
                    f"(velocity under {self.cfg.gravity_settle_velocity_threshold_mps*1000:.1f}mm/s "
                    f"held for {self.cfg.gravity_settle_hold_s:.2f}s) within "
                    f"{self.cfg.gravity_settle_timeout_s:.1f}s"
                )
                return

            quiet_now = False
            self.master_settle_velocity_norm_mps = (
                _norm3(self._latest_master_vel) if self._latest_master_vel is not None else None)
            self.slave_settle_velocity_norm_mps = (
                _norm3(self._latest_slave_vel) if self._latest_slave_vel is not None else None)
            if self._latest_master_vel is not None and self._latest_slave_vel is not None:
                m_speed = self.master_settle_velocity_norm_mps
                s_speed = self.slave_settle_velocity_norm_mps
                quiet_now = (m_speed < self.cfg.gravity_settle_velocity_threshold_mps
                            and s_speed < self.cfg.gravity_settle_velocity_threshold_mps)

            # 2026-09-09 Part 3 (validation-infrastructure follow-up)
            # observability: track pose drift over the CURRENT quiet window
            # (position now vs. position when this quiet window started) --
            # purpose is to catch the velocity gate passing on a pathological
            # oscillation zero-crossing (near-zero velocity for an instant
            # while still moving through a large-amplitude swing), which a
            # velocity-only check cannot distinguish from genuine rest.
            if quiet_now:
                if self._quiet_since_t is None:
                    self._quiet_since_t = now
                    self._quiet_window_master_pos0 = (
                        list(self._latest_master.pos) if self._latest_master else None)
                    self._quiet_window_slave_pos0 = (
                        list(self._latest_slave.pos) if self._latest_slave else None)
                    self.master_settle_pose_drift_m = 0.0
                    self.slave_settle_pose_drift_m = 0.0
                elif self._latest_master is not None and self._quiet_window_master_pos0 is not None:
                    d = _norm3([self._latest_master.pos[i] - self._quiet_window_master_pos0[i]
                               for i in range(3)])
                    self.master_settle_pose_drift_m = max(self.master_settle_pose_drift_m or 0.0, d)
                    if self._latest_slave is not None and self._quiet_window_slave_pos0 is not None:
                        ds = _norm3([self._latest_slave.pos[i] - self._quiet_window_slave_pos0[i]
                                    for i in range(3)])
                        self.slave_settle_pose_drift_m = max(self.slave_settle_pose_drift_m or 0.0, ds)
            else:
                self._quiet_since_t = None
                self._quiet_window_master_pos0 = None
                self._quiet_window_slave_pos0 = None
                self.master_settle_pose_drift_m = None
                self.slave_settle_pose_drift_m = None

            quiet_hold_satisfied = (
                self._quiet_since_t is not None
                and now - self._quiet_since_t >= self.cfg.gravity_settle_hold_s
            )
            min_time_satisfied = elapsed >= self.cfg.gravity_settle_min_time_s
            self.settle_quiet_window_accumulated_s = (
                (now - self._quiet_since_t) if self._quiet_since_t is not None else 0.0)

            if quiet_hold_satisfied and min_time_satisfied:
                # Report the FINAL ACCEPTED quiet window's max pose drift
                # before _begin_phase() resets GRAVITY_SETTLE's own tracking
                # state (see _begin_phase()'s "if new_state == GRAVITY_SETTLE"
                # branch) -- this is the Part 3 "max pose drift during the
                # final accepted quiet window" report value, frozen at the
                # moment the gate passes rather than reset to None.
                self.last_accepted_settle_master_pose_drift_m = self.master_settle_pose_drift_m
                self.last_accepted_settle_slave_pose_drift_m = self.slave_settle_pose_drift_m
                self._gravity_settle_gate_passed = True
                if self.gravity_settle_hold_mode:
                    # Stay in GRAVITY_SETTLE -- see the hold-mode check at
                    # the top of this branch, which now takes over on every
                    # subsequent tick. Do NOT transition to COUPLING_RAMP_UP.
                    return
                self._begin_phase(COUPLING_RAMP_UP)
            return

        if self.state == COUPLING_RAMP_UP:
            if self._disarmed_requested:
                self._disarmed_requested = False
                self._begin_phase(COUPLING_RAMP_DOWN)
                return
            if self.external_coupling_alpha_control:
                # 2026-09-09 production-integration task: the caller (bridge_
                # master_slave.py) owns an AdaptiveCouplingRamp and has
                # ALREADY set self.coupling_ramp_alpha to the force-budget-
                # accepted value for this tick, via set_coupling_alpha(),
                # before calling tick() -- this module only watches for the
                # completion condition, it does not compute the value itself
                # while this flag is set (see set_coupling_alpha()'s own
                # docstring for why this is the single authoritative value,
                # not a second hidden path).
                if self.coupling_ramp_alpha >= 1.0:
                    self._transition(ACTIVE)
                return
            elapsed = now - self._phase_start_t
            self.coupling_ramp_alpha = self._ramp_value(elapsed, self.cfg.ramp_duration_s, +1)
            if self.coupling_ramp_alpha >= 1.0:
                self._transition(ACTIVE)
            return

        if self.state == ACTIVE:
            self.gravity_ramp_alpha = 1.0
            self.coupling_ramp_alpha = 1.0
            if self._disarmed_requested:
                self._disarmed_requested = False
                self._begin_phase(COUPLING_RAMP_DOWN)
            return

        # ---- DISARM sequence: COUPLING_RAMP_DOWN -> GRAVITY_RAMP_DOWN ->
        #      READY. Gravity stays at 1.0 through COUPLING_RAMP_DOWN and its
        #      settle window -- this is what removes the disarm-side
        #      interaction (see the module docstring). ----

        if self.state == COUPLING_RAMP_DOWN:
            elapsed = now - self._phase_start_t
            self.coupling_ramp_alpha = self._ramp_value(elapsed, self.cfg.coupling_ramp_down_duration_s, -1)
            if self.coupling_ramp_alpha <= 0.0:
                self._begin_phase(GRAVITY_RAMP_DOWN)
                # SELF_FOLLOW_CONFIRMED is represented as the first
                # gravity_settle_duration_s-scale portion of GRAVITY_RAMP_DOWN
                # rather than a separate state, to avoid a state whose only
                # job is waiting -- diagnostics can distinguish it via
                # self_follow_settle_elapsed() below if needed. See Part 1's
                # "or an equivalent design if a smaller state representation
                # is cleaner" allowance.
                self._self_follow_settle_start_t = now
            return

        if self.state == GRAVITY_RAMP_DOWN:
            self.coupling_ramp_alpha = 0.0
            settle_elapsed = now - getattr(self, "_self_follow_settle_start_t", now)
            if settle_elapsed < self.cfg.self_follow_settle_duration_s:
                self.gravity_ramp_alpha = 1.0  # SELF_FOLLOW_CONFIRMED sub-phase: gravity still full
                return
            elapsed = now - self._phase_start_t - self.cfg.self_follow_settle_duration_s
            self.gravity_ramp_alpha = self._ramp_value(
                max(0.0, elapsed), self.cfg.gravity_ramp_down_duration_s, -1)
            if self.gravity_ramp_alpha <= 0.0:
                self._transition(READY)
            return

    # ---- output: blend the bridge's already-computed target -----------

    def blend_target(self, full_target: List[float]) -> List[float]:
        """Blends the slave's target between its own live pose (zero
        relative motion, zero-force-safe -- see the 2026-09-09 unarmed-
        free-motion fix) and the full C2/C3-computed target, by
        coupling_ramp_alpha ONLY -- gravity_ramp_alpha never enters this
        formula (Part 1's explicit separation requirement).

        Using the CONTINUOUSLY LIVE pose as the blend's zero-coupling
        anchor (rather than a one-time snapshot captured at the start of
        COUPLING_RAMP_UP) is a strictly stronger version of Part 2's "capture
        fresh master and slave poses AGAIN at the start of this phase": at
        alpha=0 this is ALWAYS exactly the live pose, at every tick, not just
        the first one, so there is no way for a captured-but-then-stale
        snapshot to exist -- see test_coupling_ramp_up_uses_continuously_live_origin.
        At alpha=1 (ACTIVE) this is full_target unchanged, byte for byte,
        the pre-existing formula, exactly as before staging was introduced."""
        a = self.coupling_ramp_alpha
        if self._latest_slave is not None:
            live = self._latest_slave.pos
        elif self.right_anchor is not None:
            live = self.right_anchor
        else:
            return full_target
        return [live[i] + a * (full_target[i] - live[i]) for i in range(3)]

    def blend_left_target(self, full_target: List[float]) -> List[float]:
        """C2's counterpart to blend_target() -- same coupling_ramp_alpha-only
        blend, mirrored onto the master/left side."""
        a = self.coupling_ramp_alpha
        if self._latest_master is not None:
            live = self._latest_master.pos
        elif self.left_anchor is not None:
            live = self.left_anchor
        else:
            return full_target
        return [live[i] + a * (full_target[i] - live[i]) for i in range(3)]

    # ---- diagnostics ----------------------------------------------------

    def diagnostics(self) -> Dict:
        return {
            "startup_state": self.state,
            "fault_reason": self._fault_reason,
            "master_pose_age_s": (self._now() - self._latest_master.t) if self._latest_master else None,
            "slave_pose_age_s": (self._now() - self._latest_slave.t) if self._latest_slave else None,
            "master_hold_motion_m": _peak_to_peak_norm(self._hold_master_samples) if self._hold_master_samples else None,
            "slave_hold_motion_m": _peak_to_peak_norm(self._hold_slave_samples) if self._hold_slave_samples else None,
            "left_anchor_xyz": self.left_anchor,
            "right_anchor_xyz": self.right_anchor,
            "q_left_anchor": self.q_left_anchor,
            "q_right_anchor": self.q_right_anchor,
            "initial_alignment_error_xyz": self.initial_alignment_error_xyz,
            "initial_alignment_error_norm": self.initial_alignment_error_norm,
            "gravity_ramp_alpha": self.gravity_ramp_alpha,
            "coupling_ramp_alpha": self.coupling_ramp_alpha,
            "ramp_alpha": self.ramp_alpha,
            "ready": self.ready,
            "last_transitions": self._transitions[-5:],
            "gravity_settle_quiet_since_t": self._quiet_since_t,
            "gravity_settle_master_speed_mps": self.master_settle_velocity_norm_mps,
            "gravity_settle_slave_speed_mps": self.slave_settle_velocity_norm_mps,
            "gravity_settle_master_pose_drift_m": self.master_settle_pose_drift_m,
            "gravity_settle_slave_pose_drift_m": self.slave_settle_pose_drift_m,
            "gravity_settle_quiet_window_accumulated_s": self.settle_quiet_window_accumulated_s,
            "gravity_settle_accepted_master_pose_drift_m": self.last_accepted_settle_master_pose_drift_m,
            "gravity_settle_accepted_slave_pose_drift_m": self.last_accepted_settle_slave_pose_drift_m,
        }


def _peak_to_peak_norm(samples: List[_PoseSample]) -> float:
    if not samples:
        return 0.0
    pp = [0.0, 0.0, 0.0]
    for axis in range(3):
        vals = [s.pos[axis] for s in samples]
        pp[axis] = max(vals) - min(vals)
    return sum(v * v for v in pp) ** 0.5


def _mean_pos(samples: List[_PoseSample]) -> List[float]:
    n = len(samples)
    return [sum(s.pos[axis] for s in samples) / n for axis in range(3)]


def _norm3(v: List[float]) -> float:
    return sum(x * x for x in v) ** 0.5
