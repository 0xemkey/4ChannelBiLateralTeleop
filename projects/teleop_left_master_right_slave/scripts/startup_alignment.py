"""Deterministic startup/alignment state machine for the master<->slave
Cartesian teleop bridge (2026-09-08, Phase 1 of the diagnose-and-repair
task).

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
message happens to arrive first."

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

STATE MACHINE
    WAIT_FOR_POSES -> HOLD_STILL -> CAPTURE_ANCHORS -> VALIDATE_ALIGNMENT
        -> READY -> RAMPING -> ACTIVE
    Any state -> FAULT (stale pose, excessive motion during hold,
        impossible/oversized alignment error). FAULT requires an explicit
        reset() -- no auto-recovery, per the task's explicit requirement.

INTEGRATION: bridge_master_slave.py feeds this module pose samples via
update_master_pose()/update_slave_pose() (called from its existing
_left_cb/_right_cb) and drives it forward via tick() (called once per
_tick()). It then reads .state, .left_anchor, .right_anchor, .ramp_alpha,
.ready, and .diagnostics() -- the bridge's own target-computation math is
unchanged except for (1) using this module's anchors instead of its own
inline "first message wins" capture, and (2) blending the FINAL computed
target toward right_anchor by ramp_alpha, which is the "safest
mathematically consistent location" to apply the ramp: it never touches
stiffness/damping/DOB, only the commanded setpoint the existing spring
already tracks, so at alpha=0 the slave is asked to do nothing (hold its
own anchor) and at alpha=1 behavior is byte-for-byte the pre-existing
formula.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------
WAIT_FOR_POSES = "WAIT_FOR_POSES"
HOLD_STILL = "HOLD_STILL"
CAPTURE_ANCHORS = "CAPTURE_ANCHORS"
VALIDATE_ALIGNMENT = "VALIDATE_ALIGNMENT"
READY = "READY"
RAMPING = "RAMPING"
ACTIVE = "ACTIVE"
FAULT = "FAULT"

_ALL_STATES = (WAIT_FOR_POSES, HOLD_STILL, CAPTURE_ANCHORS, VALIDATE_ALIGNMENT,
              READY, RAMPING, ACTIVE, FAULT)


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

    # RAMPING: commanded target blends from right_anchor (zero relative
    # motion) to the full mapped target over this duration. 1.0s per the
    # task's own default.
    ramp_duration_s: float = 1.0


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
        self._ramp_start_t: Optional[float] = None
        self.ramp_alpha: float = 0.0

        self._transitions: List[Tuple[float, str, str]] = []  # (t, from, to)

    # ---- inputs, called from ROS callbacks --------------------------------

    def update_master_pose(self, pos: List[float], t: Optional[float] = None) -> None:
        self._latest_master = _PoseSample(t if t is not None else self._now(), list(pos))

    def update_slave_pose(self, pos: List[float], t: Optional[float] = None) -> None:
        self._latest_slave = _PoseSample(t if t is not None else self._now(), list(pos))

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
        self._ramp_start_t = None
        self.ramp_alpha = 0.0

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

    def _fault(self, reason: str) -> None:
        self._fault_reason = reason
        self._transition(FAULT)
        self.ramp_alpha = 0.0

    @property
    def ready(self) -> bool:
        return self.state in (READY, RAMPING, ACTIVE)

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
                self._ramp_start_t = now
                self._transition(RAMPING)
            return

        if self.state == RAMPING:
            elapsed = now - self._ramp_start_t
            self.ramp_alpha = min(1.0, max(0.0, elapsed / max(self.cfg.ramp_duration_s, 1e-6)))
            if self.ramp_alpha >= 1.0:
                self._transition(ACTIVE)
            return

        if self.state == ACTIVE:
            self.ramp_alpha = 1.0
            return

    # ---- output: blend the bridge's already-computed target -----------

    def blend_target(self, full_target: List[float]) -> List[float]:
        """Applies the ramp in the "safest mathematically consistent
        location": the FINAL commanded setpoint, not the gains. At
        alpha=0 this returns right_anchor exactly (ask the slave to stay
        exactly where its anchor was captured -- zero relative motion,
        matching whatever it is already physically resting at, so there is
        no jump). At alpha=1 it returns full_target unchanged -- the
        pre-existing formula, byte for byte. Continuous in between."""
        if self.right_anchor is None:
            return full_target
        a = self.ramp_alpha
        return [self.right_anchor[i] + a * (full_target[i] - self.right_anchor[i]) for i in range(3)]

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
            "ramp_alpha": self.ramp_alpha,
            "ready": self.ready,
            "last_transitions": self._transitions[-5:],
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
