#!/usr/bin/env python3
"""Force-budget-aware bilateral coupling ramp (2026-09-09, force-aware
transition task). Pure Python, no rclpy/ROS dependency -- matches this
project's established "framework-free where the logic itself has no ROS
dependency" convention (startup_alignment.py, bilateral_4ch.py's
analyze_window()).

WHY THIS EXISTS: the first staged-transition design (gravity ramps fully
BEFORE coupling begins) made real-hardware-scale saturation WORSE, not
better, in simulation -- see compare_old_vs_staged_transition.py's honest
result (NEW clamped at 1.5N where OLD didn't). Root cause: with
max_force_n=1.5N and gravity fully deployed at 0.70N throughout the ENTIRE
coupling ramp, the remaining force authority for position tracking is
bounded (roughly, but NOT exactly -- see predict_force_vector()'s real
vector-norm computation, not a scalar Fmax-|G| intuition) far tighter than
in the old simultaneous-ramp architecture, where gravity was often still
only partially deployed while spring demand was building. A FIXED-TIME
coupling ramp has no way to know it is about to demand more force than the
remaining budget allows.

THIS MODULE: replaces "alpha = elapsed / fixed_duration" with an adaptive
advancer that, every tick, proposes a candidate alpha increment, PREDICTS
the resulting controller force vector (spring + damping + aux, the same
decomposition touch_slave.cpp's real force_ros computes and the real
worst-ramp-sample from this session's root-cause CSV analysis confirmed),
and only commits as much of that increment as keeps the predicted force
inside (max_force_n - force_reserve_n). Never decreases alpha during a
normal ramp. Watches for a genuine stall (no progress for stall_timeout_s)
and reports it rather than holding silently forever.

Per Part 12 of the task this was written for: this module does NOT embed
an assumed plant/mass model into anything a real controller would use --
predict_force_vector() only uses the SAME quantities the real controller
already has (measured pose, a velocity estimate, configured stiffness/
damping/aux) to predict what ITS OWN force_ros computation would produce
for a candidate target, not a simulated physical response. The simulation
plant in compare_old_vs_staged_transition.py is a SEPARATE, clearly-labeled
evaluation tool, never used here.

TWO ORIGIN MODES, per the task's explicit "do not confuse these, test both"
requirement:
    MOVING_LIVE_ORIGIN: origin(t) = live pose at this instant (what
        startup_alignment.py's blend_target()/blend_left_target() already
        do for the non-adaptive path -- continuously fresh, by construction).
    SNAPSHOT_ORIGIN: origin = pose captured ONCE, at the moment
        COUPLING_RAMP_UP begins (the originally-requested design in the
        very first staged-transition task, before MOVING_LIVE_ORIGIN was
        substituted as "a strictly stronger version").
Neither is asserted superior in advance -- see
compare_arm_strategies.py for the measured comparison.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class ForceBudgetConfig:
    max_force_n: float = 1.5          # matches the real slave's max_force_n -- NOT changed by this task
    force_reserve_n: float = 0.10     # configurable safety margin below max_force_n; NOT asserted optimal
    nominal_duration_s: float = 1.0   # the ramp duration used WHEN the force budget allows full-rate progress
    min_alpha_step_fraction: float = 1.0 / 64  # bisection floor, as a fraction of the nominal per-tick step
    stall_timeout_s: float = 3.0      # no progress for this long -> COUPLING_RAMP_STALLED_FORCE_BUDGET
    stall_min_progress: float = 1e-6  # alpha must advance at least this much to reset the stall timer


def predict_force_vector(pose: List[float], velocity: List[float], candidate_target: List[float],
                         stiffness_n_per_m: float, damping_n_s_per_m: float,
                         aux_vec: List[float], max_position_error_m: Optional[float] = None) -> List[float]:
    """The SAME decomposition touch_slave.cpp's real force_ros computes
    (spring_force + damping_force + auxiliary_force, BEFORE the primary
    clamp) -- see this session's root-cause worst-sample analysis, which
    confirmed this formula against real telemetry to 3+ significant figures.

    2026-09-09 correction (Part 3 controller-predictor audit): touch_slave.
    cpp clamps the position error PER-AXIS to +/-max_position_error_m
    BEFORE multiplying by stiffness (see its own control loop: `v =
    clampScalar(v, -max_position_error_m_, max_position_error_m_)` runs on
    `err` before `spring_force[i] = stiffness_n_per_m_ * err[i]`). Without
    this clamp, this function overestimates spring force whenever the raw
    error exceeds that limit -- exactly what the real master worst-ramp-
    sample showed: raw error 44.5mm, clamped to the configured 30mm,
    spring_y = 15 N/m * 0.03m = -0.45N EXACTLY (not 15*0.0445=0.6675N,
    which the uncorrected version of this function would have predicted).
    max_position_error_m is optional (None = no clamp, the old behavior)
    only because some callers may not have it available; production
    callers MUST pass the real configured value."""
    err = [candidate_target[i] - pose[i] for i in range(3)]
    if max_position_error_m is not None:
        err = [max(-max_position_error_m, min(max_position_error_m, e)) for e in err]
    spring = [stiffness_n_per_m * e for e in err]
    damp = [-damping_n_s_per_m * velocity[i] for i in range(3)]
    return [spring[i] + damp[i] + aux_vec[i] for i in range(3)]


def _norm(v: List[float]) -> float:
    return math.sqrt(sum(x * x for x in v))


class CouplingRampStalled(Exception):
    """Raised by AdaptiveCouplingRamp.step() when the stall watchdog trips
    -- COUPLING_RAMP_STALLED_FORCE_BUDGET. The caller (bridge/state machine)
    decides what to do with this (FAULT, warning, abort-to-READY); this
    module does not silently keep trying forever, per Part 4's explicit
    requirement."""
    def __init__(self, alpha: float, elapsed_since_progress: float):
        self.alpha = alpha
        self.elapsed_since_progress = elapsed_since_progress
        super().__init__(
            f"COUPLING_RAMP_STALLED_FORCE_BUDGET: alpha stuck at {alpha:.4f} for "
            f"{elapsed_since_progress:.2f}s with no force-budget-safe progress available"
        )


@dataclass
class ArmState:
    """Bundles one arm's (master's or slave's) inputs for a bilateral
    force-budget check (Part 1, 2026-09-09: 'a candidate coupling-alpha
    increment is acceptable ONLY if BOTH arms remain inside their safe
    predicted control envelopes' -- the real failed trial had slave force
    saturation AND a master position-error clamp, so constraining only one
    side is insufficient, exactly the gap this closes)."""
    live_pose: List[float]
    velocity: List[float]
    full_target: List[float]
    stiffness_n_per_m: float
    damping_n_s_per_m: float
    aux_vec: List[float]
    max_force_n: float
    max_position_error_m: float
    origin: Optional[List[float]] = None  # set by AdaptiveCouplingRamp._origin() per arm


class AdaptiveCouplingRamp:
    """Force-budget-aware coupling_ramp_alpha advancer. Pure Python, no ROS.

    Usage (mirrors how bridge_master_slave.py already owns stiffness/
    damping/max_force/pose -- this class is handed those each tick, it does
    not read them from anywhere itself):

        ramp = AdaptiveCouplingRamp(ForceBudgetConfig(), origin_mode="moving")
        ramp.start(now=t0, origin_pose=slave_live_pose)
        ...
        alpha = ramp.step(now=t, dt=0.01, live_pose=..., velocity=...,
                          full_target=..., stiffness_n_per_m=35.0,
                          damping_n_s_per_m=0.20, aux_vec=[0,0.70,0])
        # raises CouplingRampStalled if the watchdog trips
    """

    def __init__(self, config: Optional[ForceBudgetConfig] = None, origin_mode: str = "moving"):
        assert origin_mode in ("moving", "snapshot"), f"unknown origin_mode {origin_mode!r}"
        self.cfg = config or ForceBudgetConfig()
        self.origin_mode = origin_mode
        self.alpha = 0.0
        self._snapshot_origin: Optional[List[float]] = None
        self._last_progress_t: Optional[float] = None
        self.stalled = False
        # Diagnostics: last computed force margin, for external reporting
        # (Part 15/production-diagnostics: "force_budget_margin if implemented").
        self.last_force_margin_n: Optional[float] = None
        self.last_held: bool = False
        # Bilateral diagnostics -- initialised here as well as in
        # start_bilateral() because callers publish them every tick from
        # process start, i.e. long before COUPLING_RAMP_UP is ever entered.
        self.last_master_force_margin_n: Optional[float] = None
        self.last_slave_force_margin_n: Optional[float] = None
        self.last_master_position_margin_m: Optional[float] = None
        self.last_slave_position_margin_m: Optional[float] = None
        self.last_master_predicted_force_n: Optional[List[float]] = None
        self.last_slave_predicted_force_n: Optional[List[float]] = None
        # 2026-09-09 Part 1 audit fix: step_bilateral() used to publish a
        # single last_{master,slave}_{force,position}_margin_n/m pair that
        # meant DIFFERENT things depending on the tick outcome -- on a
        # progress tick it was the margin of the newly-ACCEPTED (now
        # published) target; on a HELD tick it was the margin of the
        # smallest REJECTED bisection candidate (the last one tried before
        # giving up), which can legitimately be negative (that is WHY it was
        # rejected) without the actual published target (unchanged this
        # tick, at the OLD alpha) ever being unsafe. Conflating these made a
        # perfectly correct reject-and-hold decision look like a safety
        # violation in diagnostics. These are now two explicit, always-
        # separately-populated quantities:
        #   accepted_*  -- margin of the CURRENTLY PUBLISHED target (this
        #       tick's self.alpha, recomputed fresh every tick against the
        #       live pose/velocity -- so it tracks plant motion even while
        #       alpha is held) -- THIS is what any hard safety assertion
        #       must check.
        #   candidate_* -- margin of the last bisection trial attempted this
        #       tick (accepted if progress happened, otherwise the smallest
        #       rejected step) -- diagnostic-only, explains WHY alpha did or
        #       did not advance, never a safety signal by itself.
        self.last_master_accepted_force_margin_n: Optional[float] = None
        self.last_slave_accepted_force_margin_n: Optional[float] = None
        self.last_master_accepted_position_margin_m: Optional[float] = None
        self.last_slave_accepted_position_margin_m: Optional[float] = None
        self.last_master_candidate_force_margin_n: Optional[float] = None
        self.last_slave_candidate_force_margin_n: Optional[float] = None
        self.last_master_candidate_position_margin_m: Optional[float] = None
        self.last_slave_candidate_position_margin_m: Optional[float] = None

    def start(self, now: float, origin_pose: List[float]) -> None:
        self.alpha = 0.0
        self._snapshot_origin = list(origin_pose)
        self._last_progress_t = now
        self.stalled = False
        self.last_force_margin_n = None
        self.last_held = False

    def _origin(self, live_pose: List[float]) -> List[float]:
        if self.origin_mode == "snapshot":
            assert self._snapshot_origin is not None, "start() must be called before step()"
            return self._snapshot_origin
        return live_pose

    @property
    def origin_pose(self) -> Optional[List[float]]:
        """Public accessor for the current effective origin (single-arm
        step() usage only) -- 2026-09-09 Part 6: production/caller code
        must not reach into the private _snapshot_origin attribute
        directly. Returns None for MOVING_LIVE_ORIGIN (there is no fixed
        origin to report; callers wanting the live pose already have it)."""
        return list(self._snapshot_origin) if self.origin_mode == "snapshot" and self._snapshot_origin else None

    def current_target_bilateral(self, which: str, live_pose: List[float], full_target: List[float]) -> List[float]:
        """Public accessor mirroring current_target(), for step_bilateral()
        usage -- which is 'master' or 'slave', selecting the matching
        snapshot origin captured by start_bilateral()."""
        assert which in ("master", "slave")
        snapshot = self._snapshot_origin_master if which == "master" else self._snapshot_origin_slave
        origin = snapshot if self.origin_mode == "snapshot" else live_pose
        return [origin[i] + self.alpha * (full_target[i] - origin[i]) for i in range(3)]

    def stall_elapsed_s(self, now: float) -> float:
        """Public accessor for diagnostics publishing (Part 6:
        coupling_ramp_stall_time_s) -- elapsed time since the last accepted
        alpha progress, without reaching into _last_progress_t directly."""
        if self._last_progress_t is None:
            return 0.0
        return now - self._last_progress_t

    def current_target(self, live_pose: List[float], full_target: List[float]) -> List[float]:
        """Public accessor for the target implied by the CURRENT alpha,
        given the live pose and full C2/C3 target -- callers should use
        this instead of recomputing origin(...)+alpha*(...) themselves
        against a private attribute (Part 6)."""
        origin = self._origin(live_pose)
        return [origin[i] + self.alpha * (full_target[i] - origin[i]) for i in range(3)]

    def step(self, now: float, dt: float, live_pose: List[float], velocity: List[float],
             full_target: List[float], stiffness_n_per_m: float, damping_n_s_per_m: float,
             aux_vec: List[float]) -> float:
        if self.alpha >= 1.0:
            self.last_held = False
            self.last_force_margin_n = self.cfg.max_force_n  # coupling fully engaged, nothing pending
            return self.alpha

        origin = self._origin(live_pose)
        nominal_increment = dt / max(self.cfg.nominal_duration_s, 1e-6)
        max_step = min(1.0, self.alpha + nominal_increment) - self.alpha
        min_step = max(0.0, nominal_increment * self.cfg.min_alpha_step_fraction)

        accepted_step = None
        accepted_margin = None
        step_try = max_step
        # Bounded bisection (Part 4's "reduce alpha increment" requirement):
        # try the full nominal step, then progressively smaller ones, until
        # one is safe or the step is negligibly small.
        while True:
            trial_alpha = self.alpha + step_try
            trial_target = [origin[i] + trial_alpha * (full_target[i] - origin[i]) for i in range(3)]
            f_pred = predict_force_vector(live_pose, velocity, trial_target,
                                          stiffness_n_per_m, damping_n_s_per_m, aux_vec)
            margin = (self.cfg.max_force_n - self.cfg.force_reserve_n) - _norm(f_pred)
            if margin >= 0.0 or step_try <= min_step:
                accepted_step = step_try if margin >= 0.0 else 0.0
                accepted_margin = margin
                break
            step_try /= 2.0

        self.last_force_margin_n = accepted_margin
        if accepted_step > 0.0:
            self.alpha = min(1.0, self.alpha + accepted_step)
            self._last_progress_t = now
            self.last_held = False
        else:
            self.last_held = True
            elapsed_since_progress = now - self._last_progress_t
            if elapsed_since_progress > self.cfg.stall_timeout_s:
                self.stalled = True
                raise CouplingRampStalled(self.alpha, elapsed_since_progress)

        return self.alpha

    # ---- bilateral (Part 1, 2026-09-09): both arms must accept a candidate
    # alpha, not just one. Kept as a separate entry point from step() rather
    # than folding a variable-arity arm list into step() itself, so the
    # single-arm API (and every test/caller already built against it) stays
    # unchanged. ----

    def start_bilateral(self, now: float, master_origin_pose: List[float],
                        slave_origin_pose: List[float]) -> None:
        self.alpha = 0.0
        self._snapshot_origin_master = list(master_origin_pose)
        self._snapshot_origin_slave = list(slave_origin_pose)
        self._last_progress_t = now
        self.stalled = False
        self.last_force_margin_n = None
        self.last_held = False
        # Per-arm diagnostics (Part 5's required publish fields).
        self.last_master_force_margin_n: Optional[float] = None
        self.last_slave_force_margin_n: Optional[float] = None
        self.last_master_position_margin_m: Optional[float] = None
        self.last_slave_position_margin_m: Optional[float] = None
        # Predicted pre-primary-clamp force for the accepted candidate, per
        # arm -- published in diagnostics so predictor-vs-controller
        # equivalence can be measured directly against each arm's own
        # CartesianControllerTelemetry.force_before_primary_clamp_n.
        self.last_master_predicted_force_n: Optional[List[float]] = None
        self.last_slave_predicted_force_n: Optional[List[float]] = None
        self.last_master_accepted_force_margin_n: Optional[float] = None
        self.last_slave_accepted_force_margin_n: Optional[float] = None
        self.last_master_accepted_position_margin_m: Optional[float] = None
        self.last_slave_accepted_position_margin_m: Optional[float] = None
        self.last_master_candidate_force_margin_n: Optional[float] = None
        self.last_slave_candidate_force_margin_n: Optional[float] = None
        self.last_master_candidate_position_margin_m: Optional[float] = None
        self.last_slave_candidate_position_margin_m: Optional[float] = None

    def _origin_for(self, arm: "ArmState", snapshot: List[float]) -> List[float]:
        if self.origin_mode == "snapshot":
            return snapshot
        return arm.live_pose

    def _arm_margins(self, arm: "ArmState", trial_target: List[float]) -> tuple:
        """Returns (force_margin_n, position_margin_m) for one arm at one
        candidate target -- position_margin_m is Part 2's hard planner
        constraint: max_position_error_m minus the RAW (pre-clamp) error
        norm, so a candidate that would make the controller's own per-axis
        clamp engage is rejected by the planner BEFORE it ever reaches the
        real controller, not just after force saturates."""
        f_pred = predict_force_vector(arm.live_pose, arm.velocity, trial_target,
                                      arm.stiffness_n_per_m, arm.damping_n_s_per_m, arm.aux_vec,
                                      max_position_error_m=arm.max_position_error_m)
        force_margin = (arm.max_force_n - self.cfg.force_reserve_n) - _norm(f_pred)
        raw_err = [trial_target[i] - arm.live_pose[i] for i in range(3)]
        # Per-axis, matching touch_slave.cpp's own clampScalar-per-axis
        # geometry exactly (Part 2's explicit "match the production
        # controller implementation... norm clamp vs per-axis clamp,
        # whichever is actually used" -- per-axis is what's actually used).
        position_margin = min(arm.max_position_error_m - abs(e) for e in raw_err)
        return force_margin, position_margin, f_pred

    def step_bilateral(self, now: float, dt: float, master: ArmState, slave: ArmState) -> float:
        if self.alpha >= 1.0:
            self.last_held = False
            return self.alpha

        master_origin = self._origin_for(master, self._snapshot_origin_master)
        slave_origin = self._origin_for(slave, self._snapshot_origin_slave)
        nominal_increment = dt / max(self.cfg.nominal_duration_s, 1e-6)
        max_step = min(1.0, self.alpha + nominal_increment) - self.alpha
        min_step = max(0.0, nominal_increment * self.cfg.min_alpha_step_fraction)

        accepted_step = None
        margins = None
        step_try = max_step
        while True:
            trial_alpha = self.alpha + step_try
            master_target = [master_origin[i] + trial_alpha * (master.full_target[i] - master_origin[i])
                             for i in range(3)]
            slave_target = [slave_origin[i] + trial_alpha * (slave.full_target[i] - slave_origin[i])
                            for i in range(3)]
            m_force_margin, m_pos_margin, m_pred = self._arm_margins(master, master_target)
            s_force_margin, s_pos_margin, s_pred = self._arm_margins(slave, slave_target)
            margins = (m_force_margin, s_force_margin, m_pos_margin, s_pos_margin)
            preds = (m_pred, s_pred)
            all_safe = all(m >= 0.0 for m in margins)
            if all_safe or step_try <= min_step:
                accepted_step = step_try if all_safe else 0.0
                break
            step_try /= 2.0

        # CANDIDATE margins: whatever the last bisection trial this tick
        # actually was (the accepted one on a progress tick, the smallest
        # REJECTED one on a held tick) -- diagnostic-only, see __init__'s
        # comment. Legacy last_{master,slave}_{force,position}_margin_*
        # names are kept as aliases of these (== old behavior) so nothing
        # that reads them silently breaks; new code should read
        # accepted_/candidate_ explicitly instead.
        (self.last_master_force_margin_n, self.last_slave_force_margin_n,
         self.last_master_position_margin_m, self.last_slave_position_margin_m) = margins
        (self.last_master_candidate_force_margin_n, self.last_slave_candidate_force_margin_n,
         self.last_master_candidate_position_margin_m, self.last_slave_candidate_position_margin_m) = margins
        self.last_master_predicted_force_n, self.last_slave_predicted_force_n = preds
        self.last_force_margin_n = min(self.last_master_force_margin_n, self.last_slave_force_margin_n)

        if accepted_step > 0.0:
            self.alpha = min(1.0, self.alpha + accepted_step)
            self._last_progress_t = now
            self.last_held = False
        else:
            self.last_held = True
            elapsed_since_progress = now - self._last_progress_t
            if elapsed_since_progress > self.cfg.stall_timeout_s:
                self.stalled = True
                raise CouplingRampStalled(self.alpha, elapsed_since_progress)

        # ACCEPTED margins: recomputed fresh at the CURRENTLY PUBLISHED
        # alpha (self.alpha, just updated above if progress happened, else
        # unchanged) against this tick's live pose/velocity -- this is the
        # quantity any hard safety assertion must check, since it is exactly
        # what current_target_bilateral() will publish this tick, not a
        # rejected what-if.
        final_master_target = [master_origin[i] + self.alpha * (master.full_target[i] - master_origin[i])
                               for i in range(3)]
        final_slave_target = [slave_origin[i] + self.alpha * (slave.full_target[i] - slave_origin[i])
                              for i in range(3)]
        m_acc_force, m_acc_pos, _ = self._arm_margins(master, final_master_target)
        s_acc_force, s_acc_pos, _ = self._arm_margins(slave, final_slave_target)
        self.last_master_accepted_force_margin_n = m_acc_force
        self.last_slave_accepted_force_margin_n = s_acc_force
        self.last_master_accepted_position_margin_m = m_acc_pos
        self.last_slave_accepted_position_margin_m = s_acc_pos

        return self.alpha
