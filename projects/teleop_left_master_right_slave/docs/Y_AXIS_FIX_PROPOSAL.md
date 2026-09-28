# Y-axis fix proposal (Phase 1)

Date: 2026-09-08
Based on: `docs/Y_AXIS_DIAGNOSIS.md` (real-hardware gravity ON/OFF A/B test, session `y_axis_diag_20260908`)

This is a proposal only — no controller code or launch defaults are changed
in this document.

## 1. Why gravity OFF fails

The Touch device's vertical (Y) linkage carries a real, constant downward
load from its own weight/linkage geometry. With `SLAVE_GRAVITY_COMP_N=0`,
the only force available to lift against that load is the position-error
spring term (`stiffness_n_per_m × error`, 35 N/m). Two compounding effects
follow:

- **Stiction gate**: a small commanded error produces a small spring force.
  If that force is below the linkage's static-friction breakaway threshold,
  nothing moves — this is exactly what the first 1.5s of the OFF run showed
  (slave Y frozen at 0.000mm variance while the target was already moving).
  Error has to build up past the breakaway point before any motion starts.
- **Force-budget starvation**: once moving, a large fraction of the 1.5N
  Cartesian force clamp is spent just holding/lifting against gravity,
  leaving little authority for actually closing the tracking error — this
  is why the OFF run's reconstructed spring force sat pinned at the 1.5N
  clamp for extended plateaus during upward strokes (see
  `y_controller_decomposition.png`), and why upward (against-gravity)
  excursions were severely under-reached while downward (gravity-assisted)
  excursions tracked fine.

## 2. Why 0.55N helps

A constant 0.55N upward bias offsets a substantial fraction of the static
load before the spring/damping terms are even evaluated. This:
- lowers the effective breakaway threshold the position error needs to
  overcome (less stiction gating),
- frees up force-budget headroom under the 1.5N clamp for actual tracking
  dynamics rather than static holding.
This is consistent with the measured result: ON vs OFF nearly triples mean
signed error (+8.3mm → +23.9mm) and removes the outright motion-failure
window.

## 3. Why residual bias remains (even with 0.55N ON)

Two established facts point at **undersizing**, not a wrong concept:
- The historical tuning note in `run_bridge.sh` (2026-08-26 sweep) found
  0.55N brought Y reach ratio from 0.69 to 0.85 — not to 1.0. It was never
  claimed to fully cancel the load, only to help.
- The gravity/weight load on a linkage like this is generally **pose-
  dependent** (moment arms change with shoulder/elbow angle), so any single
  constant can only be correct at one operating point/pose region and will
  under- or over-compensate elsewhere. A residual bias that doesn't vanish
  with a well-chosen constant is the expected signature of this, not a bug.

Stiction (a velocity-independent, direction-dependent threshold effect) is
a plausible secondary contributor to the residual error's shape (e.g. any
remaining small-error "stuck" periods), but this has not yet been isolated
from a simply-undersized constant — that is what Phase 2/6 are for.

## 4. Candidate fixes — ranked

| Rank | Fix | Expected impact | Implementation risk | Rationale |
|---|---|---|---|---|
| 1 | **A. Retune constant gravity magnitude** | High | Low | Directly targets the demonstrated undersizing; pure parameter sweep, no code changes, fully reversible, matches the historical sweep's own methodology. Do this first. |
| 2 | **C. Stiction-specific compensation** | Medium | Medium (if bounded) | Only meaningful once A is optimized — some of what looks like stiction may just be gravity-load underestimate at low error. Needs explicit breakaway-force characterization (Phase 6) before designing anything, and should stay small/bounded/direction-aware if implemented — unbounded breakaway boosts are a real safety concern on a device a person is holding. |
| 3 | **B. Pose-dependent gravity compensation** `F_g(q) = J(q)^{-T} τ_g(q)` | Potentially highest ceiling | High | Only justified if A (+ C) leave a residual that clearly varies with pose (Phase 5's error-vs-pose check). Higher complexity, more failure modes, and must NOT reuse the dormant joint-space DOB/gravity path directly inside the live Cartesian controller (that path has not been validated for this use and mixing it in changes the production control law materially) — needs a clean, explicitly-designed Cartesian feedforward interface instead. |
| 4 | **D. Anchor/workspace geometry correction** | Low-moderate, unconfirmed | Medium (touches calibration, which other channels/scripts depend on) | No evidence yet isolates its contribution — the ~13mm gap has not been separated from the gravity-load effect in any test so far. Evaluate last (Phase 7), and only change calibration if direct evidence shows it's a real control error, not just an absorbed base-frame offset. |
| — | **E. Combination** | — | — | Expected end state: A now, then C only if Phase 6 shows stiction is a real residual contributor after A, then B only if Phase 5 shows genuine pose-dependence survives A+C, D only with direct Phase 7 evidence. This matches the task's own Phase 8 priority order and is the plan this proposal follows. |

## Next step

Proceed to Phase 2 (constant-magnitude sweep: 0.40/0.50/0.55/0.60/0.65/0.70 N)
per this ranking's #1 recommendation. This requires 6 more hardware capture
sessions with a human at the rig — see coordination note in the accompanying
message before starting.
