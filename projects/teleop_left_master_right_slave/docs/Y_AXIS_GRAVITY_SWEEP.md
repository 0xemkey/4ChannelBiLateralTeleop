# Y-axis gravity-comp magnitude sweep (Phases 2-5)

Date: 2026-09-08
Session: `y_axis_gravsweep_20260908`
Config held constant throughout: C3=ON, C1=C2=C4=OFF, stiffness=35.0 N/m,
damping=0.20 Ns/m, max_force_n=1.5N, workspace limits unchanged, startup
alignment active. Only `SLAVE_GRAVITY_COMP_N` varied (`SLAVE_GRAVITY_AXIS=1`
fixed).

## Runs

6 values tested: 0.40, 0.50, 0.55, 0.60, 0.65, 0.70 N. Each is one ~25s
freehand slow up/down Y-only capture (`log_phase2_scenario.py
--scenario single_axis_slow --axis y`), same operator, same style, timing
synced live (operator sent "go" the instant they started moving — no blind
countdown this time, unlike the first Stage-1 attempt). All 6 runs: **0.0%
travel-limit, 0.0% force-saturation flag** — none rejected for clipping,
all usable per the task's acceptance rule. Cross-axis isolation reasonable
throughout (RMS(X)/RMS(Y) 0.04-0.13, RMS(Z)/RMS(Y) 0.28-0.55).

## Phase 3 — metrics table

| G (N) | RMSE (mm) | MAE (mm) | mean signed (mm) | peak (mm) | p95 (mm) | lag (ms) | up RMSE (mm) | down RMSE (mm) | up/down ratio | X/Y | Z/Y | clamp % |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.40 | 16.31 | 14.21 | 13.82 | 31.13 | 24.64 | 190 | 20.32 | 8.98 | 2.26 | 0.083 | 0.430 | 0.0 |
| 0.50 | 14.65 | 11.77 | 9.90 | 27.39 | 24.30 | 145 | 19.54 | 7.82 | 2.50 | 0.128 | 0.345 | 0.0 |
| 0.55 | 12.87 | 10.69 | 9.03 | 25.32 | 21.50 | 170 | 16.57 | 6.60 | 2.51 | 0.085 | 0.432 | 0.0 |
| 0.60 | 12.35 | 10.92 | 6.19 | 24.25 | 20.33 | 145 | 15.36 | 8.40 | 1.83 | 0.072 | 0.343 | 0.0 |
| 0.65 | 12.25 | 10.78 | 4.18 | 22.59 | 19.66 | 140 | 15.24 | 8.74 | 1.74 | 0.042 | 0.277 | 0.0 |
| **0.70** | **11.67** | **10.51** | **2.48** | 20.53 | 18.54 | 140 | 13.76 | 9.31 | 1.48 | 0.110 | 0.348 | 0.0 |

Plot: `y_axis_gravity_sweep_plots/sweep_metrics_vs_gravity.png`

Position-error-clamp (±40mm) was never engaged in any sweep run (all much
smaller amplitude than Stage 1's runs) — travel-limit and force-saturation
were likewise 0.0% across the board.

## Phase 4 — best candidate, with an important caveat

**0.70N is the best of the 6 tested values** on every primary/secondary
criterion: lowest RMSE, mean signed error closest to zero, lowest up/down
asymmetry ratio, and it satisfies the saturation/travel-limit constraints
(both ≈0%).

**However: none of RMSE, mean signed error, or the asymmetry ratio has
turned over within the tested range — all are still monotonically
improving at 0.70N.** This sweep does not bracket a true minimum; 0.70N is
a lower bound on the optimum, not a confirmed one. Linear extrapolation of
the mean-signed-error trend (≈ -37.8 mm/N over 0.40→0.70N) would cross zero
around **≈0.77N** — offered as a rough estimate for where to look next, not
a validated value.

**One genuine early warning sign of an eventual turnover**: downward
(gravity-assisted) RMSE is not monotonic — it improves from 0.40N to a
minimum at 0.55N (6.60mm), then **gets worse** from 0.60N through 0.70N
(8.40 → 9.31mm) as the growing upward bias starts working against
downward-direction tracking. Total RMSE keeps improving through 0.70N only
because the upward-direction gains are currently larger than the
downward-direction losses — this margin will shrink and eventually reverse
as compensation increases further. This is the expected shape of a true
optimum starting to appear, just not yet reached in this sweep.

**Force-budget constraint**: 0.70N already commits 47% of the 1.5N
Cartesian force clamp to a static offset before any spring/damping force is
applied. The task's stop condition prohibits raising `max_force_n` to
compensate, so there is a real, and not too distant, ceiling on how far
this approach can be pushed independent of the tracking-error trend.

## Phase 5 — is a constant sufficient, or is this pose-dependent?

Plots: `y_error_vs_pose_all_gravity_values.png` (all 6 values overlaid),
`y_error_vs_pose_best_value.png` (0.70N only).

Error plotted against slave Y position, master shoulder angle, and master
elbow angle all show the **same hysteresis-loop shape** — a closed loop
traced out over each up/down cycle, not a single-valued curve. This is the
classic signature of a **velocity/dynamic-lag-dominated residual**
(controller trailing a moving target, tracing out different error at the
same position depending on direction of travel) rather than a **static,
pose-dependent gravity-load residual** (which would show as a shift in the
loop's mean level as a function of position/angle, not a loop shape).

Critically, overlaying all 6 gravity values shows each one **uniformly
translates the same loop shape downward** as compensation increases,
without reshaping it. That is exactly what a well-functioning *constant*
compensation should do to a *non-pose-dependent* load, and argues against
jumping to a pose-dependent model yet — the residual's shape is not yet
showing a signature that a constant term can't address.

**Conclusion: constant-magnitude compensation has not yet been shown
insufficient.** The evidence here does not support moving to pose-dependent
gravity feedforward (Option B) as the next step. The dominant residual
mechanism visible in this data is dynamic/lag-related (85-190ms estimated
lag across runs), which per the task's own priority order is explicitly
deferred until after magnitude, stiction, and pose-dependence are settled
— not something to fix now.

## What's not yet done (Phases 6-7)

- **Phase 6 (stiction characterization)** — not run this session. Requires
  a separate, small, low-risk hardware test (very small up/down commands
  to find breakaway force) — a new hardware round, not covered by this
  sweep's freehand motion.
- **Phase 7 (anchor gap)** — not evaluated. Requires comparing calibration
  data across sessions or a targeted test; not attempted here.

## Recommended next action

1. **Extend the magnitude sweep modestly** (e.g. 0.75N, 0.80N, 0.85N) to
   actually bracket the turnover point suggested by the downward-RMSE
   uptick — this is still Rank 1 (lowest risk, direct continuation of what
   already worked) and should come before Phases 6/7.
2. Then run Phase 6 (stiction breakaway characterization) to determine how
   much of the *remaining* residual (whatever it is once magnitude is
   properly optimized) is stiction vs. lag vs. still-unexplained.
3. Phase 7 (anchor gap) last, only with direct evidence.
4. Do not proceed to pose-dependent gravity feedforward (Option B) unless
   a future error-vs-pose check (after 1-2 above) starts showing the loop
   *shape itself* changing with gravity comp, rather than just shifting.

## Status

**MORE_DATA_REQUIRED** — a clear direction is established (constant-magnitude
compensation, currently undersized, no pose-dependence signature yet) but
the sweep has not been extended far enough to select a final validated
constant, and Phases 6-7 have not been run.
