# Y-axis gravity model: offline validation and hardware-test gate

Date: 2026-09-08
Scope: validate `gravity_right_minimal.json` against real hardware logs
before considering it for a hardware A/B against the empirical 0.70N
constant Y compensation (`docs/Y_AXIS_GRAVITY_SWEEP.md`).

**Result up front: the model does not pass the validity gate. No hardware
feedforward test was run. The 0.70N constant remains the baseline.**

## Phase 1 — model audit (code, no hardware)

**Equation** (`touch_dob.hpp:540-544`, `gravityTorqueNm`):
```
tau_g_j(q2, q3) = c0*cos(q2) + c1*sin(q2) + c2*cos(q3) + c3*sin(q3)
```
evaluated identically for each joint j ∈ {waist, shoulder, elbow}, each with
its own fitted coefficient vector `c`, using ONLY shoulder (q2) and elbow
(q3) angle — waist's own angle (q1) never enters any joint's gravity term.
Units: radians in, N·m out, no zero-offset applied (`touch_slave.cpp:882,
1011` feed raw `HD_CURRENT_JOINT_ANGLES` straight in).

**Coefficients** (`gravity_right_minimal.json`, right/slave-specific,
confirmed via `run_slave_right.sh:38`'s explicit right-arm default path):

| joint | c0 (cos q2) | c1 (sin q2) | c2 (cos q3) | c3 (sin q3) | R² |
|---|---:|---:|---:|---:|---:|
| waist | 0.0333 | 0.0315 | 0.0255 | -0.0687 | 0.396 |
| shoulder | 0.1212 | 0.0371 | -0.0059 | -0.0639 | 0.961 |
| elbow | 0.0743 | 0.0143 | -0.0235 | 0.0066 | 0.984 |

**Joint order**: 0=waist, 1=shoulder, 2=elbow, consistent everywhere from
JSON load through evaluation (`dob_json_loader.hpp:47-50`,
`touch_dob.hpp:200-216`) — no ordering bug found.

**Jacobian / Cartesian conversion**: closed-form 3×3 (`touch_dob.hpp:224-297`,
Jarillo-Silva et al. 2009 form, L1=L2=0.135m), solving `J^T F = tau_g` for
`F` via adjugate/determinant inverse — never a pseudo-inverse. Singularity
guarded by a hard runtime gate (`SingularityMetrics.near_singular`,
`kMinRadialMeters=0.02`, `kMinAlignCos=0.20`), not damped least squares.
`plot_phase2_scenario.py`'s `jacobian()`/`cartesian_force_from_joint_torque()`
are byte-identical ports of this same math — this analysis reuses them
directly rather than reimplementing.

**Fitted valid range**: the deployed 6-pose fit spans shoulder q2 ∈
**[-15.3°, +25.8°]** = [-0.267, 0.450] rad (`FOUR_CHANNEL_TODO.md:341`).

**Production status: confirmed dormant.** `joint_space_control=false` and
`dob_enable=false` are both the default for the right/slave device
(`run_slave_right.sh:31,37`), so `touch_slave.cpp:1011`'s evaluation of
this model sits inside a branch (`:976`) that never executes in the
production launch path. The model is loaded but never applied as live
feedforward — consistent with the earlier Y-axis code trace.

## Rig/frame audit (part A of the validity gate)

Read `identification/fit_gravity.py` in full. Key findings:

- **The model is not a physics-derived rigid-body model with an explicit
  gravity vector.** It's fitted empirically from breakaway torque pairs
  (`tau_plus`, `tau_minus` at each pose) — there is no `g=[0,0,-9.81]`
  parameter to inspect or correct.
- **The deployed basis structurally assumes a level base.** The docstring
  is explicit: "for a 2-link arm in a vertical plane... gravity torque... is
  NOT a function of q1 at all -- the waist rotates about the vertical axis,
  which cannot change potential energy. That q1 independence is a physical
  prediction" — true only when the base's vertical axis is actually
  vertical. `BASIS_MINIMAL` (the deployed basis) enforces exactly this
  assumption by construction (no q1 term at all).
- **The tilt IS already measured, and a tilt-aware basis already exists in
  this same file** (`BASIS_TILT`/`BASIS_TILT_REACH`, adding `sin q1`/`cos q1`
  terms) — specifically for a non-level base, since tilt gives gravity a
  horizontal component that CAN load the waist. Measured tilt: correlation
  between q1 and waist gravity torque = -0.841, in-sample R²=0.78,
  "implying roughly 3-6 degrees of tilt."
- **The tilt-aware basis is deliberately NOT used**, not because tilt
  doesn't matter, but because leave-one-out cross-validation showed it makes
  the map *worse* out-of-sample on every joint with the current pose count
  (waist LOO error 0.01133→0.01516, shoulder 0.00732→0.00783, elbow
  0.00692→0.01355) — training residual improves while generalization
  degrades, the classic overfitting signature the file's own comments call
  out explicitly.
- **Conclusion**: the real base is tilted (measured, ~3-6°), the deployed
  model's basis structurally cannot represent that tilt's effect, and the
  basis that could represent it is known-worse with current data. This is
  a structural model error, not a coefficient-tuning error — per the
  task's own instruction, this is reported as such rather than
  "compensated" by adjusting fitted coefficients.

## Phase 2 — offline evaluation on real hardware logs

Evaluated `tau_g_hat(q)` and `F_g_y(q)` sample-by-sample on all 6 gravity
magnitude-sweep captures (`y_axis_gravsweep_20260908`, 0.40-0.70N
constant-comp runs, ~5000 samples each), using the slave's own logged
`slave_q_waist/shoulder/elbow`. Zero singular samples encountered (device
stayed well clear of the Jacobian's singularity guard throughout).

| G_test (N) | n | % q2 out-of-fit-range | F_g_y min | max | mean | median | p5 | p95 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.40 | 5000 | 8.6% | 0.749 | 0.888 | 0.850 | 0.868 | 0.760 | 0.882 |
| 0.50 | 4999 | 29.1% | 0.726 | 0.962 | 0.846 | 0.850 | 0.752 | 0.923 |
| 0.55 | 5000 | 26.9% | 0.711 | 0.838 | 0.783 | 0.783 | 0.731 | 0.828 |
| 0.60 | 5000 | 2.4% | 0.675 | 0.833 | 0.758 | 0.762 | 0.681 | 0.810 |
| 0.65 | 5000 | 6.9% | 0.758 | 0.982 | 0.867 | 0.871 | 0.769 | 0.963 |
| 0.70 | 4999 | 11.3% | 0.831 | 1.069 | 0.947 | 0.943 | 0.851 | 1.055 |

**Aggregate**: 4264/29998 samples (**14.2%**) across all 6 runs fall outside
the fitted shoulder range — this is real workspace usage during normal
teleoperation-style motion, not a contrived edge case.

Plots: `y_gravity_model_validation_plots/tau_g_and_F_g_y_vs_time.png`,
`F_g_y_vs_pose.png`.

**Main sanity question — magnitude/sign check**: the model predicts F_g_y
in roughly **0.68-1.07N**, mean/median mostly 0.75-0.95N, always positive
(same sign as the empirical compensation). This is in the same order of
magnitude as the empirically successful 0.70N constant — **not obviously
nonsensical**. This one gate criterion (magnitude/sign sanity) passes.

Minor caveat visible in `F_g_y_vs_pose.png`: the 6 sweep runs also differ
in which elbow-angle sub-range they happened to sample (freehand motion,
not a repeated fixed trajectory), so overlaying F_g_y-vs-shoulder-angle
across runs isn't perfectly apples-to-apples — part of the vertical spread
between colors reflects elbow differences, not just shoulder. This doesn't
change the conclusion (the extrapolation and tilt findings are independent
of this), but is noted for completeness.

## Phase 3 — separating gravity from hysteresis

Isolated same-pose samples (shoulder angle binned to 0.01 rad) during
upward vs downward motion in the 0.70N run. Representative bins:

| shoulder q (rad) | F_g_y, upward | F_g_y, downward | tracking err, upward (mm) | tracking err, downward (mm) |
|---:|---:|---:|---:|---:|
| 0.15 | 0.875 | 0.875 | +15.17 | -11.22 |
| 0.20 | 0.903 | 0.905 | +16.99 | -10.91 |
| 0.30 | 0.958 | 0.957 | +17.73 | -10.55 |
| 0.40 | 1.010 | 1.007 | +16.09 | -8.60 |

**Model prediction is essentially direction-independent at matched pose**
(differences in the 3rd decimal, as it should be — gravity depends on
configuration, not velocity sign). **Tracking error is strongly
direction-dependent at the exact same pose** (differences of 25-28mm
between up/down at the same q). The gravity model cannot explain this by
construction (it has no velocity/direction input), and empirically doesn't
— this residual is classified as **friction/stiction/velocity-dependent
dynamics**, per the task's explicit instruction not to force the gravity
model to explain direction-dependent hysteresis. This matches the earlier
`Y_AXIS_GRAVITY_SWEEP.md` finding (hysteresis-loop shape, unchanged across
gravity values) from a different angle, now confirmed against the model's
own predictions rather than just the constant sweep.

## Model validity gate (Phase 4 precondition)

| Criterion | Result | Evidence |
|---|---|---|
| Base tilt resolved? | **FAIL** — unresolved | `FOUR_CHANNEL_TODO.md` Phase 5 "Rig quality" still `❌ NOT STARTED` |
| Gravity frame convention consistent with real rig? | **FAIL** — structural mismatch | Deployed basis assumes level base by construction; real base has measured ~3-6° tilt; tilt-aware basis exists but is known-worse with current pose count |
| Workspace coverage within identification range? | **FAIL** — significant extrapolation | 14.2% of real hardware samples outside fitted shoulder range (up to 29.1% in one run) |
| Residuals stable/improve with broader pose coverage? | **FAIL** — residuals get worse | R² degraded on ALL three joints when pose count went 6→14 (waist 0.40→0.07, shoulder 0.96→0.25, elbow 0.98→0.71) |
| Magnitude/sign consistent with 0.70N empirical baseline? | **PASS** | F_g_y ≈ 0.68-1.07N, same sign, same order of magnitude as 0.70N |

**4 of 5 criteria fail.** Per the gate rule, hardware feedforward testing of
this model is not authorized.

## Classification

**REIDENTIFICATION_REQUIRED_AFTER_RIG_FIX** (with `MODEL_INVALID_FOR_CURRENT_WORKSPACE`
as the immediate practical state — the two are the same underlying problem
viewed as "what's wrong now" vs "what fixes it").

Phases 4-6 (hardware A/B: constant 0.70N vs model-based feedforward) were
**not run**. No hardware was touched for this task — everything above is
offline analysis of already-collected logs and code.

## Recommendation (per the task's Phase 8 / the validity gate's Part D)

1. **Keep the empirically-validated 0.70N constant Y compensation as the
   baseline.** It remains the best-evidenced option; nothing here changes
   that conclusion (`Y_AXIS_GRAVITY_SWEEP.md` still stands, including its
   own recommendation to extend the sweep toward ~0.77-0.85N before calling
   the constant itself finalized).
2. **Do not attempt a new gravity identification yet.** Per the explicit
   instruction, and per this repo's own prior finding, more/wider
   static-sweep poses alone will not fix a base-tilt problem — it needs the
   base leveled first (Phase 5).
3. **Level the physical base** (Phase 5), or explicitly model the measured
   tilt in a way that doesn't require more poses than currently available
   to fit reliably.
4. **After that**, collect ONE new identification dataset spanning the
   actual intended teleoperation workspace (not just a narrow static-sweep
   cluster), fit gravity over it, validate against held-out trajectories,
   and only then reconsider a model-based-vs-0.70N hardware A/B.
5. Separately, Phase 3's finding (large direction-dependent hysteresis at
   matched pose, not explained by gravity) suggests that even a perfect
   gravity model would leave a substantial friction/stiction-shaped
   residual — worth keeping in mind as a second, independent problem for
   whenever Phase 6 (stiction characterization, from the prior task) is
   eventually run.

## Status

Per the task's Phase 7 categories, extended for this gate outcome:
**REIDENTIFICATION_REQUIRED_AFTER_RIG_FIX.** No hardware A/B was performed;
none is recommended until the base-tilt prerequisite is resolved.
