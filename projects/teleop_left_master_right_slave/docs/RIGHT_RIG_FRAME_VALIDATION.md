# Right/slave Omni rig-frame validation (Phase 1)

Date: 2026-09-08
No hardware was touched to produce this document — it is a synthesis of
existing code, existing fit data, and existing prior identification
sessions (2026-08-24, 2026-09-01), plus one honest gap: no direct physical
tilt measurement (inclinometer/level) has ever been taken. That gap is
flagged, not papered over, in the recommendation below.

## Coordinate-frame convention (from code, confirmed by direct read)

- **Joint order**: index 0=waist (q1), 1=shoulder (q2), 2=elbow (q3),
  consistent across `touch_dob.hpp`, `touch_slave.cpp`, `dob_json_loader.hpp`,
  and the Python port in `plot_phase2_scenario.py` (confirmed in the prior
  gravity-model audit, `docs/Y_GRAVITY_REAL_WORLD_VALIDATION.md`).
- **Joint angles**: raw `HD_CURRENT_JOINT_ANGLES` in radians, no zero-offset
  applied before use anywhere in the gravity/Jacobian path.
- **Kinematic model**: 2-link planar arm in a vertical plane (Jarillo-Silva
  et al. 2009 form), L1=L2=0.135m, `touch_dob.hpp:224-297`.
- **Base orientation assumption baked into the kinematics**: the FK/Jacobian
  themselves are a fixed geometric relation (link lengths and joint axes),
  independent of gravity — they do not assume a level base, and are not in
  question here. What DOES assume a level base is the **gravity model's
  basis**, not the kinematics.

## Gravity vector / frame handling in the identification code

Re-confirmed from `identification/fit_gravity.py` (full read, see also
`Y_GRAVITY_REAL_WORLD_VALIDATION.md`'s rig/frame audit section):

- The gravity model is **not** a first-principles `g_W = [0,0,-9.81]`
  rigid-body computation with an explicit rotation `g_B = R_BW * g_W`. It is
  an empirical fit of breakaway joint torque vs. pose, using a basis whose
  functional form is *derived assuming* a level base (`BASIS_MINIMAL =
  [cos q2, sin q2, cos q3, sin q3]`, no q1 term, because "the waist rotates
  about the vertical axis, which cannot change potential energy" — true
  only if the rotation axis is actually vertical).
- The codebase already contains the mechanism this task wants
  (`BASIS_TILT`/`BASIS_TILT_REACH`, adding `sin q1`/`cos q1` terms — the
  correct first-order representation of `R_BW * g_W` projected onto a
  tilted waist axis) but it is not deployed, for the reasons in the next
  section.

## Existing tilt estimate — STATISTICAL, not a direct physical measurement

**This is the key gap.** The only tilt estimate that exists comes from
fitting `BASIS_TILT` to static-sweep torque data and reading off the
correlation between q1 and the fitted waist-gravity term:

| Session | Poses | q1↔waist-gravity correlation | In-sample R² | Implied tilt |
|---|---:|---:|---:|---|
| 2026-08-24 (session B) | 10 | -0.841 | 0.78 | "roughly 3-6 degrees, depending on assumed arm mass" |
| 2026-09-01 (right arm, addendum) | 6→14 | (not separately reported; R² degraded on ALL joints when widened) | — | Confirmed as a rig-level issue, not shoulder-specific |

This is an **indirect, model-dependent estimate** — it depends on an
assumed arm mass and on the tilt basis itself being the right functional
form, and it was explicitly NOT trusted enough to deploy (LOO
cross-validation showed it overfits with current pose counts). It is
**not** a substitute for a direct physical measurement (e.g., a digital
angle gauge or bubble level placed on the device base plate). No such
direct measurement has been taken as of this document — that is a real,
outstanding gap in the evidence chain, not something this analysis can
close from existing logs.

**Uncertainty**: the "3-6 degree" range itself is a rough band ("depending
on assumed arm mass"), not a number with a stated confidence interval — a
second reason not to treat it as precise enough to hard-code into a
production frame correction without independent confirmation.

## Recommendation (this is a decision point, not something I can resolve alone)

Per the task's preferred order — physically level the base if practical,
then measure residual tilt, then record final orientation — **step 1
requires a human at the rig** (loosening/adjusting the mount, checking
level). I can't do that. Two honest paths forward:

1. **Physically check and, if practical, level the base** — cheapest,
   highest-value option if feasible. A simple digital angle gauge or
   bubble level on the device's base plate would independently confirm or
   refute the "3-6°" statistical estimate in minutes, with no hardware
   risk. If the base can be shimmed/adjusted to level, this removes the
   problem at its root rather than modeling around it.
2. **If the base cannot be physically leveled** (bolted mount, fixed
   surface, etc.), the tilt must be explicitly modeled — but the existing
   `BASIS_TILT` attempt already shows this needs *more identification data
   across a wider pose spread* than the current 6-14 poses to fit reliably
   (Phase 2 of this task proposes 20-40 poses, which may be enough — see
   `RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md`).

**This document does not resolve which path to take — that's an operator
decision** (is the mount physically adjustable? is a level/angle gauge
available?). The identification protocol below is designed to work either
way: if the base gets leveled, `BASIS_MINIMAL` becomes trustworthy again
with the proposed wider pose coverage; if it can't be leveled, the same
data collection also supports fitting `BASIS_TILT` properly for the first
time (current failures were with only 6-14 poses, well short of the 20-40
now proposed).

## Physical measurement result (2026-09-08) — OVERRIDES the earlier statistical estimate

**Measured**: roll 0°, pitch 0° (table also measured 0°, as a reference
check). Method: iPhone built-in Level app. No obvious physical base tilt
observed. Base plate not adjusted (already read as level, so no leveling
action was needed).

**Conclusion: the earlier ~3-6° tilt estimate is NOT confirmed by direct
physical measurement.** That estimate was always flagged in this document
as *statistical/indirect* (a correlation in a prior fit, dependent on an
assumed arm mass, no formal confidence interval) — this is exactly the
kind of disagreement that flag was there to catch. Per the task's explicit
instruction, **do not assume base tilt is the cause of the previous
gravity-model degradation.** It remains a candidate, but is no longer the
leading one, and other candidates stay open:
- incorrect gravity model structure (basis terms, cross-coupling)
- coordinate/sign convention mismatch (despite the earlier code audit
  finding no bug — worth re-checking against real data, not just code)
- insufficient excitation / identifiability (6-14 poses may simply be too
  few/too clustered regardless of tilt)
- friction contamination of the gravity estimate (breakaway decomposition
  assumptions not holding)
- poor pose coverage (already documented — old model's range vs. real
  usage)
- torque/logging semantics (e.g. the `HD_LAST_JOINT_TORQUE` mN·m/N·m unit
  bug found elsewhere in this codebase — worth checking whether anything
  downstream of the static-sweep tools has a similar unit assumption issue)

**Measurement precision caveat, stated honestly**: an iPhone Level is a
real, meaningful check (it would have caught a gross few-degree tilt), but
it is a consumer tool, not a calibrated digital inclinometer — practical
resolution/repeatability is more like ±0.3-0.5° than the ~0.05-0.1° a
dedicated gauge would give. This measurement rules out a *large* (3-6°)
tilt with good confidence; it does not rule out a *small* (sub-1°) tilt
that a more precise instrument might still find. Given the fitted gravity
coefficients here are themselves small (tens of mN·m), even a sub-degree
tilt is not automatically negligible — but it is a much weaker candidate
than a 3-6° tilt was, and the other causes above should be investigated
with at least equal priority now, not tilt-first.

**`--base-orientation` string for this session**:
```
--base-orientation "roll=0deg,pitch=0deg,unc=~0.5deg,method=iphone_level,leveled=already_level,confidence=moderate"
```

## Physical measurement procedure (approved 2026-09-08, superseded by the result above — kept for reference)

Do NOT infer the final base orientation from previous fit residuals — the
task is explicit about this, and it matches this document's own earlier
conclusion. This is an operator-performed physical measurement.

**Tooling**: a digital angle gauge or bubble level, placed on the device's
base plate (not the arm, not the table — the plate the arm's waist axis is
actually bolted to).

**Procedure**:
1. With the device powered off (no need for force output during this — pure
   physical measurement), place the level/gauge on the base plate.
2. Record **roll** and **pitch** relative to true level (yaw is not
   meaningful here — rotation about the vertical doesn't create a gravity
   component in this model, same reasoning as the basis discussion above).
3. Record the **measurement uncertainty** (the gauge's stated resolution/
   accuracy, plus repeatability — take 2-3 readings and note the spread).
4. Record the **method** (device used, e.g. "digital angle gauge, ±0.1°
   resolution" or "bubble level, ~0.5° readable").
5. Determine and record whether the **mount can be mechanically adjusted**
   (shims, adjustable feet, re-bolting) — this decides which path below
   applies.

**Preferred outcome**: mechanically level the device (shim/adjust until
roll and pitch both read ~0°), then re-measure and record the **residual**
orientation (there will always be some small residual — record it honestly,
don't round to zero).

**If mechanical leveling is not practical**: keep the measured orientation
and use it explicitly in the gravity model as
```
g_B = R_BW * [0, 0, -9.81]^T
```
rather than assuming `g_B = [0, 0, -9.81]^T`. This is exactly what
`BASIS_TILT` in `fit_gravity.py` is for (see above) — the measured
roll/pitch feeds the `sin q1`/`cos q1` terms' physical interpretation,
though the basis itself is fit empirically rather than by directly
plugging in `R_BW` (consistent with how the rest of this model works).

**Recording format**: once measured, pass the result to
`hdapi_pose_hold_sweep.cpp` via `--base-orientation`, e.g.:
```
--base-orientation "roll=+0.3deg,pitch=-4.1deg,unc=0.2deg,method=digital_angle_gauge,adjustable=no"
```
This string is written into every logged row (`base_orientation` column) —
not a separate config file that could drift out of sync with the data it
describes. Running the tool without `--base-orientation` logs
`UNRESOLVED` in every row and prints a warning, rather than silently
defaulting to a level-base assumption.

## Frame validation checklist status

| Item | Status |
|---|---|
| Physical base orientation (roll/pitch/yaw vs. world gravity) | **Measured 2026-09-08**: roll 0°, pitch 0° (iPhone Level, ±~0.5° practical resolution) — no large tilt found |
| Uncertainty of that estimate | ~0.5° (consumer-tool precision; does not rule out a sub-degree tilt) |
| Coordinate-frame convention (controller/kinematics) | **Resolved** — documented above, consistent throughout the codebase |
| Tilt compensated anywhere in production | **No** — deployed model uses the level-base-assuming basis (moot for now: no confirmed tilt to compensate) |
| Gravity vector orientation configurable | **No** — model is empirical torque-vs-pose, not a parametric `R_BW * g_W` term; tilt basis exists as an alternative regressor, not a rotation parameter |
| Same orientation used in identification and production | Base orientation now recorded (`roll=0deg,pitch=0deg,...`) and passed via `--base-orientation` to the identification tool — consistent going forward |

**Status: rig frame orientation is RESOLVED (measured, no significant tilt
found) as of 2026-09-08.** This overturns the earlier leading hypothesis —
**do not assume base tilt explains the prior gravity-model degradation.**
The other candidate causes listed above (model structure, sign convention,
insufficient excitation, friction contamination, coverage, torque/logging
semantics) remain open and should now be investigated with equal or higher
priority. The pilot identification session (see
`RIGHT_GRAVITY_PILOT_RESULTS.md` once it exists) is designed to surface
evidence toward several of these at once.
