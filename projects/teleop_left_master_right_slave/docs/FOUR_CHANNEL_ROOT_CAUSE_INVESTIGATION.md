# Four-channel root-cause investigation (offline only)

> **UPDATE 2026-09-08**: §11's C1 sign discrepancy (docstring `-w1`, code
> had no negation) has since been confirmed via simulation ground truth
> and **corrected** (`scripts/bilateral_4ch.py`, branch
> `c1-sign-fix-sim-validation`). See `docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md`
> for the fix and its validation. This document is left as originally
> written — a record of how the defect was found.

Date: 2026-09-08. **No hardware was used to produce this document** — pure
source-code reading, existing-CSV analysis, and offline reconstruction.
Data sources: `~/dob_logs/force_reflection_val_20260908/` (this session's
condition-separated captures), `docs/four_channel_validation_plots/`,
`docs/force_reflection_validation_plots/`, `touch_slave.cpp`,
`touch_dob.hpp`, `bilateral_4ch.py`, `run_master_left.sh`, `run_bridge.sh`,
and the existing `sim/tests/` suite.

## 1. Executive summary

The four-channel architecture is **wired correctly at the mechanism
level** (channels stay active, propagate, mask correctly, scale linearly)
but that is a much weaker claim than "good bilateral teleoperation," and
several independent, non-mutually-exclusive factors explain the gap:

1. **`applied_force` is not a valid transparency signal** — it's the sum
   of tracking-spring + damping + gravity/aux + relay, not a measurement
   of interaction force (H1, confirmed from source).
2. **A likely sign discrepancy in C1's relay** — the code's docstring
   specifies a `-w1` (negative) relay; the actual implementation applies
   no negation. Three independent lines of evidence converge on this
   (docstring, code, and this session's own empirical anti-phase plot) —
   see §11 and §16.
3. **C2 is deliberately weaker than C3** (~0.43× stiffness, 0.5× damping,
   by explicit design comment) — position-channel asymmetry is by design,
   not a bug, but it does limit bilateral position coupling.
4. **The DOB's contamination floor is already extensively documented
   in-code** as consuming 40-56%+ of full-scale torque at typical
   cutoffs — this is known, not newly discovered, and directly limits
   how clean any force-based channel (C1/C4) can be.
5. **Y-axis gravity bias is a known, separately-tracked, unresolved issue**
   (0.70N constant, undersized per prior sweep work) — not new.
6. **Travel-limit clamping is significant and axis-dependent** (master
   travel-limit fired 11-54% of the time depending on axis in this
   session's Condition-A data) — large fractions of some tests are
   operating under a clamp, where the "clean" linear channel equations
   don't apply.

No single cause fully explains "poor bilateral behavior" — it's a
combination, ranked in §16.

## 2. What is actually validated (as of this session and the prior one)

- C1/C4 signal propagation: source→received, correct masking (shoulder),
  100% channel-active, no dropout, ~5ms delay, gain matching configured
  weight to 3 decimals (0.500) — `FOUR_CHANNEL_FORCE_REFLECTION_VALIDATION.md`.
- All four channels coexist without oscillation, trip, or interference
  (`FOUR_CHANNEL_SYSTEM_VALIDATION.md`).
- The DOB's own external-force estimate sign (`tau_ext_hat = -tau_dis_hat`)
  is validated against simulated ground truth
  (`sim/tests/test_force_estimate_sign.py`, passing per its presence in
  the test suite — not re-run here, hardware/sim execution forbidden this
  session).
- `applied_force_n`'s exact composition (position + damping + gravity/aux
  + relay, post-both-clamps) — traced from source this session.

## 3. What is NOT validated

- **C1 relay sign against physical ground truth** — no test in the
  existing suite checks this specific step (only the DOB's own estimate
  sign is tested, one stage upstream of the relay).
- Absolute force accuracy (no force sensor exists on this rig — stated
  repeatedly throughout this project's prior documents).
- Felt-quality/transparency in the human-perceptual sense.
- Whether C1/C4 degrade an otherwise-good C2/C3 baseline, or whether C2/C3
  are already the dominant error source (H12 — only partially answerable
  from existing data, see §15).

## 4. Exact controller equations (from `touch_slave.cpp`)

Traced by this session's research agent, direct quotes with line numbers:

```
err[i]           = clamp(target_pos_m[i] - pos_m[i], ±max_position_error_m_)
spring_force[i]  = stiffness_n_per_m_ * err[i]
damping_force[i] = -damping_n_s_per_m_ * vel_mps[i]
force_ros[i]     = spring_force[i] + damping_force[i]
force_ros[i]    += auxiliary_force[i]                    // gravity/aux, from force_cmd topic
[primary clamp: clampNorm(force_ros, max_force_n_)]
force_device      = mapRosForceToDevice(force_ros)
[if C1/C4 gated in: f_ext = cartesianForceFromJointTorque(J(q), tau_ext, det); force_device[i] += f_ext[i]]
[final clamp: clampNorm(force_device, max_force_n_)]
hdSetDoublev(HD_CURRENT_FORCE, force_device)
```

This is the **same code, same order, for both master and slave nodes**
(one C++ class, symmetric, differing only in runtime parameters — see §6).
`applied_force_n` mirrors exactly what's written to hardware — **post
both clamps, relay included, gravity/aux included**. A prior bug (fixed
2026-09-04, per an in-code comment the agent found) had `applied_force`
captured *before* the relay addition, silently omitting C1/C4 — this is
already fixed and not a current concern.

## 5. Applied-force decomposition (H1)

**`applied_force` is definitively NOT a clean transparency proxy.** It
conflates:
- the C2/C3 tracking-error spring/damper (exists purely to track a moving
  target, present even with zero environment contact),
- the constant gravity/auxiliary term (0.70N on slave Y, always-on),
- the C1/C4 relay contribution (the only part that's actually about
  external interaction).

`Fm ≈ -Fs` tested on `applied_force` directly is **not a meaningful
test** — see `plot1_force_decomposition.png` for the reconstructed
breakdown per axis (position/damping/gravity components dominate the
total in most windows; the relay is a smaller additive layer, consistent
with the earlier session's `X_fig3_rendered_force.png` finding).

**Correct comparison** (§11, §16): the *relay-only* Cartesian
contribution (`f_ext` from step 5 of the equation above), isolated from
the tracking spring/damper, is the only telemetered quantity that
represents "what the other side's estimated interaction force
contributed" — this is what `plot6_transparency_proxy.png` and the prior
session's `haptic_force_c1/c4` plots isolate.

## 6. C2 reverse-position authority analysis (H2)

| param | C2 (master, when enabled) | C3 (slave) | ratio |
|---|---:|---:|---:|
| stiffness | 15.0 N/m | 35.0 N/m | 0.43× |
| damping | 0.10 Ns/m | 0.20 Ns/m | 0.5× |
| max_position_error | 0.03 m | 0.04 m | 0.75× |
| max_force | 2.0N device / 1.5N bridge-enforced | 1.5 N | ≈ same |
| bridge-side offset clamp | `left_max_offset_m=0.08` m | `max_offset_xyz` up to 0.19m | tighter |

This is **explicitly documented as intentional** in `run_master_left.sh`'s
own comments ("Deliberately softer than the slave's... this is a feedback
channel felt by a human hand, not a position-tracking slave"). C2 is not
"broken," it's **deliberately weak** — but this does structurally cap how
strongly the slave's position can pull the master, independent of any
force-channel (C1/C4) behavior.

**Motion transfer ratio, Condition B** (this session, master held /
slave perturbed — the master's residual motion here is *leakage* through
C2, not deliberate driving):

| Axis | master range (mm) | slave range (mm) | ratio |
|---|---:|---:|---:|
| X | 15.56 | 181.99 | 0.086 |
| Y | 2.77 | 173.34 | 0.016 |
| Z | 6.74 | 153.71 | 0.044 |

Small ratios are *expected and desired* here (the test's whole premise
was the operator holding master fixed) — not a C2 strength measurement.
A dedicated C2-driving-master test (slave moved by C3 target, C2 pulling
master, master's hand released) would be needed for a real C2-authority
number; not available in existing data (§18).

## 7. C3 forward-position tracking analysis (H3)

| Axis | RMSE (mm) | MAE (mm) | mean signed (mm) | peak (mm) | p95 (mm) | reach ratio |
|---|---:|---:|---:|---:|---:|---:|
| X | 9.69 | 8.05 | +0.59 | 20.47 | 15.89 | 0.918 |
| Y | 10.02 | 9.32 | +4.57 | 20.34 | 15.96 | 0.975 |
| Z | 8.81 | 7.45 | +5.82 | 26.62 | 15.60 | 0.896 |

**This is the C3-only forward loop, already imperfect before any C1/C4
consideration**: 9-10mm RMSE, ~15-27mm peak errors, all three axes show a
positive mean signed bias (not just Y — X and Z show non-trivial bias
too, 0.6mm and 5.8mm respectively, in *this specific full-4ch-active*
capture). **This confirms H3's concern is partially correct: C3 tracking
quality itself is a real, pre-existing limitation** independent of the
bilateral-force question — see §15 for whether this is worse than an
isolated C3-only baseline.

## 8. Y-axis gravity/bias analysis (H4)

Not re-derived here — this is already extensively characterized in
`Y_AXIS_DIAGNOSIS.md`, `Y_AXIS_GRAVITY_SWEEP.md`, and
`Y_GRAVITY_REAL_WORLD_VALIDATION.md` (this project's own prior work).
Summary relevant to this investigation:
- Constant Cartesian force, injected via the bridge's `force_cmd` topic
  → `auxiliary_force[i]` in the equation above → **is included in
  `slave_applied_force_y`**, confirmed by §5's trace.
- Current value 0.70N, empirically the best-tested constant but shown to
  still be **undersized** (residual bias persists even at 0.70N; a
  17-pose configuration-dependent model was attempted and **failed
  validation** — kept the constant, per `RIGHT_GRAVITY_IDENTIFICATION_FINAL.md`).
- This session's Y RMSE (10.02mm) and mean signed bias (+4.57mm) are
  **consistent with, not worse than**, prior gravity-sweep numbers at
  the same 0.70N setting (11.67-17.6mm RMSE across various motion
  amplitudes) — no new Y-specific degradation found; this is the same,
  already-tracked, already-explained residual.
- Per the task's constraint, the rejected pose-dependent model was **not**
  reconsidered or integrated here.

## 9. Cross-axis/Jacobian coupling (H5)

| Test axis | coupling into other axes (RMS ratio) |
|---|---|
| X | Y: 0.113, Z: 0.197 |
| Y | X: 0.085, Z: 0.441 |
| Z | X: 0.141, Y: 0.518 |

**Y↔Z coupling is consistently the largest cross term** (0.44-0.52),
roughly 3-5× larger than X's coupling into either axis. This matches
this project's own prior documented finding (`run_bridge.sh` comments:
"Z shares some of the same gravity load despite the bias being on Y
only, consistent with this rig's known base-tilt/non-level-base
coupling"). **This is a real, physical/kinematic contributor to why Y
(and to a lesser extent Z) plots look worse — not purely a control-gain
problem.** A perfectly-tuned Y controller cannot fully suppress this
coupling without addressing the underlying geometric/mounting coupling.

## 10. DOB contamination analysis (H6)

**Two levels of evidence, both included:**

**(a) Already documented in-code** (found by this session's research
agent, `touch_dob.hpp` header comments and `DobDiagnostics.msg`'s own
docstring): the residual floor "consumes 40-56% of full-scale torque at
`g_dis=2π·5` and more at higher cutoffs," explicitly attributed to
uncompensated friction (right arm has no identified friction model),
off-diagonal inertial coupling (scalar `M_n`, not full `M(q)`), and
residual gravity-model error. This is the primary, authoritative
characterization — not something this session discovered, but something
this session confirms is directly relevant to why force channels look
noisy/contaminated.

**(b) This session's own correlation check** (Condition-A/normal-teleop
data, `force_ext_hat` vs `qdot`/`qddot`/position per joint):

| Axis test | waist | shoulder | elbow |
|---|---|---|---|
| X | MODERATE (corr w/ position -0.36) | LOW | LOW |
| Y | LOW | LOW | LOW |
| Z | LOW | MODERATE (corr w/ position -0.49) | LOW |

Correlations with `qdot`/`qddot` are mostly weak (<0.25 in every case),
but correlation with raw **position** reaches -0.36 to -0.49 in two
cases — consistent with a residual-gravity/posture-dependent
contamination (matches (a)'s "residual gravity-model error" attribution)
rather than a velocity/acceleration-driven (inertial-mismatch) one. See
`plot3_dob_contamination.png`.

**Net assessment: MODERATE contamination, position/posture-correlated,
already well-characterized and expected — not a new or surprising
finding, but directly relevant to why C1/C4-based transparency looks
imperfect even where the relay mechanism itself is correct.**

## 11. C1/C4 relay-vs-transparency analysis (H7, H11)

**Relay correctness (confirmed, prior session)**: `tau_received ≈ 0.5 *
tau_source`, exactly matching the configured weight, sign-consistent by
the code's own (uncontested) internal logic.

**This is NOT the same claim as `Fm ≈ -Fs`.** Three separate things would
all need to hold for that: (1) correct DOB sign (✅ tested,
`test_force_estimate_sign.py`), (2) correct relay sign (⚠️ **unresolved**,
see below), (3) matching Jacobian/pose context between master and slave
(not generally true — different poses have different Jacobians, so even
perfect torque relay doesn't guarantee equal-and-opposite Cartesian force).

**The relay-sign question — the most significant new finding of this
investigation:**

`bilateral_4ch.py`'s own module docstring (line 21) states the intended
formula:
```
tau_m += -w1 * Cf * tau_e_hat      (C1, sent to touch_left/joint_torque_ext_cmd)
```
The actual implementation (lines 440-442):
```python
tau_c1 = [w1_eff * self.args.cf * v * m for v, m in zip(tau_e_hat, self.c1_joint_mask)]
```
**No negation is applied.** C4's docstring and code, by contrast, agree
(`+w4`, no discrepancy) — this is specific to C1.

No test in the existing suite (`sim/tests/`) validates the C1 relay's
sign against ground truth — `test_force_estimate_sign.py` only validates
the earlier `tau_ext_hat = -tau_dis_hat` step (DOB internal), and
`test_c1_relay_live.py`'s own header comment quotes the relay as
`tau_c1 = w1*cf*tau_e_hat` (matching the *current code*, not the
docstring) while only checking magnitude/gating regressions, never sign
against a ground-truth reference.

**This session's own data independently supports the "code, not
docstring" hypothesis being what's actually running** —
`plot6_transparency_proxy.png` shows `-1×(slave's own Cartesian-converted
estimate)` running **anti-phase** with the master's actual C1 Cartesian
contribution on X and Z. If the code's un-negated relay is what's live,
the master's C1 contribution should align with `+1×slave estimate`, not
`-1×` — which is exactly the anti-phase pattern observed.

**What this means, stated carefully**: this is strong, convergent,
three-line evidence (docstring, code, empirical data) that C1's sign
does *not* satisfy the classic "operator feels the reaction force"
(`Fm≈-Fs`) convention as the docstring itself specifies it should. It
does **not** by itself prove the code is "wrong" in an absolute sense —
teleoperation systems can legitimately use either a mirrored
(same-direction, "feel what the tool feels") or reactive
(opposite-direction, "feel the reaction") convention depending on design
intent, and I could not find a design document in this repo explicitly
settling which one this project intends beyond the docstring's own
stated formula. Given the docstring is this file's own stated intent,
and given no test covers this exact step, **this is ranked as a high-
confidence candidate root cause requiring hardware ground-truth
resolution** (§18), not an asserted bug.

## 12. Shoulder-mask impact (H8)

Offline reconstruction (`plot4_shoulder_mask_effect.png`,
`H8_shoulder_mask_impact` in `root_cause_metrics.json`) — hypothetical
unmasked shoulder relay estimated as `0.5 × master_force_ext_hat_shoulder`
(same weight the other two joints show), clearly labeled as
**HYPOTHETICAL, not measured**:

| Test axis | X impact | Y impact | Z impact |
|---|---|---|---|
| X-motion test | moderate (+100%) | **major** (0→0.078N, currently exactly zero) | major (+673%) |
| Y-motion test | major (+36000%, from near-zero) | **major** (+16300%) | **major** (+44000%) |
| Z-motion test | minor (+20%) | major (0→0.047N) | major (+7400%) |

**The percentages are large mostly because the CURRENT (masked) values
are already very small** — the more physically meaningful reading is the
absolute RMS magnitudes: with shoulder unmasked, the hypothetical
Cartesian relay reaches up to ~1.2-1.5N (Y-axis test) vs the current
masked reach of a few mN. **Shoulder masking has a MAJOR effect on Y and
Z specifically** — consistent with the shoulder joint's large lever arm
into the vertical/depth Cartesian directions in this arm's geometry —
and a comparatively smaller (though still non-negligible) effect on X.
This directly compounds §9 and §10's Y/Z findings: the masked joint that
would matter most for Y/Z transparency is exactly the one that's
disabled (for a separate, already-documented reason — shoulder's DOB
noise floor exceeds its own torque budget).

## 13. Clamp/travel-limit/gating analysis (H9, H10)

| Axis | master travel-limit % | slave travel-limit % | force saturation % |
|---|---:|---:|---:|
| X | 23.0 | 10.3 | 0.0 |
| Y | **54.3** | 0.0 | 0.3 |
| Z | 11.6 | 0.0 | 0.0 |

**Y spends over half its Condition-A test time with the master's C2
travel-limit engaged.** This is a substantial fraction of the test
operating in a clamped, non-linear regime where the "clean" spring/
damper/relay equations no longer describe the system — any position or
force metric computed over the whole window is partly describing clamp
behavior, not free control response. See `plot2_position_transfer.png`
(red-shaded regions).

**Gating/switching frequency** (C1 relay on/off transitions,
Condition B): X 2.25/s, Y 7.0/s, Z 3.5/s. Y's much higher switching rate
is consistent with more frequent deadband crossings during the more
active Y perturbation test — not evidence of a fault by itself, but
confirms the "filled region" visual appearance in force plots is a real,
high-frequency deadband-edge phenomenon, not a plotting artifact.

## 14. Sign/frame audit (H11)

See §11 for the main finding (C1 relay sign). Full chain, confirmed
source-verified this session:

| Stage | Sign operation | Verified how |
|---|---|---|
| DOB: `tau_dis_hat` | one negative inside filter (gravity/friction subtracted from `tau_cmd`) | code read, `touch_dob.hpp:754` |
| `tau_ext_hat = -tau_dis_hat` | one explicit negation | code read + `test_force_estimate_sign.py` (passes, per suite presence) |
| Deadband | sign-preserving | code read |
| **C1 relay: `tau_c1 = +w1·cf·tau_e_hat·mask`** | **no negation (docstring says there should be one)** | code read, **not covered by any test** |
| C4 relay: `tau_c4 = +w4·cf·tau_h_hat` | no negation (docstring agrees, no discrepancy) | code read |
| Cartesian conversion (`cartesianForceFromJointTorque`) | algebraic inverse of `τ=J^Tf`, no extra flip | code read |

No axis-swap or frame-flip was found in this trace — the discrepancy is
specifically the missing negation in C1's relay step, isolated to that
one line.

## 15. Isolated-vs-full-channel comparison (H12)

**Partial answer only** — existing data doesn't include a clean, directly
comparable C3-only (unilateral) baseline at matching axis/amplitude to
this session's full-4ch captures (checked `2ch_vs_4ch`/`4ch_only`
directories in `~/dob_logs/`; both turned out to already have C1/C4
active throughout, not a true isolated split — see `root_cause_metrics.json`
for the raw check).

**What IS comparable**: Y-axis. This session's full-4ch Y RMSE (10.02mm,
mean signed +4.57mm) is **in the same range as**, not worse than, the
isolated-C3-only Y-axis gravity-sweep RMSE at the same 0.70N setting
(11.67mm, `Y_AXIS_GRAVITY_SWEEP.md`). **This one comparable data point
does NOT support "adding C1/C4 makes position tracking worse"** — but
it's a single axis, single condition; X/Z have no matching isolated
baseline in existing data. **Classification: insufficient data to fully
resolve H12 — leaning away from CHANNEL_INTERACTION_PROBLEM based on the
one available comparison, but this needs a dedicated matched-condition
test (§18), not asserted as resolved.**

## 16. Ranked root causes

| # | Cause | Rank | Evidence | Counter-evidence | Confidence | Axes | Category |
|---|---|---|---|---|---|---|---|
| 1 | C1 relay missing negation vs. documented intent | **MAJOR** | docstring, code, empirical anti-phase plot (3 convergent lines) | no ground-truth hardware test exists either way; alternate "mirror convention" reading is plausible | Medium-High | all (X/Z clearest, Y less clear from this data) | Implementation |
| 2 | `applied_force` misused/misinterpreted as a transparency signal | **MAJOR** | full source trace, H1 | none — definitively confirmed | High | all | Measurement interpretation |
| 3 | DOB internal-dynamics contamination | **MAJOR** | extensively pre-documented in-code (40-56% floor), this session's own position-correlation (moderate on 2/9 joint-axis pairs) | qdot/qddot correlation mostly LOW | High (for the documented floor), Medium (for this session's specific correlation numbers) | all, esp. where posture varies | Estimator |
| 4 | Shoulder C1 mask | **MAJOR** for Y/Z, MODERATE for X | quantitative Jacobian reconstruction, large hypothetical magnitude difference on Y/Z | reconstruction is hypothetical/offline, not measured | Medium (mechanism certain, magnitude estimate approximate) | Y, Z primarily | Tuning/config (deliberate tradeoff, not a bug) |
| 5 | Y↔Z kinematic/mounting coupling | **MODERATE** | 0.44-0.52 RMS ratio, matches prior documented base-tilt finding | — | High | Y, Z | Kinematics/rig |
| 6 | C2 deliberately weak vs. C3 | **MODERATE** | explicit design comment, numeric parameter comparison | intentional, not a fault | High | position-channel bilateral coupling, all axes | Tuning (by design) |
| 7 | Travel-limit clamping (esp. Y, 54%) | **MODERATE** | direct flag-percentage computation | — | High | Y primarily, X secondarily | Limits/nonlinearity |
| 8 | Y-axis gravity bias | **MODERATE** (already tracked) | consistent with prior sweep work, no new degradation found | already has its own extensive investigation and a decision (keep constant) | High | Y | Estimator/tuning (known, open) |
| 9 | Channel interaction (C1/C4 making C2/C3 worse) | **NOT SUPPORTED** (insufficient data) | one matching comparison (Y) shows no clear degradation | no clean X/Z isolated baseline exists to fully test | Low (data-limited) | inconclusive | Unknown — needs dedicated test |

## 17. Recommended next tuning order (analysis only — not executed here)

1. **Resolve the C1 sign question first** (§11, §16#1) — this is
   foundational; tuning gains or DOB parameters on top of a potentially
   wrong-signed relay would be premature. Needs a hardware ground-truth
   test (§18), not a parameter change.
2. **Stop using `applied_force` for any transparency judgment** — switch
   to the relay-only Cartesian reconstruction (`plot6`-style) for all
   future force-channel assessments. This is a measurement-practice fix,
   not a hardware change.
3. Only after #1 is resolved: revisit C1/C4 weighting (currently 0.5/0.3)
   with an informed target, and consider whether shoulder can be
   partially re-enabled once/if its DOB noise floor is separately
   improved (not attempted here).
4. C2 stiffness/damping tuning, informed by whatever bilateral-coupling
   target is actually desired (mirror vs. reactive convention, same
   open question as #1).
5. C3 gain tuning (stiffness/damping) — already known to have ~9-10mm
   RMSE; a legitimate target for the "next stage" fine-tuning this task
   explicitly defers.
6. Y-axis gravity/travel-limit — already has its own tracked, separate
   investigation; not re-opened here.

## 18. Missing measurements / tests requiring future hardware access

- **A dedicated C1-sign ground-truth test** (analogous to
  `test_force_estimate_sign.py`, but for the relay step specifically):
  perturb the slave with a known-direction force, measure whether the
  master pushes the operator's hand in the same or opposite direction —
  this is the single most important missing measurement identified by
  this investigation.
- **A clean, matched-condition isolated-C3-only baseline for X and Z**
  (Y already has one) — needed to properly resolve H12.
- **A dedicated C2-authority test** (slave driven by C3, master released,
  measure how strongly it's pulled) — Condition B's "master held" test
  answers a different question (leakage under active holding, not C2's
  free-driving strength).
- **Any true force-sensor ground truth** — repeatedly noted throughout
  this project as absent; nothing in this investigation changes that.

## Answers to the required questions

**A. Why do the current plots look bad?** A combination of (1) using
`applied_force` (which is not a transparency signal) as if it were one,
(2) a likely C1 sign issue that, if confirmed, would make the relay
mirror rather than oppose (or vice versa) the slave's felt force, (3)
already-documented DOB contamination, (4) shoulder masking removing a
geometrically-important joint from Y/Z force reflection, and (5)
substantial travel-limit clamping on Y specifically.

**B. Is the dominant problem control gains, gravity, DOB, force scaling,
shoulder mask, Jacobian/coupling, limits/clamps, sign/frame convention,
or a combination?** **A combination**, with the sign-convention question
(§11) and the applied-force-misinterpretation issue (§5) as the two
highest-leverage items to resolve first, since they affect how every
other measurement should be interpreted.

**C. Is the architecture structurally correct?** Largely yes, at the
signal-routing level (confirmed by the two prior validation sessions).
The one structural-adjacent concern is the C1 sign discrepancy — if
confirmed by a ground-truth test, it would be a genuine implementation
bug, not an architectural flaw (the fix would be a one-line sign change,
not a redesign).

**D. Which parameter group should be tuned first?** None yet — per §17,
the sign question must be resolved (a correctness question, not a tuning
question) before any gain tuning is meaningful.

**E. Which conclusions cannot be reached without future hardware tests?**
Whether C1's actual sign (docstring vs. code) is correct against physical
ground truth; whether C2's authority is adequate when actually
driving the master (not just leaking under a held hand); whether X/Z
position tracking is degraded by C1/C4 relative to an isolated baseline
(Y's comparison doesn't generalize); any absolute force-accuracy claim.

## Stop condition

This task ends here. No hardware was used, no parameters were changed, no
tuning was performed, no autonomous experiments were run.
