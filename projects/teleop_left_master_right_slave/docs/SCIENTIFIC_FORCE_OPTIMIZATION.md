# Scientific force optimization (Phase 6) — C1/C4 subsystem, restricted scope

Date: 2026-09-09. Offline / simulation only
(`touch_slave_sim`/`hd_sim` + `bilateral_4ch.py`). No hardware touched.

## Scope restriction (per explicit user instruction)

**`ELBOW_C1_SIGN = UNRESOLVED`, `REASON =
INSUFFICIENT_ABOVE_THRESHOLD_GROUND_TRUTH_IN_PAIRED_SIMULATION`.** This
is a testability limitation — not evidence the elbow sign is correct,
and not evidence it is wrong. See "Open scientific issue" below for the
full record.

**For the primary C1 optimization objective in this document: SHOULDER
is the validated reference channel. Elbow does not contribute to gain
selection, ranking, or any transparency claim in this document — every
elbow number below is explicitly labeled `DIAGNOSTIC_ONLY,
SIGN_UNRESOLVED`.**

**Explicitly NOT concluded from this**: "elbow should remain masked."
The current result only supports "the existing simulation test
infrastructure could not provide a clean above-threshold paired-
configuration sign validation for elbow." The shoulder hard mask remains
`SAFETY_ONLY / LEGACY_COMMISSIONING_PRESET`, not `SCIENTIFIC_OPTIMUM` —
unchanged from the assumption register's existing classification.

## C1 gain characterization — shoulder reference channel

**Method**: `analysis/scientific_4ch/phase6_c1_shoulder_gain_sweep.py`.
Waist and elbow masked OUT of the relay entirely
(`--c1-disable-joints waist,elbow`) so every received sample is
unambiguously attributable to shoulder — this avoids the elbow confound
by construction, not by post-hoc filtering. `w1 ∈ {0.3, 0.5, 0.7, 1.0}`
(0.3 = C4's current baseline, 0.5 = C1's current baseline, 0.7/1.0 =
higher-authority candidates — a bounded grid, not an invented range),
N=5 repeats × 2 conditions (contact, free-motion) = 40 runs, zero
errors.

**This does not imply the shoulder mask should be lifted** in the
deployed default — that stays the separate, untouched SAFETY_ONLY
decision. This experiment characterizes what C1 gain would do on a
clean channel, to inform gain selection independent of the mask
question, exactly as instructed.

### Results

| w1 | Contact gain (shoulder) | Sign (opposing, all N=5) | Free-motion false force (shoulder p99) | Oscillation abort |
|---:|---:|---|---:|---|
| 0.3 | 0.1609 ± 0.0016 | ✅ all 5/5 | 0.000000 | none |
| 0.5 (current) | 0.2625 ± 0.0020 | ✅ all 5/5 | 0.000000 | none |
| 0.7 | 0.3719 ± 0.0066 | ✅ all 5/5 | 0.000000 | none |
| 1.0 | 0.5370 ± 0.0058 | ✅ all 5/5 | 0.000000 | none |

**Direction/sign agreement**: opposing (matching the decided `F_m≈-F_s`
target) in all 20/20 contact runs across the full gain range — this is
the CLEANEST, most robust confirmation of the C1 sign fix obtained so
far, on the one channel with a signal well above its own noise floor.

**Gain scales linearly**: `gain/w1` ≈ 0.536, 0.525, 0.531, 0.537 across
the 4 candidates — remarkably consistent, confirming the relay behaves
as a clean linear multiply with no saturation or deadband nonlinearity
across this range (expected from the code: `tau_c1 = -w1*cf*tau_e_hat`,
no nonlinear terms).

**Free-motion false force is exactly zero at every gain tested,
including w1=1.0.** This is a genuinely reassuring result, but should be
read precisely: it means shoulder's own free-space DOB estimate stays
below ITS OWN deadband threshold even at full scale in this session's
settled-condition test (consistent with Phase 4A's finding that shoulder
showed zero measurable free-space variance across 20 repeats) — **not**
a general claim that no gain-dependent noise cost exists under all
conditions (e.g., during active master motion, not tested here).

**Phase lag / onset-release delay / coherence / bandwidth**:
`INSUFFICIENT_DATA` — not attempted, per the same clock-alignment
limitation identified in Phase 4B (the `DobDiagnostics`/`ForceEstimate`
ROS-clock timestamps and the truth CSV's internal `t_s` have no
established common time base). Steady-state tail-averaged metrics
(gain, sign) sidestep this; time-resolved metrics do not.

**Clamp/saturation activity**: not separately logged this pass (no
`master_travel_limit_flag`-equivalent checked here); `oscillation_abort`
(the one stability proxy checked) never fired across all 20 contact
runs.

**Elbow and waist, `DIAGNOSTIC_ONLY, SIGN_UNRESOLVED`** (masked out of
the relay in this experiment by design — these are the SOURCE-side
diagnostic readings from the wall-contact ground truth, not received
relay values, retained for completeness only, not used in any ranking
above):

| w1 | Elbow ground truth (N·m) diagnostic | Waist ground truth (N·m) diagnostic |
|---:|---:|---:|
| all | ~-0.0355 (below elbow's own 0.05 threshold, per the open issue below) | ~0.0 (not excited by this Y-directed contact scenario at all) |

## Waist status — reported honestly, not assumed

**`WAIST: NOT_EXCITED_BY_CURRENT_TEST_SCENARIO / UNTESTED`**, not
validated and not contradicted. The Y-directed wall-contact scenario
used for every sign/gain test this session (Phase 1, 2, 4B, and this
Phase 6) produces essentially zero ground-truth waist torque throughout
— consistent with `jacobian()`'s own structure (the Y row has a zero
waist-velocity coefficient at every pose, so a pure-Y Cartesian force
maps to zero waist joint torque via `τ=J^T F` regardless of gain or
depth). Waist's C1 sign/gain has simply never been exercised by any test
in this session. A future test would need a Cartesian excitation with a
genuine X or Z component to say anything about waist at all.

## C4 characterization — ground truth exists, verified independent of the elbow issue

**Verification, per instruction, before sweeping**: C4's ground-truth
mechanism is `hd_sim`'s hand model (`environment.hpp`'s `Hand` class,
`TOUCH_SIM_HAND`/`TOUCH_SIM_HAND_MOTION` env vars), which applies to the
**master** device and logs to the same truth-CSV mechanism
(`hd_sim_api.cpp:370`, `ext.f_tip_n += h.f_tip_n`, then
`writeTruth(d, tau_cmd, ext.f_tip_n)`) — a completely separate physical
scenario (master hand contact) and code path from the slave's wall
contact used for C1. **`PHASE_6_C4 = NOT_BLOCKED`** — confirmed, not
assumed.

**One confirmatory run** (`w4=1.0`, `TOUCH_SIM_HAND=stiff`,
`TOUCH_SIM_HAND_MOTION=step`, single run — not yet a full N=5×4-value
sweep, flagged as a scoped next step, not fabricated):

```
mean ground truth (master hand contact) = [-0.0123, 0.0658, 0.0719]
mean received (slave, C4)               = [ 0.0000, 0.0009, 0.0178]
cos_sim = +0.763   (target for C4: F_s ~= +F_m, same-direction — matches; C4's docstring/code
                     already agreed with no discrepancy found in the earlier sign audit)
```

**Result: C4's same-direction convention is confirmed by genuine
simulation ground truth** for the first time (previously only a
code-level docstring/implementation match, not simulation-validated).
Gain at `w4=1.0` (whole-vector) ≈ 0.18 — lower than C1's shoulder-only
0.537, though this is a whole-vector comparison against a per-joint
comparison and not directly comparable without more matched testing.
Waist and shoulder components in this single run are small relative to
elbow — **a full N=5 × multi-gain, per-joint C4 sweep (mirroring this
document's C1 treatment) is a scoped next step, not completed this
pass.**

## Open scientific issue

```
OPEN_SCIENTIFIC_ISSUE: ELBOW_FORCE_SIGN_VALIDATION

True torque achieved in paired simulation: ~0.0354-0.0355 N*m
  (Y-directed wall contact, master+slave+bilateral_4ch.py all running
  concurrently), consistently across every configuration tried.

Elbow's own DOB deadband threshold: 0.05 N*m (touch_slave.cpp:500,
  kThreshold[2], derived from 2x measured hardware p99 floor).

Attempts made (all this session, 2026-09-09):
  1. Contact depth sweep (TARGET_Y from -0.18 to -0.45): no change in
     ground-truth elbow torque -- stuck at ~0.0354-0.0356 regardless.
  2. Slave max_force_n sweep, ISOLATED single-slave test (no master, no
     bilateral_4ch.py): DID respond -- 0.0483 N*m at 1.5N clamp ->
     0.0523 N*m at 3.0N clamp (crosses threshold) -> plateaus at 6.0N,
     10.0N (a wall-stiffness/contact-mechanics saturation, not a device
     clamp artifact).
  3. Same max_force_n sweep, PAIRED configuration (master + slave +
     bilateral_4ch.py concurrently, the configuration actually needed
     for a sign test): did NOT respond -- stuck at ~0.0354 N*m
     (below threshold) at every clamp value tried (1.5, 3.0, 6.0N) and
     every target depth tried (-0.18 to -0.45).
  4. Direction analysis via jacobian(): pure-Y push is already close to
     optimal for elbow torque at this pose (tau_elbow =
     L2*(sin(q3)*Fy + cos(q3)*Fz), sin(0.90)=0.78 > cos(0.90)=0.62) --
     ruled out "wrong push direction" as the limiting factor.

Isolated-vs-paired discrepancy: confirmed real and reproducible, cause
  not confirmed. Leading hypothesis (not verified): scheduling
  contention between 3 concurrent real-time-paced processes
  (touch_slave_sim x2 + bilateral_4ch.py) leaves less actual simulated
  progress in the same wall-clock drive duration than the isolated
  single-process case -- consistent with, but not proven by, Phase 4A's
  independently-established finding that this simulator's real-time
  pacing is sensitive to wall-clock/scheduling variability.

Why current evidence is insufficient: every attempt to load elbow above
  its own threshold IN THE CONFIGURATION A SIGN TEST ACTUALLY NEEDS
  failed. The apparent "same-direction" reading obtained below threshold
  (documented in SCIENTIFIC_SIGN_FRAME_AUDIT.md's Phase 6 caveat) cannot
  be trusted as a genuine sign measurement -- it may reflect DOB
  contamination (already well-documented elsewhere in this project) that
  happens to correlate with this scenario's specific pose, rather than
  the true external-disturbance sign.

Future resolution paths (not implemented, no new architecture built this
  session, per instruction):
  - simulator-time-controlled / deterministic-step execution, decoupling
    simulated progress from wall-clock scheduling
  - direct joint-torque injection (bypass the Cartesian contact model
    entirely, command tau_ext_true directly for elbow)
  - shared deterministic stepping across the 3 concurrent processes
  - a dedicated simulation-only force-injection interface, sized to
    guarantee above-threshold elbow excitation in the paired config
  - future physical validation (a real force sensor, or a physical
    ground-truth proxy) -- REQUIRES_FUTURE_HARDWARE_VALIDATION regardless
```

## Phase 6 claim limit

**C1 optimization in this document is valid for the shoulder reference
channel only.** The selected gain characterization (linear scaling,
opposing sign confirmed 20/20, zero measured free-motion noise cost) is
NOT generalized to full 3-joint force transparency.

| Joint | Status |
|---|---|
| SHOULDER | **VALIDATED** — sign opposing, gain linear, N=5×4 gains, zero errors |
| ELBOW | **UNRESOLVED** — testability limitation, not evidence either way (open issue above) |
| WAIST | **NOT_EXCITED_BY_CURRENT_TEST_SCENARIO / UNTESTED** — no test in this session has loaded it |
| FULL_3DOF_C1 | **NOT YET VALIDATED** |
| C4 (whole-vector, single run) | Sign confirmed same-direction, matching its own documented (unmodified) convention; gain not yet swept |

## No scalar objective invented

Per instruction: gain, sign, free-motion noise, and oscillation-abort
are reported as separate, independent metrics (Pareto-style). No
weighted combination was computed. No C1 gain was selected as "the"
answer — the table above is evidence for a future decision, not a
decision.

## Phase 6 continuation (2026-09-09) — Next Step A: C1 active-motion noise test

**Motivation**: the settled free-motion condition above measured exactly
zero false force at every gain — too quiet to reveal any contact-
fidelity-vs-noise tradeoff. This step drives the slave through a
sustained Cartesian trajectory (no wall, no hand model — by definition
any relayed signal is false, since there is no true external
disturbance) to generate genuine velocity/acceleration content.

**Calibration performed first** (not part of the final sweep, but
recorded): tried amplitude/frequency combinations from (0.02m, 0.5Hz) up
to (0.08m, 3.0Hz) on a single `w1=1.0` probe; all showed exactly zero
transmitted signal. A direct diagnostic check (raw `tau_dis_hat`, not
the deadbanded/relayed value) at (0.08m, 3.0Hz) confirmed genuine
motion was occurring (slave Y range -0.205 to -0.092m) and that the raw
shoulder estimate DOES briefly exceed its own threshold (max 0.078 N·m
vs. threshold 0.064 N·m, but p99 only 0.068 — barely above, and mean
-0.024 — mostly well below). This confirmed the null result is not a
"motion wasn't happening" artifact — the above-threshold moments exist
but are brief/rare relative to sustained deadbanded relay content.

**Final sweep**: `analysis/scientific_4ch/phase6_c1_active_motion_noise.py`,
amplitude 0.08m, frequency 3.0Hz (validated aggressive), N=5 × 4 gains =
20 runs, zero errors.

| w1 | RMS (N·m) | p99 (N·m) | Peak (N·m) | Fraction nonzero samples | Oscillation abort |
|---:|---:|---:|---:|---:|---|
| 0.3 | 0.000000 | 0.000000 | 0.000000 | 0.0000 | none |
| 0.5 | 0.000000 | 0.000000 | 0.000000 | 0.0000 | none |
| 0.7 | 0.000000 | 0.000000 | 0.000000 | 0.0000 | none |
| 1.0 | 0.000000 | 0.000000 | 0.000000 | 0.0000 | none |

**Reported honestly, per instruction: no tradeoff appears, even under
active motion.** Shoulder's C1 channel shows zero measurable
motion-induced false force at every gain tested, up to and including
full scale (`w1=1.0`), under a fairly aggressive commanded trajectory
(0.08m amplitude at 3.0Hz — a brisk, sustained oscillation). This is
consistent with, and extends, the free-motion (settled) result: shoulder
appears to have a genuinely clean, well-behaved DOB estimate across both
tested conditions, not merely at rest.

**What this does NOT establish**: gains beyond 1.0, other motion
profiles (this tested one amplitude/frequency combination, not a sweep
of those), other poses, or elbow/waist (still `UNRESOLVED`/`UNTESTED`)
might reveal a tradeoff not visible here. Absence of evidence in this
specific, bounded test is not proof of absence generally.

## Phase 6 continuation — Next Step B: full C4 gain sweep

**Candidate set**: `w4 ∈ {0.3, 0.5, 0.7, 1.0}` — 0.3 and 1.0 are
EXISTING preset values (`PRESETS["4ch_safe"]`=0.3, `PRESETS["4ch"]`=1.0);
0.5 and 0.7 mirror C1's own grid for direct cross-channel comparability,
not independently invented. **Whole-vector characterization** — C4 has
no per-joint mask in `bilateral_4ch.py` (confirmed by source read), so
unlike C1's shoulder-isolated treatment, this cannot be cleanly
attributed to one joint; reported as such, not silently compared
apples-to-apples with C1's per-joint numbers.

**Method**: `analysis/scientific_4ch/phase6_c4_gain_sweep.py`, three
conditions × N=5 × 4 gains = 60 runs, zero errors.
- **contact**: `hd_sim`'s hand model on the master
  (`TOUCH_SIM_HAND=stiff`, `TOUCH_SIM_HAND_MOTION=step`) — ground truth
  via the same truth-CSV mechanism verified independent of the elbow C1
  issue (separate physical scenario, separate code path).
- **free_motion**: master at rest, no hand, no motion.
- **active_motion**: master driven via its own `target_pose` sinusoid
  (0.08m, 3.0Hz — same validated profile as C1's test), no hand model.

### Results

| w4 | Contact gain (whole-vector) | Direction cos_sim | Free-motion RMS (N·m) | Active-motion RMS (N·m) | Oscillation abort |
|---:|---:|---:|---:|---:|---|
| 0.3 | 0.0543 ± 0.0000 | 0.7635 ± 0.0035 | 0.000000 | 0.000000 | none |
| 0.5 | 0.0905 ± 0.0001 | 0.7629 ± 0.0038 | 0.000000 | 0.000000 | none |
| 0.7 | 0.1267 ± 0.0000 | 0.7657 ± 0.0040 | 0.000000 | 0.000000 | none |
| 1.0 (tested max) | 0.1811 ± 0.0000 | 0.7621 ± 0.0009 | 0.000000 | 0.000000 | none |

**Sign agreement**: confirmed same-direction (`cos_sim≈0.76`,
consistent across every gain and repeat, std ≤0.004) — matches C4's own
already-established, code-verified convention (no discrepancy found in
the original sign audit). This is the first GENUINE simulation
ground-truth confirmation of that convention (previously only a
docstring/implementation match), now repeated 20 times across the full
gain range with tight agreement.

**Gain scales linearly**: `gain/w4` ≈ 0.181 at every tested value
(0.0543/0.3=0.181, 0.0905/0.5=0.181, 0.1267/0.7=0.181, 0.1811/1.0=0.181)
— essentially exact, confirming clean linear relay behavior, same
pattern as C1.

**Free-motion and active-motion false force: exactly zero at every
gain**, same clean null result as C1's shoulder channel. No oscillation
aborts in any of the 60 runs.

**RMSE**: not computed as a time-resolved metric (same clock-alignment
limitation as Phase 4B/C1 — `INSUFFICIENT_DATA` for time-resolved
comparison); the steady-state gain/cos_sim metrics above sidestep this.

## Summary — six required points

**1. C1 contact-fidelity vs. active-motion-noise Pareto result**: no
tradeoff was found. Contact gain increases linearly and cleanly with
`w1` on the validated shoulder channel; motion-induced false force
measured exactly zero at every gain tested (0.3-1.0), under both settled
and active (0.08m/3.0Hz) conditions. Reported honestly per instruction —
this is a real result, not a failure to find one.

**2. C4 full gain sweep result**: same pattern. Contact gain scales
linearly with `w4` (whole-vector, since C4 has no per-joint mask);
direction confirmed same-direction/correct at every gain; zero
free-motion and active-motion false force at every gain tested (0.3-1.0).

**3. Candidate Pareto sets**: **not a meaningful Pareto front** on the
metrics tested — with the noise axis pinned at zero across the entire
tested gain range for both channels, there is no tradeoff curve to plot;
every tested point differs only in contact gain, monotonically
increasing with `w1`/`w4`, with no observed cost.

**4. Whether w1 or w4 can now be scientifically selected**: **No.** The
absence of a measured tradeoff means the evidence available does not
argue AGAINST a higher gain, but it is also insufficient breadth to
select a final deployment value: only one motion profile was tested per
condition, only one pose, gains above 1.0 were not tested, C1's
validation is shoulder-only (not full 3-DOF), and none of this is
hardware-validated. Selecting a gain now would be extrapolating past
what was actually measured.

**5. Unresolved assumptions**:
- `ELBOW_C1_SIGN = UNRESOLVED` (open issue, §above) — full 3-DOF C1
  transparency cannot be claimed.
- `WAIST_C1 = UNTESTED` — no test this session has excited it.
- Whether the zero-noise result holds at gains >1.0, other motion
  profiles/frequencies, other poses, or under genuine (not commanded)
  human-driven motion — not tested.
- C4's per-joint breakdown is unavailable (no mask mechanism exists to
  isolate it) — whole-vector only.
- Deadband parameters (`threshold_nm`, `kMinRelayTorqueNormNm`) remain
  `BASELINE_ONLY`/`ARBITRARY` per the assumption register — not
  re-examined this pass, and likely the direct cause of the clean
  zero-noise result (the deadband is, by design, suppressing exactly
  this kind of content — which is reassuring for the deployed system but
  means this test mostly validates the deadband's own effectiveness
  rather than probing the relay's intrinsic noise floor absent any
  deadband).

**6. What still requires future hardware validation**: everything in
this document. `REQUIRES_FUTURE_HARDWARE_VALIDATION` for: the C1/C4
sign conventions on physical devices (simulation-validated only, per the
sign/frame audit), whether real DOB noise under real friction/backlash/
sensor-quantization behaves as cleanly as `hd_sim`'s model, and any
claim about operator-perceived transparency or "feel" — nothing here
constitutes a felt-quality assessment.

## No gains deployed

Per instruction: no gain was selected, and nothing was written to
`run_master_left.sh`, `run_slave_right.sh`, or any shipped launch
default. All results live in `analysis/scientific_4ch/` and this
document only.

## Stop condition

No hardware used. `touch_slave_sim`/`hd_sim` + `bilateral_4ch.py` only
throughout. No shipped default changed. No new architecture/interface
built for the elbow open issue (per instruction, future resolution paths
are listed, not implemented).
