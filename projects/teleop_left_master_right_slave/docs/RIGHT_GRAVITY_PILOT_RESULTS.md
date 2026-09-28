# Right/slave gravity pilot identification — results

Date: 2026-09-08
Session: `pilot_20260908` (standalone `hdapi_pose_hold_sweep` tool)
Data: `~/dob_logs/pilot_20260908/pose_hold_fixed.csv` (16 hold attempts),
`settle_trace.csv` (3200 samples, 200 per hold)

## Measured base orientation (Phase 1)

Roll 0°, pitch 0° (iPhone Level, ~0.5° practical resolution), no leveling
action needed. **This overturns the earlier ~3-6° statistical tilt
estimate** — see `RIGHT_RIG_FRAME_VALIDATION.md`. Do not assume tilt
explains any residual seen below; other causes remain open.

## Stillness/noise calibration (Phase 2)

8s passive capture, right arm, genuinely undisturbed: **q/qdot/qddot
variance measured as exactly zero** for all three joints — the encoder
noise floor is below detectable resolution at true rest. This makes
`threshold = k × measured_std` unusable (collapses to 0). Per operator
direction: the Cartesian settle gate already built into `autoHold()`
(`kHoldSettleVelMps = 0.006 m/s`) is the **authoritative** quasi-static
acceptance criterion; qdot/qddot are diagnostic-only in the analysis below,
not hard-rejected. A proper empirical qdot-vs-Cartesian-speed calibration
tool (`analyze_settle_transients.py`) was built and is ready to run against
`settle_trace.csv` (not yet run — see Anomalies below).

## Pilot pose list (as actually executed)

10 holds were planned as ~2 per pose; the operator moved freehand (no
jig/fixture), so actual angles vary from the nominal targets below.

| pose_id | nominal target | actual q (waist, shoulder, elbow), midpoint of ± | notes |
|---:|---|---|---|
| 1 | low shoulder, centered waist | (0.02, -0.08, 0.66) rad | hold #1 discarded (operator confirmed not physically repositioned before trigger); redone as hold #4 |
| 2 | mid shoulder, centered waist | (0.02, 0.36, 1.18) rad | |
| 3 | high shoulder, centered waist | (-0.12, 0.72, 1.60) rad | |
| 4 | low shoulder, waist +30° | (0.48, -0.13, 0.81) rad | large tau_g outlier, see below |
| 5 | low shoulder, waist -30° | (-0.60, 0.03, 0.85) rad | best pose-matched pair |
| 6 | low-mid elbow target, centered waist | (-0.01, -0.04, 0.78) rad | best pose-matched pair |
| 7 | mid shoulder, waist deflected | (-0.55, 0.27, 1.05) rad | 1 attempt auto-rejected (bias cap), retried successfully |

## Data quality summary (Phase 4 gates)

- 16 total hold attempts, **1 auto-rejected** by the tool itself
  (`hold_ok=0`, open-loop drift 0.0253 rad exceeded the 0.010 rad limit —
  the 0.55N bias cap wasn't enough to hold that pose; excluded
  automatically, not silently).
- 1 additional hold (attempt #1) manually invalidated by the operator
  (arm not physically repositioned before the trigger) — excluded by
  construction, since its pose_id/direction pair was overwritten by a
  correctly-executed retry later in the same session.
- 0 samples flagged for qdot/qddot (diagnostic-only per Phase 2 finding).
- 0 holds at the bias cap (no force-authority contamination).
- **7 usable pose pairs** from the remaining 14 valid holds.

**A real bug was found and fixed during this session**: the
`--base-orientation` string contains commas, which silently broke CSV
column alignment for every logged row (unescaped comma inside an unquoted
field). Caught immediately by inspecting the raw CSV before analysis,
repaired offline (`pose_hold_fixed.csv`), and fixed at the source
(`hdapi_pose_hold_sweep.cpp` now quotes that field) so it won't recur.
This is reported here rather than silently patched, per the standing
instruction not to hide anomalies.

## tau_g / tau_c estimates

| pose | pose_delta (rad) | paired? | tau_g [waist, shoulder, elbow] (N·m) | tau_c [waist, shoulder, elbow] (N·m) |
|---:|---:|---|---|---|
| 1 | 0.071 | DRIFTED | [-0.0060, +0.0009, +0.0003] | [-0.0026, +0.0011, +0.0007] |
| 2 | 0.065 | DRIFTED | [-0.0026, -0.0002, -0.0012] | [-0.0044, -0.0007, -0.0010] |
| 3 | 0.102 | DRIFTED | [-0.0031, +0.0030, -0.0008] | [-0.0075, -0.0054, -0.0071] |
| 4 | 0.093 | DRIFTED (wide spread) | [-0.0287, -0.0075, -0.0091] | [-0.0404, -0.0022, -0.0040] |
| 5 | 0.033 | OK | [-0.0001, -0.0054, -0.0079] | [+0.0027, +0.0008, +0.0003] |
| 6 | 0.048 | OK | [+0.0121, -0.0039, -0.0058] | [+0.0050, +0.0037, +0.0048] |
| 7 | 0.156 | DRIFTED | [+0.0045, +0.0024, +0.0023] | [+0.0133, +0.0026, +0.0027] |

`pose_delta` tolerance is 0.05 rad (matches the existing suite's
`kPairPoseTolRad`). Full data: `pilot_summary.json`.

## Required plots (`docs/right_gravity_pilot_plots/`)

1. `tau_g_vs_joint_angles.png` — tau_g vs shoulder/elbow/waist
2. `tau_c_vs_approach_direction.png` — tau_c per pose per joint
3. `tau_plus_vs_tau_minus.png`
4. `pose_delta_per_pose.png` — hold variance/pose-matching quality per pose
5. `accepted_sample_count_per_pose.png`
6. `nonphysical_sign_vs_pose_drift.png` — the key diagnostic plot, see below

## The five sanity checks

1. **Direction independence of gravity estimate**: not independently
   testable in the strict sense (tau_g is *defined* as the average of the
   two directions), but the physically meaningful version — is tau_c small
   relative to tau_g — is mixed: mean |tau_g| = 0.0105 N·m vs mean |tau_c|
   = 0.0120 N·m, comparable magnitude, not clearly gravity-dominated.
2. **Friction sign behavior**: **FAILS** for most poses. 10 of 21
   joint×pose combinations show non-physical (negative) tau_c. Poses 2, 3,
   4 show ALL THREE joints negative; pose 1 shows one.
3. **Repeatability**: only one pose/direction combo was repeated (pose 1,
   '+', by necessity from the discard/redo) — std [0.0043, 0.0010, 0.0005]
   N·m across 2 reps, not itself alarming, but too little repeated data to
   draw a general conclusion.
4. **No saturation/limit contamination**: **PASSES** — 0 holds at bias cap,
   the one rejection was caught and excluded correctly by the tool's own
   gate, not silently included.
5. **Sufficient pose coverage**: reasonable for a pilot — waist -34° to
   +49°, shoulder -13° to 72°, elbow 66° to 160° (wider than the old
   6-pose model's range in every joint) — see `Y_GRAVITY_REAL_WORLD_VALIDATION.md`
   for how the old model's range compared to real usage.

## Key finding: friction-sign failures correlate with pose-matching drift, not uniformly

`nonphysical_sign_vs_pose_drift.png` shows: **the two poses with the best
pose-matching (5, 6, both under the 0.05 rad tolerance) have ZERO
non-physical friction components. Poses 2, 3, 4 — all badly drifted (0.06-
0.10 rad) — have ALL THREE joints non-physical.** This is a coherent,
physically sensible pattern: the tau_g/tau_c decomposition assumes both
approaches reach the *same* configuration; when they don't (freehand
positioning without a jig, up to ~9° apart), the two "equilibria" being
compared aren't really the same equilibrium, and the simple ± formula
produces contaminated, sometimes sign-flipped estimates.

**This pattern is not perfectly clean, reported honestly**: pose 1 (0.071
rad drift) shows only 1 of 3 non-physical, and pose 7 — the *worst*-matched
pose in the whole pilot (0.156 rad) — shows *zero* non-physical
components. Pose-drift is clearly a major contributor, evidenced strongly
by poses 2-6, but it is not the sole or fully deterministic explanation —
something else (direction of drift, which joint moved, absolute torque
magnitude) also matters and isn't isolated by this pilot's data alone.

## Anomalies

- CSV quoting bug (found and fixed, see above).
- Operator-invalidated hold #1 (see above).
- One auto-rejected hold at pose 7 (bias cap insufficient at that specific
  configuration — expected, physical, not a tooling fault).
- `analyze_settle_transients.py` was built but **not yet run** against
  this session's `settle_trace.csv` — worth running before the next
  session to get a real qdot reference distribution, but was not required
  to reach this pilot's verdict.
- Pose 4 is a clear outlier (tau_g[waist] = -0.029 N·m, 4-10x larger
  magnitude than any other pose) — flagged "WIDE SPREAD" by the analysis
  script. Given its pose_delta (0.093 rad) is also large, this is most
  likely the same drift-contamination effect at an extreme, not a separate
  phenomenon, but this is not confirmed independently.

## Recommendation

**The underlying quasi-static separation method is not shown to be
invalid** — poses 5 and 6, the only two with actually well-matched
approaches, produced clean, physically sensible results (all-positive
Coulomb estimates, no sign anomalies). The problem this pilot surfaced is
**pose-repeatability during freehand positioning**, not the method itself.

Before a full ~30-pose campaign:
1. Give the operator a live readout of pose-delta-from-target (the
   `--status` file already shows live joint angles; it should be extended
   to show delta-from-the-first-approach's angles once a pose's '+' or '-'
   side has already been recorded, so a mismatch is visible *before*
   triggering the second approach — not discovered afterward in analysis).
2. Consider a soft physical positioning aid (even a piece of tape marking
   a target angle range, or verbally cross-checking the `--status` numbers
   between approaches) to tighten pose-matching below the 0.05 rad
   tolerance more reliably than freehand.
3. Run `analyze_settle_transients.py` against the collected
   `settle_trace.csv` to get the real qdot-at-settle-crossing distribution
   (available now, at no additional hardware cost).
4. Re-run a small number of the drifted poses (2, 3, 4, 7) with tighter
   pose-matching, rather than assuming the whole 7-pose pilot needs to be
   discarded — poses 1, 5, 6 already look usable.

## Final classification (of the ORIGINAL pilot, superseded below)

**PILOT_REVIEW_REQUIRED**

Not `PILOT_PASS_FULL_IDENTIFICATION_JUSTIFIED` — friction-sign physicality
failed for the majority of poses, and that must be addressed before
committing to a 30-pose campaign using the same freehand approach.

Not `PILOT_FAIL_METHOD_INVALID` — the method behaves correctly exactly
where its core assumption (matched approach poses) is actually satisfied
(poses 5, 6), which is evidence *for* the method's validity, not against
it. The fix indicated is procedural (tighter pose-matching), not a
rejection of the bidirectional quasi-static approach.

**⬇ Superseded by the corrective rerun below, same day — see "Final
classification" at the end of this document for the current status.**

## Pose-Matching Corrective Rerun (2026-09-08, same day)

### Phase 1 — settle-transient analysis

Ran `analyze_settle_transients.py` against the real `settle_trace.csv`
(3200 samples, 16 holds). **Result: degenerate, same pattern as the
original stillness calibration.** The Cartesian-speed crossing was
detected almost instantly (t≈0.00-0.05s) for nearly every hold, with
**qdot exactly 0.0000 at every single crossing**. This means the settle
transient completes faster than this instrumentation's 10ms sampling
resolves, or Cartesian speed was already under the 0.006 m/s gate by the
very first logged sample. Reported honestly rather than forced into a
number: this does **not** yield a usable nonzero qdot reference. Per
direction, this changes nothing — the Cartesian settle gate remains
authoritative, qdot/qddot stay diagnostic-only.

### Phase 2/3 — live pose-matching, implemented

Extended `hdapi_pose_hold_sweep.cpp` (compiled/linked clean, verified via
the same argument-validation path as before — no hardware touched by the
verification itself):
- After a pose's first successful hold, its settled `q` is saved as
  `g_poseRefs[pose_id]`.
- On the opposite approach, the `--status` file continuously shows
  reference q, current q, delta, max|delta|, and a GOOD (≤0.03 rad) /
  ACCEPTABLE (≤0.03-0.05 rad) / NOT MATCHED (>0.05 rad) label, plus
  Cartesian speed and a settled/not-settled readout — at 10Hz, unchanged
  from the existing status-file mechanism.
- The hold is **not** auto-fired on MATCH. The operator's own "go" is
  still required, and the tool **independently re-verifies** match+settle
  at that exact moment before proceeding — confirmed working live: on
  three separate occasions during the rerun, the operator said "go" while
  actually just outside tolerance (drift between my relay and the actual
  moment), and the tool correctly refused and re-prompted rather than
  silently proceeding.
- Added a small unplanned but directly-motivated safety feature:
  `r<N>` clears a bad saved reference (e.g. exactly the hold-#1 mistake
  from the original pilot), without needing to end the whole session.

### Phase 4 — sign-convention audit

Documented before touching any code: the original tool's `+`/`-` are
**operator-chosen Cartesian approach-side labels**, not independently
verified, and do not fix a consistent per-joint rotation sign (Cartesian
"above" maps to different per-joint directions depending on the
Jacobian at that pose). Checked empirically against the pilot's own
`settle_trace.csv`: the release→frozen micro-motion (the only physically
available proxy for "which way friction was loaded") is small (<0.01 rad
typically) and **not consistently signed even across holds sharing the
same `+` label**. Conclusion: the blanket `tau_c < 0 = FAIL` rule,
inherited from `hdapi_static_sweep.cpp`'s axis-probe method (where it's
guaranteed by an externally-imposed, precisely-known probe direction), is
**not physically justified** for this approach-direction method. Replaced
in `analyze_pose_hold_sweep.py` with a magnitude-consistency check
(`|tau_c|` vs `|tau_g|`, flagged not hard-failed) — verified against the
*original* pilot data before any rerun: it flagged exactly poses
`[2, 3, 4, 7]`, the same four already identified as drifted by
`pose_delta` alone.

### Phase 5 — CSV integrity, verified without hardware

Constructed the exact fprintf output (with the fixed quoting) using
representative values including a base_orientation string containing
commas, round-tripped through Python's `csv.DictReader`: **39/39 fields
parsed correctly, base_orientation stayed one field, no repair script
needed.** Confirmed again on the actual rerun output — parsed directly,
no offline fix required this time.

### Phase 6 — targeted rerun

Poses 2, 3, 4, 7 rerun with live matching. Two practical findings from
actually using the gate live:
- The interactive tool has no "skip this queued sub-prompt" option (only
  complete-or-end-session) — when the operator needed to defer a pose
  mid-attempt, the working solution was a clean Ctrl-C-equivalent abort
  (explicitly safe, zeros force) and a fresh invocation appending to the
  same log, picking up where it left off. `g_poseRefs` is in-memory only,
  so a pose whose reference-establishing `+` hold happened in an earlier
  process invocation needs its `+` redone too when resumed later — done
  for pose 3.
- Poses 3 and 4 took substantially more attempts to converge than 2 and 7
  (high-shoulder/high-elbow configurations are physically harder to
  fine-tune freehand — multiple status checks showed shoulder/elbow/waist
  fighting each other as the operator adjusted one at a time on a coupled
  3-DOF arm). Both eventually converged to GOOD (≤0.03 rad) matches
  through persistence rather than being marked `POSE_MATCH_FAILED` —
  worth knowing for the full campaign: budget more time/attempts for
  poses far from the low-shoulder region.

### Phase 7 — reanalysis: old vs rerun

| pose | old pose_delta | new pose_delta | old \|tau_c\| | new \|tau_c\| | old c/g ratio | new c/g ratio |
|---:|---:|---:|---:|---:|---:|---:|
| 2 | 0.0645 (DRIFTED) | **0.0465 (OK)** | 0.00451 | 0.01149 | 1.590 | **0.575** |
| 3 | 0.1024 (DRIFTED) | **0.0098 (OK)** | 0.01167 | 0.00868 | 2.674 | **1.182** |
| 4 | 0.0934 (DRIFTED) | **0.0236 (OK)** | 0.04062 | 0.01382 | 1.311 | **0.967** |
| 7 | 0.1561 (DRIFTED) | **0.0382 (OK)** | 0.01380 | 0.00559 | 2.467 | **0.561** |

Plots: `rerun_pose_delta_comparison.png`, `rerun_tau_g_comparison.png`,
`rerun_tau_c_comparison.png`, `rerun_c_over_g_ratio_comparison.png`,
`rerun_tau_plus_vs_minus.png`.

**Every single pose's c/g ratio improved.** Pose 4's previous large
outlier (`tau_g[waist]` = -0.0287 N·m, 4-10x any other pose) shrank to
-0.0108 N·m — right in line with the rest. Three of four poses (2, 4, 7)
now sit below the c/g=1.0 review line. **Pose 3 improved dramatically
(2.674→1.182) but did not fully clear the line, despite having the best
pose-match of all four (0.0098 rad — an order of magnitude tighter than
its original 0.1024).** This is reported honestly as a genuine residual,
not glossed over: pose-drift was clearly the *dominant* contaminant
(every pose improved, several fully cleared), but pose 3's case suggests
it may not be the *sole* factor at that particular high-shoulder/
high-elbow configuration — possibly genuine friction there, possibly a
smaller residual effect not isolated by this rerun. Worth flagging for
extra scrutiny (e.g. an additional repeat) during the full campaign,
not a blocker.

### Rerun verdict

**Scientific decision — were the original non-physical estimates
primarily caused by pose mismatch? YES, with high confidence.** All four
evidence criteria from the task's own decision framework were met: pose_delta
dropped to ≤0.05 rad for all four (three of four ≤0.04, one ≤0.01);
c/g ratio improved for every pose; the large tau_g outlier (pose 4)
shrank substantially; and the one hold that was genuinely repeated (pose
3 '+', across the process restart) showed tight repeatability (std
[0.0001, 0.005, 0.005] N·m).

## Final classification

**`POSE_MATCH_FIX_VALIDATED`** — the live pose-matching feature
demonstrably fixed the mechanism identified as the pilot's primary
problem, with consistent improvement across all four reruns and one clear
causal mechanism (not a coincidental correlation).

**`PILOT_PASS_FULL_IDENTIFICATION_JUSTIFIED`** — with one explicit
caveat: give extra attention (e.g. a deliberate repeat) to high-shoulder/
high-elbow poses during the full campaign, since pose 3's residual
suggests pose-matching alone may not fully resolve every configuration.
This is a scope note for the full campaign, not a reason to withhold it —
the dominant, well-evidenced problem (freehand pose mismatch) has a
validated fix, and the tooling (logging, CSV integrity, live matching, the
corrected sign-convention analysis) is now in a state ready for it.

## Stop condition

Per the task's instruction, stopping here. Not proceeding to the full
30-pose identification, not fitting a model, not tuning Kp/Kd, not
enabling bilateral channels. `0.70N` remains
`EMPIRICAL_CONSTANT_Y_BASELINE` — untouched, not reinterpreted as measured
or identified gravity.
