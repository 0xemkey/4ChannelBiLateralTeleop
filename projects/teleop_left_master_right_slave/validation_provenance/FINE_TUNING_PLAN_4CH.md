# 4-Channel Fine-Tuning Plan — prepared 2026-09-10

Base: `20260910T155604Z_full4ch_current_setup_baseline`, architecture
frozen per `ARCHITECTURE_FREEZE_4CH.md`. Target: preserve current real
4ch behavior, move from roughly 70% toward ~90% usable performance.
**No topology changes.** One parameter family per experiment. Do not run
hardware today unless explicitly instructed — this is a plan, not an
execution.

## Priority ranking (evidence-based)

1. **Slave final-force clamp** (Part 10) — real device-write ceiling was
   reached (41 events, up to 1.719N pre-clamp against a 1.5N cap). This
   is the only finding that touches the actual safety ceiling, so it
   ranks first even though its duty (1.35%) is small.
2. **Force-budget accounting gap** (Part 9) — not a tuning target itself,
   but MUST be fixed (extend the planner's prediction to include an
   estimated relay term, or explicitly re-label the metric) before any
   future tuning experiment is allowed to trust
   `*_accepted_force_budget_margin_n` as a total-force safety signal
   while C1/C4 are active. Treat as a prerequisite fix, not an
   "experiment."
3. **C2/C3 position clamp duty** (master 23%, slave 14.5%) — high enough
   to plausibly explain part of the RMSE/gain-error budget.
4. **C4 comparatively weak activation** (0.737 correlation, 3.16% applied
   duty vs C1's 0.944 / 23.9%) — real asymmetry, not yet root-caused.
5. **C2/C3 dynamic RMSE/lag** (RMSE ~2.5cm both channels, C3 lag 0.13s,
   C2 lag 0.0s) — track alongside items 3-4 since they share root causes
   (clamp duty directly trades off against tracking fidelity).
6. **C1/C4 balance** (weight 0.5 vs 0.3, both commissioning values, never
   derived from a transparency objective) — lowest priority; changing
   relative weights before understanding the C4 correlation weakness
   (item 4) risks tuning around a bug rather than a real gain mismatch.

## Investigation order

### A. Position clamp duty (master 23%, slave 14.5%)
- **Do not** raise `max_position_error_m` reflexively.
- First: extract clamp intervals from `raw_master.csv`/`raw_slave.csv`
  (already have the columns: `position_error_clamped`,
  `position_error_clamped_m_{x,y,z}`, `target_position_m_{x,y,z}`) and
  classify by axis, whether transient (onset of a fast operator motion)
  or steady-state (sustained clamp during slow tracking).
- If dominated by transient onset: this is a mapping/rate-limit question
  (`max_speed_mps`, bridge rate), not a clamp-size question.
- If dominated by steady-state: this is a genuine spring-authority
  question (same family as the C2 breakaway investigation earlier this
  session) — quantify the force demand at the clamp boundary before
  proposing any numeric change, exactly as the C2 breakaway force-budget
  derivation did (`required = target_spring + aux + reserve`, never a
  bare guess).

### B. Slave final-force clamp (41 events, 1.35% duty) — DECOMPOSED, 2026-09-10 closure
- **Done**: per-event vector decomposition completed for all 41 events
  (see `CURRENT_FULL4CH_AUDIT.md`'s Task 5 resolution). Result: **spring
  force dominates every event** (1.33-2.10N component norm vs. 1.59-1.72N
  total clamped norm), gravity/aux is a constant 0.550N secondary term,
  C4 relay is real but modest (0.11-0.23N, tertiary), damping is
  negligible (<0.03N).
- **The original "C4 relay coincides with clamp events" hypothesis is
  NOT supported** by the per-event data — relay is present but clearly
  secondary to spring at every single event. Superseded, not acted on.
- **Implication for tuning order**: this is a position-clamp-family
  question (item A), not primarily a C4/relay tuning question. Do NOT
  reduce C4 weight/Cf to chase this clamp — the data doesn't support that
  as the fix. Revisit item A's own investigation (transient vs.
  steady-state spring saturation) with this confirmation in hand.
- Events cluster into 4 distinct excitation bursts (t≈44.0-44.3s,
  45.4-45.6s, 54.6-54.9s, 55.9-56.3s) — consistent with brief,
  motion-triggered saturation, not continuous/routine clamping.

### C. C4 comparatively weak activation
- Candidate causes to test, not assume: master-side DOB floor (does the
  master's own resting disturbance estimate sit closer to the relay
  deadband than the slave's does?), weight asymmetry (0.3 vs 0.5 — is
  0.3 simply sub-threshold more often given the same deadband floor?),
  or genuinely lower master-side excitation during the baseline trial
  (operator interacted with the slave more than the master, by chance of
  the trial protocol, not by any controller property).
- First diagnostic: compare master vs slave DOB `tau_dis_hat_nm` resting
  floor from `raw_bridge.csv`/DOB diagnostics in the SAME baseline
  session (already recorded, no new hardware run needed) before proposing
  any weight change.

### D. C2/C3 dynamic RMSE/lag
- Track alongside A — a real reduction in clamp duty should directly
  reduce RMSE if clamping is a meaningful contributor. Re-measure RMSE
  after any change from item A before treating this as independently
  solved or unsolved.

### E. C1/C4 balance
- Deferred until C is understood — do not rebalance weights to compensate
  for what might be a C4-specific defect rather than a genuine gain
  mismatch.

## Metrics to track every experiment

**POSITION** (per channel): gain error from 1.0, RMSE, lag, position
clamp duty, settling time, return residual.

**FORCE** (per channel): source→relay correlation, per-joint sign
correlation, relay amplitude (peak/RMS), `relay_applied` duty during
excitation windows specifically (not the whole trial — matches this
session's own established practice of windowing to the actual excitation
period, learned the hard way during the C2 breakaway test's mis-timed
global-peak analysis).

**SAFETY**: primary clamp duty, final clamp duty, longest clamp interval,
max pre-final-clamp force norm, any oscillation/instability indicator
(bilateral_4ch's own energy-tank/oscillation-abort trip state, if it
fired at all — check `bilateral_status`'s safety fields for the baseline
session, not yet done in this audit).

**Explicit scoring bias, per the task's own instruction**: persistent
clamp, oscillation, wrong sign, stale relay, or a missing/masked channel
must be strongly penalized in any before/after comparison — a "better"
RMSE achieved by hiding interaction (e.g., lowering gain until nothing
saturates but also nothing is felt) is not an improvement.

## DOB

`g_dis=[10,10,5] rad/s`, `dob_g_dis_scale=1.0` — **not touched in this
plan.** Per the task's explicit instruction, DOB bandwidth is not tuned
first unless evidence specifically implicates observer dynamics as the
limiting factor; nothing in this audit's findings points there (the C1/C4
weaknesses found are correlation/duty/accounting issues, not
frequency-response symptoms).

## Prerequisite fixes before the next hardware tuning session

**2026-09-10 pre-tuning closure update: items 1-4 below are DONE.** See
`CURRENT_FULL4CH_AUDIT.md`'s "PRE-TUNING CLOSURE RESOLUTION" section for
full evidence; summary kept here for the tuning-plan's own record.

1. ~~Fix `predict_force_vector`'s missing relay term or re-label the
   force-budget margin fields~~ — **DONE**: not renamed (would be an
   architecture-adjacent live-topic change), instead documented +
   a new, separate ground-truth diagnostic
   (`actual_preclamp_force_headroom()`) added to `analyze_manual_motion.py`
   and run against the baseline (slave min actual headroom: -0.219N).
2. ~~Fix `channel_isolation_check()`'s C2/C3 source-of-truth~~ — **DONE**,
   4 new regression tests, re-verified against real baseline data.
3. ~~Harden the qualification gate to check force sign/correlation~~ —
   **DONE**: `write_qualification_artifact.py` rewritten (schema 2.0.0),
   14 new adversarial tests, independently re-verified against both the
   historical qualification trials (correctly rejected — data-availability
   gap, not a force-quality failure, see audit) and the full4ch baseline's
   own raw data (passes outright).
4. ~~Add adversarial validator unit tests~~ — **PARTIALLY DONE**: the 14
   qualification-gate tests plus 4 channel-isolation regression tests
   cover the C1/C4-force and C2/C3-semantics adversarial cases this audit
   specifically found real bugs in. The BROADER adversarial list from the
   original audit prompt's Part 7 (all-zero position dataset, constant
   pose, missing master/slave stream, unrelated-maxima-at-different-times
   for the POSITION analyzer specifically) was **not** separately added —
   `validate_trial()`'s position-side logic (`WRONG_DIRECTION_MOTION`,
   `FAILURE_TO_RETURN`, `insufficient_excitation`) was not touched by this
   closure pass and has no NEW bug evidence against it, so hardening it
   further is deferred as a normal backlog item, not a blocker.

New item found during this closure (not previously tracked): the two
original C1/C4 qualification trials' raw data predates a recorder schema
change without a corresponding `RECORDER_VERSION` bump — a real
provenance-tracking gap (see `ARCHITECTURE_FREEZE_4CH.md` item 6). Not
fixed in this pass; add a `RECORDER_VERSION` bump discipline as a small
follow-up before the next recording session, independent of tuning.

None of the above changed topology or runtime gains; all were done in
sim/unit-test/data-analysis only, no hardware touched.
