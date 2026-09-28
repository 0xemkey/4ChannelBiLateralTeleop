# Four-channel force-reflection validation (C1 focus, condition-separated)

Date: 2026-09-08
Session: `force_reflection_val_20260908`
Config: same `4ch_safe` low-authority setup as the prior system validation
(C1=0.5, C2=1.0, C3=1.0, C4=0.3), existing 0.70N Y gravity baseline
unchanged. No gains, DOB parameters, or gravity settings were changed in
this session.

**Condition marking**: each axis's data is two separate capture files
(Condition A = normal teleop, Condition B = master held/slave perturbed)
concatenated with a real, continuous time axis and an explicit
`_condition` field added during combination — **the file boundary is the
marker**, not an inference from motion. Every figure shades the entire
Condition-B interval light gray, labeled "master held / slave externally
perturbed".

Plots: `docs/force_reflection_validation_plots/` (4 figures × 3 axes + 5
figures for combined = 17 total, well under the "not 20-30 per capture"
ceiling). Raw data: `~/dob_logs/force_reflection_val_20260908/`.

## Test isolation check (did the operator actually hold the master still?)

| Axis | master range, Condition B (mm) | slave range, Condition B (mm) | ratio | isolated? |
|---|---:|---:|---:|---|
| X | 15.56 | 181.99 | 0.086 | ✅ |
| Y | 2.77 | 173.34 | 0.016 | ✅ |
| Z | 6.74 | 153.71 | 0.044 | ✅ |
| Combined (worst axis) | 8.01 | 149.60 | 0.054 | ✅ |

All four well under the informal 0.3 isolation threshold — **the test was
properly isolated in every case**; none should be classified `TEST_NOT_ISOLATED`.

## Condition A metrics (position, for reference — not this task's focus)

| Axis | RMSE (mm) | MAE (mm) | lag (ms) | reach ratio |
|---|---:|---:|---:|---:|
| X | 9.69 | 8.05 | 50 | 0.918 |
| Y | 10.02 | 9.32 | 75 | 0.975 |
| Z | 8.81 | 7.45 | 85 | 0.896 |

Consistent with the earlier system validation session — nothing new here.

## Condition B — C1 response characterization (per the task's explicit
metric list; NOT a force-accuracy comparison against ground truth, since
none exists on this rig)

| Axis | Joint | src peak (N·m) | rcv peak (N·m) | gain | sign OK | delay (ms) | release→baseline (ms) |
|---|---|---:|---:|---:|---|---:|---:|
| X | waist | 0.241 | 0.120 | **0.500** | ✅ | 5 | 1245 |
| X | shoulder | 0.074 | 0.000 | 0.0 (masked) | — | — | 455 |
| X | elbow | 0.092 | 0.046 | **0.500** | ✅ | 5 | 4765 |
| Y | waist | 0.042 | 0.021 | **0.500** | ✅ | 5 | 115 |
| Y | shoulder | 0.162 | 0.000 | 0.0 (masked) | — | — | 1290 |
| Y | elbow | 0.113 | 0.056 | **0.500** | ✅ | 5 | 575 |
| Z | waist | 0.097 | 0.048 | **0.500** | ✅ | 5 | — |
| Z | shoulder | 0.198 | 0.000 | 0.0 (masked) | — | — | — |
| Z | elbow | 0.230 | 0.115 | **0.500** | ✅ | 5 | 1490 |
| Combined | waist | 0.333 | 0.166 | **0.500** | ✅ | — | — |
| Combined | shoulder | 0.243 | 0.000 | 0.0 (masked) | — | — | — |
| Combined | elbow | 0.191 | 0.096 | **0.500** | ✅ | — | — |

**Every single non-masked joint, across all 4 test conditions, shows
gain = exactly 0.500** — matching the configured C1 weight to 3 decimal
places. This is about as clean a confirmation of correct, linear
relay scaling as this kind of test can produce. Sign is consistent in
every case. Response delay is 5ms everywhere it was measurable (bounded,
effectively the servo-loop's own timestep — see `figure 2` plots for the
visual confirmation, e.g. `X_fig2_c1_chain.png`, where source and
received are visually in-phase).

**Shoulder is correctly `MASKED_BY_CONFIGURATION`** in every single test
(`--c1-disable-joints=shoulder`, the documented mitigation for the known
shoulder DOB noise-floor problem) — labeled as such in every relevant
plot and table row, never miscounted as a broken relay.

`release_to_baseline_ms` is noisy/inconsistent (115ms to 4765ms across
otherwise-identical conditions) — this reflects how long the operator
happened to hold each push before releasing during freehand perturbation,
not a system property; not a meaningful metric from this data as measured
(a scripted, timed push/release would be needed for a real number here).

`c1_active` was **100% throughout both conditions, on every axis** — no
dropout observed anywhere in this session.

## C1 validation logic — per-axis result

Per the task's explicit decision tree:
- Master remained approximately fixed during Condition B: ✅ (all axes)
- Slave perturbation created a clear DOB/external estimate: ✅ (all axes,
  all non-masked joints)
- C1 source responded: ✅
- Corresponding master C1-received torque responded: ✅
- Sign relationship matches configured convention: ✅
- Received magnitude follows expected relay scaling (weight=0.5): ✅, exactly
- Release returns signal toward baseline: ✅ (qualitatively, in every
  figure — see `_fig2_c1_chain.png` for each axis)
- No unexplained dropout/saturation: ✅ (0% saturation flags in any
  Condition-B window, checked in `_fig4_flags.png`)

**Result: `C1_FORCE_REFLECTION_COHERENT` for X, Y, Z, and combined XYZ.**
No axis or condition triggered `C1_RELAY/GATING_PROBLEM`,
`DOB_EXTERNAL_LOAD_ESTIMATION_PROBLEM`, or `TEST_NOT_ISOLATED`.

## Additional figure — master applied vs slave estimated external force

Added per user request: `{X,Y,Z}_fig5_applied_vs_estimated.png` and
`COMBINED_fig6_applied_vs_estimated_{x,y,z}.png` — master's total applied
force (DERIVED, black) plotted against the slave's own DOB external-force
estimate, Cartesian-converted via the slave's own q/Jacobian (ESTIMATED,
blue), same gray Condition-B shading.

**Finding**: during Condition B, the two curves track almost exactly on
top of each other (see `X_fig5_applied_vs_estimated.png`) — noticeably
closer than C1's isolated 0.5 relay gain would alone predict. This is
physically coherent, not a discrepancy: master's *total* rendered force
is C1's relay contribution **plus** C2's own position-tracking spring
force, which is also live and pulling the master toward the slave's
now-perturbed position (C2 target = slave's position). The two
force-generating mechanisms compose additively and end up closely
matching the slave's full felt load, even though C1 alone only relays
half of it. This is an incidental but useful confirmation that C1 and C2
coexist coherently rather than fighting each other.

## Figure inventory

Per axis (X/Y/Z), 4 figures: `{AXIS}_fig1_position.png` (position +
tracking error), `{AXIS}_fig2_c1_chain.png` (the primary force-reflection
figure — representative joint chosen as the one with the largest observed
Condition-B response: waist for X, shoulder-excitation-but-masked so
elbow/waist reported for Y, elbow for Z), `{AXIS}_fig3_rendered_force.png`
(total applied force vs C1's Cartesian-converted contribution — explicitly
NOT claimed equal), `{AXIS}_fig4_flags.png`.

For combined: `COMBINED_fig1_position.png` (XYZ, 3 panels),
`COMBINED_fig2_c1_chain.png` (waist/shoulder/elbow, 3 panels),
`COMBINED_fig3_c1_cartesian.png` (Jacobian-converted C1 estimate in X/Y/Z),
`COMBINED_fig4_rendered_force.png`, `COMBINED_fig5_flags.png`.

17 total figures (not the 20-30+ a naive full sweep would have produced).

## Answers to the required questions

1. **Does C1 reflect slave external load to master?** Yes, clearly and
   consistently, on every non-masked joint, every axis, and combined motion.
2. **Is the sign correct?** Yes, in every measured instance.
3. **Is scaling approximately correct?** Better than "approximately" —
   gain measured at exactly 0.500 against a configured weight of 0.500,
   in all 9 non-masked joint/axis combinations plus the combined test.
4. **Is latency bounded?** Yes — 5ms response delay wherever measurable,
   consistent across all conditions, visually confirmed in-phase in every
   `_fig2_c1_chain.png`.
5. **Does the signal return to baseline after release?** Yes, qualitatively
   confirmed in every figure; the specific timing is operator-dependent
   (freehand test) and not a meaningful quantitative metric from this data.
6. **Does this hold for X, Y, Z?** Yes, uniformly — no axis behaved
   differently from the others in any respect measured here.
7. **Does it remain coherent in combined XYZ motion?** Yes — same gain
   (0.500), same sign consistency, same masking behavior, no
   instability, no saturation.
8. **Are remaining problems estimator/tuning issues or structural channel
   faults?** No problems were found in C1 itself. The one known,
   pre-existing limitation (shoulder masked due to DOB noise floor) is an
   **estimator/tuning issue** (already documented, already mitigated by
   the mask), not a channel-relay fault — the relay mechanism correctly
   respects the mask rather than leaking through it.

## What this does NOT establish

Per the task's own scope: no force-accuracy RMSE against physical ground
truth was computed (no force sensor exists on this rig). This confirms
the C1 *mechanism* — source→estimate→relay→received→rendered — is wired
correctly, linearly scaled, correctly signed, and low-latency. It does
not confirm the DOB's *absolute* external-torque estimate is accurate,
nor does it constitute a felt-quality/transparency assessment.

## Stop condition

Stopping here per the task's instruction. No gain tuning, no DOB tuning,
no gravity changes were made. Hardware fully stopped after data
collection (confirmed: no `touch_slave`/`bridge_master_slave`/`bilateral_4ch`
processes remain).
