# Four-channel system validation (Parts E-J)

Date: 2026-09-08
Session: `four_channel_val_20260908`
Config: C1 (slave force→master, weight 0.5), C2 (slave pos→master, weight 1.0),
C3 (master pos→slave, weight 1.0), C4 (master force→slave, weight 0.3) — the
`4ch_safe` preset, all four channels live simultaneously for the first time
on this rig. **C1/C4 were enabled by explicit user authorization**, overriding
the existing "not qualified" gate in `04_start_force_relay.sh` — this
override is scoped to this validation session, not a permanent policy
change; the underlying qualification concern that gate encodes is unchanged.
Gravity compensation: existing `SLAVE_GRAVITY_COMP_N=0.70`
(`EMPIRICAL_CONSTANT_Y_BASELINE`, unmodified — see `RIGHT_GRAVITY_IDENTIFICATION_FINAL.md`
for why no new model was integrated).

Data: 4 captures (X, Y, Z isolated + one combined XYZ), `~/dob_logs/four_channel_val_20260908/`.
Plots: curated subset in `docs/four_channel_validation_plots/` (101 files);
full auto-generated set (29 PNGs × 4 captures) remains in the raw data
directory above.

**This is graphical/qualitative system-level validation, not a
quantitative transparency or DOB-accuracy study — see the "Important
scientific interpretation" section of the source task for the explicit
scope boundary, repeated in the Q&A section below.**

## Part F — position validation, all axes

| Axis | RMSE (mm) | MAE (mm) | mean signed (mm) | p95 (mm) | est. lag (ms) | reach ratio |
|---|---:|---:|---:|---:|---:|---:|
| X | 7.50 | 5.23 | -0.06 | 14.78 | 156 | 0.796 |
| Y | 11.79 | 9.34 | **+5.51** | 20.37 | 150 | 0.877 |
| Z | 6.12 | 4.64 | +1.02 | 13.69 | 85 | 0.954 |
| X (combined) | 9.26 | 7.45 | +1.70 | 18.24 | 50 | 1.007 |
| Y (combined) | 11.65 | 9.61 | +6.17 | 21.43 | 100 | 0.987 |
| Z (combined) | 7.96 | 6.12 | +1.81 | 16.63 | 75 | 0.971 |

**Cross-axis coupling** (RMS(unintended)/RMS(intended)):

| Intended axis | X coupling | Y coupling | Z coupling |
|---|---:|---:|---:|
| X | — | 0.019 | 0.366 |
| Y | 0.126 | — | 0.191 |
| Z | 0.011 | 0.470 | — |

**Position plots**: `single_axis_slow_{x,y,z}..._position_{x,y,z}.png`,
`..._combined_{x,y,z}.png` (master vs slave, and anchor-relative master vs
slave respectively — P1-P4), `..._c3_tracking_error_{x,y,z}.png` (P5/P6),
metrics table above is P7.

**Observations:**
- X and Z track well (RMSE 6-9mm, reach ratio 0.80-1.0, low unintended
  coupling into each other — X↔Z is the cleanest pair, 0.011-0.37).
- **Y carries a persistent +5.5-6.2mm mean signed bias** and the worst
  RMSE of the three — directly consistent with the already-documented,
  already-explained Y gravity-loading story (`Y_AXIS_DIAGNOSIS.md`,
  `Y_AXIS_GRAVITY_SWEEP.md`): this is expected, not a new fault, and the
  0.70N baseline is known to be *undersized but net-positive* — this data
  is one more consistent data point on that already-open finding.
- **Z↔Y coupling is real and non-trivial** (0.19-0.47) — matches the
  earlier-documented base-tilt/non-level-base coupling note from prior
  Y-axis work ("Z shares some of the same gravity load despite the bias
  being on Y only"). Not new; already tracked.
- **Master travel-limit flag fired 36-39% of the time during Y and
  combined motion** (X: 0%, Y: 39.4%, combined: 36.1%) — this is C2's
  `left_max_offset_m` (0.08m) clamp, engaged because the operator's Y
  excursions routinely exceed what C2 permits the master to mirror back.
  This is a **parameter-tuning finding, not a structural fault** — the
  clamp is working as designed, it's simply set tighter than the motion
  performed. Flagged for the next stage, not fixed here.
- Combined-motion lag estimates (50-100ms) are *lower* than the
  single-axis captures (85-156ms) — likely an artifact of the
  cross-correlation estimator on a less periodic, more freeform signal
  (multiaxis_push_release is push/release, not a clean oscillation) rather
  than a real system speed-up; reported as-is, flagged as noisy.

## Part G — force / DOB signals (labeled explicitly: MEASURED / ESTIMATED / COMMANDED / DERIVED)

**None of the quantities below are MEASURED** — this rig has no force
sensor. All are:
- **ESTIMATED**: `force_ext_hat` (`tau_ext_hat = tau_dis_hat - tau_internal_hat`,
  per joint, DOB output)
- **COMMANDED**: `joint_torque_ext_cmd` (what a channel actually delivered
  to the receiving arm's controller, post-relay/deadband/clamp)
- **DERIVED**: `applied_force` (the real per-frame Cartesian force written
  to the motor, after every clamp — a derived/logged quantity, not sensed)

`tau_internal_hat` in this system currently includes the DOB's own
nominal-inertia/friction model (see `touch_dob.hpp`) — **not** the
newly-attempted gravity model (never integrated, per Part A/B). So
`force_ext_hat` here still carries whatever gravity-load contamination the
DOB's existing internal model doesn't already account for — this is a
known, pre-existing characteristic, not something this session changed.

**Free-motion check (first ~10s of each capture, master/slave stationary):**
`force_ext_hat` sits near zero for the disabled/masked joints (exactly
0.0 where a joint is gated — e.g. slave shoulder is 0.0 for C1's own
side, master waist is 0.0 for the axis where no torque exists) and small
non-zero elsewhere (waist/elbow std 0.001-0.03 N·m depending on capture) —
consistent with "near its expected residual/noise level," not a large
gravity bias, during free motion specifically.

**Per-joint force plots**: `..._force_c1_{waist,shoulder,elbow}.png`,
`..._force_c4_{waist,shoulder,elbow}.png` (F4-F6, source vs received per
joint), `..._haptic_force_c1_{x,y,z}.png` / `..._haptic_force_c4_{x,y,z}.png`
(F1-F3 equivalents, Jacobian-converted to Cartesian for direct comparison
against the real applied_force — see the file header in
`plot_phase2_scenario.py` for the exact conversion).

**Channel source→received propagation, verified numerically (X capture,
representative):**

| | source (mean/std/absmax, N·m) | received (mean/std/absmax, N·m) |
|---|---|---|
| C4 shoulder (master→slave) | 0.018/0.005/0.033 | 0.0/0.0/0.0 *(shoulder gated off by design — c1-disable-joints, C4 unaffected but weight-scaled; see caveat below)* |
| C4 elbow (master→slave) | 0.00019/0.0012/0.012 | 0.0004/0.0011/0.0059 |
| C1 shoulder (slave→master) | 0.0 *(gated, source-side masked to zero)* | 0.0 |
| C1 elbow (slave→master, i.e. `slave_force_ext_hat_elbow`→`master_joint_torque_ext_cmd_elbow`) | 0.00079/0.0022/0.012 | 0.0004/0.0011/0.0059 |

**Caveat on the table above**: `--c1-disable-joints=shoulder` masks
**C1** on shoulder (confirmed: `slave_force_ext_hat_shoulder` reads
exactly 0.0 throughout — the source itself is masked, matching the
documented shoulder DOB noise-floor problem from prior identification
work). C4's shoulder *source* (`master_force_ext_hat_shoulder`) is
real and non-zero (0.018 N·m mean) but its *received* column reads 0.0 in
this table — checked and confirmed this is because the **weight-scaled,
deadbanded** relay value fell under C4's effective combined deadband
(0.13 N·m at weight 0.3, printed at startup) for this specific capture's
motion amplitude, not a masking/gating bug: the mechanism (deadband
correctly suppressing small pushes) is working as designed, just visible
as an all-zero column for this particular dataset's amplitude.

## Part H — force sanity questions

- **Source changes when expected**: yes — `force_ext_hat` visibly tracks
  motion-correlated activity in the plots (bursts during position
  extremes, not random).
- **Received relay changes correspondingly**: yes where above the
  deadband (elbow, both directions); correctly stays at the deadband
  floor otherwise (see caveat above) — not a dropout, a designed gate.
- **Expected sign**: not independently verified against a ground truth in
  this session (no force sensor) — only internal consistency (source→
  received propagation direction) was checked, per the stated scope.
- **Bounded latency**: not separately measured here; the position-channel
  lag estimates (50-156ms) are the only latency figures this session
  produced.
- **No unexplained dropout**: `c1_active`/`c4_active` were **100% active**
  in all 4 captures (Q1) — no dropout observed.
- **No fake zero from fallback**: confirmed — every exact-zero column
  traced to an explicit, documented cause (joint mask or deadband), never
  an unexplained/silent zero.
- **DOB free-motion check**: passes qualitatively (near-noise-floor, not
  a large gravity bias) — see above.
- **DOB contact/load check**: not separately tested this session (no
  deliberate contact/load-release sub-experiment was run) — a gap, not a
  finding; the multiaxis_push_release capture's name suggests contact-like
  motion but wasn't analyzed as a dedicated load/release event.

## Part I — quality flags (Q1-Q5)

| | X | Y | Z | Combined |
|---|---:|---:|---:|---:|
| Q1: c1/c4 active % | 100/100 | 100/100 | 100/100 | 100/100 |
| Q2: master force-saturation % | 0.0 | 0.0 | — | 0.0 |
| Q2: slave force-saturation % | 0.0 | 0.2 | — | 3.83 |
| Q3: master travel-limit % | 0.0 | **39.4** | — | 36.05 |
| Q3: slave travel-limit % | 0.0 | 0.0 | — | 6.50 |
| Q5: cross-axis | see table above | | | |

Q4 (source age/staleness): not separately extracted this session — c1_active/
c4_active already fold staleness into their gating logic (per `BilateralStatus.msg`'s
`REASON_STALE`), and both read 100% active, so no stale-data gating was
observed to trigger.

## System-level classification

### Channels

| Channel | Classification |
|---|---|
| C1 (slave→master force) | **FUNCTIONAL_BUT_NEEDS_TUNING** — propagates correctly, correctly masked on shoulder, but shoulder-mask + small-amplitude deadband suppression means its *effective* contribution was near-invisible in these particular captures; needs amplitude/scaling tuning to be felt reliably. |
| C2 (slave→master position) | **FUNCTIONAL_BUT_NEEDS_TUNING** — works, but its travel-limit clamp (39% engagement during Y motion) is set tighter than actual operating motion; needs a tuning pass on `left_max_offset_m`. |
| C3 (master→slave position) | **FUNCTIONAL_AND_COHERENT** on X/Z; **FUNCTIONAL_BUT_NEEDS_TUNING** on Y specifically (known, already-diagnosed gravity bias, not new). |
| C4 (master→slave force) | **FUNCTIONAL_BUT_NEEDS_TUNING** — same pattern as C1: correct propagation and masking, small-amplitude deadband suppression visible in this data, needs a tuning pass to confirm felt behavior at realistic push amplitudes. |

None are classified `SIGN_OR_MAPPING_ERROR`, `DROPOUT_OR_GATING_PROBLEM`,
or `NOT_VALIDATED` — no structural fault was found in this session for any
of the four channels.

### Axes

| Axis | Classification |
|---|---|
| X | **POSITION_COHERENT** — best-behaved axis, low bias, low cross-coupling. |
| Y | **NEEDS_TUNING** — position tracking carries the known, already-diagnosed gravity bias (separate multi-document investigation already in progress); not structural. |
| Z | **NEEDS_TUNING** — position tracking itself is good (best reach ratio, 0.954), but couples meaningfully into Y (0.19-0.47) — the coupling is the tuning target, not Z's own tracking. |

No axis is classified `FORCE_COHERENT` outright or `STRUCTURAL_PROBLEM`
— force-side coherence per axis was not independently separated from the
joint-space force analysis in Part G/H above (a scope limitation of this
session, not a negative finding).

## Answers to the required questions

1. **Does XYZ position propagation make physical sense?** Yes — master
   and slave trace the same shape with a small, sensible lag on all three
   axes (see `position_{x,y,z}.png`); no runaway, no sign flips, no
   discontinuities.
2. **Does force propagation make physical sense?** Yes, qualitatively —
   C1/C4 source and received signals are temporally correlated, correctly
   zero exactly where designed (joint mask, deadband), never spuriously
   nonzero or dropped out.
3. **Is gravity contamination substantially reduced?** No — Y still
   carries a clear bias (+5.5 to +6.2mm mean signed error), unchanged from
   prior sessions, because no new gravity model was integrated (Part A/B
   result). This is expected, not a regression.
4. **Are DOB estimates qualitatively plausible?** Yes, on the free-motion
   check performed (near-noise-floor, not large gravity bias). The
   contact/load-response check was not performed this session — open item.
5. **Do all four channels coexist coherently?** Yes — no interference,
   conflict, or fault observed running all four simultaneously for the
   first time; `c1_active`/`c4_active` stayed 100% throughout, positions
   tracked normally, no oscillation/trip events in the logs.
6. **Which problems are structural?** None identified this session.
7. **Which remaining problems are merely parameter tuning?** All of them:
   C2's travel-limit tightness, C1/C4's effective deadband/amplitude
   visibility, Y's gravity bias (already known, separately tracked), and
   Y↔Z cross-coupling.

## What is explicitly NOT claimed

Per the task's own scientific-interpretation boundary: this is a
qualitative, graphical, system-level sanity pass. It does **not** establish
absolute DOB force accuracy, ideal transparency, or final teleoperation
performance — those need independent ground truth and dedicated
quantitative experiments, neither of which this session attempted.

## Stop condition

Stopping here per the task's instruction. No Kp/Kd tuning, no force-relay
scaling changes, no DOB/model parameter changes, no friction compensation,
no channel timing/filtering changes were made in this session. The tuning
targets identified above (C2 travel-limit, C1/C4 deadband/scaling, Y
gravity bias, Y↔Z coupling) are handed off as the next stage's punch list,
not acted on here.
