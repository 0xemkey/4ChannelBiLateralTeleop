# Scientific position optimization (Phase 5) — C2/C3 subsystem

Date: 2026-09-08. Offline / simulation only
(`touch_slave_sim`/`hd_sim` + `bridge_master_slave.py`, the **real,
hardware-confirmed Cartesian implementation** of C2/C3 — not
`bilateral_4ch.py`'s separate joint-space C2/C3, which that script's own
docstring says is NOT the working path). No hardware touched. C1/C4
absent throughout (`bridge_master_slave.py` carries no force-relay
channel of its own; the only force present is each device's own
Cartesian tracking spring/damper).

## Method

**Objective, per the task's own rule**: position/velocity transparency
error only (`e_x = x_m - x_s`, `e_v = xdot_m - xdot_s`), C1/C4 excluded.
No `J_pos` scalar weighting was invented — metrics are reported
separately (Pareto-style), never collapsed into one number.

**Driving signal**: a constant 0.4N force step on the master (X axis),
held 4.0s, then released for 3.0s — a clean step-response condition for
rise-time/overshoot/settling analysis, plus a release/recovery phase.
Both C2 (slave position → master target) and C3 (master position →
slave target) active (`--enable-c2 true`; C3 is `bridge_master_slave.py`'s
default-on behavior).

**Three gain configurations, none invented** — pulled directly from
existing shipped defaults:

| Config | master (K, D) | slave (K, D) |
|---|---|---|
| `baseline_asymmetric` (current) | (15.0 N/m, 0.10 Ns/m) | (35.0 N/m, 0.20 Ns/m) |
| `symmetric_soft` | (15.0, 0.10) | (15.0, 0.10) |
| `symmetric_stiff` | (35.0, 0.20) | (35.0, 0.20) |

N=5 repeats per configuration (15 sim runs,
`analysis/scientific_4ch/phase5_position_subsystem.py`).

**Test-infrastructure finding, documented not hidden**: the first attempt
at this experiment produced a completely frozen slave (0 tracking motion
despite the master moving ~19mm). Root-caused via direct debugging (not
assumed): `bridge_master_slave.py`'s `startup_alignment.py` state machine
requires an explicit `arm_teleop` ROS param to transition
`READY → RAMPING → ACTIVE` — a **deliberate operator-confirmation safety
gate**, not a bug. Fixed by calling
`ros2 param set /touch_master_slave_bridge arm_teleop true` after the
"teleoperation live" marker, before driving. This is now built into the
experiment script with an explanatory comment, so future Phase 5/6 work
does not rediscover this.

## Results (N=5 per config; std shown, not hidden)

| Metric | baseline_asymmetric | symmetric_soft | symmetric_stiff |
|---|---:|---:|---:|
| RMSE, `e_x` (mm) | 3.02 ± 0.05 | 6.62 ± 0.04 | **2.80 ± 0.01** |
| MAE (mm) | 2.75 ± 0.01 | 6.43 ± 0.03 | **2.69 ± 0.01** |
| Signed bias (mm) | -2.74 ± 0.01 | -6.42 ± 0.03 | **-2.66 ± 0.02** |
| Peak error (mm) | 16.97 ± 0.92 | 21.84 ± 0.48 | **10.15 ± 0.33** |
| Overshoot (%) | 14.49 ± 0.53 | 35.28 ± 0.30 | **7.97 ± 0.62** |
| Reach ratio (slave/master disp.) | 1.394 ± 0.053 | 1.407 ± 0.031 | **1.046 ± 0.017** |
| Lag, rise-time (s) | **0.018 ± 0.002** | 0.031 ± 0.002 | 0.031 ± 0.002 |
| Master rise time (s) | **0.070 ± 0.001** | 0.071 ± 0.002 | 0.127 ± 0.003 |
| Cross-axis Y+Z / X-disp. ratio | 0.359 ± 0.013 | 0.358 ± 0.004 | **0.306 ± 0.003** |
| Settling (within 2mm) | not reached (any config) | not reached | not reached |

Plots: `docs/scientific_4ch_plots/phase5_pareto_comparison.png` (3
separate 2D Pareto views, no scalar collapse),
`docs/scientific_4ch_plots/phase5_step_response.png` (representative
time series per config, master dashed / slave solid).

**Settling-time caveat, not hidden**: the fixed 2mm epsilon this analysis
chose never actually resolves in ANY configuration, because every
configuration's steady-state signed bias (2.7-6.4mm) already exceeds
2mm — **the 2mm threshold itself was an arbitrary, undeclared choice by
this analysis**, not derived from any requirement. Reported as `NaN`
rather than silently switched to a looser threshold that would happen to
"pass" — a properly derived settling criterion is
`ASSUMPTION_REQUIRES_USER_DECISION` (see below), not something to
quietly pick after seeing it doesn't resolve.

## Interpretation — a real Pareto tradeoff, not a single winner

- **`symmetric_stiff` dominates on tracking fidelity**: lowest RMSE, MAE,
  peak error, overshoot, and the reach ratio closest to the ideal 1.0
  (current baseline and soft configs both substantially OVERSHOOT the
  master's actual displacement — a genuine, quantified tracking-fidelity
  problem with the current asymmetric baseline, not previously measured).
  It also has the lowest cross-axis coupling.
- **`baseline_asymmetric` (current) dominates on responsiveness**: lowest
  lag (18ms vs. 31ms) and fastest master rise time (70ms vs. 127ms for
  `symmetric_stiff`) — the master itself moves and settles faster under
  the softer baseline gain, which is presumably part of why it was
  chosen (a human hand feels a faster-responding master).
- **`symmetric_soft` is dominated** — worse than baseline on every
  tracking-fidelity metric AND no better on lag/responsiveness than
  `symmetric_stiff`. No evidence supports this configuration on any
  axis measured. (Included as the third natural point in the 2×2 gain
  family per the task's own instruction to test symmetric/asymmetric
  families, not because it was expected to win.)

**This is quantitative evidence bearing on the current asymmetric
baseline** (per the task's "any asymmetry that survives must have
quantitative evidence" rule): the asymmetry is NOT free — it buys real
responsiveness (lag, master rise time) at a real, now-measured cost in
tracking fidelity (RMSE, overshoot, reach ratio, cross-axis coupling).
Whether that tradeoff is the right one is a preference question this
document does not answer (see below).

## `ASSUMPTION_REQUIRES_USER_DECISION`

**Settling-time criterion: RESOLVED by explicit user instruction
(2026-09-09) — do not invent one.** The 2mm epsilon below (and any
replacement value, including an 8mm or percentage-based alternative) is
rejected. Settling time is reported as `INSUFFICIENT_REQUIREMENT`
throughout this document; rise time, RMSE, MAE, overshoot, reach ratio,
lag, bias, and cross-axis coupling remain the valid metrics.

**Which point on the Pareto front to prefer** (responsiveness vs.
tracking fidelity) is a stability/transparency preference the task's own
rules reserve for explicit user decision — this document presents the
tradeoff, quantified, and does not select a winner. See the "Updated
Pareto picture" section below for the X/Y/Z-informed version of this
question.

## What this phase does NOT establish

- Y/Z-axis-specific step responses (only X was driven; cross-axis
  coupling was measured as a side effect, not as a primary driven test
  of Y/Z tracking).
- Saturation/travel-limit activity under this specific push magnitude
  (0.4N was chosen to produce clear step responses without saturating
  `--left-max-offset-m=0.08` or `max_position_error_m`; not verified this
  session whether a larger, more provocative push changes the ranking).
- Control effort (integral of applied force) — not computed this pass;
  could be added if it becomes the operative question for Phase 6.
- Any claim about force-channel (C1/C4) interaction with these dynamics
  — by design, out of scope for Phase 5.

## Extension — matched Y/Z-axis experiments (2026-09-09, per user instruction)

**Per explicit instruction: no final Pareto operating point was chosen
from the X-only result. This extension adds matched Y and Z driven
experiments (same 3 configs, same N=5, same push magnitude 0.4N, same
methodology) before any preference decision.** The 2mm settling-time
epsilon is **rejected outright** (not replaced with 8mm or a percentage)
— settling time is reported as `INSUFFICIENT_REQUIREMENT` throughout,
including for the original X results above (superseded by this
section). Rise time, RMSE, MAE, overshoot, reach ratio, lag, bias, and
cross-axis coupling remain independently valid and are reported for all
three driven axes.

Scripts: `analysis/scientific_4ch/phase5_position_subsystem.py <axis>`
(generalized to any driven axis, same `arm_teleop` fix carried over),
`analysis/scientific_4ch/phase5_analysis_xyz.py`. 30 additional sim runs
(N=5 × 3 configs × 2 new axes), zero errors.

### X-axis (restated from above, now alongside Y/Z)

| Metric | baseline_asymmetric | symmetric_soft | symmetric_stiff |
|---|---:|---:|---:|
| RMSE (mm) | 3.02 ± 0.05 | 6.62 ± 0.04 | **2.80 ± 0.01** |
| Overshoot (%) | 14.49 ± 0.53 | 35.28 ± 0.30 | **7.97 ± 0.62** |
| Reach ratio | 1.394 ± 0.053 | 1.407 ± 0.031 | **1.046 ± 0.017** |
| Lag (s) | **0.018 ± 0.002** | 0.031 ± 0.002 | 0.031 ± 0.002 |
| Master rise (s) | **0.070 ± 0.001** | 0.071 ± 0.002 | 0.127 ± 0.003 |
| Cross → Y | 0.312 ± 0.011 | 0.311 ± 0.003 | **0.265 ± 0.003** |
| Cross → Z | 0.047 ± 0.002 | 0.047 ± 0.000 | **0.041 ± 0.000** |

### Z-axis

| Metric | baseline_asymmetric | symmetric_soft | symmetric_stiff |
|---|---:|---:|---:|
| RMSE (mm) | 4.29 ± 0.12 | 7.62 ± 0.02 | **3.23 ± 0.08** |
| Overshoot (%) | **11.37 ± 0.26** | 12.69 ± 0.08 | 5.76 ± 0.02 |
| Reach ratio | 0.984 ± 0.0003 | 0.961 ± 0.0005 | **0.989 ± 0.0005** |
| Lag (s) | 0.0132 ± 0.0018 | 0.0179 ± 0.0006 | **-0.0037 ± 0.0064** (slave keeps pace or leads) |
| Master rise (s) | **0.607 ± 0.006** | 0.768 ± 0.002 | 0.833 ± 0.015 |
| Cross → X | 0.741 ± 0.019 | 0.730 ± 0.004 | **0.451 ± 0.011** |
| Cross → Y | 0.0108 ± 0.0002 | **0.0046 ± 0.0000** | 0.0051 ± 0.0000 |

**On Z, `symmetric_stiff` is essentially strictly better** — lowest
RMSE, lowest overshoot, best reach ratio, AND best (most negative/near-
zero) lag, plus lowest X-coupling. The X-axis tradeoff (baseline wins
lag) does **not** reproduce on Z: there, stiff wins lag too. Only
`master_rise_s` (a diagnostic, not one of the task's requested transparency
metrics) is slower for stiff on Z, same pattern as X.

### Y-axis — data collected, but flagged low-confidence, not a clean result

| Metric | baseline_asymmetric | symmetric_soft | symmetric_stiff |
|---|---:|---:|---:|
| RMSE (mm) | 0.03 ± 0.00 | 0.01 ± 0.00 | 0.05 ± 0.00 |
| Master rise (s) | 2.75 ± 0.03 | 2.96 ± 0.01 | 3.08 ± 0.03 |
| Reach ratio | 0.990 ± 0.004 | 0.917 ± 0.003 | 1.012 ± 0.003 |
| Cross → X | **10.83 ± 0.05** | 11.67 ± 0.02 | 11.97 ± 0.04 |
| Cross → Z | 0.316 ± 0.002 | 0.340 ± 0.001 | 0.349 ± 0.001 |

**This is not a clean step response and should not be read with the same
confidence as X/Z.** The same 0.4N push produces net Y displacement in
the tens-of-micrometers range (RMSE ~10-50 μm, vs. 3-8mm for X/Z — two to
three orders of magnitude smaller) and a master rise time of ~2.75-3.08s
— close to the ENTIRE 4.0s push duration, meaning the arm is still
slowly creeping when the push ends, not settled into a clean step. **The
cross-axis finding is the one robust result from this axis**: driving Y
produces 10.8-12.0× MORE slave motion on X than on Y itself — an order
of magnitude larger than any other cross-coupling measured in this
project (X→Y: 0.27-0.31, Z→X: 0.45-0.74). This is consistent with, and
adds new simulation evidence to, this project's own extensively
pre-documented Y-axis difficulties (base-tilt/gravity-axis coupling,
`Y_AXIS_DIAGNOSIS.md`, `Y_AXIS_GRAVITY_SWEEP.md`) — **but whether the
tiny net Y displacement reflects a genuine kinematic property of this
arm at its rest pose (low mechanical advantage in Y) or simply that
0.4N is too small a push for Y specifically (untested: whether a larger
push produces a cleaner Y step response) is not resolved by this data.**
Flagged, not concluded.

### `symmetric_soft` — dominance status across axes

**`DOMINATED` on X and Z** (robust, clean data): worse than both
alternatives on RMSE, overshoot, and reach ratio in both axes, and no
better than `symmetric_stiff` on lag/responsiveness in either. **No
replacement candidate is proposed** — per instruction, not inventing
one. Y-axis data does not contradict this (soft is not clearly better on
anything there either) but is too low-confidence to add to the dominance
claim.

### Updated Pareto picture (X + Y + Z together)

The **X-axis tradeoff is genuine and axis-specific, not general**: on X,
`baseline_asymmetric` wins on lag/master-rise while `symmetric_stiff`
wins on tracking fidelity. On Z, `symmetric_stiff` wins on essentially
everything measured, including lag — the tradeoff narrows to just
`master_rise_s` (a diagnostic property of the master's own local
response, not itself one of the task's core transparency metrics). Y is
inconclusive. **A genuine, still-unresolved tradeoff remains on X**
(and, to a lesser extent, on master responsiveness generally) — per
instruction, this is reported and NOT resolved by this document.

Updated plots: `docs/scientific_4ch_plots/phase5_pareto_xyz.png` (3
Pareto views across all 3 axes × 3 configs, marker shape = axis, color =
config), `docs/scientific_4ch_plots/phase5_cross_axis_xyz.png` (full
cross-coupling bar chart, all driven directions, not just X).

## Y-axis anomaly investigation (2026-09-09, per user instruction — resolved)

**Question**: is Y's near-zero net displacement and 10.8-12.0× Y→X
cross-coupling a push-magnitude artifact (0.4N too small for Y
specifically), or a genuine dynamic property of this arm at its rest
pose?

**Method**: `baseline_asymmetric` only (config comparison is not the
question here), driven axis Y, 6 push magnitudes: 0.1, 0.2, 0.3, 0.4,
1.0, 2.0 N, N=5 repeats each (30 sim runs,
`analysis/scientific_4ch/phase5_y_magnitude_sweep.py` +
`phase5_y_subclamp_raw.json`).

**Confound found and reported, not hidden**: the master `SimNode`'s
`max_force_n` was never overridden in any Phase 5 script and defaults
to **0.45N** (`touch_slave.cpp`'s struct default). 1.0N and 2.0N gave
statistically indistinguishable Y displacement (0.6301 ± 0.0014mm vs.
0.6305 ± 0.0010mm) — the classic signature of a hard input clamp, not
genuine dynamic saturation. **This means the 1.0N/2.0N points do not
test what they appear to** — both are silently clamped to 0.45N before
ever reaching the arm's dynamics.

**Result, using only the 4 sub-clamp magnitudes (0.1-0.4N) where the
push force is genuinely what's requested:**

| Push (N) | Y displacement (mm) | X/Y cross-coupling ratio |
|---:|---:|---:|
| 0.1 | 0.0787 ± 0.0014 | 13.07 |
| 0.2 | 0.1596 ± 0.0002 | 12.21 |
| 0.3 | 0.2870 ± 0.0019 | 11.59 |
| 0.4 | 0.4879 ± 0.0030 | 10.80 |
| ~~1.0~~ (clamped) | 0.6301 ± 0.0014 | 10.33 |
| ~~2.0~~ (clamped) | 0.6305 ± 0.0010 | 10.31 |

**Answer: NOT a push-magnitude artifact.** Y displacement scales
roughly linearly with force below the clamp (consistent with ordinary
small-signal spring-like behavior — no sign of the arm "waking up" at
higher force), and the cross-coupling ratio stays in the same 10-13×
order of magnitude across the ENTIRE tested range, including the
*smallest* magnitude (0.1N) — if anything, the relative disadvantage for
Y is slightly *larger* at low force (13.07×) than at 0.4N (10.80×), the
opposite of what a "just needs a bigger push" explanation would predict.

**Leading explanation (well-evidenced, not fully derived from first
principles)**: Y is very likely this arm's vertical/gravity-loaded axis.
This is independently consistent with `run_bridge.sh`'s own
`SLAVE_GRAVITY_AXIS=1` (index 1 = Y) convention and this project's
extensive pre-existing Y-axis documentation
(`Y_AXIS_DIAGNOSIS.md`, `Y_AXIS_GRAVITY_SWEEP.md`). A Cartesian force in
a gravity-loaded direction must do work against the arm's own
gravitational torque, not just inertia/damping — producing much smaller
net displacement per newton than a gravity-neutral direction (X, closer
to a waist/yaw rotation about the vertical axis, per `jacobian()`'s own
structure: the Y row of `J` has a zero waist-velocity coefficient,
meaning Y motion is necessarily shoulder/elbow-driven, exactly the joints
gravity acts on). **This is a physically coherent, evidence-consistent
explanation, not a numerically derived proof** — a full account would
need the arm's actual mass matrix / gravity model, not attempted here.

**Consequence for Phase 5's Pareto question**: none — the X-axis
tradeoff (baseline vs. `symmetric_stiff`) is unaffected by this finding.
The Y-axis anomaly is now characterized (magnitude-independent, likely
gravity-related) rather than open, but remains its own separate item —
possibly relevant to future C2/C3 Y-specific tuning or to the existing
Y-axis gravity-compensation work, not something this document resolves
further.

**New, previously-undocumented fact surfaced**: the master `SimNode`'s
default `max_force_n=0.45N` was silently active throughout ALL of Phase
5 (X, Y, Z) — harmless for the 0.4N pushes used elsewhere (just under
the ceiling), but worth recording since it wasn't an explicit,
deliberate choice.

## Pareto decision (user, 2026-09-09)

**Tracking fidelity prioritized over responsiveness.** User selected
`symmetric_stiff` (K=35.0 N/m, D=0.20 Ns/m, both master and slave) as
the preferred scientific operating point, after reviewing the full X/Y/Z
evidence above and the resolved Y-axis anomaly (confirmed unrelated to
this choice).

**What this changes concretely**: only the **master's** gains move, from
the current baseline (15.0 N/m, 0.10 Ns/m) to (35.0 N/m, 0.20 Ns/m) —
the **slave's** gains are already 35.0/0.20 in `baseline_asymmetric` and
are unchanged by this decision. `symmetric_soft` remains `DOMINATED`,
not selected.

**Status: `SELECTED (simulation-based Pareto decision)`, NOT applied to
the shipped launch configuration.** This session did not edit
`run_master_left.sh`'s `LEFT_STIFFNESS_N_PER_M`/`LEFT_DAMPING_N_S_PER_M`
defaults (15.0/0.10) — deploying this choice to the real launch config is
a separate, hardware-consequential action not requested here, and this
task's own scope is simulation/offline only throughout.
`REQUIRES_FUTURE_HARDWARE_VALIDATION` before this gain change is trusted
on physical devices — everything above is `hd_sim` evidence only, not a
claim about real Touch device dynamics or a human operator's felt
experience.

## Stop condition

No hardware used. `touch_slave_sim`/`hd_sim` + `bridge_master_slave.py`
only throughout, including the Y-axis follow-up. No shipped launch
script was edited. The Pareto decision is recorded, not yet deployed.
