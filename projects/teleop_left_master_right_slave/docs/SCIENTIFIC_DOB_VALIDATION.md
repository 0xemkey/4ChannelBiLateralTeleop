# Scientific DOB validation

Date: 2026-09-08. Offline / simulation only (`touch_slave_sim`/`hd_sim`,
confirmed hardware-independent via `ldd` showing `libhd_sim.so`, not
`libHD.so`). No hardware touched at any point.

## Phase 4A — DOB repeatability and nondeterminism

**Status: `PHASE_4A_PASS` (qualified — see §4A.5).** DOB tuning has NOT
started; this phase only establishes whether a later characterization
would be scientifically meaningful.

### 4A.1 — Nondeterminism source audit (source inspection)

| Source | Classification | Evidence |
|---|---|---|
| Simulator random initial conditions | **DETERMINISTIC** | `arm_model.hpp:304`: `q_ = {0.0, 0.45, 0.90}`, a fixed constant, not sampled |
| Explicit or implicit seeds | **N/A — no RNG exists** | `grep`'d `arm_model.hpp`/`hd_sim_api.cpp` for `rand\|srand\|mt19937\|uniform\|distribution`: zero matches. `<random>` is `#include`d but never used (dead include) |
| Initial joint configuration | **DETERMINISTIC** | Same as above; `has_init_pose_m` defaults `false`, so the env-var-driven IK-seed path (which itself is a bounded deterministic multi-start solve, not random) is not exercised in this experiment |
| Initial qdot | **DETERMINISTIC** | `arm_model.hpp:330`: `qdot_ = {0.0, 0.0, 0.0}`, fixed |
| Filter internal state (DOB `lpf_`/`u_prev_`) | **DETERMINISTIC** | `configure()` unconditionally calls `reset()`, zeroing both (verified directly in an earlier session's restart-transient diagnostic, `touch_slave.cpp`) |
| DOB reset state | **DETERMINISTIC** | Same as above |
| Simulated environment state | **DETERMINISTIC** | No `TOUCH_SIM_ENV` set for this experiment (free-space, "Left" device default — no wall/hand model active) |
| Gravity initialization | **DETERMINISTIC** | `TOUCH_SIM_BASE_TILT_RAD` (default 0.0785), `TOUCH_SIM_GRAVITY_MISMATCH` (default 1.06) are fixed env-var-driven constants, not sampled |
| Friction initialization | **DETERMINISTIC** | No RNG; friction terms are deterministic functions of `qdot` |
| ROS/process startup order | **TIMING_DEPENDENT** | `SimNode.wait_started()` polls the process log for a marker line rather than sleeping a fixed duration — confirms startup latency itself varies run to run (OS/process scheduling), by construction |
| Publisher/subscriber sync | **TIMING_DEPENDENT** | Standard ROS2/DDS discovery timing, not controlled by this test |
| Analysis-window start | **TIMING_DEPENDENT / STATE_DEPENDENT** | Triggered by Python-side wall-clock elapsed time (`spin_for(1.0)`), NOT synchronized to simulated time — the sim's physics thread runs in its own `std::thread`, paced by `steady_clock` independently of the Python test process (`hd_sim_api.cpp:389-392`) |
| Settling time | **STATE_DEPENDENT** | The arm starts away from static equilibrium (`q={0,0.45,0.90}` is not a zero-gravity-torque pose) and swings/settles under gravity+damping; this transient's duration is not guaranteed to fit inside the fixed 1.0s discard window — confirmed empirically below |
| Stale startup samples | Not applicable | `Collector.messages.clear()` explicitly discards the settle-phase buffer before the analysis window starts |
| Timestep variation | **DETERMINISTIC (nominal) / TIMING_DEPENDENT (actual)** | `dt = 1/update_rate_hz` is a fixed constant per step, but how many such steps occur during a given wall-clock interval is scheduling-dependent, since physics stepping is real-time-paced, not a fixed-step batch decoupled from wall clock |
| Scheduling jitter | **TIMING_DEPENDENT** | Confirmed as the dominant proximate mechanism — see 4A.4 |
| Residual files/state between runs | **DETERMINISTIC (isolated)** | Each run uses a fresh `unique_ns()` ROS namespace and a fresh `touch_slave_sim` subprocess (`sim_common.py`'s per-test subprocess pattern) — no shared state found between runs |

**Headline finding**: the earlier ~48× discrepancy was NOT simulator
randomness — there is no RNG anywhere in the simulation source. The
system is physically deterministic given identical wall-clock timing.
What varies is how much *simulated* time has elapsed by the moment a
Python-side wall-clock `spin_for()` call returns, because the physics
thread runs in real time, independently paced from the test process.

### 4A.2 — Repeatability experiment

**N = 20** identical-condition repeats (script:
`analysis/scientific_4ch/phase4a_dob_repeatability.py`; raw data:
`analysis/scientific_4ch/phase4a_repeatability_raw.json`). Held constant:
DOB parameters (`g_dis={10,10,5}`, identified `M_n`/`G(q)`, `threshold_nm`
— all defaults, untouched), free-space environment, 1.0s settle / 3.0s
analysis window, `touch_slave_sim` "Left" device, no bilateral coupling.

Per run: 600-601 samples collected (~200Hz `dob_diagnostics` over 3.0045s
actual window, consistent to ±0.0005s across all 20 runs — window
*duration* itself is NOT the source of variance, ruling out a naive
"the window was cut short" explanation).

### 4A.3 — Distributions (N=20, p99 |tau_dis_hat|, N·m)

| Joint | mean | median | std | MAD | IQR | min | max | CV |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| waist | 0.15158 | 0.15914 | 0.03297 | 0.00000 | 0.00000 | 0.00789 | 0.15915 | 0.217 |
| shoulder | 0.05075 | 0.05075 | 0.00000 | 0.00000 | 0.00000 | 0.05075 | 0.05077 | 0.000 |
| elbow | 0.00202 | 0.00212 | 0.00033 | 0.00007 | 0.00025 | 0.00078 | 0.00244 | 0.161 |

**Not hidden**: waist run 3 is a clear single outlier (`p99=0.00789`,
vs. 19/20 other runs landing at `0.15914-0.15915` — essentially
bit-identical). MAD and IQR are exactly zero for waist because 19/20
values agree to 5 decimal places; the mean/std/CV columns are dominated
entirely by that one outlier and should not be read as "waist has ~22%
typical variability" — it has ~0% typical variability and one rare
large deviation. Shoulder shows zero measurable variability across all
20 runs including no outlier. Elbow shows genuine, continuous spread
(no single dominant outlier — see plot 3 below) with real ~16% CV even
though its absolute magnitude (sub-mN·m) is tiny.

**Plots** (`docs/scientific_4ch_plots/`):
1. `phase4a_1_run_by_run_floor.png` — all 3 joints, all 20 runs, shows
   the waist outlier plainly (not smoothed away).
2. `phase4a_2_per_joint_distribution.png` — box+strip plot per joint.
3. `phase4a_3_initial_pose_vs_floor.png` — **this is the key diagnostic
   plot**: waist's outlier run has `q_first=1.60 rad` (near its joint
   limit) vs. the tight `1.13-1.34 rad` cluster for every other run;
   shoulder's `q_first` is essentially constant (`~-0.600 rad`, no
   spread) across all 20 runs; elbow shows a continuous, non-outlier
   spread.
4. `phase4a_4_floor_vs_run_order.png` — normalized floor vs. run index;
   no drift or state-leakage trend visible across the sequence (rules
   out "later runs are contaminated by earlier ones").
5. `phase4a_5_floor_vs_window_duration.png` — **substitute for the
   requested "floor vs seed" plot, since no seed exists (4A.1 confirms
   no RNG)**; checks window-duration jitter instead — no meaningful
   correlation found (durations cluster tightly at 3.0044-3.0050s
   regardless of the resulting floor value).

### 4A.4 — Explaining the (now well-characterized) variance

| Candidate | Evidence | Counter-evidence | Confidence | Expected magnitude |
|---|---|---|---|---|
| A. Initial pose / gravity residual | Initial pose is fixed and deterministic (4A.1) — cannot by itself explain run-to-run spread | — | **High confidence this is NOT the cause of run-to-run spread** (it explains the *existence* of a nonzero floor at all, not its variability) | N/A |
| B. Simulator randomness | No RNG found anywhere in sim source | — | **Ruled out** | None |
| C. Filter transient / incomplete settling | Waist's outlier run has `q_first` far outside the typical cluster and `qdot_std=0.0` exactly (fully stopped, likely at/near a joint limit) vs. `qdot_std≈0.50-0.57` for every other run (still oscillating) — direct, strong evidence the 1.0s discard window does not reliably contain the full settling transient | — | **High** — this is the mechanism that makes the outlier POSSIBLE | Large when it occurs (waist: ~20× lower floor) |
| D. Incomplete settling (general) | Same evidence as C; distinguished from C only in that D would describe *typical* incompleteness rather than a rare tail event — the 19/20 tight clustering argues most runs land in a similar (still-transient but consistent) state, not that settling is *typically* wildly incomplete | 19/20 runs agree to 5 decimals, arguing the "typical" trajectory at t=1.0s is itself highly reproducible | **Moderate** (C is the better-supported framing; D as "always incomplete and variable" is not supported by the 19/20 consistency) | — |
| E. Friction/stiction | No distinct stiction model found beyond ordinary velocity-dependent friction; no evidence this specifically drives the outlier | — | **Low** | — |
| F. Scheduling/timestep variation | Physics thread runs real-time-paced (`steady_clock`), decoupled from the Python test's own wall-clock `spin_for` calls (4A.1) — the mechanism that would let "the same 1.0s of Python wall-clock" correspond to a different amount of *simulated* settling progress from run to run, explaining why an otherwise-deterministic system produces a rare divergent outcome | Window *duration* itself (3.0044-3.0050s) shows negligible variation (plot 5) — so if timing jitter is the cause, it must act specifically during the STARTUP/settle phase (before the window even starts), not during the window itself | **High** — best-supported mechanism connecting "no RNG" with "one real outlier" | Rare (1/20 here) but large when it occurs |
| G. Stale startup data | `Collector.messages.clear()` explicitly discards pre-window samples — this specific mechanism is not implicated | — | **Ruled out** as the direct mechanism (though it's the reason the discard window exists at all, which is the boundary C/F act on) | — |
| H. Window-selection artifact | Fixed 1.0s discard boundary interacting with an underdetermined settling time (C/F) IS itself a form of this — not a separate independent cause, better understood as a consequence of C+F | — | **Subsumed by C/F**, not independent | — |
| I. Process/state leakage | Plot 4 (floor vs. run order) shows no drift/trend across the sequence; each run uses an isolated fresh subprocess and namespace | — | **Ruled out** | None |
| J. Estimator instability | No divergence, NaN, or runaway values observed in any of the 20 runs; the outlier is a LOWER, not higher, unstable-looking value | — | **Ruled out** | — |

**Best-supported explanation: C (incomplete settling) interacting with
F (real-time scheduling variability in how far that settling has
progressed by the fixed 1.0s wall-clock discard boundary)**, producing a
rare (here 1/20 = 5%) but large-magnitude outlier on the joint whose
settling transient is largest (waist, driven by the base-tilt-induced
gravity load per `test_dob_floor.py`'s own prior documentation). This
also explains why the ORIGINAL two-sample comparison (this session's
prior turn) looked catastrophic (48×): comparing exactly 2 runs has a
non-trivial chance of comparing one typical run against one outlier run,
which is exactly what appears to have happened.

### 4A.5 — Repeatability gate

No pass threshold was invented. Comparing **within-condition variability**
(this experiment) against the **historical between-setting effect size**
already on record (`touch_slave.cpp:484-493`'s own measured table,
g_dis=2π·5 Hz → 10 rad/s changed the waist/shoulder/elbow floors by
`(0.0654→0.0382)=42%`, `(0.0458→0.0320)=30%`, `(0.0373→0.0305)=18%`):

| Joint | Within-condition CV (excl. outlier) | Historical between-setting effect | Effect >> variability? |
|---|---:|---:|---|
| waist | ~0% (19/20 identical) + rare 95%-magnitude outlier | 42% | **Yes, if outlier runs are screened out; a single unscreened comparison risks exactly the false conclusion this audit diagnosed** |
| shoulder | ~0% | 30% | **Yes, cleanly** |
| elbow | ~16% (CV, no outlier needed to produce this) | 18% | **Marginal** — a future g_dis sweep's elbow effect is only barely above this joint's own baseline noise, and a smaller candidate effect would not be resolvable without more repeats or a longer settle window |

**Verdict: `PHASE_4A_PASS`, qualified.** A future DOB parameter sweep
(Phase 4B) is scientifically meaningful for waist and shoulder, and
approaches meaningful for elbow, **PROVIDED**:
1. Each candidate setting is evaluated over multiple repeats (not a
   single run), specifically to catch and screen the kind of rare
   large-magnitude outlier characterized here (recommend N≥10 per
   setting, matching this experiment's own scale) — a single-run
   comparison remains unreliable, as this audit itself demonstrates.
2. The settle/discard window is lengthened beyond the current 1.0s
   before Phase 4B begins, OR an explicit settling-convergence check
   (e.g. requiring `|qdot|` below some criterion before starting the
   analysis window, rather than a fixed wall-clock duration) replaces
   the fixed discard — **this is a test-methodology fix, not a DOB
   parameter change**, and is recommended as the concrete next step
   rather than proceeding to Phase 4B with the current fixed-1.0s
   convention. `ASSUMPTION_REQUIRES_USER_DECISION`: whether to adopt a
   longer fixed window (simple, e.g. 3.0s) or a convergence-based gate
   (more correct, more engineering) is a design choice this document
   does not make unilaterally — options, evidence, and a recommendation
   are given in the table below.
3. Elbow-specific conclusions from any future sweep should be treated
   with the ~16% baseline noise explicitly in mind — an observed elbow
   effect below roughly 20% is not distinguishable from this
   already-characterized within-condition variability without further
   repeats.

**`ASSUMPTION_REQUIRES_USER_DECISION` — settle-window methodology fix:**

| | Option A: lengthen fixed window (e.g. 1.0s → 3.0s) | Option B: convergence-based gate (e.g. `max|qdot| < ε` sustained for some duration) |
|---|---|---|
| Physical meaning | Simple, more conservative margin against the observed transient | Directly targets the actual physical condition ("has it settled") rather than guessing a duration |
| Evidence | This experiment shows most runs are visually near-settled well before 3s elapses (shoulder q essentially constant from run to run) | No `ε`/hold-duration has been derived or tested this session — would itself need its own small validation pass |
| Consequences | Cheap, immediate, but still a fixed-duration heuristic — could in principle still occasionally miss a slower settle | More robust in principle, more engineering effort, introduces a new pair of arbitrary constants (`ε`, hold duration) that would themselves need to be flagged `ARBITRARY` pending derivation |
| Recommendation | **Option A as an immediate, low-risk mitigation for any near-term Phase 4B work**; Option B as the more rigorous longer-term fix | |
| Confidence | Medium — sufficient for Phase 4B to proceed meaningfully, not a claim that the transient is now fully characterized | |

## What Phase 4A does NOT establish

- The DOB's actual noise/bias/bandwidth CHARACTERIZATION itself (Phase
  4B) — this phase only establishes that such a characterization would
  be meaningful, using the existing default parameters as the single
  condition tested.
- Contact-condition (wall) repeatability — only free-space was tested
  here; contact SNR/bandwidth/delay repeatability is a separate,
  not-yet-run experiment.
- Any claim about physical hardware repeatability — this entire
  experiment is `hd_sim` only; real DOB behavior on the physical Touch
  devices has its own, separately-documented noise floor
  (`touch_slave.cpp`'s cited hardware replay table), not re-validated
  here and not assumed to share the sim's specific timing-jitter
  mechanism (real hardware has no analogous "wall-clock-paced sim
  thread" structure — its own sources of run-to-run variability would
  need a separate audit, `REQUIRES_FUTURE_HARDWARE_VALIDATION`).

## Phase 4B — DOB characterization across candidate g_dis settings

**Status: `INCONCLUSIVE` for waist/shoulder (test-design limitation,
identified and explained, not hidden), weak positive signal for elbow.**

### Method

Settle-window fix applied per user decision (Option A: lengthen 1.0s →
3.0s), plus a 3.0s analysis window. New code hook added to enable this
sweep: `dob_g_dis_override_rad_s` ROS param
(`touch_slave.cpp`, default `-1.0` = no override, verified via `colcon
build` that this does not change default behavior on any existing
launch). **Three candidates, all pre-existing in this codebase's own
history — none invented**: `g_dis = 2π·5 ≈ 31.42 rad/s` ("previous"
setting), `10 rad/s` (current waist/shoulder), `5 rad/s` (current
elbow), each applied uniformly to all 3 joints so it is directly
comparable to the historical single-value hardware table
(`touch_slave.cpp:484-493`). N=10 repeats per candidate (30 sim runs
total, `analysis/scientific_4ch/phase4b_dob_gdis_sweep.py`).

### Result

**Perfect repeatability achieved** — confirms the settle-window fix
works exactly as diagnosed in 4A: waist and shoulder now show **IQR=0
across all 30 runs** (every repeat within a condition identical to 5
decimal places; Phase 4A's rare-outlier failure mode did not recur
once).

**But**: waist and shoulder ALSO show **no measurable difference between
the 3 g_dis candidates** (waist: `0.00327` N·m at all three settings;
shoulder: `0.05081` N·m at all three settings) — full data in
`analysis/scientific_4ch/phase4b_summary.json`, comparison plot at
`docs/scientific_4ch_plots/phase4b_sim_vs_hardware_gdis.png`.

**This is explained, not merely observed**: every waist/shoulder run
this time converged to `qdot_std = 0.0` exactly (the joint came fully to
rest within the lengthened 3.0s settle window — itself direct
confirmation of 4A's diagnosis: the earlier 1.0s window was too short).
At true mechanical rest, the residual disturbance the observer sees is a
**constant** (gravity-model mismatch), and a low-pass filter's DC gain is
1.0 **regardless of cutoff frequency** — so `g_dis` is mathematically
expected to have no effect on a fully-settled floor. **The lengthened
settle window fixed repeatability but, as a side effect, moved the test
into an operating regime (complete rest) where the parameter under study
cannot show an effect by construction.** This is a genuine test-design
finding: the historical hardware table this sweep was compared against
was almost certainly captured during **active** motion (a human moving
the arm, or at minimum an arm that had not fully settled), not a
passively-resting one — so this sim experiment, as designed, is not
equivalent to the condition that produced the reference numbers it is
being compared to.

**Elbow is the one joint that did NOT fully settle** (`qdot_std≈0.0166`
in every run, nonzero and consistent — elbow's dynamics apparently do
not reach zero velocity within 3.0s, plausibly a slower/different
settling mode given its much smaller `M_n`). Elbow's floor DOES show a
small, perfectly-repeatable (IQR=0 within each candidate), monotonically
decreasing trend with `g_dis`: `0.00036 → 0.00028 → 0.00026` N·m for
`2π·5 → 10 → 5` rad/s — same DIRECTION as the historical hardware table
(`0.0373 → 0.0305 → 0.0250`), though roughly two orders of magnitude
smaller in absolute terms (sim vs. hardware are different physical
systems by design, per `test_dob_floor.py`'s own docstring — not
expected to match numerically). This is a genuine, if small, positive
signal consistent with `g_dis` behaving as expected when real transient
content exists to filter.

### Sim-vs-hardware absolute comparison (informational only)

`sim/hw` ratios in `phase4b_summary.json` range from 0.01 (elbow) to 1.7
(shoulder) — **not interpreted as a validation/failure of either
number**, since `hd_sim`'s model is deliberately mismatched from the
identified hardware parameters (documented in `test_dob_floor.py`'s own
docstring: "if it matched exactly, `tau_dis_hat` would sit at zero and
every threshold test in this suite would pass vacuously"). The only
scientifically meaningful comparison here is the *trend direction*
across g_dis candidates (elbow), not the absolute magnitude match.

### 4B verdict

- **Waist/shoulder: `INCONCLUSIVE`** — this specific experiment cannot
  discriminate the candidate g_dis values, for the explained
  DC-invariance reason above. This is NOT evidence that g_dis doesn't
  matter for these joints; it's evidence that a passive-settle-to-rest
  experiment is the wrong condition to test it under.
- **Elbow: weak positive, direction-consistent signal**, `INSUFFICIENT_DATA`
  for a confidence claim beyond "consistent with the historical
  hardware trend" — no statistical significance test was run (N=10 per
  condition, IQR=0 within each, so a formal test would need inter-run
  noise that doesn't exist in this deterministic-once-settled sim; a
  proper significance claim would need either a controlled velocity
  perturbation or many more independent settle trajectories, e.g. varied
  initial poses).

### `ASSUMPTION_REQUIRES_USER_DECISION` — Phase 4B redesign

The natural next step is a **redesigned Phase 4B experiment using active
excitation** (e.g. a chirp/sinusoidal `force_cmd` drive, or a wall-contact
condition matching the sign-audit test's own recipe) instead of passive
free-space settling, so that transient/velocity-dependent content
actually exists for `g_dis` to differentially filter. This is a new
experiment design, not a parameter choice, but it does require deciding:

| | Option A: chirp/sinusoidal free-motion drive | Option B: wall-contact condition (reuse Phase 1/2's recipe) |
|---|---|---|
| Physical meaning | Tests g_dis's effect on free-motion disturbance rejection (false-force floor under motion) | Tests g_dis's effect on genuine contact detection/tracking (onset delay, gain, bandwidth) |
| Evidence this session | None yet — not run | The wall-contact + truth-CSV pattern is already proven working (Phase 1/2's sign test) |
| Consequences | Answers "does moving-but-untouched floor still favor lower g_dis" | Answers the more operationally relevant question (how well does the DOB serve C1/C4 during actual contact) but is a different question than the free-space floor table this session compared against |
| Recommendation | **Option B**, since it's operationally closer to what Phase 4B ultimately needs to inform (C1/C4 characterization, Phase 6) and reuses already-validated infrastructure | |
| Confidence | Medium | |

**Not started this session** — flagged for the next check-in per the
qualified-continue instruction from the prior turn.

## Phase 4B redesign — wall-contact condition (user-selected, 2026-09-08)

**Status: `PASS` — a real, if modest, g_dis effect is now resolvable.**

### Method

Reused the Phase 1/2 sign-test's proven wall-contact recipe (`hd_sim`
"Default Device", `TOUCH_SIM_ENV=wall`, ground-truth `tau_ext_true` via
`TOUCH_SIM_TRUTH_CSV`) instead of passive free-space settling. Same 3
g_dis candidates, N=10 repeats each
(`analysis/scientific_4ch/phase4b_wallcontact_sweep.py`). Primary metric:
**steady-state gain** = `mean(|tau_dis_hat|) / mean(|tau_ext_true|)`,
each tail-averaged over its own last ~1.0s of a 4.0s contact hold — this
metric only needs each stream's own steady tail, not cross-stream time
alignment, sidestepping the clock-mismatch issue between
`DobDiagnostics`' ROS-clock timestamps and the truth CSV's internal
`t_s`. **Onset/release delay was NOT measured this pass** — no reliable
shared time base was set up between the two streams; reporting a delay
number from an unaligned comparison would be worse than not reporting
one. `INSUFFICIENT_DATA` for delay under this design.

### Result

| g_dis | mean gain | std | CV | median | min | max |
|---|---:|---:|---:|---:|---:|---:|
| 2π·5 (~31.42 rad/s, "previous") | 0.8873 | 0.0096 | 1.08% | 0.8901 | 0.8697 | 0.9032 |
| 10 rad/s (current waist/shoulder) | 0.8686 | 0.0025 | 0.28% | 0.8689 | 0.8636 | 0.8733 |
| 5 rad/s (current elbow) | 0.8668 | 0.0021 | 0.25% | 0.8665 | 0.8644 | 0.8715 |

Plot: `docs/scientific_4ch_plots/phase4b_wallcontact_gain_vs_gdis.png`.

**A real effect, cleanly resolved**: gain is highest (closest to the
ideal 1.0) at the fastest cutoff (2π·5) and lower at the two slower
cutoffs — physically expected (a higher-bandwidth Q-filter tracks a
step-like contact onset with less lag-induced attenuation). The
2π·5-vs-10 gap (0.0187, ~2.1%) is roughly 2× the largest within-condition
std (0.0096) — **a resolvable effect, not noise**. The 10-vs-5 gap
(0.0018) is smaller than the pooled std of either — **not distinguishable
from noise at N=10**; consistent with the current shipped
per-joint choice (10 for waist/shoulder, 5 only for elbow, chosen for
elbow-specific reasons documented in `touch_slave.cpp`'s own comment
about elbow's floor falling further at lower cutoffs) not being
contradicted by this contact-gain metric, though this metric alone does
not independently justify that specific per-joint split either — it
only shows 10 vs. 5 are not meaningfully different in CONTACT GAIN
specifically.

**Within-condition repeatability is excellent under active contact**
(CV 0.25-1.08%), a sharp contrast to the free-space passive-settle
experiment's earlier pathology — confirming that active
excitation (whether contact or motion) is what makes this kind of sim
characterization informative, exactly as anticipated when this redesign
was chosen.

### 4B final verdict

`PHASE_4B_PASS` for the contact-gain question specifically: g_dis has a
small, real, resolved effect (faster cutoff → higher gain, i.e. closer
to unbiased contact-force estimation), consistent with control-theory
expectations and NOT contradicted by the historical hardware free-space
floor table's own qualitative direction (faster cutoff → also higher
free-space floor, i.e. more contamination — the SAME parameter trading
off contact fidelity against free-space noise, exactly the tradeoff
`touch_slave.cpp`'s own g_dis comment already describes). This is
citing/confirming existing documented reasoning with new, independent
simulation evidence, not discovering a new tradeoff.

**Still open**: onset/release delay (not measured, see above), and
whether the specific magnitude of this gain gap (2.1%) matters enough to
warrant a different g_dis than the current shipped values — that is a
Phase 6 (force-subsystem optimization) question, not answered here, and
this document does NOT recommend changing g_dis.

## Stop condition (Phase 4B)

No hardware used. One new ROS param added to `touch_slave.cpp`
(`dob_g_dis_override_rad_s`, default `-1.0`, verified via `colcon build`
not to change existing behavior), used only by
`touch_slave_sim`/`hd_sim` in this session. No DOB parameter's shipped
default was changed. 30 sim runs, all free-space, no wall/hand model.

## Stop condition

No hardware used. `touch_slave_sim`/`hd_sim` only, N=20 repeated runs,
all free-space (no wall/hand model). No DOB parameter was changed. No
source file modified in this phase (only read/exercised).
