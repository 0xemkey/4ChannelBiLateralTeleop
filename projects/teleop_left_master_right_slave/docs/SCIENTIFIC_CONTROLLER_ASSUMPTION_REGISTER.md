# Scientific controller assumption register

Date: 2026-09-08. **Offline, source-inspection only — no hardware was
started or accessed to produce this document.** This is Phase 0 of the
"Scientific redesign and validation process" task. It inventories every
controller parameter/structural choice currently in the codebase and
classifies its epistemic status. Nothing here is a recommendation to
change anything — classification only.

Classification key: DERIVED / IDENTIFIED / EMPIRICALLY_SUPPORTED /
SAFETY_CONSTRAINT / HEURISTIC / ARBITRARY / UNKNOWN.

## C1 (slave force → master relay)

| Field | Value |
|---|---|
| Name | C1 weight `w1` |
| Current value | 0.5 (`4ch_safe` preset) |
| Code location | `bilateral_4ch.py:81` (`PRESETS["4ch_safe"]`), applied at `bilateral_4ch.py:417,441` |
| Runtime source | `--architecture 4ch_safe` CLI arg, or `--w1` under `--architecture custom` |
| Purpose | Scales slave's estimated external torque before relaying to master |
| Origin | No derivation or sweep found in repo/logs for this specific value |
| Evidence supporting it | None found — no A/B gain sweep exists in `~/dob_logs/` or `docs/` |
| Mathematically derived? | No |
| Experimentally identified? | No |
| Optimized? | No |
| Safety-related? | Partially — kept ≤1.0 and paired with the tank/trip-abort system (`REASON_TRIPPED`, `REASON_TANK_EMPTY`) as a conservative starting point, per `bilateral_4ch.py` "safe" naming |
| Heuristic? | Yes |
| Arbitrary? | **Yes — no evidence found justifying 0.5 specifically over e.g. 0.4 or 0.7** |
| Affects transparency? | Directly — linear multiplier on force transparency error |
| Must be revisited? | **Yes — ASSUMPTION_REQUIRES_USER_DECISION, see below** |
| **Classification** | **ARBITRARY** |

## C4 (master force → slave relay)

| Field | Value |
|---|---|
| Current value | 0.3 (`4ch_safe`) |
| Code location | `bilateral_4ch.py:81`, applied at `bilateral_4ch.py:418,446` |
| Same findings as C1 | No sweep/derivation found; asymmetric vs C1 (0.3 vs 0.5) with no documented rationale for the asymmetry itself |
| **Classification** | **ARBITRARY** |

## Cf (global force-channel scale)

| Field | Value |
|---|---|
| Current value | 1.0 (default) |
| Code location | `bilateral_4ch.py:639` (`--cf`, default 1.0) |
| Purpose | Multiplies both C1 and C4 uniformly |
| Evidence | None found for 1.0 specifically vs any other scale |
| **Classification** | **ARBITRARY** (default; never overridden per logs checked) |

## C1 shoulder mask (`--c1-disable-joints=shoulder`)

| Field | Value |
|---|---|
| Current value | shoulder masked (weight forced to 0.0) |
| Code location | `bilateral_4ch.py:219-227` (`c1_joint_mask`), default `"shoulder"` at `bilateral_4ch.py:643` |
| Purpose | Prevents shoulder's DOB noise floor from being relayed as force |
| Origin | Directly traceable to a **measured** DOB floor: `touch_slave.cpp:481-493` — p99 `|tau_dis_hat|` floors of 0.0654/0.0458/0.0373 N·m (waist/shoulder/elbow) at the previous cutoff, 0.0382/0.0320/0.0305 at the current one — shoulder consistently the worst or near-worst fraction of its own torque budget |
| Mathematically derived? | No |
| Experimentally identified? | **Yes — the floor magnitude that motivates the mask is a measured quantity** (`dob_replay_cpp.cpp` over `dyn_Left.csv`) |
| Optimized? | No — masking (weight=0) is one point on a continuum (see Phase 7 alternatives below), not shown to be optimal, only sufficient |
| Safety-related? | Yes — prevents free-motion noise being rendered as a haptic force |
| Heuristic? | The choice of **hard mask specifically**, vs. a reduced weight or confidence-based scaling, is heuristic |
| Arbitrary? | No — the *need* for some mitigation is evidenced; the *hard-mask-to-zero* form of it is a heuristic choice among alternatives, not arbitrary |
| **Classification** | **IDENTIFIED (need) + HEURISTIC (specific form: hard mask vs. graded alternatives)** |
| Must be revisited? | Yes — Phase 7 requires evaluating graded alternatives before accepting the hard mask as final |

## C2 (slave position → master target)

| Field | Value |
|---|---|
| Name | `LEFT_STIFFNESS_N_PER_M`, `LEFT_DAMPING_N_S_PER_M`, `LEFT_MAX_POSITION_ERROR_M` |
| Current value | 15.0 N/m, 0.10 Ns/m, 0.03 m |
| Code location | `run_master_left.sh:37-39` |
| Purpose | Master-side position-tracking spring/damper toward the slave's (transformed) position |
| Origin | Set deliberately **weaker** than C3 by a documented design comment ("this is a feedback channel felt by a human hand, not a position-tracking slave") |
| Evidence | Qualitative design rationale only — no quantitative comparison of alternative stiffness values against a defined objective was found |
| Mathematically derived? | No |
| Experimentally identified? | No |
| Optimized? | No |
| Safety-related? | Partially (`MAX_FORCE_N=2.0` device ceiling, `LEFT_MAX_POSITION_ERROR_M=0.03` clamp are safety-adjacent) |
| Heuristic? | Yes — "should feel soft to a human hand" is a design intuition, not a derived or measured target |
| Arbitrary? | The specific numbers (15.0, 0.10) are arbitrary; the qualitative direction (softer than C3) is a stated, non-arbitrary design intent |
| **Classification** | **HEURISTIC** |

## C3 (master position → slave target)

| Field | Value |
|---|---|
| Name | `STIFFNESS_N_PER_M`, `DAMPING_N_S_PER_M`, `MAX_POSITION_ERROR_M` |
| Current value | 35.0 N/m, 0.20 Ns/m, 0.04 m |
| Code location | `run_slave_right.sh:21-23` |
| Origin | No derivation/sweep found; values predate this session's work |
| **Classification** | **UNKNOWN origin for the specific numbers; HEURISTIC in spirit (a "should track firmly" intuition), same as C2** |

## DOB — `M_n` (nominal inertia), `g_dis` (Q-filter cutoff), `threshold_nm`

| Field | Value |
|---|---|
| Current value | `g_dis` = {10, 10, 5} rad/s (waist/shoulder/elbow); `threshold_nm` = {0.07632, 0.06400, 0.05000} N·m; `M_n` from `dob_inertia_json` (identified, not inline) |
| Code location | `touch_slave.cpp:499-509` |
| Origin | **`g_dis` explicitly cites a reference paper's per-joint choice for this exact hardware**, then adjusted from an earlier 2π·5 rad/s uniform value after discovering it sat on the arm's 4.4 Hz mechanical resonance (measured). `threshold_nm` = 2× measured p99 floor per joint, **directly computed from logged data** (`dob_replay_cpp.cpp` over `dyn_Left.csv`), not guessed. |
| Mathematically derived? | Partially (Q-filter structure itself is derived, per the header's Ohnishi-form documentation) |
| Experimentally identified? | **Yes, unusually well — this is one of the best-evidenced parameter groups in the codebase**: measured floor table across three cutoff candidates, explicit tradeoff note (elbow's 200ms vs 100ms time constant, contact-onset lag accepted as a cost) |
| Optimized? | Yes, in the informal sense of "compared 3 candidates and picked one by a stated criterion" — not a formal optimization run |
| **Classification** | **IDENTIFIED / EMPIRICALLY_SUPPORTED** — the strongest-evidenced group in this register; still worth re-deriving under Phase 4's formal DOB validation, but not "arbitrary" by any reasonable reading |

## Gravity compensation (`SLAVE_GRAVITY_COMP_N`, axis)

| Field | Value |
|---|---|
| Current value | 0.55 N constant (bridge default) / 0.70 N (empirically preferred per this project's own prior sweep, not yet promoted to the script default), Y-axis only |
| Code location | `run_bridge.sh:69-70` |
| Origin | Empirical sweep (`Y_AXIS_GRAVITY_SWEEP.md`, this project's own prior work) tested 0.40-0.70N; separately, a 17-pose configuration-dependent model was built and **failed its own validation** (`RIGHT_GRAVITY_IDENTIFICATION_FINAL.md`) — constant retained specifically because the more-derived alternative did not survive testing |
| Mathematically derived? | A pose-dependent model was derived but rejected on validation grounds |
| Experimentally identified? | Yes — 0.70N is the best of an empirically swept set, not a guess, though the sweep did not fully converge (own prior documents' own conclusion) |
| **Classification** | **EMPIRICALLY_SUPPORTED (partially — best-of-swept-set, not confirmed-optimal)** |

## Force/torque clamps

| Field | Value |
|---|---|
| `MAX_FORCE_N` (master) | 2.0 N — `run_master_left.sh:26`, comment: "kept above the bridge's REFLECT_MAX_N (1.5N) so the bridge's clamp is the binding one" |
| `MAX_FORCE_N` (slave) | 1.5 N — `run_slave_right.sh:20` |
| `REFLECT_MAX_N` | 1.5 N — `run_bridge.sh:100` |
| `dob_torque_f_max_n` | 0.6 N (default) — `touch_slave.cpp:296`, also drives `kThreshold`/`kMaxTorque` derivation |
| `kMinRelayTorqueNormNm` | 0.02 N·m — `touch_slave.cpp:1272`, a relay-side floor, undocumented derivation found |
| **Classification** | **SAFETY_CONSTRAINT** for the device-level ceilings (explicitly there to stay under the Touch's ~0.88N continuous-force rating, per `run_master_left.sh:72-73`'s own warning check); **ARBITRARY/UNKNOWN** for `kMinRelayTorqueNormNm`'s specific 0.02 value — no derivation found |

## Travel/workspace limits (`MAX_OFFSET_M`, `MAX_OFFSET_XYZ`, `LEFT_MAX_OFFSET_M`)

| Field | Value |
|---|---|
| Current value | `MAX_OFFSET_M`=0.12 (scalar fallback), `MAX_OFFSET_XYZ`=0.12,0.19,0.15, `LEFT_MAX_OFFSET_M`=0.08 |
| Code location | `run_bridge.sh:21,48,93` |
| Origin | Y given a larger allowance (0.19) than X/Z (0.12/0.15) — comment references "vertical needing more range than horizontal," no measurement cited |
| **Classification** | **HEURISTIC** (workspace-shape-informed guess) — this session's own H9 finding (Y master travel-limit fired 54.3% of one test) suggests this specific value may be under-sized relative to actual demand, but that is a new offline finding, not yet a re-derivation |

## Relay deadband (`effective_threshold_nm`)

| Field | Value |
|---|---|
| Formula | `THRESHOLD_NM[j] + MIN_RELAY_TORQUE_NORM_NM / weight` — `bilateral_4ch.py:118-133` |
| Origin | `THRESHOLD_NM` component inherits the DOB's own identified floor (see above); `MIN_RELAY_TORQUE_NORM_NM` component is the same undocumented 0.02 N·m |
| **Classification** | **Mixed: IDENTIFIED (THRESHOLD_NM term) + ARBITRARY (MIN_RELAY_TORQUE_NORM_NM term)** |

## Relay ramp (`--ramp-time-s`)

| Value | 0.5s, `bilateral_4ch.py:675-676` |
| Purpose | Avoid a step force into the operator's hand when a channel weight turns on |
| Evidence | Stated rationale (avoid a step), no measured/optimized time-constant |
| **Classification** | **HEURISTIC** |

## Energy tank (`--tank-capacity-j`, `--tank-recharge-time-s`)

| Value | 0.05 J capacity, 2.0s recharge — `bilateral_4ch.py:680-681` |
| Purpose | Passivity-style energy budget limiting how much active energy C1 can inject |
| Evidence | No derivation from a measured passivity margin or operator-impedance model found |
| **Classification** | **SAFETY_CONSTRAINT in intent, ARBITRARY in specific values** |

## Oscillation abort (`--trip-pp-nm`, `--min-oscillation-hz`)

| Value | 0.4 N·m peak-peak trip, 1.5 Hz minimum oscillation frequency — `bilateral_4ch.py:683,688` |
| Purpose | Detects and aborts a growing force oscillation (limit-cycle protection) |
| Evidence | No sweep/derivation found; `growing` early-warning logic references a fraction of `trip_pp_nm` (see `bilateral_4ch.py:497-508`) but the base threshold itself is not derived |
| **Classification** | **SAFETY_CONSTRAINT (function) / ARBITRARY (specific thresholds)** |

## Sign convention — C1 relay

| Field | Value |
|---|---|
| Documented intent | `bilateral_4ch.py:21`: `tau_m += -w1 * Cf * tau_e_hat` (negative) |
| Actual code | `bilateral_4ch.py:440-441`: `tau_c1 = [w1_eff * self.args.cf * v * m ...]` — **no negation** |
| C4 comparison | Docstring (`+w4`) and code agree — no discrepancy on C4 |
| Test coverage | `test_force_estimate_sign.py` validates only the upstream DOB sign (`tau_ext_hat = -tau_dis_hat`), not this relay step; `test_c1_relay_live.py` checks magnitude/oscillation-abort only |
| Empirical cross-check | This session's `plot6_transparency_proxy.png` (from `FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md`) shows the master's actual C1 Cartesian contribution running **anti-phase** with `-1×slave estimate` — consistent with the un-negated code being what's actually live |
| **Classification (UPDATED 2026-09-08)** | **RESOLVED IN SIMULATION AND CORRECTED.** User decided the scientific target is `F_m ≈ -F_s`; a new hardware-independent sim test (`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`) confirmed the pre-fix code was same-direction (`cos_sim=+0.87`); the one-line negation was applied on branch `c1-sign-fix-sim-validation`; re-run confirmed `cos_sim=-0.87` (anti-parallel, matching target); full 31-test hardware-independent regression suite passed. **Classification changes from UNKNOWN/CORRECTNESS-UNRESOLVED to `IDENTIFIED` (need + direction) `+ SIMULATION-VALIDATED` (the correction itself). `REQUIRES_FUTURE_HARDWARE_VALIDATION` remains for confirming the real device's own force-output sign convention matches `hd_sim`'s.** See `docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md` for full detail. |

## Master/slave frame mapping, Jacobian convention, axis mapping

| Field | Value |
|---|---|
| Code location | `jacobian()`, `cartesian_force_from_joint_torque()` in `plot_phase2_scenario.py`, mirrored in `touch_slave.cpp`'s C++ implementation (previously verified bitwise-identical to the Python port in an earlier session) |
| `T_sm` (slave→master frame transform) | **Not found as an explicit transform anywhere in the codebase** — C1/C4 relay operates in raw joint-torque space (`tau_c1`/`tau_c4` arrays, one array shared by both arms' identical joint ordering), only becoming Cartesian downstream inside each node's own `cartesianForceFromJointTorque(J(q_own), tau)` call using **that node's own current q**, not a cross-device transform |
| Implication | The task's requested `T_sm` is implicitly **identity in joint space**, but the Cartesian projection uses *asymmetric* Jacobians (each side's own current pose) — so `e_F = F_m + T_sm·F_s` as stated in the task cannot be evaluated as a single fixed linear transform; it is inherently pose-dependent on both sides simultaneously |
| **Classification** | **UNKNOWN whether an explicit T_sm was ever intended vs. joint-space identity being the implicit design** — flagged as `ASSUMPTION_REQUIRES_USER_DECISION` (see below), since assuming identity is itself a design choice that hasn't been stated as such anywhere in the repo |

## Filters — velocity estimate (`qdot-alpha`)

| Value | 0.2 (EMA coefficient) — `bilateral_4ch.py:677-679` |
| Purpose | Smooths the relay-side velocity estimate used for gating/logging |
| Evidence | No derivation found |
| **Classification** | **HEURISTIC** |

## Source-validity / stale-command handling

| Field | Value |
|---|---|
| `--stale-timeout-s` | 0.1s — `bilateral_4ch.py:674` |
| Purpose | Zeros a channel if its upstream topic hasn't updated within this window |
| Evidence | Standard watchdog practice, no project-specific derivation, but this is squarely a **safety** mechanism (stale data must not drive a live force), not a performance-tuning parameter |
| **Classification** | **SAFETY_CONSTRAINT** — explicitly out of scope for the coming optimization stages per the task's own Phase 11 |

## Friction parameters

Not found as an explicit, separately-identified term anywhere in the
current DOB configuration — `touch_dob.hpp`'s own header comments
(§H6 of the prior root-cause report) state uncompensated friction is
one of the sources of the DOB's residual floor. **Classification:
UNKNOWN / NOT_MODELED** — this is a gap, not a hidden assumption; there
is no friction parameter to classify because none exists in the current
model.

---

## Summary table (status column per Phase 14's required schema)

| Parameter | Classification | Status |
|---|---|---|
| C1 weight (0.5) | ARBITRARY | **REQUIRES_USER_DECISION** |
| C4 weight (0.3) | ARBITRARY | **REQUIRES_USER_DECISION** |
| Cf (1.0) | ARBITRARY | REOPTIMIZE (low priority, currently neutral) |
| C1 shoulder mask (hard, weight=0) | IDENTIFIED need / HEURISTIC form | **REQUIRES_USER_DECISION** (graded alternatives unexplored) |
| C2 (master) stiffness/damping | **SELECTED (sim Pareto decision, 2026-09-09): 15.0/0.10 → 35.0/0.20**, matching slave — `docs/SCIENTIFIC_POSITION_OPTIMIZATION.md`. Not yet applied to `run_master_left.sh`'s shipped defaults. `REQUIRES_FUTURE_HARDWARE_VALIDATION` | SELECTED, PENDING DEPLOYMENT DECISION |
| C3 (slave) stiffness/damping (35.0/0.20) | Unchanged by the Pareto decision (already the winning value) | KEEP_IDENTIFIED (as a baseline point that happened to also be selected) |
| DOB `g_dis`/`threshold_nm` | IDENTIFIED/EMPIRICALLY_SUPPORTED | KEEP_IDENTIFIED (re-verify under Phase 4, not re-derive from scratch) |
| Gravity comp (0.55/0.70N, Y) | EMPIRICALLY_SUPPORTED (partial) | KEEP_IDENTIFIED, flagged INSUFFICIENT_DATA for full convergence |
| Force/torque clamps (device ceilings) | SAFETY_CONSTRAINT | SAFETY_ONLY |
| `kMinRelayTorqueNormNm` (0.02) | ARBITRARY/UNKNOWN | INSUFFICIENT_DATA |
| Travel limits (0.12/0.19/0.15/0.08) | HEURISTIC | REOPTIMIZE |
| Relay deadband | Mixed | REOPTIMIZE (arbitrary part only) |
| Ramp time (0.5s) | HEURISTIC | REOPTIMIZE (low priority) |
| Energy tank (0.05J/2.0s) | SAFETY_CONSTRAINT (intent) / ARBITRARY (values) | SAFETY_ONLY for the mechanism; INSUFFICIENT_DATA for the specific numbers |
| Oscillation abort (0.4 N·m/1.5Hz) | SAFETY_CONSTRAINT (intent) / ARBITRARY (values) | SAFETY_ONLY / INSUFFICIENT_DATA |
| **C1 relay sign** | **SIMULATION-VALIDATED CORRECTION APPLIED (2026-09-08)** | **KEEP_IDENTIFIED (sim-level); REQUIRES_FUTURE_HARDWARE_VALIDATION (physical-device level)** |
| T_sm (implicit identity, joint-space) | UNKNOWN whether intended | **REQUIRES_USER_DECISION** |
| qdot-alpha filter (0.2) | HEURISTIC | REOPTIMIZE (low priority) |
| Stale timeout (0.1s) | SAFETY_CONSTRAINT | SAFETY_ONLY |
| Friction model | NOT_MODELED | INSUFFICIENT_DATA (gap, not a parameter) |

---

## ASSUMPTION_REQUIRES_USER_DECISION items surfaced by this phase

### 1. Bilateral force convention (blocks Phase 2, Phase 6, and the sign audit)

- **Parameter**: intended relationship between `F_m` and `F_s` at the C1 channel.
- **Possible values**:
  (a) **Reactive/opposing**: `F_m ≈ -F_s` (docstring's stated `-w1`) — operator feels a reaction force, as if pushing back against what's happening at the slave.
  (b) **Mirror/same-direction**: `F_m ≈ +F_s` (current code's actual behavior) — operator feels what the slave tool feels, same direction.
- **Physical meaning**: (a) is closer to a literal force-reflection/Newton's-third-law-style convention; (b) is closer to a "remote sensing" convention where the operator's hand mimics the tool's experience.
- **Evidence**: the module's own docstring states (a); the running code implements (b); this session's empirical anti-phase plot is consistent with (b) being what's actually live; no design document in the repo settles which is intended.
- **Consequences**: choosing wrong doesn't just mistune a gain — it flips the sign of every force-transparency metric computed downstream, and a hardware test built assuming the wrong convention would "pass" while training the operator to expect the opposite of what happens at the slave.
- **DECIDED (2026-09-08)**: user selected (a), `F_m ≈ -F_s`. The corresponding one-line code fix has been applied and simulation-validated (`cos_sim=-0.87`) — see `docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md`. Physical-hardware confirmation of this convention remains open (`REQUIRES_FUTURE_HARDWARE_VALIDATION`).
- **Confidence**: Medium (documentation + empirical pattern agree; no ground-truth *hardware* test exists to confirm — simulation ground truth now does confirm the code matches the decided target).

### 2. C1/C4 gain values (0.5/0.3)

- **Possible values**: any weighting in [0,1] per channel, or an asymmetric/objective-derived pair.
- **Physical meaning**: relative authority of force-relay vs. safety margin.
- **Evidence**: none found for the current specific numbers beyond a "safe" preset label.
- **Consequences**: too low → weak transparency; too high → risk of instability/oscillation (mitigated partially by the tank/trip-abort safety layer, which is itself unvalidated against a formal margin).
- **Recommended option**: none yet — Phase 6 (force subsystem optimization) is required before any recommendation, and per this task's rules that optimization cannot start until Phase 2 (sign correctness) is resolved.
- **Confidence**: N/A (data does not yet exist to recommend a value).

### 3. Shoulder mask: hard mask vs. graded alternative

- **Possible values**: (1) full contribution, (2) fixed reduced weight, (3) confidence-weighted, (4) frequency-filtered, (5) improved DOB/model, (6) current hard mask.
- **Evidence**: the *need* for some mitigation is well-evidenced (measured floor); *which* mitigation is best is not evaluated anywhere in the repo.
- **Consequences**: per this session's own H8 finding, the masked joint would dominate Y/Z Cartesian force reflection if fully unmasked — so the choice materially affects those two axes' transparency ceiling.
- **Recommended option**: none — Phase 7 requires an offline comparison of options 1-5 before treating option 6 as final.
- **Confidence**: N/A.

### 4. T_sm — is joint-space identity the intended frame transform?

- **Possible values**: (a) accept current implicit behavior (no explicit T_sm; each side projects through its own live Jacobian), (b) require an explicit, pose-normalized Cartesian transform so `F_m` and `F_s` are compared in a common, pose-independent frame.
- **Evidence**: no explicit `T_sm` exists in code; nothing in the repo states whether this was a deliberate simplification or an unexamined gap.
- **Consequences**: (a) means force-transparency error is contaminated by master/slave pose mismatch (both Jacobians differ whenever the two arms aren't in matching poses) — likely part of why this session's H5 cross-axis coupling and the general "poor felt transparency" reports don't cleanly separate a control-gain problem from a geometry problem. (b) requires new engineering (a defined common frame, likely the base/world Cartesian frame both already partially use) but removes this confound.
- **Recommended option**: (b) for any rigorous transparency metric, while (a) remains fine for the relay's actual real-time operation which cannot practically retarget frames on the fly without redesign.
- **Confidence**: Low-Medium — this is more a scope decision than a technical one.

---

## Goal 9 status update (2026-09-08, hardware-code-readiness pass)

Per `docs/SCIENTIFIC_HARDWARE_CODE_READINESS.md`'s required vocabulary
(distinct from Phase 14's original REOPTIMIZE/KEEP_IDENTIFIED/etc. set
above — both are kept, this table is the authoritative one for this
pass):

| Item | Status |
|---|---|
| **C1 sign** | **CORRECTNESS_FIXED** (simulation-validated: `cos_sim=-0.87`; `REQUIRES_FUTURE_HARDWARE_VALIDATION` for the physical-device convention) |
| C1 gain (0.5) | **BASELINE_ONLY** (relabeled in `PRESETS`'s comment; not changed) |
| C4 gain (0.3) | **BASELINE_ONLY** (relabeled; not changed; sign unchanged — no discrepancy found) |
| Shoulder mask (hard, default-on) | **SAFETY_ONLY / LEGACY_COMMISSIONING_PRESET** (relabeled in argparse help + code comment; default value NOT changed — a new numeric/graded alternative would be **REQUIRES_USER_DECISION**, not invented) |
| Joint weighting (per-joint, beyond the binary C1 mask) | **REQUIRES_USER_DECISION** — no per-joint weighting concept currently exists in code (only binary mask); inventing one was explicitly out of scope this pass |
| C2/C3 asymmetry (15.0/0.10 vs 35.0/0.20) | **TO_BE_OPTIMIZED** (relabeled in `run_master_left.sh`/`run_slave_right.sh`; already configurable via env vars; values unchanged) |
| Deadbands (`effective_threshold_nm`, `kMinRelayTorqueNormNm`) | **BASELINE_ONLY** for the arbitrary `kMinRelayTorqueNormNm=0.02` component; **TO_BE_OPTIMIZED** would require a defined objective first (Phase 6, blocked) |
| Thresholds (DOB `threshold_nm`, `g_dis`) | **Unchanged classification: IDENTIFIED/EMPIRICALLY_SUPPORTED** (well-evidenced, see main table above) — not touched this pass |
| Gravity offset (0.55/0.70N, Y) | **Unchanged classification: EMPIRICALLY_SUPPORTED (partial)** — not touched this pass, per the task's explicit instruction not to alter gravity parameters |
| Clamps (device force ceilings, workspace limits, energy tank, oscillation trip) | **SAFETY_ONLY** (labeled as such in code; values unchanged; none weakened) |
| Filters (`qdot-alpha`) | **TO_BE_OPTIMIZED** (unchanged value; not relabeled in code this pass — low priority, noted here for completeness) |

## What this phase does NOT do

Per the task's own staging, this document stops at inventory and
classification. It does not select values, does not run simulations,
and does not resolve the sign question — those are Phases 1-15,
sequenced in `docs/SCIENTIFIC_4CH_DESIGN_PROCESS.md`.
