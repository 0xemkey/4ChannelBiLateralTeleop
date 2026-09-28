# Scientific 4-channel design process — status and roadmap

Date: 2026-09-08. Mode: OFFLINE ANALYSIS / SIMULATION / SOURCE INSPECTION
ONLY throughout. **No physical hardware was started, opened, or
commanded at any point in this process** — confirmed per-phase below.

Decided project target (user, 2026-09-08): force transparency defined as
`F_m ≈ -F_s` (reactive/opposing convention), matching `bilateral_4ch.py`'s
own module docstring — adopted as the scientific design target for all
downstream phases; the current code's actual behavior is measured
against this target, not used to redefine it.

## Phase status

| Phase | Deliverable | Status | Notes |
|---|---|---|---|
| 0 — Assumption inventory | `SCIENTIFIC_CONTROLLER_ASSUMPTION_REGISTER.md` | **DONE** | Every controller parameter classified DERIVED/IDENTIFIED/EMPIRICALLY_SUPPORTED/SAFETY_CONSTRAINT/HEURISTIC/ARBITRARY/UNKNOWN; 4 `ASSUMPTION_REQUIRES_USER_DECISION` items surfaced, 1 resolved this session (force convention) |
| 1 — Signal chain equations | `SCIENTIFIC_SIGN_FRAME_AUDIT.md` §Phase 1 | **DONE** | Reused/confirmed from the prior root-cause report's source trace |
| 2 — Sign/frame correctness | `SCIENTIFIC_SIGN_FRAME_AUDIT.md` §Phase 2 | **DONE + CORRECTED for C1** | Sim test confirmed defect (`cos_sim=+0.87`, same-direction); one-line fix applied on branch `c1-sign-fix-sim-validation`; re-validated (`cos_sim=-0.87`, anti-parallel); 31/31 hardware-independent regression tests pass (1 pre-existing assertion updated to match the new, decided sign convention). `REQUIRES_FUTURE_HARDWARE_VALIDATION` remains for the physical-device force-sign convention. C4/H/other sub-items: no discrepancy found or not investigated (see audit doc's table) |
| 3 — Physical truth variable definitions | `SCIENTIFIC_SIGNAL_DICTIONARY.md` | **DONE** | Every signal classified MEASURED/ESTIMATED/COMMANDED/DERIVED/HYPOTHETICAL with transparency-metric suitability |
| 4A — DOB repeatability audit | `SCIENTIFIC_DOB_VALIDATION.md` §Phase 4A | **DONE — `PHASE_4A_PASS` (qualified)** | N=20 identical-condition free-space sim repeats; confirmed NO RNG exists anywhere in the sim (grep-verified) — the earlier 48× discrepancy was traced to real-time scheduling jitter interacting with an incomplete 1.0s settling window (well-evidenced: the one outlier run has `qdot_std=0.0`, i.e. fully stopped near a joint limit, vs. `~0.5 rad/s` for all 19 other runs). 19/20 runs agree to 5 decimals on waist/shoulder; elbow has genuine ~16% CV. Gate: within-condition variability << historical between-setting effect size for waist/shoulder, marginal for elbow. One `ASSUMPTION_REQUIRES_USER_DECISION` raised: settle-window fix (lengthen fixed window vs. convergence-based gate) before Phase 4B begins |
| 4B — DOB characterization (passive, free-space) | `SCIENTIFIC_DOB_VALIDATION.md` §Phase 4B | **DONE, INCONCLUSIVE (explained)** | Passive settle-to-rest converges to a DC/gravity-residual floor that is mathematically g_dis-invariant (low-pass DC gain=1) — cannot discriminate candidates on waist/shoulder; small hardware-consistent trend on elbow (never fully settles). Motivated the redesign below. |
| 4B redesign — DOB characterization (active wall contact) | `SCIENTIFIC_DOB_VALIDATION.md` §Phase 4B redesign | **DONE — `PHASE_4B_PASS`** | Reused Phase 1/2's wall-contact recipe; N=10×3 g_dis candidates; steady-state contact gain shows a real, resolved effect (2π·5: 0.887 vs 10/5 rad/s: ~0.867-0.869, gap 2× the largest within-condition std) — faster cutoff tracks contact more faithfully, matching control-theory expectation and the existing documented free-space-noise/contact-fidelity tradeoff. Onset/release delay not measured (clock-alignment limitation, `INSUFFICIENT_DATA`). Does not recommend changing g_dis — that's a Phase 6 question. |
| 6 — Force subsystem (C1/C4) optimization | `SCIENTIFIC_FORCE_OPTIMIZATION.md` | **DONE, restricted scope** | Elbow classified `ELBOW_C1_SIGN=UNRESOLVED` (testability limitation, open issue documented with 4 attempted resolutions, none succeeded). C1 gain characterized on SHOULDER-only (validated reference channel, waist+elbow masked out of the relay by design): N=5×4 gains, 40/40 runs clean, sign opposing 20/20 contact runs, gain scales linearly (~0.53/unit w1), zero free-motion false force at every gain up to 1.0. WAIST: not excited by any test this session (Y-directed contact scenario structurally can't load it), honestly reported UNTESTED not validated. C4: confirmed NOT blocked by the elbow issue (separate ground-truth mechanism, master-side hand model) — one confirmatory run shows same-direction sign matching its own already-correct convention; full C4 sweep scoped as next step, not completed. No scalar objective invented; no C1 gain selected. **Continuation (2026-09-09): Next Step A (C1 active-motion noise, N=5×4 gains) found NO tradeoff — exactly zero transmitted noise at every gain up to 1.0, even under aggressive motion (0.08m/3.0Hz), reported honestly per instruction. Next Step B (full C4 sweep, 3 conditions × N=5 × 4 gains = 60 runs) found the same pattern: linear gain scaling, confirmed correct sign (cos_sim≈0.76 at every gain), zero noise. Conclusion: gain CANNOT yet be scientifically selected — absence of a measured tradeoff isn't sufficient breadth (one motion profile, one pose, no gains >1.0, shoulder-only for C1, zero hardware validation).** |
| 5 — Position subsystem (C2/C3) optimization | `SCIENTIFIC_POSITION_OPTIMIZATION.md` | **DONE — X/Y/Z Pareto comparison, no winner selected** | N=5×3 configs × 3 driven axes (45 sim runs total) via `bridge_master_slave.py`. X: genuine tradeoff (baseline wins lag/responsiveness, stiff wins fidelity). **Z: stiff wins nearly everything, including lag — the X tradeoff does not generalize.** Y: data collected but flagged low-confidence (0.4N push produces only tens-of-μm net Y motion, master rise ~2.75-3.08s close to the full push window — not a clean step response); Y's one robust finding is a dramatic 10.8-12.0× Y→X cross-coupling, an order of magnitude larger than any other axis pair, consistent with this project's known Y-axis/base-tilt history but not yet explained. `symmetric_soft` confirmed `DOMINATED` on X and Z. Settling-time epsilon question resolved by explicit instruction: report `INSUFFICIENT_REQUIREMENT`, never invent a threshold. One genuine Pareto tradeoff remains (X-axis) — asked user to choose, per instruction, rather than resolving it here. **Y-anomaly follow-up (2026-09-09): resolved, NOT a magnitude artifact** — 6-magnitude sweep (0.1-2.0N) shows roughly linear scaling below a previously-undocumented 0.45N master force clamp, and the 10-13× X-coupling ratio holds across the entire range (if anything larger at low force). Leading explanation: Y is this arm's gravity-loaded axis (consistent with `SLAVE_GRAVITY_AXIS=1` and prior Y-axis docs), not derived from first principles. Pareto preference question re-posed to user. **Decision made (2026-09-09): `symmetric_stiff` selected** (master gains 15.0/0.10 → 35.0/0.20, matching slave; slave unchanged). Recorded as `SELECTED, PENDING DEPLOYMENT DECISION` in the assumption register — NOT applied to `run_master_left.sh`'s shipped defaults this session (a separate hardware-consequential action not requested) |
| 6 — Force subsystem (C1/C4) optimization | `SCIENTIFIC_FORCE_OPTIMIZATION.md` | **BLOCKED, correctly** | Per the task's own ordering rule ("only after estimator correctness and position subsystem characterization"), and because Phase 2 found the C1 sign itself unresolved for physical hardware — optimizing gains on top of an unconfirmed sign is explicitly disallowed by the task |
| 7 — Shoulder-mask alternatives | (folds into Phase 6 doc when written) | **NOT STARTED** | Register already lists the 6 candidate policies; comparing them needs the same simulation infrastructure as Phase 4/6 |
| 8 — Multi-objective framework | (folds into Pareto doc) | **NOT STARTED** | |
| 9 — Staged optimization (A-F) | — | **Stage A (sign/frame) DONE for C1**; B-F not started | |
| 10 — Robustness / Monte Carlo | — | **NOT STARTED** | |
| 11 — Safety-constraint separation | Reflected in the assumption register's classification column | **DONE** (as classification; not re-verified operationally) | |
| 12 — Graphing rules | Applied retroactively to `SCIENTIFIC_SIGNAL_DICTIONARY.md`'s suitability table | **DONE as a rule; not yet re-applied to regenerate old plots** | |
| 13 — Output documents | This file + the 3 above | **4 of 8 written this session** (register, sign/frame audit, signal dictionary, this roadmap); DOB validation, position optimization, force optimization, Pareto analysis remain |
| 14 — Decision table | Embedded in the assumption register's summary table | **DONE** | |
| 15 — Claim-standard discipline | Applied throughout — every finding above is stated with its evidence, not asserted | **Ongoing discipline, not a one-time deliverable** | |

## Why Phases 4-10 stop here, explicitly

This is not an evasion — it's the honest boundary of what a single
offline session can respons­ibly close out. Specifically:

- **Phase 4** needs a genuine per-joint sweep (free-space bias, RMS/p99
  noise, contact SNR, bandwidth, cross-joint coupling) run across
  multiple simulated conditions — the *mechanism* now exists (proven by
  this session's sign test, which reused and extended the same
  sim-plus-truth-CSV pattern), but the sweep itself is new data
  collection, not something derivable from documents already on disk.
- **Phase 5/6** each require a defined, non-arbitrary objective function
  before any search can start — and the task's own rules (Phase 5's
  lambda-weighting instruction, Phase 8's "do not collapse to one scalar
  until normalization/weights are justified") make picking those weights
  an `ASSUMPTION_REQUIRES_USER_DECISION` item in its own right, not
  something to default through.
- **Phase 10** (Monte Carlo robustness) is only meaningful once Phase 5/6
  have produced candidate optima to test the robustness of — there's
  nothing to stress-test yet.

## What changed this session, concretely

1. `bilateral_4ch.py`'s C1 sign moved from a documentation mismatch
   flagged by reading code, to **simulation-confirmed defect**
   (`cos_sim=+0.87`), to **applied, simulation-validated correction**
   (`cos_sim=-0.87`), on branch `c1-sign-fix-sim-validation`, backed by
   a reusable regression test
   (`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`) and a
   full 31/31-passing hardware-independent regression run.
2. The fix is a single line (`scripts/bilateral_4ch.py:441`, add a
   negation). No hardware-facing C++ was touched. No physical hardware
   was accessed at any point. The fix is **not merged to the working
   branch used for real teleoperation** and **not deployed to any real
   device** — it exists only on the isolated dev branch pending the
   physical-hardware validation this document repeatedly flags as
   outstanding.

## Recommended next steps (not started, require explicit user go-ahead per-phase)

1. Authorize a Phase 4 DOB-characterization sweep (simulation-only, safe
   to run without further hardware discussion) — this is the natural
   next block of work and has no open `ASSUMPTION_REQUIRES_USER_DECISION`
   blocking it. **User has now authorized proceeding to Phase 4** (this
   message) — see the new §Phase 4 status below once that work starts.
2. Decide the C1 sign fix's disposition: merge the dev branch into the
   working branch (still without touching hardware — the fix stays
   simulation-only either way until a physical test exists), keep it
   isolated pending a physical ground-truth test, or something else.
   **Not yet asked explicitly** — flagging here.
3. Once Phase 4 exists, define `J_pos`'s weighting (Phase 5) — will
   require another `ASSUMPTION_REQUIRES_USER_DECISION` round.

## Hardware-safety confirmation

Every command run in this process this session: `grep`, `find`, `Read`,
`python3 -m pytest` against `touch_slave_sim`/`hd_sim` (confirmed via
`ldd` output showing `libhd_sim.so`, distinct from the real-hardware
`touch_slave` binary which links `libHD.so`). No `/dev/ttyACM*` or
`/dev/ttyUSB*` was opened (verifiable: `hd_sim_api.cpp` contains no such
path). No `01_start_master.sh`/`02_start_slave.sh`/`03_start_position_bridge.sh`
or any real-hardware launcher was run. `hardware_interaction = 0`,
`hardware_commands = 0`, `real_device_access = 0`,
`physical_experiments = 0` — maintained throughout.
