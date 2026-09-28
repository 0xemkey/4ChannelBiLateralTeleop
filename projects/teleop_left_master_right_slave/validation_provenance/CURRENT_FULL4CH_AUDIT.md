# Full-4ch Integrity Audit — 2026-09-10

> **2026-09-10, pre-tuning closure update**: everything below this note is
> the ORIGINAL audit, preserved exactly as written. A follow-up closure
> pass resolved the open items this audit correctly flagged as `UNKNOWN`
> or as gaps (Part 4's mask state, Part 6's qualification gate, Part 8's
> channel semantics, Part 9's force-budget semantics, Part 10's clamp
> cause). See **"PRE-TUNING CLOSURE RESOLUTION (2026-09-10, follow-up)"**
> at the very end of this document for the resolutions, each citing new
> concrete evidence. Nothing below was edited to look cleaner in
> hindsight — this is the historical record.

Scope note (honesty over completeness): this audit is evidence-based and
traces every claim to source code, raw session data, or a specific
command run during this audit. It is NOT an exhaustive line-by-line
review of every file in the repository — given the size of the codebase,
checks were prioritized by risk (real-hardware runtime path first, test
tooling last). Where a check was not run to completion, it is marked
`UNKNOWN` rather than assumed clean, per the project's own rule.

## PART 1 — Provenance Snapshot

- Branch: `c1-sign-fix-sim-validation`
- HEAD: `4363b5c3a716d5deae04c59d931b88f8ae939ebe` (2026-09-07 18:25:23 +0300)
- Working tree: dirty. `git diff --stat` (tracked files only):
  23 files changed, 2125 insertions(+), 257 deletions(-) — spans
  `bridge_master_slave.py`, `bilateral_4ch.py`, `startup_alignment.py`,
  `touch_slave.cpp`, `BridgeDiagnostics.msg`, `BilateralStatus.msg`,
  `run_master_left.sh`, `run_slave_right.sh`, `run_bridge.sh`,
  `04_start_force_relay.sh`, `03b_start_bridge_c2_diagnostic.sh`, and others.
- Untracked (relevant): `hardware_validation/` (qualification artifacts,
  analyzers, `arm_production_bridge.py`, `pre_handoff_check.py`),
  `hardware_experiments/`, `scripts/coupling_force_budget.py`,
  `experiment_launch/08_qualify_c1_only.sh`, `09_qualify_c4_only.sh`,
  `10_start_full_4ch_production.sh`, `run_hardware_test.sh`,
  `03c_start_bridge_c2_only_diagnostic.sh`, `validation_provenance/`
  (pre-existing, contains `LAST_KNOWN_GOOD.md` dated 2026-09-10).
- No destructive git action was taken (no reset/checkout/clean) during
  this audit or the session leading up to it, as far as this audit can
  observe from `git status`/`git log`.

### Baseline session identity

- Session: `20260910T155604Z_full4ch_current_setup_baseline`
- Trial: `.../manual_motion/current_setup_baseline`
- `manifest.env` (session-level): `ROS_DOMAIN_ID=0`, master device `Left`,
  slave device `Default Device`, DOB files pointing at
  `~/dob_logs/results/{inertia,gravity}_{left,right}*.json`,
  `DOB_G_DIS_RAD_S=10,10,5`, `POSITION_GAINS=master=15/0.10,slave=35/0.20`,
  `LIMITS=master=2.0N,slave=1.5N,position=0.03/0.04m,relay_floor=0.02Nm`,
  `RUN_HARDWARE_TEST_MODE=full4ch`,
  `FORCE_RELAY_WEIGHTS_C1_C2_C3_C4=0.5,0.0,0.0,0.3`.
- Launch chain used: `run_hardware_test.sh --mode full4ch` →
  `10_start_full_4ch_production.sh` → `04_start_force_relay.sh --full4ch`
  (`bilateral_4ch.py`, weights C1=0.5,C2=0,C3=0,C4=0.3) +
  `run_bridge.sh` (`ENABLE_C2=true C3_ENABLE=true`, master
  K=15.0/D=0.10/Fmax=2.0/max_error=0.03, `LEFT_GRAVITY_COMP_N=0.55`).
- **Important divergence from earlier in this working session**: the C2
  gains here are the plain **production** defaults (K=15, Fmax=2.0), NOT
  the "Candidate A" low-authority diagnostic values (K=5, Fmax=0.85–0.90)
  this session had been separately investigating for C2 breakaway. Per
  `LAST_KNOWN_GOOD.md` (in this directory, dated 2026-09-10, written by a
  process outside this audit's own visible actions), this was a deliberate
  reconstruction back to the last known logged 4-channel hardware
  configuration (`four_channel_val_20260908` and siblings), on the
  assessment that the diagnostic K=5 launchers were a regression for
  judging production bilateral behavior. This audit did not re-derive
  that decision from scratch; it is recorded here as provenance, not
  re-litigated.
- Analyzer used to produce the reported baseline verdict:
  `hardware_validation/analyze_full4ch_baseline.py` (untracked, present on
  disk), output: `analysis/FULL4CH_TEST_SUMMARY.md`, `analysis/metrics.json`.
  This is a **different, more thorough** tool than
  `hardware_validation/analyze_manual_motion.py` — see Part 7.
- Qualification artifacts consumed by `10_start_full_4ch_production.sh`'s
  gate: `hardware_validation/qualification/C1_QUALIFIED.json` (session
  `20260910T153620Z_c1only_c1_qualification`) and `C4_QUALIFIED.json`
  (session `20260910T153834Z_c4only_c4_qualification`) — both written
  ~2 hours before the full4ch baseline, by `write_qualification_artifact.py`.

## PART 2 — Mock/Synthetic/Demo Path Audit

Targeted grep for `mock|synthetic|fake|demo|replay|playback|rosbag|canned|
fixture|stub|dummy|fallback|placeholder|hard-?coded|substitute` across the
real-hardware-relevant runtime files (`scripts/bridge_master_slave.py`,
`scripts/bilateral_4ch.py`, `scripts/run_*.sh`, `experiment_launch/*.sh`,
`ros2_ws/src/touch_haptics/src/touch_slave.cpp`).

Every match inspected individually was benign:
- `touch_slave.cpp`'s `replay` hits all reference `tools/dob_replay_cpp.cpp`,
  a separate **offline** analysis tool over already-recorded CSVs, not
  anything reachable from the live device-write path.
- `bridge_master_slave.py`'s `fallback` hits are a CLI-arg default-value
  helper (`parse_triple`) and comments, not data injection.
- `bilateral_4ch.py`'s `fallback`/`fake`/`replay` hits are prose (the
  module docstring literally says "not something this script can fake by
  weighting alone" — a warning against a wrong assumption, not a
  mechanism).
- `01b_start_master_c2_diagnostic.sh` / `03_start_position_bridge.sh`'s
  `hardcoded` hits are comments describing a literal numeric default, not
  a synthetic-data path.

No `sim`/`touch_slave_sim` import, subscription, or launch path was found
reachable from any real-hardware launcher (`00_preflight.sh` through
`10_start_full_4ch_production.sh`, `run_hardware_test.sh`). The sim
executable lives at a physically separate build output
(`install/touch_haptics/lib/touch_haptics/touch_slave_sim`) and is only
invoked by `ros2_ws/src/touch_haptics/sim/tests/*.py`.

**Direct evidence the baseline session's master/slave were real hardware,
not simulated** (log inspection, not just topic names):
```
master.log: "touch_slave started with max_force=2.000 N stiffness=15.000 N/m ..."
slave.log:  "touch_slave started with max_force=1.500 N stiffness=35.000 N/m ..."
```
Neither log contains the sim binary's own self-identifying startup line
(`[hd_sim] initialised simulated device ... NOT HARDWARE.`), which this
session independently confirmed appears verbatim in every sim run earlier
in this conversation (e.g. `test_production_arm_sequence_live.py`'s own
sim logs). Absence of that marker plus presence of the real executable's
own startup line is treated as strong (not 100%-exhaustive) evidence.

Evidence table (spot-checked, not a full topic enumeration):

| Topic | Publisher (from log/manifest) | Classification | Confidence |
|---|---|---|---|
| `/touch_left/pose`, `/touch_left/cartesian_controller_telemetry` | `teleop_master_left` (`touch_slave`, real device `Left`) | Hardware | High — log's own startup line, no sim marker |
| `/touch_right/pose`, `/touch_right/cartesian_controller_telemetry` | `teleop_slave_right` (`touch_slave`, real device `Default Device`) | Hardware | High — same |
| `/bridge_diagnostics` | `touch_master_slave_bridge` (`bridge_master_slave.py`, argv logged) | Real production bridge process | High |
| `/bilateral_status` | `bilateral_4ch` node (`force_relay.log` startup line: `bilateral_4ch started: architecture='custom' target_weights=[0.5,0.0,0.0,0.3]`) | Real relay process | High |
| `joint_torque_ext_cmd` (both arms) | Same `bilateral_4ch` process | Real relay path | High |
| `relay_force_n`, `applied_force_n` | `touch_slave.cpp` runtime fields, recorded via `CartesianControllerTelemetry` | Hardware device-write telemetry | High (field-level; not independently re-derived from the .cpp for this audit — see Part 12) |

**Conclusion for Part 2: no mock/synthetic/replay path found active in
the baseline.** This is based on a targeted, risk-prioritized sweep, not
an exhaustive audit of the ~30 files a broader grep matched (most were
test/tooling files not executed during a real-hardware session).

## PART 3 — Real Hardware Device Path Audit

**Not independently re-traced from the OpenHaptics SDK source in this
audit** (out of budget). What this audit DOES confirm: `touch_slave.cpp`
opens the device via `device` parameter (`"Left"` / `"Default Device"`),
and the startup log line format (`touch_slave started with max_force=...`)
is unique to the real binary, distinct from `touch_slave_sim`'s own
distinct startup banner. `/dev/ttyACM0`/`/dev/ttyACM1` are checked by
`00_preflight.sh` for device-file presence/permissions but that is a
`00_preflight.sh`-level sanity check, not proof of the actual OpenHaptics
HD API device-open call — this audit does **not** claim otherwise.
**Classification: UNKNOWN (partially evidenced) for the deep SDK chain;
HIGH confidence for "this was not the sim binary."**

## PART 4 — Channel-by-Channel Physical Causal Audit

Traced from `analysis/FULL4CH_TEST_SUMMARY.md` (produced by
`analyze_full4ch_baseline.py`, which DOES compute per-joint sign
correlation and source→relay correlation — see Part 7 for why this
matters):

**C2** (slave position → master): `correlated_gain=1.078`,
`rmse=0.0255m`, `peak_error=0.0938m`. Source is `/touch_right/pose`
(real device), target `/touch_left/target_pose` via
`bridge_master_slave.py`'s `compute_c2_target`, force applied through the
master's own spring/damper to `applied_force_n`. No zero-gain or masked
axis found in this metric set.

**C3** (master position → slave): `correlated_gain=0.925`,
`rmse=0.0243m`, `peak_error=0.0938m`, `positive_lag=0.130s`. Same chain,
opposite direction, via `compute_c3_target`.

**C1** (slave DOB → master force): sign correlation **-0.965 (waist),
-0.998 (shoulder), -0.991 (elbow)** — all "correct", consistent with the
documented formula `tau_c1 = -w1 * cf * tau_e_hat * mask` (negative sign
expected). `source_norm_to_relay_force_norm_correlation=0.944`.
`relay_valid_duty=1.0`, `relay_fresh_duty=1.0`, `relay_applied_duty=0.239`
(the 0.02 N·m deadband floor was cleared 24% of the samples — expected
for intermittent excitation, not itself a defect).
`c1-disable-joints` was **not** checked in this audit's grep of the
baseline's actual `force_relay.log` startup line — the line shown
(`target_weights(C1..C4)=[0.5, 0.0, 0.0, 0.3]`) does not print the joint
mask; this should be confirmed from the full argv before declaring "no
mask" with full confidence. **Flagging as a gap, not resolving it here.**

**C4** (master DOB → slave force): sign correlation **~+1.0 on all three
joints** — "correct", consistent with `tau_c4 = +w4 * cf * tau_h_hat`.
`source_norm_to_relay_force_norm_correlation=0.737` — real but weaker
than C1's 0.944. `relay_applied_duty=0.0316` — an order of magnitude
lower than C1's. This is a genuine, evidenced asymmetry (not obviously a
bug — could be excitation-amplitude-dependent, master DOB floor, or
weight asymmetry 0.5 vs 0.3) flagged for the fine-tuning plan (Part 15,
item C), not resolved here.

No `c1-disable-joints`/`disabled_joints`/mask zeroing that would silently
drop a joint from C1/C4 was found in `bilateral_4ch.py`'s core mixing
logic beyond the documented, disclosed `--c1-disable-joints` (shoulder,
by default, per the module's own long-standing comment about the
right-arm DOB floor) — **but this audit did not re-verify the actual
runtime value used for THIS baseline session's argv line-by-line beyond
what's shown above.** Classification: `UNKNOWN` on that one specific point.

## PART 5 — Gain/Weight/Mask Audit

From `manifest.env` and `force_relay.log`:

| Parameter | Value | Source |
|---|---|---|
| C1 weight | 0.5 | `FORCE_RELAY_WEIGHTS_C1_C2_C3_C4=0.5,0.0,0.0,0.3`, `04_start_force_relay.sh`'s `full4ch` case default |
| C2 weight (bilateral_4ch's own) | 0.0 | same — position channels intentionally zeroed in bilateral_4ch since the bridge owns C2/C3 |
| C3 weight (bilateral_4ch's own) | 0.0 | same |
| C4 weight | 0.3 | same |
| Cf | 1.0 | `CF="${CF:-1.0}"` in `04_start_force_relay.sh`, not overridden in manifest |
| Master K/D/Fmax/max_error | 15.0/0.10/2.0/0.03 | `manifest.env` `MASTER_*`, matches `master.log`'s own startup line |
| Slave K/D/Fmax/max_error | 35.0/0.20/1.5/0.04 | `manifest.env` `LIMITS`/`POSITION_GAINS`, matches `slave.log` |
| `LEFT_GRAVITY_COMP_N` | 0.55 | `manifest.env` |
| `SLAVE_GRAVITY_COMP_N` | 0.55 (default, not overridden) | `run_bridge.sh` default |
| DOB `g_dis` | `[10, 10, 5]` rad/s, `dob_g_dis_scale` not set (=1.0 default) | `manifest.env` `DOB_G_DIS_RAD_S` |

C1 != 0 (0.5), C4 != 0 (0.3), C2/C3 active via the bridge (confirmed
`c2_enabled=true`/`c3_enabled=true` in `validation.json`'s `configured`
block, cross-checked against `bridge_diagnostics`). These are explicitly
**commissioning gains** — `bilateral_4ch.py`'s own docstring already
labels the `4ch_safe` preset's 0.5/0.3 values as "never derived from a
defined transparency objective" — carried into the fine-tuning plan
(Part 15) rather than treated as validated optima here.

## PART 6 — Qualification Artifact Forensics

`write_qualification_artifact.py` gates on: `validation.overall_result ==
"VALID"`, `manifest.expected_channel_configuration[channel] is True`,
`validation.channel_isolation.live_channel_active_available` is true, and
`missing_expected_activation` is empty/falsy. No `--force` or bypass flag
exists in this writer.

**Traced the actual C1/C4 qualification trials' raw validation**
(`hardware_validation/qualification/C1_QUALIFIED.json` /
`C4_QUALIFIED.json`, both produced via `analyze_manual_motion.py` against
sessions `20260910T153620Z_c1only_c1_qualification` /
`20260910T153834Z_c4only_c4_qualification`):

```
C1: source_peak_y_m=0.0820 (slave), follower_peak_y_m=-0.000165 (master),
    positive_motion_gain=-0.00201,
    force_dob.master_peak_applied_force_n_y=0.7321
C4: source_peak_y_m=0.0481 (master), follower_peak_y_m=0.000482 (slave),
    positive_motion_gain=0.01002,
    force_dob.slave_peak_applied_force_n_y=0.5500
```

**Finding (real, requires review — not a bypass, but a genuine gap):**
`analyze_manual_motion.py`'s `validate_trial()` — the function that
actually decides `VALID`/`INVALID_TRIAL` — **never reads `force_dob` at
all.** Confirmed by reading the function source: its flags are
`BRIDGE_FAULT`, `FINAL_FORCE_CLAMP` (a FLAG, not gating), `TELEMETRY_STALE`,
`UNEXPECTED_CHANNEL_ACTIVATION`, `MISSING_BILATERAL_STATUS`,
`EXPECTED_CHANNEL_NOT_ACTIVE`, `CONFIGURED_CHANNEL_MISMATCH`,
`WRONG_DIRECTION_MOTION` (about the SOURCE's own position excursion
direction, not the force channel's sign), `FAILURE_TO_RETURN`. **None of
these reference the relayed force's magnitude, sign, or correlation with
the source disturbance.** A trial where the relay applied exactly zero
force (e.g., a wiring regression, a sign flip that happened to cancel, or
a stale-relay bug) would still be written `VALID` by this path as long as
the SOURCE arm moved enough and channel config/isolation matched.

In this specific instance, real non-trivial force was present
(0.73N / 0.55N peaks) — so the two artifacts on disk are not vacuous. But
the **gate itself provides no structural guarantee** against a future
vacuous pass. This is the single most important actionable defect this
audit found in the qualification pipeline. **Recommendation (not applied
in this audit): before trusting future C1/C4 qualification runs, either
(a) route `write_qualification_artifact.py` through
`analyze_full4ch_baseline.py`'s stronger sign/correlation logic instead
of `analyze_manual_motion.py`, or (b) add force-magnitude/correlation
gating to `analyze_manual_motion.py`'s `validate_trial()` directly.**

No evidence a manually-authored JSON could pass as an artifact — the
writer script requires `manifest.json`+`validation.json`+`metrics.json`
already produced by a real trial directory; nothing in
`10_start_full_4ch_production.sh`'s check does more than `[[ -f ... ]]`,
so a **hand-crafted JSON with the right shape at the right path would
satisfy the launcher's own check** — this is a file-existence gate, not a
cryptographic/schema-enforcing one. Flagging as a low-probability but
real gap (requires deliberately fabricating a file, which the project's
own non-negotiable rules already forbid — this audit did not do so and
did not find evidence anyone did).

## PART 7 — Validator/Analyzer Audit

Two analyzers exist with materially different rigor:

- `hardware_validation/analyze_manual_motion.py` — used to gate
  individual C1-only/C4-only **qualification artifacts**. Confirmed:
  direction-role mapping (`c1`→slave-sources/master-follows,
  `c4`→master-sources/slave-follows) is correct and channel-aware (Part 8
  note aside). Does **not** compute force sign/correlation (Part 6).
  `VALID` requires `not insufficient_excitation` and no INVALID-severity
  flag — an all-zero/constant dataset would fail via
  `insufficient_excitation` (not independently re-verified with an
  adversarial run in this audit — recommend adding the adversarial unit
  tests the task explicitly lists, none of which currently exist in
  `hardware_validation/tests/` as far as this audit found).
- `hardware_validation/analyze_full4ch_baseline.py` — used only for the
  **post-hoc full4ch baseline report**, not any gate. This one DOES
  compute per-joint sign correlation and source↔relay-force correlation
  (Part 4's numbers came from here). **This is the stronger tool and it
  is not currently wired into the qualification gate at all** — the gate
  uses the weaker one. This mismatch is itself worth fixing (use the same
  rigor for gating that is trusted for reporting).

Adversarial unit tests (all-zero dataset, constant pose, missing
slave/master, missing bilateral_status, wrong C1 sign, relay_valid=true
but relay_applied=false, source-moves-follower-doesn't, unrelated maxima
at different times, synthetic-perfect-1:1 test, final-force-clamp
throughout): **none of these were found in
`hardware_validation/tests/` as pre-existing tests for either analyzer.**
This audit did not add them (explicitly a "prepare tomorrow" item per the
task, not "do blind tuning today" — writing adversarial tests IS
in-scope future validator hardening work, tracked in Part 15/deferred,
not fabricated under this audit's time budget).

## PART 8 — Channel State Telemetry Bug

**Confirmed, traced from source.** The baseline trial's own
`validation.json`:
```
"configured": {"c1": true, "c2": true, "c3": true, "c4": true},
"expected":   {"c1": true, "c2": true, "c3": true, "c4": true},
"configured_matches_expected": true,
"missing_expected_activation": [{"channel": "c2"}, {"channel": "c3"}]
```
`configured` is read from `bridge_diagnostics`' own
`c1/c2/c3/c4_enabled` fields — correct, and it correctly shows all four
true. `missing_expected_activation` is computed **only** from
`bilateral_status`'s per-channel `channel_active_N` flags (confirmed in
`channel_isolation_check()`'s source). In this architecture,
`bilateral_4ch.py` is launched with `w2=w3=0` **by design**
(`04_start_force_relay.sh`'s `full4ch` case: "Position channels are owned
by bridge_master_slave.py... Keep bilateral_4ch.py's joint-space setpoint
blend at zero") — so `bilateral_status` will **always** report C2/C3 as
inactive in this architecture, regardless of whether the bridge's real C2/C3
position coupling is working. **This is a validator semantic bug, not a
channel-masking defect**: C2/C3 genuinely are active (bridge-owned,
confirmed), the tool is just asking the wrong process. Fix (not applied
here, to avoid scope creep into "blind tuning" territory, but concretely
specified): `channel_isolation_check()` should source C2/C3 "active"
truth from `configured` (bridge diagnostics) and reserve
`bilateral_status`'s `channel_active_N` for C1/C4 only, matching the
task's own stated hypothesis, now verified from source.

## PART 9 — Force Budget Accounting Audit

**Confirmed real accounting gap, traced to source.** `grep -n "relay"
scripts/coupling_force_budget.py` returns **zero matches**. The force
prediction used for `master/slave_accepted_force_budget_margin_n` is
`predict_force_vector()`: `spring + damping + auxiliary` only —
`margin = (max_force_n - force_reserve_n) - |predict_force_vector(...)|`.
**It structurally cannot see the C1/C4 relay force contribution**, because
that force is added later, inside `touch_slave.cpp`'s own controller
(the "C1/C4-onto-Cartesian relay" block), entirely downstream of and
invisible to `bridge_master_slave.py`'s planner.

This is exactly why the baseline shows `slave_accepted_force_budget_margin_n
≈ +0.85N` (a healthy-looking position-only prediction) at the same
session where `force_before_final_clamp_norm_n` reached **1.719N** against
a **1.5N** hard `max_applied_force_norm_n` ceiling — the margin metric and
the real device-write ceiling are answering two different questions. The
**margin is not wrong on its own terms** (it correctly reports the
position-channel-only prediction has headroom); it is **misleadingly
labeled/consumed** as if it were the total force budget once C1/C4 are
active.

**Safety-relevant conclusion**: the actual hardware safety backstop —
`touch_slave.cpp`'s final force clamp — did its job correctly: applied
force never exceeded 1.5N even when the pre-clamp total reached 1.719N.
No device was over-driven. The defect is in the **predictive/reporting**
metric, not the real-time safety enforcement. Recommend (deferred, not
applied): either extend `predict_force_vector()`/the planner to include an
estimated relay-force term when C1/C4 are configured, or explicitly
re-label `*_accepted_force_budget_margin_n` in `BridgeDiagnostics.msg`
and any operator-facing tooling as "position-channel-only, excludes C1/C4
relay" so it is never read as a total-force guarantee.

## PART 10 — Force Clamp Forensics

From `FULL4CH_TEST_SUMMARY.md`'s `safety.slave.final_force_clamp`:
`count=41`, `duration_s=0.880`, `duty=1.35%`, `longest_s=0.445`,
`max_force_before_final_clamp_norm_n=1.719`, `max_applied_force_norm_n=1.5`
(exactly at the configured ceiling, confirming the clamp fired and held
the line). **Per-event force decomposition (spring/damping/aux/relay
breakdown for each of the 41 events) was not extracted in this audit** —
the raw CSV (`raw_slave.csv`) contains the needed columns
(`spring_force_n`, `damping_force_n`, `auxiliary_force_n`, `relay_force_n`)
but per-event classification requires a dedicated pass this audit's time
budget did not include. **Classification: UNKNOWN (data available, not
yet extracted).** Given `relay_applied_duty` for C4 is only 3.16% overall
while the slave clamp duty is 1.35%, and both are intermittent/brief, a
plausible hypothesis is C4 relay events coinciding with high position-error
spring excursions — but this is a hypothesis, not a traced conclusion,
and is listed as the first concrete task in the fine-tuning plan (Part 15,
item B) rather than asserted here.

1.35% duty with a 0.445s longest single interval, 41 discrete brief
events (not one continuous saturation) reads as **intermittent
commissioning-level saturation, not routine/continuous saturation** —
but this is a qualitative read of the aggregate numbers, not a
per-event-verified conclusion.

## PART 11 — Position Clamp Forensics

Master `position_error_clamped` duty ≈ 23.0%, slave ≈ 14.5%. **Not**
addressed further in this audit beyond recording the numbers — flagged
as Part 15 item A for the fine-tuning plan. No parameter was changed
here (per the explicit instruction not to auto-raise `max_position_error`).

## PART 12 — Real Output vs Reconstructed Output Audit

Checked `analysis/FULL4CH_TEST_SUMMARY.md` and `analyze_full4ch_baseline.py`'s
field naming: the summary explicitly separates
`receiving_relay_force_n` (a component) from
`receiving_applied_force_change_n` (change in the real `applied_force_n`
ground truth) — this analyzer does **not** appear to mislabel a
reconstructed sum as "applied force." **A full grep of every plotting
script/report for a mislabeled "applied force" was not completed in this
audit** (time budget) — spot check of the summary and the field names in
`analyze_full4ch_baseline.py`'s output found no violation. Classification:
`UNKNOWN` for full repo coverage, `PASS` for the specific baseline
report/summary checked.

## PART 13 — Mock-Free Baseline Verdict

**MOCK-FREE PASS WITH TOOLING DEFECTS.**

Real hardware master: evidenced (Part 2/3). Real hardware slave:
evidenced (Part 2/3). Real C1: evidenced, sign correct, correlated
(Part 4). Real C2: evidenced, gain ≈1.08 (Part 4). Real C3: evidenced,
gain ≈0.92 (Part 4). Real C4: evidenced, sign correct, correlated but
weaker (Part 4). No hidden mask found for C1/C4 mixing beyond the
disclosed, pre-existing `c1-disable-joints=shoulder` default (not
independently re-verified for this exact run's argv — Part 4 gap). No
zero channel (C1=0.5, C4=0.3, C2/C3 active). No synthetic publisher or
replay path found participating (Part 2). Qualification artifacts are
hardware-derived (real trial directories, real non-zero force evidence)
but the **gate that produced them has a structural gap** (Part 6) —
this did not, in this instance, produce a false pass, but it could in a
future run. The force-budget margin is real but **incompletely
accounted** (Part 9) — again not a false safety claim today (the hard
clamp caught it), but a metric that should not be trusted as a total-force
guarantee while C1/C4 are active. The channel-activation-mismatch report
(Part 8) is a validator bug, not evidence of masking.

## PART 14 — Architecture Freeze

See `ARCHITECTURE_FREEZE_4CH.md` in this directory.

## PART 15 — Fine-Tuning Plan

See `FINE_TUNING_PLAN_4CH.md` in this directory.

---

# PRE-TUNING CLOSURE RESOLUTION (2026-09-10, follow-up)

No hardware was touched to produce anything below. Everything here comes
from already-recorded session data (`20260910T155604Z_full4ch_current_setup_baseline`,
`20260910T153620Z_c1only_c1_qualification`, `20260910T153834Z_c4only_c4_qualification`)
and source-code tracing.

## Task 1 resolution — exact baseline mask state

**Definitive, directly-recorded evidence, not inference from source
defaults.** Two independent proofs:

1. `manifest.env` for the baseline session records the LAUNCHER'S actual
   resolved value: `FORCE_RELAY_C1_DISABLE_JOINTS=''` (empty — written by
   `04_start_force_relay.sh`'s own `printf` at launch time, not a
   default I assumed). `04_start_force_relay.sh`'s own default is
   `C1_DISABLE_JOINTS="${C1_DISABLE_JOINTS:-}"` — **empty**, which
   OVERRIDES `bilateral_4ch.py`'s own internal `--c1-disable-joints`
   argparse default (`"shoulder"`) for every launch that goes through
   this launcher. The prior audit's Part 4 gap ("the baseline session's
   actual argv value was not independently re-verified") is now closed:
   the manifest IS that independent verification, and it shows no mask
   was requested.
2. **Directly recorded, not inferred**: `bilateral_4ch.py` publishes its
   own live per-joint mask array on `/bilateral_status` (`c1_joint_mask`,
   added 2026-09-08 specifically to make this "a first-class, directly-
   observed quantity"). Read from the baseline trial's own
   `raw_bilateral.csv` (13,058 recorded samples spanning the entire
   trial): `c1_joint_mask_waist`, `c1_joint_mask_shoulder`,
   `c1_joint_mask_elbow` are **all exactly `1.0` for every single
   recorded sample, no exceptions.**
3. C4 has no per-joint mask mechanism at all — confirmed in
   `BilateralStatus.msg`'s own comment: "C4 has no equivalent per-joint
   mask in the current implementation (c4_joint_mask does not exist in
   bilateral_4ch.py)" — and confirmed by `grep -n "c4_disable\|c4_joint_mask"
   scripts/bilateral_4ch.py` returning zero matches. All three C4 joints
   are structurally unable to be masked by any existing mechanism.

```
EXACT BASELINE MASK STATE:
  C1 waist:     ACTIVE  (recorded c1_joint_mask_waist=1.0, all 13058 samples)
  C1 shoulder:  ACTIVE  (recorded c1_joint_mask_shoulder=1.0, all 13058 samples)
  C1 elbow:     ACTIVE  (recorded c1_joint_mask_elbow=1.0, all 13058 samples)
  C4 waist:     ACTIVE  (no masking mechanism exists for C4 in bilateral_4ch.py)
  C4 shoulder:  ACTIVE  (same)
  C4 elbow:     ACTIVE  (same)
```

**HIDDEN MASK = NO.**

Audit wording update: the Part 13 verdict's use of "unmasked" is now
justified for THIS baseline. Corrected final-conclusion wording (per this
task's own required phrasing rule): **"real-hardware, mock/replay-free,
unmasked full-4ch baseline."**

## Task 2 resolution — qualification gate hardened for future cycles

`hardware_validation/write_qualification_artifact.py` rewritten
(schema_version bumped `1.0.0` → `2.0.0`). It now independently computes
force evidence directly from raw CSVs (`compute_force_evidence()`,
self-contained, does not depend on `analyze_manual_motion.py`'s own
limited metrics) and REQUIRES, before writing an artifact:

- correct expected channel configuration (unchanged from before)
- `raw_bilateral.csv` and the receiving arm's raw CSV present and
  non-empty
- `live_channel_active_available` (unchanged from before)
- non-zero source disturbance (peak ≥ `MIN_SOURCE_DISTURBANCE_NM=0.02`
  N·m — reused, not invented: this is `touch_slave.cpp`'s own
  `kMinRelayTorqueNormNm` production relay-apply floor)
- non-zero commanded relay torque (same 0.02 N·m floor)
- non-zero receiving relay force (peak ≥ `MIN_RECEIVING_RELAY_FORCE_N=0.01`
  N — **PROVISIONAL**, no independently-derived force-telemetry noise
  floor exists for this project; documented inline with the reasoning
  and an explicit call to revisit with a measured quiet-state floor)
- `relay_valid_duty` ≥ 0.99, `relay_fresh_duty` ≥ 0.99 (the same booleans
  the production stale-timeout safety gate uses)
- `relay_applied_duty` > 0 (relay must have cleared the deadband at least
  once)
- per-required-joint sign correlation meeting `MIN_ABS_SIGN_CORRELATION=0.4`
  in the expected direction (negative for C1, positive for C4) —
  **PROVISIONAL**, chosen to sit below both known-real trials' weakest
  evidence (C4's 0.737) while rejecting near-zero/random correlation;
  documented inline, flagged for a proper null-distribution derivation
  later
- masked joints (per an explicit `--disabled-joints` argument, meant to
  be given the SAME value the launch actually used) are excluded from
  the sign requirement, not silently required to pass

Artifact provenance fields added: `schema_version`, `qualification_type`,
`source_session`, `git_head`, `git_dirty`, `git_branch`,
`config_fingerprint`, `analyzer_version`, `expected_channel_configuration`,
`force_evidence` (full per-joint breakdown), `verdict`.

**14 adversarial tests added**
(`hardware_validation/tests/test_write_qualification_artifact.py`, all
passing): empty/missing trial dir, malformed manifest JSON, wrong
expected channel, missing bilateral-status evidence, missing/stale
expected activation, analyzer-INVALID result, zero force evidence, wrong
sign, stale relay (`relay_fresh` false), relay never applied, and masked-
joint exclusion — plus a positive control (`test_real_looking_c1_trial_passes`)
proving the gate isn't merely strict-by-accident.

**Historical evidence vs. future gate robustness — kept explicitly
separate, per this task's own instruction:**

- Neither existing artifact (`C1_QUALIFIED.json`, `C4_QUALIFIED.json`)
  was touched, regenerated, or invalidated.
- Re-running the NEW gate against the two ORIGINAL qualification trial
  directories (read-only) shows it would **reject** both — but for a
  data-availability reason, not a force-quality reason: those two
  trials' `raw_bilateral.csv` (recorded ~15:36-15:40) predates a
  recorder-schema change and has NONE of the per-joint
  `tau_e_hat_nm_*`/`tau_h_hat_nm_*`/`master_torque_cmd_nm_*`/
  `slave_torque_cmd_nm_*` columns the new gate needs (confirmed: same
  `git_head`/`recorder_version` string as the later full4ch baseline
  trial, but a genuinely different actual CSV column set — an
  additional, real provenance gap: `recorder_version` was not bumped
  when the schema changed, so it cannot be used to distinguish the two
  schemas). The receiving-side evidence in those two trials IS present
  and real (`relay_valid_duty=1.0`, `relay_fresh_duty=1.0`,
  `relay_applied_duty=0.93`/`0.47`, peak relay force 0.73N/0.55N,
  independently re-derived from `raw_master.csv`/`raw_slave.csv` here
  and matching the original artifacts' own `force_dob` numbers) — only
  the bilateral-side per-joint sign/correlation check is unavailable for
  these two specific historical trials.
- Independently re-running the new gate against the LATER full4ch
  baseline trial's OWN `raw_bilateral.csv` (which DOES have the richer
  schema) reproduces, from fresh code, sign correlations matching
  `FULL4CH_TEST_SUMMARY.md` to 3+ significant figures (C1: -0.965/-0.998/
  -0.991 waist/shoulder/elbow; C4: +1.000/+1.000/+1.000) and **PASSES**
  the new gate outright for both C1 and C4. This is the strongest
  available evidence that C1/C4's sign/correlation properties are real
  for the architecture as a whole, even though it technically comes from
  the full4ch run rather than the two dedicated single-channel
  qualification trials.

**Conclusion**: the two existing qualification artifacts remain valid
historical evidence (their own recorded force numbers are real, verified
independently here); the gate that produced them had a real structural
weakness (confirmed, not merely suspected) which is now fixed for future
cycles; and the actual full4ch baseline result this whole audit concerns
itself with has now been independently re-verified against the new,
stronger gate and passes it directly from its own raw data.

```
QUALIFICATION GATE:            FIXED (for future cycles)
C1 FUTURE QUALIFICATION FORCE-AWARE:  YES
C4 FUTURE QUALIFICATION FORCE-AWARE:  YES
```

## Task 3 resolution — force-budget semantics made honest

Confirmed from source (unchanged from the original Part 9 finding, now
with a concrete fix applied): `coupling_force_budget.py` has zero
references to "relay" anywhere. `predict_force_vector()` sums ONLY
spring + damping + auxiliary. `*_accepted_force_budget_margin_n` /
`*_candidate_force_budget_margin_n` (published on `/bridge_diagnostics`)
are **position-channel-only predictions** and structurally cannot see
C1/C4 relay force, which is added later, entirely inside
`touch_slave.cpp`'s own Cartesian controller.

**No message schema change made** (renaming a live topic field is an
architecture-adjacent, disruptive change explicitly out of scope for
"tooling corrections only" — see Task 7). Instead:

1. Documented the exact scope of these fields in this audit (Part 9,
   preserved above) and in `ARCHITECTURE_FREEZE_4CH.md`'s "Known
   limitations."
2. Added a NEW, separate, ground-truth diagnostic:
   `actual_preclamp_force_headroom()` in `analyze_manual_motion.py` —
   computes `configured_max_force_n - |force_before_final_clamp_n|` per
   sample directly from the raw device-write force path (the same
   `force_before_final_clamp_n` ground-truth field the original audit
   already identified as authoritative), explicitly labeled distinct
   from the planner's predicted margin. `max_force_n` is supplied by the
   caller from the session manifest (not itself recorded per-sample by
   the current recorder — a minor, separately-noted gap, not fixed here
   since it would mean extending the recorder's own CSV schema, an
   architecture-adjacent change).
3. **Ran the new diagnostic against the real baseline data**:
   - Slave: `peak_preclamp_force_norm_n=1.719N` (matches the original
     audit's number exactly), `min_actual_preclamp_headroom_n=-0.219N`
     (**negative** — the real ground-truth headroom actually went
     negative at the clamp event, correctly co-occurring with
     `final_force_clamped=True` at that exact sample) — in stark,
     now-quantified contrast to the planner's reported
     `slave_accepted_force_budget_margin_n ≈ +0.85N` at the same session.
   - Master: `min_actual_preclamp_headroom_n=+1.132N` (stayed positive
     throughout — consistent with the master never final-clamping).

```
FORCE-BUDGET SEMANTICS: FIXED
  position_path_force_margin (existing planner metric, documented, unchanged)
    != actual_preclamp_force_headroom_n (new, ground-truth, this task)
  Baseline slave minimum actual headroom: -0.219 N (went negative;
    planner's own margin never reflected this, by design/limitation now
    documented, not a planner bug)
```

## Task 4 resolution — C2/C3 channel-state semantics fixed

`channel_isolation_check()` in `analyze_manual_motion.py` rewritten.
Precise fix (subtler than a first pass suggested — see the two reverted/
corrected iterations in this session's own working history, kept honest
here rather than presented as if it were right the first time):

- **"Is an expected channel really active?"** (`missing_expected_activation`)
  is now answered ONLY by that channel's true owner:
  `bridge_diagnostics` (`c2_enabled`/`c3_enabled`) for C2/C3,
  `bilateral_status` (`channel_active_0`/`channel_active_3`) for C1/C4.
- **"Is an unexpected channel accidentally active?"** (`unexpected_activation`,
  a safety check) still checks BOTH possible sources for C2/C3 — the
  bridge's own configured state AND bilateral_4ch's `channel_active` —
  because `bilateral_4ch.py` also HAS a (normally-zeroed) C2/C3
  implementation; if it ever reported active, that's a genuine duplicate-
  position-authority danger the owner-only check alone would miss. This
  preserves the exact safety property an earlier, overly-broad first
  attempt at this fix accidentally deleted (caught by the pre-existing
  test suite, not shipped).
- `live_channel_active_available` was NOT redefined to mean "any
  channel's source is available" (an earlier attempt did this and broke
  `MISSING_BILATERAL_STATUS`'s C1/C4-specific meaning) — it stays
  specifically about `bilateral_status` presence.

**Regression tests added** (`hardware_validation/tests/test_manual_motion_analysis.py`,
4 new, all passing alongside the pre-existing 20):
`test_c2_c3_active_via_bridge_not_flagged_missing_when_bilateral_reports_inactive`
(the exact full4ch bug, reproduced), `test_c2_c3_active_via_bridge_with_no_bilateral_status_at_all`,
`test_c2_active_via_bilateral_status_alone_is_flagged_as_duplicate_authority`
(the safety property preserved), `test_c1_c4_still_require_bilateral_status_as_before`.

**Re-ran the fixed analyzer against the real baseline trial** (not just
synthetic tests): `channel_isolation.missing_expected_activation` is now
`null` (was `[{"channel":"c2"},{"channel":"c3"}]`),
`unexpected_activation` remains `null`, `overall_result` remains `VALID`.

```
C2/C3 STATE SEMANTICS: FIXED (verified against real baseline data + 4 new regression tests)
```

## Task 5 resolution — baseline clamp cause, decomposed

Per-event vector decomposition of all 41 slave `final_force_clamped`
events, computed directly from `raw_slave.csv` (grouped into contiguous
clamp runs; force components read at the peak `force_before_final_clamp_n`
sample within each run). Events cluster into 4 distinct excitation bursts
(t≈44.0-44.3s, t≈45.4-45.6s, t≈54.6-54.9s, t≈55.9-56.3s), not 41
independent random moments.

Ranked, per-event, across all 41 events:

- **dominant cause = spring force** — component norm 1.33-2.10N at every
  single event, by far the largest individual term (compare: total
  clamped norm only 1.59-1.72N — vector cancellation with other terms
  means the spring's own norm can exceed the combined total, consistent
  with components not being perfectly aligned; this is real vector
  arithmetic, not an error).
- **secondary cause = auxiliary/gravity compensation** — a constant
  0.550N contribution at every single event (matches `SLAVE_GRAVITY_COMP_N`
  exactly).
- **relay (C4) contribution** — real and consistently present, but
  clearly tertiary: 0.106-0.229N (peaking at the single longest event,
  #36, 0.440s duration, t=55.92s, where relay also peaked at 0.229N —
  consistent with, not proof of, some correlation between the longest
  clamp and the strongest relay sample, not independently statistically
  tested here).
- **damping contribution** — negligible throughout, 0.002-0.024N.

**This REVISES the original audit's own tentative hypothesis** ("C4
relay coincides with clamp events" was floated as untested in the
original Part 10) — the actual per-event data shows spring dominates
overwhelmingly; C4 relay is a real but modest secondary contributor, not
the primary cause. Flagging the original hypothesis as superseded rather
than quietly dropping it.

```
SLAVE CLAMP DOMINANT CAUSE:
  dominant = spring force (position-channel/C3 tracking demand)
  secondary = gravity/auxiliary compensation (constant 0.55N)
  relay (C4) contribution = real, modest, tertiary (0.11-0.23N)
  damping contribution = negligible (<0.03N)
```

## Task 6 — this document, `ARCHITECTURE_FREEZE_4CH.md`, and
`FINE_TUNING_PLAN_4CH.md` updated accordingly (see those files' own
2026-09-10 follow-up sections/edits).

## Task 1 revisited — final integrity statement wording

Per this task's own required phrasing rule and the now-proven mask
state:

**"Real-hardware, mock/replay-free, unmasked full-4ch baseline."**
(The earlier turn's "genuinely real-hardware and unmasked" phrasing is
now fully supported by direct evidence, not merely asserted.)
