# Final report — H3/H4 manual-motion recorder tooling (2026-09-10)

Narrow continuation of Phase 5 (`FINAL_REPORT_2026-09-10_PHASE5.md`). Scope
was deliberately limited to implementing and validating the missing H3/H4
real-hardware manual-motion recorder/analyzer. No controller behavior,
command authority, channel ownership, gains, DOB equations, gravity
compensation, or safety state-machine semantics were touched.
`visualization/` was not touched. No real hardware actuation occurred.

## 1. What files were created?

- `hardware_validation/record_manual_motion.py` — subscribe-only recorder.
- `hardware_validation/analyze_manual_motion.py` — pure post-hoc analyzer.
- `hardware_validation/tests/test_manual_motion_recorder.py` — static +
  runtime "zero publishers" assertions, module-import smoke test, and
  live-hardware-refusal logic tests (against a fake node, no real ROS
  graph needed).
- `hardware_validation/tests/test_manual_motion_analysis.py` — 15 pure
  tests covering event detection, metrics, channel isolation, and
  validation classification (scenarios A/B/C/D/E/F/H/I plus
  failure-to-return and no-excitation force/DOB handling).
- `ros2_ws/src/touch_haptics/sim/tests/test_manual_motion_recorder_live.py`
  — end-to-end live-sim test: real bridge + real recorder subprocess +
  real analyzer, sine-driven simulated hand as the "manual motion" stand-in
  (same technique as `test_part3_c3_slave_tracking_sim.py`).
- This report.

## 2. What existing files were modified?

- `hardware_experiments/2026-09-09_.../ONSITE_RUNBOOK.md` — H3/H4 sections
  updated from "not yet built" to TOOLING_READY_SIM_VALIDATED /
  HARDWARE_VALIDATION_PENDING with exact commands; top-of-file H7-H9
  status line corrected to match the sections below it (was still saying
  "BLOCKED" even though those sections had already been updated to
  "prepared but hardware-unvalidated" in the prior Phase-5 pass).
- `hardware_validation/README.md` — lists the two new canonical scripts
  and the (partial) `tests/` exception.
- `hardware_validation/OFFSITE_REMAINING_WORK.md` — item 10 marked DONE.

## 3. Does the recorder contain any publisher?

**No.** `record_manual_motion.py`'s `RecorderNode` calls
`create_subscription()` four times and `create_publisher()` zero times.
Enforced two ways: (a) a static AST source-grep
(`test_recorder_source_contains_no_create_publisher_call`) that fails the
build if this file is ever edited to add one, and (b) a runtime check
(`test_recorder_node_creates_no_publishers_at_runtime`) confirming rclpy's
own publisher list for the constructed node is empty apart from the
framework's automatic `/rosout`/`/parameter_events`.

## 4. What exact topics does it subscribe to?

`--left-telemetry-topic` (default `/touch_left/cartesian_controller_telemetry`,
`CartesianControllerTelemetry`), `--right-telemetry-topic` (default
`/touch_right/cartesian_controller_telemetry`, same type),
`--bridge-diagnostics-topic` (default `/bridge_diagnostics`,
`BridgeDiagnostics`), `--bilateral-status-topic` (default
`/bilateral_status`, `BilateralStatus`). All four are read-only
subscriptions; the bilateral one is expected to have zero publishers for a
C3-only H3 trial (`bilateral_4ch.py` isn't running) — this is handled
explicitly (`bilateral_available=False` per row), not silently defaulted.

## 5. How are timestamps synchronized?

Every row stores BOTH the ROS message timestamp (`t_ros`, from
`header.stamp`) and the local monotonic receive time
(`t_recv_monotonic`, `time.monotonic()` at callback time) — never
assumed simultaneous. The analyzer's `merge_streams()` joins on
`t_recv_monotonic` via nearest-neighbor within `JOIN_TOLERANCE_S=0.05s`
(documented constant, not a value from `TELEMETRY_CONTRACT.md`, which
does not specify a join rule); a stream with nothing inside that window
gets an explicit `None` for every one of its columns at that row — never
a fabricated zero. `manifest.json`'s `stream_stats` reports, per stream:
sample count, effective Hz, max gap, and a dropout count (gaps exceeding
`2/effective_hz`), so "no measurement" is always distinguishable from
"measured zero" in both the raw output and the merged dataset.

## 6. How are anchors captured?

`RecorderNode._on_master`/`_on_slave` latch `master_anchor`/`slave_anchor`
from the FIRST telemetry sample of each stream (single atomic sample, same
pattern as `part2_master_only_y_motion.py`'s `latch_origin()`), when
`--auto-anchor` is set (default true). Every subsequent sample computes
`xyz - anchor` before any mapping is applied.

## 7. How is mapped Y computed?

The master's anchor-relative delta is run through `parse_axis_map`/
`apply_axis_map` — imported directly from `bridge_master_slave.py`, NOT
reimplemented, so the recorder can never silently drift out of sync with
the bridge's own mapping — using `FROZEN.axis_map`/`axis_scale`, applied
EXACTLY ONCE (`_mapped_relative()`). The slave is reported in its OWN
native anchor-relative frame with no map applied, since that native frame
is what C3 itself targets directly. No physical Y sign is hardcoded
anywhere — both raw-native and mapped-anchor-relative XYZ are written to
`raw_master.csv`/`raw_slave.csv` so a consumer can verify the mapping
against a specific trial's own data if the convention is ever in doubt.

## 8. How is +5mm motion detected?

`detect_events()` is a state/threshold walk over `master_y` (mapped,
anchor-relative), not a fixed-timestamp guess: BASELINE = sample 0;
MOTION_START = first index where `SUSTAIN_SAMPLES=3` CONSECUTIVE samples
all exceed `MOTION_THRESHOLD_M=1mm` in absolute value (a single-sample
spike cannot trigger it — covered by
`test_noise_is_never_classified_as_motion`); POSITIVE_PEAK (or
NEGATIVE_PEAK, if the excursion went the wrong way) = the max-|value|
sample after MOTION_START; HOLD = the contiguous region around the peak
where `|y| >= 0.85*|peak|` (a VALUE-based window, not a velocity/derivative
one — an earlier version used a per-step-delta threshold and
misclassified the entire slow, smooth outward ramp as HOLD, because a
raised-cosine profile has near-zero velocity at its own endpoints too;
fixed before this report was written, see the code comment at that
function). No thresholds are hardcoded displacement magnitudes — 5mm is
never assumed, only whatever the operator actually did.

## 9. How is return detected?

RETURN_START = first index after HOLD where `|y|` has dropped below
`(1 - RETURN_DROP_FRACTION=0.3) * |peak|`. RETURN_COMPLETE = first index
after that where `SUSTAIN_SAMPLES` consecutive samples are all within
`NOISE_BAND_M=0.5mm` of zero. If RETURN_START is never found, the trial is
flagged `FAILURE_TO_RETURN` (INVALID_TRIAL) rather than silently omitted
(see `test_failure_to_return_invalid`).

## 10. How are tracking RMSE/gain/lag computed?

RMSE/peak error: `master_y - slave_y` over every synchronized
(non-`None`) sample from MOTION_START onward. Gain: outward =
`slave_peak_y / master_peak_y`; return = `1 - |slave_return_peak| /
|slave_peak_y|`. Lag: coarse cross-correlation (same technique as
`test_part3_c3_slave_tracking_sim.py`'s own lag estimate — sample-index
based, not publication-grade), computed separately for the outward and
return windows, each requiring ≥6 synchronized samples or returning `None`
with an explicit `*_na_reason` string — never a fabricated 0.0. Below
`MIN_EXCITATION_M=1mm` peak excursion, `compute_metrics()` returns
`{"result": "NOT_EVALUABLE", "reason": ...}` for the WHOLE metrics block,
not per-field N/As (section 9's "do not report a lag value when
excitation is insufficient" is enforced at the top level, before any
per-metric computation is even attempted).

## 11. How is cross-axis coupling measured?

`metrics["cross_axis"]` reports `peak_abs(master_x)/peak_abs(master_y)`
and the Z equivalent (mapped-frame, master side), and the same ratios for
the slave (native frame, `peak_abs(slave_x)/peak_abs(slave_y)`) — `None`
when the denominator is zero/unavailable rather than a divide-by-zero or
a silently wrong number.

## 12. How is channel isolation verified?

`channel_isolation_check()` reports the CONFIGURED gate state (from the
most recent `BridgeDiagnostics` row's `c1_enabled..c4_enabled` booleans)
and the LIVE `channel_active[4]` state (from `BilateralStatus` rows, when
that stream is available) SEPARATELY, never conflated — matching
`TELEMETRY_CONTRACT.md`'s explicit distinction between the two. If
`BilateralStatus` has zero samples (the normal case for a C3-only H3
trial, since `bilateral_4ch.py` isn't running), `unexpected_activation`
is `None` (evaluated as "unknown/not applicable"), not silently treated
as "confirmed inactive" — covered by
`test_no_bilateral_status_does_not_falsely_flag_unexpected_activation`.

## 13. What makes a trial INVALID?

`validate_trial()`'s `overall_result` is `INVALID_TRIAL` if any flag has
`severity="INVALID"`: `BRIDGE_FAULT`, `UNEXPECTED_CHANNEL_ACTIVATION`,
`CONFIGURED_CHANNEL_MISMATCH`, `WRONG_DIRECTION_MOTION`, or
`FAILURE_TO_RETURN`. `FINAL_FORCE_CLAMP` and `TELEMETRY_STALE` are
`severity="FLAG"` — reported, but not by themselves grounds for
invalidation (a single clamp or a brief stale gap is not automatically
"unsafe," but must never be silently hidden either — both are always
listed in `validation.json`'s `flags` array regardless of the overall
verdict). `overall_result` is `INSUFFICIENT_EXCITATION` (distinct from
both `VALID` and `INVALID_TRIAL`) when `detect_events()` found no
sustained motion at all — a no-motion trial is not "invalid," it is a
different, equally legitimate outcome that must be reported as such
(section 9's explicit requirement).

## 14. What happens if telemetry becomes stale?

The recorder records it (per-row `dropout_gap_s`, per-stream
`stream_stats` in `manifest.json`) but takes no action itself (it is an
instrument, not a controller — staleness handling on real hardware is the
bridge's own `stale_timeout_s`/`force_budget_telemetry_stale_s`
responsibility, unmodified by this task). The analyzer flags
`TELEMETRY_STALE` if any stream's `max_gap_s` exceeds
`FROZEN.force_budget_telemetry_stale_s=0.2s` — the one frozen/documented
freshness threshold that generalizes here (section 13's "use thresholds
already frozen/documented where available").

## 15. What happens if deadman is released?

The recorder records `deadman_ok` (from `CartesianControllerTelemetry`)
per master sample — visible directly in `raw_master.csv` and plotted in
`09_deadman_enable_timeline.png`. The recorder does not act on a release
(again: instrument, not controller — the bridge's own deadman gate,
unmodified by this task, is what actually disables force). No frozen
threshold exists for "how much deadman-loss-during-a-force-stage is
tolerable," so `validate_trial()` lists this explicitly under
`thresholds_not_defined` rather than inventing a pass/fail cutoff
(section 13's explicit instruction).

## 16. What simulation scenarios were tested?

Task section 17 lists A–J. Covered:
- **A** (nominal +5mm Y and return): live-sim end-to-end
  (`test_manual_motion_recorder_live.py`, real bridge + real recorder
  subprocess + real analyzer against a sine-driven simulated hand) AND
  pure-analyzer (`test_scenario_a_nominal_c3_only_valid`).
- **B** (no-motion/insufficient excitation): pure-analyzer
  (`test_scenario_b_no_motion_insufficient_excitation`,
  `test_noise_is_never_classified_as_motion`).
- **C** (wrong-direction Y): pure-analyzer
  (`test_scenario_c_wrong_direction_invalid`).
- **D** (cross-axis contamination): pure-analyzer
  (`test_scenario_d_cross_axis_contamination_reported`,
  `test_cross_axis_clean_trial_reports_near_zero_ratio`).
- **E** (telemetry dropout/stale interval): pure-analyzer
  (`test_scenario_e_dropout_flagged_via_stream_stats`,
  `test_no_dropout_does_not_flag_stale`).
- **F** (unexpected C2 activation): pure-analyzer
  (`test_scenario_f_unexpected_c2_activation_invalid`,
  `test_no_bilateral_status_does_not_falsely_flag_unexpected_activation`).
- **H** (bridge FAULT during trial): pure-analyzer
  (`test_scenario_h_bridge_fault_invalid`).
- **I** (final-force clamp during trial): pure-analyzer
  (`test_scenario_i_final_force_clamp_flagged_not_necessarily_invalid`).

NOT covered, honestly:
- **G** (unexpected C1/C4 activation) — uses the exact same
  `UNEXPECTED_CHANNEL_ACTIVATION` code path as F (channel index is a
  parameter, not special-cased logic), so F stands in for it rather than
  a separately written, functionally-identical test.
- **J** (repeated +Y/return cycles) — `detect_events()` is a
  first-crossing state machine (BASELINE→MOTION_START→PEAK→RETURN_START→
  RETURN_COMPLETE) and does not attempt multi-cycle segmentation. A trial
  with multiple excursions will have its FIRST cycle detected correctly
  and everything after RETURN_COMPLETE simply included in the merged
  dataset/plots without further event markers. This is a genuine, not
  silently claimed, limitation — repeatability analysis (section 9's
  "repeatability metrics if multiple cycles are present") is NOT
  implemented this pass.

A significant unplanned finding while building the live-sim test: this
machine has the REAL hardware driver nodes (`teleop_master_left`/
`teleop_slave_right`, idle, `enabled_on_start=false`) running persistently
in the background on the default ROS domain (confirmed via `ps aux`,
running since 2026-09-09 — not started by this task). The recorder's own
live-hardware refusal check correctly detected and refused to attach to
this domain on the FIRST attempt to run the live-sim test — proof the
safety check works as designed, not a defect. All live-sim testing for
this task was then run under `ROS_DOMAIN_ID=42` (set at test-module import
time, before any rclpy-touching code) to stay fully isolated from that
live domain; see the comment at the top of
`test_manual_motion_recorder_live.py`.

## 17. What tests passed/failed?

- New pure tests: 22 (`test_manual_motion_recorder.py`: 7 — 4 publisher/
  import assertions + 3 live-hardware-refusal-logic tests against a fake
  node; `test_manual_motion_analysis.py`: 15) — all passing.
- Combined pure/unit regression (`scripts/` + `hardware_experiments/2026-
  09-09_.../` + `hardware_validation/`): **87 passed, 0 failed** (was 65
  before this task; +22 new, 0 regressed).
- New live-sim test (`test_manual_motion_recorder_live.py`): 1 test,
  passing in isolation (27.23s).
- Full combined live-sim regression, 16 files including the new one, run
  together under `ROS_DOMAIN_ID=42` (`test_c2_live.py`, `test_c3_live.py`,
  `test_gravity_comp_arm_gate_live.py`, `test_gravcomp_trial_recorder_live.py`,
  `test_force_budget_coupling_live.py`,
  `test_sim_initial_condition_repeatability.py`, `test_force_budget_117mm_live.py`,
  `test_fail_safes_live.py` (2 tests), `test_fail_safe_telemetry_stale_live.py`,
  `test_staged_disarm_live.py`, `test_gap_matrix_live.py`,
  `test_part2_displacement_candidates_sim.py`, `test_part3_c3_slave_tracking_sim.py`,
  `test_master_deadman_gate_live.py` (2 tests), `test_channel_isolation_live.py`
  (2 tests), `test_manual_motion_recorder_live.py`): **19 passed, 0 failed**
  (319.23s) — was 18 before this task; +1 new, 0 regressed.
- **Combined grand total: 106 passed, 0 failed, 0 skipped** (87 pure + 19
  live-sim).

## 18. Did you modify controller behavior?

**NO.** No change to `bridge_master_slave.py`, `bilateral_4ch.py`,
`startup_alignment.py`, `touch_slave.cpp`, or any `.msg` file. The
recorder imports (does not modify) `parse_axis_map`/`apply_axis_map` from
`bridge_master_slave.py` for read-only mapping consistency.

## 19. Did you modify visualization/?

**NO.**

## 20. Did any real hardware actuation occur?

**NO.** Every test this task ran was against simulation
(`touch_slave_sim`, linked against `hd_sim`) under an isolated
`ROS_DOMAIN_ID=42`. The one interaction with real-hardware-adjacent state
was read-only: the recorder's live-hardware detection check observed (via
`ros2` graph introspection) that the real driver nodes were present on the
DEFAULT domain, logged that fact, and refused to proceed — it never
subscribed to, published to, or otherwise touched those nodes' topics.
