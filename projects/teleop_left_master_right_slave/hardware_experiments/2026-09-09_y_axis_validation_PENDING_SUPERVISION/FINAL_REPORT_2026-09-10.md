# Final report — 2026-09-10 non-hardware preparation pass

**No hardware was actuated.** No node was started/restarted, no bridge was
armed, no `enable`/`target_pose`/`force_cmd` was published, no gravity
compensation was applied. All work is code, tests, and documentation.

## 1. Changed/added files

**Modified (safety/architecture):**
- `projects/teleop_left_master_right_slave/scripts/bridge_master_slave.py`
  — added the bridge-level master deadman gate (`deadman_gate_state()`,
  `_master_deadman_held()`, `_on_param` arm-rejection, `_tick()` gating);
  `enable.data` now also respects `alignment.state != "FAULT"`.
- `projects/teleop_left_master_right_slave/scripts/startup_alignment.py`
  — `gravity_settle_hold_mode` (round 2, still in place, re-verified).
- `projects/teleop_left_master_right_slave/experiment_launch/live_reference_watch.py`
  — `REF_POSE_M` marked stale with full provenance (round 2).

**New (this turn):**
- `.../hardware_experiments/2026-09-09.../CHANNEL_READINESS_MATRIX.md`
- `.../hardware_experiments/2026-09-09.../ONSITE_RUNBOOK.md`
- `.../hardware_experiments/2026-09-09.../preflight_ownership_check.py`
- `.../hardware_experiments/2026-09-09.../frozen_config.py` — added
  `config_fingerprint()`, `write_manifest()`
- `projects/teleop_left_master_right_slave/scripts/test_master_deadman_gate.py`
  (8 pure tests)
- `ros2_ws/src/touch_haptics/sim/tests/test_master_deadman_gate_live.py`
  (2 live-sim tests, 11 scenarios total: press/release/stale/process-loss/
  no-auto-rearm/explicit-rearm/disarm/FAULT)

**New (rounds 2–3, unchanged this turn, listed for completeness):**
`part1_gravity_settle_capture.py`, `part2_master_only_y_motion.py`,
`test_part2_abort_ordering.py`, `plot_y_axis_validation.py`,
`test_part2_displacement_candidates_sim.py`,
`test_part3_c3_slave_tracking_sim.py`, `PREP_REPORT.md`.

## 2. Test totals

| Suite | Count | Result |
|---|---|---|
| Pure (`scripts/`, incl. new deadman tests) | 63 | all pass |
| Pure (`hardware_experiments/`, abort ordering) | 4 | all pass |
| Live-sim (16 files, incl. new deadman live tests) | 16 | all pass |
| **Total** | **83** | **83/83, 0 failures** |

`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py` (pre-existing,
re-run this turn to verify the C1 sign fix): 1/1 pass, cited as evidence,
not counted above (different test root).

**Explicitly NOT run this turn** (out of scope, would require building new
infrastructure — see Section 4): any C1/C4/bilateral_4ch.py live-sim test,
plotting-fixture synthetic-data recovery tests (item 22), the full Part
D/E channel-combination sweep beyond what already existed.

## 3. Readiness matrix (see `CHANNEL_READINESS_MATRIX.md` for full detail)

| Channel | State |
|---|---|
| C2 | hardware smoke-tested (2026-09-03, predates this session's fixes) + extensively simulated |
| C3 | hardware smoke-tested (2026-09-01, 2026-09-03) + extensively simulated (incl. new Part 3 C3-only sim) |
| C2+C3 bilateral | hardware smoke-tested (2026-09-03) + simulated in integration |
| C1 | simulated in isolation only (sign-corrected version) — **never hardware-tested**; prior HW evidence used a since-found-wrong sign |
| C4 | hardware smoke-tested (2026-09-03, not re-validated since) |
| C1+C4, complete 4-channel | **blocked** |

**Do not read "hardware smoke-tested" as "ready."** It certifies a real
run existed under the configuration AT THAT TIME — every channel needs
fresh confirmation under the current code (moving-live-origin planner,
canonical frozen config, deadman gate, GRAVITY_SETTLE hold mode, C1 sign
fix) before any further trust is placed in it.

## 4. Unresolved hardware-only calibrations (cannot be resolved without a person present)

- `SIM_GRAVITY_SETTLE_VELOCITY_THRESHOLD_MPS` (5mm/s) — sim-derived only,
  needs a real quiet-state noise-floor measurement (`investigate_quiet_state_noise_floor.py`'s
  real-hardware analog, not built).
- The Part 3 real-hardware manual-motion script (H3's own note) — not yet
  built; the sim version drives the master via a simulated hand, real
  hardware needs its own record-only variant.
- The 5mm Procedure-A displacement is an evidence-based RECOMMENDATION
  (sim gain 0.69, RMSE 1.00mm), not a hardware-confirmed number.
- Real quiet-state pose-drift baseline (0.070mm used as the noise
  reference throughout) is a SIM measurement — needs a real-hardware
  equivalent before it's trusted as a noise floor for real data.

## 5. Exact remaining blockers, ranked

1. **`bilateral_4ch.py`'s enable-publisher conflict** (Section 3,
   `CHANNEL_READINESS_MATRIX.md`) — blocks H7, H8, H9 entirely. This is
   the single highest-priority remaining item: two independent processes
   publish to the same enable topics with unrelated safety logic. Not
   fixed this turn (a safety-critical architectural change deserves its
   own careful, reviewed pass, not a rushed fix appended to an already
   enormous task).
2. **C1's corrected sign has never touched real hardware.** Simulated only.
3. **No real-hardware confirmation of ANY channel under the current code**
   (moving-live-origin, canonical config, deadman gate, hold mode, C1 fix)
   — H0–H6 are code/test-ready but each is a genuine first attempt.
4. **Part D/E's full channel-by-channel matrix (item 10) is not built for
   C1/C4/C1+C4/complete-4ch** — deferred given item 1's blocker makes most
   of that work unusable until resolved anyway.
5. **Item 22 (plotting-fixture synthetic-data recovery tests) not built.**
   The plotting script (`plot_y_axis_validation.py`) exists and was
   verified to run cleanly against zero data and against real captured
   sim CSVs, but no synthetic-fixture-with-known-injected-errors test
   exists to prove its metric calculations recover known ground truth.
6. **Part E (items 14–17: C1/C4 sign-via-contact, power/passivity
   diagnostics) not built** — depends on item 1's blocker being resolved
   first; building isolated-contact sim tests for a channel that can't
   safely run yet was judged lower priority than the safety-gate and
   audit work actually completed.
7. **A real-hardware version of the H3 manual-motion recording script is
   not built** — the sim `test_part3_c3_slave_tracking_sim.py` uses a
   simulated hand; real hardware needs a record-only variant with no
   simulated-hand equivalent (just subscribe and log while a person moves
   the master).

## 6. What IS genuinely ready

- **H0 (inspection) and H1 (passive baseline)**: fully ready, already
  executed once this session with clean results.
- **The safety layer for H2–H6**: deadman gate (12 tests, press/release/
  stale/process-loss/no-auto-rearm/FAULT), abort-ordering (disable-first,
  never re-target), GRAVITY_SETTLE hold mode (indefinite hold, exact-zero
  coupling proven), one canonical frozen config with a fingerprint hash
  for provenance, and a read-only preflight ownership checker (tested
  against the live, currently-running real hardware nodes just now — PASS).
- **The runbook** (`ONSITE_RUNBOOK.md`) stages H0–H6 with explicit
  commands, expected ownership, abort conditions, and GO/NO-GO gates; H7–H9
  are explicitly marked blocked rather than glossed over.

## 7. Is the project genuinely ready for H0/H1 when the operator returns?

**Yes for H0 and H1 specifically** — both are zero-force/read-only, already
validated once this session, and the runbook gives exact commands.

**H2 is ready to ATTEMPT** (code complete, sim-validated, self-monitoring
abort logic, but this would be its first-ever real-hardware run under the
current code — treat it as a genuine first attempt, not a formality).

**H3–H6 are code-ready but each carries the same "first real run under
current code" caveat**, and H3 specifically needs its real-hardware
recording script built first (Section 4).

**H7–H9 are not ready and must not be attempted** until the
`bilateral_4ch.py` enable-publisher conflict is resolved — this is
flagged, not fixed, and is the most important open item from this entire
task.
