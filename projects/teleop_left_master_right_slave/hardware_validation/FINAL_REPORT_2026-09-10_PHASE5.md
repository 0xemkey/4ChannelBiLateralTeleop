# Final preparation report — Phase 5 (2026-09-10)

Continuation of `hardware_experiments/2026-09-09_y_axis_validation_PENDING_
SUPERVISION/FINAL_REPORT_2026-09-10.md` (Phase 4). This report covers only
what changed in this phase; it does not repeat Phase 4's content.
Codex's `visualization/` directory was not touched.

## What changed this phase

- Command-authority conflict eliminated: bridge now owns both arms'
  `/enable` unconditionally; `bilateral_4ch.py`'s own enable-publishing
  now defaults off (`--publish-enable` opt-in). See
  `hardware_experiments/.../COMMAND_AUTHORITY_ADR.md`.
- Master-deadman gate (`deadman_gate_state`) built and composed into the
  single enable decision, including a FAULT override and stale-latch
  behavior requiring explicit re-arm.
- C3 made genuinely gateable (`--c3-enable`, previously always-on).
  C1/C4 gates (`--c1-enable`/`--c4-enable`) added, composing
  `BilateralStatus.safety_state` fail-closed.
- `hardware_validation/` canonical package built: `frozen_config.py`
  (moved here, old location re-exports), `preflight.py`, `safe_stop.py`,
  `README.md`, this report, `TELEMETRY_CONTRACT.md`,
  `OFFSITE_REMAINING_WORK.md`.
- `ONSITE_RUNBOOK.md` H2–H9 rewritten: explicit per-stage channel-active
  tables, deadman requirement extended to H2 (gravity-only), H7–H9 moved
  from "BLOCKED" to "prepared but hardware-unvalidated."
- `part1_gravity_settle_capture.py` corrected to launch with
  `enable_c2=False, c3_enable=False, require_master_deadman=True`
  (previously launched with `enable_c2=True` and no deadman requirement —
  wrong for a gravity-only stage).
- New live-sim tests: `test_master_deadman_gate_live.py` (2 tests),
  `test_channel_isolation_live.py` (2 tests).

## Test totals (this phase, full commands re-run to produce these numbers)

- Pure/unit tests (`scripts/`, `hardware_experiments/2026-09-09_.../`,
  `hardware_validation/`, ROS2 env sourced): **65 passed, 0 failed.**
- Live-sim tests (`ros2_ws/src/touch_haptics/sim/tests/`, 15 files
  including the 2 new ones, run together in one batch to rule out
  isolation-only passes): **18 passed, 0 failed** (295.69s).
- **Combined: 83 passed, 0 failed, 0 skipped.**
- The one observed flake this phase (`test_master_deadman_gate_live.py`
  failing when run in a 3-file batch, passing in isolation) was
  re-verified as resource contention, NOT a regression — it passed
  cleanly as part of THIS phase's full 15-file, 18-test batch run above.

## Answers to the 10 final questions

1. **Was the multi-publisher conflict fully eliminated?** Yes, for
   `/touch_left/enable` and `/touch_right/enable` — exactly one publisher
   each (`bridge_master_slave.py`), verified by code (the old
   `master_enable_pub`/gated `left_enable_pub` were removed/consolidated)
   and by `hardware_validation/preflight.py`'s `UNSAFE_COMBOS` check.
   Not independently re-verified with `bilateral_4ch.py` actually running
   alongside the bridge on real hardware (that requires H7-level access).

2. **Is each stage genuinely isolated?** C2-only and observation-only are
   PROVEN isolated in sim (`test_channel_isolation_live.py`, slave
   tracking error stays ~0 with C3 off). C3-only, C1-only, C4-only, and
   every combination stage are ARCHITECTURALLY isolated (each gate is an
   independent boolean feeding independent code paths) but not
   individually sim-tested this phase beyond C2-only/observation-only.

3. **Which channels are code-ready?** All four (C1–C4) have working,
   gateable, tested-in-isolation-where-tested code paths. C2/C3 have
   real-hardware history (pre-existing, from before this phase) under the
   OLD (non-gated) architecture; behavior under the NEW gated architecture
   is sim-validated only.

4. **Which channels remain hardware-unvalidated?** C1 (corrected sign),
   C4 (since 2026-09-03), and C2/C3 specifically UNDER the new gating +
   deadman architecture (their old, pre-gating hardware history does not
   carry over as evidence for the current code paths).

5. **Which tasks are strictly impossible without physical access?** All
   of H2–H9's actual execution; C1/C4 hardware sign/behavior validation;
   the 8 remaining DOB/Kd/gravity TODO items; the H3 deadman-ergonomics
   question (needs a real operator's hands). Full list:
   `OFFSITE_REMAINING_WORK.md`.

6. **Are H0-H9 scripts ready?** H0/H1: yes, complete. H2: yes, code-ready
   (this phase's correction applied). H3/H4: NO — the real-hardware
   recorder script does not exist yet. H5/H6: yes, code-ready (existing
   scripts + new `--c3-enable`/`--require-master-deadman` flags). H7-H9:
   commands specified in `ONSITE_RUNBOOK.md` but no dedicated launcher
   script exists for them yet (documented as manual two-process commands).

7. **What exact command should be run first when the operator arrives?**
   `python3 hardware_validation/preflight.py` (read-only, confirms
   `IDLE_TOPOLOGY_PASS` before anything is launched), then H0/H1 per
   `ONSITE_RUNBOOK.md`.

8. **What files did you modify?** `startup_alignment.py`,
   `bridge_master_slave.py`, `bilateral_4ch.py`,
   `BridgeDiagnostics.msg`, `part1_gravity_settle_capture.py`,
   `ONSITE_RUNBOOK.md`; created `hardware_validation/{frozen_config.py,
   preflight.py, safe_stop.py, README.md, TELEMETRY_CONTRACT.md,
   OFFSITE_REMAINING_WORK.md, this report}`,
   `test_master_deadman_gate_live.py`, `test_channel_isolation_live.py`,
   `COMMAND_AUTHORITY_ADR.md` (created in the immediately-preceding
   sub-phase, listed here for completeness). Did NOT touch
   `visualization/`.

9. **What tests passed or failed?** 83/83 passed, 0 failed, 0 skipped
   (65 pure + 18 live-sim). Exact commands and counts in the "Test
   totals" section above.

10. **Did any real hardware actuation occur?** **No.** Every change this
    phase was source-code, documentation, or simulation-backed testing.
    No `ros2 param set arm_teleop true` was ever issued against a real
    hardware bridge process, no `enable=true` was published to a real
    device topic, and no calibration/gravity/motion command was sent to
    physical hardware at any point.
