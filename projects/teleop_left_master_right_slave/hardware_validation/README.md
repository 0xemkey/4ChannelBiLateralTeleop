# hardware_validation/

The canonical home for real-hardware validation tooling (2026-09-10). This
directory does **not** overlap `visualization/` (Codex-owned: simulation
UI, synthetic visualization, replay) — this directory is architecture,
safety, and physical-data-capture tooling only. No dashboard is built here.

## What's here vs. what's aliased

**Canonical, net-new here:**
- `frozen_config.py` — the ONE authoritative parameter table (item 7).
  The older copy at `hardware_experiments/2026-09-09_.../frozen_config.py`
  now re-exports from this file rather than duplicating it.
- `preflight.py` — read-only ROS-graph ownership + config-fingerprint
  check, distinguishing `IDLE_TOPOLOGY_PASS` /
  `STAGE_LAUNCHED_UNARMED` / `ARMED_TOPOLOGY_ELIGIBLE` (item 14).
- `safe_stop.py` — session-specific stop, dry-run by default, never a
  hardcoded PID.
- `resolve_driver_session.py` (2026-09-10, H2 safety-hardening) — resolves
  the CURRENTLY-RUNNING master/slave driver session from live PGIDs
  (never a hardcoded/reused session id); fails closed on any ambiguity.
- `pre_arm_check.py` / `verify_disarmed.py` (2026-09-10, H2 safety-
  hardening) — PRE-ARM (before `arm_teleop` is ever sent) and POST-DISARM
  (fresh subscription after a run) live verification, structurally
  separate per their own module docstrings. Both used by
  `part1_gravity_settle_capture.py`/`run_h2.sh`.
- `record_manual_motion.py` (2026-09-10, H3/H4 recorder task) — the
  real-hardware H3/H4 manual-motion recorder. SUBSCRIBE-ONLY (creates zero
  publishers; enforced by a static source-grep test) -- refuses to start
  if it detects the real `teleop_master_left`/`teleop_slave_right` driver
  nodes in an unexpected ROS graph. Status:
  TOOLING_READY_SIM_VALIDATED / HARDWARE_VALIDATION_PENDING.
- `analyze_manual_motion.py` (2026-09-10) — pure post-hoc analysis for the
  recorder's output: event detection (BASELINE/MOTION_START/PEAK/HOLD/
  RETURN_START/RETURN_COMPLETE), tracking/gain/lag/cross-axis/force
  metrics, channel-isolation and safety validation, plots. No ROS
  dependency at all -- directly unit-testable.
- `TELEMETRY_CONTRACT.md` — see "See also" below.
- `OFFSITE_REMAINING_WORK.md` — see "See also" below.

**Existing capability this directory reuses rather than duplicates** (still
living in `hardware_experiments/2026-09-09_y_axis_validation_PENDING_SUPERVISION/`
— not moved, to avoid breaking already-passing tests mid-task):
- `part1_gravity_settle_capture.py` — this IS the H2 stage script
  (gravity-settle-only capture). Not renamed to `record_all.py`; it already
  does exactly what that name implies for H2 specifically.
- `part2_master_only_y_motion.py` — the self-actuated (Procedure B, NOT
  recommended) master-Y test.
- `plot_y_axis_validation.py` — the analysis/plot generator for whatever
  this task calls "analyze_run.py"/"generate_report.py" — same
  functionality, different name; not force-split into two files without a
  concrete reason to.
- `ONSITE_RUNBOOK.md`, `CHANNEL_READINESS_MATRIX.md`,
  `COMMAND_AUTHORITY_ADR.md`, `PREP_REPORT.md`, `FINAL_REPORT_2026-09-10.md`
  — all still in that directory; this README does not re-home them a
  second time.

**Explicitly NOT built this pass** (see the final report's blocker list):
- `session_manager.py` as a standalone module — session ID generation and
  PID bookkeeping already exist in `experiment_launch/common.sh`
  (`parse_common_args`, `start_component`) and are reused by
  `safe_stop.py` above rather than reimplemented in Python.
- `mark_event.py` — `experiment_launch/07_mark_event.sh` already exists
  and does this; not duplicated.
- `hardware_validation/tests/` (partial exception, 2026-09-10) — the
  recorder/analyzer's OWN tests now live here
  (`tests/test_manual_motion_recorder.py`,
  `tests/test_manual_motion_analysis.py`), since they test files that are
  ALSO canonically homed here; the live-sim end-to-end test for the
  recorder still lives with every other live-sim test at
  `ros2_ws/.../sim/tests/test_manual_motion_recorder_live.py`, unmoved,
  for the same relative-import reason as everything else in this list.

## Quick start (dry-run, safe to run any time)

```
python3 preflight.py                              # read-only ownership check
python3 safe_stop.py                               # prints usage, touches nothing
python3 safe_stop.py --session <ID> --dry-run       # shows what a real stop would target
```

## Canonical configuration

```python
from frozen_config import FROZEN, bridge_cli_args, config_fingerprint, write_manifest
```

`FROZEN` is the single source for every K/D/Fmax/position-error-limit/
gravity/deadman/channel-gate/rate/timeout parameter this project uses for
hardware validation. See its own module docstring for the specific
reconciliation of `left_max_offset_m` (0.08, not the historically-narrated
0.15 or the C2-diagnostic-only 0.05) and `left_gravity_comp_n` (0.0, not
the unswept 0.45).

## See also

- `CHANNEL_READINESS_MATRIX.md` — what's actually validated, per channel.
- `COMMAND_AUTHORITY_ADR.md` — why the bridge is the sole final-enable
  authority, and what changed in `bridge_master_slave.py`/
  `bilateral_4ch.py` to make that true.
- `ONSITE_RUNBOOK.md` — H0–H10 staged procedure.
- `TELEMETRY_CONTRACT.md` — the stable topic/field contract for Codex's
  `visualization/` track (force decomposition, channel/bilateral status,
  stage/ownership state).
- `OFFSITE_REMAINING_WORK.md` — what genuinely cannot be done without
  physical hardware access or a user decision, itemized with reasons.
- `hardware_experiments/2026-09-09_.../FINAL_REPORT_2026-09-10.md` (Phase
  4) and `FINAL_REPORT_2026-09-10_PHASE5.md` (this directory, Phase 5) —
  exact test totals, blockers, and the answer to "is this ready," per
  phase.
