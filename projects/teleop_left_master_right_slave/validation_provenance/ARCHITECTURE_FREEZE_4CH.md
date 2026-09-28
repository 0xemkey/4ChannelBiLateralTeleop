# 4-Channel Architecture Freeze — 2026-09-10

Baseline session: `20260910T155604Z_full4ch_current_setup_baseline`
(see `CURRENT_FULL4CH_AUDIT.md` for the full audit this freeze is based
on — verdict: MOCK-FREE PASS WITH TOOLING DEFECTS, architecture-level
findings are real and unmasked; the two defects found are in
reporting/gating tools, not in the physical signal path).

## Process topology

```
teleop_master_left (touch_slave, device=Left)
  <-> /touch_left/{pose,target_pose,force_cmd,enable,applied_force,
       joint_torque_ext_cmd,cartesian_controller_telemetry,dob_diagnostics}

teleop_slave_right (touch_slave, device="Default Device")
  <-> /touch_right/{pose,target_pose,force_cmd,enable,applied_force,
       joint_torque_ext_cmd,cartesian_controller_telemetry,dob_diagnostics}

touch_master_slave_bridge (bridge_master_slave.py)
  subscribes: /touch_left/pose, /touch_right/pose,
              /touch_left/cartesian_controller_telemetry,
              /touch_right/cartesian_controller_telemetry
  publishes:  /touch_left/target_pose, /touch_right/target_pose,
              /touch_left/enable, /touch_right/enable, /bridge_diagnostics
  OWNS: C2, C3, and the single /enable authority (both arms).

bilateral_4ch (bilateral_4ch.py)
  subscribes: /touch_left/force_ext_hat, /touch_right/force_ext_hat,
              /touch_left/joint_angles, /touch_right/joint_angles
  publishes:  /touch_left/joint_torque_ext_cmd,
              /touch_right/joint_torque_ext_cmd, /bilateral_status
  OWNS: C1, C4 only. C2/C3 weights held at 0 in this architecture
        (--publish-enable NOT used; the bridge is the single /enable
        authority).
```

## C1/C2/C3/C4 semantics (as implemented, not as originally planned)

- C2 (slave position → master), C3 (master position → slave): Cartesian,
  implemented entirely inside `bridge_master_slave.py`
  (`compute_c2_target`/`compute_c3_target`) → each arm's own
  spring/damper Cartesian controller in `touch_slave.cpp` →
  `HD_CURRENT_FORCE`. `bilateral_4ch.py` plays NO role in C2/C3 in this
  architecture (its own joint-space C2/C3 path exists in code but is
  kept at weight 0 here).
- C1 (slave DOB torque estimate → master force), C4 (master DOB torque
  estimate → slave force): implemented in `bilateral_4ch.py`
  (`tau_c1 = -w1 * cf * tau_e_hat * mask`, `tau_c4 = +w4 * cf * tau_h_hat`),
  published as joint torque to `joint_torque_ext_cmd`, converted to
  Cartesian force by the RECEIVING `touch_slave.cpp`'s own "C1/C4-onto-
  Cartesian relay" block (Jacobian-inverse-transpose), added to that
  arm's Cartesian force before the final clamp, gated by
  `command.enabled && deadman_ok && !near_singular && !joint_space_control_`
  plus a freshness check (`stale_timeout_s`) and a deadband floor
  (0.02 N·m). **Not** gated by `TOUCH_DOB_ENABLE_TORQUE_WRITE` — that
  compile-time flag only affects the SEPARATE joint-space PD write path,
  which is not used in this architecture (`joint_space_control=false`
  throughout).

## Enable ownership

`bridge_master_slave.py` is the sole `/touch_left/enable` and
`/touch_right/enable` publisher. `bilateral_4ch.py` is launched with
`--publish-enable` OFF (the default) in every launcher audited
(`08_qualify_c1_only.sh`, `09_qualify_c4_only.sh`,
`10_start_full_4ch_production.sh` via `04_start_force_relay.sh`).

## Runtime gains (baseline session, commissioning — not validated optima)

Master: K=15.0 N/m, D=0.10 Ns/m, Fmax=2.0N, max_position_error=0.03m,
gravity comp 0.55N (Y). Slave: K=35.0 N/m, D=0.20 Ns/m, Fmax=1.5N,
max_position_error=0.04m, gravity comp 0.55N (Y). C1 weight=0.5, C4
weight=0.3, Cf=1.0. DOB `g_dis=[10,10,5] rad/s` (`dob_g_dis_scale=1.0`,
unchanged baseline).

## Masks

`c1-disable-joints` defaults to `shoulder` in `bilateral_4ch.py` (a
disclosed, pre-existing per-rig finding about the right-arm DOB floor,
not a hidden mask) — **the exact value used in the baseline session's
argv was not independently re-verified in this audit** (see
`CURRENT_FULL4CH_AUDIT.md` Part 4). No other masking found.

## Safety gates

- Startup alignment state machine (`startup_alignment.py`):
  `WAIT_FOR_POSES → HOLD_STILL → CAPTURE_ANCHORS → VALIDATE_ALIGNMENT →
  READY → GRAVITY_RAMP_UP → GRAVITY_SETTLE → COUPLING_RAMP_UP → ACTIVE`,
  any state → `FAULT`.
- `planner_parameter_match_left/right`: bridge cross-checks its declared
  K/D/Fmax/max_error against each controller's own live telemetry; a
  mismatch faults the state machine (real bug fixed earlier this session:
  `run_master_left.sh` used to only declare these params when
  `ENABLE_C2=true`).
- Adaptive force-budget ramp (`coupling_force_budget.py`): bisects the
  coupling ramp to keep predicted (position-channel-only — see audit
  Part 9) force under `max_force_n - force_reserve_n`.
- Final hard force clamp in `touch_slave.cpp`: caps `applied_force_n` at
  `max_force_n` regardless of source (spring+damping+aux+relay combined).
  Confirmed firing correctly in the baseline (41 brief events, applied
  force never exceeded 1.5N even when pre-clamp force reached 1.719N).
- `reject_live_component` / PID-file ownership checks (this session's
  `run_hardware_test.sh`) prevent duplicate bridge/relay instances.
- `arm_production_bridge.py`: single-instance check, publisher-ownership
  check, pre/post-arm telemetry freshness, state-progression verification,
  channel-state verification, and (added this session) an explicit
  positive-force-budget-margin assertion before declaring ACTIVE.

## Qualification artifacts

`hardware_validation/qualification/C1_QUALIFIED.json`,
`C4_QUALIFIED.json` — both real, hardware-derived, non-zero force
evidence present. **Known gap** (see audit Part 6): the gate that writes
them does not itself check force sign/correlation/magnitude thresholds —
recommend fixing before the NEXT qualification cycle, not required to
invalidate this cycle's artifacts (their underlying trial data is
independently verified, in this audit, to show real correlated,
correctly-signed force).

## Baseline session

`20260910T155604Z_full4ch_current_setup_baseline` — see
`CURRENT_FULL4CH_AUDIT.md` for full evidence.

## Known limitations (do not silently fix by loosening a gate)

**2026-09-10 pre-tuning closure update: items 1-3 and 5 below are
RESOLVED — see `CURRENT_FULL4CH_AUDIT.md`'s "PRE-TUNING CLOSURE
RESOLUTION" section for the full evidence. Kept here, marked resolved,
rather than deleted, so the freeze record shows what was found AND what
was done about it.**

1. ~~Force-budget margin excludes C1/C4 relay force~~ — **RESOLVED**:
   confirmed from source (still true, `coupling_force_budget.py` has no
   relay term), now explicitly documented as "position_path_force_margin"
   scope, and a new ground-truth diagnostic
   (`actual_preclamp_force_headroom()`) was added and run against the
   baseline: slave's real headroom went to **-0.219N** at the clamp event
   (planner's own margin still reported +0.85N at the same session — the
   gap is now quantified, not just asserted). The hard clamp caught the
   actual excursion; no device was over-driven.
2. ~~Qualification gate does not verify force sign/correlation~~ —
   **RESOLVED** for future cycles: `write_qualification_artifact.py`
   rewritten (schema 2.0.0) to independently compute and REQUIRE source
   disturbance, commanded torque, receiving relay force, relay_valid/
   fresh/applied duty, and per-joint sign correlation from raw telemetry,
   with 14 new adversarial tests. Historical artifacts unchanged
   (neither regenerated nor invalidated) — see the audit doc for the
   historical-vs-future distinction.
3. ~~Channel-activation mismatch conflates bilateral_status with C2/C3
   truth~~ — **RESOLVED**: `channel_isolation_check()` fixed to source
   C2/C3's positive confirmation from `bridge_diagnostics` while still
   checking `bilateral_status` for unexpected/duplicate C2/C3 authority
   (a safety property preserved, not dropped). 4 new regression tests;
   re-verified against real baseline data (`missing_expected_activation`
   now correctly `null`).
4. C4's source→relay correlation (0.737) and `relay_applied_duty` (3.16%)
   are both markedly weaker than C1's (0.944 / 23.9%) — real, unexplained
   asymmetry, **still open**, tracked as fine-tuning investigation item C
   in `FINE_TUNING_PLAN_4CH.md`, not yet root-caused.
5. ~~`c1-disable-joints` argv value for the baseline session not
   independently re-verified~~ — **RESOLVED**: `manifest.env` records
   `FORCE_RELAY_C1_DISABLE_JOINTS=''` (empty, launcher-written at run
   time) AND the baseline's own `raw_bilateral.csv` records
   `c1_joint_mask_{waist,shoulder,elbow}=1.0` for all 13,058 samples.
   **HIDDEN MASK = NO**, directly evidenced.

New item found during closure:
6. The two original C1-only/C4-only qualification trials' `raw_bilateral.csv`
   (recorded ~15:36-15:40) lacks the per-joint columns the new
   qualification gate needs, despite recording the SAME `git_head` and
   `recorder_version` string as the later full4ch baseline trial (~18:56)
   which DOES have them — `recorder_version` was not bumped when the
   recorder's schema changed. This is a real provenance gap: the version
   string alone cannot currently distinguish these two schemas. Not fixed
   in this closure pass (would mean editing `record_manual_motion.py`'s
   `RECORDER_VERSION` bump discipline — a process fix, tracked, not
   applied here to stay within "tooling corrections, no architecture
   changes").

## Freeze declaration

**ARCHITECTURE FREEZE: APPROVED.**

The process topology, channel semantics, enable ownership, and safety
gate structure described above are frozen as of this baseline. Future
tuning work (see `FINE_TUNING_PLAN_4CH.md`) MUST NOT change topology,
ownership, or channel implementation unless a genuine architecture bug is
discovered (in which case: stop, document the bug, get it reviewed before
changing topology — do not fix it as a side effect of a tuning
experiment). The known limitations above are TOOLING/ACCOUNTING defects
to fix in the validator/planner layer, not architecture changes.
