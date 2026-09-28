# On-site staged hardware runbook — 2026-09-10

Read `CHANNEL_READINESS_MATRIX.md` first. **H7, H8, H9 are "prepared but
hardware-unvalidated"** (see their sections below) — the `bilateral_4ch.py`
enable-publisher conflict that used to BLOCK them outright is resolved
(`COMMAND_AUTHORITY_ADR.md`), but no channel involving C1/C4 has touched
real hardware under the corrected code; treat each as a genuine first
attempt, not a formality. **H3/H4's recorder tooling is now built and
simulation-validated** (`hardware_validation/record_manual_motion.py` +
`analyze_manual_motion.py`, TOOLING_READY_SIM_VALIDATED /
HARDWARE_VALIDATION_PENDING) — see their sections below for the exact
command. Every other stage is ready to attempt, pending the go-ahead this
document itself requires at each step.

## Standing rules for every stage

- A session ID is generated fresh per stage (`--session <ID>`, see
  `common.sh`). **Never reuse a hardcoded PID or session ID from a prior
  document or memory** — PIDs change on every relaunch.
- After ANY node/bridge launch, the FIRST action is to print and confirm
  the session-specific stop command:
  `bash 99_stop_setup.sh --session <THIS_STAGE_SESSION_ID>`
- Manual E-stop (independent of session bookkeeping, use if the above is
  uncertain): `kill -TERM -<master_pgid> -<slave_pgid>` (read the actual
  PGIDs from `<session_dir>/pids/*.pgid` for the CURRENT session — do not
  reuse a number from an earlier stage).
- Configuration fingerprint: every stage's manifest/output directory must
  record the frozen-config hash (see `frozen_config.py`'s `config_fingerprint()`,
  Part B item 6) alongside `git rev-parse HEAD` and `git status --porcelain`.
- `REF_POSE_M` is never used as a target. Every stage that needs a starting
  pose LATCHES it live, freshly, for that specific run (see
  `part1_gravity_settle_capture.py`'s `latch_origin()` pattern).
- Physical-supervision confirmation (a person physically beside the robots,
  workspace clear, E-stop reachable) is asked and answered explicitly
  before EVERY stage that enables force — not assumed carried over from a
  previous stage in the same session.

## Dry-run vs armed command

Every stage below has two invocations:

- **Dry-run / check**: prints the exact command that would run, validates
  preflight (topics live, config fingerprint, no competing publishers),
  actuates NOTHING. Safe to run any time, including remotely.
- **Armed**: identical command plus explicit flags:
  - `--session <NEW_SESSION_ID>` (fresh, never reused)
  - `--confirmed-supervision` (only after the blocking physical-supervision
    question above has been answered "yes" in this conversation/session)
  - `--master-deadman-confirmed` (operator has physically checked the
    master's deadman button works before relying on it)
  - requires `00_preflight.sh` (or this task's own preflight check, Part F
    item 18) to have passed for this exact session first

---

## H0 — Physical inspection, clear workspace, E-stop, device identity

**Command:** none yet — this is a physical/manual checklist, then:
```
python3 preflight_ownership_check.py   # read-only, see Part F below
```
**Expected ownership:** no bridge, no bilateral_4ch running yet; only
`teleop_master_left`/`teleop_slave_right` (or none, if starting fresh).
**Config fingerprint:** N/A yet (no bridge launched).
**Who holds the deadman:** n/a.
**Expected physical behavior:** nothing moves; this is inspection only.
**Abort conditions:** N/A.
**Manual E-stop:** N/A (nothing armed).
**Plots/metrics:** none.
**GO/NO-GO:** GO only if — workspace clear, E-stop physically reachable,
both devices visually undamaged, cables secure, a person is physically
present. NO-GO otherwise, unconditionally.
**Rollback:** n/a.

## H1 — Zero-force passive baseline

**Command:** `python3 passive_baseline_capture.py <out>.json` (already
built, read-only, already validated this session — see `PREP_REPORT.md`
Part 1).
**Ownership:** only the two `touch_slave` nodes' own native publishers.
**Fingerprint:** N/A (no bridge).
**Deadman:** n/a (no force possible, nodes not enabled).
**Expected:** all-zero velocity, force, no clamps, matching this session's
already-captured evidence.
**Abort:** N/A (fully passive).
**GO/NO-GO:** GO if output matches the already-captured baseline shape (zero
everything); NO-GO if ANY nonzero force/velocity appears with nothing armed
— stop and investigate before continuing.

## 2026-09-10 (round 2) correction, read before H2 onward

**All active channels for every stage must now be stated EXPLICITLY as
C1/C2/C3/C4 booleans, not implied.** The original H5 ("C2-only") was never
actually C2-only — C3 was architecturally always active, so the slave was
always being pulled by the master too. This is now fixed at the source:
`bridge_master_slave.py` has a genuine `--c3-enable` flag (default True,
to preserve old behavior for existing callers; **every isolated-channel
stage below must pass it explicitly**). Verified empirically,
`ros2_ws/.../sim/tests/test_channel_isolation_live.py`: with
`--enable-c2 true --c3-enable false`, the slave's own tracking error
stayed 0 (peak 0.09mm) — genuine isolation, not just an assumption.

**The master deadman gate now applies to every force-producing stage,
including gravity support (H2 onward)** — see `COMMAND_AUTHORITY_ADR.md`
and `bridge_master_slave.py`'s `_tick()`: the single master `/enable`
publish is gated on `_master_deadman_held()`, and (per
`touch_slave.cpp:842`) that ONE flag gates ALL force on that arm,
including the auxiliary/gravity term. **Use `--require-master-deadman true`
starting at H2**, not just H5 onward as the previous version of this
document said.

**The `bilateral_4ch.py` publisher-enable conflict is now FIXED at the
architecture level** (see `COMMAND_AUTHORITY_ADR.md`) — `bridge_master_
slave.py` now publishes both arms' `/enable` unconditionally, and
`bilateral_4ch.py`'s own enable-publishing defaults OFF
(`--publish-enable` must be explicitly passed to get the old, unsafe
behavior back). **H7–H9 are downgraded from BLOCKED to "prepared but
hardware-unvalidated"** — the architectural blocker is resolved, but NO
channel involving C1/C4 has been tested on real hardware under the
corrected code. Do not read "unblocked" as "ready."

## H2 — Slave gravity-settle-only capture

**Active channels: C1=false, C2=false, C3=false, C4=false.** Gravity
support only — no bilateral target/spring/relay action of any kind.
**Command (armed):**
```
python3 part1_gravity_settle_capture.py --out-dir <session_dir> \
  --confirmed-supervision
```
**2026-09-10 correction: applied.** `part1_gravity_settle_capture.py` now
calls `bridge_cli_args(..., enable_c2=False, c3_enable=False,
require_master_deadman=True)`, matching the channel table above.
(dry-run: omit `--confirmed-supervision`, confirms it refuses and exits 3)
**Ownership:** bridge owns `/touch_right/target_pose`, `/touch_right/enable`,
`/touch_right/force_cmd` (gravity comp only), `/touch_left/enable`
(unconditional, deadman-gated, per the 2026-09-10 consolidation) — NO
`/touch_left/target_pose` once `enable_c2=False` is applied.
**Fingerprint:** printed at script start (`frozen_config.FROZEN` /
`config_fingerprint()`).
**Deadman: REQUIRED.** Gravity support is real force on the slave; per
the correction above, the master's deadman must be held for it to apply
at all (this bridge-level gate is now unconditional across every force
path, gravity included). **Who holds it: the on-site operator, gripping
the master's configured deadman button (mask=1, `HD_DEVICE_BUTTON_1` by
default) continuously for the whole capture.**
**Expected:** `GRAVITY_RAMP_UP`→`GRAVITY_SETTLE` (HOLD MODE — stays here,
never auto-advances, `coupling_ramp_alpha` exactly 0 the whole time,
proven for an arbitrarily long hold by `test_hold_mode_stays_in_gravity_
settle_indefinitely`), slave shows ~0.55N aux force settling in, master
shows its own TRUE uncompensated residual (not "compensated" — see
`CHANNEL_READINESS_MATRIX.md`'s correction).
**Abort:** unexpected spring+damping force >0.05N (excludes the known aux
term), clamp, telemetry stale, FAULT, deadman release/stale (latched,
requires explicit re-arm — `test_master_deadman_gate_live.py` covers
press/release/stale/no-auto-rearm/FAULT) — all self-monitored, disarm
automatically.
**Manual E-stop:** `python3 hardware_validation/safe_stop.py --session <ID>`.
**Plots:** `plot_y_axis_validation.py <session_dir>` → `part1_gravity_settle.png`.
**GO/NO-GO:** GO if no abort fires and metrics resemble this session's sim
predictions in ORDER OF MAGNITUDE; NO-GO on any abort, deadman-gate
misbehavior, or a >2x deviation from the sim-predicted gravity residual.
**Rollback:** script disarms automatically at the end regardless of outcome.

## H3 — C3-only manual relative +5mm Y and return (Procedure A)

**Active channels: C1=false, C2=false, C3=true, C4=false.**
**Status: displacement magnitude unconfirmed by you** — 5mm is an
evidence-based RECOMMENDATION (sim: gain 0.69, RMSE 1.00mm), not a
hardware-validated number; the recorder does not enforce a target
displacement, so whatever value the operator actually moves is what gets
recorded and analyzed.
**Status: TOOLING_READY_SIM_VALIDATED / HARDWARE_VALIDATION_PENDING**
(2026-09-10, H3/H4 recorder tooling task) — the real-hardware recorder now
exists: `hardware_validation/record_manual_motion.py` (SUBSCRIBE-ONLY,
enforced by a static source-grep test that fails the build if a publisher
is ever added) + `hardware_validation/analyze_manual_motion.py` (event
detection, tracking/gain/lag/cross-axis metrics, channel-isolation and
safety validation, plots). Validated end-to-end in simulation only (a
sine-driven "hand" stands in for the operator's manual motion, exactly as
`test_part3_c3_slave_tracking_sim.py` already does) — see
`FINAL_REPORT_2026-09-10_H3_H4_RECORDER.md` for the full test list. NEVER
run on real hardware yet; this status label exists specifically so H3/H4
are not mistaken for hardware-validated just because the tooling gap is
closed.

**Command (armed, bridge process — start this FIRST):**
```
bridge_master_slave.py <canonical args via hardware_validation.frozen_config.bridge_cli_args(
  ..., enable_c2=False, c3_enable=True, require_master_deadman=True)>
```
**Command (recorder — start once the bridge reports `bridge_state: ACTIVE`
on `/bridge_diagnostics`; run in a SEPARATE terminal from the bridge):**
```
python3 hardware_validation/record_manual_motion.py \
  --out-dir <session_dir>/manual_motion/h3_trial1 \
  --duration-s 30 --operator-mode single_operator \
  --expected-stage H3 --expected-c3 true \
  --notes "H3 +5mm Y, operator moved master by hand"
python3 hardware_validation/analyze_manual_motion.py <session_dir>/manual_motion/h3_trial1
```
The recorder refuses to start (exit 4) if it detects the real hardware
driver nodes (`teleop_master_left`/`teleop_slave_right`) already present
under an unexpected/unintended ROS graph — this is a genuine safety check,
not a bug, if it ever fires unexpectedly on-site; investigate before
overriding it.
**Physical-ergonomics question, still unresolved (see
`OFFSITE_REMAINING_WORK.md` item 7):** the same operator cannot both hold
a deadman button and move the master by hand with a normal grip. The
recorder's `--operator-mode {single_operator,assisted_deadman}` flag lets
this be DOCUMENTED (metadata only) once a policy answer exists on-site —
it does not itself change the bridge's deadman requirement.
**Ownership:** bridge owns `/touch_right/target_pose`/`/touch_right/enable`
(always) and `/touch_left/enable` (always, deadman-gated); `/touch_left/
target_pose` NEVER published (`--enable-c2 false`) — confirmed by code +
`test_channel_isolation_live.py`.
**Deadman: REQUIRED**, even though the master's own CONTROLLER is
unpowered here (no target_pose driving it) — the gate still applies
because it is what allows the SLAVE's actuation (C3's force on the slave
is gated by the SAME single bridge-level enable decision, which factors
in the master deadman regardless of which arm the force lands on). **Who
holds it: the on-site operator, with their OTHER hand or a foot switch if
available — they are ALSO the one physically moving the master, so this
needs either a genuinely hands-free deadman mechanism or a second person.
Flag this physical-ergonomics question explicitly on-site before
starting; do not assume it is solved by this document.**
**Expected:** slave Y tracks master Y with lag/gain broadly similar to the
sim result (gain ~0.5-0.9, RMSE order 1-2mm) — real numbers WILL differ,
report them, do not force-fit to the sim prediction.
**Abort:** slave clamp, slave telemetry stale, FAULT, deadman release/
stale, coupling_ramp_alpha not reaching/holding 1.0.
**GO/NO-GO:** GO if slave visibly and safely follows master's Y motion
with no clamp/fault; NO-GO on any clamp, fault, deadman misbehavior, or
the slave failing to move at all (possible sign/frame issue — stop
immediately, do not proceed to H4).
**Rollback:** disarm (`arm_teleop=false`), confirm READY before continuing.

## H4 — repeat C3 on X and Z

**Active channels: same as H3 (C3 only).** Same deadman/ownership/abort
requirements as H3. Same recorder (TOOLING_READY_SIM_VALIDATED /
HARDWARE_VALIDATION_PENDING) — rerun `record_manual_motion.py` with a new
`--out-dir` per axis/direction, then `analyze_manual_motion.py` on each.
**Report each axis's positive AND negative direction SEPARATELY** (a
symmetric peak-to-peak number must not conceal directional asymmetry,
exactly the historical Y-axis finding this whole task originated from).
**GO/NO-GO:** same criteria as H3, per axis, per direction (6 sub-checks:
+X,-X,+Y,-Y,+Z,-Z — Y already covered in H3, this stage does the other 4
plus a Y re-check for consistency).

## H5 — genuinely C2-only

**Active channels: C1=false, C2=true, C3=FALSE, C4=false.** This is the
CORRECTED version of the previous runbook's H5 — `--c3-enable false` is
now mandatory for this stage to actually mean what its name says.
**Command:** bridge with `--enable-c2 true --c3-enable false
--require-master-deadman true`.
**Ownership:** `/touch_left/target_pose` bridge-owned (C2 on);
`/touch_right/target_pose` bridge-owned but held at the slave's own live
pose the whole time (C3 off — verified, `test_channel_isolation_live.py::
test_c2_only_genuine_isolation_slave_does_not_move`, slave tracking error
peak 0.09mm over a 2s ACTIVE window).
**Deadman: REQUIRED.** The OPERATOR holds the master's deadman for the
MASTER's own force output (C2's spring pulling the master toward the
slave) to apply at all.
**GO/NO-GO:** GO if the MASTER visibly and safely follows the SLAVE (moved
by the operator's other hand, or a fixture) with the slave itself staying
put (no C3 leak); NO-GO if the slave moves at all, or any clamp/fault.

## H6 — C2+C3 bilateral position

**Active channels: C1=false, C2=true, C3=true, C4=false.**
**Command:** bridge with `--enable-c2 true --c3-enable true
--require-master-deadman true`, full normal arm sequence (NOT hold mode)
— this is the FULL validated architecture from this session's 83-test
regression, now for the first time on real hardware under the corrected
moving-live-origin planner AND the deadman gate.
**Ownership:** both target_pose topics + both enable topics bridge-owned.
**Deadman: REQUIRED**, gates both arms' actuation via the single enable
decision.
**Expected:** matches this session's simulation validation shape (no
clamps, alpha reaches 1.0, gap-matrix-comparable tracking) — first
REAL-HARDWARE confirmation of the corrected architecture.
**GO/NO-GO:** GO only if the FULL Part 5 metric set (Y tracking RMSE, peak
error, lag, peak forces, margin minimums) is within the same order of
magnitude as the sim gap-matrix corpus; NO-GO on any clamp, fault, or an
order-of-magnitude mismatch (investigate before any further stage).

## H7 — corrected-sign C1-only — **prepared but hardware-unvalidated**

**Active channels: C1=true, C2=false, C3=false, C4=false.**
**No longer architecturally blocked** — `bridge_master_slave.py`'s
`--c1-enable true` subscribes to `/bilateral_status` and composes
`bilateral_4ch.py`'s own safety_state into the bridge's single enable
decision (fail-closed on stale/missing status); `bilateral_4ch.py` itself
must be launched with `--publish-enable` OMITTED (the new safe default)
so it never independently touches `/enable`.
**Still true:** the corrected C1 sign (`-w1`, fixing a real un-negated
bug) has **never been hardware-tested** — only simulated
(`test_c1_sign_vs_ground_truth.py`, passing). **Do not treat "prepared"
as "safe to assume will feel right" — this is a genuine first attempt at
the corrected sign on real hardware.**
**Command (both processes, this order):**
```
# bridge, C1 declared, C2/C3/C4 off:
bridge_master_slave.py ... --c1-enable true --c3-enable false \
  --require-master-deadman true
# bilateral_4ch.py, C1 only, --publish-enable OMITTED:
bilateral_4ch.py --architecture custom --w1 0.3 --w2 0 --w3 0 --w4 0
```
(low, conservative w1 for a first attempt — do not use `4ch_safe`'s 0.5
or `4ch`'s 1.0 without a specific reason; these are LEGACY_COMMISSIONING
values, not scientific optima, per `bilateral_4ch.py`'s own comment)
**Ownership:** `bridge_diag`'s `c1_enabled=true`; verify via
`hardware_validation/preflight.py` that `/touch_left/enable` has exactly
ONE publisher (`bridge_master_slave`) even with `bilateral_4ch.py`
running.
**Deadman: REQUIRED**, same single gate, now also composed with
`bilateral_4ch.py`'s own tank/oscillation-trip state.
**Abort:** any bridge FAULT, deadman release/stale, `bilateral_status.
safety_state==TRIPPED` (composed into enable automatically), any clamp.
**GO/NO-GO:** GO only after confirming, in this order: (1) free-space
zero force on the master with no slave contact, (2) a SLOW, gentle slave
push produces an OPPOSING (not amplifying) force on the master, (3) no
oscillation/trip during a short session. NO-GO on any sign ambiguity —
stop immediately, this is exactly the kind of finding that must not be
rationalized away.

## H8 — C4-only contact/reflection — **prepared but hardware-unvalidated**

**Active channels: C1=false, C2=false, C3=false, C4=true.** Same
unblocking as H7 (`--c4-enable true` on the bridge, `bilateral_4ch.py`
`--w4` only, `--publish-enable` omitted). C4's sign was not flagged as
wrong in the 2026-09-07/09-10 work, but has not been re-validated since
2026-09-03 against ANY of this session's changes (moving-live-origin,
deadman, the enable consolidation). Same GO/NO-GO shape as H7, mirrored
(master push → opposing slave contact-avoidance behavior, not the C1
direction).

## H9 — complete four-channel low-force mode — **prepared but hardware-unvalidated**

**Active channels: C1=true, C2=true, C3=true, C4=true**, all at
conservative weights. Depends on H5–H8 each independently passing first.
Do not attempt as a first four-channel test — this is a composition of
four already-individually-verified channels, not a new thing to debug
from scratch.

## H10 — performance characterization/tuning

Only after H0–H9 all pass. Out of scope for this document — this is the
first stage where deliberate parameter tuning is even considered, and
even then, only against a documented transparency objective, not ad hoc.

---

## Summary GO/NO-GO for "is the project ready for H0/H1 right now"

**YES for H0/H1** (physical inspection + passive baseline) — both are
read-only/zero-force and fully prepared.

**H2 is code-ready** (`part1_gravity_settle_capture.py` now launches with
`enable_c2=False, c3_enable=False, require_master_deadman=True`) but, like
H3–H9, has zero real-hardware confirmation under the current code.

**H3/H4 need their real-hardware recorder script built** (not yet done)
and a physical-ergonomics answer for who holds the deadman while the same
operator moves the master by hand.

**H5–H6 are code/test-ready** (frozen config, deadman gate, genuine C3
gating, verified in sim) but have **zero real-hardware confirmation under
the current code**.

**H7–H9 are "prepared but hardware-unvalidated"** — the architectural
enable-publisher blocker is RESOLVED (`COMMAND_AUTHORITY_ADR.md`), but no
channel involving C1/C4 has touched real hardware under any of this
session's changes. Treat every one of H7–H9 as a genuine first attempt.
