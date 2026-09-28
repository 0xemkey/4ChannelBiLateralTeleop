# Y-axis hardware validation — preparation report (NO ACTUATION TAKEN)

Status: **BLOCKED on physical-supervision confirmation.** Nothing beyond the
passive read-only baseline below has been commanded. No bridge armed, no
`enable`, no gravity compensation, no motion. No live hardware process was
restarted or modified.

## 1. Passive baseline (completed, safe, zero-force)

`passive_baseline_capture.py` (this directory) subscribed read-only to the
already-running `/touch_left` and `/touch_right`
`cartesian_controller_telemetry` topics for 10.0s (2001 samples/arm @ ~200Hz).
No command was published. Full raw data + summary: `passive_baseline.json`.

| | master (left) | slave (right) |
|---|---|---|
| mean vy | 0.0 | 0.0 |
| RMS vy | 0.0 | 0.0 |
| peak &#124;vy&#124; | 0.0 | 0.0 |
| y drift over window | 0.0 m | 0.0 m |
| y position (resting) | -0.05813 m | -0.05990 m |
| mean auxiliary force (y) | 0.0 N | 0.0 N |
| mean applied-force residual | 0.0 N | 0.0 N |
| any clamp | false | false |

Both arms are genuinely at rest, unforced, no bridge armed (no gravity
compensation active). No anomaly in this state — expected, since nothing is
being commanded yet.

## 2. `99_stop_setup.sh` verification (dry-run only, not executed for real)

```
$ cd projects/teleop_left_master_right_slave/experiment_launch
$ bash 99_stop_setup.sh --dry-run
[stop] targeted session=.../experiment_runs/teleop_setup/<timestamp>_dob_observe
[stop] would TERM only PID groups recorded by this session
```

`--dry-run` works as documented (prints intent, takes no action, exits 0).

**Cross-checked against the actual live session** (`.current_session` points
at `20260909T100330Z_dob_observe`): that session's recorded PIDs —

```
pids/master.pid = 1529135   pids/master.pgid = 1529135
pids/slave.pid  = 1525443   pids/slave.pgid  = 1525443
```

— exactly match the live `teleop_master_left` / `teleop_slave_right`
process-group leaders (`ps aux` confirms both are running, launched via
`setsid` so PID==PGID, each with a `touch_slave` C++ child in the same
group). **Conclusion (by static inspection, not executed): running**
`bash 99_stop_setup.sh --session 20260909T100330Z_dob_observe`
**(no `--dry-run`) would correctly `SIGTERM` both live hardware processes.**
The script also notes the hardware's own watchdog independently zeros stale
commands after 0.05s regardless.

## 3. Emergency-stop commands for a second terminal

Primary (uses the project's own session-aware stop path):
```
cd /home/nural/HIL-RL/projects/teleop_left_master_right_slave/experiment_launch
bash 99_stop_setup.sh --session 20260909T100330Z_dob_observe
```

Raw fallback (independent of session bookkeeping — sends SIGTERM directly to
the known live process groups):
```
kill -TERM -1525443 -1529135
```

Software soft-stop (zero-force without killing the process, if the process
groups above have since changed — publish `enable:=false` to both arms):
```
ros2 topic pub -1 /touch_left/enable std_msgs/msg/Bool "{data: false}"
ros2 topic pub -1 /touch_right/enable std_msgs/msg/Bool "{data: false}"
```
This is the SOFTER option — prefer the `kill`/stop-script form as the actual
emergency stop, since it does not depend on the enable-gating logic being
correct.

## 4. `require_deadman` investigation (code read only — NOT relaunched)

`touch_slave.cpp:307-308,830`: `require_deadman` (default `true` in the
source, but the two LIVE processes were explicitly launched with
`require_deadman:=false`) gates force output as:

```
deadman_ok = !require_deadman_ || (buttons & deadman_button_mask_) != 0
force applied only if: command.enabled && deadman_ok
```

`deadman_button_mask_` defaults to `HD_DEVICE_BUTTON_1` (the stylus's
primary button). **This is a fully implemented, ready-to-use safety gate —
no code change needed**, only a launch-parameter change
(`-p require_deadman:=true`) the next time these nodes are (re)started. Not
applied to the live processes per your explicit "do not restart without
approval."

## 5. Staged test procedure (prepared, not run)

Exactly as you specified, each step gated on an explicit go-ahead:

1. **GRAVITY_SETTLE only.** Arm the bridge (`arm_teleop=true`), capture
   telemetry through `GRAVITY_RAMP_UP`→`GRAVITY_SETTLE`. Because the current
   state machine has no "hold at GRAVITY_SETTLE forever" mode (it
   automatically proceeds into `COUPLING_RAMP_UP` once the quiet-hold gate
   passes — see `startup_alignment.py`), the prepared script watches
   `bridge_state` and issues `arm_teleop=false` (disarm) the INSTANT it sees
   `COUPLING_RAMP_UP`, minimizing coupling exposure to at most one bridge
   tick (~10ms) at essentially `coupling_ramp_alpha≈0`. This satisfies "with
   bilateral coupling OFF" as closely as the real state machine allows
   without a code change (no code change made — matches the "do not modify
   controller gains/logic unless required to fix an obvious bug" constraint).
2. **Stop and inspect telemetry.** Disarm fully to READY, print/save the
   Part 1 metrics (mean/RMS/peak vy, drift, gravity/DOB residual for both
   arms), pause for explicit review before continuing.
3. **Controlled small +Y master motion** (Part 2) — bridge stays unarmed
   (coupling OFF), smooth bounded ramp of the master's OWN target only,
   toward the existing validated reference `REF_POSE_M = (0.00738771,
   0.00043958, 0.00795783)` (from `live_reference_watch.py`) if the master
   needs to move to it, else a small bounded +Y excursion and return. X/Z
   held at current values. This step still needs a small design decision
   (exactly how a "master-only" position command is issued without the
   bilateral bridge, since the bridge is what normally drives target_pose)
   — will be resolved and re-confirmed with you before this step runs.
4. **Stop again.**
5. **Plots** — master/slave Y position, velocity, commanded/applied force,
   gravity/auxiliary, spring, damping, saturation flags, state transitions,
   tracking error — generated from the recorded CSV/JSON after each stage,
   not live.

## 6. Automatic abort conditions (from the ALREADY-VALIDATED config — nothing invented, nothing loosened)

The prepared script will self-monitor every tick and immediately disarm /
zero-force / stop if ANY of the following (all pre-existing, validated
limits/flags, not new thresholds) is observed:

| Condition | Source | Action |
|---|---|---|
| `bridge_state == "FAULT"` | any cause (`GRAVITY_SETTLE_TIMEOUT`, `COUPLING_RAMP_TELEMETRY_STALE`, `COUPLING_RAMP_PARAMETER_MISMATCH`, `COUPLING_RAMP_STALLED_FORCE_BUDGET`, or an alignment stale/error fault) | already immediate force-safe (gravity_ramp_alpha, coupling_ramp_alpha → 0 same tick) — script also disarms and halts the procedure |
| `position_error_clamped` true (either arm) | controller's own per-axis clamp — master 0.03m, slave 0.04m | disarm + abort |
| `primary_force_clamped` or `final_force_clamped` true (either arm) | master Fmax=2.0N, slave Fmax=1.5N | disarm + abort |
| `master_accepted_force_budget_margin_n < 0` or `slave_accepted_force_budget_margin_n < 0` | force-budget planner's own accepted-target safety check (Part 1 fix this session) | disarm + abort |
| `master_accepted_position_error_margin_m < 0` or slave equivalent | same planner check | disarm + abort |
| `!force_budget_telemetry_fresh_left` or `!force_budget_telemetry_fresh_right` for > `force_budget_telemetry_stale_s` (0.2s) | telemetry-stale fail-safe | already leads to `COUPLING_RAMP_TELEMETRY_STALE` FAULT — script also treats sustained staleness as its own abort trigger before the bridge's own 3.0s stall timeout, so the OPERATOR sees it faster |
| `!planner_parameter_match_left` or `!planner_parameter_match_right` | parameter-consistency check | disarm + abort |
| `abs(applied_force) > 0.5N` sustained for > 0.5s during any "should be quiescent" phase (idle, post-disarm) | conservative script-level trigger, well under both arms' real Fmax | disarm + abort, flagged as anomalous even though under the hard clamp |
| Master or slave `abs(position) - abs(reference/anchor)` exceeds `left_max_offset_m` (0.08m, C2) or the C3 `max_offset` equivalent | existing travel-limit clamp | disarm + abort |
| Manual: operator sends SIGINT / runs the E-stop command above | — | immediate |

None of these thresholds are new or loosened from the validated
configuration; the script only adds faster/earlier detection on top of the
bridge's own existing fault paths.

## 2026-09-10 UPDATE — corrections after further review

**REF_POSE_M is no longer used as a motion target anywhere.** Traced its
exact provenance: `experiment_runs/teleop_setup/20260909T100330Z_dob_observe/
h2abs_freespace_hold_trial2/` (trial_start=2026-09-09T14:28:53+03:00,
`trial_meta.txt`: "master held still **by hand**"). Final telemetry samples
in that trial — `position_m=(0.0073919, 0.0005772, 0.0077866)` — match
`REF_POSE_M` to sub-mm precision, confirming it. It was an **unplanned,
hand-held pose during an unrelated DOB/free-space trial**, never an
engineered or safety-reviewed Cartesian target. `live_reference_watch.py`'s
docstring has been updated in place to document this and mark it STALE /
not-a-motion-target (code-only change, no live process touched).

**Axis-mapping sign, confirmed via code/history only (no actuation):**
`/touch_left/pose` and `cartesian_controller_telemetry.position_m` are
published from the identical `s.pos_m` value in the same tick
(`touch_slave.cpp:1505-1541`) — no unit/frame/sign difference between them,
so the earlier apparent discrepancy was purely "master isn't currently near
that stale pose," not a topic mismatch. Separately, **physical upward motion
= ROS +Y**, confirmed from two independent real-hardware documents from
this project's own history: `FOUR_CHANNEL_TODO.md:122` ("the operator
specifically checking +Y (gravity-opposed) vs -Y (gravity-assisted)
motion") and `Y_AXIS_DIAGNOSIS.md:65` (same convention, independently
stated). `axis_map=x,y,z` (identity) is the validated master/slave mapping
(project memory), so this sign convention holds for both arms consistently.

**No validated relative-displacement magnitude exists for this exact
scenario** — reported per your explicit "if none exists, stop" instruction,
not invented. The simulation gap matrix (20/50/80/117/140mm) validated the
**bilateral force-budget planner's** control path; the "~1-2mm" figure from
`test_ramp_disarm_root_cause.py` is a per-tick smoothness property inside a
1.0s ramp, not a total displacement. Part 2's redesigned script
(`part2_master_only_y_motion.py`) drives the master directly against its own
native PD controller with no bridge/planner involved — a materially
different control path than either of those validated numbers. `DY_M` is
left `None` in that script and the script refuses to run without an
explicit `--dy-m` — **awaiting your decision on the magnitude** (the master's
hard clamp, `max_position_error_m=0.03m`, bounds worst-case force regardless
of the value chosen, but that's a ceiling, not evidence of a "validated safe"
number).

**`require_deadman:=true` — not applied, PIDs will change on restart.**
Confirmed via `ps -o pid,pgid` that the current E-stop PIDs (`1529135`
master, `1525443` slave, both `require_deadman:=false`) are still live and
correct **for the currently-running processes only**. If/when these nodes
are restarted with `require_deadman:=true` (a launch-parameter change only,
no code change — see `touch_slave.cpp:307-308,830`, gates on
`enabled && (HD_DEVICE_BUTTON_1 held)` by default), **the PGIDs above become
stale immediately** — `00_preflight.sh`/equivalent must be re-run and the
new session's `pids/*.pid` files re-verified before relying on any E-stop
command again. No restart has been performed.

**New prepared artifacts in this directory (none executed):**
- `part1_gravity_settle_capture.py` — arms the bridge with the frozen,
  validated config (K/D/Fmax/position-error-limit/reserve unchanged;
  gravity values `--slave-gravity-comp-n 0.55` / `--left-gravity-comp-n
  0.45`, sourced from `03_start_position_bridge.sh` /
  `03b_start_bridge_c2_diagnostic.sh` respectively, not invented), watches
  `bridge_state`, disarms the instant `COUPLING_RAMP_UP` is first seen
  (minimizing coupling exposure to ≤1 tick at α≈0), self-monitors the abort
  table above every tick, and refuses to run without an explicit
  `--confirmed-supervision` flag (verified: exits 3, no subprocess spawned,
  without it).
- `part2_master_only_y_motion.py` — the redesigned, live-latched (not
  `REF_POSE_M`-based) master-only +Y ramp-hold-ramp, same abort table,
  refuses to run without an explicit `--dy-m` (verified: exits 2 without
  it).
- `plot_y_axis_validation.py` — generates all Part 6 plots from whichever of
  Part 1 / Part 2 / Part 4 (standard recorder schema) CSVs exist; verified
  to run cleanly right now with zero data (prints "not found -- skipping"
  for each, no crash).

## 2026-09-10 UPDATE (round 2) — safety/design revision, per your 6-item request

Still zero actuation. All items are code/config/test-only changes.

**Item 1 — GRAVITY_SETTLE hold mode (source-level, not a race).**
`startup_alignment.py` gained a real `gravity_settle_hold_mode` flag (default
`False`, existing production behavior fully unchanged). When set, once the
existing quiet-hold gate passes, the state machine remains in
`GRAVITY_SETTLE` **indefinitely** — `gravity_ramp_alpha=1`,
`coupling_ramp_alpha=0`, no re-triggered timeout, no auto-advance into
`COUPLING_RAMP_UP` — exiting only via explicit `disarm()`. Wired to
`bridge_master_slave.py` as `--gravity-settle-hold-mode`. 3 new regression
tests added to `scripts/tests/test_gravity_settle_gate.py` (now 10/10
passing): held for 20× the ordinary timeout with `coupling_ramp_alpha`/
`gravity_ramp_alpha` exact throughout; `blend_target()`/`blend_left_target()`
proven to equal the live pose exactly every tick during the hold (zero
bilateral spring pull); exits only via `disarm()`.
`part1_gravity_settle_capture.py` rewritten to use this instead of racing
`COUPLING_RAMP_UP`.

**Item 2 — Part 1 abort logic fixed.** The old flat 0.5N applied-force
threshold would have falsely aborted on the slave's own expected 0.55N
gravity support. Rewritten to check **spring_force_n + damping_force_n only**
(excluding the known auxiliary/gravity term) against a 0.05N tolerance —
during a true gravity-only hold, target==live pose exactly (hold mode's own
structural guarantee), so spring+damping should read ~0; any departure is a
real anomaly. Also checks `coupling_ramp_alpha != 0` explicitly (defense in
depth), clamps, telemetry staleness, and bridge FAULT. **Correction to my own
earlier report**: I previously called `LEFT_GRAVITY_COMP_N=0.45N`
"validated" — `run_bridge.sh`'s own comment says otherwise ("has NOT been
through a controlled sweep yet, so there is no validated value to default
to"). Canonical value is now `0.0` (see `frozen_config.py`), meaning Part 1
measures the master's TRUE uncompensated gravity residual.

**Item 3 — Part 2 abort ordering fixed.** `abort_sequence()` (pure,
independently testable) now: disables (`enable=False`) 5× over ~0.1s
FIRST, then records the pose at abort — and **never publishes a target**.
The old code re-published the stale latched origin as a new target before
disabling, which could pull the device backward through a reverse spring
force if it had already moved close to the commanded displacement. 4 new
pure tests in `test_part2_abort_ordering.py` (all passing) prove: disable
precedes recording, disable repeats ≥3×, no target action ever appears, and
every enable action is `False`.

**Item 4 — Part 2 vs. Part 3 scope clarified; topic ownership proven.**
Part 2 validates ONLY that the master physically moves in commanded +Y —
it says nothing about slave tracking. Confirmed via code inspection (no
live process touched): `bridge_master_slave.py` creates `left_target_pub`
(the `/touch_left/target_pose` publisher) **only if `--enable-c2 true`**
(`bridge_master_slave.py:425-426`), and `_update_c2()` is never called
otherwise. So launching the bridge with `--enable-c2 false` guarantees the
bridge never touches `/touch_left/target_pose` — Part 3 can run Part 2's
own master-driving script unmodified, alongside the bridge (C2 off, C3
always on), with zero risk of two publishers fighting. `/touch_right/
target_pose` stays exclusively owned by the bridge's C3. Force reflection
(`--reflect-enable`) defaults to `False` already; C1/C4 (`bilateral_4ch.py`)
are a wholly separate process never referenced by the position-bridge
launch chain, so they're off by construction, not by an extra flag. Part 3
itself is DESIGNED (this section) but not yet built as an executable
script — proposing the exact command is the next step, not done here.

**Item 5 — displacement candidates validated in sim, evidence reported.**
New `ros2_ws/src/touch_haptics/sim/tests/test_part2_displacement_candidates_sim.py`
reuses Part 2's own ramp profile (`_smoothstep`, `RAMP_DURATION_S`,
`HOLD_DURATION_S`) against a real `touch_slave_sim` master instance (same
compiled controller code as real hardware) with the frozen K/D/Fmax/
position-error config, swept at 2/5/10mm:

| DY | peak applied | peak err | final err | peak vy | achieved excursion | vs. quiet-state drift (0.070mm) |
|---|---|---|---|---|---|---|
| 2mm | 0.023N | 1.53mm | 0.019mm | 0.55mm/s | 0.56mm | 8.0× |
| 5mm | 0.057N | 3.78mm | 0.008mm | 1.13mm/s | 1.37mm | 19.5× |
| 10mm | 0.113N | 7.53mm | 0.001mm | 2.30mm/s | 2.69mm | 38.4× |

No clamps at any candidate; peak force stays two orders of magnitude under
Fmax=2.0N. Note: the master device defaults to hd_sim's "kFree"/unheld
physics (nothing resembling a hand) — every other master-motion sim test
this session used a hand preset, and this one now uses `TOUCH_SIM_HAND=
loose` ("~fingertips" per `environment.hpp`'s own comment) as a stand-in
for an operator's grip; this is a passive spring-damper approximation of a
human hand, not an actively-compliant one, and is flagged as a modeling
choice, not a perfect match to real hardware. **Recommendation from this
evidence (NOT a hardware GO): 5mm** — smallest candidate clearing a ≥10×
margin over the only real, already-measured position-noise reference this
project has (0.070mm quiet-state drift); 2mm falls short of that margin at
8.0×; 10mm is also safe but not obviously necessary over 5mm. This is a
recommendation for you to accept, adjust, or reject — not a decision made
on your behalf.

**Item 6 — one canonical frozen parameter table.** New `frozen_config.py`
is now the single source every script in this directory reads from
(`part1_gravity_settle_capture.py`, `part2_master_only_y_motion.py`).
**Two real, previously-unreconciled discrepancies found and resolved by
using the actually-persisted script defaults (not narrative doc mentions):**
`left_max_offset_m` had THREE different values across this project's own
history (0.08 in `run_bridge.sh`'s real default; 0.05 in
`03b_start_bridge_c2_diagnostic.sh`; 0.15 only narrated in
`FOUR_CHANNEL_TODO.md`, never persisted) — canonical is now 0.08.
`left_gravity_comp_n` had the same issue (0.0 real default vs. 0.45
narrated-only) — canonical is now 0.0, per item 2's correction above. Also
added the previously-omitted `max_offset_xyz`, `max_offset_mode`,
`max_speed_mps`, `deadband_m`, `axis_scale` (all now explicit, matching
`run_bridge.sh`'s real defaults, not silently relying on
`bridge_master_slave.py`'s own separate CLI defaults which happened to
agree for some but not all of these).

**Full regression after this round's changes:** see the live run in this
turn's output — pure suite (`scripts/`, `scripts/tests/`, `experiment_launch/`,
this directory's `test_part2_abort_ordering.py`) all green; live-sim suite
including the new displacement-candidate sim test in progress/reported
separately.

## 2026-09-10 UPDATE (round 3) — corrected metrics overturn the 5mm recommendation

**Do NOT proceed to hardware. The 5mm recommendation from round 2 is withdrawn.**

**Items 1+2 — metrics fixed, candidates re-run, honest result is NONE.**
The old `settling_time_s=0.0` bug (checking error from t=0, where it is
trivially ~0) is fixed — settling is now measured only after RAMP_UP
completes, against the commanded target, requiring a continuous 0.2s hold
within tolerance. Full corrected metrics (achieved/commanded ratio,
steady-state error, peak/RMS tracking error, rise time, settling time,
return-to-origin error) added and re-run at 2/5/10mm:

| DY | achieved | ratio | steady-state err | peak/RMS err | settling time | return-to-origin |
|---|---|---|---|---|---|---|
| 2mm | 0.52mm | 26.0% | 1.48mm | 1.57/1.10mm | never (None) | 0.10mm |
| 5mm | 1.34mm | 26.8% | 3.66mm | 3.78/2.69mm | never (None) | 0.20mm |
| 10mm | 2.67mm | 26.7% | 7.33mm | 7.49/5.34mm | never (None) | 0.45mm |

The ratio is ~27% at every magnitude (a linear system property: K_master/
(K_master+k_hand) = 15/(15+40) ≈ 27%, invariant to displacement size under
the "loose" hand model) — confirming this is a structural property of the
self-actuated control path, not something a different magnitude would fix.
**None of the three candidates meet the required 80% bar. Recommendation:
NONE**, exactly as you diagnosed. Settling time is `None` at every
magnitude too — the persistent ~73% steady-state offset never enters the
commanded target's tolerance band, so "settling" (relative to the
commanded value) never happens, which is the metric now correctly refusing
to report a number that would misrepresent this.

**Item 3 — the two procedures, explicitly separated:**
- **Procedure A (supervised manual motion) — the preferred, valid
  teleoperation test.** An on-site operator physically holds and moves the
  master; the master's own controller is NOT commanding a target (no
  `target_pose` published to it at all); C3 (always on) drives the slave to
  follow. This is what the new Part 3 simulation (below) validates, and
  what real hardware Part 3 should use.
- **Procedure B (self-actuated motion) — NOT validated by anything in this
  task.** The master's own controller drives itself toward a commanded
  `target_pose`, unheld. The corrected Part 2 sim results above show this
  achieves only ~27% of any commanded displacement under a passive
  "loose"-hand-equivalent load and would require separate controller/gain/
  gravity-compensation work (out of scope here, and gains must not be
  changed per your standing constraint) before it could be considered
  validated. Do not treat Part 2 as validating Procedure B for real
  hardware.

**Item 4 — full Part 3 simulation built and run.**
New `ros2_ws/src/touch_haptics/sim/tests/test_part3_c3_slave_tracking_sim.py`:
bridge launched with `--enable-c2 false` (C2 off), C3 always on, force
reflection off by default (never passed `--reflect-enable`), C1/C4 off by
construction (separate process, never launched). A SIMULATED OPERATOR
moves the master via `TOUCH_SIM_HAND=stiff` + a continuous low-frequency
sine trajectory (±5mm peak, smooth, non-step) rather than the master's own
`target_pose` — modeling choice documented in the file: hd_sim's hand-motion
clock is local to the master process and not synchronized to when the
bridge reaches `ACTIVE`, so a one-shot step could land before or after the
observation window depending on arm-sequence timing; a continuous sine
sidesteps that entirely. Confirmed working (PASSED):

```
master Y peak-to-peak = 4.53mm (commanded amplitude 10mm p-p -- even the
                         "stiff" hand preset doesn't perfectly track its
                         own scripted trajectory against the arm's inertia)
slave  Y peak-to-peak = 3.13mm
gain (slave/master)   = 0.69
master-vs-slave RMSE  = 1.00mm
peak abs error         = 1.85mm
estimated lag          = 326ms (coarse, cross-correlation based)
peak master/slave applied = 0.0000N / 0.5045N  (master's OWN force is
                         exactly zero -- its controller was never enabled,
                         confirming Procedure A's "master is a passive,
                         freely-moved input device" property empirically,
                         not just by design)
any clamp = False; coupling_ramp_alpha = 1.0 throughout ACTIVE
master target_pose ever published = False (C2 off, confirmed empirically)
```
Both devices are then disarmed via the bridge's own staged sequence
(`arm_teleop=false`), confirmed reaching `READY`.

**Item 5 — publisher ownership proven for all 5 command topics** (code
inspection of `bridge_master_slave.py`, cross-checked against every script
in this directory):

| topic | bridge owns it when... | any of our scripts also touch it? |
|---|---|---|
| `/touch_left/target_pose` | `--enable-c2 true` only (`left_target_pub`) | Part 2's script (bridge never runs in Part 2); Part 1 also arms with `enable_c2=true` but coupling stays 0 the whole hold, so no real pull either way; Part 3 explicitly OFF, confirmed empirically |
| `/touch_left/enable` | `--enable-c2 true` only (`left_enable_pub`) | same as above |
| `/touch_right/target_pose` | ALWAYS (unconditional `target_pub`, C3 always on) | never touched by any script here |
| `/touch_right/enable` | ALWAYS (unconditional `enable_pub`) | never touched by any script here |
| `/touch_right/force_cmd` | only if `slave_gravity_comp_n != 0.0` (`slave_force_pub`) | never touched by any script here |

Exactly one intended publisher per topic in every phase used by this task.
`master_force_pub`/`master_enable_pub` (force-reflection-only) are never
created anywhere in this task (`--reflect-enable` never passed).

**Item 6 — Part 1 description corrected.** With the canonical
`left_gravity_comp_n=0.0` (item 2 of round 2's own correction), Part 1 is
**not** a two-arm gravity-compensation validation. Relabeled everywhere in
this directory as: **slave gravity-SUPPORT settle (0.55N, controlled-sweep
validated) plus master observation under its own, uncompensated, true
gravity residual.** Do not describe Part 1 as validating master gravity
compensation — none is applied.

**Regression after this round:** 55 pure (`scripts/`) + 4 pure
(`test_part2_abort_ordering.py`) + 14 live-sim (13 from round 2 plus the
new `test_part3_c3_slave_tracking_sim.py`) = **73/73 passing, 0 failures.**

## Outstanding before any real force is applied

Per your instruction, before Part 1's gravity-only step (or anything later)
is run for real, I will ask one blocking question and wait for it to be
answered explicitly in the conversation:

> Is a person physically present beside the robots right now, is the
> workspace clear, and is the physical E-stop immediately reachable?

I will not infer "yes" from context (reference pose being near, watch
scripts running, an SSH stop script existing, etc.) — only an explicit
confirmation in this conversation counts.
