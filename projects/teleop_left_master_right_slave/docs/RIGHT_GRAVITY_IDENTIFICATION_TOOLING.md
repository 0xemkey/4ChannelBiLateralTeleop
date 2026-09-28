# Right/slave gravity identification tooling (Phases A-D, implemented)

Date: 2026-09-08
**No hardware was touched to produce this document.** Everything below was
verified by compiling/linking the C++ tool and exercising its argument-
validation path (which exits before `hdInitDevice()` is ever called), and
by import-checking the Python scripts. No force was commanded, no ROS node
subscribed to a live topic, no device was opened.

## A. Logging extension — `tools/hdapi_pose_hold_sweep.cpp`

A new sibling tool, **forked from** (not edited in place) the proven
`hdapi_static_sweep.cpp`. Every safety-critical primitive — `servoLoop`'s
force clamps and abort handling, `autoHold()`'s find→average→freeze→verify
sequence, `kMaxForceN`/`kMaxBiasN`/`kMaxTotalN` — is byte-for-byte
unchanged from the original file. The only additions are:

- `qdot`/`qddot` computed in `servoLoop` via backward difference against a
  **measured** wall-clock dt (`clock_gettime(CLOCK_MONOTONIC, ...)`), not
  an assumed 1kHz — mirrors `DobDiagnostics.msg`'s own convention and its
  "quantisation-noisy by construction" caveat.
- `autoHold()` gained four **optional** out-parameters (default `nullptr`,
  zero effect on existing behavior) exposing diagnostics it already
  computed internally (`sag`, `maxWander`, `unsettledFrac`, open-loop
  drift) — no control-logic changes, purely exposing existing locals.
- New session metadata (`--session <id>`, required; `--base-orientation
  <string>`, optional with an explicit `UNRESOLVED` default and a printed
  warning if omitted; `git_commit` auto-detected via `git rev-parse HEAD`).
- A new per-hold logging function (`logPoseHoldRow`) with the schema below,
  replacing the file's original `logRow` (which was specific to the
  axis-probe/breakaway schema this fork doesn't produce).
- The 6-direction axis-probe machinery (`verifySelfHold`, `rampDirection`,
  `DirResult`, and the associated constants) was **removed**, not left
  dead — this tool never ramps a probe force, it only finds and freezes
  the holding force at an operator-chosen approach.

**CSV schema** (`tool=pose_hold_sweep`):
```
tool,session_id,git_commit,device,base_orientation,unix_time,
pose_id,approach_dir,
q1,q2,q3,qdot1,qdot2,qdot3,qddot1,qddot2,qddot3,
qdot_valid,qddot_valid,
tau_hold_nm_1,tau_hold_nm_2,tau_hold_nm_3,
bias_1,bias_2,bias_3,
pos_x,pos_y,pos_z,
sag_m,max_wander_m,unsettled_frac,settled,open_loop_drift_rad,hold_ok,
cartesian_controller_telemetry,dob_disturbance_estimate,
commanded_joint_torque_from_controller,controller_decomposition
```
The last four columns are **always** the literal string `NA` — this tool
structurally cannot produce them (see `RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md`'s
architecture decision). Never `0` — `NA` cannot be mistaken for a real
zero reading the way a fallback would.

**Build** (from `ros2_ws/src/touch_haptics/tools/`):
```bash
g++ -std=gnu++17 -I../include hdapi_pose_hold_sweep.cpp -o hdapi_pose_hold_sweep -lHD -lHDU -lrt -lpthread
```
Verified this session: compiles clean (`-Wall`, zero warnings beyond one
pre-existing harmless `stdcall` attribute warning shared with the original
tool), links against the real `libHD`/`libHDU` present on this machine,
and the built binary correctly refuses to start without `--session`
(exit code 2, printed message, **never calls `hdInitDevice`** — confirmed
by reading the code path, not just observing the exit).

**Run** (operator, hardware-affecting — not done this session):
```bash
cd ros2_ws/src/touch_haptics/tools
./hdapi_pose_hold_sweep "Default Device" --session pilot_20260909 \
  --base-orientation "roll=+0.3deg,pitch=-4.1deg,unc=0.2deg,method=digital_angle_gauge,adjustable=no" \
  --log ~/dob_logs/pose_hold_pilot.csv --status ~/dob_logs/pose_hold_status.txt
```
Per hold, the tool prompts for pose_id, approach direction (`+`/`-`), then
runs the unchanged `autoHold()` sequence (operator holds the arm at the
pose, releases on cue, the PD finds and freezes the holding force, verifies
open-loop). `'q'` at any prompt ends the session; Ctrl-C is safe at any
time (unchanged abort path).

## B. Stillness-calibration tooling — `scripts/calibrate_stillness_noise.py`

Entirely passive — subscribes only to `/touch_right/dob_diagnostics`
(`DobDiagnostics.msg`, "OBSERVATION ONLY... nothing here feeds the command
path"). No force commanded, ever. Run against the existing safe zero-force
DOB-observation startup (`00_preflight.sh` → `01_start_master.sh` →
`02_start_slave.sh`, unchanged).

Verified this session: imports cleanly against the built ROS workspace
(`from touch_haptics.msg import DobDiagnostics` resolves), `--help` runs
correctly. Not run against a live topic (would require the passive startup
sequence, a hardware-affecting step not taken this session).

```bash
python3 scripts/calibrate_stillness_noise.py --duration-s 8 \
  --out-json ~/dob_logs/results/stillness_calibration_right.json
```
Outputs qdot/qddot std per joint, stationary position variance, message-
gap/dropout detection, and `derived_qdot_threshold_rad_s` /
`derived_qddot_threshold_rad_s2` (= k × measured std, k=4.5 default) —
feed directly into `analyze_pose_hold_sweep.py --qdot-threshold-rad-s`.

## C. Pilot-session tooling

### Analysis script — `identification/analyze_pose_hold_sweep.py`

Pairs rows by `pose_id`, computes `tau_g_measured`/`tau_f_static` per the
approved `[tau_plus ± tau_minus]/2` formula, and checks the 5 approved
sanity criteria: friction sign physical (no negative Coulomb estimates),
pose-pair drift tolerance, repeated-hold consistency (if any pose/direction
was repeated), hold quality (unsettled fraction, wander), bias-cap
contamination. Prints a `PASS`/`REVIEW`/`FAIL` verdict — per the approved
plan, a non-`PASS` verdict means **stop before collecting the remaining
poses**. Tested this session against a synthetic 2-row CSV (one pose pair)
— correctly computed the gravity/friction split and flagged a
deliberately-injected bias-cap value.

```bash
python3 identification/analyze_pose_hold_sweep.py ~/dob_logs/pose_hold_pilot.csv \
  --qdot-threshold-rad-s <value from calibrate_stillness_noise.py> \
  --out-json pilot_summary.json
```

### Pilot pose list (8-10 poses, for review)

Per the approved plan's requirements — low/middle/high shoulder, meaningful
waist variation, both approach directions per pose, distributed across the
actual intended workspace (using the real observed ranges from
`RIGHT_GRAVITY_IDENTIFICATION_PROTOCOL.md`: shoulder -6° to 75°, elbow 1°
to 37°, plus deliberately wider waist than the Y-axis-only logs ever
exercised):

| pose_id | waist (deg) | shoulder (deg) | elbow (deg) | purpose |
|---:|---:|---:|---:|---|
| 1 | 0 | 0 (low) | 10 | low-shoulder baseline, centered waist |
| 2 | 0 | 40 (mid) | 20 | mid-shoulder baseline, centered waist |
| 3 | 0 | 70 (high) | 30 | high-shoulder, near real workspace ceiling |
| 4 | -30 | 0 (low) | 10 | low-shoulder, waist deflected negative |
| 5 | +30 | 0 (low) | 10 | low-shoulder, waist deflected positive — pairs with #4 for tilt leverage |
| 6 | -30 | 40 (mid) | 25 | mid-shoulder, waist deflected negative |
| 7 | +30 | 40 (mid) | 25 | mid-shoulder, waist deflected positive — pairs with #6 |
| 8 | 0 | 20 | 5 (low elbow) | elbow-range coverage, low end |
| 9 | 0 | 50 | 35 (high elbow) | elbow-range coverage, high end |
| 10 (optional) | -15 | 60 | 20 | edge-of-range combination, extrapolation-guard poses of the eventual full set |

Each pose visited twice (once `+`, once `-` approach) = 16-20 holds for the
pilot. **This list is for review, not final** — the operator may adjust
individual angles for what's physically comfortable/safe to reach, as long
as the low/mid/high shoulder and waist-variation goals are preserved.

## D. Rig-orientation recording procedure

See `RIGHT_RIG_FRAME_VALIDATION.md`'s new "Physical measurement procedure"
section — the fillable checklist (roll, pitch, uncertainty, method,
adjustability) and its wiring into `hdapi_pose_hold_sweep.cpp`'s
`--base-orientation` flag.

## Verification summary (this session, no hardware)

| Item | Verified how |
|---|---|
| `hdapi_pose_hold_sweep.cpp` compiles | `g++ -c`, zero errors, one pre-existing warning |
| `hdapi_pose_hold_sweep.cpp` links | Full build against real `libHD`/`libHDU`, produced `hdapi_pose_hold_sweep` binary |
| Argument validation works, never touches device | Ran without `--session`: exit 2, printed error, code path confirmed to exit before `hdInitDevice()` |
| `analyze_pose_hold_sweep.py` logic | Ran against a synthetic 2-row CSV, produced correct gravity/friction split and correctly flagged an injected bias-cap violation |
| `calibrate_stillness_noise.py` imports/CLI | `--help` exercised the full import chain including `touch_haptics.msg.DobDiagnostics`; not run against a live topic |

## What is still NOT done

Per the approved plan's stop condition: the physical base-orientation
measurement, running the stillness calibration against real hardware, and
the actual ~8-10 pose pilot session — all require an operator at the rig
and are not started.
