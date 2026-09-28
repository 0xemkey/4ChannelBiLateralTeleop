# Right/slave gravity + friction identification protocol (Phase 2 design)

Date: 2026-09-08, updated 2026-09-08 with implemented tooling
**Status: TOOLING IMPLEMENTED (A/B/C/D below), approved 2026-09-08. Still
no new identification DATA has been collected and no hardware has been
touched — the C++ tool was compiled and its argument-validation path
exercised only (see `RIGHT_GRAVITY_IDENTIFICATION_TOOLING.md`). Per the
approved plan's stop condition: pilot data collection (~8-10 poses) does
not start until this tooling is reviewed.**

## Architecture decision (resolves this doc's original open question)

The original version of this document proposed extending the ROS-based
`log_phase2_scenario.py` to capture `CartesianControllerTelemetry`/DOB
telemetry during identification. Investigating further surfaced a
structural conflict: doing that would require `touch_slave.cpp` to own the
device, but the existing, already-*validated* gravity/friction
identification methodology (`hdapi_static_sweep.cpp`) is a **standalone**
HDAPI tool that owns the device directly — the two cannot run at once
(single-owner OpenHaptics constraint, true throughout this codebase).

**Decision (operator-approved): extend the proven standalone-tool approach,
not build a new ROS-based one.** `CartesianControllerTelemetry`/DOB
telemetry are out of scope for pose-hold identification data as a result —
this is by design, not an oversight (it also means the static measurement
isn't contaminated by the production controller's own spring/damping
dynamics). See `RIGHT_GRAVITY_IDENTIFICATION_TOOLING.md` for exactly what
was built.

## Phase 0 — baseline framing (explicit, per task instruction)

**0.70N is recorded as: "Empirically selected constant Cartesian Y
feedforward baseline."** It is not identified gravity, not a measured
physical load, and is not being extended further (no blind sweep to 0.75/
0.80/0.90N). It remains the production baseline until a validated model
replaces it.

## Why the old 6-14 pose approach isn't being repeated

Both prior fits (6 poses, then 14) used a narrow, hand-picked pose cluster
and got progressively worse — not better — as coverage widened, because too
few poses left the tilt/gravity/collinearity problem underdetermined. This
protocol targets a **wider, denser** pose set (20-40, vs. 6-14 before) and
— critically — **spans the workspace actually used during teleoperation**,
not just a static-sweep-convenient cluster.

## Real observed workspace (data, not guessed)

Pulled from all 8 real-hardware Y-axis captures collected so far (Stage 1
A/B + the 6-point gravity sweep, 43,995 samples total, right/slave device):

| Joint | Observed range (rad) | Observed range (deg) | Old model's fitted range |
|---|---:|---:|---|
| waist (q1) | [-0.264, -0.065] | [-15.1°, -3.7°] | not in basis (by design) |
| shoulder (q2) | [-0.104, 1.315] | [-6.0°, 75.3°] | [-15.3°, 25.8°] |
| elbow (q3) | [0.020, 0.643] | [1.1°, 36.8°] | not separately stated |

**The old model's fitted shoulder range (25.8° max) covers only about a
third of the shoulder range actually used in real Y-axis teleoperation
(up to 75.3°)** — this is the concrete, data-grounded version of the
"14-29% extrapolation" finding from the validation report, and the main
reason the new pose set below is deliberately much wider.

**Important limitation of this data for waist coverage**: because the
Y-axis tests deliberately minimized X motion, waist angle barely varied
(-15.1° to -3.7°, a 11.4° window) — this tells us essentially nothing about
waist behavior across its real range. If `BASIS_TILT` (which needs genuine
q1 variation to be identifiable at all) is going to be fit properly, the
new pose set MUST include deliberate waist-angle variation that the
Y-axis-only logs never exercised.

## Proposed pose coverage (for review)

**~30 poses** (within the 20-40 target), as a grid over the union of (a)
the real observed Y-axis workspace above with margin, and (b) enough
waist spread to make `BASIS_TILT` identifiable if needed:

- **Shoulder (q2)**: 6 levels, roughly evenly spaced from -10° to 80°
  (covers the full observed range plus a small margin, well beyond the old
  model's 25.8° ceiling).
- **Elbow (q3)**: 5 levels, roughly evenly spaced from 0° to 45° (covers
  observed 1.1-36.8° with margin).
- **Waist (q1)**: 4-5 levels spanning a MUCH wider range than the -15° to
  -4° window Y-axis testing produced — propose roughly -45° to +45°
  (symmetric around the Y-axis-only cluster) specifically so a tilt term,
  if needed, has genuine leverage to be fit against.

Not a full 6×5×5=150-pose factorial (too long a session) — propose a
**space-filling subset** (e.g. Latin-hypercube-style or a hand-picked grid
skipping redundant corners) targeting ~30 poses that still spans all three
joints' marginal ranges above, with denser sampling in the region most
used during real teleoperation (shoulder 0-50°, elbow 10-35°) and sparser
sampling at the range edges (pure coverage/extrapolation-guard poses).
**Exact pose list to be generated and shown before the session**, not
buried in this document — this is a parameter of the design, not a fixed
decision yet.

## Bidirectional approach procedure (per pose)

For each pose q_i:
1. **Negative-side approach**: operator brings the arm to q_i from a lower
   angle (approaching from below/behind on the dominant axis of motion for
   that pose), settles, holds still, waits for the acceptance window (below)
   to pass, sample is recorded as `tau_minus(q_i)`.
2. **Positive-side approach**: operator returns to a neutral pose, then
   brings the arm to the SAME q_i from a higher angle, settles, holds,
   records `tau_plus(q_i)`.
3. Both directions recorded before moving to the next pose.

Gravity and static/Coulomb friction are separated per the task's spec:
```
tau_g_measured(q)  ≈ [tau_plus(q) + tau_minus(q)] / 2
tau_c_measured(q)  ≈ [tau_plus(q) - tau_minus(q)] / 2
```

## Acceptance thresholds — DERIVATION STATUS: PROVISIONAL, NOT YET DATA-GROUNDED

The task asks for thresholds "derived from real sensor noise rather than
arbitrarily chosen." I checked existing logs for a clean stationary window
to derive this and **found neither available window trustworthy**:

- The gravity-OFF Stage-1 run's first 1.5s showed the slave's joint angles
  reading **exactly bit-identical** for 300 consecutive samples (qdot=0.000
  computed) — but this was the device **stiction-locked**, not freely
  resting; it demonstrates the encoder's resolution floor is at or below
  what 1.5s of true stillness can reveal, but is not a valid general noise
  estimate (a stuck joint isn't representative of a joint being actively,
  quietly held).
- The gravity-ON Stage-1 run's first 1.5s showed qdot std of 0.006-0.040
  rad/s across joints — but this window included a human hand on the
  master with natural physiological tremor transmitted through 1:1
  position coupling, plus possibly some early intentional motion before
  the "go" cue — an upper bound contaminated by real motion, not a clean
  noise floor either.

**Implemented**: `scripts/calibrate_stillness_noise.py` (approved 2026-09-08,
part C of the tooling). Entirely passive — subscribes only to
`/touch_right/dob_diagnostics` (`DobDiagnostics.msg`, "OBSERVATION
ONLY... nothing here feeds the command path"), no force commanded. Run
against the existing safe zero-force DOB-observation startup, with nothing
touching the stylus for a settle window (default 2s) + measurement window
(default 8s, task minimum 5-10s). Computes qdot/qddot std per joint,
stationary position variance, `tau_dis_hat`/`tau_cmd` noise, message-gap/
dropout detection, and derives `threshold = k × measured_std` (default
k=4.5, task proposed 4-5). Output feeds directly into
`analyze_pose_hold_sweep.py --qdot-threshold-rad-s`.

**Still provisional until this script is actually run against hardware**:
the 0.02 rad/s / 0.5 rad/s² placeholders below remain the fallback if the
calibration script hasn't been run yet, but should not be trusted over its
output once available.
- `|qdot| < 0.02 rad/s` per joint (placeholder, above the demonstrated
  near-zero stiction-locked floor, comfortably below the ~0.04-0.2 rad/s
  seen during actual slow motion in existing logs).
- `|qddot| < 0.5 rad/s²` per joint (placeholder, if acceleration is judged
  reliable — see logging section).

## Required logging (per accepted sample) — IMPLEMENTED

Per the architecture decision above, this is `hdapi_pose_hold_sweep.cpp`'s
log schema (tool tag `pose_hold_sweep`), not an extension of
`log_phase2_scenario.py`. Every field the task asked for, mapped to what's
actually produced — **no silent fallback zeros; unavailable fields are the
literal string `"NA"`, never `0`**:

| Task's requested field | Status | Source |
|---|---|---|
| timestamp | ✅ logged | `unix_time` column |
| joint position q1/q2/q3 | ✅ logged | `q1,q2,q3` |
| joint velocity | ✅ logged | `qdot1,qdot2,qdot3`, backward difference at measured (not assumed) dt |
| joint acceleration | ✅ logged, flagged | `qddot1,qddot2,qddot3` + `qddot_valid` |
| commanded joint torque | ✅ logged | `tau_hold_nm_1..3` = `J^T(q) · bias`, exact (bias is a known commanded Cartesian force, same "no calibration needed" property as the original tool) |
| Cartesian pose | ✅ logged | `pos_x,pos_y,pos_z` |
| approach direction | ✅ logged | `approach_dir` ('+'/'-') |
| pose ID | ✅ logged | `pose_id` |
| validity flags | ✅ logged | `qdot_valid`, `qddot_valid`, `settled`, `hold_ok` |
| saturation/clamp flags | ✅ logged (this tool's own clamps) | `bias_i` reaching `kMaxBiasN`/`kMaxTotalN` — checked in `analyze_pose_hold_sweep.py`'s `bias_cap_contamination` |
| base orientation | ✅ logged | `base_orientation` column, from `--base-orientation` CLI arg, `"UNRESOLVED"` if not given |
| commit hash | ✅ logged | `git_commit`, auto-detected via `git rev-parse HEAD` |
| device identifier | ✅ logged | `device` column |
| session identifier | ✅ logged, **required** | `--session <id>` — tool refuses to start without it |
| CartesianControllerTelemetry | **N/A, explicit "NA"** | structurally unavailable, see architecture decision above |
| DOB disturbance estimate | **N/A, explicit "NA"** | same |
| controller decomposition | **N/A, explicit "NA"** | same |
| current gravity-compensation command | N/A (this tool bypasses the production gravity-comp mechanism entirely — it commands its own known bias) | — |
| telemetry source age / staleness | Partially: `sag_m`/`max_wander_m`/`unsettled_frac`/`open_loop_drift_rad` (autoHold's own settle/verify diagnostics) serve the equivalent role for this tool's own control loop | — |

See `RIGHT_GRAVITY_IDENTIFICATION_TOOLING.md` for the exact CSV schema and
how each field is computed.

## Data quality gates (Phase 4, applied per sample before fitting)

Reject or flag:
- force saturation flag set
- travel-limit flag set
- sample within N seconds of a startup/arm ramp (`ramp_alpha < 1.0`)
- stale telemetry (age exceeds `stale_timeout_s`)
- `|qdot|`/`|qddot|` over threshold (from the calibration step above)
- obvious operator disturbance (large single-sample jump inconsistent with
  neighboring samples — simple outlier flag, not a judgment call made
  per-sample by a human)
- sample taken during a target transition (bridge `bridge_state` != holding
  steady, or target changed within the settling window)
- unstable holding behavior (position variance within the "hold" window
  exceeds some multiple of the calibration noise floor)

**Per-pose report** (required before any fitting, per the task): accepted
sample count, standard deviation, +/- approach-direction agreement
(`|tau_plus - (-tau_minus)|`-style consistency check), and a simple
confidence/quality score combining the above. Fitting does not proceed
until this summary is produced and reviewed.

## Model formulation (Phase 5, for the actual fit once data exists)

Regressor form per joint, matching the existing (structurally sound)
`fit_gravity.py` machinery:
```
tau_g_j(q) = Y_g(q) · θ_g
```
with `Y_g(q)` built from `BASIS_MINIMAL` initially, and `BASIS_TILT`
fit **only if** Phase 1's frame question resolves toward "base cannot be
leveled" — using the new ~30-pose, wide-waist-coverage dataset this
protocol produces (unlike the earlier attempts, which tried `BASIS_TILT`
with only 6-14 poses concentrated in a narrow waist range where it's not
even well-identified).

Fitting: `scipy.optimize.least_squares` with `soft_l1` robust loss
(matches the existing tool, down-weights poses where the ± decomposition
partially failed rather than letting one bad pose dominate).

**Train/validation split**: propose holding out ~20-25% of poses (roughly
6-8 of ~30), chosen to include some poses near the workspace edges (to test
extrapolation behavior specifically) and some interior poses (to test
interpolation) — not a pure random split, since edge-vs-interior
generalization is exactly the failure mode being tested for.

## Friction model (Phase 7, transparent, per task instruction)

Start from `tau_f = tau_c * sign(qdot) + b * qdot` (Coulomb + viscous).
`tau_c` comes directly from the same ± decomposition
(`tau_c_measured = [tau_plus - tau_minus]/2`). Additional complexity only
added if residuals clearly demand it — no neural network, per instruction.

## What happens next

Tooling (A: logging extension, B: stillness calibration, C: pilot-session
tooling, D: rig-orientation recording procedure) is now **implemented** —
see `RIGHT_GRAVITY_IDENTIFICATION_TOOLING.md`. Per the approved plan's own
stop condition, still **not started, pending pilot data-quality review**:

1. The physical base-orientation measurement itself (tooling to record it
   exists; the measurement hasn't been taken).
2. Running `calibrate_stillness_noise.py` against real hardware to replace
   the provisional qdot/qddot thresholds.
3. The ~8-10 pose PILOT session (not the full ~30-pose set — that only
   starts after the pilot is reviewed and passes).
4. Full ~30-pose identification, model fitting, friction characterization —
   all explicitly gated on the pilot passing.
