# Road to 4-channel bilateral teleoperation — status

Last updated 2026-09-03. Ticked items are done and verified; the evidence is
named so nothing here has to be taken on trust.

---

## System architecture — how the 4-channel setup actually works

This section didn't exist before 2026-09-03 despite the project having run
for over a week; the rest of this file is a chronological dev log, which
is good for evidence but bad for "how does this work right now." This is
the missing "current state of the machine" answer.

### Processes (4 terminals, all ROS2 nodes)

| terminal | process | node name | role |
|---|---|---|---|
| 1 | `touch_slave` (`run_master_left.sh`) | `teleop_master_left` | drives the **master** (left) Touch device: reads its pose, writes Cartesian force to it, runs its own DOB, publishes its own `force_ext_hat` |
| 2 | `touch_slave` (slave launch) | `teleop_slave_right` | same binary, drives the **slave** (right) device, symmetric to Terminal 1 |
| 3 | `bridge_master_slave.py` | `touch_master_slave_bridge` | the **position** loop: reads both arms' live pose, publishes `target_pose`/`force_cmd` to both -- this is C2 and C3 |
| 4 | `bilateral_4ch.py` | `bilateral_4ch` | the **force relay**: reads both arms' `force_ext_hat`, weights and republishes to `joint_torque_ext_cmd` -- this is C1 and C4 |

Terminals 1 and 2 are the only processes that actually talk to the real
HDAPI devices. Terminals 3 and 4 are pure ROS nodes sitting on top,
reading/writing topics on both arms -- neither one can do anything if
either Terminal 1 or 2 is down (confirmed the hard way 2026-09-03: Terminal
1 being silently down took out C2, C3, *and* C4's input simultaneously,
see the relevant dated section below).

### C2/C3 — the position bridge (Cartesian, always-on foundation)

`bridge_master_slave.py` reads each arm's live Cartesian pose and, subject
to per-axis offset limits, deadband, speed caps and a clutch button, writes
the *other* arm's `target_pose` -- each `touch_slave` node then runs its
own Cartesian PD (`stiffness_n_per_m`/`damping_n_s_per_m`) to chase that
target. C2 is the slave's real position driving the master's target; C3
is the reverse. This is the older, better-tuned half of the system --
confirmed on hardware since 2026-09-01 -- and everything else is built as
an addition on top of it, not a replacement.

### C1/C4 — the force relay (layered on top, 2026-09-03)

Each `touch_slave` node runs its own per-arm joint-space disturbance
observer (DOB, `touch_dob.hpp`) purely for **sensing**, regardless of
whether that arm is in Cartesian or joint-space control mode. Its raw
output (`tau_dis_hat_nm`) becomes a wire-ready estimate via
`computeForceEstimate()` (`touch_force_estimate.hpp`), published on that
arm's own `force_ext_hat` topic. `bilateral_4ch.py` subscribes to *both*
arms' `force_ext_hat`, applies a channel weight (`w1` for slave->master,
`w4` for master->slave) and an per-joint mask (`--c1-disable-joints`,
default excludes shoulder -- its identification is known-bad), and
republishes the result to the *other* arm's `joint_torque_ext_cmd`. That
receiving `touch_slave` node converts the relayed joint torque into a
Cartesian force via `cartesianForceFromJointTorque()` (the closed-form
inverse of the Jacobian-transpose relation `tau = J^T F`) and adds it to
whatever the Cartesian position spring (from C2/C3) is already commanding,
before the combined force is clamped and written to the device.

**Two deadbands stack in series, both real and both independently
tunable** -- this is the single most important fact for interpreting any
"why did nothing happen" result:
1. `kThreshold` in `touch_slave.cpp` (per joint: waist 0.076, shoulder
   0.064, elbow 0.050 N*m, "2x measured p99 noise") -- subtracted from the
   raw DOB estimate *before* anything leaves that arm's `force_ext_hat`
   topic at all. Below this, the relay's input is exactly zero, not small.
2. `kMinRelayTorqueNormNm = 0.02` N*m in the relay itself -- applied to
   the *already-weighted* (`w1`/`w4` applied) signal, before Cartesian
   conversion. This exists specifically to stop resting-state noise (never
   exactly zero) from leaking into position tracking (see the "workspace
   has problems" bug below).

Combined, the raw push needed at the source arm is roughly
`threshold + (0.02 / w_channel)` -- at the session's default `w1=0.15`
that's ~0.18-0.21 N*m depending on joint, which is why several genuinely
firm test pushes read as exactly zero until this was worked out
analytically and `w1`/`w4` were raised to 0.4 for verification.

### Safety: the oscillation-abort latch

`bilateral_4ch.py` continuously tracks the peak-to-peak of its own output
in 1-second windows. It trips (`self.tripped = True`, both `w1_eff` and
`w4_eff` forced to 0 until manually cleared) if either the pp exceeds
`--trip-pp-nm` (default 0.4 N*m) outright, *or* three consecutive windows
show a strictly increasing pp above `0.25 x trip_pp_nm` -- a "growing"
heuristic meant to catch a developing oscillation before it reaches the
hard ceiling. Confirmed 2026-09-03 to also fire on: a rapid, deliberately-
escalating verification push sequence (a false positive, since a human
ramping up force to find a threshold looks identical to a real growing
oscillation to this heuristic), a fast multi-axis push/release sequence
with no pauses, and a stale-to-live jump in input data right after a
node restart. Clear with `ros2 param set /bilateral_4ch manual_rearm
true` -- a live parameter set, no process restart needed.

### How "verified" is defined in this document

Every ✅ in the table below means at least one of these, and the entry
says which: (a) a live, real-hardware capture of the actual ROS topic the
mechanism depends on, with numbers printed here, not just "it felt right"
-- pushes below a computed deadband floor are expected to read zero and
are not evidence of a broken pipe; (b) a passing automated sim test,
named, that exercises the real C++ control code (not a Python
reimplementation that could drift from it); or (c) both. Anything resting
only on operator "it feels fine" without a corresponding capture is
flagged as such explicitly rather than folded into a ✅.

---

## Where this actually stands

**All four channels now confirmed on real hardware. C2 and C3 via the
Cartesian bridge (`bridge_master_slave.py`). C1 and C4 layered onto that
SAME bridge via a new Jacobian-based relay in `touch_slave.cpp` --
sim-verified, then confirmed live 2026-09-03 with quantitative topic-level
proof (see the last dated section above the "teleop quality" heading),
at `w1=w4=0.4`. Not yet felt-quality-tuned, and w1=0.5's oscillation trip
was never independently re-confirmed as a false positive.**

| | channel | direction | status | blocked on |
|---|---|---|---|---|
| C1 | slave **force** -> master | <- | ✅ | **Confirmed live on real hardware 2026-09-03** (see "all four channels" section below) with a multi-axis push sweep: waist -0.061/+0.013 N*m, elbow +0.074/-0.039 N*m, shoulder held at exactly 0.0 both directions (mask working as designed). Full mechanism -- deadbands, weighting, Jacobian relay -- described in "System architecture" above. Real bugs found and fixed along the way: (1) the oscillation-abort safety trip, working as designed, latched three separate times during testing (large test pushes, a rapid multi-axis sequence, and a master-reconnect transient) -- each cleared with `manual_rearm`, none were left tripped; (2) the joint-space PD's `Kd` damping term was reading raw, unfiltered qdot -- fixed to use `qdot_filt_rad_s_`; (3) elbow's torque-headroom problem (gravity feedforward consuming 75-94% of its clamp ceiling) -- fixed by raising `dob_torque_f_max_n`'s default 0.6->0.9N; (4) **`HD_CURRENT_JOINT_TORQUE`'s real units are mN*m, not N*m** -- confirmed against the vendor OpenHaptics reference PDF -- every joint torque this project ever wrote to real hardware was ~1000x too small, fixed with a real-hardware-only unit conversion; (5) tried switching C1/C2/C3/C4 all onto the joint-space path (`bilateral_4ch.py`'s own position blending) -- **rejected live**: too stiff, no leader/follower distinction, not a 1:1 Cartesian response. **Rebuilt as a relay onto the Cartesian bridge** instead (new `cartesianForceFromJointTorque` in `touch_dob.hpp`) -- found and fixed a real bug within minutes of first live use (resting-noise leaking into position tracking), now gated by a 0.02 N*m floor before conversion. Not yet felt-quality-tuned (verification so far is numeric/topic-level, not "does it feel good"); right-arm identification (shoulder blocked, Phase 4; base-tilt, Phase 5) is still the deeper prerequisite for fully trusting C1's numbers. |
| C2 | slave **position** -> master | <- | ✅ | **confirmed on real hardware 2026-09-03,** and reconfirmed alive 2026-09-03 (later) after the master node's restart: `/touch_left/target_pose` publishing steadily at 100Hz. Two real gaps found and fixed live: (1) the master has the same unweighted-vertical-axis problem the slave once had -- added `LEFT_GRAVITY_COMP_N` (new, symmetric to `SLAVE_GRAVITY_COMP_N`), working value found live around 0.45N, confirmed by the operator specifically checking +Y (gravity-opposed) vs -Y (gravity-assisted) motion, not just general "Y feels weak"; (2) `LEFT_MAX_OFFSET_M` (0.08m) was far tighter than the slave's own already-tuned `MAX_OFFSET_XYZ` (0.12-0.19m), giving C2 a visibly narrower workspace than C3 -- raised to 0.15m. **Neither number has been through a dedicated reach sweep yet** (unlike the slave's, which got one) -- treat as a working starting point, not a final tuned value |
| C3 | master **position** -> slave | -> | ✅ | working, confirmed 2026-09-01 with a real live single-axis-motion capture (5 tests, no wrong-axis/sign-flip in any), reconfirmed in a sim regression test 2026-09-02, and reconfirmed alive 2026-09-03 (later) after the master node's restart: `/touch_right/target_pose` publishing steadily at 100Hz, no tracking complaints during the same session's extensive C1/C4 testing (both arms were moved throughout). Requires default `axis_map=x,y,z`; do not reintroduce the unvalidated `x,z,-y` override |
| C4 | master **force** -> slave | -> | ✅ | **Confirmed live on real hardware 2026-09-03** (see "all four channels" section below): peak relay output norm 0.046 N*m, waist-dominant, sustained across a 40s multi-push capture, zero anomalous (>1.0 N*m) samples on the clean re-verification run. This was C4's first-ever real-hardware attempt -- previously only `test_write_armed_tracks_joint_target` had passed, sim-only. The joint-space PD control law itself needed no changes to get there -- two prior hypotheses for an earlier sim tracking gap (inertial mass-matrix coupling, then gravity-feedforward coupling) were both checked directly against evidence and found WRONG; the real cause was a bug in a TEST's own `_drive_and_measure` helper (fixed in the test, not the control law). Inherited every one of C1's 2026-09-03 fixes for free, since the code paths are symmetric (`cartesianForceFromJointTorque` in `touch_slave.cpp`, C1 on the master / C4 on the slave, same function, same 0.02 N*m deadband). One unexplained transient during testing (a single 2.79 N*m reading, ~25-35x every other measurement, no physical cause reported by the operator) did not reproduce on a fresh isolated capture (peak 0.218 N*m raw, zero samples over 0.5 N*m) -- most likely a one-off filter/state transient tied to the master's restart moments earlier, flagged rather than discarded in case it recurs. Not yet felt-quality-tuned. |

With `FORCE_REFLECT=false` (the default) there is no return path at all, so the
system is *unilateral*, not bilateral. With it true you get a force-shaped
signal on the master, but it is `gain x tracking_error` -- inferred from failure
to track, not measured. That is not C1.

**The distinction that matters:** a real force channel reports force whether or
not motion was commanded. The current scheme cannot, which is exactly why it
needs a stall gate.

---

## Phase 0 — Foundations (LEFT arm) ✅ COMPLETE

- [x] **FK / Jacobian convention verified** against hardware
      `hdapi_fk_residual`, 8 poses, RMS 1.674 mm over a 407 mm span. PASS.
      Nearest alternative convention was 35x worse, so there is no ambiguity.
      Everything downstream reports real N*m because of this.
- [x] **Gravity map G(q)** from static breakaway sweep
      `fit_gravity.py`, session B, 10 poses. Shoulder R^2 0.966, elbow 0.936,
      design condition 33.6. Waist NOT modellable (see open issues).
- [x] **Friction floor** per joint
      `fit_friction.py`. Elbow credible; waist and shoulder weak, because
      Cartesian excitation cannot isolate single joints.
- [x] **Nominal inertia M_n** from bounded dynamic excitation
      `fit_inertia.py`. waist 3.3694e-3, shoulder 2.9452e-3, elbow 2.3291e-4
      kg*m^2. Tip-inertia cross-check 26.4 g against a 45 g spec.
- [x] **Auto-hold** added to the sweep and excitation tools
      Finds and freezes the support force per pose, so poses that will not
      hold themselves can still be measured without a hand on the arm.

## Phase 1 — Observer (LEFT arm) ✅ COMPLETE

- [x] **DOB implemented** in `touch_dob.hpp`, Ohnishi form, Tustin discretised
- [x] **Gravity compensation wired in** -- cut the shoulder/elbow floor 54-58%.
      Without it the shoulder floor was 142% of that joint's torque authority,
      i.e. the gate could not fire on anything the actuator can produce.
- [x] **56 unit tests passing** (run-verified 2026-08-26, not source-counted --
      `test_touch_dob` binary printed 56 PASS / 0 FAIL / "ALL TESTS PASSED";
      an earlier "55" here was a source-level assertion-macro count, off by
      one from what the binary actually reports), including the steady-state
      sign check, the one-frame gravity alignment, and uncalibrated-scale
      suppression
- [x] **Mutation-tested** -- each guard verified to actually catch its bug
- [x] **Offline replay through the REAL C++ observer** (`dob_replay_cpp.cpp`),
      cross-checked against the Python prototype. Deltas grew with cutoff
      exactly as the ZOH-vs-Tustin argument predicts; ranking unchanged.
- [x] **Wired into `touch_slave.cpp`, OBSERVATION ONLY**
      `/touch_left/dob_diagnostics` at 200 Hz. All four write gates closed.
- [x] **Contact-annotation + analysis harness** built and smoke-tested

## Phase 2 — The gating experiment ✅ DONE — STOP

- [x] **Contact separability session** on the left arm (2026-08-26)
      Bag: `~/dob_logs/separability_session1` (280 s, 167,980 msgs, 4 annotated
      contact episodes, contamination guard clean — exactly one
      `/contact_annotation` publisher, no alternating/duplicate-publisher
      artifact). Analysed with `analyze_contact_separability.py`.

**Result, per joint, exactly as the script reports it — not averaged:**

| joint | unloaded p99 | contact p99 | separation ratio | vs. offline replay | gate false-positive (unloaded) | gate true-positive (contact) |
|---|---|---|---|---|---|---|
| waist | 0.03172 N·m | 0.01748 N·m | **0.6x** | 0.48x | 0.0% | 0.0% |
| shoulder | 0.09362 N·m | 0.09362 N·m | **1.0x** | 2.04x | 91.3% | 91.4% |
| elbow | 0.07605 N·m | 0.08105 N·m | **1.1x** | 2.04x | 32.0% | 30.9% |

**Verdict, per the pre-committed decision rule, applied as written (not
loosened):**

- **waist: STOP.** Ratio 0.6x — contact reads *lower* than the unloaded
  floor, the wrong direction. The gate never fired in either segment (0.0%
  both ways): `threshold_nm` sits above anything this session produced,
  loaded or not.
- **shoulder: STOP.** Ratio 1.0x — flat, no separation at all. The gate fires
  on ~91% of samples regardless of contact, so it is not gating on contact,
  it is gating on being on. `unloaded p99` is also 2.04x the offline replay
  prediction (0.0458 N·m), i.e. live behaviour is materially worse than what
  the offline capture predicted.
- **elbow: STOP.** Ratio 1.1x — essentially flat. Same `2.04x` gap against
  the offline replay prediction as the shoulder.

**No joint reaches the >=2-3x proceed threshold. This is a full STOP, not a
mixed result** — all three joints land in or below the ~1x STOP band; none
even reach the 1.5-2x marginal range.

**Gravity-extrapolation control check.** A pure control stretch (before any
annotated contact, 17,669 samples, master moved through the gravity map's
fitted q2 range with nothing touching it) was isolated from the full-session
aggregate and checked separately: shoulder false-positive 86.1%, elbow 34.5%
— both close to the full-session unloaded numbers (91.3% / 32.0%). **This
rules out "the false positives are concentrated at out-of-range low poses"
as the explanation** — the high shoulder/elbow firing rate is present
throughout normal-range motion, not an artifact of dwelling near the edge of
the gravity map's fit. The floor itself, not extrapolation, is the limit.

**What this means:** C1 and C4 are not achievable on this hardware with the
current observer, at the current `g_dis`/`threshold_nm`. This is exactly the
outcome Part 1's noise-floor analysis predicted (40-57% of torque authority
consumed by the floor) and confirms it against real annotated contact rather
than an offline replay assumption. Per the task's explicit instruction, no
`threshold_nm` or `g_dis` retuning was attempted in this session — that is a
separate, later decision for the operator, not a default response to a
STOP result.

**Independently re-verified, 2026-08-26 audit.** Re-ran
`analyze_contact_separability.py` against `~/dob_logs/separability_session1`
fresh rather than trusting this document's transcription: reproduced 0.6x /
1.0x / 1.1x (0.55x / 1.00x / 1.07x precise) and the exact gate fire
percentages above, bit for bit. The bag's own `ros2 bag info` also matches
the claimed 280s / 167,980-message shape. This STOP result is genuinely
traceable to an artifact on disk, not asserted on faith.

## Phase 3 — Prerequisites for any torque write 🟡 PARTIALLY STARTED

**Corrected 2026-08-26 audit** — two claims below were previously wrong:

- [x] ~~Fix the missing abort handler~~ **Already present.**
      `sigaction(SIGINT, &sa, nullptr);` exists at `hdapi_torque_calib.cpp:240`.
      Grepped and confirmed directly; there is no gap here to fix.
- [~] **`hdapi_torque_calib` has been run once, partially, on the left arm.**
      `~/dob_logs/calib_left.csv` contains 13 rows of real measured data
      (dated 2026-08-25, e.g. `last_jt` 17.47/34.95/64.07 raw units at
      0.1/0.2/0.3 N on the Y axis) -- one axis, one pose, not the x/y/z at
      2-3 poses the procedure calls for. `~/dob_logs/calib_right.csv` exists
      (same timestamp) but contains only the header row -- an attempt was
      made on the right arm and produced no usable data.
      `last_joint_torque_scale_nm_per_unit` therefore remains at its
      sentinel; this partial run does not by itself satisfy prerequisite (a).
- [ ] **Derive per-joint torque clamps** (prerequisite (b))
      `derivePerJointTorqueLimits()` exists; the caveat that three independent
      per-joint limits are NOT equivalent to the Cartesian clamp they replace
      still has to be resolved.
- [ ] **Set `calibration_a_complete` / `clamp_b_complete`** deliberately
      Still `false` in every session actually run: the only code path that
      sets them (`configureBilateral()`) requires `joint_space_control:=true`,
      which no launch script passes (confirmed by grep, see C4 above).

## Phase 4 — RIGHT arm identification 🟡 IN PROGRESS (started 2026-09-01)

Everything in Phase 0, repeated. The arms are two physical units with their own
calibration files, their own friction, and their own base tilt -- symmetry
cannot be assumed.

- [x] **FK / Jacobian check on the right arm** -- `hdapi_fk_residual "Default
      Device"`, 13 poses, spread widely including near-limit configurations.
      **PASS**: RMS residual after offset-removal = 1.489mm (worst pose
      2.482mm), well under the 2mm bar; relative residual (RMS/span) = 0.38%.
      Alternative-convention search confirms nothing beats the as-implemented
      mapping. Angle convention, frame convention, and both link lengths
      (L1=L2=0.135m) are confirmed consistent for the right arm. Fitted base
      offset c = [-0.000232, 0.024168, -0.170744] m.
- [x] **Static sweep -> gravity map** -- `hdapi_static_sweep "Default Device"
      --log ~/dob_logs/sweep_right.csv`, 6 poses x 6 directions (36 breakaway
      measurements). Shoulder gravity fit R²=0.961, elbow R²=0.984 (both
      strong). **Waist gravity is unreliable**: max|g_waist|=0.013 N·m
      (should be ≈0, gravity cannot load a joint rotating about vertical) --
      matches the already-documented Phase 5 base-tilt symptom seen on the
      LEFT arm, so this is a known rig-quality issue, not a new right-arm
      defect. Only 6 poses (README recommends 10+); acceptable first pass,
      not maximally robust.
- [x] **Dynamic excitation -> M_n** -- `hdapi_dynamic_excitation "Default
      Device"`, 2 sessions (`dyn_right.csv` + `dyn_right_2.csv`, merged to
      `dyn_right_merged.csv` after dropping a bad pose-4 capture from
      session 1 -- see incident note below). 6 clean poses x 3 axes.
      Fit: M_n waist=4.30e-3, shoulder=3.21e-3, elbow=1.36e-4 kg·m²; R²
      0.49-0.78 (moderate, expected given Cartesian-not-joint-space
      excitation). Coulomb cross-check (dynamic vs static breakaway):
      **elbow ratio 0.61 (physically sensible, sliding < breakaway as
      expected)**; waist ratio undefined (same known base-tilt issue);
      **shoulder ratio 17.02 -- flagged by the tool itself as "one of the
      two is wrong," unexplained, do not trust shoulder's friction number
      yet.** Apparent-tip-mass sanity check: 15.0g median vs 45g spec
      (0.33x) -- PLAUSIBLE, same order of magnitude (weak check, can't catch
      <3x errors).
      **Bad-capture incident (session 1, pose 4):** operator released the
      arm at an extended/awkward pose; the tool's auto-averaging captured an
      unstable reading ("wandered up to 131.3mm"), producing a wrong holding
      force. Settled 138.6mm below the release point (expected ~9mm) and
      then produced abnormal chirp displacement (peak 0.95 rad vs 0.01-0.4
      rad normal range) before being aborted. Confirmed operator/device
      were fine; the pose-4 rows were excluded from the merged CSV before
      fitting, and a clean redo of that data point was captured in
      `dyn_right_2.csv` instead.
- [x] Fit gravity/friction/inertia -- written to
      `~/dob_logs/results/gravity_right_minimal.json`,
      `~/dob_logs/results/friction_right.json`,
      `~/dob_logs/results/inertia_right.json` (mirrors the left arm's
      existing file layout in the same directory). **Not yet assigned to
      `touch_dob.hpp`'s `JointDobParams`** -- that remains a separate,
      explicit step, and the DOB stays inactive on the right arm until then.
- [x] **Verify against the right arm's own offline replay** --
      `replay_dob.py` against `dyn_right_merged.csv` with the fitted M_n/
      gravity, sweeping g_dis 5-80Hz, output at
      `~/dob_logs/results/dob_tuning_right.json`. **Confirms the shoulder
      anomaly is real and blocking**: comparing suggested `threshold_nm`
      (p99 x2) against `max_torque_nm` at f_max=1.5N (waist=0.405,
      shoulder=0.2025, elbow=0.2025 N·m) -- shoulder's threshold (0.204 N·m
      even at the most conservative 5Hz cutoff) meets or exceeds its own
      torque budget at *every* cutoff tested, meaning the noise floor alone
      consumes the whole per-joint torque ceiling. **Shoulder is not usable
      for C1 with the current identification** -- needs a redone static
      sweep with more/wider shoulder-focused poses before revisiting.
      **Waist and elbow both have workable headroom at a conservative
      cutoff (g_dis≈31.4 rad/s / 5Hz)**: threshold sits below the torque
      ceiling for both (waist 0.156 vs 0.405; elbow 0.170 vs 0.2025), though
      waist's gravity term still carries the known base-tilt caveat. For
      reference, the already-committed LEFT arm values are even more
      conservative (1.59Hz shoulder/waist, 0.80Hz elbow), so 5Hz is a
      cautious, not aggressive, starting point.

### 2026-09-01 addendum -- gravity model degrades with wider pose coverage; this is a rig issue, not a shoulder-specific one

Attempted to fix the shoulder anomaly above by redoing the static sweep with
much wider shoulder-angle (q2) coverage: 8 more poses
(`sweep_right_2.csv`), spanning -11.8&deg; to +32.6&deg; vs the original
6 poses' -15.3&deg; to +25.8&deg;, merged into `sweep_right_merged.csv`
(14 poses total). Refitting `fit_gravity.py` against the wider set made
**every** joint's R&sup2; worse, not just shoulder's:

| joint | R&sup2; (original 6 poses) | R&sup2; (widened, 14 poses) |
|---|---|---|
| waist | 0.396 | 0.073 |
| shoulder | 0.961 | 0.254 |
| elbow | 0.984 | 0.708 |

Friction-floor numbers from `fit_friction.py` stayed essentially unchanged
between the two pose sets (shoulder median \|c\| 0.00143 -> 0.00164, elbow
0.02362 -> 0.02270) -- the degradation is specific to the analytical
gravity fit, not the raw friction data.

**Interpretation:** the original 6-pose fit's apparently strong shoulder/
elbow R&sup2; was very likely an artifact of narrow pose clustering, not
evidence of a genuinely good model -- exactly the failure mode
`fit_gravity.py`'s own README warns about ("low residuals do not vindicate
it -- collinear terms always fit the sampled points and then extrapolate
nonsense"). Since ALL THREE joints degraded together when coverage widened
(not just shoulder, which is the one already flagged), this points at a
**rig-level modelling limitation** -- most likely the same base-tilt issue
already tracked in Phase 5 for the left arm. A tilted base makes "vertical"
inconsistent across the workspace; a `q2`/`q3`-only gravity model can fit
well over a narrow pose cluster where the tilt's effect is roughly constant,
then degrade as poses spread out and the tilt's varying contribution gets
exposed. **More/wider static-sweep poses alone will not fix this** -- it
needs the base leveled first (Phase 5) before the gravity model can be
trusted across a realistic pose range.

**Decision:** reverted `gravity_right_minimal.json`/`friction_right.json`
to the original 6-pose fit as the working interim baseline (locally
consistent within that narrow range, even if it doesn't generalize) rather
than the worse-fitting 14-pose version. The 14-pose data
(`sweep_right_merged.csv`) is preserved in `~/dob_logs/` in case it's
useful once the base is leveled and the fit can be legitimately redone
across a wide range. **Do not re-run the gravity fit against the wider
pose set and expect it to be an improvement** -- it demonstrably is not,
until Phase 5 happens first.

Roughly two hardware sessions; both are now substantially done in one
session (Steps 0-2 all complete, only offline-replay verification and the
explicit JointDobParams assignment remain).

## Phase 5 — Rig quality ❌ NOT STARTED

- [ ] **Level the base.** Waist gravity is unmodelled; the data shows a
      3-6 degree tilt, and the tilt terms failed cross-validation on 10 poses.
      **Now confirmed on the RIGHT arm too (2026-09-01)**: widening static-
      sweep pose coverage from 6 to 14 poses made the gravity fit R² worse
      for ALL THREE joints (waist 0.40->0.07, shoulder 0.96->0.25, elbow
      0.98->0.71), not just one -- see Phase 4's 2026-09-01 addendum. This
      is the blocking dependency for getting a trustworthy right-arm gravity
      model across a realistic pose range, not more static-sweep data.
- [ ] **Widen gravity pose coverage.** Currently fitted over q2 in
      [+0.175, +1.359]; teleoperation uses more than that, and 46.8% of the
      dynamic samples were already extrapolating.
- [x] **Friction model.** No longer true as of Phase 2.6 (2026-08-26) for the
      LEFT arm -- see that section below. Still true for the RIGHT arm, which
      has no identified friction (or any) parameters at all (Phase 4).

## Phase 6 — Close the loop 🟡 IN PROGRESS (sim-only so far)

- [ ] Open `TOUCH_DOB_ENABLE_TORQUE_WRITE` on real hardware, one arm, low
      ceiling -- not yet attempted, everything below is sim-only
- [ ] Open `enable_torque_write` at runtime, verify clamps bite (real hw)
- [ ] **C4**: master force estimate -> slave -- control law stable in sim
      but not passing its own test yet, see channel table
- [x] **C2**: slave position -> master (bridge must command the master) --
      built and sim-confirmed 2026-09-02, not yet run on real hardware
- [x] **C1**: slave force estimate -> master -- DOB pipeline + `bilateral_4ch.py`
      relay both confirmed working in sim 2026-09-02, not yet run on real
      hardware
- [x] Local controllers both sides; choose the 4-channel weighting --
      `bilateral_4ch.py` presets exist (`unilateral/pp/pf/4ch_safe/4ch`)
- [ ] Stability: passivity margin, transparency, verify no limit cycle --
      C4's control law still has this open; C1/C2 clean so far in sim
- [ ] Re-tune with a human in the loop -- needs real hardware

---

## 2026-09-02 — C1/C2 built and sim-verified; C4 fixes banked; two bugs found and fixed in bilateral_4ch.py

Continuation of the 2026-09-01 hardware session's momentum, this time
sim-only per explicit instruction (no force applied to real hardware).

### C4 — stable fixes banked, still not passing its own test

- **Torque clamp properly parameterized.** `configureDob()`'s per-joint
  torque ceiling (`kMaxTorque`) was a hardcoded `{0.162, 0.081, 0.081}`
  array, completely disconnected from the `derivePerJointTorqueLimits()`
  helper that supposedly computes it -- confirmed by direct `tau_cmd`
  logging that a test-only `max_force_n` override had zero effect on the
  actual clamp. Fixed: new `dob_torque_f_max_n` ROS parameter (default
  0.6N, reproducing the exact prior hardcoded values byte-for-byte), wired
  through `derivePerJointTorqueLimits()` at `configureDob()` time. Real
  hardware behavior is unchanged unless this parameter is explicitly
  overridden.
- **Friction breakaway feedforward: tried, reverted, then re-added with a
  real fix -- the limit cycle is genuinely gone.** First attempt added a
  Coulomb-term feedforward (gated on raw `|qdot|`) to fix small-residual
  stalls; confirmed working in isolation, but once the torque budget was
  widened enough for it to have any effect, it sustained a **limit cycle**
  on the waist joint -- raw `qdot` is a single-tick backward difference at
  ~1kHz, and the simulated encoder is quantized, so a single tick produces
  a fixed-magnitude "velocity" spike (~0.4 rad/s) even at genuine rest,
  indistinguishable from real motion by a threshold gate; the resulting
  rapid open/close gate chatter (not the feedforward's magnitude) is what
  sustained the cycle. **Fixed 2026-09-02 (second pass):** added a
  separate, low-pass filtered velocity estimate (`qdot_filt_rad_s_`,
  single-pole, ~30ms time constant, its own state distinct from
  `dob_bank_`'s deliberately-unfiltered observer `qdot`) and gated the
  breakaway term on that instead. Verified: a smooth, isolated 4-second
  probe now shows clean monotonic convergence to ~80% closed by t=2.5s, no
  oscillation, no stalling -- a real, confirmed fix for the limit-cycle bug.
- **Still not passing the test, but the reason changed and the earlier
  theory here was wrong.** Chasing why the *test itself* (not the isolated
  probe) still fails led to an initial hypothesis of inter-joint inertial
  coupling (waist/shoulder's motion physically dragging elbow via mass-
  matrix off-diagonal terms) -- **this was checked and is incorrect**:
  `arm_model.hpp` explicitly documents that the simulator's `m_coupling`
  (inertial cross-coupling) is zero by default and no validated M(q) with
  off-diagonal terms exists anywhere in this repo ("pretending to a full
  M(q) here would be worse than the honest diagonal"). There is no
  inertial coupling in this sim to compensate for. The real mechanism is
  more likely gravity-feedforward coupling: `gravityTorqueNm(q2, q3)` is
  evaluated with the CURRENT q2/q3 for every joint, including elbow's own
  term, so as shoulder actively moves toward its own target, elbow's
  gravity feedforward genuinely shifts in response (legitimate physics,
  not a bug) -- interacting with torque saturation, this may be
  overpowering elbow's own small PD authority. **Not yet confirmed** --
  next session should log elbow's live gravity-feedforward contribution
  during a drive before building anything, rather than assume the
  mechanism. Do not attempt a mass-matrix/M(q) feedforward for this: it
  would target a coupling that does not exist in the model.
- **Current state**: stable (no crashes, no oscillation, confirmed via
  repeated isolated runs), still fails `test_write_armed_tracks_joint_target`
  in its exact 2.5s/49Hz-publish scenario, now in the roughly -60% to -75%
  range (up from -93% to -118% before this fix -- real improvement, not
  fully sufficient). Also confirmed the test's own failure is NOT primarily
  a publish-rate artifact: a 4x-faster-publish repro showed the same
  wrong-direction elbow pattern in 2 of 3 runs.
- **`test_dob_floor.py`'s waist failure -- root-caused and fixed, not a
  bug.** Flagged earlier this session as unrelated/pre-existing; actually
  investigated afterward. Cause: `hd_sim`'s default base-tilt mismatch
  (`TOUCH_SIM_BASE_TILT_RAD`, ~4.5deg) is deliberately always-on (per
  `hd_sim_api.cpp`'s own comment, representing "the rig's actual physical
  base tilt") and loads the waist specifically -- gravity should only
  reach a joint rotating about vertical when the base is NOT level.
  Confirmed directly: forcing tilt to 0 drops waist p99 from 0.159 to
  0.062 N*m (ratio 1.62, inside the old band) while shoulder/elbow barely
  move (0.0508->0.0508, 0.0018->0.0022) -- exactly the physically-expected
  pattern. The default tilt value is not arbitrary either: it matches this
  same project's real-hardware-measured base tilt (Phase 5's "3-6 degree
  tilt" finding) -- so the sim was correctly reproducing a real rig
  property, the test's shared 2.5x ceiling just never accounted for how
  disproportionately it loads the waist. Fixed by giving waist its own
  ceiling (4.5x) with the reasoning recorded in the test file.

### C1 — DOB pipeline + relay confirmed working, two real bugs found and fixed

Built a sim harness: two `touch_slave_sim` instances (master idle, slave
with a synthetic wall contact via `TOUCH_SIM_ENV=wall`), plus
`bilateral_4ch.py --architecture custom --w1 1.0 --w2 0 --w3 0 --w4 0` to
isolate C1 from the other channels.

- **First attempt tripped the oscillation-abort safety latch almost
  immediately**, both with an abrupt force step and a gentle 3-second
  ramp, at the identical moment, with `pp=0.0165 N*m` against a `0.4 N*m`
  threshold. Root cause: the "growing" detector (3 consecutive
  strictly-increasing 1-second peak-to-peak windows) triggers on the
  ordinary onset of **any** signal appearing from a near-zero baseline --
  a real contact ramping up from nothing always looks like "growing" for
  its first few seconds, indistinguishable from a genuine building
  oscillation by this check alone.
- **Fixed**: added a magnitude floor (`kGrowingFloorFrac = 0.25`) --
  "growing" now only counts if the latest window has also reached 25% of
  `trip_pp_nm`, so a small ordinary contact onset that plateaus below that
  floor no longer false-trips, while a genuinely dangerous ramp (which
  would cross this floor well before reaching the raw threshold) is still
  caught.
- **Second bug found while trying to recover from the (at the time,
  false) trip**: `ros2 param set /bilateral_4ch manual_rearm true` crashed
  the node -- `AttributeError: 'bool' object has no attribute
  'bool_value'` in `_on_param()`. rclpy's parameter callback already hands
  back the unwrapped Python `bool` via `Parameter.value`, not a raw
  `ParameterValue` message; the code was written for the wrong API shape.
  **Fixed**: `p.value.bool_value` -> `p.value`. Verified both via a live
  trip-and-recover path and a direct unit-style check (constructed a
  `Parameter`, called `_on_param` against a mock node state, confirmed it
  clears `tripped`/history without crashing).
- **After both fixes**: re-ran the same contact-push scenario, no false
  trip, and the relay confirmed exact: `peak |right force_ext_hat| during
  push = 0.01301 N*m`, `peak |left joint_torque_ext_cmd| during push =
  0.01301 N*m` -- identical. **C1's software path (DOB estimate -> C1
  weighting -> master's commanded external torque topic) is confirmed
  correct in sim.**
- **Never run on real hardware.**

### C2 — sim-confirmed, twice

- Unit-level: `_update_c2`'s math (axis-map inverse, clamp) checked
  directly against 4 hand-computed cases (no motion, single-axis push,
  clamp saturation, remapped-axis push) -- all matched exactly.
- Live integration: two `touch_slave_sim` instances + the bridge with
  `--enable-c2 true`, force-pushed the slave stand-in, subscribed to both
  `/touch_right/pose` and `/touch_left/target_pose` continuously (not a
  single before/after snapshot). Result: `right_pose delta` and
  `left_target delta` identical to the millimeter (`+24.07mm, +4.06mm,
  +0.70mm` both), through the push and the release/settle back to
  baseline.
- **Never run on real hardware.**

### C3 — sim regression added, still hardware-confirmed

- Same two-sim-node harness, force-pushed the **master** stand-in this
  time (opposite direction from the C2 test), confirmed `left_pose delta`
  exactly matches `right_target delta`.
- Reconfirmed a second time while `bilateral_4ch.py` (C1-only) ran
  concurrently, to check the two systems don't interfere -- still an exact
  match (`+29.61mm` etc. on all 3 axes), confirming the bridge (Cartesian,
  C3/C2) and `bilateral_4ch.py` (joint-space, C1/C4) can coexist without
  topic conflicts.

### Connected-channels test

Ran the bridge (C3, proven) and `bilateral_4ch.py` scoped to C1-only
(`--w2 0 --w3 0 --w4 0`, since C4's control law isn't ready) **concurrently**:
pushed the master (C3 test) and the slave into the wall (C1 test) at the
same time. C3 stayed exact throughout. C1 read zero on both sides this
specific run -- not a regression, this run's random resting pose happened
to settle almost exactly on the wall plane (`right_anchor.y=-0.0503` vs
`wall_y=-0.05`), so the push added negligible extra penetration; C1's
mechanism was already cleanly proven nonzero in the isolated run
immediately before this one. The actual point of this test -- do the two
systems interfere -- is answered: they don't.

### Real-hardware test plans

**C2 (slave position -> master): DONE, 2026-09-03.** Executed the plan
below; two real gaps found and fixed along the way, not anticipated by the
plan itself:

1. Standard 3-terminal launch, `ENABLE_C2=true` on both master and bridge,
   confirmed `teleoperation live` and the C2-on log lines. Moved the slave
   by hand while holding the master still.
2. **First gap: the master's commanded target (`/touch_left/target_pose`)
   was updating correctly on all 3 axes (confirmed via direct topic echo
   -- X ~55mm, Y ~77mm, Z ~35mm range, all real), but the master's own
   arm only visibly followed on X** -- a software-vs-physical split,
   confirmed by checking the raw topic rather than trusting feel alone.
   Root cause: same unweighted-vertical-axis problem the slave had before
   `SLAVE_GRAVITY_COMP_N` existed -- the master had no equivalent. Fixed:
   added `LEFT_GRAVITY_COMP_N`/`LEFT_GRAVITY_AXIS` to `bridge_master_slave.py`
   (`_publish_left_gravity_comp()`, symmetric to the existing slave-side
   method) and wired into `run_bridge.sh`. Tuned live in steps (0.15 ->
   0.30 -> 0.45), each step specifically checked against **+Y (gravity-
   opposed) vs -Y (gravity-assisted)** motion -- the operator noticed early
   that +Y specifically didn't follow while -Y did, which is what pointed
   at gravity rather than a general weakness. Landed around 0.45N as a
   working value; **not yet through a dedicated sweep** the way the
   slave's 0.55N was.
3. **Second gap, found after the first was fixed: C2's usable range felt
   much narrower than C3's.** Cause: `LEFT_MAX_OFFSET_M` defaulted to a
   flat, never-tuned 0.08m, well under the slave's own already-swept
   `MAX_OFFSET_XYZ` (0.12-0.19m per axis). Raised to 0.15m as a working
   starting point -- also not yet swept per-axis the way the slave's was.
4. Master's own stiffness (`LEFT_STIFFNESS_N_PER_M`) was also raised from
   the original conservative 15 N/m to 28 N/m during this session as an
   early (partially effective on its own) attempt before the gravity-comp
   fix was built -- kept, since it did help Z/X responsiveness even though
   it didn't solve Y alone.

**Follow-up not yet done:** a proper reach/gravity sweep for
`LEFT_GRAVITY_COMP_N` and `LEFT_MAX_OFFSET_M`, mirroring the controlled
sweep that produced the slave's own numbers (Phase "teleop quality"
section above), rather than the live trial-and-error values currently set.

**C1 (slave force -> master): ATTEMPTED, 2026-09-03 -- pipeline confirmed
correct end-to-end at the ROS level, "felt at master" still unconfirmed.**
Full narrative in the dated section below (`## 2026-09-03 -- C1's first
real-hardware attempt`). Short version: the compile-time write gate was
armed for the real build (new opt-in `TOUCH_HAPTICS_ENABLE_REAL_TORQUE_
WRITE` CMake option), the relay was verified correct at every stage up to
and including the actual `HD_CURRENT_JOINT_TORQUE` write, two real bugs
were found and fixed (oscillation-abort needing re-arm after early large
test pushes; `Kd` damping using raw instead of filtered qdot, causing
visible noise on the waist axis), and gravity feedforward was found to
already saturate the shoulder's entire torque ceiling at rest, leaving it
zero headroom regardless of any C1 signal (separate from, and on top of,
shoulder's existing exclusion from the relay for identification-quality
reasons). The session ended before getting a clean, unambiguous "yes, felt
it" or "no, still nothing" from a final controlled push -- **next session
should start with one clean, low-weight waist push, trip cleared, Kd fix
already in place, and get that answer before doing anything else with C1.**

**C4 (master force -> slave):**
- **Not ready for a hardware trial.** The control law doesn't pass its own
  sim test yet, and even if it did, `TOUCH_DOB_ENABLE_TORQUE_WRITE` would
  need to be deliberately enabled for a hardware build first (see Phase 6's
  first checklist item) -- a separate, careful decision given it arms
  torque write on real hardware. Do not attempt until C4's sim test passes
  cleanly and that compile-time decision has been made deliberately, not
  as a side effect of testing something else.

---

## 2026-09-03 -- C1's first real-hardware attempt

Given the go-ahead to arm C1's real torque-write path, which this project
had never done before (the compile-time gate had only ever been enabled for
`touch_slave_sim`).

**Build change.** Added `TOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE`, a new
CMake option defaulting `OFF`, to `CMakeLists.txt` -- when `ON`, it adds
`TOUCH_DOB_ENABLE_TORQUE_WRITE=1` to the real `touch_slave` target only
(unlike the sim target, which always has it). This preserves the existing
safety argument (a routine build never arms real torque write) while making
it possible to opt in deliberately with
`-DTOUCH_HAPTICS_ENABLE_REAL_TORQUE_WRITE=ON`. Rebuilt with it on; the sim
regression suite (`test_c1_relay_live.py`, `test_c2_live.py`,
`test_c3_live.py`, `test_dob_floor.py`) still passes cleanly afterward.

**Launch scripts extended.** `run_master_left.sh` and `run_slave_right.sh`
gained `JOINT_SPACE_CONTROL`, `DOB_TORQUE_F_MAX_N` (both), and
`DOB_ENABLE`/`DOB_INERTIA_JSON`/`DOB_GRAVITY_JSON` (slave only, defaulting
to the right arm's own fitted files, not the left-arm defaults baked into
`touch_slave.cpp`) -- all off/default by default, matching the project's
existing "opt-in, byte-identical otherwise" convention.

**Shoulder excluded from the relay by code, not just by convention.** Added
`--c1-disable-joints` to `bilateral_4ch.py` (default `"shoulder"`) -- a
per-joint mask applied to `tau_c1` before weighting, so the known-bad
shoulder identification (Phase 4's finding: its noise floor meets or
exceeds its own torque budget) can never ride along in the relay
regardless of `--w1`.

**Live findings, in the order they were found:**

1. **Nothing felt at the master, first pass (`w1=0.1`, `dob_torque_f_max_n
   =0.15`).** Traced to two compounding causes, not one: the weight itself
   was very conservative, and `derivePerJointTorqueLimits(0.15N)` produces
   a per-joint ceiling small enough (waist 0.0405, shoulder/elbow 0.02025
   N*m) that gravity feedforward alone was found to already consume most
   or all of it -- confirmed directly via `/touch_left/joint_torque`
   (the real `HD_CURRENT_JOINT_TORQUE` readback): shoulder sat pinned
   exactly at its clamp ceiling at rest, before any C1 signal.
2. **A real push registered almost entirely on shoulder, not the joint
   being touched.** Pushing near the elbow link produced a large response
   on `force_ext_hat`'s shoulder axis and almost none on elbow -- expected
   serial-arm kinematics (a contact force generates reaction torque at
   every upstream joint via the Jacobian, not just the nearest one), not a
   bug, but it meant the first real test happened to land on exactly the
   joint excluded from the relay.
3. **Waist confirmed as a clean, high-headroom test axis** (near-zero
   gravity load, largest per-joint ceiling of the three) -- rotating it
   produced real `force_ext_hat` swings up to ~0.2-0.4 N*m.
4. **Oscillation-abort trip latched (`channel_reason=6` on all channels),
   confirmed via `/bilateral_status`.** Correct, designed behavior --
   large deliberate push-and-release motions produce exactly the kind of
   1-second peak-to-peak swing the trip watches for. Cleared with
   `ros2 param set /bilateral_4ch manual_rearm true`, but retripped almost
   immediately on the next large push -- a real usability friction point
   for testing with big motions specifically, not a bug in the trip logic
   itself.
5. **With the trip cleared, the relay chain was traced end-to-end and
   found correct up to the master's real torque write** -- `force_ext_hat`
   (slave) -> `/bilateral_status`'s `master_torque_cmd_nm` (relay) ->
   `/touch_left/joint_torque` (actual `HD_CURRENT_JOINT_TORQUE` readback)
   were checked together live and matched at every stage once the trip
   wasn't the thing zeroing everything out.
6. **A second, unrelated real bug found while checking the master's
   resting torque: `x` (waist) was "jumping meaninglessly" at rest.**
   Root cause: the joint-space PD's `Kd*qdot` damping term was reading
   `qdot` (the DOB's raw, unfiltered backward-difference velocity), which
   the code's own existing comments already document as spiking ~0.4 rad/s
   from encoder quantization even when a joint is genuinely still. Waist's
   `Kd=0.5` (highest of the three) and near-zero gravity load (least
   masking) made this the most visible joint for it. A filtered velocity
   (`qdot_filt_rad_s_`) already existed in the file, built earlier for the
   Coulomb-breakaway term specifically -- it just hadn't been wired into
   the main PD damping term too. Fixed in `touch_slave.cpp`; confirmed
   live afterward that waist's resting torque is now flat instead of
   jumping. Rebuilt, sim suite re-run clean.

**Not resolved by end of session:** a final, clean, low-weight push with
the trip cleared and the Kd fix live did not get a definitive felt/not-felt
answer before the session moved to restoring normal C2/C3 teleoperation
(the C1 test setup, using `joint_space_control:=true` with `w2=w3=0`,
necessarily disables the normal Cartesian position channels while it
runs -- restoring C2/C3 meant stopping it). **Next session's first step for
C1 should be exactly that one clean test**, not a new investigation --
trip status check, gentle-to-firm waist push, watch `/touch_left/
joint_torque`'s `x`, and get an actual yes/no on whether it's perceptible
by hand.

---

## 2026-09-03, later same day — C4's real bug found and fixed; C1's torque headroom gap found and fixed (sim-only, no hardware)

Given a mandate to get C1 and C4 working properly without touching real
hardware. Both fully resolved by the end of this session.

### C4: the TODO's gravity-coupling hypothesis was wrong -- the real cause was a test-harness bug

Added `joint_pd_active`/`joint_pd_gravity_ff_nm`/`joint_pd_error_term_nm`/
`joint_pd_cmd_nm`/`joint_pd_write_ok` to `DobDiagnostics.msg` so the
joint-space PD block's gravity-feedforward, error/damping/Coulomb, and
post-clamp terms can be told apart instead of only ever seen summed
(`touch_slave.cpp`, wired through `DeviceState`/`state_`/the publish block).

First checked whether the earlier same-day `Kd` filter fix (raw ->
`qdot_filt_rad_s_`, made for an unrelated real-hardware reason) had already
changed `test_write_armed_tracks_joint_target`'s result: it had -- baseline
moved from -60%/-75% to a still-failing but no-longer-negative +14%.

Built `sim/tests/diag_c4_gravity_coupling.py` to log the new fields during
the exact failing scenario. First attempts gave a diagnostic script that
consistently passed (~+0.85) while the real pytest consistently failed
(-60% to -65%) -- root-caused to the diagnostic's own `dob_diagnostics`
subscription (published near the full servo rate) perturbing the sim
server's real-time timing even when carefully rate-limited client-side.
Fixed by moving diagnostics to a separate node/executor on a background
thread, keeping the main publish loop byte-identical in shape to the real
test's.

With clean data, `joint_pd_error_term_nm` read **exactly 0.0 for 70-190
consecutive servo ticks**, at the same three points in every failing run:
at startup (DDS discovery latency, absorbed by the existing 1.5s hold
margin, not a real problem), and -- critically -- at `_drive_and_measure`'s
own `q_c.spin_for(0.1)` (hold->drive transition) and `q_c.spin_for(0.2)`
(immediately before reading the final position). Neither window ever calls
`target_pub.publish()`; combined with `stale_timeout_s_`'s 0.05s default,
`joint_target_fresh` goes false during both, silencing the PD term
entirely -- gravity feedforward alone does not track a moving target, only
approximately cancels gravity at the current pose, so the arm drifted
freely. The second gap sits right before the final measurement, so a real
chunk of the "failure" was measuring that drift, not the control law's
actual tracking.

**Confirmed this was a test bug, not a production bug**: `bilateral_4ch.py`
(the real C4 path) publishes `joint_target` unconditionally every tick with
no equivalent gap. Two hypotheses were checked and rejected before this was
found -- inertial mass-matrix coupling (checked 2026-09-02, `m_coupling` is
zero in `arm_model.hpp`) and gravity-feedforward coupling overpowering
elbow's PD authority (this session's own leading hypothesis, going in --
rejected once actually measured). Neither survived contact with evidence;
see the `RESOLVED [C4]` comment now in `touch_slave.cpp` for the full
record in place of the old speculative TODO.

**Fix**: `test_joint_space_write_gated.py`'s `_drive_and_measure` now keeps
publishing the current target during both settle windows
(`_publish_and_settle()`, replacing the bare `q_c.spin_for()` calls). No
control-law change was needed. `test_write_armed_tracks_joint_target`
passes consistently now (5/5 individual runs, 3/3 full-file runs, all
green) with no other code touched.

### C1: elbow has the same torque-headroom problem shoulder had, now caught by a permanent test

New `sim/tests/test_c1_torque_headroom.py`, reusing the `DobDiagnostics`
fields above: drives the sim through the 10 poses in
`gravity_left_minimal.json`'s own calibration table (the same file both sim
and real hardware load) and asserts gravity feedforward alone leaves at
least 30% headroom below `derivePerJointTorqueLimits`'s clamp, for waist
and elbow (the two joints C1 actually relays post-mask; shoulder is out of
scope, already excluded).

At the old `dob_torque_f_max_n` default (0.6N), waist passed comfortably
(worst case 11% consumed) but **elbow failed at 8 of 10 poses**, 75-94%
consumed (as low as 5.6% headroom) -- the same silent-saturation failure
mode found live on real hardware for shoulder earlier the same day, just
never checked for the joints C1 actually uses.

Given the choice of fix (raise the shared default / give elbow its own
ceiling / exclude elbow from C1 too), **raised `dob_torque_f_max_n`'s
default 0.6 -> 0.9N** (`touch_slave.cpp`) -- computed directly from the
calibration data so elbow's worst-case headroom clears 30% with real margin
(37.1%, not a bare pass). This widens every joint's ceiling, sim and real
hardware both, a deliberate real-hardware-affecting choice, not a silent
tuning bump -- see the comment at the parameter's declaration for the exact
computation. `test_c1_relay_live.py`'s per-axis relay check was also fixed
to assert shoulder's relay is actively zero (proving the mask works)
instead of skipping it, since it previously assumed all three axes relay
proportionally, predating the mask.

Also added `test_c1_oscillation_trip_repeated_push_baseline` (same file):
a sim baseline for the "retrips easily under repeated big pushes" friction
found live -- at `w1=0.5`/default `trip_pp_nm`, a 4-cycle 0.6N push/release
profile produced 0/4 trips in sim (a legitimate recorded baseline for this
specific profile, not a target -- real hardware's bigger, less controlled
pushes are what tripped it live).

**Full sim/tests suite: 19 passed, 1 skipped (pre-existing, unrelated --
`hdapi_torque_calib.cpp` tool binary not present in this environment), 0
failed.** Run twice to check for flakiness given the shared-code touches;
identical both times.

---

## 2026-09-03, resumed hardware session -- found and fixed a 1000x unit bug in every real joint-torque write this project has ever made

After the sim-only work above, resumed live C1 testing on real hardware
with the new fixes in place. Still felt nothing at the master, at every
weight tried (0.1 through 1.0) and every torque ceiling tried (0.15N
through 0.9N) -- including a case where the real, correctly-relayed,
correctly-clamped commanded torque swung `±0.13 N*m` on the waist axis
(confirmed via the actual `HD_CURRENT_JOINT_TORQUE` readback, not an
estimate), comparable in magnitude to `SLAVE_GRAVITY_COMP_N=0.55N`'s
Cartesian bias, which had been clearly felt earlier the same day.

**Root cause: `HD_CURRENT_JOINT_TORQUE`'s real units are milli-Newton-
meters, not Newton-meters -- confirmed directly against the vendor
documentation** (`/opt/OpenHaptics/Developer/3.4-0/doc/OpenHaptics_
ProgGuide.pdf`, Table 6-2, "New Joint Torque Parameters in OpenHaptics
3.4.0": "Units: mNm (TJ1, TJ2, TJ3)"). Every torque value in this file's
joint-space path (`tau_cmd`, `gravityTorqueNm`, `dob_torque_f_max_n`,
`fric_coulomb_nm`, ...) is N*m throughout, matching the dynamics
literature this project's math follows -- and gets passed straight into
`hdSetDoublev(HD_CURRENT_JOINT_TORQUE, ...)` with no conversion. Every
joint torque this project has EVER commanded on real hardware, across
every session, was received by the device as that many milli-newton-
meters: roughly **1000x too small to ever be felt**, no matter how much
weight or torque ceiling was raised within the (wrongly-scaled) range
tried.

**Why no sim test ever caught this:** `hd_sim_api.cpp`'s own
`hdSetDoublev`/`hdGetDoublev` stand-in for this parameter stores and
returns the value as-is, into a variable literally named `cmd_joint_nm`,
consumed directly as N*m by the sim's own physics -- the sim was built to
be self-consistent with this file's own (wrong) assumption, never checked
against the real API's actual documented unit contract. There was no real
joint-torque write to catch the discrepancy against until this session's
live C1 testing. The Cartesian `HD_CURRENT_FORCE` path (which C2/C3/
gravity-comp all use) is genuinely in Newtons per the same documentation
(Table A-8) and matches this project's own units exactly -- which is
exactly why that path has always worked correctly and this one never did.

**Fix:** added `TOUCH_HAPTICS_REAL_HD_HARDWARE`, a compile definition
applied only to real-hardware targets (`CMakeLists.txt`'s `add_touch_node`,
never `add_touch_sim_node`), and a `kJointTorqueNmToDeviceUnits` constant
in `touch_slave.cpp` (1000.0 on real hardware, 1.0 in sim) applied at
every `hdSetDoublev(HD_CURRENT_JOINT_TORQUE, ...)` call (N*m -> mN*m) and
inversely at the one `hdGetDoublev(HD_CURRENT_JOINT_TORQUE, ...)` read
(so the `joint_torque` telemetry topic stays in N*m on both real and sim,
matching every other torque value this project publishes). Internal state
(`tau_cmd`, `joint_pd_cmd_nm` diagnostics, clamping) is untouched --
conversion happens only at the two points where this file actually
crosses into/out of the real HDAPI's own units. Rebuilt; full sim suite
re-run clean (19 passed, 1 pre-existing skip, identical to before this
change) -- confirms the sim's own behavior is completely unaffected, as
intended.

**Not yet done:** a real-hardware retest with the fix in place. This is a
significant, safety-relevant change -- correctly-scaled joint torque has
never been applied to this device before, even though the underlying N*m
values (e.g. `dob_torque_f_max_n=0.9`) are the same numbers already used
today. **Next session should restart at a torque ceiling well below
today's 0.9N default** (e.g. back to 0.15-0.3N) for the first correctly-
scaled test, since "reasonable" now means something ~1000x stronger than
anything actually felt on this hardware to date.

---

## 2026-09-03, later same day -- joint-space C2/C3 tried and rejected; C1/C4 layered onto the Cartesian bridge instead

With the unit fix in place, retested C1 on real hardware -- still felt
nothing at first, traced to repeatedly checking whether the master moved
*by itself*. With `--w2 0 --w3 0` (isolated C1 test), the master never
moves autonomously by design; C1 is felt as resistance while holding the
stylus, not motion. Once clarified, moved on to building genuine 4-channel
operation instead of continuing to chase a single isolated-C1 felt-force
confirmation.

**Tried: switching everything to `bilateral_4ch.py`'s joint-space path**
(the `pp` preset, C2=C3=1.0, no force) so all 4 channels could eventually
share one unified controller, as the script was originally designed for.
**Rejected after a live hands-on test** -- three real, structural problems,
not tuning issues:
1. **Stiffness far too high.** The joint-space Kp/Kd gains were only ever
   tuned while torque was silently ~1000x too weak (the unit bug above).
   Correctly scaled, the same gain numbers are far stiffer than anyone has
   ever actually felt.
2. **No leader/follower distinction.** `pp` sets both C2 and C3 to full
   weight simultaneously -- a symmetric tug-of-war, not a clear dominant/
   compliant relationship.
3. **Not a 1:1 Cartesian response.** `Kp` acts on joint angles, not
   Cartesian x/y/z -- how much Cartesian motion a given joint error
   produces depends on the Jacobian at that pose, not a fixed, intuitive
   direction. Described live as movement continuing along x "up to a
   threshold" then unexpectedly shifting into z. This is a structural
   property of naive joint-space PD, not a tunable bug -- it's exactly why
   the Cartesian bridge (`bridge_master_slave.py`) was built and preferred
   in the first place.

**Built instead: C1/C4 layered onto the existing Cartesian bridge**, so
force channels ride alongside the already-good Cartesian position feel
instead of requiring the joint-space path. New math in `touch_dob.hpp`:
`cartesianForceFromJointTorque(J, tau, det)` -- solves `J^T * F = tau` for
`F` via a closed-form 3x3 inverse (adjugate/determinant), the inverse of
the already-existing `jacobianTransposeTimes`. Precondition (caller must
have already checked `!singularityMetrics(q).near_singular`) is
deliberately not re-checked internally, since `det(J^T) == det(J)` means
that existing check already covers it -- avoiding a second source of truth
for the same fact.

Wired into `touch_slave.cpp`'s Cartesian write path (a new block right
before the final `HD_CURRENT_FORCE` write, independent of `joint_space_
control_`'s own branches): when a fresh `joint_torque_ext_cmd` arrives (C1
on the master, C4 on the slave -- symmetric code, works on whichever node
it runs on), converts it to a Cartesian force via the arm's own Jacobian
and adds it to `force_device`, re-clamped by the existing `max_force_n_`
ceiling.

**First version had a real bug, found live within minutes of testing**:
it applied ANY fresh signal unconditionally, including bilateral_4ch's own
resting-state residual (confirmed: waist/elbow's disturbance estimate is
never exactly zero, ~0.0003-0.05 N*m, even with nothing touching the peer
arm). Continuously nudging the receiving node's TRUE Cartesian position
with that residual let `bridge_master_slave.py` -- which reads this node's
live measured position to compute the peer's C2/C3 target -- mistake the
nudge for deliberate motion, slowly dragging the peer arm in an axis
nobody intended. This is exactly the "workspace has problems" symptom
reported live, and it was specifically confirmed to only start after this
new relay was added (isolating it from any pre-existing bridge behavior).

**Fixed** with `kMinRelayTorqueNormNm = 0.02` N*m: the raw joint torque
must clear a real, deliberate-contact-sized floor BEFORE any Cartesian
conversion happens at all -- not a downstream clamp on the result, which
would not have prevented the position-tracking contamination (the leak
was from a *small* force continuously nudging position, not from an
unbounded one). Also added a gap found while implementing this: the
relay had no `command.enabled`/deadman gate at all, meaning it could
apply force to a device the operator had not enabled -- now requires the
same gate the primary Cartesian spring/damper already does.

Added `sim/tests/test_c1_cartesian_relay_deadband.py` -- drives
`touch_slave_sim` in plain Cartesian mode with no target_pose/force_cmd
(so the ONLY possible nonzero output is this relay), publishes a
below-floor and an above-floor `joint_torque_ext_cmd`, and asserts the
former produces no Cartesian force while the latter does. Verification is
via a `TOUCH_DIAG_CARTESIAN_RELAY=1` stderr trace of `force_device` at the
actual write site, not the `last_force` ROS telemetry topic -- that topic
was found, while building this test, to not reliably reflect a
Cartesian-only write in this exact scenario (a separate, pre-existing sim
readback quirk, independent of the relay logic itself, which the direct
trace confirms is correct).

**Full sim/tests suite: 21 passed, 1 skipped (pre-existing, unrelated), 0
failed.** Run twice for flakiness given the shared-code touches; identical
both times.

**Not yet retested on real hardware with the fix in place.** The
workspace-contamination bug was found and stopped live (Terminal 4 halted
before it caused lasting drift), then fixed and verified in sim only.
Next session's C1/C4-onto-Cartesian real-hardware test should start with a
low `--w1` (and `--w4` if testing C4 too) and confirm the deadband holds
under real, load-dependent resting noise -- not just the sim's clean
synthetic signal -- before trusting it fully.

---

## 2026-09-03, same day, later still -- C1 AND C4 confirmed live on real hardware, quantitatively

Real-hardware retest of the Cartesian relay from the section above, this
time all the way through to numeric proof, not just a felt "resistance"
report. Both channels found to need a much firmer push than expected to
even clear their own inputs' deadbands, which is why earlier attempts this
session kept reading zero even on genuine pushes -- resolved by isolating
each stage of the pipeline separately with direct topic captures rather
than continuing to guess from felt sensation alone.

**Two independent deadbands stack, and both were undershot at first:**
1. `computeForceEstimate()` (`touch_force_estimate.hpp`) subtracts each
   joint's own noise floor (`kThreshold` in `touch_slave.cpp`: waist
   0.07632, shoulder 0.064, elbow 0.050 N*m -- "2x measured p99 noise")
   from the raw DOB estimate before anything leaves that arm's
   `force_ext_hat` topic at all.
2. The Cartesian relay itself only converts to force when the
   *already-weighted* (`w1`/`w4` applied) signal clears
   `kMinRelayTorqueNormNm = 0.02` N*m.
   At the session's initial `w1=0.15`, that's roughly `threshold +
   0.02/0.15` of raw torque needed -- around 0.18-0.21 N*m depending on
   joint, well above what several genuinely firm test pushes produced
   (measured peak 0.098 N*m raw). Confirmed by direct capture of
   `/touch_right/dob_diagnostics.tau_dis_hat_nm` (real, sustained signal)
   simultaneously with `/touch_right/force_ext_hat.tau_ext_deadbanded_nm`
   (flat zero, same window) -- proved the *first* deadband was the
   bottleneck, not a broken pipe.

**Fix for verification purposes: raised `w1`/`w4` from 0.15 to 0.4**
(`bilateral_4ch.py --w1 0.4 --w4 0.4`; C4 had also never been enabled at
all up to this point -- the running command had `--w4 0` the whole
session prior to this). This is a test-time weight change, not a
control-law fix.

**Two real operational issues found and resolved along the way, unrelated
to the relay math itself:**
- Raising `w1` to 0.5 (tried before settling on 0.4) tripped
  `bilateral_4ch.py`'s oscillation-abort latch (`pp=0.1004 N*m` via the
  "3 consecutive growing windows" heuristic, not the absolute `0.4`
  ceiling). Very likely a false positive from the verification method
  itself (asking for progressively harder pushes produces exactly the
  monotonic-rise signature the heuristic watches for), not evidence of
  real instability at that weight -- not re-tested since 0.4 already gave
  a working signal margin.
- The master node (`teleop_master_left`) had been Ctrl-C'd earlier in the
  session (end of an unrelated diagnostic capture) and never restarted,
  which had silently taken out C2/C3 *and* C4's input (`tau_h_hat`,
  `REASON_STALE`) at the same time -- confirmed via `ros2 node list`
  missing the node entirely and `force_ext_hat`'s `Publisher count: 0`.
  Restarting it fixed all three at once. A second oscillation-abort trip
  immediately followed the master's reconnect (`safety_state=2` on all
  four channels) -- root-caused to the sudden stale-to-live jump in
  `tau_h_hat` looking like a spike to the same detector, cleared with one
  `manual_rearm` and did not recur.

**One unexplained transient, investigated and not reproduced:** an early
C4 capture read `norm=2.79 N*m` (waist 1.90, shoulder 2.03) -- 25-35x
every other measurement this session, physically implausible for a hand
push on this device. User confirmed no physical event (no jam/hard-stop/
bump). Re-checked with a fresh, isolated capture of the master's raw
`dob_diagnostics.tau_dis_hat_nm` during an equivalent push: peak 0.218
N*m, zero samples above 0.5 N*m across the whole window -- did not
reproduce. Most likely a one-off filter/state transient tied to the
master's restart moments earlier in this same section, not a persistent
bug; flagged here rather than silently discarded in case it recurs.

**Final confirmed numbers, both directions, same session:**
- **C1** (slave push -> master relay output,
  `/touch_left/joint_torque_ext_cmd`): peak norm 0.108 N*m (waist 0.105,
  elbow 0.082 max), nonzero on 4710/7851 samples across a 40s multi-push
  capture.
- **C4** (master push -> slave relay output,
  `/touch_right/joint_torque_ext_cmd`): peak norm 0.046 N*m (waist-
  dominant), nonzero on 2096/7853 samples across a 40s multi-push
  capture, zero anomalous (>1.0 N*m) samples on the re-verification run.

Both channels: `channel_active=true`, `channel_reason=REASON_OK`,
`safety_state=NOMINAL` at time of writing. **C1 and C4 are now confirmed
live end-to-end on real hardware for the first time**, on the Cartesian-
relay architecture, at `w1=w4=0.4`. Not yet re-tuned down from 0.4, not
yet run through a felt-quality pass (numeric proof only, not "does it feel
good"), and the `w1=0.5` oscillation trip was never independently
re-tested to confirm it's really a false positive rather than a real
margin problem.

---

## 2026-09-03, same day, final pass -- C1 axis sweep, and all four channels confirmed simultaneously for the first time

**C1 tested across multiple push directions**, not just one axis, to make
sure the earlier single-direction proof wasn't a fluke and that the
shoulder mask is a real, working exclusion rather than a coincidence.
First attempt at this (a fast +x/+y/-y/+z/-z sequence with no pauses)
tripped the oscillation-abort latch again -- consistent with the "growing"
false-positive mechanism described in the architecture section above.
Re-armed, retested slower (one axis, 2s hold, 3-4s pause, next axis) --
**did not re-trip**, and produced real, sustained, multi-axis signal at
`/touch_left/joint_torque_ext_cmd`:

| axis | max value (N*m) |
|---|---|
| waist | +0.013 / -0.061 |
| shoulder | 0.0 / 0.0 |
| elbow | +0.074 / -0.039 |

3238 of 11852 samples nonzero across the sweep. Waist and elbow both
clear the 0.02 N*m relay floor in both directions with real margin.
Shoulder reading exactly 0.0 in both directions across the entire capture
is itself a positive result, not an absence of one -- it confirms
`--c1-disable-joints=shoulder` is actively masking that joint's
(known-bad) contribution rather than shoulder simply never being loaded
by these particular pushes.

**C2 and C3 reconfirmed alive** after the master's earlier restart (see
the section above): `/touch_left/target_pose` and `/touch_right/target_pose`
both publishing steadily at 100Hz, and no tracking/wrong-axis complaints
during this entire session's C1/C4 testing despite both arms being moved
repeatedly throughout.

**Result: all four channels (C1, C2, C3, C4) confirmed active and healthy
at the same time, on real hardware, for the first time this project has
achieved that.** `bilateral_4ch.py` running `--w1 0.4 --w4 0.4`;
`/bilateral_status` shows `channel_active=[true,false,false,true]` (C2/C3
report `REASON_ZERO_WEIGHT` in this status message only because
`bilateral_4ch.py`'s own weights for them are 0 in this custom
architecture -- their actual mechanism is the separate `bridge_master_
slave.py` process, not this node, and is independently confirmed above).

**What "confirmed" does NOT yet mean, to be explicit:** none of today's
four channels have been through a felt-quality pass together (does the
combined system feel coherent, not just "does each channel's topic show
the right number in isolation"); `w1`/`w4` are at a verification-
convenience value (0.4), not a tuned-for-feel one; and the earlier
`w1=0.5` oscillation trip was never independently re-examined to
distinguish a true stability margin problem from the false-positive
mechanism identified above. Next session's natural next step is a
combined felt-quality pass, starting at the current `w1=w4=0.4` weights.

---

## 2026-09-04 — Kd overdamping comment corrected; sim-only C1/C4 Cartesian-relay performance baseline

Two separate threads, both sim-only, no real hardware touched.

**1. Corrected a stale code comment, no gain change.** `touch_slave.cpp`'s
joint-space `Kd` comment claimed "~2x overdamped"; verified against real
`inertia_left.json` `M_n` values it is actually 6-8x (waist 7.86x, shoulder
6.18x, elbow 6.31x, via `Kd_crit = 2*sqrt(Kp*M_n)`). Re-ran
`test_joint_space_write_gated.py` to check whether this over-damping was
related to the historical "C4 test failing at -60% to -75%" finding from
2026-09-03 above -- **it is not**: that test currently passes clean at
85.2% closed_fraction with these same `Kd` values unchanged, confirming
the 2026-09-03 diagnosis (test-methodology bug, not a gains bug) still
holds. Comment fixed in place; no `Kp`/`Kd` changed.

**2. New finding: `applied_force` silently omits the C1/C4 relay's
contribution.** `state_.applied_force_n = force_ros` (`touch_slave.cpp`
~line 1301) is captured from `force_ros`, computed BEFORE the "C1/C4-onto-
Cartesian relay" block runs (~lines 1189-1254, which only mutates the
separate `force_device` array). The actual `hdSetDoublev(HD_CURRENT_FORCE,
...)` write DOES include the relay's contribution (confirmed via
`TOUCH_DIAG_CARTESIAN_RELAY=1`'s stderr trace), so the device itself
renders correctly -- but the `applied_force` ROS topic, which this
project's own tooling (`log_phase2_scenario.py`'s docstring, `monitor_
forces.sh`) documents as "the REAL rendered Cartesian force... the haptic
force the operator actually feels," is stale and reports the PRE-relay
value whenever C1/C4 is active. `last_force` (reads `HD_LAST_FORCE`, the
driver's own post-write readback, one frame lagged) is correct and does
include it -- confirmed by direct comparison against the relay math trace.
**Any Phase 2 capture taken with C1/C4 active would show an incomplete
`applied_force` trace for the receiving arm.** Not fixed here (touches
shared telemetry code, flagging rather than silently patching); the fix is
straightforward (fold the relay's Cartesian delta into `force_ros`, or
just document the caveat) but wasn't applied without checking first.

**3. Sim-only baseline of the ACTIVE C1/C4 Cartesian-relay path**
(`sim/tests/diag_c1c4_cartesian_baseline.py`, new). Two `touch_slave_sim`
instances, `joint_space_control:=false` (production default), one side
gets a wall-contact model, driven with a controlled 0.6N ramp-hold-release
push; `bilateral_4ch.py` relays with position weights zeroed to isolate
the force channels. Each case run at target weight vs weight=0 to isolate
the relay's own contribution on the peer's real `last_force`.

**Independently reproduces the exact deadband-cliff mechanism the
2026-09-03 real-hardware session found empirically** (that session's
`w1=0.15` read zero, `w1=0.4` gave signal) -- strong convergent validation
that this sim harness is trustworthy for further C1/C4 tuning without
spending real-hardware time. This session's own sweep, at this scenario's
specific pose/push-axis:

| weight | C1 peer force (N) | C4 peer force (N) |
|---|---|---|
| 0.0 (control) | 0.000 | 0.000 |
| 0.3 | -- | 0.000 |
| 0.4 | 0.000 | -- |
| 0.5 | 0.000 | 0.222 |
| 0.6 | 0.137 | 0.267 |
| 0.7 | 0.167 | 0.311 |
| 0.8 | 0.199 | 0.355 |
| 0.9 | 0.236 | 0.400 |
| 1.0 | 0.277 | 0.444 |

**The last real-hardware-used weights (`w1=0.5`, and `w4=0.3` from the
`4ch_safe` preset) both sit AT OR BELOW the cliff in this scenario** --
zero relayed force for a firm 0.6N contact. Even at full weight (1.0),
only 46% (C1) / 74% (C4, magnitude) of the source's true force reaches the
peer -- meaningful attenuation persists well above the cliff, from the two
stacked deadbands plus the Cartesian round-trip conversion and re-clamp.

**Torque headroom, measured (not theoretical) from the source node's own
truth-CSV `tau_cmd` during the hold window, 0.6N contact alone:** shoulder
reaches 82-89% of its `dob_torque_f_max_n=0.6N` ceiling on BOTH arms from
contact alone, before any relay signal is added -- consistent with why C1
already excludes shoulder (`--c1-disable-joints` default), but **C4 has no
equivalent mask and relays shoulder's raw signal regardless**, despite
shoulder's identification being the same known-bad data C1 was excluded
over. Worth a decision, not yet made.

**Joint-space PD path benchmark (user's explicit ask: compare, don't
assume rejected), same push, `w=1.0`:** delivers substantially MORE torque
to the peer at the same weight -- C1's peer hits elbow at 100% of ceiling
and shoulder at 91%; C4's peer hits shoulder 89% and elbow 78% -- versus
the Cartesian-relay path's ~0.28N/0.44N Cartesian outputs, well under
saturation. The joint-space path has more raw authority at the same
weight, at the cost of running closer to the safety ceiling and (per the
2026-09-03 real-hardware felt-quality assessment already on record above)
worse position-tracking feel. A genuine trade-off, not a settled question.

**Caveat, important:** none of this sweep's runs tripped the
oscillation-abort latch (single ramp-hold-release cycle, not repeated
escalating pushes) -- but the real 2026-09-03 session's `w1=0.5` DID trip
under a different (repeated-push) pattern, never independently
re-confirmed as a false positive. **This sim sweep showing no trip at
`w1` up to 1.0 is NOT evidence that those weights are safe against the
oscillation guard** -- it only characterizes deadband/headroom, a
different failure mode. `test_c1_oscillation_trip_repeated_push_baseline.py`
already exists for that check; it should be re-run at whatever weight is
chosen from this data before trying it live.

**Recommendation:** the two production weights currently on record
(`w1=0.5`, `w4=0.3` from `4ch_safe`) are not verified-safe defaults so
much as untested-for-this-purpose ones -- they sit right at this
scenario's cliff. The 2026-09-03 session's own verification value
(`w1=w4=0.4`) is closer to but still below where this sweep finds
consistent signal (0.6+). Before the next real-hardware session: (1) run
`test_c1_oscillation_trip_repeated_push_baseline.py`-style repeated pushes
at `w1=w4=0.6-0.7` in sim to check the safety-latch margin at a weight that
this sweep shows actually produces signal; (2) decide C4's shoulder-mask
question; (3) fix or at least prominently document the `applied_force`
gap before trusting any future Phase 2 capture that has C1/C4 active.

Full raw sweep data: `/tmp/c1c4_baseline_results.json`; per-run 1kHz
ground truth: `/tmp/c1c4_baseline_truth/*.csv`; summary plot:
`/tmp/c1c4_weight_cliff.png`. Harness:
`ros2_ws/src/touch_haptics/sim/tests/diag_c1c4_cartesian_baseline.py`.

---

## 2026-09-04, continued -- applied_force fixed with a regression test; the
## two-stage deadband mathematically characterized and 5 architectures
## compared. No hardware-facing parameter changed.

**1. `applied_force` fixed** (`touch_slave.cpp` ~line 1301). Was `force_ros`
(pre-relay); now `joint_path_commanded ? force_ros :
mapDeviceVectorToRos(force_device)` -- `force_device` is the actual,
post-relay, post-clamp value written to `HD_CURRENT_FORCE`, mirroring the
exact pattern `last_force` already used. Joint-space branch intentionally
left unchanged (out of scope, off by default). New
`sim/tests/test_applied_force_reflects_relay.py`: confirmed to FAIL
against the pre-fix code (applied_force stuck at 0.0 with the relay
active) and PASS against the fix, plus a relay-inactive regression check
and an applied_force/last_force cross-consistency check. Full sim suite:
35 passed, 1 pre-existing skip, 0 failed.

**2. Deadband math, precisely.** Two stages, different units/domain/form,
confirmed by reading `computeForceEstimate()` and the Cartesian relay
block directly:
- **Stage 1** (`touch_force_estimate.hpp`, SOURCE node, PER-JOINT, BEFORE
  weighting): `y[j] = sign(x[j]) * max(0, |x[j]| - threshold1[j])` --
  SHRINKING/continuous (C0, a kink not a jump). `threshold1` = waist
  0.07632 / shoulder 0.064 / elbow 0.05 N·m.
- **Stage 2** (`touch_slave.cpp`'s relay block, PEER node, on the NORM of
  the full 3-vector, AFTER weighting): `output = weighted if
  ||weighted||>=0.02 else 0` -- HARD gate, a genuine step of up to 0.02
  N·m at the boundary.
- Combined minimum raw signal for joint j (unmasked, single-axis
  dominant) to produce ANY output: `|x[j]| >= threshold1[j] +
  0.02/(w*Cf)` -- exactly `bilateral_4ch.py`'s own existing
  `effective_threshold_nm()`, now independently re-derived and confirmed.
- **With the calibration-file bug below fixed**, this predicts and
  matches the empirical cliff much more closely than the previous
  (buggy-calibration) sweep did: real signal starts around `w1=0.5`,
  consistent with the 2026-09-03 real-hardware session's own finding that
  `w1=0.4` was the minimum verification-usable weight.

**Bug found while doing this: the C1 sim captures were using the LEFT
arm's identified `inertia_left.json`/`gravity_left_minimal.json` on the
"slave" (right-arm role) node**, because `touch_slave.cpp`'s own
`declare_parameter` defaults always point at the left-arm files and
nothing in the capture script overrode them. `run_slave_right.sh` (the
real launch script) DOES override `DOB_INERTIA_JSON`/`DOB_GRAVITY_JSON` to
the `*_right.json` files -- production was never affected, only this new
sim tooling was. Fixed in `diag_deadband_capture.py`; all C1 numbers below
are from the corrected capture.

**3. Five deadband architectures compared**, reconstructed OFFLINE from
one captured raw (`tau_ext_nm`, pre-any-deadband) signal per source node
across 3 push patterns (`escalating_fast`/`escalating_slow`/
`multiaxis_fast`, the last matching the exact push sequence that tripped
the real 2026-09-03 oscillation latch) -- not 5 separate C++ builds, so
every variant is compared against the identical underlying signal, and no
production code path was touched to get this data:

| variant | transmission @ w=0.6 (C1/C4) | ever tripped latch | notes |
|---|---|---|---|
| (a) current 2-stage | 18.8% / 31.5% | No | the shipped system |
| (b) current, higher w | same formula, swept | No (0.4-1.0 tested) | not a different architecture |
| (c1) stage-1 only (drop the norm gate) | 18.8% / 31.5% (same, but ALSO works at w=0.4 where (a) is exactly 0) | No | **best candidate** |
| (c2) stage-2 only (drop per-joint floor) | 60% / 60% (best transmission) | **Yes** -- tripped at nearly every weight under `multiaxis_fast` | reject, unsafe |
| (d) single deadband after summation | 18.0% / 25.9% (worse than current) | No | more conservative, no benefit found |
| (e) continuous/soft-knee | 11.3% / 27.3% (worse than current) | No | smoother (no step) but weaker in this regime |

**(c1) strictly weakly dominates (a) at every weight tested** in this
data: identical output once past stage 2's old cliff, but ALSO recovers
signal at `w=0.4` (current gives exactly 0.0 there) with no observed
safety regression across any of the 3 push patterns. **(c2) has to be
rejected regardless of its transmission advantage** -- it's the only
variant that ever tripped the oscillation latch, and it did so
specifically under the pattern that matches the real historical trip,
strong circumstantial evidence the per-joint stage-1 floor (which c2
removes) is doing real anti-oscillation work, not just noise rejection.

**4. C1/C4 shoulder-mask question, evaluated with corrected per-arm
data:** unmasking C1's shoulder activates the relay earlier (works even
at `w=0.4`, where masked C1 is exactly 0) and gives more signal at every
weight -- but reaches 108-188% of shoulder's torque budget in the
`multiaxis_fast` pattern at higher weights (the real Cartesian clamp
would catch this before the device itself sees an over-torque, but it
means the relay is regularly asking for more than shoulder's ceiling
allows). This is exactly the mechanism the existing mask was built to
avoid. C4 (master/left-arm source) shows much less difference between
masked and unmasked in this data -- consistent with the "known-bad"
identification finding being specifically about the RIGHT arm's shoulder,
not a general Phantom Omni property -- but C4-unmasked still reaches
112-159% of budget under aggressive multi-axis pushing at higher weight,
so the same caution applies if C4's weight is ever raised much past where
it sits today.

**Caveat, same as before:** the oscillation-latch replay reuses the real
`analyze_window()` function but reconstructs the surrounding window/
decision scaffolding offline, against reconstructed rather than live
signal -- it does not experience real message timing/DDS jitter. A clean
result here is a reassuring screen, not proof of real-hardware safety.

**Recommendation:** (c1) stage-1-only is the strongest evidence-based
candidate found -- same safety profile as the shipped system in every
test run, structurally recovers the specific dead zone this whole
investigation was about, and doesn't require guessing at a new weight.
Raising `w1`/`w4` alone (variant b) also works but is strictly dominated
by (c1) at matched transmission levels (see the table). **No production
change made** -- this is characterization only, per the task's explicit
constraint.

Full data: `/tmp/deadband_variant_results.json` (300 rows), raw captures
in `/tmp/deadband_capture/*.json`, comparison plot
`/tmp/deadband_variant_pareto.png`. New scripts:
`sim/tests/diag_deadband_capture.py`, `sim/tests/diag_deadband_variants.py`,
`sim/tests/test_applied_force_reflects_relay.py`.

---

## Separate from the channel work: teleop quality

These are ordinary tuning issues, not blockers, but they affect every test.

- [x] Diagnosed slave oscillation: damping 0.20 N*s/m is ~8% of critical,
      giving a ~4.4 Hz resonator with ~78% overshoot
- [ ] Raise `DAMPING_N_S_PER_M` (0.75 -> 1.25 -> 1.76 for zeta 0.3/0.5/0.7)
- [x] Diagnosed Y under-travel: no gravity compensation on a vertical axis
- [ ] Tune `SLAVE_GRAVITY_COMP_N` (0.15 -> 0.30); currently 0.60 -> 0.80 of
      commanded travel. Check the bridge's `reach mm` line for `SHORT`
      (force-limited) versus `@LIMIT` (our clamp is the ceiling).

---

---

## Real-vs-visual (simulated) encoder harness — added 2026-08-26, not yet run on hardware

A hardware-independent way to characterize the same noise floor Phase 2
hit, using the digital twin in `../../../ros2_ws/src/touch_haptics/sim/`
instead of a second real device. Built and smoke-tested (two
`touch_slave_sim` instances, synthetic driven target, recorded bag, full
plot set rendered) while away from the physical arms; **not yet exercised
against a real arm**, since that requires hardware presence.

- [x] `scripts/run_master_left_sim.sh` / `run_slave_right_sim.sh` — a
      `touch_slave_sim` "visual" twin per arm, subscribed to the SAME
      `target_pose`/`force_cmd`/`enable` topics the real node receives,
      outputs remapped to `/touch_{left,right}_sim/...` so they never collide
      with the real node's own topics on the bus.
- [x] `scripts/record_real_vs_sim_bag.sh` — records real + visual topics for
      both arms plus `/contact_annotation`, into `~/dob_logs/real_vs_sim_*`.
- [x] `../../../ros2_ws/src/touch_haptics/identification/plot_real_vs_sim_encoders.py`
      — reads one such bag; for the master, plots position (3D + XYZ), force
      (3D + XYZ), and a combined per-axis noise panel (`real - visual`
      position and force, overlaid); for the slave, position and force
      (3D + XYZ). Real and visual traces overlaid on every panel.

**What this is and is not.** The real-vs-visual residual is raw noise —
friction, gravity-model error, base tilt, and genuine contact all still
mixed together, deliberately not separated at this stage. Separating it
(gravity/friction/environmental) is the next step, reusing `fit_gravity.py`
/ `fit_friction.py` / `analyze_contact_separability.py` against the residual
instead of raw `tau_dis_hat` — see the plan this section was added from,
`/home/nural/.claude/plans/now-i-want-you-enchanted-waterfall.md`, Phase 2.
It does not resolve the Phase 2 STOP by itself, and it is not a substitute
for real force/torque sensing (still an open item, see
`FINDINGS_AND_CONTEXT.md` §6.1).

**Next, once back at the hardware:** run `run_master_left.sh` +
`run_master_left_sim.sh` together (same for the right arm), record a session
with `record_real_vs_sim_bag.sh` covering both free motion and annotated
contact, then run the plotting script on it. Compare the resulting noise
plots' magnitude and structure against the 2026-08-26 separability numbers
before deciding whether to retune `threshold_nm`/`g_dis` or wire in a
friction model first (the latter is what the existing diagnosis points to).

---

## Phase 2.5 addendum — plotting-script audit, 2026-08-26

`plot_real_vs_sim_encoders.py` was audited line-by-line against four observed
plotting anomalies. This had no prior standing document -- recorded here now
so it isn't conversation-only. Findings:

- **Independent zero-referencing is correct.** Real and visual position are
  each subtracted from their OWN first sample (`:384-385`, `:392-400`); no
  shared reference point, unlike the master-force case below.
- **Peak-force bars saturating at the 1.5N clamp is a genuine data finding,
  not a script bug.** Both traces use the identical `max(||F(t)||)` formula
  (`:324-331`); real and sim independently hitting the same clamp is what
  produces near-identical bars.
- **`_interp_to`'s `numpy.interp` silently holds a flat boundary value** for
  timestamps outside a stream's recorded range (`:358-368`) rather than
  flagging missing data -- a latent risk at the start/end of any noise-panel
  recording, not exercised by the sessions analysed so far.
- **No slave noise panel currently exists.** `_plot_noise` only runs
  `if is_master` (`:447`); any "slave noise panel" observation does not come
  from this script as written.
- Master force matching almost exactly, traced to the same `target_fresh`
  control-law fact recorded under C4 above -- not a plotting artifact.
- Slave Y-axis sign inversion during one press window, confirmed via bag
  timestamps to be a direct, correctly-computed consequence of real and sim
  sitting on opposite sides of the same commanded target (both use the
  identical `force_ros = stiffness*err - damping*vel` formula) -- not a sign
  convention bug in the script, the control law, or `hd_sim`.

## Phase 2.6 — Friction compensation in the live observer ✅ COMPLETE (left arm)

- [x] **`JointDobParams` extended** with `fric_coulomb_nm`, `b_visc_nm_s_per_rad`,
      `vel_thresh_rad_s` (`touch_dob.hpp`). Deliberately OPTIONAL, unlike the
      gravity model -- zero is a valid "no compensation configured" default,
      so an unloaded/right-arm config degrades gracefully to the pre-2.6
      behaviour rather than refusing to start.
- [x] **Loaded from `inertia_left.json`'s `b_visc`/`c_coulomb_nm`**
      (`dob_json_loader.hpp`), deliberately NOT from `friction_left.json`'s
      raw per-joint median -- that file's own waist value is negative
      (-0.0058 N·m as of the 2026-08-24 fit) and non-physical per
      `identification/README.md`'s own caveat, and it carries no viscous
      term at all. `inertia_left.json`'s values are positive,
      well-conditioned, and cross-checked against the static sweep.
- [x] **Subtracted in `JointDob::update()`** alongside `G(q)`, same
      justification (modelled term, not disturbance): `tau_fric =
      fric_coulomb*tanh(qdot/vel_thresh) + b_visc*qdot`, using the same
      previous-frame `qdot_rad_s` already aligned for the gravity term -- no
      new required call-site argument.
- [x] **`test_touch_dob`: 56/56 pass, 0 regressions** (run-verified, not
      source-counted).
- [~] **`sim/tests/` pytest suite: 1 new failure, explained, not a bug.**
      `test_dob_floor.py`'s waist case now reads 4.17x its hardware-floor
      reference (band is [0.15, 2.5]) -- isolated via an A/B rebuild
      (friction loading temporarily disabled, confirming the other 7
      failures in that suite are pre-existing and unrelated) to be caused
      specifically by this change. Root cause: the simulator's own friction
      model is DELIBERATELY mismatched from the identified values (see
      `arm_model.hpp`'s header note) and its waist viscous coefficient in
      particular is nowhere near the real 0.0637 N·m·s/rad now being
      subtracted -- so compensating for real friction on simulated data
      over-subtracts. **Real-hardware validation instead**, via
      `dob_replay_cpp.cpp` replayed over the actual `~/dob_logs/dyn_Left.csv`
      capture (with/without friction terms, same everything else): floor
      dropped on all three joints at the shipped `g_dis`/`f_max=0.6N` config
      -- waist 24%->16% of torque authority, shoulder 40%->29%, elbow
      31%->29% (-32%/-25%/-23% relative). The sim test's expected band is
      now stale for a friction-compensated config; updating it is a policy
      decision for the operator, not made here.
- [ ] **Not done by this phase:** re-running Phase 2's actual separability
      test against the now-compensated floor (Phase 2.7, still not started);
      the right arm has no friction model to load at all (Phase 4).

---

## 2026-08-26 late session — Y/Z/X working volume + unresolved axis-mapping bug

**Y/Z gravity-comp fix, confirmed working.** Reach ratio at commanded
100mm went 68.8mm (0.69, `SHORT`) -> 85.2mm (0.85, `OK`) via
`SLAVE_GRAVITY_COMP_N` 0.0 -> 0.55 (chosen from a controlled sweep, values
0.15/0.25/0.35/0.45/0.55/0.65 tested; 0.55 was the plateau -- pushing to
0.65 gave diminishing/mixed returns). X held flat (no regression), Z
improved too (0.66->0.85 reached) despite the bias being Y-only, consistent
with this rig's already-documented base-tilt/non-level-base coupling, not
something this fix specifically targeted. Change is live in
`run_bridge.sh`'s default.

**`MAX_OFFSET_XYZ` is a separate ceiling from the force-budget issue,
confirmed via the bridge's own `reach mm` diagnostic (`@LIMIT` vs
`SHORT`).** After the gravity-comp fix, Y and Z started hitting
`MAX_OFFSET_XYZ` (110mm / 90mm) with otherwise-healthy reach ratios
(0.91-0.92) -- genuinely `@LIMIT`, not force-limited. Raised to 190mm (Y,
matched against a manual hand-reach test that found ~185mm of real
mechanical travel from the same anchor -- the only one of the two directly
hand-verified) and 150mm (Z, sized by applying Y's same relative increase,
1.73x -- **not** independently hand-verified). **Flagged as a decision
point, not a settled number:** both values were sized from whatever the
operator happened to reach during ad hoc testing, not a stated task
requirement for how far the slave actually needs to travel.

**X-axis `SHORT` at small magnitude (29.7mm asked, 0.71 reached), observed
once, not yet explained.** Later readings at larger X magnitudes (50.7mm,
65.7mm) showed healthy reach (0.81-0.85, no flag) -- looks pose-specific or
transient rather than a standing X-axis problem the way Y was, but was
never isolated with a controlled sweep the way Y got. **Open, low
confidence, needs its own controlled reach test.**

**FK/IK reachability gap (Phase 2.5's ~90-100mm figure): mechanism now
identified.** Computed FK from each device's own live joint angles and
compared to that device's own reported Cartesian pose (`HD_CURRENT_TRANSFORM`,
not the simplified model): X matches exactly (0.0mm) on both arms, but Y is
off by exactly -23.35mm and Z by exactly +168.35mm -- **identical on both
the left and right arm**, despite very different joint angles at the time
of the two readings. An error that is bit-identical across two different
poses and two different physical units is a missing additive constant, not
a joint-limit or per-unit calibration issue. The device configs contain
`WorkspaceOffset1=-110mm`, `WorkspaceOffset2=-35mm`
(`ros2_ws/config/{Left,Default Device}.config`) that `touch_dob.hpp`'s
`fkFromJacobianConvention()` never applies. This also answers whether the
Phase 0 FK/Jacobian verification (RMS 1.674mm) was ever done on the right
arm: Phase 4 already lists that check as not started for the right arm, and
this finding is independent confirmation the *left* arm's own verification
route (`hdapi_fk_residual`) didn't catch this failure mode (it presumably
checks relative/RMS displacement, not a missing absolute offset). **Not
fixed** -- out of scope by explicit instruction (this formula also feeds
the DOB and the identification pipeline; changing it needs its own,
separate task) -- this entry only records the finding.

**🔴 UNRESOLVED, discovered live, blocks fully trusting "C3 works."**
Operator reported the master->slave position mapping producing incorrect
motion -- most recently described as "no discernible pattern," and after an
axis-swap attempt, "z and y are wrong," and after a further sign-flip
attempt, still "not correct." Investigated and ruled out, with evidence:

- The bridge's `axis_map` is a pure per-axis permutation+sign+scale
  (`apply_axis_map`, `bridge_master_slave.py`) -- no rotation, no
  quaternion, no cross-axis term anywhere in the code. Confirmed by direct
  reading, not inference.
- Both devices' launch parameters (`invert_x/y/z`, `swap_xy`) are `false`
  on both master and slave (master doesn't even pass them, falling back to
  the same `false` compiled default) -- no device-level axis remap on
  either side.
- `touch_slave.cpp`'s `targetPoseCallback` takes the incoming target x/y/z
  directly with no remap, and the control law (`force_ros[i] =
  stiffness*err[i] - damping*vel[i]`) is diagonal -- each output axis
  depends on exactly one input axis. **True cross-talk (one master axis
  moving multiple slave axes) is not mathematically possible given this
  code.**
- The FK-offset bug above was checked and ruled out as the cause -- the
  live control path never calls that formula; position comes straight from
  `HD_CURRENT_TRANSFORM` via the driver.
- **What was tried live, without full resolution:** `axis_map` changed from
  default `x,y,z` -> `x,z,y` (Y/Z swap) -> `x,z,-y` (added a sign flip on
  the new Z-sourced component). Each change was reported as incomplete or
  still wrong. Three attempts to capture clean single-axis-motion data
  (`/touch_left/pose` vs `/touch_right/target_pose` vs `/touch_right/pose`,
  logged simultaneously) all show **zero recorded motion** on every topic
  across 15s, 18s, and 90s windows -- meaning the diagnostic was never
  actually completed with real motion data, not that the mapping was
  tested and found fully broken by a clean measurement. **The
  currently-live `axis_map=x,z,-y` is an unconfirmed guess, not a
  validated fix.**
- **Next session must start here**, with a real single-axis-motion capture
  before trying another blind permutation.

### 2026-09-01 — axis-mapping bug resolved: RESOLVED, default `axis_map=x,y,z` is correct

Did the real single-axis-motion capture this section called for, on live
hardware, using the bridge's own periodic per-axis reach report
(`--report`, already built in) instead of the raw simultaneous-topic-echo
approach that produced zero usable data three times previously. **Reverted
`AXIS_MAP` to the default identity (`x,y,z`) for this** -- the `x,z,-y`
override was never validated and this investigation is what replaces it.

Five separate restart-and-move tests (peak-tracking reset each time):

| test | dominant axis (peak asked) | secondary axes | wrong-axis / sign-flip? |
|---|---|---|---|
| Y, attempt 1 | Y 156.3mm | X 12.4mm, Z 15.8mm | none |
| Y, attempt 2 | Y 128.8mm | X 8.3mm, Z 31.3mm | none |
| X | X 50.0mm | Y 7.8mm, Z absent (<5mm) | none |
| Z, attempt 1 | tied Y 103.3mm / Z 90.6mm | X 17.6mm | none |
| Z, attempt 2 | Z 81.5mm | Y 59.8mm, X 10.8mm | none |

In every test, the intended axis was also the dominant response axis, in
the matching sign/direction (checked explicitly each time: e.g. Z target
moved from anchor `-0.0466` to a peak of `+0.0346`, a clean +81mm positive
excursion matching a deliberate positive-direction push). **No test, across
five tries, ever showed a wrong destination axis or an inverted sign.**

The residual cross-axis contamination (typically 10-30% of the dominant
motion, occasionally comparable to it on the harder-to-isolate Z tests) is
concluded to be human hand-motion imprecision, not a software defect --
this device's geometry makes a genuinely pure push/pull (Z) motion hard to
execute without some incidental vertical (Y) drift, and the contamination
shrank as technique improved between the two Z attempts. A real `axis_map`
software bug would produce a *consistent* error independent of operator
technique; what was observed is the opposite. This is also consistent with
the code-level proof already on record above (diagonal control law, pure
permutation `axis_map`, no rotation/quaternion anywhere) -- true
software-level cross-talk was never mathematically possible in the first
place, so the live symptom must have come from either operator technique or
(most likely, in hindsight) the unvalidated `x,z,-y` override itself
actively producing the swapped/wrong-feeling motion it was meant to fix.

One unexplained but minor pattern, not yet investigated further: the X axis
showed `got > asked` (overshoot beyond the commanded offset) in 3 of 4 tests
where it had a nonzero reading (ratios 1.17, 1.86->1.35, 1.71, 1.41->1.30).
Not alarming (plausibly spring-dynamics overshoot on a lightly-loaded axis)
but noted here in case it recurs somewhere it matters more.

**Action taken:** `run_bridge.sh`'s `AXIS_MAP` default (`x,y,z`) requires no
change -- it was already the default; the fix is simply *not* overriding it
with `x,z,-y` anymore. **C3 is now considered confirmed-working.**

---

## Known open issues carried forward

| issue | impact |
|---|---|
| Waist gravity unmodelled (base tilt) | waist `tau_dis_hat` carries a pose-dependent bias up to 0.018 N*m |
| Floor is 40-57% of torque authority | narrow band left for contact; the Phase 2 question. Reduced to 16-29% (left arm, shipped config) by Phase 2.6's friction compensation, but Phase 2's actual separability test has not been re-run against this (Phase 2.7) -- do not assume the STOP verdict has changed until it is |
| No thermal feedback | `HD_MOTOR_TEMPERATURE` is dead; duty cycle is unguarded |
| Gravity map extrapolates below q2 = 0 | idle poses already sit outside its fitted range |
| Shoulder gate trips on gravity alone at some poses | idle `-G(q)` is within 0.04% of `threshold_nm` |
| `threshold_nm` derived from floor, never from known contact | Phase 2 is the first real test of it |

---

## Roadmap: sequenced path to 4-channel

Dependency-ordered, not a flat list. Each item states what it's blocked on
and what it unblocks. Corrected against what this session's audit actually
found -- see the rest of this document for evidence, not just assertion.

### Immediate, no dependencies (can start in any order)

1. ~~Resolve the axis-mapping bug~~ -- **done 2026-09-01**, see the
   resolution section above. C3 confirmed working under default
   `axis_map=x,y,z`.
2. **Diagnose the X-axis `SHORT` finding** -- controlled reach sweep like
   Y got (see "teleop quality" and the late-session section above),
   isolate whether it's pose-specific/transient or a standing issue.
3. **Decision needed (operator): `MAX_OFFSET_XYZ`'s real target values.**
   Currently 190mm(Y)/150mm(Z), sized from ad hoc test motion during
   diagnosis, not a stated reach requirement. Y was hand-verified against
   a real manual-reach test (~185mm); Z was not (estimated by scaling).
4. ~~Determine FK/IK reachability gap mechanism~~ -- **done this session.**
   Confirmed missing `WorkspaceOffset` constant in `touch_dob.hpp`'s FK,
   identical on both arms, not a joint-limit misconfiguration. Not fixed
   (`touch_dob.hpp` edits are out of scope for diagnostic tasks), but no
   longer an open diagnostic question.

### Blocking Phase 2.7

5. **Phase 2.6 friction model** -- already done for the left arm this
   session, confirmed via 56/56 unit tests plus a real offline-replay
   validation showing a 23-32% floor reduction on real hardware data. Not
   yet done for the right arm (no friction JSON exists for it -- depends
   on Phase 4).

### The hard gate -- nothing past this point should be scheduled until it resolves

6. **Phase 2.7** -- re-run the REAL hardware separability test (not the
   sim comparison Phase 2.5 did) using `contact_annotate.py` +
   `analyze_contact_separability.py`, now that Phase 2.6's friction term is
   live. Apply the pre-committed >=2-3x proceed / 1.5-2x judgment /
   ~1x-or-below stop bar exactly as written, no loosening. **If it fails
   again: Phases 3 onward are not worth pursuing on this hardware, and
   position-only teleop (the current, working C3, once the axis-mapping
   bug above is actually fixed) is the recommended end state for this
   rig.**

### Only if Phase 2.7 passes

7. **Phase 3, completed not restarted** -- remaining axes/poses for
   left-arm torque calibration (currently one axis, one pose only), a real
   right-arm calibration attempt (the last one produced zero data rows --
   find out why before re-running blindly), per-joint torque clamp
   derivation.
8. **Phase 4** -- full right-arm identification campaign (FK check, static
   sweep, dynamic excitation, three fits). Nothing here can be reused from
   the left arm. ~2 hardware sessions per the existing estimate.
9. **Phase 5** -- base leveling, gravity-map coverage widening (friction
   model itself is now done per item 5 above).
10. **Phase 6, one channel at a time, each independently validated:**
    - **C4 first** -- confirmed the smallest real gap: `bilateral_4ch.py`
      genuinely exists (487 lines, weighted C1-C4 mixing, energy tank,
      oscillation-abort latch, weight-ramping, preset architecture
      `unilateral/pp/pf/4ch_safe/4ch` -- verified present, not assumed) and
      the observation-side wiring (`force_ext_hat`, `joint_torque_ext_cmd`)
      already exists end to end. The actual blocker is a launch-config
      decision (`joint_space_control:=true`) plus the compile-time gate --
      scope the first test as sim-only (`touch_slave_sim`, where the write
      path is actually reachable) at a low ceiling before any
      real-hardware torque write.
    - **C2** -- genuinely unbuilt (not gated, not present in any form):
      needs a left-side target publisher added to `bridge_master_slave.py`,
      which currently has exactly one target publisher (right-side only).
    - **C1** -- needs Phase 4 (right-arm identification) complete first,
      since it needs a real observer on the slave, which needs the
      slave's own `M_n`/`G(q)`/friction fit.
    - **Full 4-channel integration** -- `bilateral_4ch.py`'s `4ch_safe`
      preset (`C1=0.5, C2=1.0, C3=1.0, C4=0.3`) as the ramp-up starting
      point, already implemented; stability validation (passivity,
      transparency, limit-cycle check) and human-in-the-loop retuning
      after.
