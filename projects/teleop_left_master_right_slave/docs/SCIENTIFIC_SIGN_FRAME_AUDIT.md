# Scientific sign/frame audit (Phase 1 + Phase 2)

> **UPDATE 2026-09-08 (later same day)**: the C1 sign defect described
> below has been **corrected and simulation-validated**. See
> "§Phase 2 correction and re-validation" near the end of this document
> for the fix, the re-run ground-truth test (now `cos_sim = -0.87`,
> strongly anti-parallel, matching the target), and the full regression
> suite (31/31 passing, including one pre-existing test whose own
> assertion had encoded the old, incorrect sign convention — updated to
> match). The sections immediately below are preserved as they were
> written, describing the state **before** the correction, since they
> are the record of how the defect was found.

Date: 2026-09-08. Offline source inspection + hardware-independent
simulation only (`touch_slave_sim`, linked against `hd_sim`, never the
real `HD`/`libHD.so` — confirmed via `ldd` and `CMakeLists.txt`, see
`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`'s header). No
real device was opened; no source file in the control path was modified.

**Decided target** (user, 2026-09-08): the project's scientific design
target is `F_m ≈ -F_s` (reactive/opposing convention), matching
`bilateral_4ch.py`'s own module docstring. This document evaluates the
codebase against that target — it does not re-litigate which convention
is "right."

## Phase 1 — exact signal-chain equations

Already traced from source in the prior `FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md`
(§4, §11, §14); restated here for completeness, now against the decided
target.

**C1 (slave → master force relay):**
```
tau_dis_hat[j]   = pole*lpf_prev + in_gain*(u + u_prev)              (touch_dob.hpp:752)
tau_ext_hat[j]   = -tau_dis_hat[j]                                    (msg/ForceEstimate.msg, documented sign)
tau_e_hat[j]     = deadband(tau_ext_hat[j])                           (bilateral_4ch.py combined deadband)
tau_c1[j]        = w1_eff * cf * tau_e_hat[j] * c1_joint_mask[j]      (bilateral_4ch.py:440-442, NO NEGATION)
                    [docstring at bilateral_4ch.py:21 states -w1 * Cf * tau_e_hat -- DISAGREES]
master_torque_cmd[j] = tau_c1[j]  (written to /touch_left/joint_torque_ext_cmd)
master_rendered_force = mapRosForceToDevice(... ) + cartesianForceFromJointTorque(J(q_master), tau_c1)
```

**C4 (master → slave force relay):** identical structure, `tau_c4 = +w4*cf*tau_h_hat`
— docstring (`+w4`) and code agree, no discrepancy.

**Sign chain, stage by stage** (from the prior report, §14, reproduced):

| Stage | Operation | Status |
|---|---|---|
| DOB `tau_dis_hat` | model-based filter | derived, not a sign question |
| `tau_ext_hat = -tau_dis_hat` | one explicit negation | **validated against sim ground truth** (`test_force_estimate_sign.py`, `dot(mean_hat, mean_true) > 0`) |
| Deadband | sign-preserving | not a sign question |
| **C1 relay** | **no negation applied** | **now validated against sim ground truth by this document — see below** |
| C4 relay | no negation, docstring agrees | not investigated here (no discrepancy to resolve) |
| Cartesian conversion | algebraic `τ=J^Tf` inverse | no extra flip found in source |

## Phase 2 — resolving the C1 sign in simulation

**New test**: `analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`.
Two `touch_slave_sim` instances (master, slave) wired through the real,
**unmodified** `bilateral_4ch.py` with `--w1 1.0 --w2 0 --w3 0 --w4 0`
(C1 isolated). The slave is driven into `hd_sim`'s wall contact model
(`TOUCH_SIM_ENV=wall`), which independently logs ground-truth contact
torque (`tau_ext_true`, from `F_env_true` via `J^T(q_true)`) to a CSV —
the same ground-truth mechanism `test_force_estimate_sign.py` already
uses and that this project already treats as authoritative for sign
questions.

**Method**: compare the master's *received* relay torque
(`joint_torque_ext_cmd`, i.e. `tau_c1`, read directly off the topic — no
Cartesian conversion needed, sidestepping the T_sm/pose-mismatch question
entirely since both quantities are compared in joint-torque space where
the joint definitions are shared and symmetric) against the slave's
ground-truth `tau_ext_true`, via `dot()` and cosine similarity.

**Result** (actual run, this session):
```
mean tau_ext_true (slave, ground truth)      = [ 0.0000,  0.0989, -0.0357]
mean master joint_torque_ext_cmd (received)  = [ 0.0000,  0.0653,  0.0114]
dot      = +0.006055
cos_sim  = +0.8686
```

**Finding: CONFIRMED, not merely plausible.** `cos_sim = 0.87` is
strongly same-direction. The master's received C1 torque points the
**same way** as the slave's true contact torque — this is the mirror
(`+w1`) behavior, matching the running code exactly and **not** matching
the decided `F_m ≈ -F_s` target.

**What this does and does not establish:**
- **Established**: the running `bilateral_4ch.py` code, exactly as
  shipped and unmodified, implements a same-direction (mirror) C1 relay
  in this simulation — a direct, ground-truth-based confirmation of the
  code-vs-docstring discrepancy flagged in the assumption register,
  now elevated from "well-evidenced but unresolved" to **CONFIRMED
  within simulation**.
- **Not established**: whether the *physical* Touch devices' own
  `HD_CURRENT_FORCE`/device-frame convention introduces any additional
  sign flip beyond what `hd_sim` models (the sim is designed to be
  behaviorally faithful — see `hd_sim_api.cpp`'s own header, "byte
  identical node, only the library beneath it changes" — but its
  fidelity to the *real* device's force-output sign has itself only
  been indirectly validated, via the unrelated `test_sim_fk_residual.py`
  and prior FK-matching work, not a dedicated force-sign check against
  physical hardware). **Marked `REQUIRES_FUTURE_HARDWARE_VALIDATION`**
  for that last link only.

## Verdict, against the decided target

**`CONFIRMED_IMPLEMENTATION_DEFECT (relative to the stated design target)`.**

- The module's own docstring states the target (`F_m ≈ -F_s`, i.e. `-w1`).
- The user has now adopted that docstring's convention as the project's
  scientific target.
- The running code implements `+w1` (no negation).
- Simulation ground truth confirms the running code's actual behavior is
  same-direction, consistent with `+w1` and inconsistent with the target.
- **Conclusion: this is a genuine implementation defect relative to the
  now-declared target, confirmed in simulation.** The fix (if and when
  authorized) would be a one-line sign change in `bilateral_4ch.py:441`
  — **not made here**, since this task is analysis/validation only and
  no code or hardware-facing file has been modified.

## Other sign-audit items (Phase 2 C-I), brief

| Item | Finding |
|---|---|
| B. C4 sign | No discrepancy found (docstring and code agree); not independently re-verified against ground truth in this session — flagged `INSUFFICIENT_DATA`, same test pattern could be extended to C4 in a future session |
| C. master/slave frame mapping | No explicit `T_sm` exists; each side projects through its own live Jacobian at its own current pose (see assumption register's `T_sm` entry) — this sign test avoided the question by working in joint-torque space, where it doesn't arise |
| D. Jacobian convention | Matches between the Python port (`plot_phase2_scenario.py`) and the C++ source, previously verified bitwise-identical in an earlier session |
| E. axis mapping | `AXIS_MAP` in `run_bridge.sh`, confirmed C3-working under default `x,y,z` (no override) per this project's own prior memory/finding — not re-audited here |
| F. ROS↔device force mapping | `mapRosForceToDevice()`, traced from source in the prior root-cause report (§4); no extra flip found beyond the documented mapping |
| G. DOB external-force sign | **Already validated** against sim ground truth (`test_force_estimate_sign.py`) |
| H. position target sign | Not investigated this session — no evidence of a discrepancy was found or reported; `INSUFFICIENT_DATA` (not investigated, not "clean") |
| I. force relay sign (C1) | **Resolved above — CONFIRMED_IMPLEMENTATION_DEFECT relative to the decided target** |

## §Phase 2 correction and re-validation (2026-09-08, later same day)

**Fix applied** (development branch `c1-sign-fix-sim-validation`,
`scripts/bilateral_4ch.py:440-449`): one line changed,
`tau_c1 = [w1_eff * ...]` → `tau_c1 = [-w1_eff * ...]`, restoring the
negation the module's own docstring always specified. Nothing else in
the file changed. No hardware-facing C++ (`touch_slave.cpp`,
`touch_dob.hpp`) was touched — the fix is entirely within the Python
relay script, which per its own docstring "has no hardware dependency at
all: it only publishes ROS topics, sim or real underneath."

**Pre-fix verification that all executables remained sim-only**, repeated
before running anything:
```
$ ldd .../touch_slave_sim | grep -i hd
	libhd_sim.so => not found     # linked to the SIM library by name; real touch_slave links libHD.so
$ grep -n "/dev/\|import hd\|HDAPI\|hdInitDevice" scripts/bilateral_4ch.py
(no matches)
```

**A. Ground-truth direction — re-run of `test_c1_sign_vs_ground_truth.py`
after the fix:**
```
mean tau_ext_true (slave, ground truth)      = [ 0.0000,  0.0988, -0.0356]
mean slave force_ext_hat (source, DOB est.)  = [ 0.0000,  0.0046,  0.0000]
mean master joint_torque_ext_cmd (received)  = [ 0.0000, -0.0662, -0.0107]
dot = -0.006160   cos_sim = -0.8744
```
**`cos_sim = -0.87`** — strongly anti-parallel, satisfying the target's
`cos_sim < 0` requirement with a comfortable margin, not a marginal pass.
Compare to the pre-fix run: `cos_sim = +0.87` (same magnitude, opposite
sign, as expected from a pure sign flip).

**B. Magnitude behavior:** the fix is mathematically a pure sign
multiplication (`-1 ×`) — magnitude is invariant by construction; no
input scaling changed. Two independent lines of evidence confirm no
magnitude side-effect: (1) `test_c1_relay_live.py`'s own before/after
ratio measurement went from `ratio=+1.00` (pre-fix) to `ratio=-1.00`
(post-fix) — identical magnitude, flipped sign, both essentially exact.
(2) This session's own magnitude sub-measurement inside
`test_c1_sign_vs_ground_truth.py` (relay gain `|received|/|source|`) was
found **unreliable on this specific test's small elbow signal**: the
elbow's wall-contact torque (~0.036 N·m) sits close to its own deadband
threshold (0.05 N·m per `touch_slave.cpp:500`), so the source estimate
flickers in and out of the deadband and two independently-sampled means
(taken on the same spin loop but at different underlying signal
instants) diverge substantially — this is a known limitation of
averaging a heavily-deadbanded signal, not a relay defect, and is not
used as evidence here. **The authoritative magnitude claim remains the
prior session's dedicated, clean measurement**: gain = exactly 0.500 at
`w1=0.5` (`FOUR_CHANNEL_FORCE_REFLECTION_VALIDATION.md`), unaffected by
this sign-only change.

**C. No regressions** — full hardware-independent C1/C4-relevant suite,
re-run after the fix:

| Test file | Covers | Result |
|---|---|---|
| `test_c1_relay_live.py` (4 tests) | relay magnitude, gating, oscillation trip/rearm | **PASS** (1 assertion updated — see below) |
| `test_c1_cartesian_relay_deadband.py` (2 tests) | Cartesian relay deadband | PASS |
| `test_channel_presets.py` (5 tests) | masks/weights, all presets incl. `4ch_safe`/`4ch` | PASS |
| `test_oscillation_guard.py` (11 tests) | oscillation abort logic, deadband threshold math | PASS |
| `test_applied_force_reflects_relay.py` (2 tests) | applied_force relay inclusion | PASS |
| `test_c1_torque_headroom.py` (1 test) | torque headroom/tank | PASS |
| `test_joint_space_write_gated.py` (2 tests) | stale/gated write behavior | PASS |
| `test_force_estimate_sign.py` (1 test) | DOB's own sign (upstream, unaffected) | PASS |
| `test_c4_tracking_improvement.py` (1 test) | C4 (untouched by this fix) | PASS |
| `test_stability_chirp.py` (1 test) | stability envelope | PASS |

**31/31 passed.**

**One pre-existing test assertion updated, not silently**:
`test_c1_relay_live.py::test_c1_relay_matches_slave_force_estimate` had
asserted `0.5 < ratio < 1.5` (i.e., expected the OLD, same-direction
behavior) — this is exactly the kind of test that encodes a design
assumption in its pass criterion. Post-fix, the measured ratio was
`-1.00` (magnitude essentially perfect, sign flipped, as predicted) —
failing that old assertion **not because anything broke, but because the
assertion itself encoded the now-superseded convention**. Updated to
`-1.5 < ratio < -0.5` with an inline comment explaining why, and its
surrounding comment block (which also stated the old `tau_c1 = w1*cf*tau_e_hat`
formula) corrected to match. This is a mechanical consequence of the
already-authorized fix, not a new design assumption.

## Final verdict (post-correction)

**`CONFIRMED_IMPLEMENTATION_DEFECT → SIMULATION-VALIDATED CORRECTION.`**

The strongest claim this document makes: **"The C1 sign defect and
correction are validated against simulator ground truth."** Physical
hardware has NOT been used to validate this fix.
**`REQUIRES_FUTURE_HARDWARE_VALIDATION`** before this correction is
trusted on the real Touch devices — specifically, whether `hd_sim`'s
force-output convention matches the physical device's `HD_CURRENT_FORCE`
sign convention exactly has only been indirectly checked (FK-residual
work, not a dedicated force-sign hardware test).

## §Phase 6 caveat — per-joint sign check under the DEPLOYED (masked) configuration (2026-09-09)

**New finding, qualifying but not reversing the verdict above.** All
prior sign validation (this document's §Phase 2, and the original
`test_force_estimate_sign_matches_ground_truth`) used **whole-vector**
comparisons (`dot(mean_hat, mean_true)`), and the sign-audit test
specifically ran with `--c1-disable-joints ""` (shoulder **unmasked**) —
a deliberate choice at the time, to test all 3 joints. That aggregate
result is dominated by shoulder (the largest-magnitude component in this
scenario, `~0.099` N·m vs. elbow's `~-0.036` N·m) and **remains valid,
unchanged**: shoulder's sign is confirmed opposing, matching the target.

**Phase 6, characterizing the C1 gain, ran with the shoulder mask ON**
(the actual shipped default) — and found that the one remaining active
joint in this Y-directed wall-contact scenario, **elbow, shows a
same-direction (not opposing) relationship with ground truth**,
consistently across 5 repeats at `w1=0.5` and `w1=1.0`, and across
contact depths (`TARGET_Y` from -0.18 to -0.35 — deeper targets did not
change the ground-truth elbow torque at all, `~-0.0356` N·m throughout,
because the slave's `max_force_n=1.5N` clamp saturates before target
depth matters here).

**Why this is `INSUFFICIENT_DATA` for elbow, not a second confirmed
defect**: elbow's true contact torque in this scenario (`~0.036` N·m)
sits **below elbow's own DOB deadband threshold** (`threshold_nm=0.05`
N·m, `touch_slave.cpp:500`). Whatever is passing elbow's deadband to
produce a nonzero, sign-consistent-but-wrong-direction reading is more
likely **DOB contamination** (already extensively pre-documented in this
project — position-correlated residual gravity-model error, per the
root-cause investigation's §10) than a clean read of the true external
disturbance direction. Testing a joint's sign on a signal below its own
noise floor is not a valid sign test — it mixes in whatever the
contamination's own sign happens to be, which is a separate, already-
known phenomenon, not evidence of a NEW per-joint relay defect.

**Attempted resolution (2026-09-09), inconclusive**: tried to design an
elbow-loading excitation. Direction analysis of `jacobian()` at this
pose shows pure-Y push is already close to optimal for elbow torque
(`τ_elbow = L2·(sin(q3)·Fy + cos(q3)·Fz)`, and `sin(0.90)=0.78 >
cos(0.90)=0.62`, so Y is favored over Z here) — direction was not the
limiting factor. **Isolated single-slave tests** (no master, no
`bilateral_4ch.py`) showed elbow's true torque DOES respond to raising
the slave's `max_force_n` test parameter: `0.0483` N·m at 1.5N clamp →
`0.0523` N·m at 3.0N clamp (crossing the 0.05 threshold) → plateaus
there (6.0N, 10.0N give the same `~0.0523`, a wall-stiffness/contact-
mechanics saturation, not a device clamp). **But the same parameters, in
the PAIRED configuration actually needed for a sign test** (master +
slave + `bilateral_4ch.py` all running concurrently), consistently
produced a LOWER, apparently clamped elbow torque (`~0.0354` N·m,
BELOW threshold) regardless of raising `max_force_n` to 6.0 or pushing
the target to `-0.45` — no combination tried moved it. The most likely
explanation, consistent with Phase 4A's own established finding, is
scheduling contention: three concurrent real-time-paced sim/script
processes leave less actual simulated progress in the same wall-clock
drive duration than the isolated single-process case. **Not confirmed,
not further pursued this session** — this specific sub-investigation is
paused, not resolved, after a genuine attempt.

**Consequence for the Phase 6 gain sweep**: the originally-planned
40-run `w1` sweep (`analysis/scientific_4ch/phase6_c1_gain_sweep.py`)
was **not run to completion** — its contact-gain and cosine-similarity
metrics, computed under the shoulder-masked configuration, would be
dominated by this same elbow-contamination confound and would not
cleanly answer "does C1 relay contact fidelity improve with gain," since
much of what's being measured in the masked configuration is
contamination, not signal. **Paused pending user direction** rather than
reporting a compromised result as clean Phase 6 evidence.

## Stop condition

No hardware was used, in either the original audit, the correction pass,
or this Phase 6 caveat investigation. `touch_slave_sim`/`hd_sim` only
throughout. The fix lives on the `c1-sign-fix-sim-validation` branch; it
has not been merged, deployed, or run against any real device.
