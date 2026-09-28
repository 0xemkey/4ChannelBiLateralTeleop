# Authoritative sign/frame definitions (Scientific hardware code path)

Date: 2026-09-08. Single source of truth for every axis/sign convention
used in the hardware-facing code path (`touch_slave.cpp`,
`bilateral_4ch.py`). Produced by source inspection only; no hardware
accessed. Where this document and any other doc's prose disagree, THIS
document and the cited source lines are authoritative — other docs
should be read as historical traces, not living references (see the
update notes added to `CONTROL_ARCHITECTURE_MAPPING.md` and
`FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md` pointing back here).

## 1. ROS Cartesian axes vs device axes

`touch_slave.cpp` maintains one small, explicit, parameterized mapping
between "ROS Cartesian" (the frame every topic in this project — target
pose, applied_force, tracking_error — is expressed in) and "device frame"
(what HDAPI's position/force registers use), via 4 boolean parameters
(`touch_slave.cpp:266-269`, all default `false` — identity mapping unless
overridden per-launch):

```cpp
mapDeviceVectorToRos(v):            // device -> ROS  (positions/velocities)
    if swap_xy: swap(v.x, v.y)
    if invert_x: v.x = -v.x
    if invert_y: v.y = -v.y
    if invert_z: v.z = -v.z

mapRosForceToDevice(f):             // ROS -> device  (forces, about to be written)
    if invert_x: f.x = -f.x
    if invert_y: f.y = -f.y
    if invert_z: f.z = -f.z
    if swap_xy: swap(f.x, f.y)
```
Note the two functions are NOT literal inverses of each other in
operation order (swap-then-invert vs invert-then-swap) — this only
matters when BOTH `swap_xy` and an invert flag are set simultaneously;
with the shipped defaults (`invert_x=invert_y=invert_z=swap_xy=false`)
both are the identity and the ordering is moot. **Flagged, not fixed**:
if any launch ever sets both `swap_xy` and an invert flag together, this
asymmetry should be re-derived from first principles before trusting the
result — not investigated further here since no current launch
configuration exercises that combination (per
`docs/CONTROL_ARCHITECTURE_MAPPING.md` row #3: `axis_map`/`axis_scale`
are identity in the current launch config).

## 2. Master↔slave position mapping

Carried by C2 (slave position → master target) and C3 (master position →
slave target), both in `bridge_master_slave.py`/`run_bridge.sh`, NOT in
`touch_slave.cpp`. `AXIS_MAP`/`AXIS_SCALE` (`run_bridge.sh`) apply an
independent, separately-configured axis remap between the two arms'
Cartesian deltas — confirmed working under the DEFAULT `x,y,z` (identity)
mapping (`docs/CONTROL_ARCHITECTURE_MAPPING.md` row #3; per this
project's own memory, do not reintroduce a `x,z,-y` override without new
evidence). **This mapping is independent of, and not to be confused
with, §1's device-frame mapping** — §1 exists inside one arm's own node;
this one exists between the two arms' bridge script.

## 3. Force mapping (device write path)

Within one node (`touch_slave.cpp`), Cartesian force in ROS frame is
built (spring + damping + auxiliary/gravity, primary clamp), then mapped
to device frame via §1's `mapRosForceToDevice`, then (if C1/C4 active)
has the Jacobian-transpose-converted relay contribution added **in
device frame** (`force_device[i] += f_ext[i]`, `touch_slave.cpp:1290-1292`),
then final-clamped, then written via `hdSetDoublev(HD_CURRENT_FORCE, ...)`.
See `docs/FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md` §4 for the exact
line-numbered equation trace (unchanged by this session's work — only the
sign feeding INTO this pipeline, from bilateral_4ch.py, changed; nothing
inside `touch_slave.cpp`'s own force-build pipeline changed).

## 4. Joint torque sign

Joint order is fixed throughout the entire codebase (`touch_dob.hpp`,
`bilateral_4ch.py`, all `.msg` files): **`[waist, shoulder, elbow]`**,
indices 0/1/2. `tau_cmd` handed to the DOB is `J^T(q) * F_cmd` from the
**previous** frame's device-frame force (see `touch_slave.cpp`'s own note
on why `HD_LAST_JOINT_TORQUE` is not used — its raw device-unit scale has
never been calibrated). No separate joint-torque sign convention exists
beyond "whatever sign `J^T(q)*F` produces for the Jacobian in §7" — there
is no additional flip applied to `tau_cmd` itself.

## 5. DOB external torque sign

**Derivation** (`msg/ForceEstimate.msg`'s own header, matches
`touch_dob.hpp`'s Q-filter structure):
```
tau_dis_hat ~ Q(s) * [tau_fric - tau_ext]     (model)
  =>  tau_ext_hat = -tau_dis_hat
```
**Validated** against simulation ground truth
(`sim/tests/test_force_estimate_sign.py`, `dot(mean_hat, mean_true) > 0`,
currently passing). This is the one sign link in the whole chain that was
already both documented AND tested before this session; not touched.

## 6. C1 sign — CORRECTED 2026-09-08

**Target** (this project's declared scientific objective, user-decided
2026-09-08): `F_m ≈ -F_s` (opposing/reactive).

**Formula, current (corrected) code** (`bilateral_4ch.py:450`, branch
`c1-sign-fix-sim-validation`):
```python
tau_c1 = [-w1_eff * cf * tau_e_hat[j] * c1_joint_mask[j] for j in joints]
```
Matches the module's own docstring (`bilateral_4ch.py:21`,
`tau_m += -w1 * Cf * tau_e_hat`) exactly — this was the source of truth
the code disagreed with pre-fix; they now agree.

**Simulation validation**: `cos_sim = -0.87` against `hd_sim` wall-contact
ground truth (`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`).
`REQUIRES_FUTURE_HARDWARE_VALIDATION` for the physical-device force-output
convention — see `docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md` for full detail,
this section is the terse authoritative summary.

## 7. C4 sign — no discrepancy found

**Formula** (`bilateral_4ch.py:456`, unchanged):
```python
tau_c4 = [w4_eff * cf * tau_h_hat[j] for j in joints]     # no per-joint mask exists for C4
```
Matches its own docstring (`bilateral_4ch.py:22`, `tau_s += +w4 * Cf * tau_h_hat`)
— **same-direction is the DOCUMENTED, DECIDED convention for C4** (not
re-litigated this session; no evidence found suggesting C4 should instead
oppose, and the task's own instruction was explicit: "Do not change C4
sign unless a separate correctness issue is proven" — none was).
**Not independently ground-truth-tested this session** (only C1 was) —
`INSUFFICIENT_DATA` for a simulation-level confirmation of C4's sign
specifically, though no discrepancy exists to resolve.

## 8. Jacobian convention

`touch_haptics::jacobian(q)` / `jacobianTransposeTimes` (C++,
`touch_slave.cpp`) and `jacobian()` / `cartesian_force_from_joint_torque()`
(Python port, `plot_phase2_scenario.py`) — previously verified
bitwise-identical between the two implementations in an earlier session
(cited, not re-verified this session). Whether this Jacobian's frame
equals HDAPI's own device frame is explicitly flagged as unverified in
the code itself (`touch_slave.cpp:1174-1177`, "NOT yet verified") —
`fkResidualVsMeasured()`/`test_sim_fk_residual.py` is the intended check;
this remains open, unrelated to the C1 sign work, and is not resolved
here.

## 9. mapRosForceToDevice / mapDeviceVectorToRos

Fully specified in §1 above — this section exists only as an index entry
so this document is a complete answer to Goal 6's checklist.

## What changed this session vs. what is unchanged

| Item | Status |
|---|---|
| §1 device-frame mapping | Unchanged |
| §2 master/slave position mapping | Unchanged |
| §3 force-build pipeline (spring/damping/aux/clamps) | Unchanged |
| §4 joint torque / Jacobian-transpose | Unchanged |
| §5 DOB external torque sign | Unchanged (already correct, already tested) |
| **§6 C1 sign** | **CORRECTED** (this session) |
| §7 C4 sign | Unchanged (already correct, not independently re-tested) |
| §8 Jacobian convention | Unchanged (pre-existing open item, unrelated) |

## Stop condition

Source inspection and prior-session sim-test results only. No hardware
accessed to produce this document.
