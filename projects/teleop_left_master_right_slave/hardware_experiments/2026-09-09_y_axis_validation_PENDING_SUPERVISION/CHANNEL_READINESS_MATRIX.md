# Channel Readiness Matrix — 2026-09-10

Traced directly from source (`bridge_master_slave.py`, `bilateral_4ch.py`,
`touch_slave.cpp`, `touch_dob.hpp`), cross-checked against
`docs/FOUR_CHANNEL_TODO.md` and the current `git diff` (uncommitted working
tree). **No hardware was touched to produce this document.**

## Architecture correction (Part A.2 — resolved from the implementation, not assumed)

`bilateral_4ch.py`'s OWN joint-space position blending (its C2/C3) was
**tried on real hardware and rejected** ("too stiff, no leader/follower
distinction, not a 1:1 Cartesian response" — `FOUR_CHANNEL_TODO.md:121`
item 5). The real system is a **hybrid of two independent processes**:

- **`bridge_master_slave.py`** — Cartesian position (C2, C3), gravity
  compensation, the force-budget coupling planner, the whole state machine
  this session's earlier work hardened.
- **`bilateral_4ch.py`** — force-channel (C1, C4) **relay only**. It reads
  each side's DOB-estimated joint torque (`force_ext_hat`), weights/masks/
  ramps/safety-limits it, and republishes to the peer's
  `joint_torque_ext_cmd`. `touch_slave.cpp` converts that into a **Cartesian**
  force via `cartesianForceFromJointTorque()` and adds it directly to the
  existing `HD_CURRENT_FORCE` write path — **not** the joint-torque write
  path (`HD_CURRENT_JOINT_TORQUE`, gated off at compile time by
  `TOUCH_DOB_ENABLE_TORQUE_WRITE=0` on the hardware build). This is why C1/C4
  can work on real hardware despite that gate: they never use it.
  `bilateral_4ch.py`'s own position-blend output is harmless in this
  configuration only because both nodes run with `joint_space_control:=false`,
  which makes the controller ignore it.

## Channel table

| | C1 | C2 | C3 | C4 |
|---|---|---|---|---|
| **Meaning/direction** | slave contact force → master (reactive/opposing) | slave position → master | master position → slave | master hand force → slave (forward) |
| **Producing process** | `bilateral_4ch.py` (relay) + `touch_slave.cpp` (Cartesian conversion, on the MASTER node) | `bridge_master_slave.py` | `bridge_master_slave.py` | `bilateral_4ch.py` (relay) + `touch_slave.cpp` (on the SLAVE node) |
| **Source topic/field** | `/touch_right/force_ext_hat` (`ForceEstimate.tau_ext_deadbanded_nm`) | `/touch_right/pose` | `/touch_left/pose` | `/touch_left/force_ext_hat` |
| **Destination topic/field** | `/touch_left/joint_torque_ext_cmd`, added into master's `HD_CURRENT_FORCE` | `/touch_left/target_pose` | `/touch_right/target_pose` | `/touch_right/joint_torque_ext_cmd`, added into slave's `HD_CURRENT_FORCE` |
| **Sign convention** | **`-w1` (negated)** — `bilateral_4ch.py:466` (2026-09-10, uncommitted working-tree fix; was UN-negated before) | anchor-relative, `axis_map=x,y,z` identity (validated; do not reintroduce `x,z,-y`) | same anchor-relative formula, C3 direction | `+w4` (not negated — forward pass-through) |
| **Coordinate transform** | `cartesianForceFromJointTorque(J(q_own), tau)` — algebraic inverse of `τ=Jᵀf`, each node's OWN Jacobian/q, no cross-device frame transform found anywhere (`SCIENTIFIC_CONTROLLER_ASSUMPTION_REGISTER.md:186`) | `apply_axis_map` + `axis_scale`, anchor-relative | same | `cartesianForceFromJointTorque(J(q_own), tau)` |
| **Gain/weight** | `w1` (architecture preset; default arch `pp`=0.0, safe arch `4ch_safe`=0.5, full `4ch`=1.0), × `--cf` (default 1.0, must be identical on both sides for power conservation) | n/a (position blend, not a weighted sum) | n/a | `w4` (default `pp`=0.0, `4ch_safe`=0.3, `4ch`=1.0) |
| **Enable/arm/deadman** | `bilateral_4ch.py` publishes `/touch_left/enable = (target_weights[0] > 0)` — **independent of `bridge_master_slave.py`'s own arm/deadman/FAULT state** (see Publisher Ownership finding below) | bridge's `arm_teleop` param + startup-alignment state machine (this session's hardened path) | same | `bilateral_4ch.py` publishes `/touch_right/enable` analogously, **also independent of the bridge** |
| **Freshness timeout** | `--stale-timeout-s` default 0.1s (bilateral_4ch.py) | `pose_stale_s` 0.2s (alignment) / `force_budget_telemetry_stale_s` 0.2s (planner) | same | 0.1s |
| **Clamp/saturation** | per-joint `MAX_TORQUE_NM=(0.162,0.081,0.081)`, deadbands `THRESHOLD_NM` (per-joint noise floor) + `MIN_RELAY_TORQUE_NORM_NM=0.02`, then master's own `max_force_n=2.0` final clamp | master `max_position_error_m=0.03`, `max_force_n=2.0` | slave `max_position_error_m=0.04`, `max_force_n=1.5` | slave's own final `max_force_n=1.5` clamp |
| **Gravity/DOB included** | YES — this channel IS a DOB signal (`tau_ext_deadbanded_nm`, DOB-estimated disturbance/contact torque) | no | no | YES — same, DOB-estimated |
| **Default state (if launched with defaults)** | **effectively OFF** (`--architecture` default `pp` → w1=0.0) | ON whenever the bridge runs with `--enable-c2 true` | ON whenever the bridge runs at all | **effectively OFF** (`pp` → w4=0.0); shoulder joint masked off by default even when armed (`--c1-disable-joints=shoulder`, a per-rig commissioning finding — shoulder's DOB noise floor meets/exceeds its own torque budget, not a design choice) |
| **Existing evidence** | Sim: `analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py` — **re-run this session, PASSES** on the current (corrected, uncommitted) sign. HW: 2026-09-03 "confirmed live" in `FOUR_CHANNEL_TODO.md:121` — **but that run used the OLD, un-negated (wrong) sign** (confirmed by reading the diff's own comment: "Prior to this fix the code carried NO negation... This correction is simulation-validated only; REQUIRES_FUTURE_HARDWARE_VALIDATION"). **The corrected sign has NEVER been hardware-tested.** | Sim: extensively covered this session (`test_c2_live.py` + all Parts 1-4 hardware-validation-infrastructure work). HW: `FOUR_CHANNEL_TODO.md:122`, 2026-09-03, reconfirmed after a restart same day. Not retested against the moving-live-origin fix or the corrected `left_max_offset_m` canonical value (0.08, see round-2 correction) — those changes postdate this HW evidence. | Sim: `test_c3_live.py`, this session's whole 117mm/gap-matrix corpus. HW: `FOUR_CHANNEL_TODO.md:123`, 2026-09-01 (5 single-axis tests) + 2026-09-03 reconfirm. | Sim: symmetric to C1, same test file/function per the doc. HW: 2026-09-03 "confirmed live" — same caveat as C1 does NOT strictly apply (C4's sign was never flagged as wrong), but it has not been RE-tested since. |

## Publisher ownership finding (Part F, item 18) — **BLOCKING for H7–H9**

`bilateral_4ch.py` (lines 331–333, 457–462) creates its OWN publishers on
`/touch_left/enable` and `/touch_right/enable` and publishes to them
**every tick**, driven purely by `target_weights[0]/[3] > 0` (its own
architecture preset) — with **zero knowledge of `bridge_master_slave.py`'s
arm/disarm/FAULT/deadman state**. If both processes run simultaneously (as
H7–H9 require), there are **two independent publishers on each enable
topic** with **unrelated decision logic**. Concretely: if the bridge
disarms, FAULTs, or (once implemented) the master deadman button is
released, `bilateral_4ch.py` will keep re-publishing `enable=true` on its
own next tick regardless, because nothing tells it to stop. This is a real
safety gap, not a hypothetical — it directly undermines this session's
entire hardened disarm/FAULT/deadman safety work the moment C1/C4 run
alongside the bridge.

**This must be resolved (either bilateral_4ch.py stops independently
publishing enable when co-launched with the bridge, or the two are
coordinated through a shared arm signal) before H7 (C1 isolated), H8 (C4
isolated), or H9 (complete four-channel) can be attempted.** Not resolved
in this pass — flagged as the top blocker in the final report. H0–H6 never
launch `bilateral_4ch.py` and are unaffected.

## Readiness states

| Channel | State | Evidence |
|---|---|---|
| C1 | **simulated in isolation** (sign-corrected version). NOT hardware-validated — prior HW evidence used a since-found-wrong sign. **blocked** for any co-launch with C2/C3 pending the enable-publisher conflict above. | `test_c1_sign_vs_ground_truth.py` (passes, re-run 2026-09-10) |
| C2 | **hardware smoke-tested** (2026-09-03, pre-dates this session's moving-live-origin/canonical-config fixes) + extensively simulated this session | `test_c2_live.py`, `FOUR_CHANNEL_TODO.md:122`, this session's 73-test regression |
| C3 | **hardware smoke-tested** (2026-09-01, 2026-09-03) + extensively simulated this session, including the full gap matrix and Part 3's C3-only slave-tracking sim | `test_c3_live.py`, `test_part3_c3_slave_tracking_sim.py`, `FOUR_CHANNEL_TODO.md:123` |
| C4 | **hardware smoke-tested** (2026-09-03) + sim-covered (symmetric to C1) — not retested since | `FOUR_CHANNEL_TODO.md:124` |
| C1+C4 (force pair) | **blocked** — enable-publisher conflict above, and the sign-corrected C1 has never run on hardware at all | this document |
| C2+C3 (bilateral position) | **hardware smoke-tested** (2026-09-03) + simulated in integration extensively this session (gap matrix, disarm, fail-safes) | this session's 73/73 regression |
| Complete four-channel | **blocked** — same enable-publisher conflict; also never re-validated against this session's C2/C3 changes | this document |

## What this matrix does NOT claim

It does not claim any channel is ready for unsupervised or performance
(H10) use. "Hardware smoke-tested" above means a real, documented live run
existed as of 2026-09-03 for the CONFIGURATION AT THAT TIME — it does not
certify the current, since-modified code (moving-live-origin planner,
canonical frozen config, the C1 sign fix, GRAVITY_SETTLE hold mode, the
not-yet-built deadman gate). Every channel needs fresh hardware
confirmation under the current code before H10 tuning.
