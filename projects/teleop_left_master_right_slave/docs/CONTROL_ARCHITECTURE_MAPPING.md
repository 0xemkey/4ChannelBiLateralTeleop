# Control architecture mapping: paper vs. project

Reference: "Learning Contact for Haptic Feedback: Switching X-lateral
Teleoperators" (Yilmaz & Tumerdem, ICRA 2024). Produced before any code
edit, per the 2026-09-07 diagnose-and-repair task's explicit requirement.

## 0. The nomenclature mismatch, precisely (not just "they differ")

The paper's own equations (10)-(13) define, for master control law `Fcm`
and remote control law `Fcr`:

```
Fcm = C1*xr - Cm*xm - C5*Fm_ext_hat - C3*Fr_ext_hat      (10)
Fcr = C2*xm - Cr*xr - C4*Fm_ext_hat - C6*Fr_ext_hat      (11)
C1 = C2 = Cm = Cr = Cp = Kp + Kd*s                        (12)
C3 = C4 = C5 = C6 = Cf                                    (13)
```

Reading off what each term actually carries:

| Paper symbol | What crosses, in the paper's own equations | Project's name for this same signal |
|---|---|---|
| paper C1 (in `Fcm`) | remote position `xr` → master control law | **project C2** (slave position → master) |
| paper C2 (in `Fcr`) | master position `xm` → remote control law | **project C3** (master position → slave) |
| paper C3 (in `Fcm`) | remote's estimated ext. force `Fr_ext_hat` → master | **project C1** (slave force → master) |
| paper C4 (in `Fcr`) | master's estimated ext. force `Fm_ext_hat` → remote | **project C4** (master force → slave) — *same index, coincidentally* |

**So paper{C1,C2,C3,C4} = project{C2,C3,C1,C4}.** Only C4 lines up by
coincidence. This document, and all project code/docs/tests, use the
**project** numbering throughout (C1=slave-force→master,
C2=slave-position→master, C3=master-position→slave,
C4=master-force→slave) — this was already the established convention
before this task; it is restated here explicitly so "C1" in any future
discussion is never ambiguous against the paper's own C1.

## 1. Signal-by-signal trace, estimator to actuator

| # | Paper signal/eq. | Project meaning | Source var/topic | Destination var/topic | Sign | Gain | Deadband | Saturation | Frame | Active? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | `xm(t)` master position | master Cartesian pose | HDAPI `HD_CURRENT_POSITION` (device axes) | `/touch_left/pose` (ROS, `mapDeviceVectorToRos`) | device-config `invert_x/y/z`, `swap_xy` (`Left.config`) | 1:1 (mm→m only) | none | workspace-limited by device | ROS frame, **master's own origin** | always |
| 2 | (not in paper — anchor scheme) | teleop delta | `bridge_master_slave.py: delta = latest_left - left_anchor` | — | — | 1 | `--deadband-m` (0.0002 m) | none | master-relative (since bridge start) | always, when C3 armed |
| 3 | `C2` (paper) = **C3** (project), Eq. `xm → Fcr` | axis remap master-delta → slave-delta | `apply_axis_map(axis_map, delta)` then `* axis_scale` | `mapped` | `--axis-map` signs (x,y,z ⇒ identity in current launch config) | `--axis-scale` (1,1,1 in current config) | none | `--max-offset-xyz` (0.12/0.19/0.15 m), per-axis clamp | slave-frame (post axis-map) | **ACTIVE** |
| 4 | — | slave target pose | `target = right_anchor + mapped` | `/touch_right/target_pose` | — | — | — | `--max-speed-mps` rate limit (0.5 m/s) | slave-relative (since bridge start) | ACTIVE |
| 5 | Cr (slave PD), Eq. (11) | slave Cartesian spring/damper | `touch_slave.cpp`: `spring_force = stiffness*(target-pos)`, `damping_force = -damping*vel` | `force_ros` → `force_device` → `HD_CURRENT_FORCE` | error sign fixed; damping opposes velocity | `stiffness_n_per_m=35`, `damping_n_s_per_m=0.20` (slave) | `max_position_error_m=0.04` clamps error before spring | `max_force_n=1.5` (slave), `clampNorm` | slave device frame | ACTIVE |
| 6 | paper **C1** = **project C2**, Eq. `xr → Fcm` | slave position → master target | `bridge_master_slave.py: _update_c2()`, undoes axis-map on `slave_disp` | `master_target_pub` → `/touch_left/target_pose` | inverse of #3's map | 1 (no separate C2 gain exposed) | none additional | `--left-max-offset-m` (0.15 m) | master-relative | **DISABLED** — `--enable-c2` defaults false, and `experiment_launch/03_start_position_bridge.sh` hardcodes `ENABLE_C2=false` unconditionally (no flag to arm it in the current safety framework) |
| 7 | Cm (master PD) | master Cartesian spring/damper | same formula as #5, master's own `stiffness_n_per_m` | `HD_CURRENT_FORCE` (master) | same | **8 N/m, 0.08 Ns/m** (zero-force default) — the C2-tuned value (`LEFT_STIFFNESS_N_PER_M=15.0`) is a *different* launch parameter, not applied unless C2 is armed | `max_position_error_m=0.04` | `max_force_n=2.0` | master device frame | technically active, but has nothing to render against while C2 is off (no fresh `target_pose`) |
| 8 | `tau_dis_hat` (DOB) | joint disturbance estimate | `JointDob::update()` in `touch_dob.hpp`, Tustin Q-filter | `dob_bank_.tauDisHatNm()` | — | `g_dis={10,10,5} rad/s` (paper-matched, see FOUR_CHANNEL_TODO.md 2026-08-25) | — | — | joint space | always (both arms) |
| 9 | `tau_ext_hat = -tau_dis_hat`, then deadbanded | force estimate, stage 1 | `computeForceEstimate()` in `touch_force_estimate.hpp` | `ForceEstimate.tau_ext_deadbanded_nm` → `/touch_{left,right}/force_ext_hat` | `ext = -dis` | — | **stage 1**: per-joint shrinking deadband, `threshold_nm` = waist 0.07632 / shoulder 0.064 / elbow 0.05 N·m | — | joint space, per-arm | always |
| 10 | paper **C3** = **project C1** | slave force → master, weighting + mask | `bilateral_4ch.py`: `tau_c1 = w1*Cf*tau_e_hat*c1_joint_mask` | `/touch_left/joint_torque_ext_cmd` | — | `w1` (**0.4** in current tests), `Cf=1.0` | **`c1_joint_mask = [1,0,1]`** — shoulder forced to exactly 0, see §2 below | **stage 2**: `kMinRelayTorqueNormNm=0.02` N·m hard gate on the weighted 3-vector norm (in `touch_slave.cpp`, not this script) | joint space | **ACTIVE, shoulder masked** |
| 11 | project C1, continued | relay → master Cartesian force | `touch_slave.cpp`: `cartesianForceFromJointTorque(J,tau_ext,det)`, added to `force_device` | `HD_CURRENT_FORCE` (master), also `/touch_left/last_force`, `/touch_left/applied_force` (fixed 2026-09-07, see below) | Jacobian-transpose inverse | — | stage-2 gate (see #10) | re-`clampNorm(force_device, max_force_n=2.0)` after addition | master device frame | ACTIVE |
| 12 | paper C4 = project C4 | master force → slave | same as #10-11, mirrored: `tau_c4 = w4*Cf*tau_h_hat` (**no joint mask** — see §3 below) | `/touch_right/joint_torque_ext_cmd` → slave `HD_CURRENT_FORCE` | — | `w4` (**0.4**) | stage-1 (source-side threshold) + stage-2 (0.02 N·m gate), no per-joint mask | `clampNorm(..., max_force_n=1.5)` | slave device frame | ACTIVE |

## 2. Why C1 shoulder received = 0 (traced, not assumed)

**Root cause: `bilateral_4ch.py`'s `c1_joint_mask`, one line, by design —
not a wiring, index, sign, or serialization bug.**

```python
# bilateral_4ch.py:227
self.c1_joint_mask = [0.0 if name in disabled else 1.0 for name in JOINTS]
# bilateral_4ch.py:643, default:
p.add_argument("--c1-disable-joints", default="shoulder", ...)
# bilateral_4ch.py:441
tau_c1 = [w1_eff * self.args.cf * v * m for v, m in zip(tau_e_hat, self.c1_joint_mask)]
```

`JOINTS = ("waist", "shoulder", "elbow")` — index 1 is forced to exactly
`0.0` after weighting, every tick, regardless of what the slave's own
`force_ext_hat` shoulder value is. This is confirmed, not inferred: an
earlier sim test this session ran C1 **unmasked** and shoulder relayed
real, nonzero signal (transmission ratio comparable to waist/elbow) — the
underlying receive path (joint index order, sign, `touch_slave.cpp`'s
`joint_torque_ext_deadbanded_nm[1]` handling) is not broken. The mask is
the entire explanation.

**Why the mask exists:** the right arm's (slave's) shoulder identification
was found to have a noise/friction floor at or above its own torque
budget at every filter cutoff tested (2026-09-01 finding, predates this
task). Relaying it would inject noise-dominated "contact" signal into the
master's hand. Confirmed independently this session: unmasking C1
reaches 108-188% of shoulder's torque budget under aggressive multi-axis
pushing before the Cartesian clamp catches it.

**Note the asymmetry:** C4 (master → slave, row 12) has **no equivalent
mask** — the master/left arm's shoulder identification was not flagged
the same way, but this has not been independently re-verified with the
same rigor as the right arm's case. Flagged as an open question, not
fixed here.

## 3. Startup position alignment — what the code actually does

**Correction to the task's framing:** the code does **not** command one
robot to jump to the other's absolute Cartesian coordinate. Confirmed by
reading `_tick()` directly: `target = right_anchor + mapped(delta)`, where
`delta = latest_left - left_anchor` — this is **already anchor-relative**,
matching the spirit of `dx_m = x_m - x_m0` / `dx_s = x_s - x_s0` the task
asks for.

**What's actually missing**, and why it still produces exactly the
symptom observed (master/slave visibly not starting aligned):

- `left_anchor` / `right_anchor` are captured on **whichever pose message
  arrives first** after the bridge starts (`_left_cb`/`_right_cb`:
  `if self.left_anchor is None: self.left_anchor = list(self.latest_left)`)
  — wherever each physical arm happened to be resting, with **no operator
  action, no "hold still" prompt, no verification the two poses
  correspond to the same real-world configuration.**
- There is **no acceptance gate** — `_tick()` starts publishing targets
  and forces as soon as both anchors exist, whatever they are.
- There is **no explicit frame-registration transform** (`R_ms`, scale,
  offset) — only `axis_map`/`axis_scale`, which are fixed launch
  parameters (currently identity/1.0), not derived from the anchor pair.
- A diagnostic (`right_minus_mapped_left` in `teleop_calibration.json`,
  `_write_calibration()`) already computes "how far off would the anchors
  be if they should have matched" — but this is logged only, never fed
  back into the control law. A per-axis trim derived from one such
  snapshot was added 2026-09-04 (`--absolute-trim-xyz`), but only applies
  to axes explicitly listed in `--absolute-axes` (empty by default), and
  is a static correction from one past measurement, not a live
  registration step.
- No ramped authority: `enable=true` and `target_pose` publish
  immediately once both anchors land, no 0.5-2s ramp.

**So:** the task's proposed remedy (explicit `CAPTURE_HOME` +
`ALIGN_RELATIVE_FRAMES` state machine, an acceptance gate, ramped
enabling) is the right prescription for a real gap — just not the gap as
originally framed (it's "uncoordinated/unverified anchors," not "absolute
coordinate jump").

## 4. Why the slave currently cannot move the master

Direct chain, confirmed by reading both the launcher and the receiving
node:

1. **C2 is architecturally complete** (`_update_c2()` in
   `bridge_master_slave.py`, undoes the axis map, targets the master) —
   this is not a stub.
2. It is gated on `args.enable_c2`, which `experiment_launch/
   03_start_position_bridge.sh` **hardcodes to `false`** with no flag to
   override it — the new operator-procedure safety framework simply never
   exposes C2 as an arming option (`OPERATOR_PROCEDURE.md` states this
   explicitly: "The explicitly armed position mode is C3=1 with
   C1=C2=C4=0").
3. Even independent of C2: `01_start_master.sh` always launches the
   master with `ENABLE_C2=false`, so the master keeps its **zero-force
   default stiffness (8 N/m, 0.08 Ns/m)**, not the C2-tuned values
   (15 N/m, 0.10 Ns/m, `LEFT_GRAVITY_COMP_N`) — even if a target arrived,
   the master's own spring is far softer than intended for this mode.
4. **Independently required, and previously undiscovered before this
   session:** the master's `command.enabled` (gates the ENTIRE Cartesian
   force pipeline in `touch_slave.cpp`, not just the relay) is driven by
   `/touch_left/enable`, published only when `--enable-c2` or
   `--reflect-enable` is passed to the bridge. With C2 off and reflect off
   (both are, in the current launch scripts), **nothing ever publishes
   `/touch_left/enable` at all** — the master's Cartesian path is fully
   gated closed regardless of what target arrives.

**Conclusion: this is expected behavior given the current arming
configuration, not a mapping/sign/index bug.** C2 is real, tested code
(confirmed on real hardware 2026-09-03, per project history), simply not
wired into the current safety-gated launch framework. Enabling it
requires: (a) a flag/path to pass `--enable-c2 true` through
`03_start_position_bridge.sh`, (b) starting the master with the C2-tuned
stiffness, (c) confirming `/touch_left/enable` reaches it (already true
once `--enable-c2` is passed, per `left_enable_pub`).

## 5. Gain/deadband/saturation values currently in force (real launch config)

| Parameter | Master | Slave |
|---|---|---|
| Cartesian stiffness | 8 N/m (C2 off) / 15 N/m (C2-tuned, unused) | 35 N/m |
| Cartesian damping | 0.08 Ns/m / 0.10 Ns/m (unused) | 0.20 Ns/m |
| max_force_n | 2.0 N | 1.5 N |
| max_position_error_m | 0.04 m | 0.04 m |
| DOB threshold_nm (w/s/e) | 0.07632 / 0.0640 / 0.0500 | same (per-arm identified) |
| DOB max_torque_nm (w/s/e) | 0.1620 / 0.0810 / 0.0810 | same |
| g_dis (w/s/e) | 10 / 10 / 5 rad/s | same |
| relay floor (kMinRelayTorqueNormNm) | 0.02 N·m | 0.02 N·m |
| C1/C4 weight | w1=0.4 | w4=0.4 |

Paper's Kp/Kd (`{6,3,2}`/`{0.08,0.12,0.03}`) are **joint-space** gains for
a different control path (`joint_space_control_:=true`) that was tried
and rejected for production (see `FOUR_CHANNEL_TODO.md`,
2026-09-03) — the table above is the actual live **Cartesian** path,
which has no direct paper equivalent to compare against 1:1; this is
itself a structural difference worth the task's attention, not just a
gain question.
