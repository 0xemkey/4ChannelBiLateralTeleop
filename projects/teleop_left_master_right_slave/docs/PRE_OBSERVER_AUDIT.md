# Pre-observer audit — bilateral Touch teleoperation

Audit date: 2026-08-18. Scope: `projects/teleop_left_master_right_slave/` plus
`ros2_ws/src/touch_haptics/src/touch_slave.cpp`.
Purpose: establish ground truth before adding a disturbance-observer torque
estimator on master and slave.

Everything below marked **[verified]** was executed on the real hardware in this
session (both Touch devices connected, `/dev/ttyACM0` + `/dev/ttyACM1`).
Items marked **[not verified]** say why.

---

## 0. Headline findings

1. **`/touch_left/joint_torque` and `/touch_right/joint_torque` are dead topics.**
   They publish identically `0.0` even while the device is actively rendering
   force. `HD_CURRENT_JOINT_TORQUE` is never written by this code (the code
   commands Cartesian `HD_CURRENT_FORCE`), so the joint-space buffer is never
   populated. The README's claim that it is a useful "command log" is wrong in
   practice.
2. **`HD_LAST_JOINT_TORQUE` (0x2802) does work and is not used anywhere.**
   It returns real, sign-correct, per-joint resolved torque. This is the
   `tau_commanded` a disturbance observer needs, and it is available for free.
3. **`HD_MOTOR_TEMPERATURE` is also dead** — flat at ~1e-10 through 12 s of
   sustained 0.5 N. Not sensed, not modelled.
4. The "nothing is measured" claim is **correct**. Only encoders are read.
5. The two-condition stall gate is a genuine **AND** and hysteresis works.
6. The sudo mount-bind in `run_slave_right.sh` is **unnecessary** with the
   current device configs.

---

## 1. Inventory

### Nodes (from source, confirmed against a live `ros2 node list`)

| Node | Source | Role |
|---|---|---|
| `teleop_master_left` | `touch_slave.cpp` | T1, left device as master |
| `teleop_slave_right` | `touch_slave.cpp` | T2, right device as slave |
| `touch_master_slave_bridge` | `bridge_master_slave.py:106` | T3, coupling at 100 Hz |

T1 and T2 are the **same binary** with different parameters and remaps. The node
name in the class constructor is `touch_slave` (`touch_slave.cpp:153`);
both launchers override it with `-r __node:=`.

### Topics per device, all declared in `touch_slave.cpp`

Published (`touch_slave.cpp:181-193`):

| Topic (remapped) | Type | Source line |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | 181 |
| `rpy` | `geometry_msgs/Vector3Stamped` | 182 |
| `buttons` | `std_msgs/Int32` | 183 |
| `clutch` | `std_msgs/Bool` | 184 |
| `applied_force` | `geometry_msgs/WrenchStamped` | 185 |
| `last_force` | `geometry_msgs/WrenchStamped` | 188 |
| `joint_torque` | `geometry_msgs/Vector3Stamped` | 189 |
| `gimbal_torque` | `geometry_msgs/Vector3Stamped` | 190 |
| `joint_angles` | `geometry_msgs/Vector3Stamped` | 191 |
| `motor_temperature` | `geometry_msgs/Vector3Stamped` | 192 |
| `tracking_error` | `geometry_msgs/Vector3Stamped` | 193 |

The last six exist only when `publish_telemetry:=true` (`touch_slave.cpp:187`).
Both shipped launchers set it true.

Subscribed (`touch_slave.cpp:196-201`): `target_pose` (`PoseStamped`),
`force_cmd` (`WrenchStamped`), `enable` (`Bool`).

### Doc-vs-code topic diff

No phantom topics: every `/touch_*/...` topic named in `README.md`,
`RUNNING_SCRIPTS.txt`, `SYSTEM_DESIGN_REPORT.html` and `docs/force-and-torque.html`
exists in code. Three topics exist in code but are not named in the docs:
`clutch`, `enable`, `rpy`. Cosmetic only.

---

## 2. Run — what actually happened [verified]

All three processes were brought up on real hardware.

- ROS 2 Humble at `/opt/ros/humble`, OpenHaptics `libHD.so.3.4.0` at `/usr/lib`.
- Both devices enumerate as `2988:0302 3D Systems Virtual Serial Port`.
- Installed binary (built 15:04:51) is newer than its source (15:04:29) — current.

### Measured rates (`ros2 topic hz`, not assumed)

```
/touch_left/pose               200.004 Hz
/touch_right/pose              200.028 Hz
/touch_left/joint_torque       200.043 Hz
/touch_right/tracking_error    200.012 Hz
/touch_left/applied_force      200.032 Hz
/touch_right/target_pose       100.000 Hz   (bridge)
/touch_right/enable            100.007 Hz   (bridge)
```

Both nodes started clean. Driver capability banner, identical on both devices:

```
nominal_max=3.300 N  nominal_continuous=0.880 N  ramping_rate=0.500
MAX_FORCE_CLAMPING=ON FORCE_RAMPING=ON SOFTWARE_FORCE_LIMIT=ON ONE_FRAME_LIMIT=ON
```

**HDAPI servo loop rate: `HD_UPDATE_RATE` = 1000 Hz; measured async callback rate
1019 Hz over 3.00 s.** The `hdCallback` runs at 1 kHz; `publishTick` runs at
`publish_hz` = 200 Hz. Any observer belongs in `hdCallback`, at 1 kHz.

### The sudo step is unnecessary

`run_slave_right.sh:48-74` rebinds `/dev/ttyACM1` onto `/dev/ttyACM0` inside a
private mount namespace, requiring sudo. The active config directory
(`GTDD_HOME=ros2_ws` → `ros2_ws/config/`) already assigns distinct channels:

```
ros2_ws/config/Left.config            Channel=300   -> /dev/ttyACM300 -> ttyACM0
ros2_ws/config/Default Device.config  Channel=301   -> /dev/ttyACM301 -> ttyACM1
```

Both symlinks exist. I started the slave with `RIGHT_TTY=MAPPED_TTY=/dev/ttyACM1`
to skip the bind branch, and it came up correctly on the right device. The bind
is a no-op left over from an older single-channel config
(`ros2_ws/config/config/Default Device.config` still has `Channel=0`).

### What a hardware-in-the-loop run would still need to confirm

I could not verify anything requiring a hand on the stylus:
- Force reflection engaging against a **real** obstruction (I proved the logic
  synthetically, not with a physical wall).
- Whether the loop oscillates at the ~10 Hz the report predicts — that needs a
  human grip closing the loop.
- Clutch button behaviour (needs a finger on button 2).
- Actual reach/travel numbers and the anchor offsets in the report's table.
- Whether `REFLECT_MASTER_DAMPING > 0` destabilises on real hardware.

---

## 3. Forward path trace — master pose to `/touch_right/target_pose`

| Stage | Code | Notes |
|---|---|---|
| encoder read | `touch_slave.cpp:336` `HD_CURRENT_TRANSFORM` | 1 kHz, in mm |
| scale to m | `touch_slave.cpp:340-342` | `pos_scale` = 0.001 |
| axis map (device→ROS) | `touch_slave.cpp:347`, impl `454-469` | swap_xy then inverts |
| publish `pose` | `touch_slave.cpp:506` | 200 Hz |
| anchor subtract | `bridge:287` | `delta = latest_left - left_anchor` |
| deadband | `bridge:288-293` | 0.2 mm, per axis, hard-zero |
| axis map | `bridge:295` | `apply_axis_map` |
| axis scale | `bridge:296` | per slave axis |
| offset clamp | `bridge:298-318` | `axis` mode default, per-axis |
| add anchor | `bridge:320` | `target = right_anchor + mapped` |
| absolute override | `bridge:326-329` | inactive by default |
| rate limit | `bridge:331`, impl `543-552` | 0.5 m/s |
| publish | `bridge:353` → `566-574` | 100 Hz |

**Order matches the documentation exactly.** `SYSTEM_DESIGN_REPORT.html`
(rendered §, "Order matters: the axis map runs before scale and clamp") is
correct: map at `bridge:295` precedes scale at `296` and clamp at `298`.

Two notes, neither a doc mismatch:

- The deadband is a dead **zone around the anchor**, not a jitter filter during
  motion. Once `|delta| > 0.2 mm` the full value passes, jitter included. The
  doc's "kills encoder jitter at rest" is accurate as written.
- **Absolute axes bypass the offset clamp.** `bridge:326-329` overwrites
  `target[i]` *after* the clamp at `298-318`, so an absolute axis is bounded only
  by the rate limiter. Inactive by default (`ABSOLUTE_AXES=""`), but note the
  report's parameter table lists `ABSOLUTE_AXES = y` as if it were on.

---

## 4. Return path trace — reflection controller

Entry `bridge:372` `_update_force_reflection`.

| Stage | Code |
|---|---|
| error source | `bridge:359-370` `_slave_error`, prefers `tracking_error` topic |
| threshold | `bridge:386` `deadband + velocity_comp * target_speed` |
| stall gate | `bridge:392-399` |
| hysteresis | `bridge:403-406` |
| **AND combine** | `bridge:407` `engaged = engaged and self.stalled` |
| excess | `bridge:423-424` |
| direction | `bridge:429` unit error, negated at `431` |
| inverse axis map | `bridge:432` |
| master damping | `bridge:449-451` (off by default) |
| low-pass | `bridge:453-456` alpha 0.15 |
| slew limit | `bridge:457`, impl `460-477` 8 N/s |
| publish | `bridge:458` → `487-500` |
| clamp to `reflect_max_n` | `bridge:430` (inside `min()`) |
| device clamp | `touch_slave.cpp:284` `clampNorm(force, max_force_n_)` |

### The stall gate is a genuine AND [verified]

`bridge:407` is a literal `and`. Confirmed empirically by driving the real
`_update_force_reflection` with synthetic state:

```
err=30.0mm slave_v=0.200 cmd_v=0.200 -> stalled=False engaged=False |F|=0.0000N
err=30.0mm slave_v=0.000 cmd_v=0.200 -> stalled=True  engaged=True  |F|=0.0800N
```

Same 30 mm error; force only when the slave is stopped. Condition A (error) alone
is not sufficient. Not an OR, not a single condition.

### Hysteresis at the boundary [verified]

Threshold 4.0 mm, release ratio 0.6 → release at 2.4 mm. Sweep up then down:

```
err=3.9mm -> engaged=False
err=4.1mm -> engaged=True     <- engage
err=3.5mm -> engaged=True     (still held)
err=2.5mm -> engaged=True
err=2.3mm -> engaged=False    <- release
```

Clean 4.0/2.4 mm band, no chatter.

**One real discontinuity at engage.** `bridge:423` computes
`excess = err_norm - threshold * release_ratio` using the release ratio *always*,
including on the first engaged frame. So at the instant of engagement the
magnitude jumps from 0 to `gain x threshold x (1 - release_ratio)` — with shipped
values `18 x 0.004 x 0.4 = 0.029 N`. Small, and the 8 N/s slew limiter smooths it
over ~4 ms. Not a bug today, but it scales with `reflect_gain` and with any
larger `reflect_deadband_m`, so it is worth knowing before you raise either.

---

## 5. The "nothing is measured" claim [verified]

### Every HDAPI get-parameter call in `touch_slave.cpp`

| Call | Line | Classification |
|---|---|---|
| `HD_CURRENT_TRANSFORM` | 336 | **(a) genuine sensor** — encoder-derived pose |
| `HD_CURRENT_VELOCITY` | 337 | **(a) genuine** — driver's encoder difference |
| `HD_CURRENT_BUTTONS` | 338 | (a) genuine — digital input |
| `HD_CURRENT_JOINT_ANGLES` | 421 | **(a) genuine sensor** — encoder |
| `HD_LAST_FORCE` | 418 | **(b) readback of our own command** |
| `HD_CURRENT_JOINT_TORQUE` | 419 | **(b) — and returns constant zero, see below** |
| `HD_CURRENT_GIMBAL_TORQUE` | 420 | (b) — zero, passive gimbal |
| `HD_MOTOR_TEMPERATURE` | 422 | dead — constant, see below |
| `HD_NOMINAL_MAX_FORCE` | 310 | static device constant, logging only |
| `HD_NOMINAL_MAX_CONTINUOUS_FORCE` | 311 | static constant, logging only |
| `HD_FORCE_RAMPING_RATE` | 312 | static constant, logging only |
| `hdIsEnabled(...)` x4 | 321-324 | capability flags, logging only |

The only write is `hdSetDoublev(HD_CURRENT_FORCE, ...)` at `touch_slave.cpp:406`.

The three `HD_NOMINAL_*` reads feed a log line and one warning
(`touch_slave.cpp:325-330`). They never enter the control law — `max_force_n_`
stays whatever the parameter said.

### Empirical: commanded force vs readback [verified]

Commanding force on the left device and reading back:

```
cmd=(0,+0.20,0)  joint_torque=(0,0,0)  last_force=(0,+0.20,0)
cmd=(0,-0.20,0)  joint_torque=(0,0,0)  last_force=(0,-0.20,0)
```

- `last_force` echoes the command exactly → pure command readback, confirmed.
- `joint_torque` stays **exactly zero while 0.2 N is being rendered**. A real
  torque sensor could not do that; neither could a working command log.

### Why `joint_torque` is zero, and what to use instead [verified]

`HD_CURRENT_JOINT_TORQUE` is listed in both the get block
(`/usr/include/HD/hdDefines.h:144`) and the set block (line 170), exactly as the
README says. But this code commands **Cartesian** `HD_CURRENT_FORCE`, so the
joint-space command buffer is never written and reads back as zero.

A standalone HDAPI probe comparing all four registers while commanding ±0.30 N:

```
cmd_fy=+0.30  CURRENT_JOINT_TORQUE=(0,0,0)  LAST_JOINT_TORQUE=(0,+24.071,-14.497)
cmd_fy=-0.30  CURRENT_JOINT_TORQUE=(0,0,0)  LAST_JOINT_TORQUE=(0,-24.071,+14.497)
cmd_fy= 0.00  CURRENT_JOINT_TORQUE=(0,0,0)  LAST_JOINT_TORQUE=(0,  0.000,  0.000)
CURRENT_MOTOR_DAC_VALUES=(0,0,0) in all cases (deprecated, not populated)
```

**`HD_LAST_JOINT_TORQUE` carries the driver's resolved per-joint torque command,
is sign-correct, and is currently unused by this codebase.** It lags by exactly
one servo frame (1 ms), consistent with the `LAST_` prefix. Units are not N·m —
0.30 N produced 24.07 on joint 2 — so it needs a scale calibration, but it is
linear and it is the honest `tau_commanded` per joint.

### `HD_MOTOR_TEMPERATURE` [verified]

Flat at ~1e-10 across 2 s idle → 12 s sustained 0.5 N → 6 s idle. It responds to
nothing. Not a sensor, not a thermal model — a dead register on this unit. This
closes the previously-open question of whether it was sensed or modelled: it is
neither, and `/touch_*/motor_temperature` should not be trusted as a duty-cycle
guard.

### `est_load`

`bridge:591` — `lag_norm * args.stiffness_n_per_m`. Position error times a
**commanded** spring constant. Also `monitor_torque.sh:107`. Both are estimates
from tracking error, not sensor reads. Correct as documented.

### Sensor code paths anywhere in the repo

Grepped the whole tree (including `vendor/`) for `strain.?gauge|load.?cell|
ft.?sensor|force.?torque.?sensor|current.?sens|motor.?current|ati_|robotiq.?ft|
wrench_sensor|torque_sensor|shunt|adc`. **Zero matches** outside one unrelated
comment in `vendor/miml_vla/.../observations.py`. Confirmed: no current-sensing,
no strain gauge, no F/T sensor path exists.

---

## 6. Stability claims

### `REFLECT_GAIN` vs `STIFFNESS_N_PER_M`

| Source | `REFLECT_GAIN` | `STIFFNESS_N_PER_M` | Bounded? |
|---|---|---|---|
| `run_bridge.sh:42` / `:37` (shipped) | 18.0 | 35.0 | yes, 0.51x |
| `bridge_master_slave.py:679` / `:658` (argparse) | 25.0 | 35.0 | yes, 0.71x |
| `SYSTEM_DESIGN_REPORT.html` table | 12 | 35 | yes, 0.34x |

The invariant "gain below the slave spring constant" **holds in all three**, but
the three disagree on the number. The report documents a value (12) that no code
path uses. If you run `run_bridge.sh` you get 18; if you invoke
`bridge_master_slave.py` directly you get 25.

### `REFLECT_MASTER_DAMPING` is genuinely disabled [verified]

- `run_bridge.sh:57` — `REFLECT_MASTER_DAMPING="${REFLECT_MASTER_DAMPING:-0.0}"`
- `bridge_master_slave.py:698` — argparse `default=0.0`
- Passed through at `run_bridge.sh:121`.

Wired in at **`bridge_master_slave.py:449-451`**:

```python
damping = float(self.args.reflect_master_damping)
if damping > 0.0:
    raw = [raw[i] - damping * self.master_vel[i] for i in range(3)]
```

Sits between the inverse axis map (`432`) and the low-pass (`453`). `master_vel`
is computed at `bridge:280-285`. **This is the natural hook for a master-side
observer term** — same place in the pipeline, same signal shape.

---

## 7. Gap report

### 7a. Documentation claims the code does not do

| Claim | Where | Reality |
|---|---|---|
| `HD_CURRENT_JOINT_TORQUE` is "useful as a command log" | `README.md:66,78` | Reads constant zero. Useless as shipped. `HD_LAST_JOINT_TORQUE` is the working one. |
| `HD_MOTOR_TEMPERATURE` "watch it during stiff runs" | `README.md:70` | Flat through 12 s at 0.5 N. Dead. |
| `REFLECT_GAIN = 12 N/m` | `SYSTEM_DESIGN_REPORT.html` table | Shipped default is 18.0; argparse 25.0. |
| `REFLECT_MAX_N = 1.0 N` | same table | Shipped 1.5 N. |
| `MAX_FORCE_N = 1.2 N` (master) | same table | `run_master_left.sh:23` ships 2.0 N. |
| `ABSOLUTE_AXES = y` | same table | `run_bridge.sh:25` ships empty (feature off). |
| reflect clamp "1.0 N → device clamp 1.2 N" | report §, rendered line 494 | Actually 1.5 N → 2.0 N. |
| Slave `tty` rebind is required | `run_slave_right.sh:6-8` | Unnecessary; configs use channels 300/301. Costs a sudo prompt for nothing. |

Note the shipped force ceilings are *higher* than documented (2.0 N vs 1.2 N
master), and 2.0 N is above the 0.88 N continuous rating. `run_master_left.sh:44`
does warn about this at startup, so it is deliberate — but the report's table
understates what the system actually permits.

### 7b. Code behaviour the documentation does not mention

| Issue | Where | Consequence |
|---|---|---|
| **Hardcoded 200.0 Hz** in the slave-speed estimator | `bridge:222` `step * 200.0` | Silently assumes the slave's `PUBLISH_HZ=200`. `master_vel` next to it uses `args.rate_hz` (`bridge:283`). Set `PUBLISH_HZ=100` on T2 and `slave_speed` doubles, the stall gate decides the slave is always moving, and **force reflection silently never engages**. |
| `master_vel` is stale across a clutch | `bridge:280-285` after the early return at `276` | `prev_left` is not updated while clutched, so on release `raw_v` is the whole hand movement times 100 Hz. Harmless today (damping off), but it lands exactly where you plan to hook the observer. |
| `self.tracking_error` is never invalidated | `bridge:244-245`, used at `366` | If T2 dies, the last value persists forever and the bridge keeps reflecting force from a stale error. |
| Absolute axes bypass the offset clamp | `bridge:326-329` vs `298-318` | Unbounded except by rate limit. Off by default. |
| Engage-instant force step | `bridge:423` | ~0.029 N jump at shipped gain; scales with gain. |

### 7c. Hardcoded dynamics constants — none read from HDAPI

All confirmed to be plain code/CLI constants:

| Constant | Value | Where |
|---|---|---|
| `stiffness_n_per_m` | 8.0 default / 35.0 shipped | `touch_slave.cpp:165`, `run_slave_right.sh:19` |
| `damping_n_s_per_m` | 0.08 default / 0.20 shipped | `touch_slave.cpp:166`, `run_slave_right.sh:20` |
| `max_force_n` | 0.45 / 1.5 slave / 2.0 master | `touch_slave.cpp:167` |
| `max_position_error_m` | 0.04 | `touch_slave.cpp:168` |
| `stale_timeout_s` | 0.05 | `touch_slave.cpp:169` |
| `pos_scale` | 0.001 | `touch_slave.cpp:159` |
| `reflect_gain` | 18.0 / 25.0 | `run_bridge.sh:42`, `bridge:679` |
| `reflect_slew_n_per_s` | 8.0 | `bridge:709` |
| `reflect_alpha`, `reflect_speed_alpha` | 0.15 | `bridge:693,695` |
| slave-speed sample rate | **200.0 hardcoded** | `bridge:222` |

**No mass, inertia, or friction constant exists anywhere in the codebase.** There
is no dynamic model at all — the slave controller is a pure PD spring/damper
(`touch_slave.cpp:392-394`). A model-based 4-channel architecture will need
M, C, G introduced from scratch; nothing to reuse and nothing to contradict.

### 7d. Where to insert the disturbance observer

Both master and slave run the **same** `touch_slave.cpp`, so one implementation
serves both. Everything happens inside `hdCallback` at **1 kHz** (measured), not
at the 200 Hz publish rate.

**Signals already available in `hdCallback`:**

| Signal | Line | Note |
|---|---|---|
| joint angles `q` | 421 | genuine encoder, but only read when `publish_telemetry` |
| Cartesian pose | 336 | genuine |
| Cartesian velocity | 337 | driver-differenced |
| `tau_commanded` (Cartesian) | 405-406 | `hd_force`, device frame, post-clamp |
| `tau_commanded` (per joint) | **not read** | add `HD_LAST_JOINT_TORQUE` next to line 419 |

**Insertion point 1 — capture `tau_commanded`.**
`touch_slave.cpp:406` is where `hd_force` is handed to the driver. The
per-joint equivalent needs a new `hdGetDoublev(HD_LAST_JOINT_TORQUE, ...)`
alongside line 419. Note it is one frame delayed, so the observer must compare it
against the *previous* frame's state, or accept 1 ms of skew.

Also move the `HD_CURRENT_JOINT_ANGLES` read (line 421) out of the
`if (self->publish_telemetry_)` block at line 415 — the observer needs `q` every
frame regardless of whether telemetry is being published.

**Insertion point 2 — add `tau_external` back into the control law.**
The force sum is `touch_slave.cpp:379-402`:

- 392-394: spring/damper term (slave only; master has no `target_pose`)
- 396-400: external `force_cmd` term (this is the master's only input today)
- 401: `clampNorm(force_ros, max_force_n_)`

A `tau_external` feedback term goes **between line 400 and the clamp at 401**, so
it is still bounded by `max_force_n_`. That keeps the existing safety envelope
intact — important, because the observer will initially be wrong.

**Master-side, bridge-level alternative.** If you would rather keep the observer
in Python first, `bridge_master_slave.py:449-451` is the existing damping hook —
same pipeline position, ahead of the low-pass and slew limiter, and it inherits
both. Lower risk for a first iteration, but it runs at 100 Hz, not 1 kHz, and the
round-trip delay the code comments warn about (~30 ms) applies. For a real DOB,
`hdCallback` at 1 kHz is the right home.

**Watch out:** `DeviceState`/`CommandState` are copied wholesale under `mtx_`
(`touch_slave.cpp:362-364`, `429-449`). Observer state must live in the callback
or be added to that mutex discipline, not read unlocked.

### 7e. What is in the `/touch_right/force_cmd` forward path today

**Nothing, by default.** [verified] With the shipped config,
`ros2 topic info /touch_right/force_cmd` reported:

```
Type: geometry_msgs/msg/WrenchStamped
Publisher count: 0
Subscription count: 1
```

The only publisher that can ever exist is the gravity-compensation bias, created
at `bridge:172-175` and published at `bridge:554-564` — a constant on one axis,
republished every tick. It is created **only if** `slave_gravity_comp_n != 0.0`,
and `run_bridge.sh:28` ships `SLAVE_GRAVITY_COMP_N=0.0`, so the publisher is
never constructed.

So the claim "gravity-comp bias only" is right in intent, and in practice the
path is completely empty. You are overwriting nothing. If you later enable
gravity comp, note it and your observer output would both land on the same
topic and **add** at `touch_slave.cpp:396-400` — one `force_cmd` subscriber, last
writer wins per message, so two publishers on that topic would fight.

---

## 8. Concrete risks before you start

1. **Do not build on `/touch_*/joint_torque`.** It is identically zero. Switch to
   `HD_LAST_JOINT_TORQUE` and calibrate its scale to N·m first — without that
   calibration every observer gain is meaningless.
2. **Fix `bridge:222`'s hardcoded 200.0** before trusting any stall-gated
   comparison, or the observer will be evaluated against a silently wrong
   `slave_speed`.
3. **Invalidate `self.tracking_error` on staleness** (`bridge:244`) before adding
   anything that reacts harder to it than the current gentle spring.
4. **`master_vel` is corrupt for one tick after a clutch release**
   (`bridge:280-285`) — fix it before enabling any velocity-dependent term.
5. **Reconcile the report's parameter table** with the shipped scripts, or you
   will tune against numbers the system is not using (gain 12 vs 18 vs 25).
6. **Keep the new term inside the `clampNorm` at `touch_slave.cpp:401`.**
7. The master ships at `MAX_FORCE_N=2.0` against a 0.88 N continuous rating. An
   observer that mis-estimates will happily sit at 2 N. Consider lowering it
   while the observer is unvalidated.
8. **Still no real torque sensing.** The observer estimates disturbance from a
   model plus encoders; it does not measure force. Every limitation in
   `README.md:84-105` still applies — it cannot separate contact from friction,
   gravity, or a workspace limit. Worth deciding explicitly whether that is
   sufficient, or whether the ATI Nano17 route is back on the table.
