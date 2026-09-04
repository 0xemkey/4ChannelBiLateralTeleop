# Findings and context — HIL-RL teleoperation work

Compiled 2026-08-20. Intended as briefing material: a consolidated list of what
is established about these systems, with each item marked by how it was
established. Written to be handed to someone (or something) that needs to come
up to speed without re-deriving it.

Verification legend:

- **[code]** read directly from source in this repo or on the mounted drive
- **[measured]** executed on the real hardware and recorded
- **[claimed]** asserted somewhere but not confirmed here
- **[open]** genuinely undecided

---

## 0. There are three distinct systems, and they are easy to conflate

| # | System | Where | Status |
|---|---|---|---|
| A | Direct bilateral teleop, left Touch master → right Touch slave | `projects/teleop_left_master_right_slave/` | working, ROS 2 |
| B | HIL teleop into IsaacLab with an ACT policy guiding the operator | repo root `scripts/`, `vendor/miml_vla/` | working, ROS 2 + IsaacLab |
| C | MATLAB/Simulink 4-channel bilateral teleop with observers + neural net | `/media/nural/3080A45980A42776/Users/pc1/Desktop/OmniSetup/` | Windows, 2024, not integrated |

**A and B are the ROS implementation. C is a separate MATLAB study.** The
observer coefficients live in C. Nothing from C has been ported into A or B.
Statements that mix them are the most common error in circulating summaries —
see §5.

---

## 1. System A — direct bilateral teleoperation

### Structure [code]

Three processes. Terminals 1 and 2 run the **same binary**
(`touch_haptics/touch_slave`) with different parameters and remaps; terminal 3
is the Python bridge.

| Terminal | Script | Node | Device |
|---|---|---|---|
| 1 | `scripts/run_master_left.sh` | `teleop_master_left` | `Left` |
| 2 | `scripts/run_slave_right.sh` | `teleop_slave_right` | `Default Device` |
| 3 | `scripts/run_bridge.sh` | `touch_master_slave_bridge` | — |

Three rates: HDAPI servo callback **1 kHz** [measured: 1019 Hz over 3.00 s],
ROS publish **200 Hz** [measured: 200.004 / 200.028 Hz], bridge **100 Hz**
[measured: 100.000 Hz].

### Key structural facts [code]

- It is **3-DOF position teleoperation only**. `_publish_target` writes
  `orientation.w = 1.0` and nothing else (`bridge_master_slave.py:573`).
  Orientation is never transmitted; the gimbal is passive.
- Both devices run with every `invert_*` and `swap_xy` false. Device frame
  equals ROS frame in this project. All remapping lives in the bridge's
  `AXIS_MAP`, in one place. (System B differs — it uses `-x,z,y` in the node.)
- The master is `touch_slave`, not `touch_publisher`, specifically so it can
  render force without a node swap.
- The master is enabled only from inside `_publish_master_force`, which exists
  only when `--reflect-enable` is set. With `FORCE_REFLECT=false` the master is
  never enabled at all.
- Everything is republished every tick because the node zeroes force on
  commands older than `stale_timeout_s = 0.05 s`. A publish-on-change design
  would make the device stutter.

### Forward path, in order [code]

```
encoder → ×0.001 → device→ROS map → publish pose (200 Hz)
  → delta = pose − left_anchor                        bridge:287
  → per-axis deadband 0.2 mm, hard-zero               bridge:288-293
  → AXIS_MAP                                          bridge:295
  → AXIS_SCALE                                        bridge:296
  → per-axis offset clamp (or norm clamp)             bridge:298-318
  → target = right_anchor + mapped                    bridge:320
  → [absolute-axis override, off by default]          bridge:326-329
  → rate limit 0.5 m/s                                bridge:331
  → publish target_pose (100 Hz)
  → F = K·clamp(err, ±0.04 m) − D·v, clampNorm(1.5 N) touch_slave.cpp:392-401
```

Order matters: **axis map before scale before clamp.** Scale and clamp are both
in slave axis order, because what is being fitted is the slave's workspace.

The deadband is a dead zone **around the anchor**, not a jitter filter during
motion. Past 0.2 mm the full value passes, jitter included.

### Return path — force reflection [code]

Two independent conditions, ANDed:

```python
threshold = REFLECT_DEADBAND_M + REFLECT_VELOCITY_COMP · target_speed
stalled   = slave_speed < max(STALL_FLOOR_MPS, STALL_RATIO · target_speed)
engaged   = (err_norm > threshold) and stalled          # bridge:407
```

The stall gate is the real discriminator: error alone cannot separate *lagging
behind* from *physically stopped*, and only the second is an obstacle.
[measured] at 30 mm error: `slave_v=0.2 → 0.0000 N`, `slave_v=0.0 → 0.0800 N`.

Hysteresis engages at `threshold`, releases at `threshold × 0.6`.
[measured] clean 4.0/2.4 mm band, no chatter.

Magnitude chain: `excess → gain → clamp → inverse axis map → [damping] →
low-pass α=0.15 → slew 8 N/s → device clampNorm`.

**Master damping is off by default and that is deliberate.** The velocity
estimate lags and the force arrives ~30 ms later, late enough to land in phase
with the motion and pump the oscillation. A closed-loop simulation of this exact
code showed peak-to-peak force rising 0.007 N → 0.65 N as damping went
0 → 4.0 N·s/m. [claimed — the simulation is referenced in the code comment at
`bridge:434-448`, results not independently reproduced here]

### Shipped coefficients [code]

| Where | Parameter | Shipped |
|---|---|---|
| T1 master | `MAX_FORCE_N` | 2.0 (above the 0.88 N continuous rating; warns) |
| T2 slave | `STIFFNESS_N_PER_M` | 35.0 |
| T2 slave | `DAMPING_N_S_PER_M` | 0.20 |
| T2 slave | `MAX_FORCE_N` | 1.5 |
| T2 slave | `MAX_POSITION_ERROR_M` | 0.04 |
| T2 slave | `STALE_TIMEOUT_S` | 0.05 |
| T3 | `RATE_HZ` | 100 (errors below 40 Hz with reflection on) |
| T3 | `AXIS_SCALE` | `1.0,1.0,1.0` |
| T3 | `MAX_OFFSET_XYZ` | `0.12,0.11,0.09` |
| T3 | `DEADBAND_M` | 0.0002 |
| T3 | `MAX_SPEED_MPS` | 0.5 |
| T3 | `REFLECT_GAIN` | **18.0** (argparse default 25.0; HTML report says 12) |
| T3 | `REFLECT_DEADBAND_M` | 0.004 |
| T3 | `REFLECT_VELOCITY_COMP` | 0.05 s |
| T3 | `REFLECT_RELEASE_RATIO` | 0.6 |
| T3 | `REFLECT_MAX_N` | 1.5 |
| T3 | `REFLECT_ALPHA` / `SPEED_ALPHA` | 0.15 |
| T3 | `REFLECT_SLEW_N_PER_S` | 8.0 |
| T3 | `REFLECT_STALL_RATIO` / `FLOOR` | 0.5 / 0.004 |
| T3 | `REFLECT_MASTER_DAMPING` | 0.0 |

Stability invariant: keep `REFLECT_GAIN` below the slave's
`STIFFNESS_N_PER_M`. At 18/35 that is 0.51×.

### Known defects [code, from the 2026-08-18 audit]

1. `bridge:222` hardcodes `step * 200.0` for slave speed while `master_vel`
   beside it uses `args.rate_hz`. Change T2's `PUBLISH_HZ` to 100 and
   `slave_speed` doubles, the stall gate decides the slave is always moving,
   and **force reflection silently never engages**.
2. `self.tracking_error` is never invalidated. If T2 dies the last value
   persists forever and the bridge keeps reflecting from a stale error.
3. `master_vel` is stale across a clutch — the early return at `bridge:276`
   precedes the `prev_left` update at `285`, so on release `raw_v` is the whole
   hand movement × 100 Hz. Harmless with damping off; lands exactly where an
   observer would hook in.
4. Engage-instant force step: `excess` at `bridge:423` always uses the release
   ratio, so magnitude jumps 0 → `gain × threshold × 0.4` = 0.029 N at shipped
   values. Scales with gain and deadband.
5. Absolute axes overwrite `target[i]` after the clamp, so they are bounded
   only by the rate limiter. Off by default.
6. The sudo mount-bind in `run_slave_right.sh:56-82` is **unnecessary**
   [measured] — the active configs already assign distinct channels
   (`Left.config` → 300, `Default Device.config` → 301). Running with
   `RIGHT_TTY=MAPPED_TTY=/dev/ttyACM1` comes up correctly and skips the sudo.

### Documentation drift [code]

`SYSTEM_DESIGN_REPORT.html`'s parameter table disagrees with shipped code in
seven places: `REFLECT_GAIN` 12 vs 18, `REFLECT_MAX_N` 1.0 vs 1.5, master
`MAX_FORCE_N` 1.2 vs 2.0, `ABSOLUTE_AXES=y` vs empty, and the reflect-clamp
narrative that follows from them. `RUNNING_SCRIPTS.txt` describes `AXIS_SCALE`
default as `1.0,0.6,1.0`; the script ships `1.0,1.0,1.0`. **The shipped force
ceilings are higher than documented, not lower.**

---

## 2. System B — HIL teleop with ACT policy guidance

Two terminals; right arm off (`SLAVE_TOUCH_ENABLE=false`). The left Touch drives
the *simulated* robot and a pretrained ACT policy
(`act_lift_seed0_100demo_endo`) runs alongside, blended 50/50
(`HIL_OPERATOR_ACTION_WEIGHT` / `HIL_POLICY_ACTION_WEIGHT`), gripper excluded.
Human gains deliberately reduced: `HIL_POS_SENS=0.8`, `HIL_ROT_SENS=0.1`.

The distinctive part is **AI guidance force**
(`train_rlt_online_miml_lift.py:1231`, `AIGuidanceForcePublisher`): a virtual
target advances each step by the motion the policy asked for, leashed to
`AI_GUIDANCE_MAX_DEVIATION=0.05 m`. Rendered force is spring-to-target minus
damping, zero inside `AI_GUIDANCE_DEADBAND=0.004 m`. Track the policy and you
feel nothing; drift and it pulls, and holds. A second term
(`AI_GUIDANCE_DEPTH_*`) is a virtual floor under the object.

Three bugs are documented as fixed, and the reasoning generalises: force
published once per sim step got stale-zeroed below 20 Hz; force gated on
`hil_intervened` vanished exactly when you held still; force from a per-step
action *difference* is a rate signal, so it chattered around zero instead of
holding. Spring mode uses accumulated position error.

---

## 3. Hardware ground truth

**There is no force or torque sensor on either arm.** Optical encoders only.
This is settled by direct measurement, not inference. [measured, 2026-08-19,
right arm `Default Device` serial 18019000365, tools in
`ros2_ws/src/touch_haptics/tools/`]

- Of eight force/torque get-parameters, only three carry signal:
  `HD_CURRENT_FORCE`, `HD_LAST_FORCE` (both echo the command), and
  `HD_LAST_JOINT_TORQUE`.
- `HD_CURRENT_JOINT_TORQUE`, `HD_CURRENT_TORQUE`, `HD_CURRENT_GIMBAL_TORQUE`,
  `HD_LAST_TORQUE`, `HD_LAST_GIMBAL_TORQUE` read **exactly zero** idle and
  under load.
- `HD_LAST_JOINT_TORQUE` is nonzero and perfectly linear in the **commanded**
  force at fixed q — |τ|/F = 149.934, 149.934, 149.897 at 0.1/0.2/0.3 N — but
  the scale changes with configuration (109.6 at a different pose). That is the
  signature of `k · J(q)ᵀ · F_commanded`, not a measurement. It reads exactly
  `[0,0,0]` at zero commanded force even with the stylus held.
- `HD_MOTOR_TEMPERATURE` is flat at ~1e-10 through 12 s of sustained 0.5 N.
  Not sensed, not modelled. Do not use it as a duty-cycle guard.
- `/touch_*/joint_torque` is a **dead topic** — identically zero, because this
  code commands Cartesian `HD_CURRENT_FORCE` and the joint-space command buffer
  is never written.

The only software signal that responds to the outside world is tracking error:
`estimated_load ≈ stiffness × |target − actual|`. It cannot separate your hand
from friction, gravity, or a workspace limit; it saturates past
`max_position_error_m` (0.04 m) and past `max_force_n`; and static friction in
the cable drive sets a noise floor of a few tenths of a newton.

Build note: OpenHaptics headers need the bare `linux` macro, so compile with
`-std=gnu++17`, not `-std=c++17`, or `hdExport.h` leaves `HDAPI` undefined.

Probe note: a probe without `hdStartScheduler` + `hdScheduleAsynchronous`
applies no force and reads stale state, reporting every parameter dead.
`hdapi_param_probe.cpp` (v1) has that bug — use v2.

---

## 4. System C — the MATLAB/Simulink observer study

Location: `/media/nural/3080A45980A42776/Users/pc1/Desktop/OmniSetup/`
(mounted Windows drive). All files dated 2024-03-07. Windows-only —
`windows.h`, `conio.h`, `.mexw64`, `.dll`.

### Contents [code]

| File | What it is |
|---|---|
| `SetUp.m` | the parameter file — observer gains, inertias, PID sets |
| `FourCh_NN_Bayesian.mdl` | the main model: 4-channel + neural net + Bayesian |
| `Omni_dataCollect.mdl.autosave` | data collection model (autosave only) |
| `Control.c` / `.mexw64` | S-function, commands `jointTorques` via OpenHaptics |
| `PhanTorque_3Dof{,_tele,_vels}.c` | Phan3DoF library S-functions, three variants |
| `emran8.c` | EMRAN neural network S-function |
| `dcsgl2.c`, `vrmult.c` | supporting S-functions, purpose not yet established |
| `plot_code.m` | plots τ_ext, q, F, P — master vs slave overlaid |
| `TrainingCode.m` | 5 lines: `X=[QR QRD]`, `FR=Fd_r(:,2:4)` |

Provenance: the `PhanTorque_3Dof*` and `Control.c` S-functions are from the
**Phan3DoF Library, Carlos Aldana (UPC), 2013, GPLv3**. `emran8.c` is
**EMRAN — Extended Minimum Resource Allocating Network, Giampiero Campa, June
2007**: an online-growing RBF network with pruning (`radius`, `E1thr`,
`E2min/max/gam`, `E3thr/gam`, `etaW/S/C`, `gamW/S/C`, `Nmax`, `prune`).

### Architectural difference that matters [code]

`Control.c` commands **joint torque** (`Haptic.jointTorques`) through
OpenHaptics. The ROS side commands **Cartesian force** (`HD_CURRENT_FORCE`).
Any port between the two crosses that boundary and needs `J(q)ᵀ` explicitly.

### The active coefficient set [code — `SetUp.m`, `%% 4Ch` block]

```matlab
g_dy = 5;      g_v = 10;                 % observer gains
g1 = 10;  g2 = 10;  g3 = 5;              % per-joint observer gains
m1 = 0.000520833;  m2 = 2.126e-4;  m3 = 3.2757e-4;   % joint inertias

P1 = 6;  I1 = 0.01;   D1 = 0.08;  N1 = 100;
P2 = 3;  I2 = 0.000;  D2 = 0.12;  N2 = 50;
P3 = 2;  I3 = 0.001;  D3 = 0.03;  N3 = 80;
```

`SetUp.m` holds **three** parameter blocks: an older commented-out set, the
active **4-channel** set above, and a commented-out **2-channel** set
(`g_dy=10`, `g1=5`, `m1=0.0050833`, `m3=5.075e-5`, `P1=50`, `D1=0.23`,
`N1=250`). Whoever tunes this next needs to know which architecture a number
belongs to. The `N` values are derivative-filter coefficients and are routinely
dropped from summaries — they are part of the tuning.

The inertias are **joint-space, order 1e-4** (kg·m²), not Cartesian masses.

### Evidence the observer is real and already running in C [code]

`plot_code.m` plots `Text1`, `Text2`, `Text3` — external torque per joint, with
**master and slave traces overlaid** — alongside `q1..q3`, and Cartesian
`F_x/F_y/F_z`, `P_x/P_y/P_z`. So the Simulink model already produces a per-joint
external-torque estimate on both sides. That is the observer output, and it is
the thing System A does not have.

`TrainingCode.m` builds `X = [QR QRD]` (reference joint position + velocity)
against `FR = Fd_r(:,2:4)` — i.e. the NN is trained to map state → force,
consistent with EMRAN acting as a force estimator inside
`FourCh_NN_Bayesian.mdl`.

---

## 5. Corrections to the circulated summary

A summary of the USB findings is in circulation. Most of it is right — the
`SetUp.m` gains reproduce exactly. These four items do not, and each would
propagate into anything built on top of it.

| Claim | Status |
|---|---|
| "X-Axis m = 0.63 kg/m, b = 4.77 N·s/m" (and Y, Z) | **Not located.** No `identification.xlsx` exists anywhere on the mounted drive (searched to depth 6, plus every `.xlsx` under `Desktop`). The only inertias found are `SetUp.m`'s joint-space 1e-4 values, three orders of magnitude smaller and in different units. `kg/m` is not a unit of mass. Treat the 26-experiment table as **on another machine and unverified**. |
| "Simulink models (`*.slx`)" | **Wrong extension.** They are `.mdl` — the pre-2012 format. `FourCh_NN_Bayesian.mdl`, `Omni_dataCollect.mdl.autosave`. No `.slx` on the drive. |
| "Where these coefficients are used: `touch_slave.cpp`, `bridge_master_slave.py`" | **False.** There is no observer, no DOB, no PID and no integral term anywhere in the ROS code — grepped. The slave controller is a pure P-D spring/damper (`touch_slave.cpp:392-394`). No mass, inertia or friction constant exists anywhere in the ROS tree. These coefficients are used **only** in System C. |
| "Wall stiffness 12 N/m" | **Stale.** 12 appears only in the outdated `SYSTEM_DESIGN_REPORT.html` table. Shipped is `REFLECT_GAIN=18.0`; argparse default is 25.0. ("Force clamp 1.5 N" is correct — that is `REFLECT_MAX_N`.) |

Also: the four `.md` files and the interactive dashboard reported as "ready in
your scratchpad directory" are **not present** in any scratchpad under this
project. If they were written, they went somewhere else.

---

## 6. Open items

1. **Real torque sensing is still unresolved.** [open] A disturbance observer
   estimates load from a model plus encoders; it does not measure force. Every
   limitation in §3 continues to apply — it cannot separate contact from
   friction, gravity, or a workspace limit. Whether to accept that or add an
   external F/T sensor (ATI Nano17/Nano25 at the tool tip, own DAQ) has been
   deferred, not decided. Deferred 2026-08-14; still open.
2. **The `identification.xlsx` coefficients are on another PC.** [open] Until
   they are in hand, the m/b values above cannot be checked, and their units
   need resolving.
3. **Nothing from System C has been ported to A or B.** [code] If porting is
   the goal, the two insertion points are: 1 kHz inside `hdCallback`, with the
   `tau_external` term between `touch_slave.cpp:400` and the `clampNorm` at
   401 so it stays inside the existing safety envelope (and
   `HD_CURRENT_JOINT_ANGLES` moved out of the `publish_telemetry_` block, since
   the observer needs `q` every frame); or 100 Hz in Python at
   `bridge_master_slave.py:449-451`, the existing damping hook, which is lower
   risk to prototype but carries the ~30 ms round trip that made damping
   unusable.
4. Before trusting any observer gain in System A, fix defects 1–3 in §1 — a
   silently wrong `slave_speed` or a stale `tracking_error` will make the
   observer look wrong when it is not.

---

## 7. Standing constraints

- **Do not apply much force to the arms.** One of them sits on a stationary
  mount. Keep test forces well under the 0.88 N nominal continuous limit;
  prefer the zero-force `hdapi_passive_read` tool. Note the master currently
  ships at `MAX_FORCE_N=2.0`, and an unvalidated observer will happily sit
  there — consider lowering it while tuning.
- **Only one OpenHaptics process may own a device.** Never run a System A
  script and a System B script at the same time.
- **This work stays on this machine.** Unreleased research; deliver documents
  as local files, do not publish or upload.
- When `HD_LAST_*` symbols are relied on, pin the OpenHaptics version — they
  are absent from the official Get Parameters tables. Here: 3.4-0 Developer
  Edition, `libHD.so.3.4.0`.
