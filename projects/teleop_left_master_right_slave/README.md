# Direct teleoperation: left master to right slave

A standalone project, separate from the HIL-RL simulation work. No IsaacLab, no
ACT policy, no RL. The left Touch drives the right Touch and nothing else.

It shares one thing with the parent repo: the `touch_haptics` ROS package in
`../../ros2_ws`, which owns the OpenHaptics device drivers. Duplicating that
driver would mean two processes fighting over the same hardware, so this project
reuses it and adds only an opt-in telemetry parameter.

## Layout

```
projects/teleop_left_master_right_slave/
  README.md                      this file
  RUNNING_SCRIPTS.txt            copy-paste commands
  validation_provenance/CURRENT_READINESS.md  current C1-C4 readiness snapshot
  scripts/run_master_left.sh     terminal 1, left device as master
  scripts/run_bridge.sh          terminal 2, master deltas to slave targets
                                  (+ calibration, + optional C2)
  scripts/run_slave_right.sh     terminal 3, right device as force slave
  scripts/bridge_master_slave.py the bridge node itself (C2/C3)
  scripts/bilateral_4ch.py       the 4-channel node (C1/C4), relay-only in hardware mode
  scripts/read_calibration.py    reads the bridge's calibration file
  scripts/monitor_forces.sh      live force on both arms, side by side
  scripts/monitor_forces.py      the monitor node
  scripts/monitor_torque.sh      single-device torque and load detail
```

## Channel status (C1-C4)

This project's target architecture is 4-channel bilateral teleoperation. The
current status is maintained in
[`validation_provenance/CURRENT_READINESS.md`](validation_provenance/CURRENT_READINESS.md).
Historical audits and older trial reports are retained for provenance and must
not be read as qualification of the current checkout. In particular, the
corrected C1 sign and the current command-authority arrangement still require
fresh controlled hardware evidence before unsupervised use.

## Calibration step

Launch order now matters: **master, then bridge, then slave** (not
master/slave/bridge as in an earlier version of this project). The bridge
captures each side's anchor position as soon as that device starts
publishing, and writes both to `~/dob_logs/teleop_calibration.json` as they
become available. Starting the slave last means the bridge already has a
fresh master anchor on file by the time the slave comes up -- useful mainly
for diagnostics (the file also records `right_minus_mapped_left`, the gap
between the two anchors under the current axis map) and for seeding the
simulated digital twins (`run_master_left_sim.sh` / `run_slave_right_sim.sh`
prefer this file over reading the live pose topic directly).

This does not move real hardware -- both real devices simply stay wherever
they physically are; the file is a log, not a command.

## C2 (slave position -> master), confirmed on real hardware

Off by default. `ENABLE_C2=true` on `run_bridge.sh` makes the bridge also
publish a target to the master, derived from the slave's actual position
(the reverse direction of the usual master-drives-slave flow) -- so the
master's own spring pulls toward wherever the slave actually is, not just
wherever it was last commanded. `run_master_left.sh` needs
`ENABLE_C2=true` too, so its own spring has something to render against
(`LEFT_STIFFNESS_N_PER_M`/`LEFT_DAMPING_N_S_PER_M`, deliberately softer
than the slave's own spring -- this is a feedback channel felt by a human
hand, not a position-tracking slave).

**Confirmed on real hardware 2026-09-03**, after fixing two real gaps:
master-side gravity loading on Y (fixed via
`LEFT_GRAVITY_COMP_N`/`LEFT_GRAVITY_AXIS`, live-tuned to 0.45N) and a
workspace mismatch (`LEFT_MAX_OFFSET_M` raised from 0.08m to 0.15m to
better match the slave's own swept `MAX_OFFSET_XYZ`). Neither value has
been through a dedicated controlled sweep yet -- see FOUR_CHANNEL_TODO.md
for the full writeup and the still-open follow-up sweep.

## How it works

```
left Touch ──pose──> bridge ──target_pose──> right Touch
                       │                          │
                    axis map                 spring + damper
                    scale                    toward target
                    clutch                        │
                    rate limit              motors drive the arm
```

The right node runs a spring/damper toward the commanded target. It is the same
control path the HIL project used for its slave mirror, but the target comes
from your hand instead of from a simulation.

The bridge republishes target and enable every tick. The right node zeroes force
on any command older than `stale_timeout_s` (0.05 s), so a bridge that published
only on change would make the slave stutter.

Hold **left button 2** to clutch: the slave freezes, you reposition your hand,
and on release both sides re-anchor so nothing jumps.

With `FORCE_REFLECT=true` the loop closes the other way as well: when the slave
cannot reach its target, the master pushes back against your hand. The full
explanation of the trigger, the stability limits, and why this is an inference
rather than a measurement is in [docs/force-and-torque.html](docs/force-and-torque.html).

## Can we read the torque value of the robot?

**Partly, and not in the way that phrase usually means.** Be careful here,
because the honest answer changes what you can build on top of it.

**There is no torque sensor on this hardware.** The Phantom Omni / Touch has
optical encoders on its joints and nothing else. It measures *position*. It does
not measure force or torque anywhere, including at the stylus.

What you *can* read back from OpenHaptics:

| Attribute | What it is | Useful? |
|---|---|---|
| `HD_CURRENT_JOINT_TORQUE` | torque **commanded** to the 3 base motors | yes, as a command log |
| `HD_CURRENT_GIMBAL_TORQUE` | gimbal torque command | no, gimbal is passive here, reads zero |
| `HD_CURRENT_FORCE` | Cartesian force command | yes, same information |
| `HD_CURRENT_JOINT_ANGLES` | measured joint angles | yes, genuinely measured |
| `HD_MOTOR_TEMPERATURE` | driver thermal estimate, 0..1 | yes, watch it during stiff runs |

The giveaway is in `/usr/include/HD/hdDefines.h`: `HD_CURRENT_JOINT_TORQUE` is
listed under **both** the get parameters and the set parameters. It is the
command channel to the motors. Reading it tells you what you asked the motors to
do, which on an open-loop device is not the same as what they achieved, and says
nothing about what is pushing back.

So `joint_torque` is honest telemetry of your own command. It is genuinely
useful for logging, for verifying the force pipeline end to end, and for
checking clamps. It is not a measurement of contact.

### The one signal that does respond to the outside world

Tracking error. The slave is commanded to a target with a known spring gain. If
it cannot reach that target, something is resisting it:

```
estimated_load ≈ stiffness_n_per_m × |target − actual|
```

`monitor_torque.sh` prints this as `est_load`. Treat it as an estimate, with
real limits:

- it cannot separate your hand from friction, gravity, or the arm reaching a
  workspace limit
- it saturates: `max_position_error_m` (0.04 m) caps the error the spring sees,
  and `max_force_n` caps the output, so past that point the estimate flattens
- it is only valid while a fresh target is being published and the device is
  enabled; it publishes zero otherwise
- static friction in the Omni's cable drive is a few tenths of a newton, which
  sets the noise floor

For a real force reading you need a sensor the device does not have: an external
6-axis load cell at the stylus or the tool tip, for example an ATI Nano17, read
on its own DAQ and published as a separate topic. Nothing in software gets you
around the missing hardware.

### Enabling telemetry

Off by default so the HIL project's behavior is unchanged. Both launcher scripts
here pass `PUBLISH_TELEMETRY=true`, which adds these topics per device:

```
/touch_<side>/joint_torque         commanded, Vector3Stamped
/touch_<side>/gimbal_torque        zero on this hardware
/touch_<side>/joint_angles         measured
/touch_<side>/motor_temperature    0..1 normalized
/touch_<side>/tracking_error       target − actual, meters
```

The `touch_haptics` package must be rebuilt once for these to appear:

```bash
cd $HOME/HIL-RL/ros2_ws && source /opt/ros/humble/setup.bash && \
  colcon build --base-paths src --cmake-args -DOPENHAPTICS_ROOT=/usr
```

## Splitting this into its own repository

It lives inside HIL-RL so it can share the driver package. To lift it out:

```bash
cp -r $HOME/HIL-RL/projects/teleop_left_master_right_slave $HOME/touch-teleop
cp -r $HOME/HIL-RL/ros2_ws $HOME/touch-teleop/ros2_ws
cd $HOME/touch-teleop && git init
```

Then set `TOUCH_WS=$HOME/touch-teleop/ros2_ws` when running the scripts, since
they otherwise resolve the workspace relative to the HIL-RL root.
