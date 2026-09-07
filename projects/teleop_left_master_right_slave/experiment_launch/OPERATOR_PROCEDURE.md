# Safe current-setup preparation procedure

This procedure prepares the two physical device nodes for DOB observation. It
does not qualify the DOB as external torque and does not enable C1/C4.

## Fixed topology

| Component | Identity | Node | Mode at zero-force startup | Principal outputs | Rate |
|---|---|---|---|---|---:|
| left master | `Left`, serial 18019000358, channel 300 | `/teleop_master_left` | Cartesian path, disabled; DOB observation enabled | `/touch_left/pose`, `/touch_left/joint_angles`, `/touch_left/dob_diagnostics`, `/touch_left/force_ext_hat`, `/touch_left/applied_force` | 200 Hz publication; DOB at HDAPI rate |
| right slave | `Default Device`, serial 18019000365, channel 301 | `/teleop_slave_right` | Cartesian path, disabled; DOB observation enabled | corresponding `/touch_right/*` topics | 200 Hz publication; DOB at HDAPI rate |
| position bridge | no hardware | `/touch_master_slave_bridge` | not started until explicit position arming | `/touch_right/target_pose`, `/touch_right/enable`; C2 topics only when enabled | 100 Hz |
| force relay | no hardware | `/bilateral_4ch` | blocked: C1/C4 not qualified | `/touch_left/right/joint_torque_ext_cmd`, `/bilateral_status` | 200 Hz |

Initial channel state is C1=C2=C3=C4=0. The explicitly armed position mode is
C3=1 with C1=C2=C4=0. `joint_space_control=false` on both device nodes.

The launchers use `RIGHT_TTY=MAPPED_TTY=/dev/ttyACM1`, relying on the active
OpenHaptics configurations' distinct channels. They never mount, rebind, or use
`sudo`.

## Terminal order

Choose one session ID and use it unchanged in every terminal.

1. Preflight:

   ```bash
   cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
   projects/teleop_left_master_right_slave/experiment_launch/00_preflight.sh \
     --session dob_setup_001 --notes "DOB observation only"
   ```

2. Start the disabled left master:

   ```bash
   cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
   projects/teleop_left_master_right_slave/experiment_launch/01_start_master.sh \
     --session dob_setup_001
   ```

3. Start the disabled right slave:

   ```bash
   cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
   projects/teleop_left_master_right_slave/experiment_launch/02_start_slave.sh \
     --session dob_setup_001
   ```

4. Verify status before any arming:

   ```bash
   cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
   projects/teleop_left_master_right_slave/experiment_launch/05_show_status.sh \
     --session dob_setup_001
   ```

   Both device nodes must be running and both pose/DOB topics must be fresh.
   Confirm `joint_space_control=false`, no bridge node, no force relay node,
   and zero commanded `applied_force`. `applied_force` is command telemetry,
   not sensor-measured contact force.

5. Stop here for DOB-observation setup. Starting the position bridge is a
   separate hardware arming action. Its exact command is:

   ```bash
   cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
   projects/teleop_left_master_right_slave/experiment_launch/03_start_position_bridge.sh \
     --session dob_setup_001 --arm-position
   ```

   Expected behavior: C3 Cartesian position control becomes active on the
   right device, with 35 N/m stiffness, 0.20 N*s/m damping, 1.5 N Cartesian
   clamp, 0.04 m position-error clamp, 0.5 m/s target limit, and 0.55 N Y-axis
   gravity-support command. C1, C2, C4, and tracking-error force reflection
   remain off. Keep clear of both workspaces before issuing it.

## Shutdown

Normal shutdown and emergency software stop use the same targeted command:

```bash
cd <path to this repo root>  # e.g. ~/HIL-RL, or wherever this portable copy was placed
projects/teleop_left_master_right_slave/experiment_launch/99_stop_setup.sh \
  --session dob_setup_001
```

It sends TERM only to process groups recorded in that session. It never uses
`pkill` or `killall`. Device command watchdogs zero stale force after 0.05 s.
If a recorded process remains after five seconds, the script reports its PID
instead of issuing an automatic KILL; inspect that PID before further action.

For an immediate physical emergency, release the stylus, move clear, execute
the stop command from a prepared terminal, and use the device/system hardware
power stop if software does not respond. Do not restart until the cause is
understood.

## Force relay gate

`04_start_force_relay.sh` exits nonzero unconditionally. The current DOB has
not passed external-torque qualification, so there is no approved C1/C4
hardware mode for this preparation stage.

## Existing and missing future-recording telemetry

Already published:

- Cartesian pose and target pose on both sides.
- Joint angles.
- DOB `tau_dis_hat`, reconstructed command torque, differentiated velocity,
  contact gate and configured state.
- Candidate `force_ext_hat`: raw/sign-inverted/deadbanded joint torque,
  confidence and validity.
- Received relayed joint torque.
- Post-Cartesian-relay/post-clamp commanded `applied_force`.
- Previous-frame driver force command, tracking error and motor-command
  telemetry.
- `/bilateral_status` channel weights, enable reasons, thresholds, energy-tank
  and safety-latch state when the relay exists.

Still missing or incomplete for the later recorder:

- Explicit mapped anchor-relative position and position-error topics (currently
  derived offline or inside the bridge).
- Separate position-controller, gravity-support and C1/C4 Cartesian force
  contributions.
- Explicit Cartesian and joint saturation flags and remaining headroom.
- A separately named, qualified `tau_internal` and corrected `tau_ext` path.
- Reset/priming state and source timestamps at the 1 kHz observer rate.
- Simulator or sensor ground-truth external torque.

Do not guess these missing topic names in a recorder; add explicit telemetry
before the recording phase.

