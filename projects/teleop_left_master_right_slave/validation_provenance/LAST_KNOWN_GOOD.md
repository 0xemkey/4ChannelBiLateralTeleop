# Last Known Good Bilateral Configuration

Date written: 2026-09-10

Repository: `/home/nural/HIL-RL`

Current branch at reconstruction time: `c1-sign-fix-sim-validation`

Current HEAD: `4363b5c3a716d5deae04c59d931b88f8ae939ebe`

Worktree state: dirty before this reconstruction; no destructive git action was taken.

## Evidence Sources

- Historical run manifests and logs under `experiment_runs/teleop_setup/`.
- Current source tracing in `projects/teleop_left_master_right_slave/scripts/` and `ros2_ws/src/touch_haptics/src/touch_slave.cpp`.
- Scientific docs under `projects/teleop_left_master_right_slave/docs/`.

## Highest Confidence Historical Hardware Configuration

Session family:

- `four_channel_val_20260908`
- `force_reflection_val_20260908`
- `4ch_test_01`
- `fourchannel_01`

Shared manifest evidence:

- `ROS_DOMAIN_ID=0`
- master device `Left`, node `/teleop_master_left`, topics `/touch_left/*`
- slave device `Default Device`, node `/teleop_slave_right`, topics `/touch_right/*`
- DOB model files:
  - left inertia: `/home/nural/dob_logs/results/inertia_left.json`
  - left gravity: `/home/nural/dob_logs/results/gravity_left_minimal.json`
  - right inertia: `/home/nural/dob_logs/results/inertia_right.json`
  - right gravity: `/home/nural/dob_logs/results/gravity_right_minimal.json`
- DOB bandwidth baseline: `g_dis=[10,10,5] rad/s`
- DOB thresholds: `[0.07632,0.06400,0.05000] N*m`
- position gains: master `15.0 N/m, 0.10 Ns/m`; slave `35.0 N/m, 0.20 Ns/m`
- force limits: master `2.0 N`; slave `1.5 N`
- position limits recorded as master/slave `0.03/0.04 m`
- relay floor: `0.02 N*m`
- publish/bridge/relay rates: `200/100/200 Hz`

Log evidence from `four_channel_val_20260908`:

- master log: `C2 ON: stiffness=15.0 N/m damping=0.10 Ns/m`; `max_force=2.0 N`
- slave log: `stiffness=35.0 N/m damping=0.20 Ns/m`; `max_force=1.5 N`
- bridge log: `C2 ON`; `left_max_offset=0.08 m`; `axis_map=x,y,z`; `axis_scale=[1,1,1]`; `max_offset_xyz=[0.12,0.19,0.15]`; slave gravity `+0.700 N`
- bilateral log: architecture `4ch_safe`, weights `[0.5,1.0,1.0,0.3]`, `cf=1.0`, C1 shoulder mask present

Confidence: **medium-high** that this is the best reconstructed historical 4-channel run configuration. It is directly logged on real hardware. It is **not** proof of good physical performance by itself.

## Regression Finding

The current automated C2 path was using `01b_start_master_c2_diagnostic.sh` and `03b/03c_*_c2_*diagnostic.sh`, whose committed history explicitly labels:

- `LEFT_STIFFNESS_N_PER_M=5.0`
- `LEFT_DAMPING_N_S_PER_M=0.05`
- `MAX_FORCE_N=0.5` originally, later `0.85`
- `LEFT_MAX_POSITION_ERROR_M=0.04`
- `LEFT_MAX_OFFSET_M=0.05`

Those values are diagnostic/low-authority values, not the historical normal bilateral C2 configuration. Judging production bilateral behavior from that path was a configuration regression.

Recovery default now used by `run_hardware_test.sh`:

- master stiffness `15.0 N/m`
- master damping `0.10 Ns/m`
- master max force `2.0 N`
- master max position error `0.03 m`
- C2 left travel offset `0.08 m`
- master gravity support `LEFT_GRAVITY_COMP_N=0.55 N`

Confidence: **high** that K=5 was diagnostic only, based on committed launcher comments and historical logs.

## Current Intended Architecture

C2/C3 position coupling:

- Implemented in `bridge_master_slave.py`.
- C3: master pose -> bridge axis map/scale/anchor/ramp/rate limit -> `/touch_right/target_pose` -> right `touch_slave` Cartesian spring/damper -> `HD_CURRENT_FORCE`.
- C2: slave pose -> inverse axis map/scale/anchor/ramp -> `/touch_left/target_pose` -> left `touch_slave` Cartesian spring/damper -> `HD_CURRENT_FORCE`.
- In the supervised hardware architecture, `bilateral_4ch.py` should not provide position authority. Its `w2/w3` are set to zero in the new `04_start_force_relay.sh --full4ch` path.

C1/C4 force relay:

- Implemented by `bilateral_4ch.py` publishing `ForceEstimate` on `/touch_left/joint_torque_ext_cmd` and `/touch_right/joint_torque_ext_cmd`.
- C1 sign in current source is `tau_c1 = -w1 * cf * tau_e_hat * mask`.
- C4 sign is `tau_c4 = +w4 * cf * tau_h_hat`.
- `touch_slave.cpp` converts fresh `joint_torque_ext_cmd` from joint torque units (`N*m`) to Cartesian force through the Jacobian inverse-transpose path, records `relay_torque_received_nm` and `relay_force_n`, then adds it to the Cartesian device force before the final clamp.

Write-path distinction:

- `TOUCH_DOB_ENABLE_TORQUE_WRITE` gates the separate `HD_CURRENT_JOINT_TORQUE` joint-space write path.
- It does **not** gate the Cartesian relay path that ends at `HD_CURRENT_FORCE` when `joint_space_control=false`.
- The relay is still gated by `command.enabled`, `deadman_ok`, freshness, singularity, relay floor, and final force clamp.

## Unknowns Remaining

- Whether the historical `four_channel_val_20260908` behavior meets the requested 70% physical milestone: **UNKNOWN** until re-run/analyzed with mode-aware tooling.
- Whether C1 and C4 pass without joint masking on current hardware: **UNKNOWN**.
- Whether `LEFT_GRAVITY_COMP_N=0.55 N` plus recovered C2 gains gives stable, useful C2: **requires the next hardware run**.
