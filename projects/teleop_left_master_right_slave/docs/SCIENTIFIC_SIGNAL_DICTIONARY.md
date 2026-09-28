# Scientific signal dictionary (Phase 3)

Date: 2026-09-08. Defines every signal used anywhere in this project's
force/position analysis, its physical meaning, frame, units, sign,
clamp state, and — critically — whether it is MEASURED, ESTIMATED,
COMMANDED, DERIVED, or HYPOTHETICAL. No signal may be used across a
transparency metric without checking this table first (per Phase 12's
graphing rules).

| Signal | Category | Frame | Units | Pre/post clamp | Meaning |
|---|---|---|---|---|---|
| `master_position` / `slave_position` | MEASURED (device encoder-derived) | device→ROS Cartesian | m | n/a | Actual arm-tip Cartesian position |
| `master_velocity` / `slave_velocity` | DERIVED (finite-difference of position) | same | m/s | n/a | Not independently measured; no dedicated velocity sensor |
| `master_DOB_external_force_estimate` / `slave_DOB_external_force_estimate` (`force_ext_hat`) | ESTIMATED | joint-torque space, per-joint | N·m | pre-deadband (raw) or post-deadband (`tau_ext_deadbanded_nm`, what's actually relayed) — **must specify which** | DOB's model-based external-torque estimate; **not a physical measurement**, no force sensor exists on this rig |
| `master_relay_force` / `slave_relay_force` (`tau_c1`/`tau_c4`, then `joint_torque_ext_cmd`) | COMMANDED (derived from ESTIMATED) | joint-torque space | N·m | pre-final-clamp | What one side's DOB estimate becomes after weighting/masking/deadband, about to be added to the other side's rendered force |
| `master_total_applied_force` / `slave_total_applied_force` (`applied_force_n`) | DERIVED | device Cartesian, post-mapping | N | **post-both-clamps** | Sum of position-tracking spring + damping + gravity/aux + relay contribution — **the actual real rendered force**, but NOT decomposed; conflates tracking-spring and interaction-force components (see `FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md` §5). **Must not be used alone as a transparency proxy.** |
| `ground_truth_sim_force` (`tau_ext_true`, sim only) | MEASURED (within simulation only) | joint-torque space | N·m | n/a | `hd_sim`'s own internally-tracked true contact torque (`J^T(q_true)*F_env_true`); exists ONLY in `touch_slave_sim` runs via `TOUCH_SIM_TRUTH_CSV`, has no real-hardware equivalent (no force sensor) |
| `gravity_component` | COMMANDED (constant, open-loop) | device Cartesian | N | pre-primary-clamp | The 0.55N (script default) / 0.70N (empirically preferred, per prior sweep) constant Y-axis auxiliary force; not a measurement of actual gravitational load, a compensating constant |
| `spring_component` (`Kp * err`) | DERIVED (reconstructed offline; not separately telemetered) | device Cartesian | N | pre-primary-clamp | Position-tracking proportional term; reconstructed in this session's `root_cause_plots.py` from logged `c3_position_error` × configured stiffness, since it is not itself a logged column |
| `damping_component` (`-Kd * v`) | DERIVED (reconstructed offline) | device Cartesian | N | pre-primary-clamp | Same caveat as spring_component — reconstructed, not logged directly |
| `relay_component` (Cartesian-converted `tau_c1`/`tau_c4`) | DERIVED (from COMMANDED `joint_torque_ext_cmd` via `cartesianForceFromJointTorque`) | device Cartesian, in the RECEIVING side's own pose frame | N | pre-final-clamp | The only telemetry-derivable quantity that isolates "what the relay contributed," as used in `plot6_transparency_proxy.png` |
| `master_travel_limit_flag` / `slave_travel_limit_flag` | MEASURED (boolean, from the position-clamp logic) | n/a | bool | n/a | True whenever `err` was clamped to `max_position_error_m` — marks a window where the tracking-spring equation is saturated, not linear |
| `master_force_saturation_flag` / `slave_force_saturation_flag` | MEASURED (boolean) | n/a | bool | n/a | True whenever the final force clamp (`max_force_n`) was active |
| `c1_active` / `c4_active` | MEASURED (boolean, from `bilateral_4ch.py`'s own gating logic) | n/a | bool | n/a | True when a channel's effective weight is nonzero AND its `REASON_*` code is `REASON_OK` |
| `master_q_*` / `slave_q_*` (waist/shoulder/elbow) | MEASURED (joint encoders) | joint space | rad | n/a | Raw joint angles, used for Jacobian evaluation |
| `HYPOTHETICAL: shoulder-unmasked relay` | **HYPOTHETICAL — explicitly labeled in every plot/table that uses it** | joint-torque space, then Cartesian | N·m / N | n/a | `0.5 × master_force_ext_hat_shoulder`, an offline reconstruction of what an unmasked C1 relay *might* deliver, reusing the other two joints' observed weight — **never measured, never rendered on real or simulated hardware** |

## Suitability for transparency metrics

| Signal | Suitable for `e_F` (force transparency)? |
|---|---|
| `applied_force` (total) | **No** — conflates tracking spring with relay, per Phase 3's own primary rule |
| `force_ext_hat` (raw DOB estimate, either side) | Only as an estimator-quality signal (Phase 4), not as `F_s` itself — it's pre-relay, pre-weighting |
| `relay_component` (Cartesian-converted `joint_torque_ext_cmd`) | **Yes, with the pose-mismatch caveat** — this is what `plot6_transparency_proxy.png` and the sign-audit test both use |
| `ground_truth_sim_force` | Yes, but simulation-only — no physical-hardware equivalent exists (no force sensor), so any conclusion built on it needs the same `REQUIRES_FUTURE_HARDWARE_VALIDATION` caveat as the sign audit |

## Stop condition

Documentation only — no hardware or simulation execution in this file.
