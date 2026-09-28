# Telemetry contract (2026-09-10, items 3 + 12)

For the Codex session building `projects/teleop_left_master_right_slave/
visualization/` (simulation UI, synthetic visualization, replay). This
document is the STABLE INTERFACE this track commits to — Codex should read
from these topics/fields, not from source, and should treat any field not
listed here as unstable/internal. This directory (`hardware_validation/`)
owns no dashboard and does not overlap `visualization/`.

## 1. Force decomposition — `CartesianControllerTelemetry`

One message per arm (`/touch_left/cartesian_controller_telemetry`,
`/touch_right/cartesian_controller_telemetry`), published every control
tick. All `float64[3]` fields are `[x, y, z]` in the arm's own Cartesian
frame (device-native for the slave; `axis_map`/`axis_scale`-transformed
for the master — see `frozen_config.py`'s `axis_map`).

**The additive force chain, in order** (each stage's output feeds the next):

| stage | field | meaning |
|---|---|---|
| 1 | `spring_force_n` | PD spring term toward `target_position_m` |
| 2 | `damping_force_n` | PD damping term |
| 3 | `auxiliary_force_n` | gravity comp (`slave_gravity_comp_n`/`left_gravity_comp_n`) — constant on the configured `*_gravity_axis`, zero elsewhere |
| 4 | `force_before_primary_clamp_n` | spring+damping+auxiliary, pre-clamp |
| 5 | `force_after_primary_clamp_n` | after `max_force_n` clamp (position-channel budget only — does NOT yet include C1/C4 relay) |
| 6 | `relay_torque_received_nm` | raw joint-torque estimate received from `bilateral_4ch.py` (only meaningful if `relay_fresh` — see the `relay_valid` caveat below) |
| 7 | `relay_force_n` | (6) converted to Cartesian and weighted — this IS the C1 (master arm) or C4 (slave arm) contribution |
| 8 | `force_before_final_clamp_n` | (5) + (7), pre-final-clamp |
| 9 | `applied_force_n` | **what was actually written to `HD_CURRENT_FORCE`** — the ground truth for "what force did the operator feel" |

**Do not sum (5)+(7) yourself and call it applied** — always read `applied_force_n`
directly; the final clamp in stage 9 can differ from the naive sum.

**Validity/freshness flags** (all `bool`): `enabled`, `deadman_ok`,
`target_valid`, `target_fresh`, `force_valid`, `force_fresh`, `relay_valid`,
`relay_fresh`, `relay_applied` (relay was actually summed in this tick —
false if `--c1-enable`/`--c4-enable` is off on the bridge even if a relay
message arrived), `tracking_valid`, `position_error_clamped`,

**`relay_valid` caveat (found 2026-09-10, real hardware):** `touch_slave.cpp`
sets `command_.joint_torque_ext_valid = true` ONCE, inside the
`joint_torque_ext_cmd` subscription callback, and never resets it anywhere
in the file — it is a permanent, process-lifetime latch ("has this
process ever received a structurally valid relay message since it
started"), NOT a live-state flag. A long-running real driver process that
has EVER been paired with a `bilateral_4ch.py` C1/C4 session will read
`relay_valid=True` forever afterward, regardless of current state. Use
`relay_fresh` (genuinely time-gated against the last message's timestamp)
for "is a relay stream currently arriving," and `relay_applied` for "was a
nonzero relayed force actually summed this tick." Do not gate any
safety/validity decision on `relay_valid` alone.
`primary_force_clamped`, `final_force_clamped`, `joint_path_commanded`
(always false on hardware builds — `TOUCH_DOB_ENABLE_TORQUE_WRITE=0`; the
C1/C4 relay goes through the Cartesian path above, not this one),
`cartesian_write_active`, `near_singular`.

**Static params echoed per-message** (so a recorded run is self-describing
without cross-referencing a separate manifest): `stiffness_n_per_m`,
`damping_n_s_per_m`, `max_position_error_m`, `max_force_n`, `stale_timeout_s`.

## 2. Channel/bilateral state — `BilateralStatus` (`/bilateral_status`)

Published by `bilateral_4ch.py` only (absent entirely if it is not
running — a stale/missing subscription is what `--c1-enable`/
`--c4-enable` on the bridge treat as fail-closed, per
`COMMAND_AUTHORITY_ADR.md`). Array index 0..3 = C1..C4 throughout
(Lawrence numbering: C1 slave-force→master, C2 slave-position→master,
C3 master-position→slave, C4 master-force→slave — C2/C3 entries in these
arrays are present for schema uniformity but not meaningful, since C2/C3
are driven by the bridge's own position blend, not this message).

- `architecture` (string): `"4ch"` / `"4ch_safe"` / `"pp"` / `"pf"` /
  `"fp"` / `"unilateral"` / `"custom"`.
- `channel_weight[4]` (float64, post-ramp): the commanded weight.
- `channel_active[4]` (bool): **the direct answer to "is this channel
  doing anything right now"** — true only if weight>0 AND source fresh
  AND estimate valid. Prefer this over reconstructing it from weight.
- `channel_reason[4]` (uint8): 0 OK, 1 ZERO_WEIGHT, 2 STALE, 3 INVALID,
  4 BORROWED_REFUSED, 5 TANK_EMPTY, 6 TRIPPED, 7 CLUTCHED.
- `safety_state` (uint8): 0 NOMINAL, 1 DERATED, 2 TRIPPED (latched, needs
  explicit re-arm). This is what the bridge's `--c1-enable`/`--c4-enable`
  compose into the single master `/enable` decision.
- `c1_joint_mask[3]` ([waist, shoulder, elbow], float64, 1.0/0.0): which
  joints currently contribute to the C1 relay. Shoulder is masked by
  default (`SAFETY_ONLY`/`LEGACY_COMMISSIONING_PRESET` classification —
  see the assumption register referenced in the .msg file; NOT evidence
  of a defect on its own).
- `passivity_energy_j`, `duty_cycle_nm`, `master_budget_used`,
  `slave_budget_used`, `c1_zero_crossing_hz`, `effective_threshold_c1_nm`,
  `effective_threshold_c4_nm`: diagnostic/tuning fields, not required for
  a basic replay, safe to surface as optional detail panels.
- `clutch_active`, `grip_detected`: bool.

## 3. Stage/ownership state — `BridgeDiagnostics`

Published by `bridge_master_slave.py` only, always present when the
bridge is running (regardless of channel gating).

- `bridge_state` (string): the state-machine name — `WAIT_FOR_POSES` /
  `HOLD_STILL` / `CAPTURE_ANCHORS` / `VALIDATE_ALIGNMENT` / `READY` /
  `GRAVITY_RAMP_UP` / `GRAVITY_SETTLE` / `COUPLING_RAMP_UP` / `ACTIVE` /
  `COUPLING_RAMP_DOWN` / `GRAVITY_RAMP_DOWN` / `FAULT`.
- `c2_enabled`, `c3_enabled`, `c1_enabled`, `c4_enabled` (bool): the
  CONFIGURED gate state (from CLI flags) — not the same as
  `BilateralStatus.channel_active`, which is the LIVE per-tick answer.
  Use both together: `c1_enabled=true` + `BilateralStatus.channel_active[0]
  =false` means "C1 is turned on but not currently doing anything"
  (e.g. `ZERO_WEIGHT` or `STALE`), a materially different state from
  `c1_enabled=false` ("C1 was never turned on").
- `master_travel_limit_flag`, `slave_travel_limit_flag` (bool): per-tick
  clamp-engaged flags (C2/C3 respectively).
- `coupling_ramp_held` (bool): planner held `alpha` this tick
  (force/position budget binding — not a fault, just "not ramping right now").
- `force_budget_telemetry_fresh_left/right` (bool).
- `planner_parameter_match_left/right` (bool): false is what forces a
  `FAULT` on a config mismatch between the bridge and the connected node.
- `master_accepted_force_budget_margin_n` / `slave_accepted_force_budget_margin_n`
  and their `*_position_error_margin_m` / `*_candidate_*` equivalents:
  the Part-1-era margin fields (see the earlier round of this task) — the
  `_accepted_` variant is the value that actually gated the last state
  transition; the `_candidate_` variant is what was being evaluated when
  it may not have passed yet. Read `_accepted_` for "what happened",
  `_candidate_` only for live debugging of a stuck gate.
- `gravity_settle_master_speed_mps` / `gravity_settle_slave_speed_mps`,
  `gravity_settle_master_pose_drift_m` / `gravity_settle_slave_pose_drift_m`,
  `gravity_settle_quiet_window_accumulated_s`,
  `gravity_settle_accepted_master_pose_drift_m` /
  `gravity_settle_accepted_slave_pose_drift_m`: the H2/`GRAVITY_SETTLE`
  gate's own live metrics — meaningful only while `bridge_state` is
  `GRAVITY_RAMP_UP`/`GRAVITY_SETTLE`.

## 4. What is a stable schema field vs. what is not

Stable (safe for Codex to build persistent UI/replay around): every field
named explicitly above, and the three message types themselves
(`CartesianControllerTelemetry`, `BilateralStatus`, `BridgeDiagnostics`)
as a whole — new fields may be appended, existing fields will not be
renamed or repurposed without updating this document in the same change.

Not yet stable / explicitly out of scope for this pass: a unified
single-topic "replay record" schema combining all three message types
plus deadman-button state into one recorded row. The existing recorder
(`hardware_experiments/2026-09-09_.../plot_y_axis_validation.py` and its
upstream capture scripts) subscribes to all three separately and
timestamps by `header.stamp`; Codex's replay tooling should do the same
(join on `header.stamp`, nearest-neighbor within one tick) rather than
wait for a merged schema this pass did not build.
