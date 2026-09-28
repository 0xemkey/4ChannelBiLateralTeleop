# Scientific hardware code readiness report

Date: 2026-09-08. Mode: source modification / static validation only.
**Zero physical hardware interaction.** Branch: `c1-sign-fix-sim-validation`
(not merged into `hardware-testing-2026-09` or any branch used for real
teleoperation).

**Final status: `CODE_READY_FOR_FUTURE_HARDWARE_VALIDATION`.**
**NOT `HARDWARE_VALIDATED` — that claim is not made anywhere in this
document or its dependencies.**

## 1. What was changed

Six files, all reviewed, all statically/sim-validated:

| File | Change |
|---|---|
| `scripts/bilateral_4ch.py` | C1 sign correction (Goal 1); shoulder-mask SAFETY_ONLY/LEGACY_COMMISSIONING_PRESET relabeling (Goal 2); C1/C4 gain BASELINE_ONLY relabeling (Goal 3); SAFETY ENVELOPE vs PERFORMANCE CONTROLLER section labels (Goal 4); publish new `c1_joint_mask` telemetry field (Goal 5) |
| `ros2_ws/src/touch_haptics/msg/BilateralStatus.msg` | New `float64[3] c1_joint_mask` field (Goal 5) |
| `ros2_ws/src/touch_haptics/sim/tests/test_c1_relay_live.py` | One assertion (`ratio` bound) and its surrounding comment updated to match the corrected sign — a mechanical consequence of the Goal 1 fix, not an independent change |
| `scripts/run_master_left.sh` | TO_BE_OPTIMIZED label added to C2 gains (Goal 7); no numeric change |
| `scripts/run_slave_right.sh` | TO_BE_OPTIMIZED label added to C3 gains (Goal 7); no numeric change |
| `docs/CONTROL_ARCHITECTURE_MAPPING.md`, `docs/FOUR_CHANNEL_ROOT_CAUSE_INVESTIGATION.md` | Update notes pointing to the sign fix, preserving the original point-in-time trace (Goal 1 consistency) |

New files: `docs/SCIENTIFIC_SIGN_FRAME_DEFINITIONS.md` (Goal 6),
`docs/SCIENTIFIC_HARDWARE_CODE_READINESS.md` (this file, Goal 10). The
assumption register (`docs/SCIENTIFIC_CONTROLLER_ASSUMPTION_REGISTER.md`)
was updated in place (Goal 9).

**No C++ source file (`touch_slave.cpp`, `touch_dob.hpp`,
`hd_sim_api.cpp`) was modified.** The C1 sign lives entirely in the
Python relay script; `touch_slave.cpp` only ever receives whatever
`joint_torque_ext_cmd` carries and was never the source of the
discrepancy.

## 2. Correctness defects fixed

**One**: the C1 relay sign (`bilateral_4ch.py:450`, was
`tau_c1 = [w1_eff * ...]`, now `tau_c1 = [-w1_eff * ...]`), restoring
agreement with the module's own docstring and the user-decided scientific
target `F_m ≈ -F_s`. Simulation-validated: `cos_sim` went from `+0.87`
(same-direction, wrong) to `-0.87` (opposing, correct) against `hd_sim`
wall-contact ground truth. Full detail:
`docs/SCIENTIFIC_SIGN_FRAME_AUDIT.md`.

**No other correctness defect was found or fixed this session.** C4's
sign, the device-frame mapping (§1 of the sign/frame doc), the DOB's own
sign, and the master/slave position mapping were all checked and found
consistent — see `docs/SCIENTIFIC_SIGN_FRAME_DEFINITIONS.md` for the
full per-item audit trail.

## 3. Heuristics demoted to baseline-only (no numeric change)

- C1 weight (0.5) and C4 weight (0.3): `PRESETS["4ch_safe"]`, now
  commented `BASELINE / LEGACY COMMISSIONING VALUES — NOT SCIENTIFIC
  OPTIMA`.
- Shoulder C1 mask default (`--c1-disable-joints=shoulder`): now labeled
  `SAFETY_ONLY / LEGACY_COMMISSIONING_PRESET` in both the argparse help
  text and an inline comment at its construction site. The controller
  architecture already supports all 3 joints (`--c1-disable-joints=""`
  runs the full path) — this was true before this session too; what
  changed is that the DEFAULT is now explicitly labeled as a
  commissioning choice, not silently presented as "the" scientific
  configuration.
- C2/C3 stiffness/damping (`run_master_left.sh`, `run_slave_right.sh`):
  now labeled `TO_BE_OPTIMIZED`; values unchanged; already configurable
  via env vars (this was already true — labeling only).

**No new numeric default was invented anywhere.** Per Goal 2's explicit
instruction, no replacement shoulder weight was chosen; per Goal 3, no
replacement C1/C4 gain was chosen.

## 4. Safety logic preserved (verified unweakened)

Checked and confirmed unchanged: `MAX_FORCE_N`/`REFLECT_MAX_N`/
`dob_torque_f_max_n` device-force ceilings, `MAX_OFFSET_M`/
`MAX_OFFSET_XYZ`/`LEFT_MAX_OFFSET_M` workspace limits, the energy tank
(`--tank-capacity-j`, `--tank-recharge-time-s`), the oscillation abort
(`--trip-pp-nm`, `--min-oscillation-hz`), and `--stale-timeout-s`. These
are now explicitly labeled `SAFETY ENVELOPE` in `bilateral_4ch.py`'s
argparse block, distinguished from the `PERFORMANCE CONTROLLER` block
immediately above it (Goal 4). None of their numeric values were
touched. The full regression suite (§6) exercises the oscillation-abort
and tank logic directly and confirms no behavioral change.

## 5. What remains scientifically unoptimized

Per the assumption register's Goal 9 status table: C1/C4 gains
(`BASELINE_ONLY`), C2/C3 asymmetry (`TO_BE_OPTIMIZED`), the shoulder
mask's specific mitigation form (`SAFETY_ONLY`/graded alternatives
`REQUIRES_USER_DECISION`), per-joint force-reflection weighting beyond
the binary mask (`REQUIRES_USER_DECISION` — doesn't exist yet), the
arbitrary deadband floor `kMinRelayTorqueNormNm=0.02` (`BASELINE_ONLY`).
None of these were tuned, re-derived, or given new numeric values this
session, per the task's explicit stop conditions.

## 6. What is statically verified

- `colcon build --packages-select touch_haptics` succeeds after the
  `BilateralStatus.msg` schema change (compile-only; confirmed via
  output showing "Finished <<< touch_haptics", no errors, only
  pre-existing unrelated compiler warnings).
- `ldd` on the rebuilt `touch_slave_sim` confirms it still links
  `libhd_sim.so`, not `libHD.so` — the message schema change did not
  alter which library any binary links against.
- `grep` confirms no `/dev/ttyACM*`/`/dev/ttyUSB*`/HDAPI-device-open call
  was introduced anywhere in this session's diffs.
- Source-level consistency check (Goal 1): grepped the full repo for
  stale `+w1`/mirror-convention prose after the fix; found two
  historical documents with the old trace, both now carry explicit
  update notes pointing to the fix rather than being silently left
  stale or rewritten as if the history never happened.

## 7. What is simulation-validated

- The corrected C1 sign: `cos_sim = -0.87` against `hd_sim` wall-contact
  ground truth (`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`).
- No regression across 31 hardware-independent tests spanning relay
  magnitude, gating, Cartesian-relay deadband, all channel presets
  (`unilateral`/`pp`/`pf`/`4ch`/`4ch_safe`), oscillation-guard logic
  (11 sub-tests), applied-force relay inclusion, torque headroom, stale/
  gated-write behavior, the DOB's own sign, C4 tracking, and stability
  envelope — run twice (before and after the `BilateralStatus.msg`
  schema change), both times 31/31 passing.
- The new `c1_joint_mask` telemetry field: confirmed to publish and be
  consumed without error across the same regression run (the schema
  change alone would have broken message (de)serialization if malformed;
  it did not).

## 8. What still requires future physical validation

**`REQUIRES_FUTURE_HARDWARE_VALIDATION`, explicitly, for:**
- Whether the corrected C1 sign, run on the real Touch devices, produces
  the intended opposing-force feel — `hd_sim`'s fidelity to the physical
  device's own `HD_CURRENT_FORCE` sign convention has only been
  indirectly checked (FK-residual work), never with a dedicated
  force-sign hardware test.
- Whether the device-frame mapping (§1 of
  `SCIENTIFIC_SIGN_FRAME_DEFINITIONS.md`) and Jacobian frame
  (`touch_slave.cpp:1174-1177`'s own long-standing "NOT yet verified"
  note) hold on physical hardware — pre-existing open items, unrelated
  to and not resolved by this session's work.
- Any claim about operator-perceived transparency, "feel," or
  qualitative bilateral performance — nothing in this document claims
  the fix makes teleoperation "better" in a felt sense, only that it now
  matches the declared mathematical target in simulation.
- The new `c1_joint_mask` telemetry field's behavior under real DOB
  noise/timing — simulation-validated only, per §7.

## 9. Exact files changed

See §1's table for the full list with per-file descriptions. Git diff
stat (this session's chain of work, excluding unrelated
`experiment_launch/` housekeeping already present before this task):
```
docs/CONTROL_ARCHITECTURE_MAPPING.md                | 10 ++++
scripts/bilateral_4ch.py                            | 66 +++++++++++++++++++++-
scripts/run_master_left.sh                          |  5 ++
scripts/run_slave_right.sh                           |  3 +
ros2_ws/.../msg/BilateralStatus.msg                  | 15 +++++
ros2_ws/.../sim/tests/test_c1_relay_live.py          | 18 ++++--
6 files changed, 110 insertions(+), 7 deletions(-)
```
Plus 2 new documentation files
(`SCIENTIFIC_SIGN_FRAME_DEFINITIONS.md`, this file) and 1 new test file
from the prior task
(`analysis/scientific_4ch/test_c1_sign_vs_ground_truth.py`, already
committed to context in the prior turn).

## 10. Exact parameters still awaiting user/scientific decisions

| Parameter | Decision needed |
|---|---|
| Shoulder mask's mitigation form | Hard mask (current) vs. graded alternative (fixed reduced weight / confidence-weighted / frequency-filtered / improved DOB) — 6 candidates listed in the assumption register, none evaluated |
| Per-joint force-reflection weighting concept | Whether to introduce one at all (currently binary mask only) |
| C1/C4 numeric gains | Any value beyond the 0.5/0.3 baseline — blocked on Phase 5/6's objective-function definition (`J_pos`/`J_f` weighting), itself a `REQUIRES_USER_DECISION` per the design-process doc |
| C2/C3 numeric gains | Same — blocked on Phase 5's `J_pos` weighting decision |
| Whether to merge `c1-sign-fix-sim-validation` | Not yet asked this session |
| DOB Phase 4 sweep | Whether to authorize the N≥10-20 repeated-run characterization flagged as incomplete in `docs/SCIENTIFIC_DOB_VALIDATION.md` |

## Stop condition

`hardware_interaction = 0`, `hardware_commands = 0`, `real_device_access = 0`,
`physical_experiments = 0` throughout. No gain, mask weight, DOB
parameter, or safety limit was tuned or changed in value. No physical
device was started, opened, or commanded. This report's strongest claim
is **"validated against simulator ground truth and static compilation"**
— never physical hardware validation.
