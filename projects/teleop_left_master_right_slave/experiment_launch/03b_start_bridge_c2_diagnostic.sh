#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_POSITION=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do [[ "$arg" == --arm-position ]] && ARM_POSITION=true || { echo "[bridge-c2-diag][error] unknown argument: $arg" >&2; exit 2; }; done

# 2026-09-10: promoted to the PRODUCTION C2+C3 launcher. These are the ONLY
# real-hardware-validated C2 gains this project has (LEFT_STIFFNESS=5.0,
# LEFT_DAMPING=0.05, MAX_FORCE=0.8, LEFT_GRAVITY_COMP_N=0.55) -- deliberately
# NOT the never-hardware-tested plain production defaults (15.0/0.10/2.0),
# and not raised further just to look more "production". "Low authority" is
# a property of these numbers, not a diagnostic label; there is currently no
# higher-authority C2 configuration with real-hardware evidence behind it.
# Paired with 01b_start_master_c2_diagnostic.sh -- run that INSTEAD of
# 01_start_master.sh for a C2+C3 session, never both.
#
# left_max_offset_m tightened to 0.05m (vs C3-only's 0.15m).
reject_live_component 'bridge_master_slave.py' 'position bridge'
# 2026-09-10 fix: same missing master-planner-declaration bug as
# 03c_start_bridge_c2_only_diagnostic.sh (found via a real-hardware C2
# FAULT on that sibling launcher) -- this file is also explicitly paired
# with 01b_start_master_c2_diagnostic.sh's 5.0/0.05/0.5 diagnostic master
# gains, so it carried the same latent mismatch even though it had not
# yet been run to expose it.
# 2026-09-10 fix #2: raised LEFT_GRAVITY_COMP_N 0.45->0.55 and
# MASTER_MAX_FORCE_N 0.5->0.8 to match 01b_start_master_c2_diagnostic.sh's
# actual current MAX_FORCE_N=0.8 and the c2_y_trial2 real-hardware finding
# (same fix as 03c_start_bridge_c2_only_diagnostic.sh -- see its comment).
# 2026-09-10 fix #3, Candidate A (C2 force-budget fix): 0.15N of spring
# authority (K=5.0 * max_position_error=0.03) plus 0.55N gravity aux was
# insufficient to move the master (measured, ~0.70N total, no clamp).
# max_position_error_m raised 0.03->0.04 (spring authority 0.15N->0.20N,
# K UNCHANGED) and MASTER_MAX_FORCE_N raised 0.8->0.85 to match (0.55 aux +
# 0.20 spring + 0.10 reserve = 0.85N exactly). K, damping, and
# LEFT_GRAVITY_COMP_N are unchanged. MUST match
# 01b_start_master_c2_diagnostic.sh's actual master telemetry exactly, or
# planner_parameter_match_left faults (see run_master_left.sh's own
# LEFT_MAX_POSITION_ERROR_M and MAX_FORCE_N for that script).
# 2026-09-10: MASTER_MAX_FORCE_N/MASTER_MAX_POSITION_ERROR_M made
# OVERRIDABLE (default unchanged at Candidate A) -- see
# 01b_start_master_c2_diagnostic.sh's matching comment. Both this script's
# declared planner values and 01b's actual master telemetry must be
# overridden TOGETHER (same values) by the caller, or
# planner_parameter_match_left faults by design.
MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N:-0.85}"
MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M:-0.04}"
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true FORCE_REFLECT=false \
  SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15 \
  LEFT_MAX_OFFSET_M=0.05 LEFT_GRAVITY_COMP_N=0.55 LEFT_GRAVITY_AXIS=1 \
  MASTER_STIFFNESS_N_PER_M=5.0 MASTER_DAMPING_N_S_PER_M=0.05 MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N}" \
  MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}" \
  MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100 \
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json" \
  "${PROJECT_ROOT}/scripts/run_bridge.sh")
echo '[bridge-c2-diag] production C2+C3: C2=1 (low authority, left_max_offset=0.05m) + C3=1; C1=C4=0'
echo '[bridge-c2-diag][warning] the master will now feel a (low-gain) spring pulling toward the slave position.'
echo "[bridge-c2-diag] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_POSITION}" != true ]]; then
  echo '[bridge-c2-diag][blocked] operator confirmation flag --arm-position is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"

# 2026-09-10 fix: this launcher had the SAME missing-arm bug as
# 03_start_position_bridge.sh (confirmed by inspection before this fix --
# no experiment_launch/*.sh script ever set arm_teleop=true). Mode B here
# runs C2 (low authority) TOGETHER WITH C3, matching this file's own
# "C2=1 + C3=1; C1=C4=0" design -- so the arm sequencer's expected-channel
# check is told c3=true, unlike the pure-C2-only diagnostic.
if [[ "${DRY_RUN}" == false ]]; then
  source_ros
  echo '[bridge-c2-diag] running arm sequence (expected: C2=true, C3=true, C1=C4=false)...'
  if ! python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 false --expected-c2 true --expected-c3 true --expected-c4 false; then
    echo '[bridge-c2-diag][error] arm sequence FAILED -- teleoperation is NOT active.' >&2
    echo "[bridge-c2-diag][error] bridge process left running (unarmed) -- stop it with:" >&2
    echo "[bridge-c2-diag][error]   ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}" >&2
    exit 1
  fi
  echo '[bridge-c2-diag] arm sequence succeeded -- C2+C3 diagnostic teleoperation is ACTIVE.'
fi
