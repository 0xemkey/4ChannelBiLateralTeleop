#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_POSITION=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do [[ "$arg" == --arm-position ]] && ARM_POSITION=true || { echo "[bridge-c2-only-diag][error] unknown argument: $arg" >&2; exit 2; }; done

# 2026-09-10, C2-only real-hardware validation prep (Task E). Distinct
# from 03b_start_bridge_c2_diagnostic.sh's "Mode B" (C2+C3 together,
# per its own docstring): this is the genuinely ISOLATED C2-only
# configuration -- C3_ENABLE=false, requiring run_bridge.sh's own
# 2026-09-10 --c3-enable wiring (previously run_bridge.sh had no way to
# turn C3 off at all; bridge_master_slave.py's own default is c3=True).
#
# Goal (per the reverse-position-channel task): slave physical movement
# -> slave pose -> C2 -> master target -> master Cartesian controller ->
# master physical movement, with NOTHING else (C1=C3=C4=false) able to
# contribute, so a felt master response is unambiguously attributable to
# C2 alone.
#
# Same low-authority gains as 03b/01b (LEFT_MAX_OFFSET_M=0.05,
# LEFT_STIFFNESS/DAMPING set on the MASTER driver side by
# 01b_start_master_c2_diagnostic.sh, which must be started INSTEAD of
# 01_start_master.sh for this session, same as Mode B).
reject_live_component 'bridge_master_slave.py' 'position bridge'
# 2026-09-10 fix: the bridge's planner_parameter_match_left check faulted
# on the first real-hardware C2-only attempt (session
# 20260910T120450Z_c2_baseline) because run_bridge.sh had no wiring for
# the master planner declaration at all and silently used its production
# defaults (15.0 / 0.10 / 2.0), which do not match the diagnostic master
# driver 01b_start_master_c2_diagnostic.sh actually starts (5.0 / 0.05 /
# 0.5). MASTER_MAX_POSITION_ERROR_M is left at run_bridge.sh's own
# default (0.03) since 01b does not override LEFT_MAX_POSITION_ERROR_M
# either -- both sides already agree on that one field.
# 2026-09-10 fix #2: real-hardware C2 Y-trial (c2_y_trial2) showed master
# Y position NEVER moving despite a large commanded offset. Traced to
# total applied Y force capping at only 0.3N (spring maxed at
# stiffness=5.0 * max_position_error_m=0.03 = 0.15N by the controller's
# own clamp, by design, + aux gravity-comp only 0.15N observed) -- not
# enough to overcome gravity/friction on the same Y axis where the SLAVE
# needed a controlled-sweep-validated 0.55N (SLAVE_GRAVITY_COMP_N,
# run_bridge.sh). Raised to match; still leaves headroom under
# MASTER_MAX_FORCE_N=0.8 (0.15 spring + 0.55 aux = 0.70N < 0.8N cap).
# 2026-09-10 fix #3, Candidate A (C2 force-budget fix): with
# LEFT_GRAVITY_COMP_N=0.55 restored above, 0.15N spring + 0.55N aux =
# 0.70N (measured) still moved nothing -- the spring's own 0.15N ceiling
# (K=5.0 * max_position_error=0.03) was the real limiter, not
# MASTER_MAX_FORCE_N. Raising max_force_n alone cannot add spring
# authority; max_position_error_m raised 0.03->0.04 instead (spring
# authority 0.15N->0.20N, K UNCHANGED), and MASTER_MAX_FORCE_N raised
# 0.8->0.85 to match the new required budget (0.55 aux + 0.20 spring +
# 0.10 reserve = 0.85N exactly). MUST match
# 01b_start_master_c2_diagnostic.sh's actual master telemetry exactly.
# 2026-09-10: MASTER_MAX_FORCE_N/MASTER_MAX_POSITION_ERROR_M made
# OVERRIDABLE (default unchanged at Candidate A) -- see
# 01b_start_master_c2_diagnostic.sh's matching comment.
MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N:-0.85}"
MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M:-0.04}"
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true C3_ENABLE=false FORCE_REFLECT=false \
  SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15 \
  LEFT_MAX_OFFSET_M=0.05 LEFT_GRAVITY_COMP_N=0.55 LEFT_GRAVITY_AXIS=1 \
  MASTER_STIFFNESS_N_PER_M=5.0 MASTER_DAMPING_N_S_PER_M=0.05 MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N}" \
  MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}" \
  MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100 \
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json" \
  "${PROJECT_ROOT}/scripts/run_bridge.sh")
echo '[bridge-c2-only-diag] resolved mode: C2=1 (low authority, left_max_offset=0.05m); C1=C3=C4=0'
echo '[bridge-c2-only-diag][warning] the master will now feel a (low-gain) spring pulling toward the slave position.'
echo "[bridge-c2-only-diag] arming command: $0 --session ${SESSION_ID} --output-dir ${OUTPUT_DIR} --arm-position"
echo "[bridge-c2-only-diag] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_POSITION}" != true ]]; then
  echo '[bridge-c2-only-diag][blocked] operator confirmation flag --arm-position is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"

if [[ "${DRY_RUN}" == false ]]; then
  source_ros
  echo '[bridge-c2-only-diag] running arm sequence (expected: C2=true, C1=C3=C4=false)...'
  if ! python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 false --expected-c2 true --expected-c3 false --expected-c4 false; then
    echo '[bridge-c2-only-diag][error] arm sequence FAILED -- teleoperation is NOT active.' >&2
    echo "[bridge-c2-only-diag][error] bridge process left running (unarmed) -- stop it with:" >&2
    echo "[bridge-c2-only-diag][error]   ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}" >&2
    exit 1
  fi
  echo '[bridge-c2-only-diag] arm sequence succeeded -- ISOLATED C2-only teleoperation is ACTIVE.'
fi
