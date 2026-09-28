#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_RELAY=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do
  [[ "${arg}" == --arm-relay ]] && ARM_RELAY=true || {
    echo "[qualify-c1][error] unknown argument: ${arg}" >&2
    exit 2
  }
done

# Isolated C1 qualification:
#   source: slave DOB external joint torque estimate
#   relay:  bilateral_4ch.py, C1 only
#   sink:   master Cartesian relay path in touch_slave.cpp
#
# A bridge is still required, but only as the single /enable authority and
# safety-state compositor. C2 and C3 are explicitly disabled and gravity
# compensation is zeroed so the receiving force is attributable to C1.

reject_live_component 'bridge_master_slave.py' 'position bridge'

relay_args=(--session "${SESSION_ID}" --output-dir "${OUTPUT_DIR}" --c1-only)
[[ "${DRY_RUN}" == true ]] && relay_args+=(--dry-run)
[[ "${ARM_RELAY}" == true ]] && relay_args+=(--arm-relay)
bash "${LAUNCH_DIR}/04_start_force_relay.sh" "${relay_args[@]}"

cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=false C3_ENABLE=false
  C1_ENABLE=true C4_ENABLE=false FORCE_REFLECT=false
  SLAVE_GRAVITY_COMP_N=0.0 LEFT_GRAVITY_COMP_N=0.0
  MASTER_STIFFNESS_N_PER_M=15.0 MASTER_DAMPING_N_S_PER_M=0.10
  MASTER_MAX_FORCE_N=2.0 MASTER_MAX_POSITION_ERROR_M=0.03
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json"
  "${PROJECT_ROOT}/scripts/run_bridge.sh")

echo '[qualify-c1] starting enable/safety bridge: C1=true, C2=C3=C4=false, no gravity aux'
if [[ "${DRY_RUN}" == false && "${ARM_RELAY}" != true ]]; then
  echo '[qualify-c1][blocked] operator confirmation flag --arm-relay is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"

if [[ "${DRY_RUN}" == false ]]; then
  source_ros
  echo '[qualify-c1] running arm sequence (expected: C1=true, C2=C3=C4=false)...'
  if ! python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 true --expected-c2 false --expected-c3 false --expected-c4 false; then
    echo '[qualify-c1][error] arm sequence FAILED -- stopping qualification session.' >&2
    bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}" --output-dir "${OUTPUT_DIR}" || true
    exit 1
  fi
  echo '[qualify-c1] C1-only relay is ACTIVE.'
fi
