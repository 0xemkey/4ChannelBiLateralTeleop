#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_RELAY=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do
  [[ "${arg}" == --arm-relay ]] && ARM_RELAY=true || {
    echo "[full4ch][error] unknown argument: ${arg}" >&2
    exit 2
  }
done

QUALIFICATION_DIR="${QUALIFICATION_DIR:-${PROJECT_ROOT}/hardware_validation/qualification}"
C1_ARTIFACT="${QUALIFICATION_DIR}/C1_QUALIFIED.json"
C4_ARTIFACT="${QUALIFICATION_DIR}/C4_QUALIFIED.json"

missing=()
[[ -f "${C1_ARTIFACT}" ]] || missing+=("${C1_ARTIFACT}")
[[ -f "${C4_ARTIFACT}" ]] || missing+=("${C4_ARTIFACT}")
if ((${#missing[@]} > 0)); then
  echo '[full4ch][blocked] C1/C4 qualification artifact(s) missing -- full 4-channel teleoperation' >&2
  echo '[full4ch][blocked] is NOT authorized to start. Missing:' >&2
  for f in "${missing[@]}"; do echo "[full4ch][blocked]   ${f}" >&2; done
  echo '[full4ch][hint] run 08_qualify_c1_only.sh and 09_qualify_c4_only.sh on real hardware first.' >&2
  exit 3
fi

reject_live_component 'bridge_master_slave.py' 'position bridge'

relay_args=(--session "${SESSION_ID}" --output-dir "${OUTPUT_DIR}" --full4ch)
[[ "${DRY_RUN}" == true ]] && relay_args+=(--dry-run)
[[ "${ARM_RELAY}" == true ]] && relay_args+=(--arm-relay)
bash "${LAUNCH_DIR}/04_start_force_relay.sh" "${relay_args[@]}"

MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M:-15.0}"
MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M:-0.10}"
MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N:-2.0}"
MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M:-0.03}"
LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N:-0.55}"
LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M:-0.08}"

cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true C3_ENABLE=true
  C1_ENABLE=true C4_ENABLE=true FORCE_REFLECT=false
  SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15
  LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M}" LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N}" LEFT_GRAVITY_AXIS=1
  MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M}" MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M}"
  MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N}" MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}"
  MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json"
  "${PROJECT_ROOT}/scripts/run_bridge.sh")

echo '[full4ch] starting bridge: C1=C2=C3=C4 true; bridge owns C2/C3 and /enable'
echo '[full4ch] bilateral_4ch.py supplies C1/C4 only; its C2/C3 joint-space weights are zero'
echo "[full4ch] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_RELAY}" != true ]]; then
  echo '[full4ch][blocked] operator confirmation flag --arm-relay is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"

if [[ "${DRY_RUN}" == false ]]; then
  source_ros
  echo '[full4ch] running arm sequence (expected: C1=C2=C3=C4=true)...'
  if ! python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 true --expected-c2 true --expected-c3 true --expected-c4 true; then
    echo '[full4ch][error] arm sequence FAILED -- stopping full-4ch session.' >&2
    bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}" --output-dir "${OUTPUT_DIR}" || true
    exit 1
  fi
  echo '[full4ch] full 4-channel teleoperation is ACTIVE.'
fi
