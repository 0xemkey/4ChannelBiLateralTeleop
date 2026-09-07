#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_POSITION=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do [[ "$arg" == --arm-position ]] && ARM_POSITION=true || { echo "[bridge][error] unknown argument: $arg" >&2; exit 2; }; done
reject_live_component 'bridge_master_slave.py' 'position bridge'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=false FORCE_REFLECT=false
  SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15
  MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json"
  "${PROJECT_ROOT}/scripts/run_bridge.sh")
echo '[bridge] resolved safe selected mode: C3=1; C1=C2=C4=0; FORCE_REFLECT=false'
echo '[bridge][warning] the bridge publishes enable=true and target_pose; starting it ARMS slave Cartesian force output.'
echo "[bridge] arming command: $0 --session ${SESSION_ID} --output-dir ${OUTPUT_DIR} --arm-position"
echo "[bridge] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_POSITION}" != true ]]; then
  echo '[bridge][blocked] operator confirmation flag --arm-position is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"

