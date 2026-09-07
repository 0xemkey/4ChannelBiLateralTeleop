#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); ARM_POSITION=false
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
for arg in "${EXTRA_ARGS[@]}"; do [[ "$arg" == --arm-position ]] && ARM_POSITION=true || { echo "[bridge-c2-diag][error] unknown argument: $arg" >&2; exit 2; }; done

# DIAGNOSTIC MODE, paired with 01b_start_master_c2_diagnostic.sh. Runs C2
# (slave position -> master, LOW authority) ALONGSIDE the normal C3 -- the
# task's "Mode B, full bilateral diagnostic" (both robots may influence
# each other), which is what actually answers "can I control the master
# from the slave" (Mode C alone, C3 off, is possible too but requires
# bridge_master_slave.py support for disabling C3's publish independently,
# which does not exist yet -- flagging rather than half-building it).
#
# left_max_offset_m tightened to 0.05m (vs production's 0.15m) -- "low
# authority" per the task, not the full reach.
reject_live_component 'bridge_master_slave.py' 'position bridge'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true FORCE_REFLECT=false \
  SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15 \
  LEFT_MAX_OFFSET_M=0.05 LEFT_GRAVITY_COMP_N=0.45 LEFT_GRAVITY_AXIS=1 \
  MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100 \
  CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json" \
  "${PROJECT_ROOT}/scripts/run_bridge.sh")
echo '[bridge-c2-diag][warning] DIAGNOSTIC MODE: C2=1 (low authority, left_max_offset=0.05m) + C3=1; C1=C4=0'
echo '[bridge-c2-diag][warning] the master will now feel a (low-gain) spring pulling toward the slave position.'
echo "[bridge-c2-diag] emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
if [[ "${DRY_RUN}" == false && "${ARM_POSITION}" != true ]]; then
  echo '[bridge-c2-diag][blocked] operator confirmation flag --arm-position is required' >&2
  exit 3
fi
start_component bridge "${cmd[@]}"
