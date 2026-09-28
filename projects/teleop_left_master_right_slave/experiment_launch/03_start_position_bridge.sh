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

# 2026-09-10 fix (confirmed real-hardware bug): --arm-position previously
# only gated whether this script started the bridge PROCESS -- nothing in
# any experiment_launch/*.sh script ever actually set arm_teleop=true on
# it (confirmed: `grep -rn arm_teleop experiment_launch/*.sh` found zero
# matches before this fix). The bridge would sit in READY forever;
# /touch_right/enable read true (the bridge's own unconditional per-tick
# publish, independent of arm state) but coupling_ramp_alpha never left 0
# and moving the master produced no slave motion. This step is the
# missing arm command -- reuses the SAME proven async SetParameters
# mechanism H2's tooling uses (not a blocking `ros2 param set` CLI call),
# with pre-arm/post-arm verification and fail-closed behavior. Does NOT
# require a deadman -- production teleoperation never has (this project's
# run_bridge.sh has never passed --require-master-deadman).
if [[ "${DRY_RUN}" == false ]]; then
  source_ros
  echo '[bridge] running arm sequence (expected: C3=true, C1=C2=C4=false)...'
  # NOTE: --expected-c2 is hardcoded false here, matching this script's OWN
  # `cmd` array above (ENABLE_C2=false is a literal, not a bash variable in
  # this script's scope) -- if a future change makes ENABLE_C2 configurable
  # here, this must be updated to match, not left silently out of sync.
  if ! python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 false --expected-c2 false --expected-c3 true --expected-c4 false; then
    echo '[bridge][error] arm sequence FAILED -- teleoperation is NOT active.' >&2
    echo "[bridge][error] see the [arm] log lines above for the exact failure. bridge process" >&2
    echo "[bridge][error] left running (unarmed) -- stop it with:" >&2
    echo "[bridge][error]   ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}" >&2
    exit 1
  fi
  echo '[bridge] arm sequence succeeded -- production teleoperation is ACTIVE.'
fi

