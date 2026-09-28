#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
[[ ${#EXTRA_ARGS[@]} -eq 0 ]] || { echo "[master][error] unknown argument: ${EXTRA_ARGS[*]}" >&2; exit 2; }
reject_live_component 'touch_slave.*(__node:=teleop_master_left|device:=Left)' 'left master'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2="${ENABLE_C2:-false}"
  JOINT_SPACE_CONTROL=false MAX_FORCE_N="${MAX_FORCE_N:-2.0}"
  LEFT_STIFFNESS_N_PER_M="${LEFT_STIFFNESS_N_PER_M:-15.0}"
  LEFT_DAMPING_N_S_PER_M="${LEFT_DAMPING_N_S_PER_M:-0.10}"
  LEFT_MAX_POSITION_ERROR_M="${LEFT_MAX_POSITION_ERROR_M:-0.03}"
  "${PROJECT_ROOT}/scripts/run_master_left.sh")
echo "[master] resolved: Left -> /teleop_master_left; enabled_on_start=false; joint_space_control=false; DOB observation on; ENABLE_C2=${ENABLE_C2:-false}"
start_component master "${cmd[@]}"
