#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
[[ ${#EXTRA_ARGS[@]} -eq 0 ]] || { echo "[slave][error] unknown argument: ${EXTRA_ARGS[*]}" >&2; exit 2; }
reject_live_component 'touch_slave.*__node:=teleop_slave_right' 'right slave'
cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" RIGHT_TTY=/dev/ttyACM1 MAPPED_TTY=/dev/ttyACM1
  ENABLED_ON_START=false JOINT_SPACE_CONTROL=false DOB_ENABLE=true MAX_FORCE_N=1.5
  DOB_INERTIA_JSON="${HOME}/dob_logs/results/inertia_right.json"
  DOB_GRAVITY_JSON="${HOME}/dob_logs/results/gravity_right_minimal.json"
  "${PROJECT_ROOT}/scripts/run_slave_right.sh")
echo '[slave] resolved: Default Device -> /teleop_slave_right; direct /dev/ttyACM1 (no sudo); enabled_on_start=false; joint_space_control=false; DOB observation on; C3 off'
start_component slave "${cmd[@]}"

