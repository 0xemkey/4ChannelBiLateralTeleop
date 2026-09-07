#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=()
source "$(cd "$(dirname "$0")" && pwd)/common.sh"
parse_common_args "$@"
[[ ${#EXTRA_ARGS[@]} -eq 0 ]] || { echo "[record][error] unknown argument: ${EXTRA_ARGS[*]}" >&2; exit 2; }
require_session
source_ros
mkdir -p "${SESSION_DIR}/raw" "${SESSION_DIR}/events"
cmd=(ros2 bag record --storage sqlite3 --max-cache-size 1073741824
  -o "${SESSION_DIR}/raw/teleop"
  /touch_left/pose /touch_right/pose
  /touch_left/rpy /touch_right/rpy
  /touch_left/buttons /touch_right/buttons
  /touch_left/joint_angles /touch_right/joint_angles
  /touch_left/joint_torque /touch_right/joint_torque
  /touch_left/last_force /touch_right/last_force
  /touch_left/target_pose /touch_right/target_pose
  /touch_left/tracking_error /touch_right/tracking_error
  /touch_left/applied_force /touch_right/applied_force
  /touch_left/dob_diagnostics /touch_right/dob_diagnostics
  /touch_left/force_ext_hat /touch_right/force_ext_hat
  /touch_left/cartesian_controller_telemetry
  /touch_right/cartesian_controller_telemetry
  /touch_left/enable /touch_right/enable
  /touch_left/force_cmd /touch_right/force_cmd
  /touch_left/joint_target /touch_right/joint_target
  /touch_left/joint_torque_ext_cmd /touch_right/joint_torque_ext_cmd
  /operator_event)
echo "[record] raw bag=${SESSION_DIR}/raw/teleop"
start_component recorder "${cmd[@]}"

