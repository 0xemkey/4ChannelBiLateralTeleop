#!/usr/bin/env bash
set -euo pipefail

# Terminal 3 of the direct teleoperation project.
# Starts the right Touch as the force-controlled slave, with telemetry on.
# Start run_master_left.sh and run_bridge.sh first (see run_bridge.sh for why
# order matters as of the 2026-08-27 calibration step).
#
# The right Touch config expects the default serial path, so this rebinds
# /dev/ttyACM1 onto /dev/ttyACM0 inside a private mount namespace. That step
# needs sudo, which is why this cannot be folded into the one-shot launcher.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"

TOUCH_WS="${TOUCH_WS:-${REPO_ROOT}/ros2_ws}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PUBLISH_HZ="${PUBLISH_HZ:-200}"
MAX_FORCE_N="${MAX_FORCE_N:-1.5}"
STIFFNESS_N_PER_M="${STIFFNESS_N_PER_M:-35.0}"
DAMPING_N_S_PER_M="${DAMPING_N_S_PER_M:-0.20}"
MAX_POSITION_ERROR_M="${MAX_POSITION_ERROR_M:-0.04}"
STALE_TIMEOUT_S="${STALE_TIMEOUT_S:-0.05}"
REQUIRE_DEADMAN="${REQUIRE_DEADMAN:-false}"
ENABLED_ON_START="${ENABLED_ON_START:-false}"
PUBLISH_TELEMETRY="${PUBLISH_TELEMETRY:-true}"
# C1 (2026-09-03): off by default, unchanged behavior unless explicitly
# turned on for a joint-space/C1 test. dob_enable stays false by default
# (was already false before this) since the right arm's DOB is only needed
# for C1; DOB_INERTIA_JSON/DOB_GRAVITY_JSON default to this arm's own
# fitted files (never the left-arm defaults baked into touch_slave.cpp),
# since this is the right/slave device.
JOINT_SPACE_CONTROL="${JOINT_SPACE_CONTROL:-false}"
DOB_ENABLE="${DOB_ENABLE:-false}"
DOB_TORQUE_F_MAX_N="${DOB_TORQUE_F_MAX_N:-0.6}"
DOB_INERTIA_JSON="${DOB_INERTIA_JSON:-${HOME}/dob_logs/results/inertia_right.json}"
DOB_GRAVITY_JSON="${DOB_GRAVITY_JSON:-${HOME}/dob_logs/results/gravity_right_minimal.json}"
RIGHT_SLAVE_INVERT_X="${RIGHT_SLAVE_INVERT_X:-false}"
RIGHT_SLAVE_INVERT_Y="${RIGHT_SLAVE_INVERT_Y:-false}"
RIGHT_SLAVE_INVERT_Z="${RIGHT_SLAVE_INVERT_Z:-false}"
RIGHT_SLAVE_SWAP_XY="${RIGHT_SLAVE_SWAP_XY:-false}"
RIGHT_TTY="${RIGHT_TTY:-/dev/ttyACM1}"
MAPPED_TTY="${MAPPED_TTY:-/dev/ttyACM0}"
TARGET_USER="${SUDO_USER:-${USER}}"

_as_double() {
  awk -v value="$1" 'BEGIN { printf "%.6f", value + 0.0 }'
}

MAX_FORCE_N="$(_as_double "${MAX_FORCE_N}")"
STIFFNESS_N_PER_M="$(_as_double "${STIFFNESS_N_PER_M}")"
DAMPING_N_S_PER_M="$(_as_double "${DAMPING_N_S_PER_M}")"
MAX_POSITION_ERROR_M="$(_as_double "${MAX_POSITION_ERROR_M}")"
STALE_TIMEOUT_S="$(_as_double "${STALE_TIMEOUT_S}")"

if ps -eo pid=,comm=,args= \
  | grep -Ev "run_slave_right|grep " \
  | grep -Eq "touch_publisher.*(__node:=touch_publisher_right|device:=Default Device)|touch_slave.*__node:=(touch_slave_right|teleop_slave_right)"; then
  echo "[teleop_slave][error] right device appears to be owned by another Touch process." >&2
  exit 1
fi

if [[ ! -e "${RIGHT_TTY}" ]]; then
  echo "[teleop_slave][error] missing right serial device: ${RIGHT_TTY}" >&2
  exit 1
fi

if [[ "${RIGHT_TTY}" != "${MAPPED_TTY}" ]]; then
  sudo -v
  exec sudo unshare -m -- bash -lc "
set -euo pipefail
mount --make-rprivate /
mount --bind '${RIGHT_TTY}' '${MAPPED_TTY}'
exec runuser -u '${TARGET_USER}' -- env \
  TOUCH_WS='${TOUCH_WS}' \
  ROS_DOMAIN_ID='${ROS_DOMAIN_ID}' \
  PUBLISH_HZ='${PUBLISH_HZ}' \
  MAX_FORCE_N='${MAX_FORCE_N}' \
  STIFFNESS_N_PER_M='${STIFFNESS_N_PER_M}' \
  DAMPING_N_S_PER_M='${DAMPING_N_S_PER_M}' \
  MAX_POSITION_ERROR_M='${MAX_POSITION_ERROR_M}' \
  STALE_TIMEOUT_S='${STALE_TIMEOUT_S}' \
  REQUIRE_DEADMAN='${REQUIRE_DEADMAN}' \
  ENABLED_ON_START='${ENABLED_ON_START}' \
  PUBLISH_TELEMETRY='${PUBLISH_TELEMETRY}' \
  JOINT_SPACE_CONTROL='${JOINT_SPACE_CONTROL}' \
  DOB_ENABLE='${DOB_ENABLE}' \
  DOB_TORQUE_F_MAX_N='${DOB_TORQUE_F_MAX_N}' \
  DOB_INERTIA_JSON='${DOB_INERTIA_JSON}' \
  DOB_GRAVITY_JSON='${DOB_GRAVITY_JSON}' \
  RIGHT_SLAVE_INVERT_X='${RIGHT_SLAVE_INVERT_X}' \
  RIGHT_SLAVE_INVERT_Y='${RIGHT_SLAVE_INVERT_Y}' \
  RIGHT_SLAVE_INVERT_Z='${RIGHT_SLAVE_INVERT_Z}' \
  RIGHT_SLAVE_SWAP_XY='${RIGHT_SLAVE_SWAP_XY}' \
  RIGHT_TTY='${MAPPED_TTY}' \
  MAPPED_TTY='${MAPPED_TTY}' \
  bash '$0'
"
fi

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
source "${TOUCH_WS}/install/setup.bash"
set -u
export GTDD_HOME="${TOUCH_WS}"

echo "[teleop_slave] right slave stiffness=${STIFFNESS_N_PER_M} N/m damping=${DAMPING_N_S_PER_M} Ns/m max_force=${MAX_FORCE_N} N telemetry=${PUBLISH_TELEMETRY}"
if [[ "${JOINT_SPACE_CONTROL}" == "1" || "${JOINT_SPACE_CONTROL}" == "true" ]]; then
  echo "[teleop_slave] JOINT_SPACE_CONTROL ON: dob_enable=${DOB_ENABLE} dob_torque_f_max_n=${DOB_TORQUE_F_MAX_N} N" \
    "inertia=${DOB_INERTIA_JSON} gravity=${DOB_GRAVITY_JSON} -- Cartesian target_pose/force_cmd from the" \
    "bridge will be ignored on this node while this is active (see touch_slave.cpp's joint_path_commanded)."
fi

exec ros2 run touch_haptics touch_slave --ros-args \
  -r __node:=teleop_slave_right \
  -p device:="Default Device" \
  -p frame_id:=touch_right_base \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p pos_scale:=0.001 \
  -p max_force_n:="${MAX_FORCE_N}" \
  -p stiffness_n_per_m:="${STIFFNESS_N_PER_M}" \
  -p damping_n_s_per_m:="${DAMPING_N_S_PER_M}" \
  -p max_position_error_m:="${MAX_POSITION_ERROR_M}" \
  -p stale_timeout_s:="${STALE_TIMEOUT_S}" \
  -p require_deadman:="${REQUIRE_DEADMAN}" \
  -p enabled_on_start:="${ENABLED_ON_START}" \
  -p publish_telemetry:="${PUBLISH_TELEMETRY}" \
  -p invert_x:="${RIGHT_SLAVE_INVERT_X}" \
  -p invert_y:="${RIGHT_SLAVE_INVERT_Y}" \
  -p invert_z:="${RIGHT_SLAVE_INVERT_Z}" \
  -p swap_xy:="${RIGHT_SLAVE_SWAP_XY}" \
  -p joint_space_control:="${JOINT_SPACE_CONTROL}" \
  -p dob_torque_f_max_n:="${DOB_TORQUE_F_MAX_N}" \
  -p dob_inertia_json:="${DOB_INERTIA_JSON}" \
  -p dob_gravity_json:="${DOB_GRAVITY_JSON}" \
  -r pose:=/touch_right/pose \
  -r rpy:=/touch_right/rpy \
  -r buttons:=/touch_right/buttons \
  -r clutch:=/touch_right/clutch \
  -r target_pose:=/touch_right/target_pose \
  -r force_cmd:=/touch_right/force_cmd \
  -r enable:=/touch_right/enable \
  -r applied_force:=/touch_right/applied_force \
  -r last_force:=/touch_right/last_force \
  -r joint_torque:=/touch_right/joint_torque \
  -r gimbal_torque:=/touch_right/gimbal_torque \
  -r joint_angles:=/touch_right/joint_angles \
  -r joint_target:=/touch_right/joint_target \
  -r joint_torque_ext_cmd:=/touch_right/joint_torque_ext_cmd \
  -r force_ext_hat:=/touch_right/force_ext_hat \
  -r motor_temperature:=/touch_right/motor_temperature \
  -r tracking_error:=/touch_right/tracking_error \
  -r dob_diagnostics:=/touch_right/dob_diagnostics \
  -p dob_enable:="${DOB_ENABLE}"
