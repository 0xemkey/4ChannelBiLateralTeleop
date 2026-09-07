#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 of the direct teleoperation project.
# Starts the left Touch as the master input device.
#
# This uses touch_slave rather than touch_publisher so the same node can render
# force on the left device later if you add master-side feedback. With no
# target_pose and no force_cmd published to it, it behaves as a pose publisher.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"

TOUCH_WS="${TOUCH_WS:-${REPO_ROOT}/ros2_ws}"
DEVICE_NAME="${DEVICE_NAME:-Left}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PUBLISH_HZ="${PUBLISH_HZ:-200}"
# Ceiling on force the master may render. The bridge only publishes force when
# started with FORCE_REFLECT=true, so with reflection off the master stays
# passive regardless of this value.
#
# Kept above the bridge's REFLECT_MAX_N (1.5 N) so the bridge's clamp is the
# one that decides, not this one. The Touch is rated ~0.88 N continuous and
# ~3.3 N peak, so this is deliberately into the short-session range.
MAX_FORCE_N="${MAX_FORCE_N:-2.0}"
PUBLISH_TELEMETRY="${PUBLISH_TELEMETRY:-true}"
# C2 (2026-09-01): slave position -> master target, off by default. When the
# bridge is started with ENABLE_C2=true it publishes to /touch_left/target_pose,
# and these params give the master's own spring/damper something to render
# against it. Deliberately softer than the slave's (35.0 N/m / 0.20 Ns/m,
# run_slave_right.sh) -- this is a feedback channel felt by a human hand, not
# a position-tracking slave, and MAX_FORCE_N above is already the harder
# ceiling. Only takes effect when ENABLE_C2=true; with it false (the
# default), this script's node params are byte-identical to before C2 existed.
ENABLE_C2="${ENABLE_C2:-false}"
LEFT_STIFFNESS_N_PER_M="${LEFT_STIFFNESS_N_PER_M:-15.0}"
LEFT_DAMPING_N_S_PER_M="${LEFT_DAMPING_N_S_PER_M:-0.10}"
LEFT_MAX_POSITION_ERROR_M="${LEFT_MAX_POSITION_ERROR_M:-0.03}"
LEFT_STALE_TIMEOUT_S="${LEFT_STALE_TIMEOUT_S:-0.05}"
# C1 (2026-09-03): off by default. The master's own dob_inertia_json/
# dob_gravity_json already default (in touch_slave.cpp) to this arm's own
# left-arm fitted files, and dob_enable already defaults to true, so no new
# json path params are needed here -- only the joint-space-control switch
# and its torque ceiling.
JOINT_SPACE_CONTROL="${JOINT_SPACE_CONTROL:-false}"
DOB_TORQUE_F_MAX_N="${DOB_TORQUE_F_MAX_N:-0.6}"

if [[ ! -f "${TOUCH_WS}/install/setup.bash" ]]; then
  echo "[teleop_master][error] missing ${TOUCH_WS}/install/setup.bash" >&2
  echo "[teleop_master][hint] build it once:" >&2
  echo "  cd ${TOUCH_WS} && source /opt/ros/humble/setup.bash && \\" >&2
  echo "    colcon build --base-paths src --cmake-args -DOPENHAPTICS_ROOT=/usr" >&2
  exit 1
fi

if ps -eo args= | grep -E "touch_(publisher|slave).*device:=${DEVICE_NAME}" | grep -v grep >/dev/null; then
  echo "[teleop_master][error] ${DEVICE_NAME} is already owned by another Touch process." >&2
  echo "[teleop_master][hint] Only one OpenHaptics process may own a device at a time." >&2
  exit 1
fi

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
source "${TOUCH_WS}/install/setup.bash"
set -u
export GTDD_HOME="${TOUCH_WS}"

if awk -v v="${MAX_FORCE_N}" 'BEGIN { exit !(v > 0.88) }'; then
  echo "[teleop_master][warning] MAX_FORCE_N=${MAX_FORCE_N} N is above the ~0.88 N continuous rating." >&2
  echo "[teleop_master][hint] Intentional for a firmer wall; let the motors rest between long sessions." >&2
fi

c2_params=()
if [[ "${ENABLE_C2}" == "1" || "${ENABLE_C2}" == "true" ]]; then
  c2_params+=(
    -p "stiffness_n_per_m:=${LEFT_STIFFNESS_N_PER_M}"
    -p "damping_n_s_per_m:=${LEFT_DAMPING_N_S_PER_M}"
    -p "max_position_error_m:=${LEFT_MAX_POSITION_ERROR_M}"
    -p "stale_timeout_s:=${LEFT_STALE_TIMEOUT_S}"
  )
  echo "[teleop_master] C2 ON: stiffness=${LEFT_STIFFNESS_N_PER_M} N/m damping=${LEFT_DAMPING_N_S_PER_M} Ns/m" \
    "(bridge must also be started with ENABLE_C2=true)"
fi

echo "[teleop_master] left master device=${DEVICE_NAME} max_force=${MAX_FORCE_N} N telemetry=${PUBLISH_TELEMETRY}"
if [[ "${JOINT_SPACE_CONTROL}" == "1" || "${JOINT_SPACE_CONTROL}" == "true" ]]; then
  echo "[teleop_master] JOINT_SPACE_CONTROL ON: dob_torque_f_max_n=${DOB_TORQUE_F_MAX_N} N -- Cartesian" \
    "target_pose/force_cmd from the bridge will be ignored on this node while this is active" \
    "(see touch_slave.cpp's joint_path_commanded)."
fi

exec ros2 run touch_haptics touch_slave --ros-args \
  -r __node:=teleop_master_left \
  -p device:="${DEVICE_NAME}" \
  -p frame_id:=touch_left_base \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p pos_scale:=0.001 \
  -p max_force_n:="${MAX_FORCE_N}" \
  -p require_deadman:=false \
  -p enabled_on_start:=false \
  -p publish_telemetry:="${PUBLISH_TELEMETRY}" \
  -p joint_space_control:="${JOINT_SPACE_CONTROL}" \
  -p dob_torque_f_max_n:="${DOB_TORQUE_F_MAX_N}" \
  "${c2_params[@]}" \
  -r pose:=/touch_left/pose \
  -r rpy:=/touch_left/rpy \
  -r buttons:=/touch_left/buttons \
  -r clutch:=/touch_left/clutch \
  -r target_pose:=/touch_left/target_pose \
  -r force_cmd:=/touch_left/force_cmd \
  -r enable:=/touch_left/enable \
  -r applied_force:=/touch_left/applied_force \
  -r last_force:=/touch_left/last_force \
  -r joint_torque:=/touch_left/joint_torque \
  -r gimbal_torque:=/touch_left/gimbal_torque \
  -r joint_angles:=/touch_left/joint_angles \
  -r joint_target:=/touch_left/joint_target \
  -r joint_torque_ext_cmd:=/touch_left/joint_torque_ext_cmd \
  -r force_ext_hat:=/touch_left/force_ext_hat \
  -r motor_temperature:=/touch_left/motor_temperature \
  -r tracking_error:=/touch_left/tracking_error \
  -r dob_diagnostics:=/touch_left/dob_diagnostics \
  -r cartesian_controller_telemetry:=/touch_left/cartesian_controller_telemetry
