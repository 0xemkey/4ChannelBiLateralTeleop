#!/usr/bin/env bash
set -euo pipefail

# "Visual" (simulated) twin of the left master, for the real-vs-sim noise
# harness described in docs/FOUR_CHANNEL_TODO.md's follow-on plan.
#
# Runs touch_slave_sim -- the SAME touch_slave.cpp source as the real master,
# linked against hd_sim instead of real HDAPI (see
# ros2_ws/src/touch_haptics/sim/). It subscribes to the exact same command
# topics as the real master (/touch_left/target_pose, /touch_left/force_cmd,
# /touch_left/enable), so whatever drives the real device also drives this
# twin. Its own outputs are remapped to a parallel /touch_left_sim/... tree
# so they never collide with the real node's publications on the same bus.
#
# No hardware required: this can run standalone (e.g. to smoke-test the
# harness) or alongside run_master_left.sh with a real device attached.
#
# hd_sim's arm model is DELIBERATELY mismatched from the identified real-arm
# parameters (inertia/gravity/base-tilt -- see sim/src/arm_model.hpp). That
# mismatch is the whole point: a real-vs-sim residual of zero would mean the
# comparison is vacuous. Do not "fix" TOUCH_SIM_* below to make the twin
# track more closely.
#
# FRAME ALIGNMENT (2026-08-26 fix): seeds the sim's initial joint state at
# the real master's current Cartesian position instead of the sim's fixed
# default reset pose -- see run_slave_right_sim.sh for the full rationale,
# which applies here too. NOTE: this does not by itself make the master's
# force comparison meaningful -- the master's control law never uses a
# position-error term at all (nothing publishes /touch_left/target_pose), so
# its force will always just echo the shared force_cmd regardless of
# position. That is a separate, larger, un-addressed issue -- this fix only
# stops the master's sim twin from starting at an arbitrary, unrelated pose.
#
# CALIBRATION (2026-08-27): prefers the bridge's calibration file
# (left_anchor) over reading /touch_left/pose directly, same reasoning as
# run_slave_right_sim.sh -- one shared, bridge-captured snapshot instead of
# each script re-sampling the live topic separately. Falls back to the direct
# topic read if the calibration file is not there yet (e.g. this is started
# before the bridge, which this project's launch order does not call for but
# which this script should not hard-fail on).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"

TOUCH_WS="${TOUCH_WS:-${REPO_ROOT}/ros2_ws}"
DEVICE_NAME="${DEVICE_NAME:-Left}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
PUBLISH_HZ="${PUBLISH_HZ:-200}"
MAX_FORCE_N="${MAX_FORCE_N:-2.0}"
PUBLISH_TELEMETRY="${PUBLISH_TELEMETRY:-true}"
DOB_ENABLE="${DOB_ENABLE:-true}"

# hd_sim tuning -- see sim/src/hd_sim_api.cpp configureDevice(). Left at the
# file's own defaults unless overridden; only exposed here so a session can
# be reproduced from its recorded .env file (see run_bag_real_vs_sim.sh).
export TOUCH_SIM_RATE_HZ="${TOUCH_SIM_RATE_HZ:-1000}"
export TOUCH_SIM_INERTIA_MISMATCH="${TOUCH_SIM_INERTIA_MISMATCH:-1.08}"
export TOUCH_SIM_GRAVITY_MISMATCH="${TOUCH_SIM_GRAVITY_MISMATCH:-1.06}"
export TOUCH_SIM_BASE_TILT_RAD="${TOUCH_SIM_BASE_TILT_RAD:-0.0785}"
export TOUCH_SIM_ENCODER_QUANTUM="${TOUCH_SIM_ENCODER_QUANTUM:-4.07e-4}"
export TOUCH_SIM_ENV="${TOUCH_SIM_ENV:-free}"
export TOUCH_SIM_HAND="${TOUCH_SIM_HAND:-none}"
export TOUCH_SIM_HAND_MOTION="${TOUCH_SIM_HAND_MOTION:-none}"
# Ground-truth CSV (q_true, gravityTrueNm, frictionTrueNm, ...), for Phase 2's
# friction/gravity/environmental separation. Empty = disabled.
export TOUCH_SIM_TRUTH_CSV="${TOUCH_SIM_TRUTH_CSV:-}"
# See run_slave_right_sim.sh for the full mode comparison.
export SIM_USE_IDENTIFIED_PARAMS="${SIM_USE_IDENTIFIED_PARAMS:-false}"
export SIM_IDENTIFIED_PARAMS_DIR="${SIM_IDENTIFIED_PARAMS_DIR:-}"

if [[ ! -f "${TOUCH_WS}/install/setup.bash" ]]; then
  echo "[teleop_master_sim][error] missing ${TOUCH_WS}/install/setup.bash" >&2
  echo "[teleop_master_sim][hint] build it once:" >&2
  echo "  cd ${TOUCH_WS} && source /opt/ros/humble/setup.bash && \\" >&2
  echo "    colcon build --base-paths src --cmake-args -DOPENHAPTICS_ROOT=/usr" >&2
  exit 1
fi

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
source "${TOUCH_WS}/install/setup.bash"
set -u
export GTDD_HOME="${TOUCH_WS}"

REAL_POSE_TOPIC="${REAL_POSE_TOPIC:-/touch_left/pose}"
CALIBRATION_FILE="${CALIBRATION_FILE:-~/dob_logs/teleop_calibration.json}"
CALIBRATION_TIMEOUT_S="${CALIBRATION_TIMEOUT_S:-3}"
INIT_POSE_TIMEOUT_S="${INIT_POSE_TIMEOUT_S:-5}"
if seed="$(python3 "${SCRIPT_DIR}/read_calibration.py" --file "${CALIBRATION_FILE}" \
    --field left_anchor --timeout "${CALIBRATION_TIMEOUT_S}" 2>/tmp/read_calibration_left.err)"; then
  read -r TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z <<< "${seed}"
  export TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z
  echo "[teleop_master_sim] seeding sim initial pose from ${CALIBRATION_FILE} left_anchor =" \
    "(${TOUCH_SIM_INIT_POS_X}, ${TOUCH_SIM_INIT_POS_Y}, ${TOUCH_SIM_INIT_POS_Z}) m"
elif seed="$(python3 "${SCRIPT_DIR}/read_current_pose.py" "${REAL_POSE_TOPIC}" --timeout "${INIT_POSE_TIMEOUT_S}" 2>/tmp/read_current_pose_left.err)"; then
  read -r TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z <<< "${seed}"
  export TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z
  echo "[teleop_master_sim][warning] no calibration file (${CALIBRATION_FILE}); falling back to" \
    "reading ${REAL_POSE_TOPIC} directly = (${TOUCH_SIM_INIT_POS_X}, ${TOUCH_SIM_INIT_POS_Y}, ${TOUCH_SIM_INIT_POS_Z}) m" >&2
  cat /tmp/read_calibration_left.err >&2 || true
else
  echo "[teleop_master_sim][warning] could not read calibration file or ${REAL_POSE_TOPIC}" \
    "(is run_master_left.sh / run_bridge.sh running?); falling back to the sim's default reset pose" >&2
  cat /tmp/read_calibration_left.err >&2 || true
  cat /tmp/read_current_pose_left.err >&2 || true
fi

echo "[teleop_master_sim] left visual twin device=${DEVICE_NAME} dob_enable=${DOB_ENABLE}" \
  "inertia_mismatch=${TOUCH_SIM_INERTIA_MISMATCH} gravity_mismatch=${TOUCH_SIM_GRAVITY_MISMATCH}"

exec ros2 run touch_haptics touch_slave_sim --ros-args \
  -r __node:=teleop_master_left_sim \
  -p device:="${DEVICE_NAME}" \
  -p frame_id:=touch_left_sim_base \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p pos_scale:=0.001 \
  -p max_force_n:="${MAX_FORCE_N}" \
  -p require_deadman:=false \
  -p enabled_on_start:=false \
  -p publish_telemetry:="${PUBLISH_TELEMETRY}" \
  -p dob_enable:="${DOB_ENABLE}" \
  -r target_pose:=/touch_left/target_pose \
  -r force_cmd:=/touch_left/force_cmd \
  -r enable:=/touch_left/enable \
  -r pose:=/touch_left_sim/pose \
  -r rpy:=/touch_left_sim/rpy \
  -r buttons:=/touch_left_sim/buttons \
  -r clutch:=/touch_left_sim/clutch \
  -r applied_force:=/touch_left_sim/applied_force \
  -r last_force:=/touch_left_sim/last_force \
  -r joint_torque:=/touch_left_sim/joint_torque \
  -r gimbal_torque:=/touch_left_sim/gimbal_torque \
  -r joint_angles:=/touch_left_sim/joint_angles \
  -r motor_temperature:=/touch_left_sim/motor_temperature \
  -r tracking_error:=/touch_left_sim/tracking_error \
  -r dob_diagnostics:=/touch_left_sim/dob_diagnostics \
  -r force_ext_hat:=/touch_left_sim/force_ext_hat \
  -r joint_target:=/touch_left_sim/joint_target \
  -r joint_torque_ext_cmd:=/touch_left_sim/joint_torque_ext_cmd
