#!/usr/bin/env bash
set -euo pipefail

# "Visual" (simulated) twin of the right slave -- see run_master_left_sim.sh
# for the harness rationale, which applies identically here.
#
# Subscribes to the same /touch_right/target_pose, /touch_right/force_cmd,
# /touch_right/enable topics the real slave receives, so it is driven by
# identical commands. Outputs are remapped under /touch_right_sim/....
#
# TOUCH_SIM_ENV / TOUCH_SIM_HAND let this run standalone, with no physical
# device at all, by synthesizing a contact episode (e.g. TOUCH_SIM_ENV=wall)
# so the harness (recording + plotting) can be exercised and validated while
# away from the hardware, per the plan's Phase 1 verification step.
#
# FRAME ALIGNMENT (2026-08-26 fix): before starting the sim node, this script
# seeds the sim's initial joint state (via TOUCH_SIM_INIT_POS_*, consumed by
# ArmModel::reset()'s solveIkToPosition()) at the same Cartesian position the
# bridge anchored the REAL slave to. Without this, the sim starts at its own
# fixed default pose while the bridge anchors /touch_right/target_pose to the
# REAL device's actual start -- a large, spurious tracking error from frame
# zero that was confirmed to slam all three simulated joints into their
# travel limits before any real disturbance occurs.
#
# CALIBRATION (2026-08-27): the preferred source is the bridge's calibration
# file (right_anchor, written by bridge_master_slave.py's _write_calibration
# once the bridge sees the real slave's first pose) rather than re-sampling
# /touch_right/pose here a second time -- one less place for a racy/stale
# read to disagree with what the bridge is actually using as its anchor.
# Given this project's launch order (master, then bridge, then slave), the
# calibration file's right_anchor should already exist by the time this runs.
# Falls back to reading /touch_right/pose directly (the original mechanism)
# if the calibration file is missing/stale, then to the sim's default reset
# pose if neither is available -- run_slave_right.sh (real) and run_bridge.sh
# first; if they are not yet up, this warns rather than blocking.

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
PUBLISH_TELEMETRY="${PUBLISH_TELEMETRY:-true}"
DOB_ENABLE="${DOB_ENABLE:-true}"

export TOUCH_SIM_RATE_HZ="${TOUCH_SIM_RATE_HZ:-1000}"
export TOUCH_SIM_INERTIA_MISMATCH="${TOUCH_SIM_INERTIA_MISMATCH:-1.08}"
export TOUCH_SIM_GRAVITY_MISMATCH="${TOUCH_SIM_GRAVITY_MISMATCH:-1.06}"
export TOUCH_SIM_BASE_TILT_RAD="${TOUCH_SIM_BASE_TILT_RAD:-0.0785}"
export TOUCH_SIM_ENCODER_QUANTUM="${TOUCH_SIM_ENCODER_QUANTUM:-4.07e-4}"
# "wall" + a hand model gives repeatable synthetic contact episodes without
# hardware -- see sim/src/hd_sim_api.cpp / sim/src/environment.hpp for the
# full parameter set (TOUCH_SIM_WALL_K, _WALL_B, _WALL_Y, _HAND_*).
export TOUCH_SIM_ENV="${TOUCH_SIM_ENV:-free}"
export TOUCH_SIM_HAND="${TOUCH_SIM_HAND:-none}"
export TOUCH_SIM_HAND_MOTION="${TOUCH_SIM_HAND_MOTION:-none}"
export TOUCH_SIM_TRUTH_CSV="${TOUCH_SIM_TRUTH_CSV:-}"
# false (default): deliberately-mismatched params, for robustness testing.
# true: loads M_n/G(q) from ~/dob_logs/results/{inertia,gravity}_left.json
# (or $SIM_IDENTIFIED_PARAMS_DIR), for using real-vs-sim divergence as a
# friction/unmodeled-dynamics estimate -- see hd_sim_api.cpp's configureDevice()
# for the full mode comparison.
export SIM_USE_IDENTIFIED_PARAMS="${SIM_USE_IDENTIFIED_PARAMS:-false}"
export SIM_IDENTIFIED_PARAMS_DIR="${SIM_IDENTIFIED_PARAMS_DIR:-}"

if [[ ! -f "${TOUCH_WS}/install/setup.bash" ]]; then
  echo "[teleop_slave_sim][error] missing ${TOUCH_WS}/install/setup.bash" >&2
  echo "[teleop_slave_sim][hint] build it once:" >&2
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

REAL_POSE_TOPIC="${REAL_POSE_TOPIC:-/touch_right/pose}"
CALIBRATION_FILE="${CALIBRATION_FILE:-~/dob_logs/teleop_calibration.json}"
CALIBRATION_TIMEOUT_S="${CALIBRATION_TIMEOUT_S:-3}"
INIT_POSE_TIMEOUT_S="${INIT_POSE_TIMEOUT_S:-5}"
if seed="$(python3 "${SCRIPT_DIR}/read_calibration.py" --file "${CALIBRATION_FILE}" \
    --field right_anchor --timeout "${CALIBRATION_TIMEOUT_S}" 2>/tmp/read_calibration_right.err)"; then
  read -r TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z <<< "${seed}"
  export TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z
  echo "[teleop_slave_sim] seeding sim initial pose from ${CALIBRATION_FILE} right_anchor =" \
    "(${TOUCH_SIM_INIT_POS_X}, ${TOUCH_SIM_INIT_POS_Y}, ${TOUCH_SIM_INIT_POS_Z}) m"
elif seed="$(python3 "${SCRIPT_DIR}/read_current_pose.py" "${REAL_POSE_TOPIC}" --timeout "${INIT_POSE_TIMEOUT_S}" 2>/tmp/read_current_pose_right.err)"; then
  read -r TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z <<< "${seed}"
  export TOUCH_SIM_INIT_POS_X TOUCH_SIM_INIT_POS_Y TOUCH_SIM_INIT_POS_Z
  echo "[teleop_slave_sim][warning] no calibration file (${CALIBRATION_FILE}); falling back to" \
    "reading ${REAL_POSE_TOPIC} directly = (${TOUCH_SIM_INIT_POS_X}, ${TOUCH_SIM_INIT_POS_Y}, ${TOUCH_SIM_INIT_POS_Z}) m" >&2
  cat /tmp/read_calibration_right.err >&2 || true
else
  echo "[teleop_slave_sim][warning] could not read calibration file or ${REAL_POSE_TOPIC}" \
    "(is run_slave_right.sh / run_bridge.sh running?); falling back to the sim's default reset pose" >&2
  cat /tmp/read_calibration_right.err >&2 || true
  cat /tmp/read_current_pose_right.err >&2 || true
fi

echo "[teleop_slave_sim] right visual twin dob_enable=${DOB_ENABLE} env=${TOUCH_SIM_ENV} hand=${TOUCH_SIM_HAND}" \
  "stiffness=${STIFFNESS_N_PER_M} N/m damping=${DAMPING_N_S_PER_M} Ns/m"

exec ros2 run touch_haptics touch_slave_sim --ros-args \
  -r __node:=teleop_slave_right_sim \
  -p device:="Default Device" \
  -p frame_id:=touch_right_sim_base \
  -p publish_hz:="${PUBLISH_HZ}" \
  -p pos_scale:=0.001 \
  -p max_force_n:="${MAX_FORCE_N}" \
  -p stiffness_n_per_m:="${STIFFNESS_N_PER_M}" \
  -p damping_n_s_per_m:="${DAMPING_N_S_PER_M}" \
  -p max_position_error_m:="${MAX_POSITION_ERROR_M}" \
  -p stale_timeout_s:="${STALE_TIMEOUT_S}" \
  -p require_deadman:=false \
  -p enabled_on_start:=false \
  -p publish_telemetry:="${PUBLISH_TELEMETRY}" \
  -p dob_enable:="${DOB_ENABLE}" \
  -r target_pose:=/touch_right/target_pose \
  -r force_cmd:=/touch_right/force_cmd \
  -r enable:=/touch_right/enable \
  -r pose:=/touch_right_sim/pose \
  -r rpy:=/touch_right_sim/rpy \
  -r buttons:=/touch_right_sim/buttons \
  -r clutch:=/touch_right_sim/clutch \
  -r applied_force:=/touch_right_sim/applied_force \
  -r last_force:=/touch_right_sim/last_force \
  -r joint_torque:=/touch_right_sim/joint_torque \
  -r gimbal_torque:=/touch_right_sim/gimbal_torque \
  -r joint_angles:=/touch_right_sim/joint_angles \
  -r motor_temperature:=/touch_right_sim/motor_temperature \
  -r tracking_error:=/touch_right_sim/tracking_error \
  -r dob_diagnostics:=/touch_right_sim/dob_diagnostics \
  -r force_ext_hat:=/touch_right_sim/force_ext_hat \
  -r joint_target:=/touch_right_sim/joint_target \
  -r joint_torque_ext_cmd:=/touch_right_sim/joint_torque_ext_cmd
