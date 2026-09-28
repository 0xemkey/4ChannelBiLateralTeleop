#!/usr/bin/env bash
set -euo pipefail

# Terminal 2 of the direct teleoperation project.
# Bridges left master pose deltas into right slave target poses.
# Start run_master_left.sh first, then this, then run_slave_right.sh.
#
# Order matters as of the 2026-08-27 calibration step: this node captures
# left_anchor as soon as the master is up and writes it to the calibration
# file immediately (see bridge_master_slave.py's _write_calibration), then
# captures right_anchor and updates the file once the slave starts. Starting
# the slave before this would still work (anchors are captured whenever each
# side's first pose arrives, in any order), but the calibration file would
# not exist yet for anything the slave's own startup wants to read from it.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SCALE="${SCALE:-1.0}"
AXIS_MAP="${AXIS_MAP:-x,y,z}"
MAX_OFFSET_M="${MAX_OFFSET_M:-0.12}"
# Per-slave-axis travel, applied after AXIS_MAP. The Omni workspace is wider
# than it is tall, so the vertical axis usually needs less scale and a tighter
# limit than the horizontal ones. Empty falls back to SCALE / MAX_OFFSET_M.
# 1:1 by default. The slave should reach the same height your hand does, so do
# not scale an axis down unless it physically cannot make the travel.
AXIS_SCALE="${AXIS_SCALE:-1.0,1.0,1.0}"
# Generous per-axis travel. These are safety stops against a runaway target,
# not a way to shape the motion -- keep them at or above the reach you actually
# want, or the slave will stop short of your hand.
#
# Y raised 0.11->0.19 (2026-08-26): confirmed @LIMIT, not force-budget, via
# the bridge's own "reach mm" line (asked 110.0 got 101.3, 0.92 reached --
# the tracking itself is healthy, the clamp is what's cutting it off) AND a
# manual hand-reach check with force disabled, which found ~185mm of genuine
# mechanical travel available from the same anchor -- 0.19 sits just above
# that with a small margin, not a guess.
#
# Z raised 0.09->0.15: also confirmed @LIMIT with healthy reach (asked 90.0
# got 82.2, 0.91 reached), but NOT separately hand-verified the way Y was --
# sized by applying Y's same relative increase (1.73x) rather than a direct
# measurement. Re-check with a manual reach test before trusting this number
# the way Y's is trusted.
#
# X left at 0.12: reach mm showed 0.81 with no @LIMIT flag at the time of
# this change -- it was not the constraint, so raising it would not have
# fixed anything.
MAX_OFFSET_XYZ="${MAX_OFFSET_XYZ:-0.12,0.19,0.15}"
# axis = per-axis limits. norm = the original vector-length clamp.
MAX_OFFSET_MODE="${MAX_OFFSET_MODE:-axis}"
# Axes that match the master's ABSOLUTE coordinate instead of its displacement.
# ABSOLUTE_AXES=y makes the slave hold the same height as your hand.
ABSOLUTE_AXES="${ABSOLUTE_AXES:-}"
ABSOLUTE_TRIM_M="${ABSOLUTE_TRIM_M:-0.0}"
# Constant upward bias so the slave is not spending its spring holding altitude.
# 0.55 N chosen from a controlled reach sweep (2026-08-26): with this bias, Y
# reach at a 100mm commanded offset improved from 68.8mm (0.69 ratio, no
# comp) to 85.2mm (0.85 ratio) with no measurable X regression (0.94-1.02
# reached across the same sweep) and continued Z improvement (0.66->0.85
# reached) -- Z shares some of the same gravity load despite the bias being
# on Y only, consistent with this rig's known base-tilt/non-level-base
# coupling. Pushing past 0.55 (tested to 0.65) showed diminishing Y returns
# and a slight dip, so this is the chosen plateau, not an arbitrary middle
# value. Manual hand-reach confirmed the device's true mechanical range is
# far beyond what any force-budget setting here can reach (~185mm+ from the
# same anchor vs. 85mm at this setting), so this remains a force-budget
# trade-off, not a hardware ceiling -- headroom exists to push further later
# if X/Z tolerance allows.
SLAVE_GRAVITY_COMP_N="${SLAVE_GRAVITY_COMP_N:-0.55}"
SLAVE_GRAVITY_AXIS="${SLAVE_GRAVITY_AXIS:-1}"
# C2's counterpart, added 2026-09-03: the master has the same unweighted-
# vertical-axis problem as the slave (confirmed live -- C2's commanded
# target updated correctly on Y, but the master's own spring could not
# visibly follow even after raising LEFT_STIFFNESS_N_PER_M, while X/Z were
# fine at the same stiffness). Off by default (0.0) -- unlike
# SLAVE_GRAVITY_COMP_N above, this has NOT been through a controlled sweep
# yet, so there is no validated value to default to. Start around 0.15 and
# raise until Y stops sagging, same method as the slave's own tuning.
LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N:-0.0}"
LEFT_GRAVITY_AXIS="${LEFT_GRAVITY_AXIS:-1}"
# Declared master planner parameters, cross-checked live against the
# master controller's own telemetry (bridge_master_slave.py's
# planner_parameter_match_left) -- a mismatch faults the startup state
# machine rather than silently coupling against the wrong physics.
# Defaults match run_master_left.sh's own production defaults (15.0 /
# 0.10 / 2.0 / 0.03). Any launcher that starts a master driver with
# different gains (e.g. 01b_start_master_c2_diagnostic.sh's 5.0 / 0.05 /
# 0.5 diagnostic gains) MUST set these to the same values, or
# planner_parameter_match_left will correctly go false and fault
# (2026-09-10, found via a real-hardware C2-only FAULT: run_bridge.sh
# had no wiring for these four bridge CLI args at all).
MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M:-15.0}"
MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M:-0.10}"
MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N:-2.0}"
MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M:-0.03}"
MAX_SPEED_MPS="${MAX_SPEED_MPS:-0.5}"
DEADBAND_M="${DEADBAND_M:-0.0002}"
RATE_HZ="${RATE_HZ:-100}"
CLUTCH_BUTTON_MASK="${CLUTCH_BUTTON_MASK:-2}"
STIFFNESS_N_PER_M="${STIFFNESS_N_PER_M:-35.0}"
REPORT="${REPORT:-true}"
REPORT_INTERVAL_S="${REPORT_INTERVAL_S:-0.5}"
# C2: slave position -> master target (see run_master_left.sh for the
# corresponding master-side params). Off by default -- same reasoning as
# FORCE_REFLECT below, but this is the position-channel counterpart rather
# than a tracking-error-derived force.
ENABLE_C2="${ENABLE_C2:-false}"
LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M:-0.08}"
# C3: master position -> slave target. On by default -- this has been the
# always-on production channel throughout this project's history; added
# as an explicit, overridable variable (2026-09-10, C2-only real-hardware
# validation prep) ONLY so a genuinely ISOLATED C2-only diagnostic
# (C3_ENABLE=false) is possible without touching bridge_master_slave.py's
# own default. Every existing caller of this script that does not set
# C3_ENABLE keeps the exact same behavior as before this variable existed.
C3_ENABLE="${C3_ENABLE:-true}"
# C1/C4 are not implemented by this bridge; these flags declare that the
# supervised force relay is expected to be running so the bridge composes
# bilateral_4ch.py's safety_state into the single /enable authority.
C1_ENABLE="${C1_ENABLE:-false}"
C4_ENABLE="${C4_ENABLE:-false}"
BILATERAL_STATUS_TOPIC="${BILATERAL_STATUS_TOPIC:-/bilateral_status}"
BILATERAL_STATUS_STALE_S="${BILATERAL_STATUS_STALE_S:-0.5}"
# Force reflection: push back on the master when the slave meets a constraint.
FORCE_REFLECT="${FORCE_REFLECT:-false}"
REFLECT_GAIN="${REFLECT_GAIN:-18.0}"
REFLECT_DEADBAND_M="${REFLECT_DEADBAND_M:-0.004}"
REFLECT_VELOCITY_COMP="${REFLECT_VELOCITY_COMP:-0.05}"
REFLECT_RELEASE_RATIO="${REFLECT_RELEASE_RATIO:-0.6}"
REFLECT_MAX_N="${REFLECT_MAX_N:-1.5}"
REFLECT_ALPHA="${REFLECT_ALPHA:-0.15}"
REFLECT_SPEED_ALPHA="${REFLECT_SPEED_ALPHA:-0.15}"
REFLECT_SLEW_N_PER_S="${REFLECT_SLEW_N_PER_S:-8.0}"
# Only push back when the slave is commanded to move but is not moving.
# This is what keeps free motion completely force-free.
REFLECT_REQUIRE_STALL="${REFLECT_REQUIRE_STALL:-true}"
REFLECT_STALL_RATIO="${REFLECT_STALL_RATIO:-0.5}"
REFLECT_STALL_FLOOR_MPS="${REFLECT_STALL_FLOOR_MPS:-0.004}"
# Damping on the master's own motion. Off: the velocity estimate lags far
# enough that this can pump the oscillation instead of damping it.
REFLECT_MASTER_DAMPING="${REFLECT_MASTER_DAMPING:-0.0}"
WAIT_FOR_DEVICES="${WAIT_FOR_DEVICES:-true}"
WAIT_TIMEOUT_S="${WAIT_TIMEOUT_S:-30}"
# Calibration step (2026-08-27): as each side's anchor is captured, the
# bridge records it here so run_master_left_sim.sh / run_slave_right_sim.sh
# (started after the bridge, per this project's master -> bridge -> slave
# launch order) can seed from the exact same snapshot instead of re-sampling
# the live pose topics themselves. Also carries a diagnostic
# right_minus_mapped_left once both anchors are known -- see
# bridge_master_slave.py's _write_calibration().
CALIBRATION_FILE="${CALIBRATION_FILE:-~/dob_logs/teleop_calibration.json}"

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
# This project's own workspace overlay. Needed since 2026-09-08:
# bridge_master_slave.py imports touch_haptics.msg.BridgeDiagnostics, its
# first-ever dependency on this project's custom message package -- until
# then this script never needed anything beyond the ROS base install. A
# user shell's own ~/.bashrc may source ROS humble too, but there is no
# guarantee it overlays *this* workspace (it may source an unrelated
# project's install/setup.bash instead), so this cannot be left implicit.
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
TOUCH_HAPTICS_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [[ ! -r "${TOUCH_HAPTICS_SETUP}" ]]; then
  echo "[run_bridge][error] workspace not built: ${TOUCH_HAPTICS_SETUP}" >&2
  exit 1
fi
source "${TOUCH_HAPTICS_SETUP}"
set -u

# Device discovery is handled inside the bridge node, not here. Polling with
# `ros2 topic info` spawns a fresh node per call and pays the DDS discovery
# delay every time, which burned the whole timeout budget and looked like a
# freeze. The node already has a live participant, so it just waits for the
# first pose message on each topic.
optional_flags=()
if [[ "${WAIT_FOR_DEVICES}" == "1" || "${WAIT_FOR_DEVICES}" == "true" ]]; then
  optional_flags+=(--wait-timeout-s "${WAIT_TIMEOUT_S}")
else
  optional_flags+=(--wait-timeout-s 0)
fi
if [[ "${REPORT}" == "1" || "${REPORT}" == "true" ]]; then
  optional_flags+=(--report)
fi
if [[ "${ENABLE_C2}" == "1" || "${ENABLE_C2}" == "true" ]]; then
  optional_flags+=(--enable-c2 true)
  echo "[teleop_bridge] C2 ON: slave position -> left_target_topic, left_max_offset=${LEFT_MAX_OFFSET_M} m" \
    "(run_master_left.sh must also be started with ENABLE_C2=true)"
fi
if [[ "${FORCE_REFLECT}" == "1" || "${FORCE_REFLECT}" == "true" ]]; then
  optional_flags+=(--reflect-enable)
  echo "[teleop_bridge] force reflection ON gain=${REFLECT_GAIN} N/m deadband=${REFLECT_DEADBAND_M} m max=${REFLECT_MAX_N} N"
  echo "[teleop_bridge] trigger: |slave target - actual| > deadband + ${REFLECT_VELOCITY_COMP}s * target_speed"
else
  echo "[teleop_bridge] force reflection OFF (set FORCE_REFLECT=true to feel slave resistance)"
fi

echo "[teleop_bridge] scale=${SCALE} axis_map=${AXIS_MAP} max_offset=${MAX_OFFSET_M} m max_speed=${MAX_SPEED_MPS} m/s"
echo "[teleop_bridge] hold left button 2 to clutch (freeze slave and reposition your hand)"

exec python3 "${SCRIPT_DIR}/bridge_master_slave.py" \
  --scale "${SCALE}" \
  --axis-map "${AXIS_MAP}" \
  --calibration-file "${CALIBRATION_FILE}" \
  --max-offset-m "${MAX_OFFSET_M}" \
  --axis-scale "${AXIS_SCALE}" \
  --max-offset-xyz "${MAX_OFFSET_XYZ}" \
  --max-offset-mode "${MAX_OFFSET_MODE}" \
  --absolute-axes "${ABSOLUTE_AXES}" \
  --absolute-trim-m "${ABSOLUTE_TRIM_M}" \
  --left-max-offset-m "${LEFT_MAX_OFFSET_M}" \
  --c3-enable "${C3_ENABLE}" \
  --c1-enable "${C1_ENABLE}" \
  --c4-enable "${C4_ENABLE}" \
  --bilateral-status-topic "${BILATERAL_STATUS_TOPIC}" \
  --bilateral-status-stale-s "${BILATERAL_STATUS_STALE_S}" \
  --slave-gravity-comp-n "${SLAVE_GRAVITY_COMP_N}" \
  --slave-gravity-axis "${SLAVE_GRAVITY_AXIS}" \
  --left-gravity-comp-n "${LEFT_GRAVITY_COMP_N}" \
  --left-gravity-axis "${LEFT_GRAVITY_AXIS}" \
  --master-stiffness-n-per-m "${MASTER_STIFFNESS_N_PER_M}" \
  --master-damping-n-s-per-m "${MASTER_DAMPING_N_S_PER_M}" \
  --master-max-force-n "${MASTER_MAX_FORCE_N}" \
  --master-max-position-error-m "${MASTER_MAX_POSITION_ERROR_M}" \
  --max-speed-mps "${MAX_SPEED_MPS}" \
  --deadband-m "${DEADBAND_M}" \
  --rate-hz "${RATE_HZ}" \
  --clutch-button-mask "${CLUTCH_BUTTON_MASK}" \
  --stiffness-n-per-m "${STIFFNESS_N_PER_M}" \
  --report-interval-s "${REPORT_INTERVAL_S}" \
  --reflect-gain "${REFLECT_GAIN}" \
  --reflect-deadband-m "${REFLECT_DEADBAND_M}" \
  --reflect-velocity-comp "${REFLECT_VELOCITY_COMP}" \
  --reflect-release-ratio "${REFLECT_RELEASE_RATIO}" \
  --reflect-max-n "${REFLECT_MAX_N}" \
  --reflect-alpha "${REFLECT_ALPHA}" \
  --reflect-speed-alpha "${REFLECT_SPEED_ALPHA}" \
  --reflect-slew-n-per-s "${REFLECT_SLEW_N_PER_S}" \
  --reflect-require-stall "${REFLECT_REQUIRE_STALL}" \
  --reflect-stall-ratio "${REFLECT_STALL_RATIO}" \
  --reflect-stall-floor-mps "${REFLECT_STALL_FLOOR_MPS}" \
  --reflect-master-damping "${REFLECT_MASTER_DAMPING}" \
  "${optional_flags[@]}"
