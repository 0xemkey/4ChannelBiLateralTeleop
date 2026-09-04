#!/usr/bin/env bash
set -euo pipefail

# Records one real-vs-visual-encoder session for the Phase 1 noise harness
# (docs/FOUR_CHANNEL_TODO.md's follow-on plan). Start this after the real
# master/slave (run_master_left.sh, run_slave_right.sh) AND their sim twins
# (run_master_left_sim.sh, run_slave_right_sim.sh) are all up.
#
# Records both real and _sim topics for both arms, plus contact annotations
# so Phase 1/2's plotting and separability scripts can align windows.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"

TOUCH_WS="${TOUCH_WS:-${REPO_ROOT}/ros2_ws}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
BAG_ROOT="${BAG_ROOT:-${HOME}/dob_logs}"
BAG_NAME="${BAG_NAME:-real_vs_sim_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="${BAG_ROOT}/${BAG_NAME}"

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
if [[ -f "${TOUCH_WS}/install/setup.bash" ]]; then
  source "${TOUCH_WS}/install/setup.bash"
fi
set -u

mkdir -p "${BAG_ROOT}"

topics=(
  # real master
  /touch_left/pose
  /touch_left/applied_force
  /touch_left/dob_diagnostics
  /touch_left/force_ext_hat
  /touch_left/joint_angles
  /touch_left/target_pose
  /touch_left/force_cmd
  # visual (sim) master twin
  /touch_left_sim/pose
  /touch_left_sim/applied_force
  /touch_left_sim/dob_diagnostics
  /touch_left_sim/force_ext_hat
  /touch_left_sim/joint_angles
  # real slave
  /touch_right/pose
  /touch_right/applied_force
  /touch_right/dob_diagnostics
  /touch_right/force_ext_hat
  /touch_right/joint_angles
  /touch_right/target_pose
  /touch_right/force_cmd
  # visual (sim) slave twin
  /touch_right_sim/pose
  /touch_right_sim/applied_force
  /touch_right_sim/dob_diagnostics
  /touch_right_sim/force_ext_hat
  /touch_right_sim/joint_angles
  # annotation, same convention as the 2026-08-26 separability session
  /contact_annotation
)

{
  echo "bag_dir=${BAG_DIR}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "touch_ws=${TOUCH_WS}"
  echo "created_at=$(date --iso-8601=seconds)"
  echo "topics=${topics[*]}"
} > "${BAG_DIR}.env"

echo "[real_vs_sim_bag] output=${BAG_DIR}"
echo "[real_vs_sim_bag] press Ctrl+C to stop recording"

exec ros2 bag record -o "${BAG_DIR}" "${topics[@]}"
