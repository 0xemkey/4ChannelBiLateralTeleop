#!/usr/bin/env bash
set -euo pipefail

# Terminal 4: live force readout for both arms side by side.
# Start the master and slave first; this only listens.
#
# Both columns are COMMANDED force after clamping, read from each node's
# applied_force topic. Neither Touch can measure force. The track_err column is
# the only number here that reacts to the physical world.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RATE_HZ="${RATE_HZ:-5}"
HEADER_EVERY="${HEADER_EVERY:-20}"
STIFFNESS_N_PER_M="${STIFFNESS_N_PER_M:-35.0}"
LOAD_FLAG_N="${LOAD_FLAG_N:-0.15}"

export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY

set +u
source /opt/ros/humble/setup.bash
set -u

exec python3 "${SCRIPT_DIR}/monitor_forces.py" \
  --rate-hz "${RATE_HZ}" \
  --header-every "${HEADER_EVERY}" \
  --stiffness-n-per-m "${STIFFNESS_N_PER_M}" \
  --load-flag-n "${LOAD_FLAG_N}"
