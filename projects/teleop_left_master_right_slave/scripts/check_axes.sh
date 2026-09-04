#!/usr/bin/env bash
set -euo pipefail
# Diagnoses which slave axis responds to each master axis.
# Run with all three terminals up, then move the master one direction at a time.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RATE_HZ="${RATE_HZ:-2}"
MIN_TRAVEL_M="${MIN_TRAVEL_M:-0.015}"
export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY
set +u; source /opt/ros/humble/setup.bash; set -u
exec python3 "${SCRIPT_DIR}/check_axes.py" --rate-hz "${RATE_HZ}" --min-travel-m "${MIN_TRAVEL_M}"
