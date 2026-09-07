#!/usr/bin/env bash
set -euo pipefail
LAUNCH_DIR="$(cd "$(dirname "$0")" && pwd)"
SESSION_ID=""; OUTPUT_DIR=""; LABEL=""
while (($#)); do
  case "$1" in
    --session) SESSION_ID="${2:?}"; shift 2 ;;
    --output-dir) OUTPUT_DIR="${2:?}"; shift 2 ;;
    --label) LABEL="${2:?}"; shift 2 ;;
    *) echo "[event][error] unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -n "$SESSION_ID" && -n "$OUTPUT_DIR" && -n "$LABEL" ]] || {
  echo '[event][error] --session, --output-dir and --label are required' >&2; exit 2; }
source /opt/ros/humble/setup.bash
source /home/nural/HIL-RL/ros2_ws/install/setup.bash
session_dir="${OUTPUT_DIR}/${SESSION_ID}"
mkdir -p "${session_dir}/events"
stamp="$(date --iso-8601=ns)"
printf '%s\t%s\n' "$stamp" "$LABEL" >>"${session_dir}/events/operator_events.tsv"
ros2 topic pub --once /operator_event std_msgs/msg/String "{data: '${LABEL}'}"
echo "[event] ${stamp} ${LABEL}"

