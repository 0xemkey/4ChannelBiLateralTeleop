#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); LABEL=""
source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
i=0
while ((i < ${#EXTRA_ARGS[@]})); do
  case "${EXTRA_ARGS[$i]}" in
    --label) LABEL="${EXTRA_ARGS[$((i+1))]:?--label needs a value}"; i=$((i+2)) ;;
    *) echo "[event][error] unknown argument: ${EXTRA_ARGS[$i]}" >&2; exit 2 ;;
  esac
done
[[ -n "$LABEL" ]] || { echo '[event][error] --label is required' >&2; exit 2; }
require_session
source_ros
session_dir="${SESSION_DIR}"
mkdir -p "${session_dir}/events"
stamp="$(date --iso-8601=ns)"
printf '%s\t%s\n' "$stamp" "$LABEL" >>"${session_dir}/events/operator_events.tsv"
ros2 topic pub --once /operator_event std_msgs/msg/String "{data: '${LABEL}'}"
echo "[event] ${stamp} ${LABEL}"

