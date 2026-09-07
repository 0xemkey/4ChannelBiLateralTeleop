#!/usr/bin/env bash
set -euo pipefail

LAUNCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${LAUNCH_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"
TOUCH_WS="${TOUCH_WS:-${REPO_ROOT}/ros2_ws}"
RUN_ROOT_DEFAULT="${REPO_ROOT}/experiment_runs/teleop_setup"

SESSION_ID="${SESSION_ID:-}"
OUTPUT_DIR="${OUTPUT_DIR:-${RUN_ROOT_DEFAULT}}"
CONTROLLER_MODE="${CONTROLLER_MODE:-dob_observe}"
OPERATOR_NOTES="${OPERATOR_NOTES:-}"
DRY_RUN=false

parse_common_args() {
  while (($#)); do
    case "$1" in
      --session) SESSION_ID="${2:?--session needs a value}"; shift 2 ;;
      --output-dir) OUTPUT_DIR="${2:?--output-dir needs a value}"; shift 2 ;;
      --mode) CONTROLLER_MODE="${2:?--mode needs a value}"; shift 2 ;;
      --notes) OPERATOR_NOTES="${2:?--notes needs a value}"; shift 2 ;;
      --dry-run) DRY_RUN=true; shift ;;
      *) EXTRA_ARGS+=("$1"); shift ;;
    esac
  done
  if [[ -z "${SESSION_ID}" ]]; then
    SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)_dob_observe"
  fi
  if [[ ! "${SESSION_ID}" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "[setup][error] unsafe session id: ${SESSION_ID}" >&2
    exit 2
  fi
  SESSION_DIR="${OUTPUT_DIR}/${SESSION_ID}"
  PID_DIR="${SESSION_DIR}/pids"
  LOG_DIR="${SESSION_DIR}/logs"
  MANIFEST="${SESSION_DIR}/manifest.env"
}

source_ros() {
  [[ -r /opt/ros/humble/setup.bash ]] || {
    echo "[setup][error] missing /opt/ros/humble/setup.bash" >&2; exit 1; }
  [[ -r "${TOUCH_WS}/install/setup.bash" ]] || {
    echo "[setup][error] workspace not built: ${TOUCH_WS}/install/setup.bash" >&2; exit 1; }
  set +u
  source /opt/ros/humble/setup.bash
  source "${TOUCH_WS}/install/setup.bash"
  set -u
}

print_command() { printf ' '; printf '%q ' "$@"; printf '\n'; }

require_session() {
  if [[ "${DRY_RUN}" == true ]]; then return; fi
  [[ -d "${SESSION_DIR}" && -f "${MANIFEST}" ]] || {
    echo "[setup][error] run 00_preflight.sh first for session ${SESSION_ID}" >&2; exit 1; }
}

reject_live_component() {
  local pattern="$1" label="$2"
  if ps -eo args= | grep -E "${pattern}" | grep -v -E 'grep|experiment_launch' >/dev/null; then
    echo "[setup][error] conflicting ${label} process already exists" >&2
    exit 1
  fi
}

start_component() {
  local name="$1"; shift
  require_session
  echo "[setup] ${name} command:"
  print_command "$@"
  if [[ "${DRY_RUN}" == true ]]; then return; fi
  [[ ! -f "${PID_DIR}/${name}.pid" ]] || {
    echo "[setup][error] ${name} already recorded in this session" >&2; exit 1; }
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${PID_DIR}/${name}.pid"
  printf '%s\n' "${pid}" >"${PID_DIR}/${name}.pgid"
  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[setup][error] ${name} exited; see ${LOG_DIR}/${name}.log" >&2
    return 1
  fi
  echo "[setup] ${name} started pid/pgid=${pid}; log=${LOG_DIR}/${name}.log"
}

