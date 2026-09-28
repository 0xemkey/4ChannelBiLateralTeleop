#!/usr/bin/env bash
set -euo pipefail
# 2026-09-10: reusable real-hardware test orchestrator. Owns cleanup/
# ownership resolution, session creation, approved-launcher startup,
# pre-handoff validation, and read-only recording -- so the operator no
# longer manually copies launcher commands. NEVER moves either device
# itself: it starts only approved bridge/relay configurations and a
# subscribe-only recorder; all motion/disturbance excitation is performed
# by the operator by hand. See the top-level task's MAY/MUST-NOT list.
#
# Usage:
#   run_hardware_test.sh --mode c2-only  [--purpose TEXT] [--master-max-force-n F] \
#       [--master-max-position-error-m E] [--duration-s N] [--session ID] [--no-recorder]
#   run_hardware_test.sh --mode c2c3     [same flags]
#   run_hardware_test.sh --mode c1-only|c4-only|full4ch [same flags]
#   run_hardware_test.sh --mode full4ch --manual-stop [same flags]
#   run_hardware_test.sh --stop --session ID
#   run_hardware_test.sh --post-analyze OUT_DIR

LAUNCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${LAUNCH_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"
RUN_ROOT="${REPO_ROOT}/experiment_runs/teleop_setup"

MODE=""
PURPOSE="test"
SESSION_ID=""
MASTER_MAX_FORCE_N_OVERRIDE=""
MASTER_MAX_POSITION_ERROR_M_OVERRIDE=""
MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M:-15.0}"
MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M:-0.10}"
LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N:-0.55}"
LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M:-0.08}"
DURATION_S=90
WANT_RECORDER=true
MANUAL_STOP=false
STOP_ONLY=false
STOP_SESSION=""
POST_ANALYZE_DIR=""

while (($#)); do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --purpose) PURPOSE="$2"; shift 2 ;;
    --session) SESSION_ID="$2"; shift 2 ;;
    --master-max-force-n) MASTER_MAX_FORCE_N_OVERRIDE="$2"; shift 2 ;;
    --master-max-position-error-m) MASTER_MAX_POSITION_ERROR_M_OVERRIDE="$2"; shift 2 ;;
    --duration-s) DURATION_S="$2"; shift 2 ;;
    --no-recorder) WANT_RECORDER=false; shift ;;
    --manual-stop) MANUAL_STOP=true; shift ;;
    --stop) STOP_ONLY=true; shift ;;
    --post-analyze) POST_ANALYZE_DIR="$2"; shift 2 ;;
    *) echo "[run-hw-test][error] unknown argument: $1" >&2; exit 2 ;;
  esac
done

log() { echo "[run-hw-test] $*"; }
err() { echo "[run-hw-test][error] $*" >&2; }

print_command() { printf ' '; printf '%q ' "$@"; printf '\n'; }

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
  set -u
}

reject_live_component() {
  local pattern="$1" label="$2"
  if ps -eo args= | grep -E "${pattern}" | grep -v -E 'grep|experiment_launch' >/dev/null; then
    err "conflicting ${label} process already exists"
    exit 1
  fi
}

start_component() {
  local name="$1"; shift
  log "${name} command:"
  print_command "$@"
  [[ ! -f "${PID_DIR}/${name}.pid" ]] || {
    err "${name} already recorded in this session"
    exit 1
  }
  setsid "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
  local pid=$!
  printf '%s\n' "${pid}" >"${PID_DIR}/${name}.pid"
  printf '%s\n' "${pid}" >"${PID_DIR}/${name}.pgid"
  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    err "${name} exited; see ${LOG_DIR}/${name}.log"
    exit 1
  fi
  log "${name} started pid/pgid=${pid}; log=${LOG_DIR}/${name}.log"
}

# ---------------------------------------------------------------------------
# --post-analyze: standalone path, no session/process interaction at all.
# ---------------------------------------------------------------------------
if [[ -n "${POST_ANALYZE_DIR}" ]]; then
  ANALYSIS_STATUS=0
  python3 "${PROJECT_ROOT}/hardware_validation/analyze_manual_motion.py" "${POST_ANALYZE_DIR}" || ANALYSIS_STATUS=$?
  SESSION_DIR_FOR_POST="$(cd "${POST_ANALYZE_DIR}/../.." && pwd)"
  if [[ -f "${PROJECT_ROOT}/hardware_validation/analyze_full4ch_baseline.py" ]]; then
    python3 "${PROJECT_ROOT}/hardware_validation/analyze_full4ch_baseline.py" \
      --session-dir "${SESSION_DIR_FOR_POST}" --trial-dir "${POST_ANALYZE_DIR}" || true
  fi
  exit "${ANALYSIS_STATUS}"
fi

# ---------------------------------------------------------------------------
# --stop: standalone path.
# ---------------------------------------------------------------------------
if [[ "${STOP_ONLY}" == true ]]; then
  [[ -n "${SESSION_ID}" ]] || { err "--stop requires --session"; exit 2; }
  exec bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}"
fi

[[ -n "${MODE}" ]] || { err "--mode is required (c2-only|c2c3|c1-only|c4-only|full4ch)"; exit 2; }

case "${MODE}" in
  c2-only|c2c3|c1-only|c4-only|full4ch) ;;
  *) err "unknown --mode ${MODE} (expected c2-only|c2c3|c1-only|c4-only|full4ch)"; exit 2 ;;
esac

# ---------------------------------------------------------------------------
# PHASE 1: cleanup / ownership resolution.
#
# Never trusts ".current_session" or "newest experiment_runs directory" as
# proof of ownership by itself -- only a LIVE PID that (a) matches a
# pattern for a teleop component AND (b) is byte-identical to a PID
# recorded in some session's pids/*.pid file counts as "owned". A live
# matching process with NO recorded owner, or matching DIFFERENT sessions
# for different components, is a hard-fail-and-explain condition.
# ---------------------------------------------------------------------------
declare -A OWNER_SESSION   # component -> session_id
declare -A OWNER_PID       # component -> pid
UNOWNED_LIVE=()

find_live_pid() {
  # $1 = grep -E pattern. Prints the FIRST matching PID (there should be
  # at most one live master/slave/bridge/relay at a time; reject_live_
  # component in the individual launchers is what actually enforces that).
  #
  # "|| true" is required, not decorative: under `set -o pipefail`, a grep
  # that matches ZERO lines (the normal, expected case -- e.g. no relay
  # process running) exits 1, and pipefail propagates that as the whole
  # pipeline's exit status even though awk/head both succeed downstream.
  # Without this guard, `pid="$(find_live_pid ...)"` under `set -e` aborts
  # the ENTIRE script silently (no output at all) the first time a pattern
  # legitimately finds nothing -- confirmed live: this is exactly what
  # happened on this script's first real invocation.
  ps -eo pid=,args= | grep -E "$1" | grep -v grep | awk '{print $1}' | head -1 || true
}

resolve_component() {
  local label="$1" pattern="$2" pid
  pid="$(find_live_pid "${pattern}")"
  [[ -n "${pid}" ]] || return 0
  local found_session=""
  local f
  for f in "${RUN_ROOT}"/*/pids/*.pid; do
    [[ -e "${f}" ]] || continue
    local recorded_pid
    recorded_pid="$(<"${f}")"
    if [[ "${recorded_pid}" == "${pid}" ]]; then
      found_session="$(basename "$(dirname "$(dirname "${f}")")")"
      break
    fi
  done
  if [[ -z "${found_session}" ]]; then
    UNOWNED_LIVE+=("${label} pid=${pid} (no session pids/*.pid file matches this PID)")
  else
    OWNER_SESSION["${label}"]="${found_session}"
    OWNER_PID["${label}"]="${pid}"
  fi
}

resolve_component "master" 'touch_slave.*__node:=teleop_master_left'
resolve_component "slave" 'touch_slave.*__node:=teleop_slave_right'
resolve_component "bridge" 'bridge_master_slave\.py'
resolve_component "relay" 'bilateral_4ch\.py'

if ((${#UNOWNED_LIVE[@]} > 0)); then
  err "cannot establish ownership of the following live process(es) -- refusing to touch them:"
  for u in "${UNOWNED_LIVE[@]}"; do err "  ${u}"; done
  err "resolve this manually before running run_hardware_test.sh."
  exit 1
fi

RESOLVED_SESSIONS=()
for k in "${!OWNER_SESSION[@]}"; do RESOLVED_SESSIONS+=("${OWNER_SESSION[$k]}"); done
UNIQUE_SESSIONS=($(printf '%s\n' "${RESOLVED_SESSIONS[@]+"${RESOLVED_SESSIONS[@]}"}" | sort -u))

if ((${#UNIQUE_SESSIONS[@]} > 1)); then
  err "live teleop components resolve to DIFFERENT sessions -- refusing to guess which to stop:"
  for k in "${!OWNER_SESSION[@]}"; do err "  ${k} -> session=${OWNER_SESSION[$k]} pid=${OWNER_PID[$k]}"; done
  exit 1
fi

if ((${#UNIQUE_SESSIONS[@]} == 1)); then
  OLD_SESSION="${UNIQUE_SESSIONS[0]}"
  log "identified a currently-owned, PID-verified live session: ${OLD_SESSION}"
  for k in "${!OWNER_SESSION[@]}"; do log "  ${k} pid=${OWNER_PID[$k]}"; done
  log "stopping it (a fresh session with this invocation's own parameters is required next) ..."
  bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${OLD_SESSION}"
  sleep 1
  STILL_LIVE=""
  for pat in 'touch_slave.*__node:=teleop_master_left' 'touch_slave.*__node:=teleop_slave_right' \
             'bridge_master_slave\.py' 'bilateral_4ch\.py'; do
    p="$(find_live_pid "${pat}")"
    [[ -z "${p}" ]] || STILL_LIVE+="${pat}=pid:${p} "
  done
  if [[ -n "${STILL_LIVE}" ]]; then
    err "99_stop_setup.sh returned but these are still alive: ${STILL_LIVE}"
    err "refusing to proceed -- investigate manually (do not force-kill unrelated processes)."
    exit 1
  fi
  log "verified clean: no master/slave/bridge/relay process remains."
else
  log "no live teleop master/slave/bridge/relay process found -- nothing to clean up."
fi

# ---------------------------------------------------------------------------
# PHASE 2: create a fresh, uniquely-named session.
# ---------------------------------------------------------------------------
if [[ -z "${SESSION_ID}" ]]; then
  SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)_${MODE//-/}_${PURPOSE}"
fi
SESSION_DIR="${RUN_ROOT}/${SESSION_ID}"
PID_DIR="${SESSION_DIR}/pids"
LOG_DIR="${SESSION_DIR}/logs"
log "session: ${SESSION_ID}"
bash "${LAUNCH_DIR}/00_preflight.sh" --session "${SESSION_ID}"
log "preflight passed."

# ---------------------------------------------------------------------------
# PHASE 3: start the approved configuration for this mode.
# ---------------------------------------------------------------------------
MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N_OVERRIDE:-2.0}"
MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M_OVERRIDE:-0.03}"
log "master controller for this run: K=${MASTER_STIFFNESS_N_PER_M} D=${MASTER_DAMPING_N_S_PER_M} max_force_n=${MASTER_MAX_FORCE_N} max_position_error_m=${MASTER_MAX_POSITION_ERROR_M}"
{
  printf 'RUN_HARDWARE_TEST_MODE=%q\n' "${MODE}"
  printf 'RUN_HARDWARE_TEST_PURPOSE=%q\n' "${PURPOSE}"
  printf 'RUN_HARDWARE_TEST_DURATION_S=%q\n' "${DURATION_S}"
  printf 'RUN_HARDWARE_TEST_MANUAL_STOP=%q\n' "${MANUAL_STOP}"
  printf 'MASTER_STIFFNESS_N_PER_M=%q\n' "${MASTER_STIFFNESS_N_PER_M}"
  printf 'MASTER_DAMPING_N_S_PER_M=%q\n' "${MASTER_DAMPING_N_S_PER_M}"
  printf 'MASTER_MAX_FORCE_N=%q\n' "${MASTER_MAX_FORCE_N}"
  printf 'MASTER_MAX_POSITION_ERROR_M=%q\n' "${MASTER_MAX_POSITION_ERROR_M}"
  printf 'LEFT_GRAVITY_COMP_N=%q\n' "${LEFT_GRAVITY_COMP_N}"
  printf 'LEFT_MAX_OFFSET_M=%q\n' "${LEFT_MAX_OFFSET_M}"
} >>"${SESSION_DIR}/manifest.env"

if [[ "${MODE}" == c2-only || "${MODE}" == c2c3 || "${MODE}" == full4ch ]]; then
  ENABLE_C2=true MAX_FORCE_N="${MASTER_MAX_FORCE_N}" \
    LEFT_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M}" \
    LEFT_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M}" \
    LEFT_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}" \
    bash "${LAUNCH_DIR}/01_start_master.sh" --session "${SESSION_ID}"
else
  bash "${LAUNCH_DIR}/01_start_master.sh" --session "${SESSION_ID}"
fi
bash "${LAUNCH_DIR}/02_start_slave.sh" --session "${SESSION_ID}"

case "${MODE}" in
  c2-only)
    reject_live_component 'bridge_master_slave.py' 'position bridge'
    cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true C3_ENABLE=false
      C1_ENABLE=false C4_ENABLE=false FORCE_REFLECT=false
      SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15
      LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M}" LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N}" LEFT_GRAVITY_AXIS=1
      MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M}" MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M}"
      MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N}" MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}"
      MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.0 RATE_HZ=100
      CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json"
      "${PROJECT_ROOT}/scripts/run_bridge.sh")
    start_component bridge "${cmd[@]}"
    source_ros
    python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 false --expected-c2 true --expected-c3 false --expected-c4 false
    EXPECTED_C1=false; EXPECTED_C2=true; EXPECTED_C3=false; EXPECTED_C4=false
    ;;
  c2c3)
    reject_live_component 'bridge_master_slave.py' 'position bridge'
    cmd=(env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" ENABLE_C2=true C3_ENABLE=true
      C1_ENABLE=false C4_ENABLE=false FORCE_REFLECT=false
      SCALE=1.0 AXIS_MAP=x,y,z AXIS_SCALE=1.0,1.0,1.0 MAX_OFFSET_XYZ=0.12,0.19,0.15
      LEFT_MAX_OFFSET_M="${LEFT_MAX_OFFSET_M}" LEFT_GRAVITY_COMP_N="${LEFT_GRAVITY_COMP_N}" LEFT_GRAVITY_AXIS=1
      MASTER_STIFFNESS_N_PER_M="${MASTER_STIFFNESS_N_PER_M}" MASTER_DAMPING_N_S_PER_M="${MASTER_DAMPING_N_S_PER_M}"
      MASTER_MAX_FORCE_N="${MASTER_MAX_FORCE_N}" MASTER_MAX_POSITION_ERROR_M="${MASTER_MAX_POSITION_ERROR_M}"
      MAX_SPEED_MPS=0.5 SLAVE_GRAVITY_COMP_N=0.55 RATE_HZ=100
      CALIBRATION_FILE="${SESSION_DIR}/teleop_calibration.json"
      "${PROJECT_ROOT}/scripts/run_bridge.sh")
    start_component bridge "${cmd[@]}"
    source_ros
    python3 "${PROJECT_ROOT}/hardware_validation/arm_production_bridge.py" \
      --expected-c1 false --expected-c2 true --expected-c3 true --expected-c4 false
    EXPECTED_C1=false; EXPECTED_C2=true; EXPECTED_C3=true; EXPECTED_C4=false
    ;;
  c1-only)
    bash "${LAUNCH_DIR}/08_qualify_c1_only.sh" --session "${SESSION_ID}" --arm-relay
    EXPECTED_C1=true; EXPECTED_C2=false; EXPECTED_C3=false; EXPECTED_C4=false
    ;;
  c4-only)
    bash "${LAUNCH_DIR}/09_qualify_c4_only.sh" --session "${SESSION_ID}" --arm-relay
    EXPECTED_C1=false; EXPECTED_C2=false; EXPECTED_C3=false; EXPECTED_C4=true
    ;;
  full4ch)
    bash "${LAUNCH_DIR}/10_start_full_4ch_production.sh" --session "${SESSION_ID}" --arm-relay
    EXPECTED_C1=true; EXPECTED_C2=true; EXPECTED_C3=true; EXPECTED_C4=true
    ;;
esac
{
  printf 'EXPECTED_CHANNELS=%q\n' "C1=${EXPECTED_C1},C2=${EXPECTED_C2},C3=${EXPECTED_C3},C4=${EXPECTED_C4}"
  printf 'ENABLE_PUBLISHER=%q\n' 'bridge_master_slave.py only'
} >>"${SESSION_DIR}/manifest.env"
log "bridge armed and ACTIVE (arm_production_bridge.py already verified: instance/ownership,"
log "pre/post-arm freshness, state progression, channel state, planner_parameter_match, positive"
log "force-budget margins)."

# ---------------------------------------------------------------------------
# PHASE 4: pre-handoff validation -- the checks arm_production_bridge.py
# does not already cover (explicit K/D/Fmax/max_error printout, no clamp
# latched at rest, applied_force_n plausibility).
# ---------------------------------------------------------------------------
# ROS's own setup.bash references unset variables (e.g.
# AMENT_TRACE_SETUP_FILES) -- fatal under this script's `set -u`. Confirmed
# live: this crashed the orchestrator immediately after a real bridge had
# already reached ACTIVE on real hardware, right before the pre-handoff
# check/recorder -- same "set +u around ROS sourcing" pattern common.sh's
# own source_ros() already uses, just not reachable here since this script
# doesn't source common.sh (it shells out to the launchers instead).
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u
log "running pre-handoff check ..."
if ! python3 "${PROJECT_ROOT}/hardware_validation/pre_handoff_check.py" \
    --expected-stiffness "${MASTER_STIFFNESS_N_PER_M}" --expected-damping "${MASTER_DAMPING_N_S_PER_M}" \
    --expected-max-force "${MASTER_MAX_FORCE_N}" --expected-max-position-error "${MASTER_MAX_POSITION_ERROR_M}"; then
  err "pre-handoff check FAILED -- stopping session ${SESSION_ID}, not handing off to the operator."
  bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}" || true
  exit 1
fi
log "pre-handoff check passed."

# ---------------------------------------------------------------------------
# PHASE 5: start the read-only recorder.
# ---------------------------------------------------------------------------
RECORDER_OUT="${RUN_ROOT}/${SESSION_ID}/manual_motion/${PURPOSE}"
RECORDER_STOP_FILE="${SESSION_DIR}/STOP_RECORDING"
mkdir -p "${RECORDER_OUT}"
if [[ "${WANT_RECORDER}" == true ]]; then
  RECORDER_ARGS=(
    "${PROJECT_ROOT}/hardware_validation/record_manual_motion.py"
    --out-dir "${RECORDER_OUT}" --duration-s "${DURATION_S}"
    --expected-c1 "${EXPECTED_C1}" --expected-c2 "${EXPECTED_C2}"
    --expected-c3 "${EXPECTED_C3}" --expected-c4 "${EXPECTED_C4}"
    --expected-stage "${MODE}" --notes "${PURPOSE}"
    --allow-live-domain-CHECK-DISABLED
  )
  if [[ "${MANUAL_STOP}" == true ]]; then
    rm -f "${RECORDER_STOP_FILE}"
    RECORDER_ARGS+=(--stop-file "${RECORDER_STOP_FILE}")
  fi
  nohup python3 "${PROJECT_ROOT}/hardware_validation/record_manual_motion.py" \
    "${RECORDER_ARGS[@]:1}" \
    >"${RUN_ROOT}/${SESSION_ID}/logs/recorder.log" 2>&1 &
  RECORDER_PID=$!
  echo "${RECORDER_PID}" > "${RUN_ROOT}/${SESSION_ID}/pids/recorder.pid"
  echo "${RECORDER_PID}" > "${RUN_ROOT}/${SESSION_ID}/pids/recorder.pgid"
  log "recorder started: pid=${RECORDER_PID} out=${RECORDER_OUT} duration=${DURATION_S}s"
else
  RECORDER_PID=""
  log "recorder skipped (--no-recorder)."
fi

case "${MODE}" in
  c2-only)
    OPERATOR_INSTRUCTION="Do not touch the master. Move the slave slowly +Y by ~2 cm, hold briefly, then return."
    ;;
  c2c3)
    OPERATOR_INSTRUCTION="Move the master slowly +/-Y by ~2 cm, then move the slave slowly +/-Y by ~2 cm, returning each to rest."
    ;;
  c1-only)
    OPERATOR_INSTRUCTION="Do not touch the master. Apply and hold a small disturbance on the slave, then release."
    ;;
  c4-only)
    OPERATOR_INSTRUCTION="Do not touch the slave. Apply and hold a small disturbance on the master, then release."
    ;;
  full4ch)
    OPERATOR_INSTRUCTION="Move master +/-Y, move slave +/-Y, resist slave briefly, resist master briefly, then release both."
    ;;
esac

echo ""
echo "============================================================"
echo "HARDWARE READY — OPERATOR TAKE CONTROL"
echo "SESSION=${SESSION_ID}"
if [[ "${MODE}" == full4ch ]]; then
  echo "MODE=FULL 4CH"
  echo "RECORDER=RUNNING"
else
  echo "MODE=${MODE}"
fi
echo "============================================================"
echo "${OPERATOR_INSTRUCTION}"
if [[ "${MODE}" == full4ch ]]; then
  echo "A. Move master +/-Y, hold briefly, return."
  echo "B. Move slave +/-Y, hold briefly, return."
  echo "C. Resist slave briefly, then release."
  echo "D. Resist master briefly, then release."
  echo "E. Return both devices to neutral and tell Codex: done."
fi

if [[ "${WANT_RECORDER}" == true ]]; then
  if [[ "${MANUAL_STOP}" == true ]]; then
    log "waiting for recorder pid=${RECORDER_PID}; create ${RECORDER_STOP_FILE} when operator says done (max ${DURATION_S}s)."
  else
    log "waiting for recorder pid=${RECORDER_PID} to finish (${DURATION_S}s window) ..."
  fi
  RECORDER_STATUS=0
  wait "${RECORDER_PID}" || RECORDER_STATUS=$?
  if [[ "${RECORDER_STATUS}" -ne 0 ]]; then
    err "recorder exited with status ${RECORDER_STATUS}; stopping session before analysis."
    bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}" || true
    exit "${RECORDER_STATUS}"
  fi
  log "recorder finished; running post-analysis."
  ANALYSIS_STATUS=0
  python3 "${PROJECT_ROOT}/hardware_validation/analyze_manual_motion.py" "${RECORDER_OUT}" || ANALYSIS_STATUS=$?
  if [[ "${MODE}" == full4ch ]]; then
    python3 "${PROJECT_ROOT}/hardware_validation/analyze_full4ch_baseline.py" \
      --session-dir "${SESSION_DIR}" --trial-dir "${RECORDER_OUT}" || true
  fi
  if [[ "${ANALYSIS_STATUS}" -eq 0 ]]; then
    case "${MODE}" in
      c1-only)
        python3 "${PROJECT_ROOT}/hardware_validation/write_qualification_artifact.py" \
          --trial-dir "${RECORDER_OUT}" --channel c1 --session "${SESSION_ID}" \
          --out-dir "${PROJECT_ROOT}/hardware_validation/qualification"
        ;;
      c4-only)
        python3 "${PROJECT_ROOT}/hardware_validation/write_qualification_artifact.py" \
          --trial-dir "${RECORDER_OUT}" --channel c4 --session "${SESSION_ID}" \
          --out-dir "${PROJECT_ROOT}/hardware_validation/qualification"
        ;;
    esac
  fi
  log "stopping session ${SESSION_ID} after recording/analysis."
  bash "${LAUNCH_DIR}/99_stop_setup.sh" --session "${SESSION_ID}" || true
  exit "${ANALYSIS_STATUS}"
fi
