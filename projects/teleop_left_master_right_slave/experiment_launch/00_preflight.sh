#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=()
source "$(cd "$(dirname "$0")" && pwd)/common.sh"
parse_common_args "$@"
[[ ${#EXTRA_ARGS[@]} -eq 0 ]] || { echo "[preflight][error] unknown argument: ${EXTRA_ARGS[*]}" >&2; exit 2; }

LEFT_INERTIA="${HOME}/dob_logs/results/inertia_left.json"
LEFT_GRAVITY="${HOME}/dob_logs/results/gravity_left_minimal.json"
RIGHT_INERTIA="${HOME}/dob_logs/results/inertia_right.json"
RIGHT_GRAVITY="${HOME}/dob_logs/results/gravity_right_minimal.json"
LEFT_CFG="${HOME}/.3dsystems/config/Left.config"
RIGHT_CFG="${HOME}/.3dsystems/config/Default Device.config"

source_ros
command -v ros2 >/dev/null || { echo "[preflight][error] ros2 unavailable" >&2; exit 1; }
command -v python3 >/dev/null || { echo "[preflight][error] python3 unavailable" >&2; exit 1; }
[[ -x "${TOUCH_WS}/install/touch_haptics/lib/touch_haptics/touch_slave" ]] || {
  echo "[preflight][error] touch_slave executable missing" >&2; exit 1; }
for f in "${LEFT_INERTIA}" "${LEFT_GRAVITY}" "${RIGHT_INERTIA}" "${RIGHT_GRAVITY}" "${LEFT_CFG}" "${RIGHT_CFG}"; do
  [[ -r "$f" ]] || { echo "[preflight][error] missing required file: $f" >&2; exit 1; }
done
grep -q '^Channel=300$' "${LEFT_CFG}" || { echo "[preflight][error] Left.config must select channel 300" >&2; exit 1; }
grep -q '^DeviceSerialNumber=18019000358$' "${LEFT_CFG}" || { echo "[preflight][error] Left.config serial identity check failed" >&2; exit 1; }
grep -q '^Channel=301$' "${RIGHT_CFG}" || { echo "[preflight][error] Default Device.config must select channel 301" >&2; exit 1; }
grep -q '^DeviceSerialNumber=18019000365$' "${RIGHT_CFG}" || { echo "[preflight][error] Default Device.config serial identity check failed" >&2; exit 1; }
cmp -s "${LEFT_CFG}" "${RIGHT_CFG}" && {
  echo "[preflight][error] left/right device configs are identical; assignment is ambiguous" >&2; exit 1; }
reject_live_component 'touch_(publisher|slave)' 'Touch hardware owner'
reject_live_component 'bridge_master_slave.py' 'position bridge'
reject_live_component 'bilateral_4ch.py' 'force relay'

if [[ "${DRY_RUN}" == false ]]; then
  python3 "${PROJECT_ROOT}/hardware_validation/check_touch_devices.py" \
    --left-config "${LEFT_CFG}" \
    --right-config "${RIGHT_CFG}"
fi

current_file="${LAUNCH_DIR}/.current_session"
if [[ -f "${current_file}" ]]; then
  old="$(<"${current_file}")"
  [[ ! -d "${old}" ]] || { echo "[preflight][error] stale/active session exists: ${old}" >&2; exit 1; }
fi

echo "[preflight] session=${SESSION_ID} mode=${CONTROLLER_MODE} output=${SESSION_DIR}"
echo "[preflight] master=Left (channel 300, /dev/ttyACM300, usb-0:10.1); slave=Default Device (channel 301, /dev/ttyACM301, usb-0:10.2)"
echo "[preflight] startup is ZERO-FORCE: joint_space_control=false, enabled_on_start=false, C1=C2=C3=C4=0"
echo "[preflight] DOB observation: left enabled; right explicitly enabled; no DOB output is applied"
echo "[preflight] normal/emergency stop: ${LAUNCH_DIR}/99_stop_setup.sh --session ${SESSION_ID} --output-dir ${OUTPUT_DIR}"
df -h "${OUTPUT_DIR%/*}" 2>/dev/null | tail -n 1 || true
date --iso-8601=seconds

if [[ "${DRY_RUN}" == true ]]; then
  echo "[preflight] DRY RUN PASS (device presence/permissions intentionally not required)"
  exit 0
fi
mkdir -p "${PID_DIR}" "${LOG_DIR}"
{
  printf 'SESSION_ID=%q\n' "${SESSION_ID}"
  printf 'SESSION_DIR=%q\n' "${SESSION_DIR}"
  printf 'CREATED_AT=%q\n' "$(date --iso-8601=seconds)"
  printf 'CONTROLLER_MODE=%q\n' "${CONTROLLER_MODE}"
  printf 'OPERATOR_NOTES=%q\n' "${OPERATOR_NOTES}"
  printf 'ROS_DOMAIN_ID=%q\n' "${ROS_DOMAIN_ID:-0}"
  printf 'MASTER_DEVICE=%q\n' 'Left'
  printf 'SLAVE_DEVICE=%q\n' 'Default Device'
  printf 'LEFT_INERTIA=%q\nLEFT_GRAVITY=%q\nRIGHT_INERTIA=%q\nRIGHT_GRAVITY=%q\n' \
    "${LEFT_INERTIA}" "${LEFT_GRAVITY}" "${RIGHT_INERTIA}" "${RIGHT_GRAVITY}"
  printf 'INITIAL_CHANNELS=%q\n' 'C1=0,C2=0,C3=0,C4=0'
  printf 'DOB_G_DIS_RAD_S=%q\nDOB_THRESHOLDS_NM=%q\n' '10,10,5' '0.07632,0.06400,0.05000'
  printf 'POSITION_GAINS=%q\n' 'master=15/0.10,slave=35/0.20 (N/m,Ns/m)'
  printf 'LIMITS=%q\n' 'master=2.0N,slave=1.5N,position=0.03/0.04m,relay_floor=0.02Nm'
  printf 'RATES=%q\n' 'servo~1000Hz,publish=200Hz,bridge=100Hz,relay=200Hz'
} >"${MANIFEST}"
printf '%s\n' "${SESSION_DIR}" >"${current_file}"
echo "[preflight] PASS; manifest=${MANIFEST}"
