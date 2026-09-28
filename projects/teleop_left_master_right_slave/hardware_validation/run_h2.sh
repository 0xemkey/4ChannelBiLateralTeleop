#!/usr/bin/env bash
# H2 terminal script (2026-09-10) -- gravity-settle-only capture on real
# hardware. C1=C2=C3=C4=OFF, require_master_deadman=true,
# gravity_settle_hold_mode=true. This is the FIRST force-enabling stage in
# ONSITE_RUNBOOK.md and requires physical supervision, confirmed per this
# session's conversation.
#
# Usage:
#   bash hardware_validation/run_h2.sh [session_suffix] [observation_window_s] [--driver-session ID]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HV_DIR="${REPO_ROOT}/projects/teleop_left_master_right_slave/hardware_validation"
HW_DIR="${REPO_ROOT}/projects/teleop_left_master_right_slave/hardware_experiments/2026-09-09_y_axis_validation_PENDING_SUPERVISION"

SESSION_SUFFIX="${1:-h2}"
OBS_WINDOW_S="${2:-10.0}"
EXPLICIT_DRIVER_SESSION="${3:-}"
SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)_${SESSION_SUFFIX}"
SESSION_DIR="${REPO_ROOT}/experiment_runs/teleop_setup/${SESSION_ID}"
mkdir -p "${SESSION_DIR}"

echo "================================================================"
echo " H2: gravity-settle-only capture (C1=C2=C3=C4=OFF, deadman required)"
echo " session:  ${SESSION_ID}"
echo " output:   ${SESSION_DIR}"
echo "================================================================"

echo "[env] sourcing ROS 2 workspace..."
set +u
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

echo
echo "---------------------------------------------------------------"
echo " Single-instance lock (2026-09-10 safety-hardening, Task 3)"
echo "---------------------------------------------------------------"
# Two H2 orchestration runs started only ~3s apart on real hardware caused
# a previous run's bridge process (still mid-shutdown) to race a newly-
# launched one -- both briefly alive, both named touch_master_slave_bridge,
# producing an observed READY<->GRAVITY_RAMP_DOWN diagnostics oscillation
# before the new run's own arm attempt. This lock makes a second
# concurrent run_h2.sh invocation fail immediately (not queue, not wait)
# rather than silently interleaving with an in-progress one. Defense in
# depth: part1_gravity_settle_capture.py ALSO independently refuses to
# launch a new bridge if one is already visible in the ROS graph, and
# verifies the bridge node/publishers have actually disappeared (not just
# that the OS process exited) before this script's own control returns --
# see its module docstring.
H2_LOCKFILE="/tmp/h2_orchestration.lock"
exec 200>"${H2_LOCKFILE}"
if ! flock -n 200; then
  echo "[FAIL CLOSED] another H2 orchestration run appears to be in progress"
  echo "(lock held: ${H2_LOCKFILE}) -- refusing to start a second one"
  echo "concurrently. Wait for it to finish, or if you are certain no other"
  echo "run_h2.sh is actually running, investigate before removing the lock"
  echo "file by hand."
  exit 1
fi
echo "[ok] single-instance lock acquired (${H2_LOCKFILE})"

echo
echo "---------------------------------------------------------------"
echo " Step 0: resolve the driver-owning session at RUNTIME (fail closed)"
echo "---------------------------------------------------------------"
# 2026-09-10 safety-hardening change: NEVER hardcode/reuse a session ID
# for an emergency stop. resolve_driver_session.py derives the live pgids
# of the actually-running teleop_master_left/teleop_slave_right processes
# from the OS process table and finds the ONE session directory whose
# recorded pgids match exactly -- or fails closed (nonzero exit) on any
# ambiguity. This script refuses to proceed to arming if that fails: the
# "E-stop identified and reachable" checklist item in Step 3 below would
# otherwise be showing an unverified command.
RESOLVE_ARGS=()
if [[ -n "${EXPLICIT_DRIVER_SESSION}" ]]; then
  RESOLVE_ARGS=(--driver-session "${EXPLICIT_DRIVER_SESSION}")
fi
set +e
RESOLVE_OUTPUT="$(python3 "${HV_DIR}/resolve_driver_session.py" "${RESOLVE_ARGS[@]}")"
RESOLVE_EXIT=$?
set -e
echo "${RESOLVE_OUTPUT}"
if [[ "${RESOLVE_EXIT}" -ne 0 ]]; then
  echo
  echo "[FAIL CLOSED] could not verify the currently-running master/slave driver"
  echo "session from live process state -- refusing to proceed. See the reason"
  echo "above. Identify the correct session manually, then re-run this script"
  echo "with it as a third argument:"
  echo "  bash hardware_validation/run_h2.sh ${SESSION_SUFFIX} ${OBS_WINDOW_S} <verified_driver_session_id>"
  exit 1
fi
DRIVER_SESSION_ID="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['session_id'])" "${RESOLVE_OUTPUT}")"
echo "[ok] driver session verified at runtime: ${DRIVER_SESSION_ID}"

echo
echo "---------------------------------------------------------------"
echo " Step 1: print resolved H2 configuration (no actuation yet)"
echo "---------------------------------------------------------------"
# This preview call deliberately omits --confirmed-supervision, so the
# script prints its resolved config/fingerprint/bridge command and then
# exits(3) BY DESIGN without arming anything -- `set +e` around it so that
# expected, documented non-zero exit does not abort this wrapper script.
set +e
python3 "${HW_DIR}/part1_gravity_settle_capture.py" \
  --out-dir "${SESSION_DIR}" --session-id "${SESSION_ID}" \
  --observation-window-s "${OBS_WINDOW_S}"
set -e
echo "[info] the run above printed the resolved config + fingerprint + bridge"
echo "       command and exited WITHOUT arming (no --confirmed-supervision"
echo "       was passed) -- review it now."

echo
echo "---------------------------------------------------------------"
echo " Step 2: read-only preflight -- confirm sole enable authority,"
echo "         C1-C4 gates off, no unexpected publishers"
echo "---------------------------------------------------------------"
python3 "${HV_DIR}/preflight.py"

echo
echo "---------------------------------------------------------------"
echo " Step 3: physical-supervision re-confirmation for THIS force-"
echo "         enabling stage"
echo "---------------------------------------------------------------"
cat <<EOF
  [ ] You are physically beside both robots right now
  [ ] Workspace still clear, nothing changed since H0/H1
  [ ] You are ready to hold the master's deadman button continuously for
      the whole capture (--require-master-deadman=true -- gravity support
      force will NOT apply without it held)
  [ ] E-stop identified and reachable:
        python3 hardware_validation/safe_stop.py --session ${DRIVER_SESSION_ID}
      (this targets the CURRENTLY-RUNNING master/slave driver session,
      verified against live PIDs immediately before this script was
      written -- it will terminate BOTH driver processes if run)
  [ ] You understand this stage applies REAL gravity-support force to the
      slave arm (~0.55N) once the quiet-hold gate passes
EOF
read -r -p "Press Enter once every item above is confirmed (Ctrl-C to abort): " _

echo
echo "---------------------------------------------------------------"
echo " Step 4: ARMED run -- hold the master deadman button now"
echo "---------------------------------------------------------------"
set +e
python3 "${HW_DIR}/part1_gravity_settle_capture.py" \
  --out-dir "${SESSION_DIR}" --session-id "${SESSION_ID}" \
  --observation-window-s "${OBS_WINDOW_S}" \
  --confirmed-supervision
CAPTURE_EXIT=$?
set -e

echo
echo "---------------------------------------------------------------"
echo " Step 5: analysis"
echo "---------------------------------------------------------------"
set +e
python3 "${HW_DIR}/analyze_h2_gravity_settle.py" "${SESSION_DIR}"
ANALYZE_EXIT=$?
set -e

echo
echo "---------------------------------------------------------------"
echo " Step 6: LIVE post-disarm safety verification (fresh subscription,"
echo "         NOT just the last recorded CSV row -- 2026-09-10"
echo "         safety-hardening change)"
echo "---------------------------------------------------------------"
set +e
VERIFY_OUTPUT="$(python3 "${HV_DIR}/verify_disarmed.py")"
VERIFY_EXIT=$?
set -e
echo "${VERIFY_OUTPUT}"
VERIFY_STATUS="$(python3 -c "import json,sys; print(json.loads(sys.argv[1])['status'])" "${VERIFY_OUTPUT}" 2>/dev/null || echo "UNKNOWN")"

echo
echo "================================================================"
if [[ "${ANALYZE_EXIT}" -eq 0 ]]; then
  H2_GATE_RESULT="PASS"
else
  H2_GATE_RESULT="FAIL"
fi
echo " H2 gate result:        ${H2_GATE_RESULT}"
echo " Post-disarm safety:    ${VERIFY_STATUS}"

if [[ "${H2_GATE_RESULT}" == "PASS" && "${VERIFY_STATUS}" == "CONFIRMED_DISABLED" ]]; then
  OVERALL="PASS"
elif [[ "${VERIFY_STATUS}" == "CONFIRMED_NOT_DISABLED" ]]; then
  OVERALL="FAIL -- DEVICES CONFIRMED NOT DISABLED, INVESTIGATE IMMEDIATELY"
elif [[ "${VERIFY_STATUS}" == "SAFETY_INCOMPLETE" || "${VERIFY_STATUS}" == "UNKNOWN" ]]; then
  OVERALL="SAFETY-INCOMPLETE -- disarmed state could not be confirmed live"
else
  OVERALL="FAIL"
fi
echo " OVERALL:               ${OVERALL}"
if [[ "${OVERALL}" != "PASS" ]]; then
  echo " DO NOT PROCEED TO H3."
fi
echo
echo " Data:       ${SESSION_DIR}"
echo " Metrics:    ${SESSION_DIR}/h2_metrics.json"
echo " Validation: ${SESSION_DIR}/h2_validation.json"
echo " Plots:"
ls -1 "${SESSION_DIR}/plots" 2>/dev/null | sed 's/^/   /'
echo
echo " Driver-level E-stop (NOT invoked -- would terminate the master/slave"
echo " driver processes themselves, ending the whole hardware session, not"
echo " just this stage). Runtime-verified target, run it yourself if you"
echo " want to end the session now:"
echo "   python3 hardware_validation/safe_stop.py --session ${DRIVER_SESSION_ID}"
echo "================================================================"

if [[ "${OVERALL}" == "PASS" ]]; then
  exit 0
else
  exit 1
fi
