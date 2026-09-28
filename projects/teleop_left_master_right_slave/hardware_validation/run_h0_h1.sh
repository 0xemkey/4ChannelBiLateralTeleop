#!/usr/bin/env bash
# H0/H1 terminal script (2026-09-10) -- physical inspection preflight +
# passive zero-force baseline capture + plots. NO force is enabled by
# anything in this script: preflight.py is read-only, and
# passive_baseline_capture.py only SUBSCRIBES to the already-running
# /teleop_master_left and /teleop_slave_right nodes' telemetry (both
# started with enabled_on_start=false per OPERATOR_PROCEDURE.md) -- it
# never calls create_publisher(). This script does not arm the bridge,
# does not touch arm_teleop, and does not require --confirmed-supervision
# for that reason: H0/H1 apply zero force by construction, not because the
# operator's physical presence stops mattering (H0's own checklist below
# still assumes a person is physically present to do the visual checks).
#
# Usage:
#   bash hardware_validation/run_h0_h1.sh [session_suffix]
#
# Run from the repo root (~/HIL-RL) or anywhere -- this script cd's to the
# repo root itself, so the working directory you launch it from does not
# matter.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
HV_DIR="${REPO_ROOT}/projects/teleop_left_master_right_slave/hardware_validation"
HW_DIR="${REPO_ROOT}/projects/teleop_left_master_right_slave/hardware_experiments/2026-09-09_y_axis_validation_PENDING_SUPERVISION"

SESSION_SUFFIX="${1:-h0h1}"
SESSION_ID="$(date -u +%Y%m%dT%H%M%SZ)_${SESSION_SUFFIX}"
SESSION_DIR="${REPO_ROOT}/experiment_runs/teleop_setup/${SESSION_ID}"
mkdir -p "${SESSION_DIR}"

echo "================================================================"
echo " H0/H1: physical inspection preflight + passive baseline capture"
echo " session: ${SESSION_ID}"
echo " output:  ${SESSION_DIR}"
echo "================================================================"

echo "[env] sourcing ROS 2 workspace..."
set +u  # both setup.bash files reference unbound vars under `set -u` --
        # same accommodation experiment_launch/common.sh already makes.
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

echo
echo "---------------------------------------------------------------"
echo " H0a: manual physical checklist -- confirm BEFORE continuing"
echo "---------------------------------------------------------------"
cat <<'EOF'
  [ ] Workspace clear -- no loose cables/objects in either device's range
  [ ] Both stylus tips visually intact, no damage
  [ ] E-stop / stop command identified and reachable:
        python3 hardware_validation/safe_stop.py --session <ID>
  [ ] You (the operator) are physically beside both robots right now
  [ ] Nobody's hand is on either stylus yet (H1 is a PASSIVE zero-force
      baseline -- the devices must be at rest, untouched, for this to be
      a meaningful baseline)
EOF
read -r -p "Press Enter once every item above is confirmed (Ctrl-C to abort): " _

echo
echo "---------------------------------------------------------------"
echo " H0b: read-only ROS-graph preflight check"
echo "---------------------------------------------------------------"
python3 "${HV_DIR}/preflight.py"

echo
echo "---------------------------------------------------------------"
echo " H1: passive zero-force baseline capture (10s, subscribe-only)"
echo "---------------------------------------------------------------"
echo "Do not touch either device during this capture."
python3 "${HW_DIR}/passive_baseline_capture.py" "${SESSION_DIR}/passive_baseline.json"

echo
echo "---------------------------------------------------------------"
echo " Plotting"
echo "---------------------------------------------------------------"
python3 "${HW_DIR}/plot_passive_baseline.py" "${SESSION_DIR}/passive_baseline.json" "${SESSION_DIR}"

echo
echo "================================================================"
echo " Done. Data + plots: ${SESSION_DIR}"
echo " Plots:"
ls -1 "${SESSION_DIR}/plots" 2>/dev/null | sed 's/^/   /'
echo
echo " Next (only after reviewing H1's output and re-confirming physical"
echo " supervision explicitly for a FORCE-ENABLING stage): H2, per"
echo " ONSITE_RUNBOOK.md -- do NOT proceed to H2 automatically from here."
echo "================================================================"
