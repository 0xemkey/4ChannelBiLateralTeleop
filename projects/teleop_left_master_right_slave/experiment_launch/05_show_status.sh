#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
require_session
echo "[status] session=${SESSION_DIR} mode=${CONTROLLER_MODE}"
[[ "${DRY_RUN}" == true ]] && { echo '[status] would show manifest, owned PIDs, ROS nodes/topics and rates'; exit 0; }
source_ros
sed -n '1,200p' "${MANIFEST}"
for f in "${PID_DIR}"/*.pid; do
  [[ -e "$f" ]] || continue
  pid="$(<"$f")"; name="$(basename "$f" .pid)"
  if kill -0 "$pid" 2>/dev/null; then echo "[status] ${name}: RUNNING pid=${pid}"; else echo "[status] ${name}: STOPPED pid=${pid}"; fi
done
timeout 5 ros2 node list || true
for topic in /touch_left/pose /touch_right/pose /touch_left/dob_diagnostics /touch_right/dob_diagnostics /touch_left/applied_force /touch_right/applied_force; do
  timeout 4 ros2 topic hz --window 20 "$topic" 2>/dev/null | tail -n 2 || echo "[status] no rate for ${topic}"
done
echo '[status] applied_force is commanded Cartesian force, not measured contact force.'

