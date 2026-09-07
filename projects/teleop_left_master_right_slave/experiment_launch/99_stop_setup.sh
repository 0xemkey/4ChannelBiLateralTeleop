#!/usr/bin/env bash
set -euo pipefail
EXTRA_ARGS=(); source "$(cd "$(dirname "$0")" && pwd)/common.sh"; parse_common_args "$@"
require_session
echo "[stop] targeted session=${SESSION_DIR}"
[[ "${DRY_RUN}" == true ]] && { echo '[stop] would TERM only PID groups recorded by this session'; exit 0; }
for name in recorder bridge force_relay slave master; do
  f="${PID_DIR}/${name}.pgid"; [[ -f "$f" ]] || continue
  pgid="$(<"$f")"
  [[ "$pgid" =~ ^[0-9]+$ ]] || { echo "[stop][warning] invalid PGID file: $f" >&2; continue; }
  if kill -0 "$pgid" 2>/dev/null; then
    echo "[stop] TERM ${name} process group ${pgid}"
    kill -TERM -- "-${pgid}" 2>/dev/null || true
  fi
done
deadline=$((SECONDS + 5))
while ((SECONDS < deadline)); do
  alive=false
  for f in "${PID_DIR}"/*.pid; do [[ -e "$f" ]] || continue; kill -0 "$(<"$f")" 2>/dev/null && alive=true; done
  [[ "$alive" == false ]] && break
  sleep 0.2
done
for f in "${PID_DIR}"/*.pid; do
  [[ -e "$f" ]] || continue; pid="$(<"$f")"
  if kill -0 "$pid" 2>/dev/null; then echo "[stop][warning] pid ${pid} remains; inspect before any manual KILL" >&2; fi
done
current_file="${LAUNCH_DIR}/.current_session"
[[ -f "$current_file" && "$(<"$current_file")" == "${SESSION_DIR}" ]] && rm -f "$current_file"
echo '[stop] shutdown request complete; hardware watchdog also zeros stale commands after 0.05 s.'
