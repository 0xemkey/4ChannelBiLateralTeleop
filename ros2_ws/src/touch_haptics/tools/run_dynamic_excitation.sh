#!/usr/bin/env bash
# Step 2: bounded dynamic excitation for M_n identification.
# Run this in YOUR terminal. You drive it; Ctrl-C stops it.
#
#   ./run_dynamic_excitation.sh              -> device "Left"
#   ./run_dynamic_excitation.sh "Default Device"
#
# Watch live angles from a SECOND terminal:
#   ./watch_sweep_status.sh          (same status file, works for both tools)
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE="${1:-Left}"
LOG_DIR="${LOG_DIR:-$HOME/dob_logs}"
LOG="${LOG:-$LOG_DIR/dyn_${DEVICE// /_}.csv}"
STATUS="${STATUS:-/tmp/touch_sweep_status.txt}"

AMP="${AMP:-0.25}"          # chirp amplitude, N (reduced automatically if the
                            # support bias leaves less headroom than this)
DUR="${DUR:-8}"             # seconds per capture
F0="${F0:-0.5}"             # chirp start, Hz
F1="${F1:-6.0}"             # chirp end, Hz
AXES="${AXES:-xyz}"

mkdir -p "$LOG_DIR"

busy=0
for p in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
  if grep -qs libHD "/proc/$p/maps" 2>/dev/null; then
    echo "[dyn][error] PID $p already holds a Touch device:" >&2
    tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null; echo >&2
    busy=1
  fi
done
[ "$busy" = 1 ] && { echo "[dyn][hint] stop it first, then re-run." >&2; exit 1; }

if [ ! -x "$SCRIPT_DIR/hdapi_dynamic_excitation" ]; then
  echo "[dyn][error] not built. From $SCRIPT_DIR run:" >&2
  echo "  g++ -std=gnu++17 -I../include hdapi_dynamic_excitation.cpp -o hdapi_dynamic_excitation -lHD -lHDU -lrt -lpthread" >&2
  exit 1
fi

echo "============================================================"
echo " Dynamic excitation - M_n identification (Step 2)"
echo "   device : $DEVICE"
echo "   log    : $LOG        (appends; safe to stop and resume)"
echo "   chirp  : ${F0} -> ${F1} Hz over ${DUR}s, amplitude ${AMP} N"
echo "   axes   : $AXES"
echo "   hold   : AUTOMATIC per pose (same routine as the sweep)"
echo
echo " In a SECOND terminal:  $SCRIPT_DIR/watch_sweep_status.sh"
echo
echo " PER POSE:"
echo "   1. HOLD the arm where you want it, clear of the table."
echo "   2. Enter. It captures the target, you RELEASE when told,"
echo "      it finds the holding force and freezes it."
echo "   3. Enter once per axis - HANDS OFF - to run each chirp."
echo
echo " The arm WILL oscillate visibly during a capture. That is the"
echo " point: M_n cannot be identified without real acceleration."
echo
echo " The amplitude is reduced automatically if the support bias"
echo " leaves too little headroom. That is preferable to clipping:"
echo " a clipped chirp is distorted, and the regression would read"
echo " the resulting harmonics as inertia."
echo
echo " NO THERMAL FEEDBACK EXISTS on this device. Unlike the sweep's"
echo " brief ramps this oscillates for seconds at a time. Keep the"
echo " session short and let the motors rest between runs."
echo
echo " Ctrl-C is safe at any moment. It zeroes the chirp AND the"
echo " hold, so the arm goes limp and WILL sag. Keep hands clear."
echo "============================================================"
echo

cd "$SCRIPT_DIR" || exit 1
./hdapi_dynamic_excitation "$DEVICE" --log "$LOG" --status "$STATUS" \
    --amp "$AMP" --dur "$DUR" --f0 "$F0" --f1 "$F1" --axes "$AXES"
rc=$?

echo
echo "[dyn] exited (rc=$rc). Log: $LOG"
if [ -f "$LOG" ]; then
  echo "[dyn] samples: $(( $(wc -l < "$LOG") - 1 ))"
  echo "[dyn] poses  : $(tail -n +2 "$LOG" | cut -d, -f3 | sort -un | tr '\n' ' ')"
  clip=$(tail -n +2 "$LOG" | awk -F, '{if($NF+0>0) c++} END{print c+0}')
  [ "$clip" != "0" ] && echo "[dyn] WARNING: $clip sample rows came from clipped captures - treat as suspect."
fi
exit $rc
