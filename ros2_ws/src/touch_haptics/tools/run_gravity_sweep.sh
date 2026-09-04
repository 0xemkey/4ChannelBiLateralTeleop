#!/usr/bin/env bash
# Gravity + friction static sweep. Run this in YOUR terminal.
#
#   ./run_gravity_sweep.sh            -> device "Left"
#   ./run_gravity_sweep.sh "Default Device"
#
# You drive it directly: it prints the prompts, you press Enter, Ctrl-C stops.
# Force is zeroed inside the servo loop within one frame (~1 ms) on Ctrl-C.
#
# Watch the live angle display from a SECOND terminal:
#   ./watch_sweep_status.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEVICE="${1:-Left}"
LOG_DIR="${LOG_DIR:-$HOME/dob_logs}"
LOG="${LOG:-$LOG_DIR/sweep_${DEVICE// /_}.csv}"
STATUS="${STATUS:-/tmp/touch_sweep_status.txt}"
# Constant known support force, newtons, so poses that would sag can be held
# without a hand. Adjustable per pose by typing a number at the pose prompt.
BIAS_N="${BIAS_N:-0.20}"
BIAS_AXIS="${BIAS_AXIS:-1}"      # 1 = y = up on a standard Touch

mkdir -p "$LOG_DIR"

# One OpenHaptics process may own a device at a time. Fail loudly rather than
# producing a confusing hdInitDevice error.
busy=0
for p in $(ls /proc 2>/dev/null | grep -E '^[0-9]+$'); do
  if grep -qs libHD "/proc/$p/maps" 2>/dev/null; then
    echo "[sweep][error] PID $p already holds a Touch device:" >&2
    tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null; echo >&2
    busy=1
  fi
done
if [ "$busy" = 1 ]; then
  echo "[sweep][hint] stop that process first, then re-run." >&2
  exit 1
fi

if [ ! -x "$SCRIPT_DIR/hdapi_static_sweep" ]; then
  echo "[sweep][error] hdapi_static_sweep not built. From $SCRIPT_DIR run:" >&2
  echo "  g++ -std=gnu++17 -I../include hdapi_static_sweep.cpp -o hdapi_static_sweep -lHD -lHDU -lrt -lpthread" >&2
  exit 1
fi

echo "============================================================"
echo " Gravity / friction static sweep"
echo "   device : $DEVICE"
echo "   log    : $LOG        (appends; safe to stop and resume)"
echo "   status : $STATUS"
echo
echo " In a SECOND terminal, run:   $SCRIPT_DIR/watch_sweep_status.sh"
echo
echo "   hold   : AUTOMATIC (finds the support force per pose)"
echo
echo " PER POSE:"
echo "   1. HOLD the arm where you want it, clear of the table."
echo "   2. Press Enter.  It captures that position as the target."
echo "   3. RELEASE when told.  It finds the force that holds the"
echo "      arm there, averages it, and freezes it as a constant."
echo "   4. It verifies the frozen force really holds, then you"
echo "      press Enter six times - HANDS OFF - for the ramps."
echo
echo " The arm settles a few mm BELOW where you held it. That is"
echo " normal: a PD needs standing error to make force. The pose"
echo " actually logged is where it settles, so the fit is correct."
echo
echo " NEVER HOLD THE ARM DURING A RAMP. Your hand's force is"
echo " unknown and lands straight in the gravity estimate. The"
echo " commanded hold is known exactly and is subtracted offline."
echo
echo " If a pose cannot be held within the force cap it is REJECTED"
echo " and re-prompted. Reposition or skip it - do not reach in."
echo
echo " Ctrl-C is safe at any moment. It zeroes the ramp AND the"
echo " hold, so the arm goes fully limp and WILL sag. Keep your"
echo " hand clear of where it would fall."
echo "============================================================"
echo

cd "$SCRIPT_DIR" || exit 1
# Foreground, so stdin/Ctrl-C belong to the operator's terminal.
# --auto-hold is the default; MANUAL_BIAS=1 falls back to the older
# type-a-number-per-pose behaviour if the auto routine misbehaves.
if [ "${MANUAL_BIAS:-0}" = "1" ]; then
  ./hdapi_static_sweep "$DEVICE" --log "$LOG" --status "$STATUS" \
      --no-auto-hold --bias-n "$BIAS_N" --bias-axis "$BIAS_AXIS"
else
  ./hdapi_static_sweep "$DEVICE" --log "$LOG" --status "$STATUS" --auto-hold
fi
rc=$?

echo
echo "[sweep] exited (rc=$rc). Rows now in: $LOG"
if [ -f "$LOG" ]; then
  echo "[sweep] ramps logged: $(( $(wc -l < "$LOG") - 1 ))"
  echo "[sweep] poses seen  : $(tail -n +2 "$LOG" | cut -d, -f3 | sort -un | tr '\n' ' ')"
fi
exit $rc
