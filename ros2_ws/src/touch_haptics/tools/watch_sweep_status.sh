#!/usr/bin/env bash
# Live angle / position display for the static sweep.
# Run in a SECOND terminal while run_gravity_sweep.sh is going.
# Ctrl-C here only closes the display; it does NOT touch the sweep.
set -uo pipefail
STATUS="${STATUS:-/tmp/touch_sweep_status.txt}"

echo "watching $STATUS  (Ctrl-C closes this display only)"
echo "waiting for the sweep to start..."
while [ ! -f "$STATUS" ]; do sleep 0.3; done

# Redraw in place rather than scrolling, so the numbers stay readable.
printf '\033[?25l'                      # hide cursor
trap 'printf "\033[?25h\n"' EXIT INT TERM   # restore on the way out
while true; do
  printf '\033[H\033[J'                 # home + clear
  echo "=== Touch live status ==="
  cat "$STATUS" 2>/dev/null
  echo
  echo "  STEADY  -> safe to press Enter for the next ramp"
  echo "  MOVING  -> wait; the 0.010 rad self-hold check would fail"
  sleep 0.2
done
