#!/usr/bin/env bash
# Opens every PNG plot from the most recently generated Phase 2 capture.
# Usage:
#   ./view_latest_plots.sh                 # most recent capture's plots
#   ./view_latest_plots.sh <name-fragment>  # most recent capture matching this text
set -euo pipefail

CAPTURE_DIR="${CAPTURE_DIR:-$HOME/dob_logs/phase2_captures}"
FILTER="${1:-}"

if [[ ! -d "$CAPTURE_DIR" ]]; then
  echo "[view_latest_plots] no such directory: $CAPTURE_DIR" >&2
  exit 1
fi

# Find the most recently modified CSV (optionally filtered by name), then
# open every PNG that shares its stem.
latest_csv="$(ls -t "$CAPTURE_DIR"/*"$FILTER"*.csv 2>/dev/null | head -1 || true)"
if [[ -z "$latest_csv" ]]; then
  echo "[view_latest_plots] no capture CSV found matching '*${FILTER}*.csv' in $CAPTURE_DIR" >&2
  exit 1
fi

stem="$(basename "$latest_csv" .csv)"
echo "[view_latest_plots] most recent capture: ${stem}"

shopt -s nullglob
pngs=("$CAPTURE_DIR/${stem}"_*.png)
shopt -u nullglob

if [[ ${#pngs[@]} -eq 0 ]]; then
  echo "[view_latest_plots] no plots found for this capture yet -- run plot_phase2_scenario.py on it first:" >&2
  echo "  python3 $(dirname "$0")/plot_phase2_scenario.py '$latest_csv'" >&2
  exit 1
fi

echo "[view_latest_plots] opening ${#pngs[@]} plots..."
for f in "${pngs[@]}"; do
  xdg-open "$f" >/dev/null 2>&1 &
done
wait
