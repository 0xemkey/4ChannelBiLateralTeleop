#!/usr/bin/env bash
set -euo pipefail

SESSION="$(date -u +%Y%m%dT%H%M%SZ)_gdis_2p0"

echo "Starting SESSION=$SESSION"

DOB_G_DIS_SCALE=2.0 bash 00_preflight.sh --session "$SESSION"

DOB_G_DIS_SCALE=2.0 bash 01_start_master.sh --session "$SESSION"
sleep 2

DOB_G_DIS_SCALE=2.0 bash 02_start_slave.sh --session "$SESSION"
sleep 2

DOB_G_DIS_SCALE=2.0 bash 03_start_position_bridge.sh \
  --session "$SESSION" \
  --arm-position

echo
echo "SESSION=$SESSION"
echo "Stop with:"
echo "bash 99_stop_setup.sh --session $SESSION"
