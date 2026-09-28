#!/usr/bin/env bash
set -euo pipefail

cd ~/HIL-RL/projects/teleop_left_master_right_slave/experiment_launch

SCALE="${1:-1.0}"

case "$SCALE" in
  1.0) TAG="1p0" ;;
  1.5) TAG="1p5" ;;
  2.0) TAG="2p0" ;;
  *)
    echo "Usage: $0 {1.0|1.5|2.0}"
    exit 1
    ;;
esac

SESSION="$(date -u +%Y%m%dT%H%M%SZ)_gdis_${TAG}"

echo "========================================"
echo "DOB g_dis hardware session"
echo "scale   = ${SCALE}"
echo "session = ${SESSION}"
echo "========================================"

DOB_G_DIS_SCALE="$SCALE" \
  bash 00_preflight.sh --session "$SESSION"

DOB_G_DIS_SCALE="$SCALE" \
  bash 01_start_master.sh --session "$SESSION"

sleep 2

DOB_G_DIS_SCALE="$SCALE" \
  bash 02_start_slave.sh --session "$SESSION"

sleep 2

DOB_G_DIS_SCALE="$SCALE" \
  bash 03_start_position_bridge.sh \
    --session "$SESSION" \
    --arm-position

echo
echo "SESSION RUNNING"
echo "SESSION=$SESSION"
echo "DOB_G_DIS_SCALE=$SCALE"
echo
echo "Stop with:"
echo "bash 99_stop_setup.sh --session $SESSION"
